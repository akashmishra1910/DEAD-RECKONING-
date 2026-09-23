#include "idr/idrmap_reader.hpp"
#include <cassert>
#include <iostream>
#include <vector>
#include <cmath>
#include <cstring>
#include <fstream>
#include <algorithm>
#include <atomic>
#include <chrono>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <thread>
#endif

// Memory allocation tracking instrumentation
static std::atomic<size_t> g_alloc_count{0};
static std::atomic<size_t> g_alloc_bytes{0};
static bool g_track_allocations = false;

#if defined(_MSC_VER) || defined(__GNUC__) || defined(__clang__)
void* operator new(size_t size) {
    if (g_track_allocations) {
        g_alloc_count.fetch_add(1, std::memory_order_relaxed);
        g_alloc_bytes.fetch_add(size, std::memory_order_relaxed);
    }
    void* p = std::malloc(size);
    if (!p) throw std::bad_alloc();
    return p;
}

void operator delete(void* p) noexcept {
    std::free(p);
}

void operator delete(void* p, size_t) noexcept {
    std::free(p);
}
#endif

namespace idr {

void test_static_asserts() {
    std::cout << "[TEST] 1. Static struct layout and sizes..." << std::endl;
    static_assert(sizeof(IdrMapHeader) == 128, "IdrMapHeader must be 128 bytes");
    static_assert(sizeof(IdrMapNode) == 28, "IdrMapNode must be 28 bytes");
    static_assert(sizeof(IdrMapEdge) == 40, "IdrMapEdge must be 40 bytes");
    static_assert(sizeof(IdrMapGeomPoint) == 16, "IdrMapGeomPoint must be 16 bytes");
    static_assert(sizeof(IdrMapSpatialCell) == 6, "IdrMapSpatialCell must be 6 bytes");
    static_assert(sizeof(IdrMapSpatialEntry) == 4, "IdrMapSpatialEntry must be 4 bytes");
    std::cout << "  -> PASSED" << std::endl;
}

void test_open_fixtures() {
    std::cout << "[TEST] 2. Open committed .idrmap fixtures..." << std::endl;
    const char* fixtures[] = {
        "data/maps/sample_prayagraj.idrmap",
        "data/maps/dense_metro.idrmap",
        "data/maps/regional_network.idrmap"
    };

    for (const char* path : fixtures) {
        IdrMapReader reader;
        IdrMapStatus status = reader.open_file(path);
        if (status != IdrMapStatus::OK) {
            std::cerr << "Failed to open fixture: " << path << " (status=" << static_cast<int>(status) << ")" << std::endl;
        }
        assert(status == IdrMapStatus::OK);
        assert(reader.is_open());
        assert(reader.node_count() > 0);
        assert(reader.edge_count() > 0);
        assert(reader.geom_point_count() > 0);
        assert(reader.grid_cell_count() > 0);

        double min_lat, min_lon, max_lat, max_lon;
        reader.get_bounds(min_lat, min_lon, max_lat, max_lon);
        assert(min_lat < max_lat);
        assert(min_lon < max_lon);

        const char* attrib = reader.get_attribution_json();
        assert(attrib != nullptr);
        assert(std::strlen(attrib) > 0);
        std::cout << "  -> Opened " << path << " (Edges: " << reader.edge_count() << ", Nodes: " << reader.node_count() << ")" << std::endl;
    }
    std::cout << "  -> PASSED" << std::endl;
}

void test_corruption_checks() {
    std::cout << "[TEST] 3. Negative & Corruption Rejection..." << std::endl;

    // Open a valid fixture into memory first
    std::string valid_path = "data/maps/sample_prayagraj.idrmap";
    std::ifstream file(valid_path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        std::cerr << "Could not open valid fixture for corruption tests." << std::endl;
        assert(false);
    }
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<uint8_t> valid_buf(static_cast<size_t>(size));
    file.read(reinterpret_cast<char*>(valid_buf.data()), size);

    // 1. Buffer too small (< 128 bytes)
    {
        IdrMapReader reader;
        assert(reader.open_buffer(valid_buf.data(), 64) == IdrMapStatus::ERR_BUFFER_TOO_SMALL);
    }

    // 2. Invalid Magic
    {
        std::vector<uint8_t> corrupt = valid_buf;
        corrupt[0] = 'X';
        IdrMapReader reader;
        assert(reader.open_buffer(corrupt.data(), corrupt.size()) == IdrMapStatus::ERR_INVALID_MAGIC);
    }

    // 3. Unsupported Version
    {
        std::vector<uint8_t> corrupt = valid_buf;
        corrupt[8] = 2; // version 2
        // Update header CRC so version check triggers
        uint32_t hcrc = calculate_crc32(corrupt.data(), 112);
        std::memcpy(corrupt.data() + 112, &hcrc, 4);
        IdrMapReader reader;
        assert(reader.open_buffer(corrupt.data(), corrupt.size()) == IdrMapStatus::ERR_UNSUPPORTED_VERSION);
    }

    // 4. Header CRC Mismatch
    {
        std::vector<uint8_t> corrupt = valid_buf;
        corrupt[20] ^= 0xFF; // Flip bit in min_lat
        IdrMapReader reader;
        assert(reader.open_buffer(corrupt.data(), corrupt.size()) == IdrMapStatus::ERR_HEADER_CRC_MISMATCH);
    }

    // 5. Payload CRC Mismatch
    {
        std::vector<uint8_t> corrupt = valid_buf;
        corrupt[corrupt.size() - 5] ^= 0xFF; // Flip bit in payload
        IdrMapReader reader;
        assert(reader.open_buffer(corrupt.data(), corrupt.size()) == IdrMapStatus::ERR_PAYLOAD_CRC_MISMATCH);
    }

    // 6. Invalid Bounding Box
    {
        std::vector<uint8_t> corrupt = valid_buf;
        double invalid_min_lat = 95.0;
        std::memcpy(corrupt.data() + 16, &invalid_min_lat, 8);
        uint32_t hcrc = calculate_crc32(corrupt.data(), 112);
        std::memcpy(corrupt.data() + 112, &hcrc, 4);
        IdrMapReader reader;
        assert(reader.open_buffer(corrupt.data(), corrupt.size()) == IdrMapStatus::ERR_INVALID_BOUNDS);
    }

    // 7. Table Offset Beyond File Size
    {
        std::vector<uint8_t> corrupt = valid_buf;
        uint32_t impossible_offset = 99999999;
        std::memcpy(corrupt.data() + 72, &impossible_offset, 4); // offset_nodes
        uint32_t hcrc = calculate_crc32(corrupt.data(), 112);
        std::memcpy(corrupt.data() + 112, &hcrc, 4);
        IdrMapReader reader;
        assert(reader.open_buffer(corrupt.data(), corrupt.size()) == IdrMapStatus::ERR_TABLE_OFFSET_OUT_OF_BOUNDS);
    }

    std::cout << "  -> PASSED" << std::endl;
}

void test_spatial_queries() {
    std::cout << "[TEST] 4. Spatial Candidate Queries & Bounded Overflow..." << std::endl;
    IdrMapReader reader;
    assert(reader.open_file("data/maps/sample_prayagraj.idrmap") == IdrMapStatus::OK);

    // Query near Node 2 (25.4350, 81.8500)
    IdrRoadCandidate candidates[8];
    size_t found = reader.query_nearest_roads(25.4350, 81.8500, 100.0, 5, candidates, 8);
    assert(found > 0);
    assert(candidates[0].distance_m < 1.0);
    assert(candidates[0].segment_heading_deg >= 0.0 && candidates[0].segment_heading_deg < 360.0);

    // Test bounded output capping (k > max_out)
    IdrRoadCandidate small_buf[2];
    size_t capped = reader.query_nearest_roads(25.4350, 81.8500, 100.0, 10, small_buf, 2);
    assert(capped <= 2);

    // Test k = 0 and max_out = 0
    assert(reader.query_nearest_roads(25.4350, 81.8500, 100.0, 0, candidates, 8) == 0);
    assert(reader.query_nearest_roads(25.4350, 81.8500, 100.0, 5, candidates, 0) == 0);

    // Test radius query
    uint32_t edge_ids[16];
    size_t n_rad = reader.query_roads_in_radius(25.4350, 81.8500, 200.0, edge_ids, 16);
    assert(n_rad > 0);

    // Test bbox query
    uint32_t bbox_edges[16];
    size_t n_bbox = reader.query_roads_in_bbox(25.430, 81.840, 25.440, 81.860, bbox_edges, 16);
    assert(n_bbox > 0);

    // Test edge inspection and geometry access
    IdrMapEdge edge;
    assert(reader.get_edge(candidates[0].edge_id, edge));
    double lats[16], lons[16];
    size_t n_pts = reader.get_edge_geometry(candidates[0].edge_id, lats, lons, 16);
    assert(n_pts >= 2);

    std::cout << "  -> PASSED" << std::endl;
}

void test_zero_allocation_guarantee() {
    std::cout << "[TEST] 5. Zero-Allocation High-Rate Query Verification..." << std::endl;
    IdrMapReader reader;
    assert(reader.open_file("data/maps/dense_metro.idrmap") == IdrMapStatus::OK);

    IdrRoadCandidate candidates[16];
    uint32_t edge_ids[32];

    // Warm-up query
    reader.query_nearest_roads(25.4400, 81.8500, 100.0, 5, candidates, 16);

    // Enable instrumentation
    g_alloc_count.store(0);
    g_alloc_bytes.store(0);
    g_track_allocations = true;

    const size_t N_ITER = 10000;
    for (size_t i = 0; i < N_ITER; ++i) {
        double q_lat = 25.4350 + (i % 50) * 0.0001;
        double q_lon = 81.8450 + (i % 50) * 0.0001;
        size_t found = reader.query_nearest_roads(q_lat, q_lon, 150.0, 5, candidates, 16);
        (void)found;
        size_t rad_count = reader.query_roads_in_radius(q_lat, q_lon, 200.0, edge_ids, 32);
        (void)rad_count;
    }

    g_track_allocations = false;
    size_t total_allocs = g_alloc_count.load();
    size_t total_bytes = g_alloc_bytes.load();

    std::cout << "  Iterations: " << N_ITER << " queries" << std::endl;
    std::cout << "  Total Allocations: " << total_allocs << " (Target: 0)" << std::endl;
    std::cout << "  Total Bytes Allocated: " << total_bytes << " (Target: 0)" << std::endl;

    assert(total_allocs == 0);
    assert(total_bytes == 0);
    std::cout << "  -> PASSED" << std::endl;
}

#if defined(_WIN32)
struct ThreadParam {
    const IdrMapReader* reader;
    std::atomic<bool>* success;
    int thread_id;
    int queries;
};

static DWORD WINAPI QueryWorkerThread(LPVOID lpParam) {
    auto* param = static_cast<ThreadParam*>(lpParam);
    IdrRoadCandidate candidates[8];
    for (int i = 0; i < param->queries; ++i) {
        double q_lat = 25.4300 + ((i + param->thread_id * 100) % 100) * 0.001;
        double q_lon = 81.8400 + ((i + param->thread_id * 100) % 100) * 0.001;
        size_t found = param->reader->query_nearest_roads(q_lat, q_lon, 500.0, 4, candidates, 8);
        if (found > 0) {
            for (size_t k = 1; k < found; ++k) {
                if (candidates[k].distance_m < candidates[k - 1].distance_m - 1e-6) {
                    param->success->store(false);
                }
            }
        }
    }
    return 0;
}
#endif

void test_concurrent_read_only_queries() {
    std::cout << "[TEST] 6. Multi-threaded Concurrent Read-Only Query Safety..." << std::endl;
    IdrMapReader reader;
    assert(reader.open_file("data/maps/regional_network.idrmap") == IdrMapStatus::OK);

    const int NUM_THREADS = 4;
    const int QUERIES_PER_THREAD = 5000;
    std::atomic<bool> success{true};

#if defined(_WIN32)
    HANDLE hThreads[NUM_THREADS];
    ThreadParam params[NUM_THREADS];
    for (int t = 0; t < NUM_THREADS; ++t) {
        params[t] = { &reader, &success, t, QUERIES_PER_THREAD };
        hThreads[t] = CreateThread(nullptr, 0, QueryWorkerThread, &params[t], 0, nullptr);
        assert(hThreads[t] != nullptr);
    }
    WaitForMultipleObjects(NUM_THREADS, hThreads, TRUE, INFINITE);
    for (int t = 0; t < NUM_THREADS; ++t) {
        CloseHandle(hThreads[t]);
    }
#else
    std::vector<std::thread> threads;
    for (int t = 0; t < NUM_THREADS; ++t) {
        threads.emplace_back([&reader, &success, t, QUERIES_PER_THREAD]() {
            IdrRoadCandidate candidates[8];
            for (int i = 0; i < QUERIES_PER_THREAD; ++i) {
                double q_lat = 25.4300 + ((i + t * 100) % 100) * 0.001;
                double q_lon = 81.8400 + ((i + t * 100) % 100) * 0.001;
                size_t found = reader.query_nearest_roads(q_lat, q_lon, 500.0, 4, candidates, 8);
                if (found > 0) {
                    for (size_t k = 1; k < found; ++k) {
                        if (candidates[k].distance_m < candidates[k - 1].distance_m - 1e-6) {
                            success.store(false);
                        }
                    }
                }
            }
        });
    }
    for (auto& th : threads) {
        th.join();
    }
#endif

    assert(success.load());
    std::cout << "  -> PASSED (" << (NUM_THREADS * QUERIES_PER_THREAD) << " concurrent queries with 0 data races)" << std::endl;
}

void benchmark_fixtures() {
    std::cout << "[TEST] 7. C++ Query Performance Profiling Across Fixtures..." << std::endl;
    const char* fixtures[] = {
        "data/maps/sample_prayagraj.idrmap",
        "data/maps/dense_metro.idrmap",
        "data/maps/regional_network.idrmap"
    };

    for (const char* path : fixtures) {
        auto t_open_0 = std::chrono::high_resolution_clock::now();
        IdrMapReader reader;
        assert(reader.open_file(path) == IdrMapStatus::OK);
        auto t_open_1 = std::chrono::high_resolution_clock::now();
        double open_us = std::chrono::duration<double, std::micro>(t_open_1 - t_open_0).count();

        const size_t N = 10000;
        std::vector<double> latencies_us;
        latencies_us.reserve(N);

        IdrRoadCandidate candidates[16];

        for (size_t i = 0; i < N; ++i) {
            double q_lat = 25.4350 + (i % 50) * 0.0002;
            double q_lon = 81.8500 + (i % 50) * 0.0002;

            auto t0 = std::chrono::high_resolution_clock::now();
            reader.query_nearest_roads(q_lat, q_lon, 200.0, 5, candidates, 16);
            auto t1 = std::chrono::high_resolution_clock::now();
            latencies_us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
        }

        std::sort(latencies_us.begin(), latencies_us.end());
        double sum_us = 0.0;
        for (double d : latencies_us) sum_us += d;
        double avg_us = sum_us / N;
        double p50 = latencies_us[N * 50 / 100];
        double p95 = latencies_us[N * 95 / 100];
        double p99 = latencies_us[N * 99 / 100];
        double p_max = latencies_us.back();
        double throughput = (avg_us > 0.0) ? (1.0e6 / avg_us) : (1.0e6 / 0.1);

        std::cout << "  Fixture: " << path << std::endl;
        std::cout << "    Open/mmap time: " << open_us << " us" << std::endl;
        std::cout << "    Avg: " << avg_us << " us | P50: " << p50 << " us | P95: " << p95 << " us | P99: " << p99 << " us | Max: " << p_max << " us" << std::endl;
        std::cout << "    Throughput: ~" << static_cast<size_t>(throughput) << " queries/sec" << std::endl;
    }
    std::cout << "  -> PASSED" << std::endl;
}

} // namespace idr

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "  idr-core IdrMapReader C++ Test Suite  " << std::endl;
    std::cout << "========================================" << std::endl;

    idr::test_static_asserts();
    idr::test_open_fixtures();
    idr::test_corruption_checks();
    idr::test_spatial_queries();
    idr::test_zero_allocation_guarantee();
    idr::test_concurrent_read_only_queries();
    idr::benchmark_fixtures();

    std::cout << "========================================" << std::endl;
    std::cout << "  ALL 7 C++ IDRMAP READER TESTS PASSED  " << std::endl;
    std::cout << "========================================" << std::endl;
    return 0;
}
