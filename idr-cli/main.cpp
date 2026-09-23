#include "idr/c_api.h"
#include "idr/navigation_engine.hpp"
#include "idr/gbdt_model.hpp"
#include "idr/feature_extractor.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <chrono>
#include <iomanip>

using namespace idr;

void print_usage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n"
              << "Options:\n"
              << "  --input <csv_path>      Path to input IMU/GNSS sensor CSV file\n"
              << "  --rate <hz>             Target rate simulation in Hz (default: 100)\n"
              << "  --outage <start> <end>  Inject simulated GNSS outage interval in seconds\n"
              << "  --benchmark             Run speed/throughput benchmark\n"
              << "  --output <csv_path>     Output CSV trajectory path\n"
              << "  --help                  Show this help message\n";
}

int main(int argc, char** argv) {
    std::string input_csv;
    std::string output_csv;
    double target_rate = 100.0;
    double outage_start = -1.0;
    double outage_end = -1.0;
    bool benchmark_mode = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--input" && i + 1 < argc) {
            input_csv = argv[++i];
        } else if (arg == "--output" && i + 1 < argc) {
            output_csv = argv[++i];
        } else if (arg == "--rate" && i + 1 < argc) {
            target_rate = std::stod(argv[++i]);
        } else if (arg == "--outage" && i + 2 < argc) {
            outage_start = std::stod(argv[++i]);
            outage_end = std::stod(argv[++i]);
        } else if (arg == "--benchmark") {
            benchmark_mode = true;
        } else if (arg == "--help") {
            print_usage(argv[0]);
            return 0;
        }
    }

    std::cout << "===============================================================\n"
              << "ISRO PS 26168: idr-cli Standalone Portable Edge Engine (C++17)\n"
              << "===============================================================\n";

    double dt = 1.0 / target_rate;
    SeamlessGNSSDeficitEngine engine(std::nullopt, std::nullopt, std::nullopt, 11.345, 9.0, dt);
    GBDTSpeedModel gbdt;
    SlidingWindowFeatureExtractor feat_ext(dt);

    if (benchmark_mode) {
        std::cout << "[BENCHMARK] Executing tactical 200 Hz IMU processing soak test...\n";
        const int TOTAL_CYCLES = 100000; // 100,000 cycles
        Vector3 a_p(0.2, 0.1, 9.80665);
        Vector3 w_p(0.01, -0.005, 0.02);

        auto start = std::chrono::high_resolution_clock::now();
        for (int c = 0; c < TOTAL_CYCLES; ++c) {
            double t = c * 0.005; // 200 Hz -> dt = 0.005s
            feat_ext.push(a_p, w_p);
            double speed = 10.0;
            if (feat_ext.is_full()) {
                std::array<double, SlidingWindowFeatureExtractor::NUM_TOP_FEATURES> feats{};
                feat_ext.compute_45_features(feats);
                speed = gbdt.predict(feats);
            }
            engine.process_step(t, a_p, w_p, 50.0, std::nullopt, false, speed, std::nullopt, std::nullopt, true);
        }
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> diff = end - start;
        double total_sec = diff.count();
        double sustained_hz = static_cast<double>(TOTAL_CYCLES) / total_sec;
        double avg_latency_us = (total_sec / static_cast<double>(TOTAL_CYCLES)) * 1e6;

        std::cout << "[BENCHMARK RESULT] Completed " << TOTAL_CYCLES << " cycles in "
                  << std::fixed << std::setprecision(3) << total_sec << " seconds.\n"
                  << "  Sustained Processing Rate: " << std::fixed << std::setprecision(1) << sustained_hz << " Hz\n"
                  << "  Mean Cycle Latency:       " << std::fixed << std::setprecision(2) << avg_latency_us << " us ("
                  << avg_latency_us / 1000.0 << " ms)\n"
                  << "  Requirement (>= 200 Hz):   " << (sustained_hz >= 200.0 ? "PASSED (EXCEEDED)" : "FAILED") << "\n";
        return 0;
    }

    if (!input_csv.empty()) {
        std::cout << "[INPUT] Processing CSV: " << input_csv << "\n";
        std::ifstream file(input_csv);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open input file " << input_csv << "\n";
            return 1;
        }

        std::ofstream out_file;
        if (!output_csv.empty()) {
            out_file.open(output_csv);
            out_file << "time_s,mode,pos_east,pos_north,pos_up,vel_east,vel_north,vel_up,speed_kmh,heading_deg,in_outage,p_trace\n";
        }

        std::string line;
        int line_count = 0;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#' || line[0] == 't') continue;
            std::stringstream ss(line);
            std::string token;
            std::vector<double> vals;
            while (std::getline(ss, token, ',')) {
                try {
                    vals.push_back(std::stod(token));
                } catch (...) {
                    vals.push_back(0.0);
                }
            }
            if (vals.size() < 7) continue;

            double time_s = vals[0];
            Vector3 a_p(vals[1], vals[2], vals[3]);
            Vector3 w_p(vals[4], vals[5], vals[6]);
            double baro_alt = (vals.size() > 7) ? vals[7] : 0.0;

            bool gnss_valid = (vals.size() > 10 && vals[8] != 0.0);
            if (outage_start >= 0.0 && time_s >= outage_start && time_s <= outage_end) {
                gnss_valid = false;
            }

            std::optional<Vector3> gnss_pos = std::nullopt;
            std::optional<Vector3> gnss_vel = std::nullopt;
            if (gnss_valid && vals.size() >= 14) {
                gnss_pos = Vector3(vals[9], vals[10], vals[11]);
                gnss_vel = Vector3(vals[12], vals[13], vals.size() > 14 ? vals[14] : 0.0);
            }

            feat_ext.push(a_p, w_p);
            double speed = 10.0;
            if (feat_ext.is_full()) {
                std::array<double, SlidingWindowFeatureExtractor::NUM_TOP_FEATURES> feats{};
                feat_ext.compute_45_features(feats);
                speed = gbdt.predict(feats);
            }

            auto telem = engine.process_step(
                time_s, a_p, w_p, baro_alt, gnss_pos, gnss_valid, speed, gnss_vel, std::nullopt, true
            );

            if (out_file.is_open()) {
                out_file << std::fixed << std::setprecision(4)
                         << telem.time_s << ","
                         << static_cast<int>(telem.mode) << ","
                         << telem.pos_enu.x << ","
                         << telem.pos_enu.y << ","
                         << telem.pos_enu.z << ","
                         << telem.vel_enu.x << ","
                         << telem.vel_enu.y << ","
                         << telem.vel_enu.z << ","
                         << telem.speed_kmh << ","
                         << telem.heading_deg << ","
                         << (telem.in_outage ? 1 : 0) << ","
                         << telem.diagnostics.p_trace << "\n";
            }
            line_count++;
        }
        std::cout << "[DONE] Processed " << line_count << " samples.\n";
        if (out_file.is_open()) {
            std::cout << "[OUTPUT] Saved trajectory to " << output_csv << "\n";
        }
    } else {
        std::cout << "No input specified. Run with --benchmark or --help.\n";
    }

    return 0;
}
