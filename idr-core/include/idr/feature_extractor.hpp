#pragma once

#include "idr/types.hpp"
#include "idr/gbdt_model.hpp"
#include <array>
#include <cstddef>

namespace idr {

class SlidingWindowFeatureExtractor {
public:
    static constexpr std::size_t WINDOW_SIZE = 10;
    static constexpr std::size_t NUM_RAW_FEATURES = 57;
    static constexpr std::size_t NUM_TOP_FEATURES = 45;

    explicit SlidingWindowFeatureExtractor(double dt = 0.1);

    void reset();

    // Ingest a new IMU sample (body frame)
    void push(const Vector3& accel_b, const Vector3& gyro_b);

    // Compute all 57 raw features
    void compute_57_features(double* out_57) const;

    // Compute the 45 features ordered for GBDTSpeedModel
    void compute_45_features(double* out_45) const;
    void compute_45_features(std::array<double, NUM_TOP_FEATURES>& out_45) const;

    std::size_t count() const { return m_count; }
    bool is_full() const { return m_count >= WINDOW_SIZE; }

private:
    double m_dt;
    std::array<Vector3, WINDOW_SIZE> m_accel_buf{};
    std::array<Vector3, WINDOW_SIZE> m_gyro_buf{};
    std::size_t m_head{0};
    std::size_t m_count{0};
};

} // namespace idr
