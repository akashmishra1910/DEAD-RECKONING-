#include "idr/navigation_engine.hpp"

namespace idr {

SeamlessGNSSDeficitEngine::SeamlessGNSSDeficitEngine(
    const std::optional<Vector3>& init_pos,
    const std::optional<Vector3>& init_vel,
    const std::optional<Vector3>& init_euler,
    double chi2_pos_threshold,
    double chi2_vel_threshold,
    double dt,
    bool enable_heading_supervisor,
    bool enable_map_matching,
    const std::optional<OfflineOSMGraph>& osm_graph,
    int min_reacquisition_dwell_epochs,
    bool enable_speed_calibration,
    bool enable_longitudinal_map_constraint
) : dt_(dt),
    chi2_pos_threshold_(chi2_pos_threshold),
    chi2_vel_threshold_(chi2_vel_threshold),
    min_reacquisition_dwell_(min_reacquisition_dwell_epochs),
    alignment_engine_(),
    vibration_filter_(dt),
    eskf_(init_pos, init_vel, init_euler),
    enable_heading_supervisor_(enable_heading_supervisor),
    enable_map_matching_(enable_map_matching),
    enable_speed_calibration_(enable_speed_calibration),
    enable_longitudinal_map_constraint_(enable_longitudinal_map_constraint)
{
    if (enable_heading_supervisor_) {
        heading_supervisor_ = std::make_unique<HeadingSupervisor>(6.635);
    }
    if (enable_map_matching_) {
        OfflineOSMGraph g = static_cast<bool>(osm_graph) ? *osm_graph : OfflineOSMGraph::build_synthetic_3d_corridor();
        map_matcher_ = std::make_unique<Causal3DMapMatcher>(g, 5.0, 1.8, 0.30, 5.0, 4.0);
    }
    if (enable_speed_calibration_) {
        speed_calibrator_ = std::make_unique<OnlineSpeedScaleBiasCalibrator>();
    }
    if (enable_longitudinal_map_constraint_) {
        longitudinal_synchronizer_ = std::make_unique<CausalLongitudinalMapSynchronizer>();
    }

    if (static_cast<bool>(init_pos)) {
        last_pos_ = *init_pos;
    }
}

std::tuple<bool, bool, double, double> SeamlessGNSSDeficitEngine::evaluate_gnss_outage_causal(
    double time_s,
    const std::optional<Vector3>& gnss_pos,
    const std::optional<Vector3>& gnss_vel,
    bool gnss_valid
) {
    if (!gnss_valid || !static_cast<bool>(gnss_pos)) {
        return {true, false, 999.0, 999.0};
    }

    if (mode_ == NavigationMode::GNSS_AIDED || mode_ == NavigationMode::GNSS_DEGRADED) {
        if (last_gnss_time_ >= 0.0 && (time_s - last_gnss_time_) > 1.5) {
            return {true, false, 999.0, 999.0};
        }
    }

    if (mode_ == NavigationMode::INITIALIZING) {
        return {false, false, 0.0, 0.0};
    }

    // Position Innovation Gate
    Vector3 dz_pos = *gnss_pos - eskf_.get_position();
    const auto& P = eskf_.get_covariance();

    Matrix3x3 S_pos;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            S_pos(r, c) = P(r, c);
        }
        S_pos(r, r) += (2.5 * 2.5);
    }

    Matrix3x3 S_pos_inv;
    double gamma_pos = 999.0;
    if (invert_matrix3x3(S_pos, S_pos_inv)) {
        Vector3 tmp = S_pos_inv * dz_pos;
        gamma_pos = dz_pos.dot(tmp);
    }

    // Velocity Innovation Gate
    double gamma_vel = 0.0;
    if (static_cast<bool>(gnss_vel)) {
        Vector3 dz_vel = *gnss_vel - eskf_.get_velocity();
        Matrix3x3 S_vel;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                S_vel(r, c) = P(3 + r, 3 + c);
            }
            S_vel(r, r) += (0.20 * 0.20);
        }
        Matrix3x3 S_vel_inv;
        if (invert_matrix3x3(S_vel, S_vel_inv)) {
            Vector3 tmp = S_vel_inv * dz_vel;
            gamma_vel = dz_vel.dot(tmp);
        }
    }

    bool is_severe_pos_jump = (gamma_pos > 40.0);
    bool is_severe_vel_jump = (static_cast<bool>(gnss_vel) && gamma_vel > 25.0);
    bool is_mild_pos_jump = (gamma_pos > 20.0);
    bool is_mild_vel_jump = (static_cast<bool>(gnss_vel) && gamma_vel > 12.0);


    if (is_severe_pos_jump && is_severe_vel_jump) {
        return {true, false, gamma_pos, gamma_vel};
    } else if (is_mild_pos_jump || is_mild_vel_jump) {
        return {false, true, gamma_pos, gamma_vel};
    }

    return {false, false, gamma_pos, gamma_vel};
}

void SeamlessGNSSDeficitEngine::update_supervisor_state_machine(double time_s, bool is_outage, bool is_degraded) {
    NavigationMode prev_mode = mode_;

    if (!is_outage && !is_degraded) {
        consecutive_valid_gnss_++;
        consecutive_outage_epochs_ = 0;
    } else {
        consecutive_outage_epochs_++;
        consecutive_valid_gnss_ = 0;
    }

    if (mode_ == NavigationMode::INITIALIZING) {
        if (!is_outage) {
            mode_ = NavigationMode::GNSS_AIDED;
            last_transition_reason_ = "Initial GNSS Lock Acquired";
            if (speed_calibrator_) speed_calibrator_->unfreeze();
        } else {
            mode_ = NavigationMode::DR_ONLY;
            last_transition_reason_ = "Init in GNSS Outage";
            outage_start_time_ = time_s;
            if (speed_calibrator_) speed_calibrator_->freeze_for_outage();
        }
    } else if (mode_ == NavigationMode::GNSS_AIDED) {
        if (is_outage) {
            mode_ = NavigationMode::DR_ONLY;
            last_transition_reason_ = "GNSS Loss-of-Lock Detected -> Pure DR Mode";
            outage_start_time_ = time_s;
            outage_distance_traveled_ = 0.0;
            if (speed_calibrator_) speed_calibrator_->freeze_for_outage();
        } else if (is_degraded) {
            mode_ = NavigationMode::GNSS_DEGRADED;
            last_transition_reason_ = "GNSS Fix Accuracy Degraded";
        }
    } else if (mode_ == NavigationMode::GNSS_DEGRADED) {
        if (is_outage) {
            mode_ = NavigationMode::DR_ONLY;
            last_transition_reason_ = "Degraded GNSS Lost Completely -> Pure DR Mode";
            outage_start_time_ = time_s;
            outage_distance_traveled_ = 0.0;
            if (speed_calibrator_) speed_calibrator_->freeze_for_outage();
        } else if (!is_degraded && consecutive_valid_gnss_ >= 3) {
            mode_ = NavigationMode::GNSS_AIDED;
            last_transition_reason_ = "Clean Line-of-Sight Restored";
            if (speed_calibrator_) speed_calibrator_->unfreeze();
        }
    } else if (mode_ == NavigationMode::TRANSITION_TO_DR || mode_ == NavigationMode::DR_ONLY) {
        if (!is_outage) {
            mode_ = NavigationMode::TRANSITION_TO_GNSS;
            last_transition_reason_ = "GNSS Return Detected -> Innovation Check";
            reacquisition_epochs_ = 0;
        } else {
            mode_ = NavigationMode::DR_ONLY;
        }
    } else if (mode_ == NavigationMode::TRANSITION_TO_GNSS) {
        if (is_outage) {
            mode_ = NavigationMode::DR_ONLY;
            last_transition_reason_ = "GNSS Lost during Recovery -> Return to DR";
            reacquisition_epochs_ = 0;
            if (speed_calibrator_) speed_calibrator_->freeze_for_outage();
        }
    } else if (mode_ == NavigationMode::GNSS_REACQUIRED) {
        if (is_outage) {
            mode_ = NavigationMode::DR_ONLY;
            last_transition_reason_ = "Secondary Lock Failure -> Return to DR";
            outage_start_time_ = time_s;
            if (speed_calibrator_) speed_calibrator_->freeze_for_outage();
        } else {
            mode_ = NavigationMode::GNSS_AIDED;
            last_transition_reason_ = "Full GNSS-Aided Authority Restored";
            if (speed_calibrator_) speed_calibrator_->unfreeze();
        }
    }

    if (mode_ != prev_mode) {
        mode_entry_time_ = time_s;
    }
}

NavigationTelemetry SeamlessGNSSDeficitEngine::process_step(
    double time_s,
    const Vector3& accel_raw_p,
    const Vector3& gyro_raw_p,
    double baro_alt,
    const std::optional<Vector3>& gnss_pos,
    bool gnss_valid,
    std::optional<double> ai_speed,
    const std::optional<Vector3>& gnss_vel,
    const std::optional<Vector3>& mag_raw_p,
    bool enable_road_constraints
) {
    bool is_stationary = (static_cast<bool>(ai_speed) && *ai_speed < 0.2) ||
                         (gnss_valid && static_cast<bool>(gnss_pos) && eskf_.get_velocity().norm() < 0.2);

    alignment_engine_.process_sample(accel_raw_p, gyro_raw_p, is_stationary);
    Vector3 accel_b = alignment_engine_.transform_phone_to_vehicle(accel_raw_p);
    Vector3 gyro_b = alignment_engine_.transform_phone_to_vehicle(gyro_raw_p);

    VibrationState vib_state = vibration_filter_.process(accel_b, gyro_b);
    Vector3 clean_accel_b = vib_state.cleaned_accel;
    Vector3 clean_gyro_b = vib_state.cleaned_gyro;

    eskf_.predict(clean_accel_b, clean_gyro_b, dt_);
    eskf_.update_altitude(baro_alt, 0.8);

    std::optional<HeadingSupervisorTelemetry> heading_telem = std::nullopt;
    if (heading_supervisor_) {
        heading_telem = heading_supervisor_->process_step(
            time_s, eskf_, alignment_engine_.get_R_p2b(), gnss_vel, gnss_valid, mag_raw_p, clean_gyro_b.z, true
        );
    }

    bool is_outage = false;
    bool is_degraded = false;
    double gamma_pos = 0.0;
    double gamma_vel = 0.0;
    std::tie(is_outage, is_degraded, gamma_pos, gamma_vel) = evaluate_gnss_outage_causal(time_s, gnss_pos, gnss_vel, gnss_valid);
    update_supervisor_state_machine(time_s, is_outage, is_degraded);

    if (!is_outage && static_cast<bool>(gnss_pos)) {
        last_gnss_time_ = time_s;
    }

    bool in_outage = (mode_ == NavigationMode::TRANSITION_TO_DR || mode_ == NavigationMode::DR_ONLY || mode_ == NavigationMode::TRANSITION_TO_GNSS);
    double outage_duration = in_outage ? (time_s - outage_start_time_) : 0.0;

    double ai_weight = 0.0;
    double gnss_weight = 0.0;
    bool map_active = false;
    const char* map_conf_str = "INVALID";
    bool long_applied = false;
    double long_innov = 0.0;

    if (mode_ == NavigationMode::GNSS_AIDED) {
        if (static_cast<bool>(gnss_pos)) {
            eskf_.update_gnss(*gnss_pos, 2.5);
        }
        if (static_cast<bool>(gnss_vel)) {
            eskf_.update_gnss_velocity(*gnss_vel, 0.20);
            gnss_weight = 1.0;
        }
        if (speed_calibrator_ && static_cast<bool>(ai_speed)) {
            double g_spd = static_cast<bool>(gnss_vel) ? std::hypot(gnss_vel->x, gnss_vel->y) : 0.0;
            speed_calibrator_->update_gnss_aiding(time_s, *ai_speed, g_spd, gnss_valid);
        }
        eskf_.update_nhc(0.08, 0.08);
        if (vib_state.is_engine_idling) {
            eskf_.update_zupt(0.01);
        }
    } else if (mode_ == NavigationMode::GNSS_DEGRADED) {
        if (static_cast<bool>(gnss_pos) && gamma_pos <= chi2_pos_threshold_) {
            eskf_.update_gnss(*gnss_pos, 6.0);
        }
        if (static_cast<bool>(gnss_vel) && gamma_vel <= chi2_vel_threshold_) {
            eskf_.update_gnss_velocity(*gnss_vel, 0.8);
            gnss_weight = 0.4;
            if (speed_calibrator_ && static_cast<bool>(ai_speed)) {
                double g_spd = std::hypot(gnss_vel->x, gnss_vel->y);
                speed_calibrator_->update_gnss_aiding(time_s, *ai_speed, g_spd, gnss_valid);
            }
        }
        if (static_cast<bool>(ai_speed)) {
            double cal_speed = speed_calibrator_ ? speed_calibrator_->correct_speed(*ai_speed) : *ai_speed;
            double eff_speed = vib_state.is_engine_idling ? 0.0 : cal_speed;
            eskf_.update_ai_velocity(eff_speed, 0.50);
            ai_weight = 0.60;
        }
        eskf_.update_nhc(0.06, 0.06);
        if (vib_state.is_engine_idling || (static_cast<bool>(ai_speed) && *ai_speed < 0.2)) {
            eskf_.update_zupt(0.01);
        }
    } else if (mode_ == NavigationMode::TRANSITION_TO_DR || mode_ == NavigationMode::DR_ONLY) {
        if (static_cast<bool>(ai_speed)) {
            double cal_speed = speed_calibrator_ ? speed_calibrator_->correct_speed(*ai_speed) : *ai_speed;
            double eff_speed = vib_state.is_engine_idling ? 0.0 : cal_speed;
            double turn_rate = std::abs(clean_gyro_b.z);
            double vib_energy = (accel_b - clean_accel_b).norm();
            double r_ai_adapt = 0.35 * (1.0 + 0.5 * vib_energy + 1.2 * turn_rate);
            eskf_.update_ai_velocity(eff_speed, r_ai_adapt);
            ai_weight = 1.0;
        }
        eskf_.update_nhc(0.05, 0.05);
        if (vib_state.is_engine_idling || (static_cast<bool>(ai_speed) && *ai_speed < 0.2)) {
            eskf_.update_zupt(0.01);
        }
        if (enable_map_matching_ && map_matcher_ && enable_road_constraints) {
            double v_mag = eskf_.get_velocity().norm();
            MapMatchResult match_res = map_matcher_->process_step(
                eskf_.get_position(), eskf_.get_yaw(), baro_alt, v_mag, dt_, true
            );
            map_conf_str = map_match_confidence_to_string(match_res.confidence);
            last_match_result_ = match_res;

            // Apply soft road constraints
            if (match_res.confidence == MapMatchConfidence::HIGH || match_res.confidence == MapMatchConfidence::MEDIUM) {
                double r_hdg = (match_res.confidence == MapMatchConfidence::HIGH) ? 0.087 : 0.174;
                double r_cross = (match_res.confidence == MapMatchConfidence::HIGH) ? 1.75 : 3.50;
                if (v_mag > 2.0) {
                    eskf_.update_road_heading(match_res.road_heading, r_hdg, 9.0);
                }
                eskf_.update_road_position(match_res.snapped_pos, match_res.road_tangent_2d, r_cross, 16.0);
                map_active = true;
            }

            if (enable_longitudinal_map_constraint_ && longitudinal_synchronizer_) {
                auto l_res = longitudinal_synchronizer_->process_step(
                    time_s, eskf_, match_res, clean_gyro_b.z, dt_, nullptr, &map_matcher_->get_graph()
                );
                long_applied = l_res.applied;
                long_innov = l_res.innovation_m;
            }
        }
    } else if (mode_ == NavigationMode::TRANSITION_TO_GNSS) {
        bool is_fix_accepted = (gamma_pos <= chi2_pos_threshold_);
        if (is_fix_accepted) {
            reacquisition_epochs_++;
            if (static_cast<bool>(gnss_pos)) {
                eskf_.update_gnss(*gnss_pos, 5.0);
            }
            if (static_cast<bool>(gnss_vel) && gamma_vel <= chi2_vel_threshold_) {
                eskf_.update_gnss_velocity(*gnss_vel, 0.6);
                gnss_weight = 0.5;
            }
            if (reacquisition_epochs_ >= min_reacquisition_dwell_) {
                mode_ = NavigationMode::GNSS_REACQUIRED;
                last_transition_reason_ = "Reacquisition Confirmed (Dwell Passed)";
            }
        } else {
            reacquisition_epochs_ = 0;
            last_transition_reason_ = "Multipath Spike Rejected in Recovery";
            if (static_cast<bool>(ai_speed)) {
                double cal_speed = speed_calibrator_ ? speed_calibrator_->correct_speed(*ai_speed) : *ai_speed;
                double eff_speed = vib_state.is_engine_idling ? 0.0 : cal_speed;
                eskf_.update_ai_velocity(eff_speed, 0.45);
                ai_weight = 0.5;
            }
        }
        eskf_.update_nhc(0.05, 0.05);
        if (vib_state.is_engine_idling || (static_cast<bool>(ai_speed) && *ai_speed < 0.2)) {
            eskf_.update_zupt(0.01);
        }
    } else if (mode_ == NavigationMode::GNSS_REACQUIRED) {
        if (static_cast<bool>(gnss_pos) && gamma_pos <= chi2_pos_threshold_) {
            eskf_.update_gnss(*gnss_pos, 3.0);
        }
        if (static_cast<bool>(gnss_vel) && gamma_vel <= chi2_vel_threshold_) {
            eskf_.update_gnss_velocity(*gnss_vel, 0.3);
            gnss_weight = 0.9;
        }
        eskf_.update_nhc(0.06, 0.06);
        if (vib_state.is_engine_idling) {
            eskf_.update_zupt(0.01);
        }
    }

    double step_dist = std::hypot(eskf_.get_position().x - last_pos_.x, eskf_.get_position().y - last_pos_.y);
    total_distance_traveled_ += step_dist;
    if (in_outage) {
        outage_distance_traveled_ += step_dist;
    }
    last_pos_ = eskf_.get_position();

    double speed_kmh = std::hypot(eskf_.get_velocity().x, eskf_.get_velocity().y) * 3.6;
    double heading_deg = std::fmod(eskf_.get_yaw() * RAD_TO_DEG + 360.0, 360.0);
    double pitch_deg = eskf_.get_pitch() * RAD_TO_DEG;
    double roll_deg = eskf_.get_roll() * RAD_TO_DEG;

    CalibratorState cal_st = speed_calibrator_ ? speed_calibrator_->get_state() : CalibratorState();

    SupervisorDiagnostics diag;
    diag.last_transition_reason = last_transition_reason_;
    diag.time_in_current_mode_s = time_s - mode_entry_time_;
    diag.gnss_pos_innovation_gamma = gamma_pos;
    diag.gnss_vel_innovation_gamma = gamma_vel;
    diag.reacquisition_confirmed_epochs = reacquisition_epochs_;
    diag.consecutive_outage_epochs = consecutive_outage_epochs_;
    diag.consecutive_valid_gnss_epochs = consecutive_valid_gnss_;
    diag.ai_vel_weight = ai_weight;
    diag.gnss_vel_weight = gnss_weight;
    diag.map_constraints_active = map_active;
    diag.map_confidence = map_conf_str;
    diag.p_trace = eskf_.get_covariance().trace();
    diag.speed_scale_est = cal_st.scale;
    diag.speed_bias_est_mps = cal_st.bias_mps;
    diag.speed_calibrated = cal_st.is_calibrated;
    diag.longitudinal_constraint_applied = long_applied;
    diag.longitudinal_innovation_m = long_innov;

    NavigationTelemetry telem;
    telem.mode = mode_;
    telem.time_s = time_s;
    telem.pos_enu = eskf_.get_position();
    telem.vel_enu = eskf_.get_velocity();
    telem.speed_kmh = speed_kmh;
    telem.heading_deg = heading_deg;
    telem.pitch_deg = pitch_deg;
    telem.roll_deg = roll_deg;
    telem.accel_bias = eskf_.get_accel_bias();
    telem.gyro_bias = eskf_.get_gyro_bias();
    telem.in_outage = in_outage;
    telem.outage_duration_s = outage_duration;
    telem.outage_dist_traveled_m = outage_distance_traveled_;
    telem.gnss_valid = gnss_valid;
    telem.is_pothole_active = vib_state.is_pothole_detected;
    telem.pothole_count = vib_state.pothole_event_count;
    telem.is_idling = vib_state.is_engine_idling;
    for (size_t i = 0; i < 15; ++i) {
        telem.covariance_diag[i] = eskf_.get_covariance()(i, i);
    }
    telem.heading_telemetry = heading_telem;
    telem.diagnostics = diag;

    // Explicit Separation of Raw DR and Map-Matched Presentation State (Phase 11.8)
    telem.raw_dr_pos_enu = eskf_.get_position();
    telem.map_available = (enable_map_matching_ && map_matcher_ != nullptr);
    if (static_cast<bool>(last_match_result_) && last_match_result_->is_valid) {

        telem.map_matched_pos_enu = last_match_result_->snapped_pos;
        telem.selected_edge_id = last_match_result_->edge_id;
        telem.selected_layer = last_match_result_->layer;
        telem.selected_edge_name = last_match_result_->edge_name;
        telem.selected_layer_label = last_match_result_->layer_label;
        telem.map_confidence = last_match_result_->confidence;
        telem.map_ambiguous = last_match_result_->is_ambiguous;
        telem.candidate_count = last_match_result_->candidate_count;
        telem.hypothesis_count = last_match_result_->hypothesis_count;
        telem.score_margin = last_match_result_->score_margin;
    } else {
        telem.map_matched_pos_enu = eskf_.get_position();
        telem.selected_edge_id = -1;
        telem.selected_layer = 0;
        telem.selected_edge_name = "Unknown";
        telem.selected_layer_label = "Ground";
        telem.map_confidence = MapMatchConfidence::INVALID;
        telem.map_ambiguous = false;
        telem.candidate_count = 0;
        telem.hypothesis_count = 0;
        telem.score_margin = 0.0;
    }

    return telem;
}

bool SeamlessGNSSDeficitEngine::load_idrmap(const char* path, double lat0, double lon0, double alt0) {
    if (!path) return false;
    owned_idrmap_reader_ = std::make_unique<IdrMapReader>();
    if (owned_idrmap_reader_->open_file(path) != IdrMapStatus::OK) {
        owned_idrmap_reader_.reset();
        return false;
    }
    map_matcher_ = std::make_unique<Causal3DMapMatcher>(
        owned_idrmap_reader_.get(), lat0, lon0, alt0
    );
    enable_map_matching_ = true;
    last_match_result_ = std::nullopt;
    return true;
}


bool SeamlessGNSSDeficitEngine::set_idrmap_reader(const IdrMapReader* reader, double lat0, double lon0, double alt0) {
    if (!reader || !reader->is_open()) return false;
    owned_idrmap_reader_.reset();
    map_matcher_ = std::make_unique<Causal3DMapMatcher>(
        reader, lat0, lon0, alt0
    );
    enable_map_matching_ = true;
    last_match_result_ = std::nullopt;
    return true;
}

void SeamlessGNSSDeficitEngine::reset() {
    eskf_.reset();
    alignment_engine_.reset();
    vibration_filter_.reset();
    if (heading_supervisor_) heading_supervisor_->reset();
    if (map_matcher_) map_matcher_->reset();
    if (speed_calibrator_) speed_calibrator_->reset();
    if (longitudinal_synchronizer_) longitudinal_synchronizer_->reset();

    mode_ = NavigationMode::INITIALIZING;
    mode_entry_time_ = 0.0;
    last_transition_reason_ = "Engine Reset";
    consecutive_valid_gnss_ = 0;
    consecutive_outage_epochs_ = 0;
    reacquisition_epochs_ = 0;
    last_gnss_time_ = -1.0;
    outage_start_time_ = 0.0;
    outage_distance_traveled_ = 0.0;
    total_distance_traveled_ = 0.0;
    last_pos_ = Vector3(0.0, 0.0, 0.0);
    last_match_result_ = std::nullopt;
}

} // namespace idr
