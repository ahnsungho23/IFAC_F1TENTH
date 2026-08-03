// ================================================================================================
// Frenet 장애물 추적기 구현
// ================================================================================================

#include "obstacle_detector/obstacle_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace obstacle_detector
{

void ObstacleTracker::configure(const TrackerParams &params, const FrenetProjector *frenet)
{
    p_ = params;
    frenet_ = frenet;
}

double ObstacleTracker::frenetDistSquared(double s1, double d1, double s2, double d2) const
{
    // s에는 폐루프 최소 거리를, d에는 일반 직선 거리를 적용한다.
    const double ds = frenet_ ? frenet_->wrapDelta(s1, s2) : (s1 - s2);
    const double dd = d1 - d2;
    return ds * ds + dd * dd;
}

void ObstacleTracker::predict(Track &t, double dt) const
{
    // 지나치게 긴 입력 간격이 한 번에 공분산을 폭증시키지 않도록 예측 시간을 제한한다.
    dt = std::clamp(dt, 0.0, p_.dt_max);

    Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
    F(0, 1) = dt;  // s += vs * dt
    F(2, 3) = dt;  // d += vd * dt

    // 이산 백색 가속도 프로세스 잡음을 (s, vs)와 (d, vd) 축에 독립적으로 적용한다.
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;
    Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
    // 종방향 s / vs 블록
    Q(0, 0) = 0.25 * dt4 * p_.process_var_vs;
    Q(0, 1) = 0.5 * dt3 * p_.process_var_vs;
    Q(1, 0) = 0.5 * dt3 * p_.process_var_vs;
    Q(1, 1) = dt2 * p_.process_var_vs;
    // 횡방향 d / vd 블록
    Q(2, 2) = 0.25 * dt4 * p_.process_var_vd;
    Q(2, 3) = 0.5 * dt3 * p_.process_var_vd;
    Q(3, 2) = 0.5 * dt3 * p_.process_var_vd;
    Q(3, 3) = dt2 * p_.process_var_vd;

    t.x = F * t.x;
    t.P = F * t.P * F.transpose() + Q;
}

Eigen::Matrix2d ObstacleTracker::measurementCovariance(const Detection &detection) const
{
    // 검출 단계에서 계산한 품질 배율을 기본 s/d 측정 분산에 적용한다.
    const double scale =
        std::isfinite(detection.variance_scale) ? std::max(1.0, detection.variance_scale) : 1.0;
    Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
    R(0, 0) = p_.meas_var_s * scale;
    R(1, 1) = p_.meas_var_d * scale;
    return R;
}

double ObstacleTracker::innovationDistanceSquared(
    const Track &t, const Detection &detection) const
{
    Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
    H(0, 0) = 1.0;
    H(1, 2) = 1.0;

    Eigen::Vector2d innovation;
    innovation(0) = frenet_ ?
        frenet_->wrapDelta(detection.s, t.x(0)) : detection.s - t.x(0);
    innovation(1) = detection.d - t.x(2);

    // innovation 공분산 S가 양의 정부호가 아니면 안전하게 연관 후보에서 제외한다.
    const Eigen::Matrix2d S =
        H * t.P * H.transpose() + measurementCovariance(detection);
    const Eigen::LDLT<Eigen::Matrix2d> solver(S);
    if (solver.info() != Eigen::Success || !solver.isPositive())
    {
        return std::numeric_limits<double>::infinity();
    }
    const Eigen::Vector2d solved = solver.solve(innovation);
    if (solver.info() != Eigen::Success || !solved.allFinite())
    {
        return std::numeric_limits<double>::infinity();
    }
    return innovation.dot(solved);
}

void ObstacleTracker::kalmanUpdate(Track &t, const Detection &detection) const
{
    // 측정 행렬 H는 상태 [s, vs, d, vd] 중 s와 d만 선택한다.
    Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
    H(0, 0) = 1.0;
    H(1, 2) = 1.0;

    const Eigen::Matrix2d R = measurementCovariance(detection);

    // s innovation에는 시작/끝 경계를 넘을 때의 폐루프 차이를 적용한다.
    Eigen::Vector2d z;
    z << detection.s, detection.d;
    Eigen::Vector2d hx;
    hx << t.x(0), t.x(2);
    Eigen::Vector2d y = z - hx;
    if (frenet_)
    {
        y(0) = frenet_->wrapDelta(detection.s, t.x(0));
    }

    Eigen::Matrix2d S = H * t.P * H.transpose() + R;
    Eigen::Matrix<double, 4, 2> K = t.P * H.transpose() * S.inverse();
    t.x = t.x + K * y;
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    t.P = (I - K * H) * t.P;

    // 갱신된 s를 폐루프의 정규 범위 [0, track_length)로 되돌린다.
    if (frenet_ && frenet_->raceline_length() > 0.0)
    {
        const double L = frenet_->raceline_length();
        t.x(0) = std::fmod(t.x(0), L);
        if (t.x(0) < 0.0)
        {
            t.x(0) += L;
        }
    }
}

double ObstacleTracker::velocityMahalanobisSquared(
    const Track &t, double rel_vs, double rel_vd) const
{
    // 속도 상태의 2x2 부분 공분산으로 map-flow 대비 속도가 0과 얼마나 유의하게 다른지 본다.
    Eigen::Matrix2d velocity_covariance;
    velocity_covariance << t.P(1, 1), t.P(1, 3),
                           t.P(3, 1), t.P(3, 3);
    const Eigen::LDLT<Eigen::Matrix2d> solver(velocity_covariance);
    if (solver.info() != Eigen::Success || !solver.isPositive())
    {
        return 0.0;
    }

    const Eigen::Vector2d relative_velocity(rel_vs, rel_vd);
    const Eigen::Vector2d solved = solver.solve(relative_velocity);
    if (solver.info() != Eigen::Success || !solved.allFinite())
    {
        return 0.0;
    }
    return std::max(0.0, relative_velocity.dot(solved));
}

void ObstacleTracker::classify(
    Track &t, double ego_yaw_rate, bool yaw_rate_fresh) const
{
    // 검출 확인과 움직임 분류는 의도적으로 별도 상태 기계로 관리한다.
    // 1..(min_hits_confirm-1)번째 관측은 발행하지 않는다. 확인 관측이 들어오면 즉시 임시 정적
    // 트랙으로 발행하고, 이후 관측부터 정적/동적 움직임 증거를 누적한다.
    if (!t.classified)
    {
        t.motion_class = MotionClass::Pending;
        t.is_static = true;
        t.dyn_streak = 0;
        t.static_streak = 0;
        t.relative_speed = 0.0;
        t.velocity_mahalanobis_sq = 0.0;
        t.dynamic_motion_reliable = false;
        if (t.is_visible && t.hits >= std::max(1, p_.min_hits_confirm))
        {
            t.classified = true;
            t.motion_class = MotionClass::ProvisionalStatic;
        }
        return;
    }

    // 움직임 증거는 연속된 실제 측정에서만 얻으며 예측만 수행한 프레임에서는 누적하지 않는다.
    if (!t.is_visible)
    {
        t.dyn_streak = 0;
        t.static_streak = 0;
        t.dynamic_motion_reliable = false;
        t.is_static = t.motion_class != MotionClass::Dynamic;
        return;
    }

    // 속도 증거: map-flow 기준에 대한 상대 속도와 히스테리시스를 사용한다.
    const double rel_vs = t.vs() - static_ref_vs_;
    const double rel_vd = t.vd() - static_ref_vd_;
    const double rel_speed = std::hypot(rel_vs, rel_vd);
    t.relative_speed = rel_speed;
    t.velocity_mahalanobis_sq = velocityMahalanobisSquared(t, rel_vs, rel_vd);

    const bool yaw_reliable =
        yaw_rate_fresh && std::isfinite(ego_yaw_rate) &&
        (p_.dyn_max_abs_yaw_rate <= 0.0 ||
         std::abs(ego_yaw_rate) <= p_.dyn_max_abs_yaw_rate);
    const bool velocity_confident =
        p_.dyn_velocity_mahalanobis_gate <= 0.0 ||
        t.velocity_mahalanobis_sq >= p_.dyn_velocity_mahalanobis_gate;
    const bool vel_dynamic =
        rel_speed > p_.dyn_vel_enter && velocity_confident && yaw_reliable;
    const bool vel_static = rel_speed < p_.dyn_vel_exit;
    t.dynamic_motion_reliable = vel_dynamic;

    // 위치 분산 투표: 최근 (s, d) 이력의 표준편차를 쓰는 ForzaETH 방식
    bool std_dynamic = false;
    bool std_static = false;
    const bool uses_std_classifier = p_.classifier_mode != ClassifierMode::Velocity;
    if (uses_std_classifier && static_cast<int>(t.hist.size()) >= p_.min_nb_meas)
    {
        double mean_s = 0.0;
        double mean_d = 0.0;
        for (const auto &h : t.hist)
        {
            mean_s += h.first;
            mean_d += h.second;
        }
        mean_s /= static_cast<double>(t.hist.size());
        mean_d /= static_cast<double>(t.hist.size());
        double var_s = 0.0;
        double var_d = 0.0;
        for (const auto &h : t.hist)
        {
            const double es = frenet_ ? frenet_->wrapDelta(h.first, mean_s) : (h.first - mean_s);
            const double ed = h.second - mean_d;
            var_s += es * es;
            var_d += ed * ed;
        }
        var_s /= static_cast<double>(t.hist.size());
        var_d /= static_cast<double>(t.hist.size());
        const double std_s = std::sqrt(var_s);
        const double std_d = std::sqrt(var_d);
        if (std_s < p_.min_std && std_d < p_.min_std)
        {
            std_static = true;
        }
        else if (std_s > p_.max_std || std_d > p_.max_std)
        {
            std_dynamic = true;
        }
    }

    bool dynamic_evidence = false;
    bool static_evidence = false;
    switch (p_.classifier_mode)
    {
        case ClassifierMode::Velocity:
            dynamic_evidence = vel_dynamic;
            static_evidence = vel_static;
            break;
        case ClassifierMode::Std:
            dynamic_evidence = std_dynamic && yaw_reliable;
            static_evidence = std_static;
            break;
        case ClassifierMode::Both:
            dynamic_evidence = vel_dynamic && std_dynamic;
            static_evidence = vel_static || std_static;
            break;
    }

    if (dynamic_evidence)
    {
        ++t.dyn_streak;
        t.static_streak = 0;
    }
    else if (static_evidence)
    {
        ++t.static_streak;
        t.dyn_streak = 0;
    }
    else
    {
        // 히스테리시스 중간 구간, 낮은 속도 신뢰도, 급격하거나 오래된 자차 yaw는 모호한 관측이다.
        // 이런 관측을 조용히 누적하지 않고 연속 증거 streak를 끊는다.
        t.dyn_streak = 0;
        t.static_streak = 0;
    }

    const int static_frames = std::max(1, p_.static_confirm_frames);
    const int dynamic_frames = std::max(1, p_.dynamic_confirm_frames);
    if (t.motion_class == MotionClass::Dynamic)
    {
        if (t.static_streak >= static_frames)
        {
            t.motion_class = MotionClass::ConfirmedStatic;
            t.static_streak = 0;
        }
    }
    else if (t.dyn_streak >= dynamic_frames)
    {
        t.motion_class = MotionClass::Dynamic;
        t.dyn_streak = 0;
    }
    else if (t.motion_class == MotionClass::ProvisionalStatic &&
             t.static_streak >= static_frames)
    {
        t.motion_class = MotionClass::ConfirmedStatic;
        t.static_streak = 0;
    }

    t.is_static = t.motion_class != MotionClass::Dynamic;
}

void ObstacleTracker::updateStaticReference()
{
    // map-flow 기준은 명확히 느린 트랙들의 평균 Frenet 속도다. 벽은 계층 1 지도 필터에서 이미
    // 제거되므로 주로 정지 장애물과 남은 정적 구조물이 이 집합을 구성한다. 이들의 공통 겉보기
    // 속도를 기준으로 상대 차량의 움직임을 측정한다. 속도 게이트가 빠른 상대 차량을 평균에서
    // 제외하므로 상대 차량이 자기 자신을 기준 속도로 만들어 버리지 않는다.
    double sum_vs = 0.0;
    double sum_vd = 0.0;
    int n = 0;
    const double static_ref_gate_sq = p_.static_ref_gate * p_.static_ref_gate;
    for (const auto &t : tracks_)
    {
        const double speed_sq = t.vs() * t.vs() + t.vd() * t.vd();
        if (p_.static_ref_gate > 0.0 && speed_sq < static_ref_gate_sq)
        {
            sum_vs += t.vs();
            sum_vd += t.vd();
            ++n;
        }
    }
    if (n > 0)
    {
        static_ref_vs_ = sum_vs / static_cast<double>(n);
        static_ref_vd_ = sum_vd / static_cast<double>(n);
    }
    else
    {
        static_ref_vs_ = 0.0;
        static_ref_vd_ = 0.0;
    }
}

void ObstacleTracker::update(
    const std::vector<Detection> &detections, double stamp,
    double ego_yaw_rate, bool yaw_rate_fresh)
{
    last_stats_ = TrackerUpdateStats{};

    double dt = 0.0;
    if (has_last_stamp_)
    {
        dt = stamp - last_stamp_;
        if (dt < 0.0)
        {
            dt = 0.0;
        }
    }
    last_stamp_ = stamp;
    has_last_stamp_ = true;

    // 1) 모든 기존 트랙을 현재 측정 시각까지 예측한다.
    for (auto &t : tracks_)
    {
        predict(t, dt);
        t.is_visible = false;
    }

    // 2) 예측된 (s, d)에서 greedy 최근접 이웃 방식으로 검출을 1:1 연관한다.
    const std::size_t nt = tracks_.size();
    const std::size_t nd = detections.size();
    std::vector<int> det_assigned(nd, -1);
    std::vector<int> trk_assigned(nt, -1);

    struct Pair
    {
        std::size_t ti;
        std::size_t di;
        double cost;
    };
    std::vector<Pair> pairs;
    pairs.reserve(nt * nd);
    for (std::size_t ti = 0; ti < nt; ++ti)
    {
        const double gate = tracks_[ti].is_static ? p_.assoc_gate : p_.assoc_gate * p_.aggro_multi;
        const double gate_sq = gate * gate;
        for (std::size_t di = 0; di < nd; ++di)
        {
            ++last_stats_.candidate_pairs;
            const double cost_sq =
                frenetDistSquared(tracks_[ti].x(0), tracks_[ti].x(2), detections[di].s,
                                   detections[di].d);
            if (cost_sq > gate_sq)
            {
                ++last_stats_.euclidean_pair_rejected;
                continue;
            }
            if (p_.assoc_use_mahalanobis)
            {
                const double m2 = innovationDistanceSquared(tracks_[ti], detections[di]);
                if (!std::isfinite(m2) || m2 > p_.assoc_mahalanobis_gate)
                {
                    ++last_stats_.mahalanobis_pair_rejected;
                    continue;
                }
            }
            // 후보 정렬은 실제 Frenet 거리 기준을 유지하고 Mahalanobis는 게이트로만 쓴다.
            // 적응형 R이 큰 원거리 저품질 검출이 정규화 m2가 작다는 이유로 가까운 검출보다
            // 먼저 선택되는 것을 막기 위함이다.
            pairs.push_back({ti, di, cost_sq});
        }
    }
    std::sort(pairs.begin(), pairs.end(),
              [](const Pair &a, const Pair &b) { return a.cost < b.cost; });
    for (const auto &pr : pairs)
    {
        if (trk_assigned[pr.ti] >= 0 || det_assigned[pr.di] >= 0)
        {
            continue;
        }
        trk_assigned[pr.ti] = static_cast<int>(pr.di);
        det_assigned[pr.di] = static_cast<int>(pr.ti);
        ++last_stats_.matched;
    }

    // 3) 연결된 트랙을 측정값으로 갱신하고 최신 형상과 관측 이력을 저장한다.
    for (std::size_t ti = 0; ti < nt; ++ti)
    {
        Track &t = tracks_[ti];
        if (trk_assigned[ti] >= 0)
        {
            const Detection &det = detections[trk_assigned[ti]];
            kalmanUpdate(t, det);
            t.hits++;
            t.is_visible = true;
            t.s_half_extent = det.s_half_extent;
            t.d_right_offset = det.d_right_offset;
            t.d_left_offset = det.d_left_offset;
            t.size = det.size;
            t.x_min_map = det.x_min;
            t.x_max_map = det.x_max;
            t.y_min_map = det.y_min;
            t.y_max_map = det.y_max;
            if (p_.classifier_mode != ClassifierMode::Velocity)
            {
                t.hist.emplace_back(t.x(0), t.x(2));
                while (static_cast<int>(t.hist.size()) > p_.std_window)
                {
                    t.hist.pop_front();
                }
            }
        }
        else
        {
            t.ttl--;
        }
    }

    // 4) 어떤 트랙에도 연결되지 않은 검출에서 새 트랙을 만든다.
    for (std::size_t di = 0; di < nd; ++di)
    {
        if (det_assigned[di] >= 0)
        {
            continue;
        }
        Track t;
        t.id = next_id_++;
        t.x << detections[di].s, 0.0, detections[di].d, 0.0;
        t.P = Eigen::Matrix4d::Identity();
        t.P(0, 0) = 0.5;
        t.P(1, 1) = 4.0;
        t.P(2, 2) = 0.5;
        t.P(3, 3) = 4.0;
        t.hits = 1;
        t.ttl = p_.ttl_static;
        t.is_static = true;
        t.is_visible = true;
        t.s_half_extent = detections[di].s_half_extent;
        t.d_right_offset = detections[di].d_right_offset;
        t.d_left_offset = detections[di].d_left_offset;
        t.size = detections[di].size;
        t.x_min_map = detections[di].x_min;
        t.x_max_map = detections[di].x_max;
        t.y_min_map = detections[di].y_min;
        t.y_max_map = detections[di].y_max;
        if (p_.classifier_mode != ClassifierMode::Velocity)
        {
            t.hist.emplace_back(t.x(0), t.x(2));
        }
        tracks_.push_back(std::move(t));
        ++last_stats_.spawned;
    }

    // 4b) 저속 트랙으로 map-flow 기준을 갱신한 뒤 모든 트랙을 그 기준에 상대적으로 분류한다.
    //     계층 2는 map과 함께 흐르는 정적 물체, 계층 3은 명확히 벗어나는 동적 상대 차량이다.
    updateStaticReference();
    for (auto &t : tracks_)
    {
        classify(t, ego_yaw_rate, yaw_rate_fresh);
        if (t.is_visible)  // 이번 프레임에서 연결됐거나 새로 생성된 트랙
        {
            t.ttl = t.is_static ? p_.ttl_static : p_.ttl_dynamic;
        }
    }

    // 5) 연속 미관측으로 TTL이 소진된 트랙을 폐기한다.
    const std::size_t tracks_before_retirement = tracks_.size();
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                 [](const Track &t) { return t.ttl <= 0; }),
                  tracks_.end());
    last_stats_.retired = tracks_before_retirement - tracks_.size();

    // 위의 사건 누적 수와 분리해 갱신 후 트랙 상태를 snapshot으로 기록한다. 1초 진단 구간에서
    // 같은 생존 트랙을 매 스캔마다 더해 실제보다 많은 것처럼 보이는 일을 막는다.
    last_stats_.total_tracks = tracks_.size();
    for (const Track &t : tracks_)
    {
        if (t.is_visible)
        {
            ++last_stats_.visible_tracks;
        }
        if (t.hits < p_.min_hits_confirm)
        {
            ++last_stats_.hit_confirmation_pending;
        }
        else if (!t.classified)
        {
            ++last_stats_.classification_pending;
        }
        else if (t.motion_class == MotionClass::ProvisionalStatic)
        {
            ++last_stats_.provisional_static;
        }
        else if (t.motion_class == MotionClass::ConfirmedStatic)
        {
            ++last_stats_.confirmed_static;
        }
        else if (t.motion_class == MotionClass::Dynamic)
        {
            ++last_stats_.confirmed_dynamic;
        }
        if (t.is_visible && t.relative_speed > p_.dyn_vel_enter &&
            !t.dynamic_motion_reliable)
        {
            ++last_stats_.dynamic_motion_gated;
        }
    }
}

}  // namespace obstacle_detector
