// ================================================================================================
// OBSTACLE TRACKER - Frenet 등속 Kalman 추적과 정적/동적 분류
// ================================================================================================
// 트랙 상태는 x = [s, vs, d, vd]인 등속 모델이며 측정값은 z = [s, d]이다.
//
// 계층 2/계층 3 분리(자차 기준 "map-flow" 관점):
//   - 벽과 정지 장애물은 움직이는 자차에서 같은 겉보기 속도로 뒤로 흐른다. 이 공통 속도를
//     map-flow 기준이라 한다. 자차 좌표에서는 -v_ego이지만, 여기서는 충분히 느린 트랙들의
//     평균 Frenet 속도 static_ref_*로 직접 추정한다. 이 방식은 자차 위치 추정의 공통 오차를
//     상쇄하고 정확한 v_ego에 의존하지 않는 자기 보정 기준이다. 월드 기준 Frenet 좌표에서는
//     자차 운동이 이미 제거되어 map-flow가 대체로 0 부근에 놓인다.
//   - 계층 2(정적): 흐름이 map-flow 기준과 비슷한 트랙(|flow - ref|가 작음).
//   - 계층 3(동적): 흐름이 map-flow 기준에서 명확히 벗어난 트랙. 같은 방향 상대 차량은 자차
//     좌표에서 벽보다 느리게 흐르며, 월드 Frenet에서는 0이 아닌 종방향 속도 vs를 가진다.
// 진입·이탈 임계값의 히스테리시스와 저속 안전 조건으로 라벨 진동을 막는다. 필요하면
// classifier_mode를 통해 위치 표준편차 기반(ForzaETH 방식) 분류도 함께 사용할 수 있다.
// ================================================================================================

#ifndef OBSTACLE_DETECTOR__OBSTACLE_TRACKER_HPP_
#define OBSTACLE_DETECTOR__OBSTACLE_TRACKER_HPP_

#include <cstddef>
#include <deque>
#include <vector>

#include <Eigen/Dense>

#include "obstacle_detector/frenet_projector.hpp"

namespace obstacle_detector
{

// 지도·경계 필터와 Frenet 투영을 마친 뒤 추적기에 전달하는 측정값
struct Detection
{
    double s{0.0};
    double d{0.0};
    // 현재 Cartesian AABB를 국소 Frenet 좌표로 투영한 형상이다. 종방향 범위는 s를 중심으로
    // 대칭이고, 곡선 raceline의 최근접 면을 정확히 보정한 뒤 횡방향 오프셋은 비대칭일 수 있다.
    double s_half_extent{0.0};
    double d_right_offset{0.0};
    double d_left_offset{0.0};
    double size{0.0};
    double x_min{0.0};
    double x_max{0.0};
    double y_min{0.0};
    double y_max{0.0};
    // 거리, 군집 밀도, 최신 자차 yaw rate로 계산한 측정 공분산 배율이다. 기본 분산은 YAML의
    // meas_var_s/meas_var_d를 그대로 사용한다.
    double variance_scale{1.0};
};

enum class ClassifierMode
{
    Velocity,  // map-flow 대비 속도만 사용
    Std,       // 위치 이력의 표준편차만 사용
    Both       // 두 판정 중 하나라도 동적이면 동적 투표
};

enum class MotionClass
{
    Pending,             // 최소 관측 횟수 미달로 아직 발행하지 않음
    ProvisionalStatic,   // 발행은 시작했지만 정적 확정 대기 중
    ConfirmedStatic,     // 연속 정적 관측으로 확정
    Dynamic              // 연속 신뢰 가능한 움직임으로 동적 확정
};

struct TrackerParams
{
    // Kalman 측정·프로세스 잡음
    double meas_var_s{0.002};
    double meas_var_d{0.002};
    double process_var_vs{2.0};
    double process_var_vd{8.0};
    // 측정-트랙 데이터 연관
    double assoc_gate{0.5};       // [m] 정적/미분류 트랙의 Frenet 거리 게이트
    double aggro_multi{2.0};      // 동적 트랙에 적용할 게이트 확대 배율
    bool assoc_use_mahalanobis{true};
    double assoc_mahalanobis_gate{9.21};  // 자유도 2, 99% 카이제곱 게이트
    // 트랙 수명과 최초 발행 조건
    int ttl_dynamic{40};
    int ttl_static{25};
    int min_hits_confirm{3};      // 트랙을 처음 발행하기 전에 필요한 측정 횟수
    // 정적/동적 분류 조건
    ClassifierMode classifier_mode{ClassifierMode::Velocity};
    double dyn_vel_enter{0.5};    // [m/s] 동적 진입에 필요한 map-flow 대비 속도
    double dyn_vel_exit{0.25};    // [m/s] 정적으로 복귀하는 속도 임계값(히스테리시스)
    int static_confirm_frames{3};   // 임시 정적 발행 후 확정에 필요한 추가 저속 관측 수
    int dynamic_confirm_frames{25}; // 동적 승격에 필요한 연속 신뢰 움직임 관측 수
    double dyn_velocity_mahalanobis_gate{9.21};  // 자유도 2 속도 신뢰도 임계값
    double dyn_max_abs_yaw_rate{1.5};  // [rad/s] 이 값을 넘는 자차 회전 중에는 동적 증거 동결
    // 벽과 정지 장애물이 자차에 대해 공유하는 겉보기 속도가 map-flow 기준이다. 자차 좌표에서는
    // -v_ego이고 월드 Frenet에서는 위치 추정 drift를 제외하면 약 0이다. 각 트랙의 흐름을 이
    // 기준에 상대적으로 비교해 정적 계층과 동적 계층을 나눈다. static_ref_gate보다 느린
    // 트랙들만 평균에 포함하므로 빠른 상대 차량이 기준값을 오염시키지 않는다.
    double static_ref_gate{0.3};  // [m/s] map-flow 기준 계산에 포함할 저속 트랙 한계
    int std_window{30};           // 표준편차 분류에 사용할 위치 이력 길이
    int min_nb_meas{5};           // 표준편차 분류가 투표하기 위한 최소 측정 수
    double min_std{0.16};         // [m] 두 축 모두 이 값 이하면 정적 투표
    double max_std{0.20};         // [m] 어느 축이든 이 값 이상이면 동적 투표
    double dt_max{0.5};           // [s] 예측 시간 간격 상한
};

struct Track
{
    int id{0};
    Eigen::Vector4d x{Eigen::Vector4d::Zero()};   // 상태 벡터 [s, vs, d, vd]
    Eigen::Matrix4d P{Eigen::Matrix4d::Identity()};
    int hits{0};
    int ttl{0};
    bool is_static{true};
    bool is_visible{false};
    // classified는 min_hits_confirm을 통과해 발행 가능한지를 뜻한다. 물리 장애물을
    // local_planning이 곧바로 사용할 수 있게 최초 상태는 ProvisionalStatic으로 둔다.
    // motion_class는 이 발행 조건과 별개인 정적/동적 신뢰 상태를 저장한다.
    bool classified{false};
    MotionClass motion_class{MotionClass::Pending};
    int dyn_streak{0};
    int static_streak{0};
    double relative_speed{0.0};
    double velocity_mahalanobis_sq{0.0};
    bool dynamic_motion_reliable{false};
    // 마지막으로 측정한 Frenet 형상을 Kalman 중심에 대한 상대 오프셋으로 보존한다. 따라서
    // 예측만 수행한 출력도 마지막 유효 형상을 이동시킬 수 있지만, 현재 Cartesian 스캔 형상을
    // 관측했다고 잘못 표시하지 않는다.
    double s_half_extent{0.0};
    double d_right_offset{0.0};
    double d_left_offset{0.0};
    double size{0.0};
    // 마지막 map 좌표 Cartesian AABB이다. 다음 측정 갱신을 위해 보관하지만, 예측은 Frenet
    // 상태만 갱신하므로 소비자는 is_visible=true일 때만 이 원시 스캔 형상을 사용해야 한다.
    double x_min_map{0.0};
    double x_max_map{0.0};
    double y_min_map{0.0};
    double y_max_map{0.0};
    std::deque<std::pair<double, double>> hist;  // 표준편차 분류용 (s, d) 이력

    double s() const { return x(0); }
    double vs() const { return x(1); }
    double d() const { return x(2); }
    double vd() const { return x(3); }
};

// 추적기 갱신 한 회의 진단값이다. 연관 거부 항목은 트랙-검출 후보 쌍 수를 누적하고, 트랙과
// 분류 항목은 현재 스캔에서 수명이 끝난 트랙을 제거한 뒤의 상태를 나타낸다.
struct TrackerUpdateStats
{
    std::size_t candidate_pairs{0};
    std::size_t matched{0};
    std::size_t spawned{0};
    std::size_t retired{0};
    std::size_t euclidean_pair_rejected{0};
    std::size_t mahalanobis_pair_rejected{0};

    std::size_t total_tracks{0};
    std::size_t visible_tracks{0};
    std::size_t hit_confirmation_pending{0};
    std::size_t classification_pending{0};
    std::size_t provisional_static{0};
    std::size_t confirmed_static{0};
    std::size_t confirmed_dynamic{0};
    std::size_t dynamic_motion_gated{0};
};

class ObstacleTracker
{
  public:
    ObstacleTracker() = default;

    void configure(const TrackerParams &params, const FrenetProjector *frenet);

    // 모든 트랙을 stamp까지 예측하고 검출 연관, Kalman 갱신, 생성·폐기, 분류를 순서대로 수행한다.
    void update(const std::vector<Detection> &detections, double stamp,
                double ego_yaw_rate = 0.0, bool yaw_rate_fresh = true);

    const std::vector<Track> &tracks() const { return tracks_; }
    const TrackerUpdateStats &lastStats() const { return last_stats_; }

    // 벽과 정지 장애물이 공유하는 현재 Frenet map-flow 기준 속도이다. 동적 상대 차량을
    // 판별할 때 사용하는 계층 2 기준값이다.
    double staticRefVs() const { return static_ref_vs_; }
    double staticRefVd() const { return static_ref_vd_; }

  private:
    void predict(Track &t, double dt) const;
    Eigen::Matrix2d measurementCovariance(const Detection &detection) const;
    double innovationDistanceSquared(const Track &t, const Detection &detection) const;
    double velocityMahalanobisSquared(const Track &t, double rel_vs, double rel_vd) const;
    void kalmanUpdate(Track &t, const Detection &detection) const;
    void updateStaticReference();
    void classify(Track &t, double ego_yaw_rate, bool yaw_rate_fresh) const;
    double frenetDistSquared(double s1, double d1, double s2, double d2) const;

    TrackerParams p_;
    const FrenetProjector *frenet_{nullptr};
    std::vector<Track> tracks_;
    int next_id_{0};
    double last_stamp_{-1.0};
    bool has_last_stamp_{false};
    double static_ref_vs_{0.0};   // Frenet s축 map-flow 기준 속도
    double static_ref_vd_{0.0};   // Frenet d축 map-flow 기준 속도
    TrackerUpdateStats last_stats_;
};

}  // namespace obstacle_detector

#endif  // OBSTACLE_DETECTOR__OBSTACLE_TRACKER_HPP_
