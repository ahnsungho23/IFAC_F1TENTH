// ================================================================================================
// FRENET 트랙 보조기 - 경계 조회, 폐루프 s 연산, RViz용 map 좌표 보간
// ================================================================================================
// 정확한 Cartesian->Frenet 투영은 global_planning::ClcsFrenetConverter가 담당한다.
// 이 클래스에는 검출기가 자체적으로 필요한 트랙 길이, 폐루프를 고려한 s 차이, 최근접 waypoint의
// d_left/d_right 경계, marker 생성용 map 좌표 보간 기능만 둔다.
// ================================================================================================

#ifndef OBSTACLE_DETECTOR__FRENET_PROJECTOR_HPP_
#define OBSTACLE_DETECTOR__FRENET_PROJECTOR_HPP_

#include <vector>

namespace obstacle_detector
{

// s 오름차순으로 정렬된 전역 raceline waypoint에서 만드는 트랙 보조 클래스
class FrenetProjector
{
  public:
    struct Waypoint
    {
        double x{0.0};
        double y{0.0};
        double s{0.0};
        double d_left{0.0};
        double d_right{0.0};
    };

    FrenetProjector() = default;

    // s 오름차순 waypoint로 보조기를 구성한다. 최소 2개 점이 필요하다.
    void build(
        std::vector<Waypoint> waypoints,
        bool closed = true,
        double track_length_override = 0.0);

    bool ready() const { return ready_; }
    double raceline_length() const { return length_; }

    // 주어진 s에 가장 가까운 waypoint의 좌·우 트랙 경계를 반환한다.
    void boundsAtS(double s, double &d_left, double &d_right) const;

    // 폐루프 경계를 고려한 최소 부호 거리 (a - b)를 반환한다.
    double wrapDelta(double a, double b) const;

    // s 위치의 raceline을 보간하고 국소 좌측 법선 방향으로 d만큼 이동한다.
    // 이미 계산된 Frenet 장애물 경계를 map 좌표에서 시각화할 때만 사용한다.
    bool toCartesian(
        double s, double d, double &x, double &y, double &yaw) const;

  private:
    std::vector<Waypoint> wpnts_;
    double length_{0.0};
    bool closed_{true};
    bool ready_{false};
};

}  // namespace obstacle_detector

#endif  // OBSTACLE_DETECTOR__FRENET_PROJECTOR_HPP_
