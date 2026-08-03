// ================================================================================================
// Frenet 트랙 보조기 구현
// ================================================================================================

#include "obstacle_detector/frenet_projector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace obstacle_detector
{

void FrenetProjector::build(
    std::vector<Waypoint> waypoints,
    bool closed,
    double track_length_override)
{
    wpnts_ = std::move(waypoints);
    closed_ = closed;
    ready_ = false;
    length_ = 0.0;

    if (wpnts_.size() < 2)
    {
        return;
    }

    // CLCS 길이가 주어지면 추적, 발행 경계, marker wrap이 같은 s 영역을 공유하도록 사용한다.
    // 단독 사용 시에는 마지막 s에 시작-끝 폐곡선 선분의 기하 길이를 더해 전체 길이를 구한다.
    const Waypoint &front = wpnts_.front();
    const Waypoint &back = wpnts_.back();
    const double closing = std::hypot(front.x - back.x, front.y - back.y);
    length_ = track_length_override > 0.0 ?
        track_length_override : back.s + (closed_ ? closing : 0.0);
    if (length_ <= 0.0)
    {
        length_ = back.s > 0.0 ? back.s : 1.0;
    }
    ready_ = true;
}

double FrenetProjector::wrapDelta(double a, double b) const
{
    // 두 방향 후보 중 절댓값이 반 바퀴 이하인 부호 거리를 남긴다.
    double diff = a - b;
    if (!closed_ || length_ <= 0.0)
    {
        return diff;
    }
    while (diff > 0.5 * length_)
    {
        diff -= length_;
    }
    while (diff < -0.5 * length_)
    {
        diff += length_;
    }
    return diff;
}

bool FrenetProjector::toCartesian(
    double s, double d, double &x, double &y, double &yaw) const
{
    if (!ready_ || wpnts_.size() < 2 || !std::isfinite(s) || !std::isfinite(d))
    {
        return false;
    }

    // 폐루프는 s를 전체 길이로 wrap하고, 열린 경로는 양 끝 범위로 clamp한다.
    double query_s = s;
    if (closed_ && length_ > 0.0)
    {
        query_s = std::fmod(query_s, length_);
        if (query_s < 0.0)
        {
            query_s += length_;
        }
    }
    else
    {
        query_s = std::clamp(query_s, wpnts_.front().s, wpnts_.back().s);
    }

    std::size_t first = 0;
    std::size_t second = 1;
    double first_s = wpnts_.front().s;
    double segment_s = wpnts_[1].s - wpnts_[0].s;
    double progress_s = query_s - first_s;

    // 마지막 waypoint 이후라면 끝점과 첫점을 잇는 폐곡선 선분에서 보간한다.
    if (closed_ && query_s >= wpnts_.back().s)
    {
        first = wpnts_.size() - 1;
        second = 0;
        first_s = wpnts_[first].s;
        segment_s = length_ - first_s;
        progress_s = query_s - first_s;
    }
    else
    {
        for (std::size_t i = 0; i + 1 < wpnts_.size(); ++i)
        {
            if (query_s <= wpnts_[i + 1].s)
            {
                first = i;
                second = i + 1;
                first_s = wpnts_[first].s;
                segment_s = wpnts_[second].s - first_s;
                progress_s = query_s - first_s;
                break;
            }
        }
    }

    const auto &a = wpnts_[first];
    const auto &b = wpnts_[second];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double geometric_length = std::hypot(dx, dy);
    if (!(geometric_length > std::numeric_limits<double>::epsilon()))
    {
        return false;
    }

    // waypoint의 s 간격이 퇴화했을 때는 실제 선분 길이를 보간 분모로 사용한다.
    const double denominator =
        segment_s > std::numeric_limits<double>::epsilon() ? segment_s : geometric_length;
    const double ratio = std::clamp(progress_s / denominator, 0.0, 1.0);
    const double base_x = a.x + ratio * dx;
    const double base_y = a.y + ratio * dy;
    // 보간한 raceline 점을 좌측 법선(-sin(yaw), cos(yaw)) 방향으로 d만큼 이동한다.
    yaw = std::atan2(dy, dx);
    x = base_x - d * std::sin(yaw);
    y = base_y + d * std::cos(yaw);
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(yaw);
}

void FrenetProjector::boundsAtS(double s, double &d_left, double &d_right) const
{
    d_left = 0.0;
    d_right = 0.0;
    if (!ready_ || wpnts_.empty())
    {
        return;
    }

    // s가 단조 증가하더라도 누락 구간에 안전하도록 전체를 선형 탐색해 최근접 waypoint를 찾는다.
    std::size_t best = 0;
    double best_ds = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < wpnts_.size(); ++i)
    {
        const double ds = std::abs(wrapDelta(wpnts_[i].s, s));
        if (ds < best_ds)
        {
            best_ds = ds;
            best = i;
        }
    }
    d_left = wpnts_[best].d_left;
    d_right = wpnts_[best].d_right;
}

}  // namespace obstacle_detector
