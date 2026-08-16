// ================================================================================================
// WALL DISTANCE FILTER implementation
// ================================================================================================

#include "obstacle_detector/wall_distance_filter.hpp"

#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace obstacle_detector
{

void WallDistanceFilter::clear()
{
    dist_to_wall_.clear();
    width_ = 0;
    height_ = 0;
    resolution_ = 0.0;
    active_ = false;
}

void WallDistanceFilter::buildFromMap(
    const nav_msgs::msg::OccupancyGrid &map, const Params &params)
{
    clear();

    const auto &info = map.info;
    const int w = static_cast<int>(info.width);
    const int h = static_cast<int>(info.height);
    if (w <= 0 || h <= 0 || info.resolution <= 0.0 ||
        map.data.size() < static_cast<std::size_t>(w) * static_cast<std::size_t>(h))
    {
        return;
    }
    width_ = w;
    height_ = h;
    resolution_ = info.resolution;
    origin_x_ = info.origin.position.x;
    origin_y_ = info.origin.position.y;
    const int n = w * h;

    // ---- occupied cells (unknown cells read as -1 and fail the threshold automatically) ----
    std::vector<bool> occupied(static_cast<std::size_t>(n), false);
    for (int i = 0; i < n; ++i)
    {
        occupied[static_cast<std::size_t>(i)] = map.data[static_cast<std::size_t>(i)] >=
            params.occupied_thresh;
    }

    // ---- connected components: 8-connectivity flood fill (grid equivalent of DBSCAN) ----
    std::vector<int> label(static_cast<std::size_t>(n), -1);
    std::vector<char> wall_cell(static_cast<std::size_t>(n), 0);
    std::vector<int> stack;
    std::vector<int> component;
    int next_label = 0;
    for (int seed = 0; seed < n; ++seed)
    {
        if (!occupied[static_cast<std::size_t>(seed)] ||
            label[static_cast<std::size_t>(seed)] >= 0)
        {
            continue;
        }
        component.clear();
        stack.assign(1, seed);
        label[static_cast<std::size_t>(seed)] = next_label;
        while (!stack.empty())
        {
            const int cur = stack.back();
            stack.pop_back();
            component.push_back(cur);
            const int cx = cur % w;
            const int cy = cur / w;
            for (int dy = -1; dy <= 1; ++dy)
            {
                for (int dx = -1; dx <= 1; ++dx)
                {
                    if (dx == 0 && dy == 0)
                    {
                        continue;
                    }
                    const int nx = cx + dx;
                    const int ny = cy + dy;
                    if (nx < 0 || ny < 0 || nx >= w || ny >= h)
                    {
                        continue;
                    }
                    const int ni = ny * w + nx;
                    if (occupied[static_cast<std::size_t>(ni)] &&
                        label[static_cast<std::size_t>(ni)] < 0)
                    {
                        label[static_cast<std::size_t>(ni)] = next_label;
                        stack.push_back(ni);
                    }
                }
            }
        }

        // ---- linearity test: 2x2 PCA on the component's cell-centre coordinates [m] ----
        double mean_x = 0.0;
        double mean_y = 0.0;
        for (const int idx : component)
        {
            mean_x += origin_x_ + (static_cast<double>(idx % w) + 0.5) * resolution_;
            mean_y += origin_y_ + (static_cast<double>(idx / w) + 0.5) * resolution_;
        }
        mean_x /= static_cast<double>(component.size());
        mean_y /= static_cast<double>(component.size());
        double sxx = 0.0;
        double sxy = 0.0;
        double syy = 0.0;
        for (const int idx : component)
        {
            const double px = origin_x_ + (static_cast<double>(idx % w) + 0.5) * resolution_ - mean_x;
            const double py = origin_y_ + (static_cast<double>(idx / w) + 0.5) * resolution_ - mean_y;
            sxx += px * px;
            sxy += px * py;
            syy += py * py;
        }
        // closed-form eigenvalues of the 2x2 scatter matrix
        const double trace = sxx + syy;
        const double det = sxx * syy - sxy * sxy;
        const double disc = std::sqrt(std::max(0.0, 0.25 * trace * trace - det));
        const double lambda_max = 0.5 * trace + disc;
        // extent along the major axis: project every cell onto the dominant eigenvector
        double axis_x = 1.0;
        double axis_y = 0.0;
        if (std::abs(sxy) > 1e-12)
        {
            axis_x = lambda_max - syy;
            axis_y = sxy;
        }
        else if (syy > sxx)
        {
            axis_x = 0.0;
            axis_y = 1.0;
        }
        const double axis_norm = std::hypot(axis_x, axis_y);
        double extent = 0.0;
        if (axis_norm > 1e-12)
        {
            axis_x /= axis_norm;
            axis_y /= axis_norm;
            double proj_min = std::numeric_limits<double>::max();
            double proj_max = std::numeric_limits<double>::lowest();
            for (const int idx : component)
            {
                const double px =
                    origin_x_ + (static_cast<double>(idx % w) + 0.5) * resolution_ - mean_x;
                const double py =
                    origin_y_ + (static_cast<double>(idx / w) + 0.5) * resolution_ - mean_y;
                const double proj = px * axis_x + py * axis_y;
                proj_min = std::min(proj_min, proj);
                proj_max = std::max(proj_max, proj);
            }
            extent = proj_max - proj_min + resolution_;  // include the end cells' own size
        }

        if (extent >= params.min_length_m)
        {
            for (const int idx : component)
            {
                wall_cell[static_cast<std::size_t>(idx)] = 1;
            }
        }
        ++next_label;
    }

    // ---- multi-source distance transform: 8-neighbour Dijkstra from all wall cells ----
    const float inf = std::numeric_limits<float>::infinity();
    dist_to_wall_.assign(static_cast<std::size_t>(n), inf);
    using Entry = std::pair<float, int>;  // (distance, cell)
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
    for (int i = 0; i < n; ++i)
    {
        if (wall_cell[static_cast<std::size_t>(i)])
        {
            dist_to_wall_[static_cast<std::size_t>(i)] = 0.0F;
            queue.emplace(0.0F, i);
        }
    }
    const float step_ortho = static_cast<float>(resolution_);
    const float step_diag = static_cast<float>(resolution_ * M_SQRT2);
    while (!queue.empty())
    {
        const auto [dist, cur] = queue.top();
        queue.pop();
        if (dist > dist_to_wall_[static_cast<std::size_t>(cur)])
        {
            continue;  // stale queue entry
        }
        const int cx = cur % w;
        const int cy = cur / w;
        for (int dy = -1; dy <= 1; ++dy)
        {
            for (int dx = -1; dx <= 1; ++dx)
            {
                if (dx == 0 && dy == 0)
                {
                    continue;
                }
                const int nx = cx + dx;
                const int ny = cy + dy;
                if (nx < 0 || ny < 0 || nx >= w || ny >= h)
                {
                    continue;
                }
                const int ni = ny * w + nx;
                const float step = (dx == 0 || dy == 0) ? step_ortho : step_diag;
                const float next = dist + step;
                if (next < dist_to_wall_[static_cast<std::size_t>(ni)])
                {
                    dist_to_wall_[static_cast<std::size_t>(ni)] = next;
                    queue.emplace(next, ni);
                }
            }
        }
    }

    active_ = true;
}

double WallDistanceFilter::distanceToWall(double x_map, double y_map) const
{
    if (!active_)
    {
        return -1.0;
    }
    const int gx = static_cast<int>(std::floor((x_map - origin_x_) / resolution_));
    const int gy = static_cast<int>(std::floor((y_map - origin_y_) / resolution_));
    if (gx < 0 || gy < 0 || gx >= width_ || gy >= height_)
    {
        return -1.0;
    }
    const float d = dist_to_wall_[static_cast<std::size_t>(gy) * width_ + gx];
    return std::isinf(d) ? -1.0 : static_cast<double>(d);
}

bool WallDistanceFilter::isWallPoint(double x_map, double y_map, double assoc_dist) const
{
    const double d = distanceToWall(x_map, y_map);
    return d >= 0.0 && d < assoc_dist;
}

}  // namespace obstacle_detector
