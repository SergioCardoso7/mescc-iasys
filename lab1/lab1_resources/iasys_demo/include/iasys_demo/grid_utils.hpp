#ifndef IASYS_DEMO__GRID_UTILS_HPP_
#define IASYS_DEMO__GRID_UTILS_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace iasys_demo
{

struct GridCell
{
  int x{};
  int y{};

  bool operator==(const GridCell & other) const noexcept
  {
    return x == other.x && y == other.y;
  }
};

struct GridCellHash
{
  std::size_t operator()(const GridCell & cell) const noexcept
  {
    const auto hx = std::hash<int>{}(cell.x);
    const auto hy = std::hash<int>{}(cell.y);
    return hx ^ (hy + 0x9e3779b9U + (hx << 6U) + (hx >> 2U));
  }
};

inline std::optional<GridCell> worldToGrid(
  double x, double y, double origin_x, double origin_y,
  double resolution, int width, int height)
{
  const int gx = static_cast<int>(std::floor((x - origin_x) / resolution));
  const int gy = static_cast<int>(std::floor((y - origin_y) / resolution));
  if (gx < 0 || gy < 0 || gx >= width || gy >= height) {
    return std::nullopt;
  }
  return GridCell{gx, gy};
}

inline std::pair<double, double> gridToWorld(
  int gx, int gy, double origin_x, double origin_y, double resolution)
{
  return {
    origin_x + (static_cast<double>(gx) + 0.5) * resolution,
    origin_y + (static_cast<double>(gy) + 0.5) * resolution};
}

inline std::vector<int8_t> inflateGrid(
  const std::vector<int8_t> & data, int width, int height, int radius_cells)
{
  if (radius_cells <= 0) {
    return data;
  }

  auto inflated = data;
  std::vector<GridCell> occupied;
  for (int gy = 0; gy < height; ++gy) {
    for (int gx = 0; gx < width; ++gx) {
      if (data[static_cast<std::size_t>(gy * width + gx)] >= 50) {
        occupied.push_back({gx, gy});
      }
    }
  }

  for (const auto & obstacle : occupied) {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        if (dx * dx + dy * dy > radius_cells * radius_cells) {
          continue;
        }
        const int nx = obstacle.x + dx;
        const int ny = obstacle.y + dy;
        if (nx >= 0 && ny >= 0 && nx < width && ny < height) {
          inflated[static_cast<std::size_t>(ny * width + nx)] = 100;
        }
      }
    }
  }
  return inflated;
}

inline std::vector<GridCell> astar(
  const std::vector<int8_t> & data, int width, int height,
  const std::optional<GridCell> & start_opt,
  const std::optional<GridCell> & goal_opt)
{
  if (!start_opt || !goal_opt) {
    return {};
  }
  const GridCell start = *start_opt;
  const GridCell goal = *goal_opt;

  const auto occupied = [&](const GridCell & c) {
      return data[static_cast<std::size_t>(c.y * width + c.x)] >= 50;
    };
  if (occupied(start) || occupied(goal)) {
    return {};
  }

  struct QueueItem
  {
    double f{};
    double g{};
    GridCell cell{};
  };
  struct Greater
  {
    bool operator()(const QueueItem & a, const QueueItem & b) const noexcept
    {
      return a.f > b.f;
    }
  };

  const auto heuristic = [](const GridCell & a, const GridCell & b) {
      return std::hypot(static_cast<double>(a.x - b.x), static_cast<double>(a.y - b.y));
    };

  struct Step
  {
    int dx;
    int dy;
    double cost;
  };
  const double diagonal = std::sqrt(2.0);
  const std::vector<Step> steps{
    {-1, 0, 1.0}, {1, 0, 1.0}, {0, -1, 1.0}, {0, 1, 1.0},
    {-1, -1, diagonal}, {-1, 1, diagonal}, {1, -1, diagonal}, {1, 1, diagonal}};

  std::priority_queue<QueueItem, std::vector<QueueItem>, Greater> open;
  std::unordered_map<GridCell, GridCell, GridCellHash> came_from;
  std::unordered_map<GridCell, double, GridCellHash> best_g;
  std::unordered_set<GridCell, GridCellHash> closed;

  best_g[start] = 0.0;
  open.push({heuristic(start, goal), 0.0, start});

  while (!open.empty()) {
    const auto current_item = open.top();
    open.pop();
    const GridCell current = current_item.cell;

    if (closed.count(current) != 0U) {
      continue;
    }
    if (current == goal) {
      std::vector<GridCell> path{current};
      GridCell cursor = current;
      while (!(cursor == start)) {
        const auto it = came_from.find(cursor);
        if (it == came_from.end()) {
          return {};
        }
        cursor = it->second;
        path.push_back(cursor);
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    closed.insert(current);
    for (const auto & step : steps) {
      const GridCell next{current.x + step.dx, current.y + step.dy};
      if (next.x < 0 || next.y < 0 || next.x >= width || next.y >= height) {
        continue;
      }
      if (occupied(next)) {
        continue;
      }

      // Prevent diagonal movement through the corner of occupied cells.
      if (step.dx != 0 && step.dy != 0) {
        const GridCell horizontal{current.x + step.dx, current.y};
        const GridCell vertical{current.x, current.y + step.dy};
        if (occupied(horizontal) || occupied(vertical)) {
          continue;
        }
      }

      const double candidate_g = current_item.g + step.cost;
      const auto best_it = best_g.find(next);
      if (best_it == best_g.end() || candidate_g < best_it->second) {
        best_g[next] = candidate_g;
        came_from[next] = current;
        open.push({candidate_g + heuristic(next, goal), candidate_g, next});
      }
    }
  }
  return {};
}

inline std::vector<GridCell> simplifyPath(const std::vector<GridCell> & cells)
{
  if (cells.size() <= 2U) {
    return cells;
  }

  std::vector<GridCell> simplified;
  simplified.push_back(cells.front());
  int prev_dx = cells[1].x - cells[0].x;
  int prev_dy = cells[1].y - cells[0].y;

  for (std::size_t i = 2; i < cells.size(); ++i) {
    const int dx = cells[i].x - cells[i - 1].x;
    const int dy = cells[i].y - cells[i - 1].y;
    if (dx != prev_dx || dy != prev_dy) {
      simplified.push_back(cells[i - 1]);
    }
    prev_dx = dx;
    prev_dy = dy;
  }
  simplified.push_back(cells.back());
  return simplified;
}

}  // namespace iasys_demo

#endif  // IASYS_DEMO__GRID_UTILS_HPP_
