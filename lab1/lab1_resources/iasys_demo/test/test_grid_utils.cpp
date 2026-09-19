#include <cstdint>
#include <vector>

#include "gtest/gtest.h"
#include "iasys_demo/grid_utils.hpp"

TEST(GridUtils, ConvertsWorldAndGridCoordinates)
{
  const auto cell = iasys_demo::worldToGrid(-7.0, -6.0, -10.0, -10.0, 0.2, 100, 100);
  ASSERT_TRUE(cell.has_value());
  EXPECT_EQ(cell->x, 15);
  EXPECT_EQ(cell->y, 20);
  const auto [x, y] = iasys_demo::gridToWorld(cell->x, cell->y, -10.0, -10.0, 0.2);
  EXPECT_NEAR(x, -6.9, 1e-9);
  EXPECT_NEAR(y, -5.9, 1e-9);
}

TEST(GridUtils, FindsPathAroundObstacle)
{
  constexpr int width = 10;
  constexpr int height = 10;
  std::vector<int8_t> grid(width * height, 0);
  for (int y = 0; y < 9; ++y) {
    grid[y * width + 5] = 100;
  }
  const auto path = iasys_demo::astar(
    grid, width, height, iasys_demo::GridCell{1, 1}, iasys_demo::GridCell{8, 1});
  ASSERT_FALSE(path.empty());
  EXPECT_EQ(path.front().x, 1);
  EXPECT_EQ(path.front().y, 1);
  EXPECT_EQ(path.back().x, 8);
  EXPECT_EQ(path.back().y, 1);
}

TEST(GridUtils, InflationExpandsOccupiedCells)
{
  constexpr int width = 7;
  constexpr int height = 7;
  std::vector<int8_t> grid(width * height, 0);
  grid[3 * width + 3] = 100;
  const auto inflated = iasys_demo::inflateGrid(grid, width, height, 1);
  EXPECT_GE(inflated[3 * width + 3], 50);
  EXPECT_GE(inflated[3 * width + 2], 50);
  EXPECT_GE(inflated[3 * width + 4], 50);
  EXPECT_GE(inflated[2 * width + 3], 50);
  EXPECT_GE(inflated[4 * width + 3], 50);
}
