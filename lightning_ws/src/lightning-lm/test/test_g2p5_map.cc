// G2P5Map 扩容单测（2026-10-08）：旧实现扩容时新原点只对齐到 0.05 m，旧数据按整块子网格（0.8 m）搬移，
// 已画的内容整体错位 |Δ/0.8 - round(Δ/0.8)| * 0.8（最多 0.4 m）。多次扩容的错位叠加，
// 就是地图 1790676615248 的 2D 栅格多层墙、相对 3D 像转了 1.25° 的原因。
// 下面的原点变化量取自 mapping_corridor_0929 包里 /map 的实际扩容（-1.80 错 0.20、-0.45 错 0.35、-7.50 错 0.30 等）。
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "core/g2p5/g2p5_map.h"

using lightning::g2p5::G2P5Map;

namespace {

constexpr float kRes = 0.05f;
constexpr float kSubGrid = kRes * (1 << G2P5Map::SUB_GRID_SIZE);  // 0.8 m

struct Pt {
    float x, y;
};

/// 在 (x, y) 处打够 ToROS 判占据所需的命中次数（visit > 3，命中要求高度递减）
void MarkHit(G2P5Map& map, const Pt& p) {
    for (int i = 0; i < 5; ++i) {
        map.SetHitPoint(p.x, p.y, true, 1.0f - 0.1f * i);
    }
}

std::vector<Pt> OccupiedCenters(G2P5Map& map) {
    const auto msg = map.ToROS();
    std::vector<Pt> out;
    for (unsigned int y = 0; y < msg.info.height; ++y) {
        for (unsigned int x = 0; x < msg.info.width; ++x) {
            if (msg.data[y * msg.info.width + x] == 100) {
                out.push_back({static_cast<float>(msg.info.origin.position.x + (x + 0.5) * kRes),
                               static_cast<float>(msg.info.origin.position.y + (y + 0.5) * kRes)});
            }
        }
    }
    return out;
}

/// 每个打过的点都必须仍落在自己的格子里（格心距离 ≤ 半格）
void ExpectAllInPlace(G2P5Map& map, const std::vector<Pt>& hits, const char* step) {
    const auto occ = OccupiedCenters(map);
    ASSERT_EQ(occ.size(), hits.size()) << step;
    for (const auto& h : hits) {
        bool found = false;
        for (const auto& o : occ) {
            if (std::fabs(o.x - h.x) <= kRes / 2 + 1e-3 && std::fabs(o.y - h.y) <= kRes / 2 + 1e-3) {
                found = true;
            }
        }
        EXPECT_TRUE(found) << step << ": hit (" << h.x << ", " << h.y << ") moved";
    }
}

}  // namespace

TEST(G2P5Map, ResizeKeepsOldContentInPlace) {
    G2P5Map::Options opt;
    opt.resolution_ = kRes;
    G2P5Map map(opt);

    // 与 G2P5::ResizeMap 相同：边界只对齐到 0.05 m，地图从 (0,0,0,0) 开始只增不减
    float min_x = -5.30f, min_y = -4.10f, max_x = 6.20f, max_y = 3.90f;
    map.Resize(min_x, min_y, max_x, max_y);

    std::vector<Pt> hits{{0.52f, 0.33f}, {-4.98f, 3.61f}};
    for (const auto& h : hits) {
        MarkHit(map, h);
    }
    ExpectAllInPlace(map, hits, "initial");

    const std::vector<Pt> grow{{-1.80f, 0.f}, {0.f, -0.45f}, {-3.25f, -0.40f}, {0.f, -7.50f},
                               {-1.40f, 0.f}, {0.f, -3.00f},  {-0.15f, 0.f},   {0.f, -6.40f}};
    int step = 0;
    for (const auto& g : grow) {
        min_x += g.x;
        min_y += g.y;
        max_x += 0.35f;  // 另一侧同时扩一点，覆盖两侧都变的情况
        max_y += 0.10f;
        map.Resize(min_x, min_y, max_x, max_y);
        ExpectAllInPlace(map, hits, ("after resize " + std::to_string(step)).c_str());

        // 扩容后新画的点也要在正确位置，并与旧内容对齐
        Pt fresh{min_x + 0.37f, min_y + 0.61f};
        MarkHit(map, fresh);
        hits.push_back(fresh);
        ExpectAllInPlace(map, hits, ("after draw " + std::to_string(step)).c_str());
        ++step;
    }

    // 原点保持在子网格格点上，且不小于调用方要求的范围
    float mx, my, Mx, My;
    map.GetMinAndMax(mx, my, Mx, My);
    EXPECT_NEAR(mx / kSubGrid, std::round(mx / kSubGrid), 1e-3);
    EXPECT_NEAR(my / kSubGrid, std::round(my / kSubGrid), 1e-3);
    EXPECT_LE(mx, min_x);
    EXPECT_LE(my, min_y);
}
