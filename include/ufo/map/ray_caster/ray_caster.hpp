/*!
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author Daniel Duberg (dduberg@kth.se)
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * @version 1.0
 * @date 2022-05-13
 *
 * @copyright Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef UFO_MAP_RAY_CASTER_HPP
#define UFO_MAP_RAY_CASTER_HPP

// UFO
#include <ufo/map/code.hpp>
#include <ufo/map/integration/grid.hpp>
#include <ufo/map/key.hpp>
#include <ufo/map/point.hpp>
#include <ufo/map/types.hpp>
#include <ufo/math/util.hpp>
#include <ufo/math/vector3.hpp>

// STL
#include <immintrin.h>

#include <cassert>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace ufo
{
void computeRay(CodeUnorderedMap<Grid>& grids, Point origin, Point goal, Key k_origin,
                Vector3f voxel_border, Vector3f dir, float grid_size,
                std::size_t inflate_unknown)
{
	constexpr auto max   = std::numeric_limits<float>::max();
	int const      size  = static_cast<int>(k_origin.step());
	depth_t const  depth = k_origin.depth();

	auto const distance = (goal - origin).norm();

	std::array<key_t, 3> step{static_cast<key_t>(sgn(dir.x) * size),
	                          static_cast<key_t>(sgn(dir.y) * size),
	                          static_cast<key_t>(sgn(dir.z) * size)};

	if (0 == step[0] && 0 == step[1] && 0 == step[2]) {
		// TODO: What to do?
	}

	Vector3f t_max(
	    step[0] ? (voxel_border.x + sgn(dir.x) * grid_size / 2.0f) / dir.x : max,
	    step[1] ? (voxel_border.y + sgn(dir.y) * grid_size / 2.0f) / dir.y : max,
	    step[2] ? (voxel_border.z + sgn(dir.z) * grid_size / 2.0f) / dir.z : max);

	Vector3f t_delta(step[0] ? grid_size / std::abs(dir.x) : max,
	                 step[1] ? grid_size / std::abs(dir.y) : max,
	                 step[2] ? grid_size / std::abs(dir.z) : max);

	// FIXME: Is this correct? Should it be zero if all zero?
	std::size_t steps =
	    (step[0] ? std::max(0.0f, std::ceil((distance - t_max[0]) / t_delta[0])) : 0) +
	    (step[1] ? std::max(0.0f, std::ceil((distance - t_max[1]) / t_delta[1])) : 0) +
	    (step[2] ? std::max(0.0f, std::ceil((distance - t_max[2]) / t_delta[2])) : 0);

	Code  prev_at_depth = Code(k_origin).toDepth(Grid::depth() + depth);
	Grid* grid          = &grids[prev_at_depth];
	grid->set(k_origin);

	while (steps--) {
		auto const advance_dim = t_max.minElementIndex();
		k_origin[advance_dim] += step[advance_dim];
		t_max[advance_dim] += t_delta[advance_dim];
		Code const cur = k_origin;

		if (!Code::equalAtDepth(prev_at_depth, cur, Grid::depth() + depth)) {
			grid          = &grids[cur.toDepth(Grid::depth() + depth)];
			prev_at_depth = cur.toDepth(Grid::depth() + depth);
		}
		grid->set(cur);
	}

	// auto last = k_origin;

	// std::size_t advance_dim{};
	// while (std::max(last[advance_dim], k_origin[advance_dim]) -
	//            std::min(last[advance_dim], k_origin[advance_dim]) !=
	//        inflate_unknown) {
	// 	advance_dim = t_max.minElementIndex();
	// 	k_origin[advance_dim] += step[advance_dim];
	// 	t_max[advance_dim] += t_delta[advance_dim];
	// 	Code const cur = k_origin;

	// 	if (!Code::equalAtDepth(prev_at_depth, cur, Grid::depth() + depth)) {
	// 		grid          = &grids[cur.toDepth(Grid::depth() + depth)];
	// 		prev_at_depth = cur.toDepth(Grid::depth() + depth);
	// 	}
	// 	grid->set(cur);
	// }
}

void computeRay(CodeUnorderedMap<Grid>& grids, CodeUnorderedMap<Grid> const& hits,
                Point origin, Point goal, Key k_origin, Vector3f voxel_border,
                float grid_size, float max_distance, std::size_t inflate_unknown,
                bool ray_passthrough_hits)
{
	constexpr auto max   = std::numeric_limits<float>::max();
	int const      size  = static_cast<int>(k_origin.step());
	depth_t const  depth = k_origin.depth();

	auto dir      = goal - origin;
	auto distance = dir.norm();
	dir /= distance;

	distance = std::min(distance, max_distance);

	std::array<key_t, 3> step{static_cast<key_t>(sgn(dir.x) * size),
	                          static_cast<key_t>(sgn(dir.y) * size),
	                          static_cast<key_t>(sgn(dir.z) * size)};

	if (0 == step[0] && 0 == step[1] && 0 == step[2]) {
		// TODO: What to do?
	}

	Vector3f t_max(
	    step[0] ? (voxel_border.x + sgn(dir.x) * grid_size / 2.0f) / dir.x : max,
	    step[1] ? (voxel_border.y + sgn(dir.y) * grid_size / 2.0f) / dir.y : max,
	    step[2] ? (voxel_border.z + sgn(dir.z) * grid_size / 2.0f) / dir.z : max);

	Vector3f t_delta(step[0] ? grid_size / std::abs(dir.x) : max,
	                 step[1] ? grid_size / std::abs(dir.y) : max,
	                 step[2] ? grid_size / std::abs(dir.z) : max);

	// FIXME: Is this correct? Should it be zero if all zero?
	std::size_t steps = (step[0] ? std::ceil((distance - t_max[0]) / t_delta[0]) : 0) +
	                    (step[1] ? std::ceil((distance - t_max[1]) / t_delta[1]) : 0) +
	                    (step[2] ? std::ceil((distance - t_max[2]) / t_delta[2]) : 0);

	Code  cur           = k_origin;
	Code  prev_at_depth = cur.toDepth(Grid::depth() + depth);
	Grid* grid          = &grids[prev_at_depth];
	grid->set(cur);

	if (ray_passthrough_hits) {
		while (steps--) {
			auto const advance_dim = t_max.minElementIndex();
			k_origin[advance_dim] += step[advance_dim];
			t_max[advance_dim] += t_delta[advance_dim];
			cur = k_origin;

			if (!Code::equalAtDepth(prev_at_depth, cur, Grid::depth() + depth)) {
				prev_at_depth = cur.toDepth(Grid::depth() + depth);
				grid          = &grids[prev_at_depth];
			}
			grid->set(cur);
		}
	} else {
		auto       hit_grid     = hits.find(prev_at_depth);
		auto const hit_grid_end = std::cend(hits);

		while (steps-- && (hit_grid_end == hit_grid || !hit_grid->second.test(cur))) {
			auto const advance_dim = t_max.minElementIndex();
			k_origin[advance_dim] += step[advance_dim];
			t_max[advance_dim] += t_delta[advance_dim];
			cur = k_origin;

			if (!Code::equalAtDepth(prev_at_depth, cur, Grid::depth() + depth)) {
				prev_at_depth = cur.toDepth(Grid::depth() + depth);
				grid          = &grids[prev_at_depth];
				hit_grid      = hits.find(prev_at_depth);
			}
			grid->set(cur);
		}
	}

	auto last = k_origin;

	std::size_t advance_dim{};
	while (std::max(last[advance_dim], k_origin[advance_dim]) -
	           std::min(last[advance_dim], k_origin[advance_dim]) !=
	       inflate_unknown) {
		advance_dim = t_max.minElementIndex();
		k_origin[advance_dim] += step[advance_dim];
		t_max[advance_dim] += t_delta[advance_dim];
		Code const cur = k_origin;

		if (!Code::equalAtDepth(prev_at_depth, cur, Grid::depth() + depth)) {
			grid          = &grids[cur.toDepth(Grid::depth() + depth)];
			prev_at_depth = cur.toDepth(Grid::depth() + depth);
		}
		grid->set(cur);
	}
}

template <class Map>
void computeRaySimple(Map const& map, CodeUnorderedMap<Grid>& grids, Point origin,
                      Point goal, depth_t depth, float step_size,
                      float max_distance        = std::numeric_limits<float>::max(),
                      float early_stop_distance = 0.0f)
{
	Vector3f dir      = goal - origin;
	float    distance = dir.norm();
	dir /= distance;

	distance = std::min(distance - early_stop_distance, max_distance);

	std::size_t num_steps = static_cast<std::size_t>(distance / step_size);
	Vector3f    step      = dir * step_size;

	Code  prev_at_depth = map.toCode(origin, Grid::depth() + depth);
	Grid* grid          = &grids[prev_at_depth];
	grid->set(map.toCode(origin, depth));
	for (std::size_t i{}; i != num_steps; ++i, origin += step) {
		Code cur = map.toCode(origin, depth);

		if (Code::equalAtDepth(prev_at_depth, cur, Grid::depth() + depth)) {
			grid          = &grids[cur.toDepth(Grid::depth() + depth)];
			prev_at_depth = cur.toDepth(Grid::depth() + depth);
		}
		grid->set(cur);
	}
}

[[nodiscard]] std::vector<Code> computeRay(
    Key origin, Key goal, float max_distance = std::numeric_limits<float>::max(),
    std::size_t early_stop = 0, float early_stop_distance = 0.0f)
{
	assert(origin.depth() == goal.depth());

	int const size = static_cast<int>(origin.step());

	Vector3f o(static_cast<float>(origin.x()), static_cast<float>(origin.y()),
	           static_cast<float>(origin.z()));
	Vector3f g(static_cast<float>(goal.x()), static_cast<float>(goal.y()),
	           static_cast<float>(goal.z()));

	Vector3f dir      = g - o;
	float    distance = dir.norm();
	dir /= distance;

	distance = std::min(distance, max_distance);

	Vector3i step(sgn(dir.x) * size, sgn(dir.y) * size, sgn(dir.z) * size);

	dir.abs();

	constexpr auto max    = std::numeric_limits<float>::max();
	float          f_size = static_cast<float>(size);

	Vector3f t_delta(step.x ? f_size / dir.x : max, step.y ? f_size / dir.y : max,
	                 step.z ? f_size / dir.z : max);

	Vector3f t_max = t_delta / 2.0f;

	// ray.reserve(static_cast<KeyRay::size_type>(1.5 * Vector3f::abs(g - o).norm() /
	// f_size));
	std::vector<Code> ray;
	ray.emplace_back(origin);
	distance -= early_stop_distance;
	while (origin != goal && t_max.min() <= distance) {
		auto advance_dim = t_max.minElementIndex();
		// TODO: How to fix this case?
		origin[advance_dim] += static_cast<key_t>(step[advance_dim]);
		t_max[advance_dim] += t_delta[advance_dim];
		ray.emplace_back(origin);
	}
	for (; 0 != early_stop && !ray.empty(); --early_stop) {
		ray.pop_back();
	}
	return ray;
}

[[nodiscard]] std::vector<Code> computeRaySimple(
    Key origin, Key goal, float step_size_factor = 1.0f,
    float max_distance = std::numeric_limits<float>::max(), std::size_t early_stop = 0,
    float early_stop_distance = 0.0f)
{
	auto const depth = origin.depth();

	assert(goal.depth() == depth);

	Vector3f current(static_cast<float>(origin.x()), static_cast<float>(origin.y()),
	                 static_cast<float>(origin.z()));
	Vector3f last(static_cast<float>(goal.x()), static_cast<float>(goal.y()),
	              static_cast<float>(goal.z()));

	Vector3f dir      = last - current;
	float    distance = dir.norm();
	dir /= distance;

	distance = std::min(distance, max_distance);

	auto step_size = static_cast<float>(1U << depth) * step_size_factor;

	std::size_t num_steps = static_cast<std::size_t>(
	    (distance - std::min(distance, early_stop_distance)) / step_size);
	Vector3f step = dir * step_size;

	num_steps = num_steps - std::min(num_steps, early_stop);

	if (0 == num_steps) {
		return std::vector<Code>();
	}

	// FIXME: Should it take one more step?
	std::vector<Code> ray;
	ray.reserve(num_steps);
	for (std::size_t i{}; i != num_steps; ++i, current += step) {
		ray.emplace_back(Key(static_cast<key_t>(current.x), static_cast<key_t>(current.y),
		                     static_cast<key_t>(current.z), depth));
	}
	return ray;
}

[[nodiscard]] std::vector<Point> computeRaySimple(
    Point origin, Point goal, float step_size,
    float max_distance = std::numeric_limits<float>::max(), std::size_t early_stop = 0,
    float early_stop_distance = 0.0f)
{
	Vector3f dir      = goal - origin;
	float    distance = dir.norm();
	dir /= distance;

	distance = std::min(distance, max_distance);

	distance -= early_stop_distance;

	std::size_t num_steps = static_cast<std::size_t>(distance / step_size);
	num_steps -= std::min(num_steps, early_stop);
	Vector3f step = dir * step_size;

	// FIXME: Should it take one more step?
	std::vector<Point> ray;
	ray.reserve(num_steps);
	for (std::size_t i{}; i != num_steps; ++i, origin += step) {
		ray.push_back(origin);
	}
	return ray;
}
}  // namespace ufo

#endif  // UFO_MAP_RAY_CASTER_HPP