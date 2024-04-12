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

#ifndef UFO_MAP_INTEGRATION_IMPL_HPP
#define UFO_MAP_INTEGRATION_IMPL_HPP

// UFO
#include <ufo/algorithm/algorithm.hpp>
#include <ufo/geometry/minimum_distance.hpp>
#include <ufo/map/bit_set.hpp>
#include <ufo/map/code.hpp>
#include <ufo/map/color/color_map.hpp>
#include <ufo/map/count/count_map.hpp>
#include <ufo/map/free/free_map.hpp>
#include <ufo/map/integration/grid.hpp>
#include <ufo/map/integration/integration.hpp>
#include <ufo/map/integration/integration_parameters.hpp>
#include <ufo/map/integration/integration_point.hpp>
#include <ufo/map/integration/integration_point_cloud.hpp>
#include <ufo/map/integration/misses.hpp>
#include <ufo/map/intensity/intensity_map.hpp>
#include <ufo/map/key.hpp>
#include <ufo/map/label/label_map.hpp>
#include <ufo/map/occupancy/occupancy_map.hpp>
#include <ufo/map/point.hpp>
#include <ufo/map/point_cloud.hpp>
#include <ufo/map/points/points_map.hpp>
// #include <ufo/map/points_color/points_color_map.hpp>
#include <ufo/map/ray_caster/ray_caster.hpp>
#include <ufo/map/reflection/reflection.hpp>
#include <ufo/map/reflection/reflection_map.hpp>
#include <ufo/map/semantic/semantic_map.hpp>
// #include <ufo/map/surfel/surfel_map.hpp>
#include <ufo/map/seen_free/seen_free_map.hpp>
#include <ufo/map/time/time_map.hpp>
#include <ufo/map/types.hpp>
#include <ufo/map/value/value_map.hpp>
#include <ufo/math/pose6.hpp>
#include <ufo/util/timing.hpp>
#include <ufo/util/type_traits.hpp>
// #include <ufo/map/distance/distance_map.hpp>

// STL
#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <future>
#include <limits>
#include <mutex>
#include <numeric>
#include <queue>
#include <shared_mutex>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <vector>

#ifdef UFO_PARALLEL
// OMP
#include <omp.h>

// STL
#include <execution>
#endif

namespace ufo::impl
{
//
// To integration cloud
//

template <class Map, class... P>
auto toIntegrationCloud(Map const& map, Cloud<P...> const& points,
                        IntegrationParams const& params)
{
	IntegrationCloud<typename Cloud<P...>::value_type> ic(points.size());

#ifdef UFO_PARALLEL
	if (params.parallel) {
		std::transform(std::execution::par_unseq, std::cbegin(points), std::cend(points),
		               std::begin(ic),
		               [&map](auto const& p) { return IntegrationPoint(p, map.toCode(p)); });

		std::sort(std::execution::par_unseq, std::begin(ic), std::end(ic),
		          [](auto const& a, auto const& b) { return a.code < b.code; });
	} else
#endif
	{
		std::ranges::transform(points, std::begin(ic), [&map](auto const& p) {
			return IntegrationPoint(p, map.toCode(p));
		});

		std::ranges::sort(ic, [](auto const& a, auto const& b) { return a.code < b.code; });
	}

	return ic;
}

//
// Down sample
//

template <class Map, class P>
[[nodiscard]] std::vector<Point> downSample(Map const&                 map,
                                            IntegrationCloud<P> const& points,
                                            DownSamplingMethod method, depth_t depth)
{
	std::vector<Point> down_sampled;
	down_sampled.reserve(points.size());
	switch (method) {
		case DownSamplingMethod::NONE:
			for (auto const& p : points) {
				down_sampled.push_back(p);
			}
			break;
		case DownSamplingMethod::CENTER:
			for (auto it = std::cbegin(points), last = std::cend(points); it != last;) {
				auto c   = it->code.toDepth(depth);
				auto cur = it;
				it       = std::find_if_not(++it, last, [c, depth](auto const& e) {
          return Code::equalAtDepth(e.code, c, depth);
        });
				down_sampled.push_back(map.toCoord(c));
			}
			break;
		case DownSamplingMethod::CENTROID:
			for (auto it = std::cbegin(points), last = std::cend(points); it != last;) {
				auto c   = it->code.toDepth(depth);
				auto cur = it;
				it       = std::find_if_not(++it, last, [c, depth](auto const& e) {
          return Code::equalAtDepth(e.code, c, depth);
        });
				// Get mean of all points that fall into voxel
				down_sampled.push_back(std::accumulate(cur, it, Point()) /
				                       std::distance(cur, it));
			}
			break;
		case DownSamplingMethod::UNIFORM:
			for (auto it = std::cbegin(points), last = std::cend(points); it != last;) {
				auto  c       = it->code.toDepth(depth);
				Point center  = map.toCoord(c);
				Point closest = *it;
				float dist_sq = center.squaredDistance(closest);
				for (++it; Code::equalAtDepth(it->code, c, depth); ++it) {
					float dsq = center.squaredDistance(*it);
					if (dsq < dist_sq) {
						dist_sq = dsq;
						closest = *it;
					}
				}
				down_sampled.push_back(closest);
			}
			break;
	}
	return down_sampled;
}

//
// Get misses
//

template <class Map, class P>
[[nodiscard]] Misses getMisses(Map const& map, IntegrationCloud<P> const& points,
                               Point const sensor_origin, IntegrationParams const& params)
{
	if (points.empty()) {
		return Misses();
	}

	// TODO: Use different miss_depth and ray_casting_depth

	// INFO: Assuming all points have same depth
	depth_t depth = std::max(points.front().code.depth(), params.miss_depth);

	params.timing[5][1].start();
	std::vector<Point> goals = downSample(map, points, params.down_sampling_method, depth);
	params.timing[5][1].stop();

	CodeUnorderedMap<Grid> misses;
	CodeUnorderedMap<Grid> hits;

	Key const   origin_key          = map.toKey(sensor_origin, depth);
	Point const origin_coord        = map.toCoord(origin_key);
	float const step_size_factor    = params.simple_ray_casting_factor;
	bool const  simple              = RayCastingMethod::SIMPLE == params.ray_casting_method;
	float const early_stop_distance = params.early_stop_distance;
	float const min_distance        = params.min_range;
	float const max_distance =
	    0 > params.max_range ? std::numeric_limits<float>::max() : params.max_range;
	float const max_distance_sq              = max_distance * max_distance;
	bool const  only_valid                   = params.only_valid;
	bool const  inflate_unknown_compensation = params.inflate_unknown_compensation;
	std::size_t inflate_unknown              = params.inflate_unknown;
	std::size_t compensate_inflate_unknown =
	    inflate_unknown_compensation ? inflate_unknown : 0;
	float const grid_size = map.size(depth);
	// float const iuc =
	// inflate_unknown_compensation ? inflate_unknown * grid_size : 0;  // * std::sqrt(3)
	bool const  ray_passthrough_hits = params.ray_passthrough_hits;
	float const inflate_hits_dist    = params.inflate_hits_dist;
	std::size_t num_threads{};

	params.timing[5][2].start();
#ifdef UFO_PARALLEL
	if (params.parallel) {
		num_threads = 0 == params.num_threads ? 8 * std::thread::hardware_concurrency()
		                                      : params.num_threads;
#pragma omp parallel num_threads(num_threads)
		{
			CodeUnorderedMap<Grid> thread_hits;
#pragma omp for schedule(static)
			for (auto goal : goals) {
				auto       direction = goal - sensor_origin;
				auto const distance  = direction.norm();
				if (min_distance >
				    distance) {  // IMPROVE: Probably have to do something more than this
					continue;
				}
				direction /= distance;
				auto origin = goal - (direction * inflate_hits_dist);

				auto origin_key = map.toKey(origin, depth);

				// FIXME: If inflate_hits_dist is zero then no direction for inflate_unknown
				computeRay(thread_hits, origin, goal, origin_key,
				           map.toCoord(origin_key) - origin, direction, grid_size,
				           inflate_unknown_compensation);
			}

#pragma omp critical
			{
				for (auto&& [code, grid] : thread_hits) {
					if (auto [it, b] = hits.try_emplace(code, std::move(grid)); !b) {
						std::transform(std::cbegin(grid), std::cend(grid), std::cbegin(it->second),
						               std::begin(it->second), [](auto a, auto b) { return a | b; });
					}
				}
			}

			CodeUnorderedMap<Grid> thread_misses;

#pragma omp barrier
#pragma omp for schedule(static)
			for (auto goal : goals) {
				computeRay(thread_misses, hits, sensor_origin, goal, origin_key,
				           origin_coord - sensor_origin, grid_size, max_distance,
				           compensate_inflate_unknown, ray_passthrough_hits);
			}

#pragma omp critical
			{
				for (auto&& [code, grid] : thread_misses) {
					if (auto [it, b] = misses.try_emplace(code, std::move(grid)); !b) {
						std::transform(std::cbegin(grid), std::cend(grid), std::cbegin(it->second),
						               std::begin(it->second), [](auto a, auto b) { return a | b; });
					}
				}
			}
		}
	} else
#endif
	{
		for (auto goal : goals) {
			auto       direction = goal - sensor_origin;
			auto const distance  = direction.norm();
			direction /= distance;
			auto origin = goal - (direction * inflate_hits_dist);

			auto origin_key = map.toKey(origin, depth);
			computeRay(hits, origin, goal, origin_key, map.toCoord(origin_key) - origin,
			           direction, grid_size, compensate_inflate_unknown);
		}

		for (auto goal : goals) {
			if (!only_valid && 0 <= max_distance) {
				auto       direction   = goal - sensor_origin;
				auto const distance_sq = direction.squaredNorm();
				if (max_distance_sq < distance_sq) {
					direction /= std::sqrt(distance_sq);
					goal = sensor_origin + (direction * max_distance);
				}
			}

			computeRay(misses, hits, sensor_origin, goal, origin_key,
			           origin_coord - sensor_origin, grid_size, max_distance,
			           compensate_inflate_unknown, ray_passthrough_hits);
		}
	}
	params.timing[5][2].stop();

	Misses x;
	if (1 < params.sliding_window_size) {
		params.timing[5][3].start();
		params.misses.resize(params.sliding_window_size);
		params.hits.resize(params.sliding_window_size);
		params.misses_idx %= params.sliding_window_size;
		params.hits_idx %= params.sliding_window_size;
		params.misses[params.misses_idx++] = std::move(misses);
		params.hits[params.hits_idx++]     = std::move(hits);

		CodeUnorderedMap<Grid> acc_misses;
		for (auto const& g : params.misses) {
			for (auto&& [code, grid] : g) {
				if (auto [it, b] = acc_misses.try_emplace(code, std::move(grid)); !b) {
					std::transform(std::cbegin(grid), std::cend(grid), std::cbegin(it->second),
					               std::begin(it->second), [](auto a, auto b) { return a | b; });
				}
			}
		}
		CodeUnorderedMap<Grid> acc_hits;
		for (auto const& g : params.hits) {
			for (auto&& [code, grid] : g) {
				if (auto [it, b] = acc_hits.try_emplace(code, std::move(grid)); !b) {
					std::transform(std::cbegin(grid), std::cend(grid), std::cbegin(it->second),
					               std::begin(it->second), [](auto a, auto b) { return a | b; });
				}
			}
		}
		params.timing[5][3].stop();

		params.timing[5][4].start();
		x = getMisses(inflate_unknown, std::move(acc_misses), std::move(acc_hits), depth,
		              num_threads);
		params.timing[5][4].stop();
	} else {
		params.timing[5][4].start();
		x = getMisses(inflate_unknown, std::move(misses), std::move(hits), depth,
		              num_threads);
		params.timing[5][4].stop();
	}

	return x;
}

//
// Integrate hits
//

template <class Map, class P>
void integrateHits(Map& map, IntegrationCloud<P> points, IntegrationParams const& params)
{
	auto const depth = params.hit_depth;
	if (depth) {
		for (auto& p : points) {
			p.code = p.code.toDepth(depth);
		}
	}

	params.timing[4][1].start();
	map.createIndicesFromCodes(points);
	params.timing[4][1].stop();

	auto    time = params.time;
	logit_t prob{};
	if constexpr (IsOccupancyMap<Map>) {
		prob = map.toOccupancyChangeLogit(params.occupancy_hit);
	}

	params.timing[4][2].start();
	for (auto it = std::cbegin(points), last = std::cend(points); it != last;) {
		auto cur  = it;
		auto node = cur->index;
		it = std::find_if_not(++it, last, [node](auto const& e) { return e.index == node; });

		map.setModified(node);

		if constexpr (IsOccupancyMap<Map>) {
			map.updateOccupancyLogit(node, prob);
		}

		if constexpr (IsTimeMap<Map>) {
			map.setTime(node, time);
		}

		if constexpr (IsColorMap<Map> && IsColor<P>) {
			// TODO: Implement
		}

		if constexpr (IsCountMap<Map>) {
			map.updateCount(node, 1);
		}

		if constexpr (IsReflectionMap<Map>) {
			map.updateReflection(node, std::distance(cur, it), 0);
		}

		if constexpr (IsPointsMap<Map>) {
			map.insertPoints(node, cur, it);
		}

		// if constexpr (IsPointsColorMap<Map> && IsColor<P>) {
		// 	// if (1 > map.numPoints(cur->index)) {  // TODO: Remove
		// 	map.insertPoints(node, cur, it);
		// 	// }
		// }

		if constexpr (IsIntensityMap<Map> && IsIntensity<P>) {
			// TODO: Implement
		}

		if constexpr (IsLabelMap<Map> && IsLabel<P>) {
			// TODO: Implement
		}

		if constexpr (IsSemanticMap<Map> && IsSemantic<P>) {
			// TODO: Implement
		}

		if constexpr (IsValueMap<Map> && IsValue<P>) {
			// float min = std::numeric_limits<float>::max();
			// for (auto it = std::cbegin(points) + i.first, last = std::cbegin(points) +
			// i.second;
			//      it != last; ++it) {
			// 	min = std::min(min, it->value);
			// }
			// map.updateValue(
			//     node, [min](value_t cur) { return 0.0f != cur ? std::min(min, cur) : min; });
		}

		// if constexpr (IsSurfelMap<Map>) {
		// 	// TODO: Implement
		// }
	}
	params.timing[4][2].stop();
}

//
// Integrate misses
//

template <class Map>
void integrateMisses(Map& map, Misses misses, IntegrationParams const& params)
{
	params.timing[6][1].start();
	map.createIndicesFromCodes(misses);
	params.timing[6][1].stop();

	logit_t prob{};
	if constexpr (IsOccupancyMap<Map>) {
		auto prob = map.toOccupancyChangeLogit(params.occupancy_miss);
	}

	params.timing[6][2].start();
#ifdef UFO_PARALLEL
	if (params.parallel) {
		std::for_each(std::execution::par_unseq, std::cbegin(misses), std::cend(misses),
		              [&map, prob, time = params.time](auto miss) {
			              for (offset_t i{}; 8 != i; ++i) {
				              if (miss.sibling[i]) {
					              auto node = map.sibling(miss.index, i);

					              map.setModified(node);

					              if constexpr (IsOccupancyMap<Map>) {
						              map.decreaseOccupancyLogit(node, prob);
					              }

					              if constexpr (IsTimeMap<Map>) {
						              map.setTime(node, time);
					              }

					              if constexpr (IsReflectionMap<Map>) {
						              map.updateReflection(node, 0, 1);
					              }

					              if constexpr (IsFreeMap<Map>) {
						              map.updateFree(node, [freedom = miss.freedom](auto cur) {
							              return std::max(cur, freedom);
						              });
					              }

					              if constexpr (IsSeenFreeMap<Map>) {
						              map.setSeenFree(node);
					              }
				              }
			              }
		              });
	} else
#endif
	{
		for (auto miss : misses) {
			for (offset_t i{}; 8 != i; ++i) {
				if (miss.sibling[i]) {
					auto node = map.sibling(miss.index, i);

					map.setModified(node);

					if constexpr (IsOccupancyMap<Map>) {
						map.decreaseOccupancyLogit(node, prob);
					}

					if constexpr (IsTimeMap<Map>) {
						map.setTime(node, time);
					}

					if constexpr (IsReflectionMap<Map>) {
						map.updateReflection(node, 0, 1);
					}

					if constexpr (IsFreeMap<Map>) {
						map.updateFree(node, [freedom = miss.freedom](auto cur) {
							return std::max(cur, freedom);
						});
					}

					if constexpr (IsSeenFreeMap<Map>) {
						map.setSeenFree(node);
					}
				}
			}
		}
	}
	params.timing[6][2].stop();
}
}  // namespace ufo::impl

#endif  // UFO_MAP_INTEGRATION_IMPL_HPP