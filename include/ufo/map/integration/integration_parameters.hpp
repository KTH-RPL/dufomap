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

#ifndef UFO_MAP_INTEGRATION_PARAMETERS_HPP
#define UFO_MAP_INTEGRATION_PARAMETERS_HPP

// UFO
#include <ufo/map/integration/grid.hpp>
#include <ufo/map/types.hpp>
#include <ufo/util/timing.hpp>

// STL
#include <cstddef>
#include <thread>
#include <vector>

namespace ufo
{
enum class RayCastingMethod { PROPER, SIMPLE };

enum class DownSamplingMethod { NONE, CENTER, CENTROID, UNIFORM };

struct IntegrationParams {
	DownSamplingMethod down_sampling_method{DownSamplingMethod::CENTER};

	depth_t hit_depth{};
	depth_t miss_depth{};
	depth_t ray_casting_depth{};

	// Min range to integrate
	float min_range{0};
	// Max range to integrate, negative value is infinity range
	float max_range{-1};

	bool only_valid{false};

	float early_stop_distance{0.0f};

	RayCastingMethod ray_casting_method{RayCastingMethod::PROPER};
	float            simple_ray_casting_factor{1.0f};

	bool ray_passthrough_hits{true};

	// int   shrink_free_steps{2};
	// bool  extend_free{false};
	// bool  extended_free_marked_hit{false};
	// float extend_hits_distance{};
	// bool  extend_hits_weighted{false};

	// Occupancy hit [0, 1]
	occupancy_t occupancy_hit{0.7f};
	// Occupancy miss [0, 1]
	occupancy_t occupancy_miss{0.4f};

	// Time
	mutable time_t time{1};
	// How much time should automatically increase after function call
	time_t time_auto_inc{1};

	// Semantic specific
	value_t value_hit{2};
	value_t value_miss{1};

	int sliding_window_size{};

	bool        parallel{true};
	std::size_t num_threads = 8 * std::thread::hardware_concurrency();

	bool propagate{false};

	// Device
	Device device{Device::CPU};

	// Inflate unknown
	std::size_t inflate_unknown{0};
	bool        inflate_unknown_compensation{false};

	// Inflate hits
	float inflate_hits_dist{0.0f};

	// IMPROVE: Move somewhere else
	mutable std::vector<CodeUnorderedMap<Grid>> misses;
	mutable std::vector<CodeUnorderedMap<Grid>> hits;
	mutable std::size_t                         misses_idx{};
	mutable std::size_t                         hits_idx{};

	// Timing
	mutable Timing timing{
	    "Insert Pointcloud",
	    {{1, {"Filter Distance 1"}},
	     {2, {"Create Int. Cloud"}},
	     {3, {"Filter Distance 2"}},
	     {4, {"Integrate Hits", {{1, {"Create Nodes"}}, {2, {"Update Nodes"}}}}},
	     {5,
	      {"Get Misses",
	       {{1, {"Generate Ray Cast Goals"}},
	        {2, {"Ray Cast"}},
	        {3, {"Accum. Grids"}},
	        {4, {"Extract Misses"}}}}},
	     {6, {"Integrate Misses", {{1, {"Create Nodes"}}, {2, {"Update Nodes"}}}}},
	     {7, {"Propagate"}}}};
};
}  // namespace ufo

#endif  // UFO_MAP_INTEGRATION_PARAMETERS_HPP