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

#ifndef UFO_MAP_INTEGRATION_HPP
#define UFO_MAP_INTEGRATION_HPP

// UFO
#include <ufo/map/integration/integration_impl.hpp>
#include <ufo/map/integration/integration_parameters.hpp>
#include <ufo/map/integration/integration_point_cloud.hpp>
#include <ufo/map/types.hpp>

// STL
#include <future>

#ifdef UFO_PARALLEL
// STL
#include <execution>
#endif

namespace ufo
{
/*!
 * Integrate a point points into a map.
 *
 * @param map Map to integrate into.
 * @param points Point points to integrate.
 * @param propagate Whether to update the inner nodes of the map.
 */
template <class Map, class... P>
void insertPointCloud(Map& map, Cloud<P...> const& points,
                      IntegrationParams const& params, bool propagate = true)
{
	// Create integration points
	auto ic = impl::toIntegrationCloud(map, points, params);

	// Integrate hits into the map
	impl::integrateHits(map, std::move(ic), params);

	if (propagate) {
		// Propagate information in the map
		map.propagateModified();
	}

	// Increase time
	params.time += params.time_auto_inc;
}

/*!
 * Integrate a point points into a map. Ray casting is used to clear free space between
 * the points in the point points and the sensor origin.
 *
 * @param map Map to integrate into.
 * @param points Point points in global reference frame to integrate.
 * @param sensor_origin Origin of the sensor in global reference frame.
 * @param propagate Whether to update the inner nodes of the map.
 */
template <class Map, class... P>
void insertPointCloud(Map& map, Cloud<P...> points, Point sensor_origin,
                      IntegrationParams const& params, bool propagate = true)
{
	params.timing.start();

	params.timing[1].start();
	if (params.only_valid && 0 <= params.max_range) {
		// Remove points that are further than max range
#ifdef UFO_PARALLEL
		if (params.parallel) {
			filterDistance(std::execution::par_unseq, points, sensor_origin, params.max_range);
		} else
#endif
		{
			filterDistance(points, sensor_origin, params.max_range);
		}
	}
	params.timing[1].stop();

	// Create integration points
	params.timing[2].start();
	auto ic = impl::toIntegrationCloud(map, std::move(points), params);
	params.timing[2].stop();

	auto f = std::async(std::launch::async, [&map, ic, sensor_origin, &params]() mutable {
		params.timing[3].start();
		if (!params.only_valid && 0 <= params.max_range) {
			// Remove points that are further than max range
#ifdef UFO_PARALLEL
			if (params.parallel) {
				filterDistance(std::execution::par_unseq, ic, sensor_origin, params.max_range);
			} else
#endif
			{
				filterDistance(ic, sensor_origin, params.max_range);
			}
		}
		params.timing[3].stop();

		// Integrate hits into the map
		params.timing[4].start();
		impl::integrateHits(map, std::move(ic), params);
		params.timing[4].stop();
	});

	// Ray cast to get misses (free space)
	params.timing[5].start();
	auto misses = impl::getMisses(map, std::move(ic), sensor_origin, params);
	params.timing[5].stop();

	// Wait until all hits has been inserted
	f.wait();

	// Integrate misses into the map
	params.timing[6].start();
	impl::integrateMisses(map, std::move(misses), params);
	params.timing[6].stop();

	params.timing[7].start();
	if (propagate) {
		// Propagate information in the map
		map.propagateModified();
	}
	params.timing[7].stop();

	// Increase time
	params.time += params.time_auto_inc;
	params.timing.stop();
}

/*!
 * Integrate a point points into a map. Ray casting is used to clear free space between
 * the points in the point points and the sensor origin.
 *
 * @param map Map to integrate into.
 * @param sensor_origin Origin of the sensor relative to frame_origin.
 * @param points Point points relative to frame_origin to integrate.
 * @param frame_origin Origin of reference frame, determines transform to be applied to
 * points and sensor_origin.
 * @param propagate Whether to update the inner nodes of the map.
 */
template <class Map, class... P>
void insertPointCloud(Map& map, Cloud<P...> points, Point sensor_origin,
                      Pose6f frame_origin, IntegrationParams const& params,
                      bool propagate = true)
{
#ifdef UFO_PARALLEL
	if (params.parallel) {
		applyTransform(std::execution::par_unseq, points, frame_origin);
	} else
#endif
	{
		applyTransform(points, frame_origin);
	}

	// FIXME: What is correct?
	// insertPointCloud(map, std::move(points),
	// frame_origin.transform(sensor_origin), params,
	//                  propagate);
	insertPointCloud(map, std::move(points), sensor_origin, params, propagate);
}
}  // namespace ufo

#endif  // UFO_MAP_INTEGRATION_HPP