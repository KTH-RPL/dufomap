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

#ifndef UFO_MAP_UFO_MAP_HPP
#define UFO_MAP_UFO_MAP_HPP

// UFO
#include <ufo/map/color/color_map.hpp>
#include <ufo/map/color/color_predicate.hpp>
// #include <ufo/map/distance/distance_map.hpp>
// #include <ufo/map/distance/distance_predicate.hpp>
#include <ufo/map/count/count_map.hpp>
#include <ufo/map/count/count_predicate.hpp>
#include <ufo/map/empty/empty_map.hpp>
#include <ufo/map/empty/empty_predicate.hpp>
#include <ufo/map/free/free_map.hpp>
#include <ufo/map/free/free_predicate.hpp>
#include <ufo/map/intensity/intensity_map.hpp>
#include <ufo/map/intensity/intensity_predicate.hpp>
#include <ufo/map/io.hpp>
#include <ufo/map/label/label_map.hpp>
#include <ufo/map/label/label_predicate.hpp>
#include <ufo/map/occupancy/occupancy_map.hpp>
#include <ufo/map/occupancy/occupancy_predicate.hpp>
#include <ufo/map/octree/octree_map.hpp>
#include <ufo/map/semantic/semantic_map.hpp>
#include <ufo/map/semantic/semantic_predicate.hpp>
#include <ufo/map/value/value_map.hpp>
#include <ufo/map/value/value_predicate.hpp>
// #include <ufo/map/labels/labels_map.hpp>
// #include <ufo/map/labels/labels_predicate.hpp>
#include <ufo/map/reflection/reflection_map.hpp>
#include <ufo/map/reflection/reflection_predicate.hpp>
// #include <ufo/map/surfel/surfel_map.hpp>
// #include <ufo/map/surfel/surfel_predicate.hpp>
#include <ufo/map/integration/integration.hpp>
#include <ufo/map/points/points_map.hpp>
#include <ufo/map/points/points_predicate.hpp>
// #include <ufo/map/points_color/points_color_map.hpp>
// #include <ufo/map/points_color/points_color_predicate.hpp>
#include <ufo/map/point/point_map.hpp>
// #include <ufo/map/point/point_predicate.hpp>
#include <ufo/map/seen_free/seen_free_map.hpp>
#include <ufo/map/seen_free/seen_free_predicate.hpp>
#include <ufo/map/time/time_map.hpp>
#include <ufo/map/time/time_predicate.hpp>
#include <ufo/map/types.hpp>

// STL
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <type_traits>

namespace ufo
{
//
// Helpers
//

template <bool C, mt_t T, template <class, std::size_t> class Map>
struct cond_map {
	template <class D, std::size_t N>
	using type = Map<D, N>;
};

template <mt_t T, template <class, std::size_t> class Map>
struct cond_map<false, T, Map> {
	template <class D, std::size_t N>
	using type = EmptyMap<T, D, N>;
};

//
// Map
//

template <mt_t MapType>
class Map
    : public OctreeMap<
          // clang-format off
          cond_map<0 != (MapType & OCCUPANCY),        OCCUPANCY,        OccupancyMap>::template type,
          cond_map<0 != (MapType & COLOR),            COLOR,            ColorMap>::template type,
          cond_map<0 != (MapType & TIME),             TIME,             TimeMap>::template type,
          cond_map<0 != (MapType & INTENSITY),        INTENSITY,        IntensityMap>::template type,
          cond_map<0 != (MapType & COUNT),            COUNT,            CountMap>::template type,
          cond_map<0 != (MapType & REFLECTION),       REFLECTION,       ReflectionMap>::template type,
          cond_map<0 != (MapType & POINT),       			POINT,       			PointMap>::template type,
					cond_map<0 != (MapType & POINTS8),          POINTS8,          PointsMap8>::template type,
					cond_map<0 != (MapType & POINTS16),         POINTS16,         PointsMap16>::template type,
					cond_map<0 != (MapType & POINTS32),         POINTS32,         PointsMap32>::template type,
					// cond_map<0 != (MapType & POINTS_COLOR8),    POINTS_COLOR8,    PointsColorMap8>::template type,
					// cond_map<0 != (MapType & POINTS_COLOR16),   POINTS_COLOR16,   PointsColorMap16>::template type,
					// cond_map<0 != (MapType & POINTS_COLOR32),   POINTS_COLOR32,   PointsColorMap32>::template type,
					cond_map<0 != (MapType & FREE),             FREE,             FreeMap>::template type,
					cond_map<0 != (MapType & SEEN_FREE),        SEEN_FREE,        SeenFreeMap>::template type,
          // cond_map<MapType & SURFEL,     SURFEL,     SurfelMap>::template type,
					cond_map<0 != (MapType & LABEL),            LABEL,            LabelMap>::template type,
					cond_map<0 != (MapType & VALUE),            VALUE,            ValueMap>::template type,
					cond_map<0 != (MapType & SEMANTIC),         SEMANTIC,         SemanticMap>::template type
          // cond_map<MapType & DISTANCE,   DISTANCE,   DistanceMap>::template type,
          // clang-format on
          >
{
 private:
	using Base = OctreeMap<
	    // clang-format off
			cond_map<0 != (MapType & OCCUPANCY),        OCCUPANCY,        OccupancyMap>::template type,
			cond_map<0 != (MapType & COLOR),            COLOR,            ColorMap>::template type,
			cond_map<0 != (MapType & TIME),             TIME,             TimeMap>::template type,
			cond_map<0 != (MapType & INTENSITY),        INTENSITY,        IntensityMap>::template type,
			cond_map<0 != (MapType & COUNT),            COUNT,            CountMap>::template type,
			cond_map<0 != (MapType & REFLECTION),       REFLECTION,       ReflectionMap>::template type,
			cond_map<0 != (MapType & POINT),       			POINT,       			PointMap>::template type,
			cond_map<0 != (MapType & POINTS8),          POINTS8,          PointsMap8>::template type,
			cond_map<0 != (MapType & POINTS16),         POINTS16,         PointsMap16>::template type,
			cond_map<0 != (MapType & POINTS32),         POINTS32,         PointsMap32>::template type,
			// cond_map<0 != (MapType & POINTS_COLOR8),    POINTS_COLOR8,    PointsColorMap8>::template type,
			// cond_map<0 != (MapType & POINTS_COLOR16),   POINTS_COLOR16,   PointsColorMap16>::template type,
			// cond_map<0 != (MapType & POINTS_COLOR32),   POINTS_COLOR32,   PointsColorMap32>::template type,
			cond_map<0 != (MapType & FREE),             FREE,             FreeMap>::template type,
			cond_map<0 != (MapType & SEEN_FREE),        SEEN_FREE,        SeenFreeMap>::template type,
			// cond_map<MapType & SURFEL,     SURFEL,     SurfelMap>::template type,
			cond_map<0 != (MapType & LABEL),            LABEL,            LabelMap>::template type,
			cond_map<0 != (MapType & VALUE),            VALUE,            ValueMap>::template type,
			cond_map<0 != (MapType & SEMANTIC),         SEMANTIC,         SemanticMap>::template type
			// cond_map<MapType & DISTANCE,   DISTANCE,   DistanceMap>::template type,
	    // clang-format on
	    >;

 public:
	//
	// Constructors
	//

	Map(node_size_t leaf_node_size = 0.1, depth_t depth_levels = 17)
	    : Base(leaf_node_size, depth_levels)
	{
	}

	Map(std::filesystem::path const& file) : Base(file) {}

	Map(std::istream& in) : Base(in) {}

	Map(ReadBuffer& in) : Base(in) {}

	Map(Map const& other) = default;

	Map(Map&& other) = default;

	template <mt_t MapType2>
	Map(Map<MapType2> const& other) : Base(other)
	{
	}

	template <mt_t MapType2>
	Map(Map<MapType2>&& other) : Base(std::move(other))
	{
	}

	//
	// Operator assignment
	//

	Map& operator=(Map const& rhs) = default;

	Map& operator=(Map&& rhs) = default;

	template <mt_t MapType2>
	Map& operator=(Map<MapType2> const& rhs)
	{
		Base::operator=(rhs);
		return *this;
	}

	template <mt_t MapType2>
	Map& operator=(Map<MapType2>&& rhs)
	{
		Base::operator=(std::move(rhs));
		return *this;
	}

	//
	// Swap
	//

	void swap(Map& other) noexcept(noexcept(Base::swap(other))) { Base::swap(other); }
};
}  // namespace ufo

namespace std
{
template <ufo::mt_t MapType>
void swap(ufo::Map<MapType>& lhs,
          ufo::Map<MapType>& rhs) noexcept(noexcept(lhs.swap(rhs)))
{
	lhs.swap(rhs);
}
}  // namespace std

#endif  // UFO_MAP_UFO_MAP_HPP