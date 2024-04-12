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

#ifndef UFO_MAP_POINT_HPP
#define UFO_MAP_POINT_HPP

// UFO
#include <ufo/map/color/color.hpp>
#include <ufo/map/semantic/semantic.hpp>
#include <ufo/map/types.hpp>
#include <ufo/math/vector3.hpp>

// STL
#include <concepts>

#include "ufo/map/intensity/intensity_map.hpp"

namespace ufo
{
using Point = Vector3<coord_t>;

struct Intensity {
	intensity_t intensity;

	constexpr Intensity(intensity_t intensity = 0) : intensity(intensity) {}
};

struct Label {
	label_t label;

	constexpr Label(label_t label = 0) : label(label) {}
};

struct Value {
	value_t value;

	constexpr Value(value_t value = 0) : value(value) {}
};

struct Dynamic {
	bool dynamic;

	constexpr Dynamic(bool dynamic = false) : dynamic(dynamic) {}
};

// struct Normal {
// 	// TODO: Implement
// };

//
// Concepts
//

template <class T>
concept IsColor =
    (std::derived_from<T, Color> || std::derived_from<typename T::value_type, Color>);

template <class... Ts>
concept IsAnyColor = (IsColor<Ts> || ...);

template <class T>
concept IsIntensity = (std::derived_from<T, Intensity> ||
                       std::derived_from<typename T::value_type, Intensity>);

template <class... Ts>
concept IsAnyIntensity = (IsIntensityMap<Ts> || ...);

template <class T>
concept IsLabel = (std::derived_from<T, Label> || std::derived_from<T, Semantic> ||
                   std::derived_from<typename T::value_type, Label> ||
                   std::derived_from<typename T::value_type, Semantic>);

template <class... Ts>
concept IsAnyLabel = (IsLabel<Ts> || ...);

template <class T>
concept IsValue = (std::derived_from<T, Value> || std::derived_from<T, Semantic> ||
                   std::derived_from<typename T::value_type, Value> ||
                   std::derived_from<typename T::value_type, Semantic>);

template <class... Ts>
concept IsAnyValue = (IsValue<Ts> || ...);

template <class T>
concept IsSemantic = (std::derived_from<T, Semantic> ||
                      std::derived_from<typename T::value_type, Semantic>);

template <class... Ts>
concept IsAnySemantic = (IsSemantic<Ts> || ...);

}  // namespace ufo

#endif  // UFO_MAP_POINT_HPP