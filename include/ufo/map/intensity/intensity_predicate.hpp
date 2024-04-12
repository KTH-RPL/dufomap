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

#ifndef UFO_MAP_INTENSITY_PREDICATE_HPP
#define UFO_MAP_INTENSITY_PREDICATE_HPP

// UFO
#include <ufo/map/intensity/intensity_map.hpp>
#include <ufo/map/node.hpp>
#include <ufo/map/predicate/predicate.hpp>
#include <ufo/map/types.hpp>

namespace ufo::pred
{
//
// Predicates
//

// REVIEW: Name?
struct IntensityMap {
};

template <PredicateCompare PC = PredicateCompare::EQUAL>
struct Intensity {
	constexpr Intensity(intensity_t intensity) : intensity(intensity) {}

	intensity_t intensity;
};

using IntensityE  = Intensity<>;
using IntensityLE = Intensity<PredicateCompare::LESS_EQUAL>;
using IntensityGE = Intensity<PredicateCompare::GREATER_EQUAL>;
using IntensityL  = Intensity<PredicateCompare::LESS>;
using IntensityG  = Intensity<PredicateCompare::GREATER>;

using IntensityMin = IntensityGE;
using IntensityMax = IntensityLE;

struct IntensityInterval {
	constexpr IntensityInterval(intensity_t min, intensity_t max) : min(min), max(max) {}

	IntensityMin min;
	IntensityMax max;
};

//
// Predicate value/return check
//

template <>
struct ValueCheck<IntensityMap> {
	using Pred = IntensityMap;

	template <class Map>
	static constexpr bool apply(Pred, Map const&, Node)
	{
		return IsIntensityMap<Map>;
	}
};

template <class PredPost>
struct ValueCheck<THEN<IntensityMap, PredPost>> {
	using Pred = THEN<IntensityMap, PredPost>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if constexpr (ValueCheck<IntensityMap>::apply(p.pre, m, n)) {
			return ValueCheck<PredPost>::apply(p.post, m, n);
		} else {
			return true;
		}
	}
};

template <PredicateCompare PC>
struct ValueCheck<Intensity<PC>> {
	using Pred = Intensity<PC>;

	template <class Map>
	static constexpr bool apply(Pred p, Map const& m, Node n)
	{
		if constexpr (PredicateCompare::EQUAL == PC) {
			return m.intensity(n.index()) == p.intensity;
		} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
			return m.intensity(n.index()) <= p.intensity;
		} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
			return m.intensity(n.index()) >= p.intensity;
		} else if constexpr (PredicateCompare::LESS == PC) {
			return m.intensity(n.index()) < p.intensity;
		} else if constexpr (PredicateCompare::GREATER == PC) {
			return m.intensity(n.index()) > p.intensity;
		}
	}
};

template <>
struct ValueCheck<IntensityInterval> {
	using Pred = IntensityInterval;

	template <class Map>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		return ValueCheck<std::decay_t<decltype(p.min)>>::apply(p.min, m, n) &&
		       ValueCheck<std::decay_t<decltype(p.max)>>::apply(p.max, m, n);
	}
};

//
// Predicate inner check
//

template <>
struct InnerCheck<IntensityMap> {
	using Pred = IntensityMap;

	template <class Map>
	static constexpr bool apply(Pred, Map const&, Node)
	{
		return IsIntensityMap<Map>;
	}
};

template <class PredPost>
struct InnerCheck<THEN<IntensityMap, PredPost>> {
	using Pred = THEN<IntensityMap, PredPost>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if constexpr (InnerCheck<IntensityMap>::apply(p.pre, m, n)) {
			return InnerCheck<PredPost>::apply(p.post, m, n);
		} else {
			return true;
		}
	}
};

template <PredicateCompare PC>
struct InnerCheck<Intensity<PC>> {
	using Pred = Intensity<PC>;

	template <class Map>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		// FIXME: Check how intensity step is propagated to determine

		if constexpr (PredicateCompare::EQUAL == PC) {
			return m.intensity(n.index()) >= p.intensity;
		} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
			return true;
		} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
			return m.intensity(n.index()) >= p.intensity;
		} else if constexpr (PredicateCompare::LESS == PC) {
			return true;
		} else if constexpr (PredicateCompare::GREATER == PC) {
			return m.intensity(n.index()) > p.intensity;
		}
	}
};

template <>
struct InnerCheck<IntensityInterval> {
	using Pred = IntensityInterval;

	template <class Map>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		return InnerCheck<std::decay_t<decltype(p.min)>>::apply(p.min, m, n) &&
		       InnerCheck<std::decay_t<decltype(p.max)>>::apply(p.max, m, n);
	}
};

}  // namespace ufo::pred

#endif  // UFO_MAP_INTENSITY_PREDICATE_HPP