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

#ifndef UFO_MAP_PREDICATE_POINTS_HPP
#define UFO_MAP_PREDICATE_POINTS_HPP

// UFO
#include <ufo/map/node.hpp>
#include <ufo/map/points/points_map.hpp>
#include <ufo/map/predicate/predicate.hpp>

// STL
#include <cstddef>

namespace ufo::pred
{

//
// Points map
//

struct PointsMap {
};

template <>
struct ValueCheck<PointsMap> {
	using Pred = PointsMap;

	template <class Map>
	static constexpr bool apply(Pred, Map const&, Node)
	{
		return IsPointsMap<Map>;
	}
};

template <class PredPost>
struct ValueCheck<THEN<PointsMap, PredPost>> {
	using Pred = THEN<PointsMap, PredPost>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if constexpr (ValueCheck<PointsMap>::apply(p.pre, m, n)) {
			return ValueCheck<PredPost>::apply(p.post, m, n);
		} else {
			return true;
		}
	}
};

template <>
struct InnerCheck<PointsMap> {
	using Pred = PointsMap;

	template <class Map>
	static constexpr bool apply(Pred, Map const&, Node)
	{
		return IsPointsMap<Map>;
	}
};

template <class PredPost>
struct InnerCheck<THEN<PointsMap, PredPost>> {
	using Pred = THEN<PointsMap, PredPost>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if constexpr (InnerCheck<PointsMap>::apply(p.pre, m, n)) {
			return InnerCheck<PredPost>::apply(p.post, m, n);
		} else {
			return true;
		}
	}
};

//
// Has points
//

template <bool Negated = false>
struct HasPoints {
};

template <bool Negated>
constexpr HasPoints<!Negated> operator!(HasPoints<Negated> const& p)
{
	return HasPoints<!Negated>();
}

template <bool Negated>
struct ValueCheck<HasPoints<Negated>> {
	using Pred = HasPoints<Negated>;

	template <class Map>
	static inline bool apply(Pred, Map const& m, Node n)
	{
		if constexpr (Negated) {
			return !m.hasPoints(n.index());
		} else {
			return m.hasPoints(n.index());
		}
	}
};

template <bool Negated>
struct InnerCheck<HasPoints<Negated>> {
	using Pred = HasPoints<Negated>;

	template <class Map>
	static inline bool apply(Pred, Map const& m, Node n)
	{
		switch (m.pointsPropagationCriteria()) {
			case PointsPropagationCriteria::NUM_POINTS: [[fallthrough]];
			case PointsPropagationCriteria::HAS_POINTS:
				if constexpr (Negated) {
					return true;
				} else {
					return m.hasPoints(n.index());
				}
			case PointsPropagationCriteria::NONE: return true;
		}
		return true;  // TODO: This should not be needed
	}
};

//
// Num points
//

template <PredicateCompare PC = PredicateCompare::EQUAL>
struct NumPoints {
	constexpr NumPoints(std::size_t count) : count(count) {}

	std::size_t count;
};

using NumPointsE  = NumPoints<>;
using NumPointsLE = NumPoints<PredicateCompare::LESS_EQUAL>;
using NumPointsGE = NumPoints<PredicateCompare::GREATER_EQUAL>;
using NumPointsL  = NumPoints<PredicateCompare::LESS>;
using NumPointsG  = NumPoints<PredicateCompare::GREATER>;

using NumPointsMin = NumPointsGE;
using NumPointsMax = NumPointsLE;

template <PredicateCompare PC>
struct ValueCheck<NumPoints<PC>> {
	using Pred = NumPoints<PC>;

	template <class Map>
	static constexpr bool apply(Pred p, Map const& m, Node n)
	{
		if constexpr (PredicateCompare::EQUAL == PC) {
			return m.numPoints(n.index()) == p.count;
		} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
			return m.numPoints(n.index()) <= p.count;
		} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
			return m.numPoints(n.index()) >= p.count;
		} else if constexpr (PredicateCompare::LESS == PC) {
			return m.numPoints(n.index()) < p.count;
		} else if constexpr (PredicateCompare::GREATER == PC) {
			return m.numPoints(n.index()) > p.count;
		}
	}
};

template <PredicateCompare PC>
struct InnerCheck<NumPoints<PC>> {
	using Pred = NumPoints<PC>;

	template <class Map>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		switch (m.pointsPropagationCriteria()) {
			case PointsPropagationCriteria::NUM_POINTS:
				if constexpr (PredicateCompare::EQUAL == PC) {
					return m.numPoints(n.index()) >= p.count;
				} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
					return true;
				} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
					return m.numPoints(n.index()) >= p.count;
				} else if constexpr (PredicateCompare::LESS == PC) {
					return true;
				} else if constexpr (PredicateCompare::GREATER == PC) {
					return m.numPoints(n.index()) > p.count;
				}
			case PointsPropagationCriteria::HAS_POINTS: [[fallthrough]];
			case PointsPropagationCriteria::NONE: return true;
		}
	}
};

//
// Num points interval
//

template <bool Negated = false>
struct NumPointsInterval {
	constexpr NumPointsInterval(std::size_t min, std::size_t max) : min(min), max(max) {}

	std::size_t min;
	std::size_t max;
};

template <bool Negated>
constexpr NumPointsInterval<!Negated> operator!(NumPointsInterval<Negated> const& p)
{
	return NumPointsInterval<!Negated>(p.min, p.max);
}

template <bool Negated>
struct ValueCheck<NumPointsInterval<Negated>> {
	using Pred = NumPointsInterval<Negated>;

	template <class Map>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		auto t = m.numPoints(n.index());
		if constexpr (Negated) {
			return p.min > t || p.max < t;
		} else {
			return p.min <= t && p.max >= t;
		}
	}
};

template <bool Negated>
struct InnerCheck<NumPointsInterval<Negated>> {
	using Pred = NumPointsInterval<Negated>;

	template <class Map>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		switch (m.pointsPropagationCriteria()) {
			case PointsPropagationCriteria::NUM_POINTS: return m.numPoints(n.index()) >= p.min;
			case PointsPropagationCriteria::HAS_POINTS: [[fallthrough]];
			case PointsPropagationCriteria::NONE: return true;
		}
		return false;
	}
};

}  // namespace ufo::pred

#endif  // UFO_MAP_PREDICATE_POINTS_HPP