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

#ifndef UFO_MAP_TIME_PREDICATE_HPP
#define UFO_MAP_TIME_PREDICATE_HPP

// UFO
#include <ufo/map/node.hpp>
#include <ufo/map/predicate/predicate.hpp>
#include <ufo/map/time/time_map.hpp>
#include <ufo/map/types.hpp>

namespace ufo::pred
{
//
// Predicates
//

// REVIEW: Name?
struct TimeMap {
};

template <PredicateCompare PC = PredicateCompare::EQUAL>
struct Time {
	constexpr Time(time_t time) : time(time) {}

	time_t time;
};

using TimeE  = Time<>;
using TimeLE = Time<PredicateCompare::LESS_EQUAL>;
using TimeGE = Time<PredicateCompare::GREATER_EQUAL>;
using TimeL  = Time<PredicateCompare::LESS>;
using TimeG  = Time<PredicateCompare::GREATER>;

using TimeMin = TimeGE;
using TimeMax = TimeLE;

template <bool Negated = false>
struct TimeInterval {
	constexpr TimeInterval(time_t min, time_t max) : min(min), max(max) {}

	time_t min;
	time_t max;
};

template <bool Negated>
constexpr TimeInterval<!Negated> operator!(TimeInterval<Negated> const& p)
{
	return TimeInterval<!Negated>(p.min, p.max);
}

//
// Predicate value/return check
//

template <>
struct ValueCheck<TimeMap> {
	using Pred = TimeMap;

	template <class Map>
	static constexpr bool apply(Pred, Map const&, Node)
	{
		return IsTimeMap<Map>;
	}
};

template <class PredPost>
struct ValueCheck<THEN<TimeMap, PredPost>> {
	using Pred = THEN<TimeMap, PredPost>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if constexpr (ValueCheck<TimeMap>::apply(p.pre, m, n)) {
			return ValueCheck<PredPost>::apply(p.post, m, n);
		} else {
			return true;
		}
	}
};

template <PredicateCompare PC>
struct ValueCheck<Time<PC>> {
	using Pred = Time<PC>;

	template <class Map>
	static constexpr bool apply(Pred p, Map const& m, Node n)
	{
		if constexpr (PredicateCompare::EQUAL == PC) {
			return m.time(n.index()) == p.time;
		} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
			return m.time(n.index()) <= p.time;
		} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
			return m.time(n.index()) >= p.time;
		} else if constexpr (PredicateCompare::LESS == PC) {
			return m.time(n.index()) < p.time;
		} else if constexpr (PredicateCompare::GREATER == PC) {
			return m.time(n.index()) > p.time;
		}
	}
};

template <bool Negated>
struct ValueCheck<TimeInterval<Negated>> {
	using Pred = TimeInterval<Negated>;

	template <class Map>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		auto t = m.time(n.index());
		if constexpr (Negated) {
			return p.min > t || p.max < t;
		} else {
			return p.min <= t && p.max >= t;
		}
	}
};

//
// Predicate inner check
//

template <>
struct InnerCheck<TimeMap> {
	using Pred = TimeMap;

	template <class Map>
	static constexpr bool apply(Pred, Map const&, Node)
	{
		return IsTimeMap<Map>;
	}
};

template <class PredPost>
struct InnerCheck<THEN<TimeMap, PredPost>> {
	using Pred = THEN<TimeMap, PredPost>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if constexpr (InnerCheck<TimeMap>::apply(p.pre, m, n)) {
			return InnerCheck<PredPost>::apply(p.post, m, n);
		} else {
			return true;
		}
	}
};

template <PredicateCompare PC>
struct InnerCheck<Time<PC>> {
	using Pred = Time<PC>;

	template <class Map>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		switch (m.timePropagationCriteria()) {
			case PropagationCriteria::MIN:
				if constexpr (PredicateCompare::EQUAL == PC) {
					return m.time(n.index()) <= p.time;
				} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
					return m.time(n.index()) <= p.time;
				} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
					return true;
				} else if constexpr (PredicateCompare::LESS == PC) {
					return m.time(n.index()) < p.time;
				} else if constexpr (PredicateCompare::GREATER == PC) {
					return true;
				}
			case PropagationCriteria::MAX:
				if constexpr (PredicateCompare::EQUAL == PC) {
					return m.time(n.index()) >= p.time;
				} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
					return true;
				} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
					return m.time(n.index()) >= p.time;
				} else if constexpr (PredicateCompare::LESS == PC) {
					return true;
				} else if constexpr (PredicateCompare::GREATER == PC) {
					return m.time(n.index()) > p.time;
				}
			default: return true;  // FIXME: Can this be better?
		}
	}
};

template <bool Negated>
struct InnerCheck<TimeInterval<Negated>> {
	using Pred = TimeInterval<Negated>;

	template <class Map>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		if constexpr (Negated) {
			return true;  // FIXME: Can this be better?
		} else {
			switch (m.timePropagationCriteria()) {
				case PropagationCriteria::MIN: return m.time(n.index()) <= p.max;
				case PropagationCriteria::MAX: return m.time(n.index()) >= p.min;
				default: return true;
			}
		}
	}
};

}  // namespace ufo::pred

#endif  // UFO_MAP_TIME_PREDICATE_HPP