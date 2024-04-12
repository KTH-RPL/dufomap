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

#ifndef UFO_MAP_PREDICATE_COUNT_HPP
#define UFO_MAP_PREDICATE_COUNT_HPP

// UFO
#include <ufo/map/count/count_map.hpp>
#include <ufo/map/node.hpp>
#include <ufo/map/predicate/predicate.hpp>

namespace ufo::pred
{
//
// Count map
//

struct CountMap {
};

template <>
struct ValueCheck<CountMap> {
	using Pred = CountMap;

	template <class Map>
	static constexpr bool apply(Pred, Map const&, Node)
	{
		return IsCountMap<Map>;
	}
};

template <class PredPost>
struct ValueCheck<THEN<CountMap, PredPost>> {
	using Pred = THEN<CountMap, PredPost>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if constexpr (ValueCheck<CountMap>::apply(p.pre, m, n)) {
			return ValueCheck<PredPost>::apply(p.post, m, n);
		} else {
			return true;
		}
	}
};

template <>
struct InnerCheck<CountMap> {
	using Pred = CountMap;

	template <class Map>
	static constexpr bool apply(Pred, Map const&, Node)
	{
		return IsCountMap<Map>;
	}
};

template <class PredPost>
struct InnerCheck<THEN<CountMap, PredPost>> {
	using Pred = THEN<CountMap, PredPost>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if constexpr (InnerCheck<CountMap>::apply(p.pre, m, n)) {
			return InnerCheck<PredPost>::apply(p.post, m, n);
		} else {
			return true;
		}
	}
};

// //
// // Count
// //

// template <PredicateCompare PC = PredicateCompare::EQUAL>
// struct Count {
// 	constexpr Count(std::size_t count) : count(count) {}

// 	std::size_t count;
// };

// using CountE  = Count<>;
// using CountLE = Count<PredicateCompare::LESS_EQUAL>;
// using CountGE = Count<PredicateCompare::GREATER_EQUAL>;
// using CountL  = Count<PredicateCompare::LESS>;
// using CountG  = Count<PredicateCompare::GREATER>;

// using CountMin = CountGE;
// using CountMax = CountLE;

// template <PredicateCompare PC>
// struct ValueCheck<Count<PC>> {
// 	using Pred = Count<PC>;

// 	template <class Map>
// 	static constexpr bool apply(Pred p, Map const& m, Node n)
// 	{
// 		if constexpr (PredicateCompare::EQUAL == PC) {
// 			return m.Count(n.index()) == p.count;
// 		} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
// 			return m.Count(n.index()) <= p.count;
// 		} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
// 			return m.Count(n.index()) >= p.count;
// 		} else if constexpr (PredicateCompare::LESS == PC) {
// 			return m.Count(n.index()) < p.count;
// 		} else if constexpr (PredicateCompare::GREATER == PC) {
// 			return m.Count(n.index()) > p.count;
// 		}
// 	}
// };

// template <PredicateCompare PC>
// struct InnerCheck<Count<PC>> {
// 	using Pred = Count<PC>;

// 	template <class Map>
// 	static inline bool apply(Pred p, Map const& m, Node n)
// 	{
// 		switch (m.PropagationCriteria()) {
// 			case PropagationCriteria::NUM_POINTS:
// 				if constexpr (PredicateCompare::EQUAL == PC) {
// 					return m.Count(n.index()) >= p.count;
// 				} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
// 					return true;
// 				} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
// 					return m.Count(n.index()) >= p.count;
// 				} else if constexpr (PredicateCompare::LESS == PC) {
// 					return true;
// 				} else if constexpr (PredicateCompare::GREATER == PC) {
// 					return m.Count(n.index()) > p.count;
// 				}
// 			case PropagationCriteria::HAS_POINTS: [[fallthrough]];
// 			case PropagationCriteria::NONE: return true;
// 		}
// 	}
// };

// //
// // Num points interval
// //

// template <bool Negated = false>
// struct CountInterval {
// 	constexpr CountInterval(std::size_t min, std::size_t max) : min(min), max(max) {}

// 	std::size_t min;
// 	std::size_t max;
// };

// template <bool Negated>
// constexpr CountInterval<!Negated> operator!(CountInterval<Negated> const& p)
// {
// 	return CountInterval<!Negated>(p.min, p.max);
// }

// template <bool Negated>
// struct ValueCheck<CountInterval<Negated>> {
// 	using Pred = CountInterval<Negated>;

// 	template <class Map>
// 	static inline bool apply(Pred p, Map const& m, Node n)
// 	{
// 		auto t = m.Count(n.index());
// 		if constexpr (Negated) {
// 			return p.min > t || p.max < t;
// 		} else {
// 			return p.min <= t && p.max >= t;
// 		}
// 	}
// };

// template <bool Negated>
// struct InnerCheck<CountInterval<Negated>> {
// 	using Pred = CountInterval<Negated>;

// 	template <class Map>
// 	static inline bool apply(Pred p, Map const& m, Node n)
// 	{
// 		switch (m.PropagationCriteria()) {
// 			case PropagationCriteria::MAX: return m.Count(n.index()) >= p.min;
// 			case PropagationCriteria::MIN: [[fallthrough]];
// 			case PropagationCriteria::NONE: return true;
// 		}
// 		return false;
// 	}
// };

}  // namespace ufo::pred

#endif  // UFO_MAP_PREDICATE_COUNT_HPP