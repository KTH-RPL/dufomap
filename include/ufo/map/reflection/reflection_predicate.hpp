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

#ifndef UFO_MAP_REFLECTION_PREDICATE_HPP
#define UFO_MAP_REFLECTION_PREDICATE_HPP

// UFO
#include <ufo/map/node.hpp>
#include <ufo/map/predicate/predicate.hpp>
#include <ufo/map/reflection/reflection.hpp>
#include <ufo/map/reflection/reflection_map.hpp>
#include <ufo/map/types.hpp>

namespace ufo::pred
{

//
// ReflectionMap
//

struct ReflectionMap {
};

template <>
struct ValueCheck<ReflectionMap> {
	using Pred = ReflectionMap;

	template <class Map>
	static constexpr bool apply(Pred, Map const&, Node)
	{
		return IsReflectionMap<Map>;
	}
};

template <class PredPost>
struct ValueCheck<THEN<ReflectionMap, PredPost>> {
	using Pred = THEN<ReflectionMap, PredPost>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if constexpr (ValueCheck<ReflectionMap>::apply(p.pre, m, n)) {
			return ValueCheck<PredPost>::apply(p.post, m, n);
		} else {
			return true;
		}
	}
};

template <>
struct InnerCheck<ReflectionMap> {
	using Pred = ReflectionMap;

	template <class Map>
	static constexpr bool apply(Pred, Map const&, Node)
	{
		return IsReflectionMap<Map>;
	}
};

template <class PredPost>
struct InnerCheck<THEN<ReflectionMap, PredPost>> {
	using Pred = THEN<ReflectionMap, PredPost>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if constexpr (InnerCheck<ReflectionMap>::apply(p.pre, m, n)) {
			return InnerCheck<PredPost>::apply(p.post, m, n);
		} else {
			return true;
		}
	}
};

//
// Hits
//

template <PredicateCompare PC = PredicateCompare::EQUAL>
struct Hits {
	constexpr Hits(count_t hits) : hits(hits) {}

	count_t hits;
};

using HitsE  = Hits<>;
using HitsLE = Hits<PredicateCompare::LESS_EQUAL>;
using HitsGE = Hits<PredicateCompare::GREATER_EQUAL>;
using HitsL  = Hits<PredicateCompare::LESS>;
using HitsG  = Hits<PredicateCompare::GREATER>;

using HitsMin = HitsGE;
using HitsMax = HitsLE;

template <PredicateCompare PC>
struct ValueCheck<Hits<PC>> {
	using Pred = Hits<PC>;

	template <class Map>
	static constexpr bool apply(Pred p, Map const& m, Node n)
	{
		if constexpr (PredicateCompare::EQUAL == PC) {
			return m.hits(n.index()) == p.hits;
		} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
			return m.hits(n.index()) <= p.hits;
		} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
			return m.hits(n.index()) >= p.hits;
		} else if constexpr (PredicateCompare::LESS == PC) {
			return m.hits(n.index()) < p.hits;
		} else if constexpr (PredicateCompare::GREATER == PC) {
			return m.hits(n.index()) > p.hits;
		}
	}
};

template <PredicateCompare PC>
struct InnerCheck<Hits<PC>> {
	using Pred = Hits<PC>;

	template <class Map>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		switch (m.reflectionPropagationCriteria()) {
			case PropagationCriteria::MIN:
				if constexpr (PredicateCompare::EQUAL == PC) {
					return m.hits(n.index()) <= p.hits;
				} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
					return m.hits(n.index()) <= p.hits;
				} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
					return true;
				} else if constexpr (PredicateCompare::LESS == PC) {
					return m.hits(n.index()) < p.hits;
				} else if constexpr (PredicateCompare::GREATER == PC) {
					return true;
				}
			case PropagationCriteria::MAX:
				if constexpr (PredicateCompare::EQUAL == PC) {
					return m.hits(n.index()) >= p.hits;
				} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
					return true;
				} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
					return m.hits(n.index()) >= p.hits;
				} else if constexpr (PredicateCompare::LESS == PC) {
					return true;
				} else if constexpr (PredicateCompare::GREATER == PC) {
					return m.hits(n.index()) > p.hits;
				}
			default: return true;  // FIXME: Can this be better?
		}
	}
};

//
// Hits interval
//

template <bool Negated = false>
struct HitsInterval {
	constexpr HitsInterval(count_t min, count_t max) : min(min), max(max) {}

	count_t min;
	count_t max;
};

template <bool Negated>
constexpr HitsInterval<!Negated> operator!(HitsInterval<Negated> const& p)
{
	return HitsInterval<!Negated>(p.min, p.max);
}

template <bool Negated>
struct ValueCheck<HitsInterval<Negated>> {
	using Pred = HitsInterval<Negated>;

	template <class Map>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		auto t = m.hits(n.index());
		if constexpr (Negated) {
			return p.min > t || p.max < t;
		} else {
			return p.min <= t && p.max >= t;
		}
	}
};

template <bool Negated>
struct InnerCheck<HitsInterval<Negated>> {
	using Pred = HitsInterval<Negated>;

	template <class Map>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		if constexpr (Negated) {
			return true;  // FIXME: Can this be better?
		} else {
			switch (m.reflectionPropagationCriteria()) {
				case PropagationCriteria::MIN: return m.hits(n.index()) <= p.max;
				case PropagationCriteria::MAX: return m.hits(n.index()) >= p.min;
				default: return true;
			}
		}
	}
};

//
// Misses
//

template <PredicateCompare PC = PredicateCompare::EQUAL>
struct Misses {
	constexpr Misses(count_t misses) : misses(misses) {}

	count_t misses;
};

using MissesE  = Misses<>;
using MissesLE = Misses<PredicateCompare::LESS_EQUAL>;
using MissesGE = Misses<PredicateCompare::GREATER_EQUAL>;
using MissesL  = Misses<PredicateCompare::LESS>;
using MissesG  = Misses<PredicateCompare::GREATER>;

using MissesMin = MissesGE;
using MissesMax = MissesLE;

template <PredicateCompare PC>
struct ValueCheck<Misses<PC>> {
	using Pred = Misses<PC>;

	template <class Map>
	static constexpr bool apply(Pred p, Map const& m, Node n)
	{
		if constexpr (PredicateCompare::EQUAL == PC) {
			return m.misses(n.index()) == p.misses;
		} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
			return m.misses(n.index()) <= p.misses;
		} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
			return m.misses(n.index()) >= p.misses;
		} else if constexpr (PredicateCompare::LESS == PC) {
			return m.misses(n.index()) < p.misses;
		} else if constexpr (PredicateCompare::GREATER == PC) {
			return m.misses(n.index()) > p.misses;
		}
	}
};

template <PredicateCompare PC>
struct InnerCheck<Misses<PC>> {
	using Pred = Misses<PC>;

	template <class Map>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		switch (m.reflectionPropagationCriteria()) {
			case PropagationCriteria::MIN:
				if constexpr (PredicateCompare::EQUAL == PC) {
					return m.misses(n.index()) <= p.misses;
				} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
					return m.misses(n.index()) <= p.misses;
				} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
					return true;
				} else if constexpr (PredicateCompare::LESS == PC) {
					return m.misses(n.index()) < p.misses;
				} else if constexpr (PredicateCompare::GREATER == PC) {
					return true;
				}
			case PropagationCriteria::MAX:
				if constexpr (PredicateCompare::EQUAL == PC) {
					return m.misses(n.index()) >= p.misses;
				} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
					return true;
				} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
					return m.misses(n.index()) >= p.misses;
				} else if constexpr (PredicateCompare::LESS == PC) {
					return true;
				} else if constexpr (PredicateCompare::GREATER == PC) {
					return m.misses(n.index()) > p.misses;
				}
			default: return true;  // FIXME: Can this be better?
		}
	}
};

//
// Misses interval
//

template <bool Negated = false>
struct MissesInterval {
	constexpr MissesInterval(count_t min, count_t max) : min(min), max(max) {}

	count_t min;
	count_t max;
};

template <bool Negated>
constexpr MissesInterval<!Negated> operator!(MissesInterval<Negated> const& p)
{
	return MissesInterval<!Negated>(p.min, p.max);
}

template <bool Negated>
struct ValueCheck<MissesInterval<Negated>> {
	using Pred = MissesInterval<Negated>;

	template <class Map>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		auto t = m.misses(n.index());
		if constexpr (Negated) {
			return p.min > t || p.max < t;
		} else {
			return p.min <= t && p.max >= t;
		}
	}
};

template <bool Negated>
struct InnerCheck<MissesInterval<Negated>> {
	using Pred = MissesInterval<Negated>;

	template <class Map>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		if constexpr (Negated) {
			return true;  // FIXME: Can this be better?
		} else {
			switch (m.reflectionPropagationCriteria()) {
				case PropagationCriteria::MIN: return m.misses(n.index()) <= p.max;
				case PropagationCriteria::MAX: return m.misses(n.index()) >= p.min;
				default: return true;
			}
		}
	}
};

//
// Reflectiveness
//

template <PredicateCompare PC = PredicateCompare::EQUAL>
struct Reflectiveness {
	constexpr Reflectiveness(reflection_t reflectiveness) : reflectiveness(reflectiveness)
	{
	}

	reflection_t reflectiveness;
};

using ReflectivenessE  = Reflectiveness<>;
using ReflectivenessLE = Reflectiveness<PredicateCompare::LESS_EQUAL>;
using ReflectivenessGE = Reflectiveness<PredicateCompare::GREATER_EQUAL>;
using ReflectivenessL  = Reflectiveness<PredicateCompare::LESS>;
using ReflectivenessG  = Reflectiveness<PredicateCompare::GREATER>;

using ReflectivenessMin = ReflectivenessGE;
using ReflectivenessMax = ReflectivenessLE;

template <PredicateCompare PC>
struct ValueCheck<Reflectiveness<PC>> {
	using Pred = Reflectiveness<PC>;

	template <class Map>
	static constexpr bool apply(Pred p, Map const& m, Node n)
	{
		if constexpr (PredicateCompare::EQUAL == PC) {
			return m.reflectiveness(n.index()) == p.reflectiveness;
		} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
			return m.reflectiveness(n.index()) <= p.reflectiveness;
		} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
			return m.reflectiveness(n.index()) >= p.reflectiveness;
		} else if constexpr (PredicateCompare::LESS == PC) {
			return m.reflectiveness(n.index()) < p.reflectiveness;
		} else if constexpr (PredicateCompare::GREATER == PC) {
			return m.reflectiveness(n.index()) > p.reflectiveness;
		}
	}
};

template <PredicateCompare PC>
struct InnerCheck<Reflectiveness<PC>> {
	using Pred = Reflectiveness<PC>;

	template <class Map>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		// TODO: Implement
		return true;
	}
};

//
// Reflectiveness interval
//

template <bool Negated = false>
struct ReflectivenessInterval {
	constexpr ReflectivenessInterval(reflection_t min, reflection_t max)
	    : min(min), max(max)
	{
	}

	reflection_t min;
	reflection_t max;
};

template <bool Negated>
constexpr ReflectivenessInterval<!Negated> operator!(
    ReflectivenessInterval<Negated> const& p)
{
	return ReflectivenessInterval<!Negated>(p.min, p.max);
}

template <bool Negated>
struct ValueCheck<ReflectivenessInterval<Negated>> {
	using Pred = ReflectivenessInterval<Negated>;

	template <class Map>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		auto t = m.reflectiveness(n.index());
		if constexpr (Negated) {
			return p.min > t || p.max < t;
		} else {
			return p.min <= t && p.max >= t;
		}
	}
};

template <bool Negated>
struct InnerCheck<ReflectivenessInterval<Negated>> {
	using Pred = ReflectivenessInterval<Negated>;

	template <class Map>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		// TODO: Implement
		return true;
	}
};
}  // namespace ufo::pred

#endif  // UFO_MAP_REFLECTION_PREDICATE_HPP