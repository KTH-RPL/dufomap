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

#ifndef UFO_MAP_PREDICATE_OCCUPANCY_HPP
#define UFO_MAP_PREDICATE_OCCUPANCY_HPP

// UFO
#include <ufo/map/node.hpp>
#include <ufo/map/occupancy/occupancy_map.hpp>
#include <ufo/map/predicate/predicate.hpp>
#include <ufo/map/types.hpp>
#include <ufo/util/type_traits.hpp>

// STL
#include <functional>

namespace ufo::pred
{
//
// Predicates
//

// REVIEW: Name?
struct OccupancyMap {
};

template <ufo::OccupancyState state>
struct OccupancyState {
};

using OccUnknown  = OccupancyState<ufo::OccupancyState::UNKNOWN>;
using OccFree     = OccupancyState<ufo::OccupancyState::FREE>;
using OccOccupied = OccupancyState<ufo::OccupancyState::OCCUPIED>;

struct OccupancyStates {
	OccupancyStates(bool unknown, bool free, bool occupied)
	    : unknown(unknown), free(free), occupied(occupied)
	{
	}

	bool unknown;
	bool free;
	bool occupied;
};

template <ufo::OccupancyState state>
struct ContainOccupancyState {
};

using ContainUnknown  = ContainOccupancyState<ufo::OccupancyState::UNKNOWN>;
using ContainFree     = ContainOccupancyState<ufo::OccupancyState::FREE>;
using ContainOccupied = ContainOccupancyState<ufo::OccupancyState::OCCUPIED>;

struct ContainOccupancyStates {
	ContainOccupancyStates(bool unknown, bool free, bool occupied)
	    : unknown(unknown), free(free), occupied(occupied)
	{
	}

	bool unknown;
	bool free;
	bool occupied;
};

template <PredicateCompare PC = PredicateCompare::EQUAL>
struct Occupancy {
	Occupancy(double occupancy) : occupancy(occupancy) {}

	constexpr void setOccupancy(double new_occupancy)
	{
		occupancy_modified = occupancy_modified || occupancy != new_occupancy;
		occupancy          = new_occupancy;
	}

	constexpr double getOccupancy() const { return occupancy; }

 private:
	double          occupancy;
	mutable float   logit_f;
	mutable uint8_t logit_u8;
	mutable bool    occupancy_modified = true;

	friend struct ValueCheck<Occupancy<PC>>;
	friend struct InnerCheck<Occupancy<PC>>;
};

using OccupancyE  = Occupancy<>;
using OccupancyLE = Occupancy<PredicateCompare::LESS_EQUAL>;
using OccupancyGE = Occupancy<PredicateCompare::GREATER_EQUAL>;
using OccupancyL  = Occupancy<PredicateCompare::LESS>;
using OccupancyG  = Occupancy<PredicateCompare::GREATER>;

using OccupancyMin = OccupancyGE;
using OccupancyMax = OccupancyLE;

struct OccupancyInterval {
	OccupancyInterval(double min, double max) : min(min), max(max) {}

	OccupancyMin min;
	OccupancyMax max;
};

//
// Predicate value/return check
//

template <>
struct ValueCheck<OccupancyMap> {
	using Pred = OccupancyMap;

	template <class Map>
	static constexpr bool apply(Pred, Map const&, Node)
	{
		return IsOccupancyMap<Map>;
	}
};

template <class PredPost>
struct ValueCheck<THEN<OccupancyMap, PredPost>> {
	using Pred = THEN<OccupancyMap, PredPost>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if constexpr (ValueCheck<OccupancyMap>::apply(p.pre, m, n)) {
			return ValueCheck<PredPost>::apply(p.post, m, n);
		} else {
			return true;
		}
	}
};

template <ufo::OccupancyState State>
struct ValueCheck<OccupancyState<State>> {
	using Pred = OccupancyState<State>;

	template <class Map>
	static inline bool apply(Pred, Map const& m, Node n)
	{
		if constexpr (ufo::OccupancyState::UNKNOWN == State) {
			return m.isUnknown(n.index());
		} else if constexpr (ufo::OccupancyState::FREE == State) {
			return m.isFree(n.index());
		} else if constexpr (ufo::OccupancyState::OCCUPIED == State) {
			return m.isOccupied(n.index());
		}
	}
};

template <>
struct ValueCheck<OccupancyStates> {
	using Pred = OccupancyStates;

	template <class Map>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		return (p.unknown && m.isUnknown(n.index())) || (p.free && m.isFree(n.index())) ||
		       (p.occupied && m.isOccupied(n.index()));
	}
};

template <ufo::OccupancyState State>
struct ValueCheck<ContainOccupancyState<State>> {
	using Pred = ContainOccupancyState<State>;

	template <class Map>
	static inline bool apply(Pred, Map const& m, Node n)
	{
		if constexpr (ufo::OccupancyState::UNKNOWN == State) {
			return m.containsUnknown(n.index());
		} else if constexpr (ufo::OccupancyState::FREE == State) {
			return m.containsFree(n.index());
		} else if constexpr (ufo::OccupancyState::OCCUPIED == State) {
			return m.containsOccupied(n.index());
		}
	}
};

template <>
struct ValueCheck<ContainOccupancyStates> {
	using Pred = ContainOccupancyStates;

	template <class Map>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		return (p.unknown && m.containsUnknown(n.index())) ||
		       (p.free && m.containsFree(n.index())) ||
		       (p.occupied && m.containsOccupied(n.index()));
	}
};

template <PredicateCompare PC>
struct ValueCheck<Occupancy<PC>> {
	using Pred = Occupancy<PC>;

	template <class Map>
	static inline bool apply(Pred const& p, Map const& m, Node n)
	{
		if (p.occupancy_modified) {
			if constexpr (std::is_same_v<uint8_t, typename ufo::logit_t>) {
				p.logit_u8 = m.toOccupancyLogit(p.occupancy);
			} else {
				p.logit_f = m.toOccupancyLogit(p.occupancy);
			}
			p.occupancy_modified = false;
		}

		if constexpr (std::is_same_v<uint8_t, typename ufo::logit_t>) {
			if constexpr (PredicateCompare::EQUAL == PC) {
				return m.getOccupancyLogit(n.index()) == p.logit_u8;
			} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
				return m.getOccupancyLogit(n.index()) <= p.logit_u8;
			} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
				return m.getOccupancyLogit(n.index()) >= p.logit_u8;
			} else if constexpr (PredicateCompare::LESS == PC) {
				return m.getOccupancyLogit(n.index()) < p.logit_u8;
			} else if constexpr (PredicateCompare::GREATER == PC) {
				return m.getOccupancyLogit(n.index()) > p.logit_u8;
			}
		} else {
			if constexpr (PredicateCompare::EQUAL == PC) {
				return m.getOccupancyLogit(n.index()) == p.logit_f;
			} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
				return m.getOccupancyLogit(n.index()) <= p.logit_f;
			} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
				return m.getOccupancyLogit(n.index()) >= p.logit_f;
			} else if constexpr (PredicateCompare::LESS == PC) {
				return m.getOccupancyLogit(n.index()) < p.logit_f;
			} else if constexpr (PredicateCompare::GREATER == PC) {
				return m.getOccupancyLogit(n.index()) > p.logit_f;
			}
		}
	}
};

template <>
struct ValueCheck<OccupancyInterval> {
	using Pred = OccupancyInterval;

	template <class Map>
	static inline bool apply(Pred const& p, Map const& m, Node n)
	{
		return ValueCheck<std::decay_t<decltype(p.min)>>::apply(p.min, m, n) &&
		       ValueCheck<std::decay_t<decltype(p.max)>>::apply(p.max, m, n);
	}
};

//
// Predicate inner check
//

template <>
struct InnerCheck<OccupancyMap> {
	using Pred = OccupancyMap;

	template <class Map>
	static constexpr bool apply(Pred, Map const&, Node)
	{
		return IsOccupancyMap<Map>;
	}
};

template <class PredPost>
struct InnerCheck<THEN<OccupancyMap, PredPost>> {
	using Pred = THEN<OccupancyMap, PredPost>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		if constexpr (InnerCheck<OccupancyMap>::apply(p.pre, m, n)) {
			return InnerCheck<PredPost>::apply(p.post, m, n);
		} else {
			return true;
		}
	}
};

template <ufo::OccupancyState State>
struct InnerCheck<OccupancyState<State>> {
	using Pred = OccupancyState<State>;

	template <class Map>
	static inline bool apply(Pred, Map const& m, Node n)
	{
		if constexpr (ufo::OccupancyState::UNKNOWN == State) {
			return m.containsUnknown(n.index());
		} else if constexpr (ufo::OccupancyState::FREE == State) {
			return m.containsFree(n.index());
		} else if constexpr (ufo::OccupancyState::OCCUPIED == State) {
			return m.containsOccupied(n.index());
		}
	}
};

template <>
struct InnerCheck<OccupancyStates> {
	using Pred = OccupancyStates;

	template <class Map>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		return (p.unknown && m.containsUnknown(n.index())) ||
		       (p.free && m.containsFree(n.index())) ||
		       (p.occupied && m.containsOccupied(n.index()));
	}
};

template <ufo::OccupancyState state>
struct InnerCheck<ContainOccupancyState<state>> {
	using Pred = ContainOccupancyState<state>;

	template <class Map>
	static inline bool apply(Pred, Map const& m, Node n)
	{
		if constexpr (ufo::OccupancyState::UNKNOWN == state) {
			return m.containsUnknown(n.index());
		} else if constexpr (ufo::OccupancyState::FREE == state) {
			return m.containsFree(n.index());
		} else if constexpr (ufo::OccupancyState::OCCUPIED == state) {
			return m.containsOccupied(n.index());
		}
	}
};

template <>
struct InnerCheck<ContainOccupancyStates> {
	using Pred = ContainOccupancyStates;

	template <class Map>
	static inline bool apply(Pred p, Map const& m, Node n)
	{
		return (p.unknown && m.containsUnknown(n.index())) ||
		       (p.free && m.containsFree(n.index())) ||
		       (p.occupied && m.containsOccupied(n.index()));
	}
};

template <PredicateCompare PC>
struct InnerCheck<Occupancy<PC>> {
	using Pred = Occupancy<PC>;

	template <class Map>
	static inline bool apply(Pred const& p, Map const& m, Node n)
	{
		if (p.occupancy_modified) {
			if constexpr (std::is_same_v<uint8_t, typename ufo::logit_t>) {
				p.logit_u8 = m.toOccupancyLogit(p.occupancy);
			} else {
				p.logit_f = m.toOccupancyLogit(p.occupancy);
			}
			p.occupancy_modified = false;
		}

		// FIXME: Check how occupancy is propagated to determine

		if constexpr (std::is_same_v<uint8_t, typename ufo::logit_t>) {
			if constexpr (PredicateCompare::EQUAL == PC) {
				return m.getOccupancyLogit(n.index()) >= p.logit_u8;
			} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
				return true;
			} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
				return m.getOccupancyLogit(n.index()) >= p.logit_u8;
			} else if constexpr (PredicateCompare::LESS == PC) {
				return true;
			} else if constexpr (PredicateCompare::GREATER == PC) {
				return m.getOccupancyLogit(n.index()) > p.logit_u8;
			}
		} else {
			if constexpr (PredicateCompare::EQUAL == PC) {
				return m.getOccupancyLogit(n.index()) >= p.logit_f;
			} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
				return true;
			} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
				return m.getOccupancyLogit(n.index()) >= p.logit_f;
			} else if constexpr (PredicateCompare::LESS == PC) {
				return true;
			} else if constexpr (PredicateCompare::GREATER == PC) {
				return m.getOccupancyLogit(n.index()) > p.logit_f;
			}
		}
	}
};

template <>
struct InnerCheck<OccupancyInterval> {
	using Pred = OccupancyInterval;

	template <class Map>
	static inline bool apply(Pred const& p, Map const& m, Node n)
	{
		return InnerCheck<std::decay_t<decltype(p.min)>>::apply(p.min, m, n) &&
		       InnerCheck<std::decay_t<decltype(p.max)>>::apply(p.max, m, n);
	}
};
}  // namespace ufo::pred

#endif  // UFO_MAP_PREDICATE_OCCUPANCY_HPP