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

#ifndef UFO_MAP_PREDICATE_OCTREE_HPP
#define UFO_MAP_PREDICATE_OCTREE_HPP

// UFO
#include <ufo/map/code.hpp>
#include <ufo/map/node.hpp>
#include <ufo/map/predicate/predicate.hpp>
#include <ufo/map/types.hpp>

// STL
#include <cmath>
#include <functional>

namespace ufo::pred
{
//
// Predicates
//

struct PureLeaf {
};

struct Leaf {
	constexpr Leaf(depth_t min_depth = 0) : min_depth(min_depth) {}

	// Depth to consider as leaf
	depth_t min_depth;
};

struct Inner {
};

struct Parent {
};

struct Exists {
};

template <PredicateCompare PC = PredicateCompare::EQUAL,
          PredicateType    PT = PredicateType::RETURN_AND_INNER>
struct Depth {
	constexpr Depth(depth_t depth) noexcept : depth(depth) {}

	depth_t depth;
};

using DepthE  = Depth<PredicateCompare::EQUAL>;
using DepthLE = Depth<PredicateCompare::LESS_EQUAL>;
using DepthGE = Depth<PredicateCompare::GREATER_EQUAL>;
using DepthL  = Depth<PredicateCompare::LESS>;
using DepthG  = Depth<PredicateCompare::GREATER>;

using DepthMin = DepthGE;
using DepthMax = DepthLE;

struct DepthInterval {
	constexpr DepthInterval(depth_t min, depth_t max) noexcept : min(min), max(max) {}

	DepthMin min;
	DepthMax max;
};

template <PredicateRounding PR, PredicateCompare PC = PredicateCompare::EQUAL,
          PredicateType PT = PredicateType::RETURN_AND_INNER>
struct Size {
	constexpr Size(double size) noexcept : size_(size), depth_(0) {}

	constexpr void setSize(double size) noexcept
	{
		depth_modified_ = depth_modified_ || size != size_;
		size_           = size;
	}

	constexpr double size() const noexcept { return size_; }

 private:
	double                size_;
	mutable Depth<PC, PT> depth_;
	mutable bool          depth_modified_ = true;

	friend struct ValueCheck<Size<PR, PC, PT>>;
	friend struct InnerCheck<Size<PR, PC, PT>>;
};

template <PredicateCompare PC = PredicateCompare::EQUAL,
          PredicateType    PT = PredicateType::RETURN_AND_INNER>
using SizeNearest   = Size<PredicateRounding::NEAREST, PC, PT>;
using SizeENearest  = SizeNearest<>;
using SizeLENearest = SizeNearest<PredicateCompare::LESS_EQUAL>;
using SizeGENearest = SizeNearest<PredicateCompare::GREATER_EQUAL>;
using SizeLNearest  = SizeNearest<PredicateCompare::LESS>;
using SizeGNearest  = SizeNearest<PredicateCompare::GREATER>;

template <PredicateCompare PC = PredicateCompare::EQUAL,
          PredicateType    PT = PredicateType::RETURN_AND_INNER>
using SizeDownwards   = Size<PredicateRounding::FLOOR, PC, PT>;
using SizeEDownwards  = SizeDownwards<>;
using SizeLEDownwards = SizeDownwards<PredicateCompare::LESS_EQUAL>;
using SizeGEDownwards = SizeDownwards<PredicateCompare::GREATER_EQUAL>;
using SizeLDownwards  = SizeDownwards<PredicateCompare::LESS>;
using SizeGDownwards  = SizeDownwards<PredicateCompare::GREATER>;

template <PredicateCompare PC = PredicateCompare::EQUAL,
          PredicateType    PT = PredicateType::RETURN_AND_INNER>
using SizeUpwards   = Size<PredicateRounding::CEIL, PC, PT>;
using SizeEUpwards  = SizeUpwards<>;
using SizeLEUpwards = SizeUpwards<PredicateCompare::LESS_EQUAL>;
using SizeGEUpwards = SizeUpwards<PredicateCompare::GREATER_EQUAL>;
using SizeLUpwards  = SizeUpwards<PredicateCompare::LESS>;
using SizeGUpwards  = SizeUpwards<PredicateCompare::GREATER>;

using SizeMin = SizeGEUpwards;
using SizeMax = SizeLEDownwards;

struct SizeInterval {
	constexpr SizeInterval(double min, double max) noexcept : min(min), max(max) {}

	SizeMin min;
	SizeMax max;
};

struct Modified {
};

struct ChildOf {
	constexpr ChildOf(Node node) noexcept : code(node.code()) {}

	constexpr ChildOf(Code code) noexcept : code(code) {}

	ChildOf(Key key) noexcept : code(key) {}

	Code code;
};

//
// Predicate value/return check
//

template <>
struct ValueCheck<PureLeaf> {
	using Pred = PureLeaf;

	template <class Map>
	static constexpr bool apply(Pred, Map const& m, Node n)
	{
		return m.isPureLeaf(n.index());
	}
};

template <>
struct ValueCheck<Leaf> {
	using Pred = Leaf;

	template <class Map>
	static constexpr bool apply(Pred p, Map const& m, Node n)
	{
		return p.min_depth == n.depth() || m.isLeaf(n.index());
	}
};

template <>
struct ValueCheck<Inner> {
	using Pred = Inner;

	template <class Map>
	static constexpr bool apply(Pred, Map const&, Node n) noexcept
	{
		return 0 != n.depth();
	}
};

template <>
struct ValueCheck<Parent> {
	using Pred = Parent;

	template <class Map>
	static constexpr bool apply(Pred, Map const& m, Node n)
	{
		return m.isParent(n.index());
	}
};

template <>
struct ValueCheck<Exists> {
	using Pred = Exists;

	template <class Map>
	static constexpr bool apply(Pred, Map const& m, Node n)
	{
		return m.exists(n.index());  // TODO: Look at
	}
};

template <PredicateCompare PC, PredicateType PT>
struct ValueCheck<Depth<PC, PT>> {
	using Pred = Depth<PC, PT>;

	template <class Map>
	static constexpr bool apply(Pred p, Map const& m, Node n) noexcept
	{
		if constexpr (PredicateType::INNER == PT) {
			return false;
		} else if constexpr (PredicateCompare::EQUAL == PC) {
			return n.depth() == p.depth;
		} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
			return n.depth() <= p.depth;
		} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
			return n.depth() >= p.depth;
		} else if constexpr (PredicateCompare::LESS == PC) {
			return n.depth() < p.depth;
		} else if constexpr (PredicateCompare::GREATER == PC) {
			return n.depth() > p.depth;
		}
	}
};

template <>
struct ValueCheck<DepthInterval> {
	using Pred = DepthInterval;

	template <class Map>
	static constexpr bool apply(Pred p, Map const& m, Node n) noexcept
	{
		return ValueCheck<std::decay_t<decltype(p.min)>>::apply(p.min, m, n) &&
		       ValueCheck<std::decay_t<decltype(p.max)>>::apply(p.max, m, n);
	}
};

template <PredicateRounding PR, PredicateCompare PC, PredicateType PT>
struct ValueCheck<Size<PR, PC, PT>> {
	using Pred = Size<PR, PC, PT>;

	template <class Map>
	static constexpr bool apply(Pred const& p, Map const& m, Node n) noexcept
	{
		if (p.depth_modified) {
			double temp = std::max(0.0, (m.resolution() / p.resolution) + 1.0);
			if constexpr (PredicateRounding::NEAREST == PR) {
				p.depth.depth = std::round(temp);
			} else if constexpr (PredicateRounding::FLOOR == PR) {
				p.depth.depth = std::floor(temp);
			} else {
				p.depth.depth = std::ceil(temp);
			}
			p.depth_modified = false;
		}

		return ValueCheck<std::decay_t<decltype(p.depth)>>::apply(p.depth, m, n);
	}
};

template <>
struct ValueCheck<SizeInterval> {
	using Pred = SizeInterval;

	template <class Map>
	static constexpr bool apply(Pred const& p, Map const& m, Node n) noexcept
	{
		return ValueCheck<std::decay_t<decltype(p.min)>>::apply(p.min, m, n) &&
		       ValueCheck<std::decay_t<decltype(p.max)>>::apply(p.max, m, n);
	}
};

template <>
struct ValueCheck<Modified> {
	using Pred = Modified;

	template <class Map>
	static constexpr bool apply(Pred, Map const& m, Node n) noexcept
	{
		return m.isModified(n.index());
	}
};

template <>
struct ValueCheck<ChildOf> {
	using Pred = ChildOf;

	template <class Map>
	static constexpr bool apply(Pred p, Map const&, Node n) noexcept
	{
		return n.depth() < p.code.depth() &&
		       Code::equalAtDepth(n.code(), p.code, p.code.depth());
	}
};

//
// Predicate inner check
//

template <>
struct InnerCheck<PureLeaf> {
	using Pred = PureLeaf;

	template <class Map>
	static constexpr bool apply(Pred, Map const&, Node) noexcept
	{
		return true;
	}
};

template <>
struct InnerCheck<Leaf> {
	using Pred = Leaf;

	template <class Map>
	static constexpr bool apply(Pred p, Map const&, Node n) noexcept
	{
		return p.min_depth < n.depth();
	}
};

template <>
struct InnerCheck<Inner> {
	using Pred = Inner;

	template <class Map>
	static constexpr bool apply(Pred, Map const& m, Node n)
	{
		return 1 < n.depth() && m.isParent(n.index());
	}
};

template <>
struct InnerCheck<Parent> {
	using Pred = Parent;

	template <class Map>
	static constexpr bool apply(Pred, Map const& m, Node n)
	{
		return 1 < n.depth() && m.isParent(n.index());
	}
};

template <>
struct InnerCheck<Exists> {
	using Pred = Exists;

	template <class Map>
	static constexpr bool apply(Pred, Map const& m, Node n)
	{
		return m.isParent(n.index());
	}
};

template <PredicateCompare PC, PredicateType PT>
struct InnerCheck<Depth<PC, PT>> {
	using Pred = Depth<PC, PT>;

	template <class Map>
	static constexpr bool apply(Pred p, Map const& m, Node n) noexcept
	{
		if constexpr (PredicateType::RETURN == PT) {
			return ValueCheck<Pred>::apply(p, m, n);
		} else if constexpr (PredicateCompare::EQUAL == PC) {
			return n.depth() > p.depth;
		} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
			return true;
		} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
			return n.depth() > p.depth;
		} else if constexpr (PredicateCompare::LESS == PC) {
			return true;
		} else if constexpr (PredicateCompare::GREATER == PC) {
			return n.depth() > (p.depth + 1U);
		}
	}
};

template <>
struct InnerCheck<DepthInterval> {
	using Pred = DepthInterval;

	template <class Map>
	static constexpr bool apply(Pred p, Map const& m, Node n) noexcept
	{
		return InnerCheck<std::decay_t<decltype(p.min)>>::apply(p.min, m, n) &&
		       InnerCheck<std::decay_t<decltype(p.max)>>::apply(p.max, m, n);
	}
};

template <PredicateRounding PR, PredicateCompare PC, PredicateType PT>
struct InnerCheck<Size<PR, PC, PT>> {
	using Pred = Size<PR, PC, PT>;

	template <class Map>
	static constexpr bool apply(Pred const& p, Map const& m, Node n) noexcept
	{
		if constexpr (PredicateType::RETURN == PT) {
			return ValueCheck<Pred>::apply(p, m, n);
		}

		if (p.depth_modified) {
			double temp = std::max(0.0, (m.resolution() / p.resolution) + 1.0);
			if constexpr (PredicateRounding::NEAREST == PR) {
				p.depth.depth = std::round(temp);
			} else if constexpr (PredicateRounding::FLOOR == PR) {
				p.depth.depth = std::floor(temp);
			} else {
				p.depth.depth = std::ceil(temp);
			}
			p.depth_modified = false;
		}

		return InnerCheck<std::decay_t<decltype(p.depth)>>::apply(p.depth, m, n);
	}
};

template <>
struct InnerCheck<SizeInterval> {
	using Pred = SizeInterval;

	template <class Map>
	static constexpr bool apply(Pred const& p, Map const& m, Node n) noexcept
	{
		return InnerCheck<std::decay_t<decltype(p.min)>>::apply(p.min, m, n) &&
		       InnerCheck<std::decay_t<decltype(p.max)>>::apply(p.max, m, n);
	}
};

template <>
struct InnerCheck<Modified> {
	using Pred = Modified;

	template <class Map>
	static constexpr bool apply(Pred, Map const& m, Node n) noexcept
	{
		return m.isModified(n.index());
	}
};

template <>
struct InnerCheck<ChildOf> {
	using Pred = ChildOf;

	template <class Map>
	static constexpr bool apply(Pred p, Map const&, Node n) noexcept
	{
		return n.depth() > p.code.depth()
		           ? Code::equalAtDepth(n.code(), p.code, n.depth())
		           : Code::equalAtDepth(n.code(), p.code, p.code.depth());
	}
};
}  // namespace ufo::pred

#endif  // UFO_MAP_PREDICATE_OCTREE_HPP