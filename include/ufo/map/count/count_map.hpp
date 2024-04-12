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

#ifndef UFO_MAP_COUNT_MAP_HPP
#define UFO_MAP_COUNT_MAP_HPP

// UFO
#include <ufo/algorithm/algorithm.hpp>
#include <ufo/map/types.hpp>

// STL
#include <algorithm>
#include <array>
#include <concepts>
#include <cstdint>
#include <iostream>
#include <type_traits>
#include <utility>
#include <vector>

namespace ufo
{
template <class Derived, std::size_t N>
class CountMap
{
 public:
	//
	// Get count
	//

	[[nodiscard]] count_t count(Index node) const { return count_[node.pos][node.offset]; }

	[[nodiscard]] count_t count(Node node) const { return count(derived().index(node)); }

	[[nodiscard]] count_t count(Code code) const { return count(derived().index(code)); }

	[[nodiscard]] count_t count(Key key) const { return count(derived().index(key)); }

	[[nodiscard]] count_t count(Point coord, depth_t depth = 0) const
	{
		return count(derived().index(coord, depth));
	}

	[[nodiscard]] count_t count(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return count(derived().index(x, y, z, depth));
	}

	//
	// Set count
	//

	void setCount(Index node, count_t count)
	{
		return derived().apply(
		    node, [this, count](Index node) { count_[node.pos][node.offset] = count; },
		    [this, count](pos_t pos) { count_[pos].fill(count); });
	}

	Node setCount(Node node, count_t count, bool propagate = true)
	{
		return derived().apply(
		    node, [this, count](Index node) { count_[node.pos][node.offset] = count; },
		    [this, count](pos_t pos) { count_[pos].fill(count); }, propagate);
	}

	Node setCount(Code code, count_t count, bool propagate = true)
	{
		return derived().apply(
		    code, [this, count](Index node) { count_[node.pos][node.offset] = count; },
		    [this, count](pos_t pos) { count_[pos].fill(count); }, propagate);
	}

	Node setCount(Key key, count_t count, bool propagate = true)
	{
		return setCount(derived().toCode(key), count, propagate);
	}

	Node setCount(Point coord, count_t count, bool propagate = true, depth_t depth = 0)
	{
		return setCount(derived().toCode(coord, depth), count, propagate);
	}

	Node setCount(coord_t x, coord_t y, coord_t z, count_t count, bool propagate = true,
	              depth_t depth = 0)
	{
		return setCount(derived().toCode(x, y, z, depth), count, propagate);
	}

	//
	// Update count
	//

	void updateCount(Index node, int change)
	{
		derived().apply(
		    node, [this, change](Index node) { count_[node.pos][node.offset] += change; },
		    [this, change](pos_t pos) {
			    for (auto& e : count_[pos]) {
				    e += change;
			    }
		    });
	}

	void updateCount(Index node, std::invocable<count_t> auto unary_op)
	{
		derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    count_[node.pos][node.offset] = unary_op(count_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : count_[pos]) {
				    e = unary_op(e);
			    }
		    });
	}

	void updateCount(Index node, std::invocable<Index, count_t> auto binary_op)
	{
		derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    count_[node.pos][node.offset] = binary_op(node, count_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (offset_t i{}; auto& e : count_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    });
	}

	Node updateCount(Node node, int change, bool propagate = true)
	{
		return derived().apply(
		    node, [this, change](Index node) { count_[node.pos][node.offset] += change; },
		    [this, change](pos_t pos) {
			    for (auto& e : count_[pos]) {
				    e += change;
			    }
		    },
		    propagate);
	}

	Node updateCount(Node node, std::invocable<count_t> auto unary_op,
	                 bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    count_[node.pos][node.offset] = unary_op(count_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : count_[pos]) {
				    e = unary_op(e);
			    }
		    },
		    propagate);
	}

	Node updateCount(Node node, std::invocable<Index, count_t> auto binary_op,
	                 bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    count_[node.pos][node.offset] = binary_op(node, count_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (offset_t i{}; auto& e : count_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    },
		    propagate);
	}

	Node updateCount(Code code, int change, bool propagate = true)
	{
		return derived().apply(
		    code, [this, change](Index node) { count_[node.pos][node.offset] += change; },
		    [this, change](pos_t pos) {
			    for (auto& e : count_[pos]) {
				    e += change;
			    }
		    },
		    propagate);
	}

	Node updateCount(Code code, std::invocable<count_t> auto unary_op,
	                 bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, unary_op](Index node) {
			    count_[node.pos][node.offset] = unary_op(count_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : count_[pos]) {
				    e = unary_op(e);
			    }
		    },
		    propagate);
	}

	Node updateCount(Code code, std::invocable<Index, count_t> auto binary_op,
	                 bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, binary_op](Index node) {
			    count_[node.pos][node.offset] = binary_op(node, count_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (offset_t i{}; auto& e : count_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    },
		    propagate);
	}

	Node updateCount(Key key, int change, bool propagate = true)
	{
		return updateCount(derived().toCode(key), change, propagate);
	}

	Node updateCount(Key key, std::invocable<count_t> auto unary_op, bool propagate = true)
	{
		return updateCount(derived().toCode(key), unary_op, propagate);
	}

	Node updateCount(Key key, std::invocable<Index, count_t> auto binary_op,
	                 bool propagate = true)
	{
		return updateCount(derived().toCode(key), binary_op, propagate);
	}

	Node updateCount(Point coord, int change, bool propagate = true, depth_t depth = 0)
	{
		return updateCount(derived().toCode(coord, depth), change, propagate);
	}

	Node updateCount(Point coord, std::invocable<count_t> auto unary_op,
	                 bool propagate = true, depth_t depth = 0)
	{
		return updateCount(derived().toCode(coord, depth), unary_op, propagate);
	}

	Node updateCount(Point coord, std::invocable<Index, count_t> auto binary_op,
	                 bool propagate = true, depth_t depth = 0)
	{
		return updateCount(derived().toCode(coord, depth), binary_op, propagate);
	}

	Node updateCount(coord_t x, coord_t y, coord_t z, int change, bool propagate = true,
	                 depth_t depth = 0)
	{
		return updateCount(derived().toCode(x, y, z, depth), change, propagate);
	}

	Node updateCount(coord_t x, coord_t y, coord_t z, std::invocable<count_t> auto unary_op,
	                 bool propagate = true, depth_t depth = 0)
	{
		return updateCount(derived().toCode(x, y, z, depth), unary_op, propagate);
	}

	Node updateCount(coord_t x, coord_t y, coord_t z,
	                 std::invocable<Index, count_t> auto binary_op, bool propagate = true,
	                 depth_t depth = 0)
	{
		return updateCount(derived().toCode(x, y, z, depth), binary_op, propagate);
	}

	//
	// Propagation criteria
	//

	[[nodiscard]] constexpr PropagationCriteria countPropagationCriteria() const noexcept
	{
		return prop_criteria_;
	}

	void setCountPropagationCriteria(PropagationCriteria prop_criteria,
	                                 bool                propagate = true)
	{
		if (prop_criteria_ == prop_criteria) {
			return;
		}

		prop_criteria_ = prop_criteria;

		derived().setModified();

		if (propagate) {
			derived().propagateModified();
		}
	}

 protected:
	//
	// Constructors
	//

	CountMap() { count_.emplace_back(); }

	CountMap(CountMap const&) = default;

	CountMap(CountMap&&) = default;

	template <class Derived2>
	CountMap(CountMap<Derived2, N> const& other)
	    : count_(other.count_), prop_criteria_(other.prop_criteria_)
	{
	}

	template <class Derived2>
	CountMap(CountMap<Derived2, N>&& other)
	    : count_(std::move(other.count_)), prop_criteria_(std::move(other.prop_criteria_))
	{
	}

	//
	// Destructor
	//

	~CountMap() = default;

	//
	// Assignment operator
	//

	CountMap& operator=(CountMap const&) = default;

	CountMap& operator=(CountMap&&) = default;

	template <class Derived2>
	CountMap& operator=(CountMap<Derived2, N> const& rhs)
	{
		count_         = rhs.count_;
		prop_criteria_ = rhs.prop_criteria_;
		return *this;
	}

	template <class Derived2>
	CountMap& operator=(CountMap<Derived2, N>&& rhs)
	{
		count_         = std::move(rhs.count_);
		prop_criteria_ = std::move(rhs.prop_criteria_);
		return *this;
	}

	//
	// Swap
	//

	void swap(CountMap& other) noexcept
	{
		std::swap(count_, other.count_);
		std::swap(prop_criteria_, other.prop_criteria_);
	}

	//
	// Derived
	//

	[[nodiscard]] constexpr Derived& derived() { return *static_cast<Derived*>(this); }

	[[nodiscard]] constexpr Derived const& derived() const
	{
		return *static_cast<Derived const*>(this);
	}

	//
	// Create node block
	//

	void createNodeBlock(Index node)
	{
		count_.emplace_back();
		count_.back().fill(count_[node.pos][node.offset]);
	}

	//
	// Resize
	//

	void resize(std::size_t count)
	{
		// TODO: Implement
		count_.resize(count);
	}

	//
	// Reserve
	//

	void reserveImpl(std::size_t new_cap) { count_.reserve(new_cap); }

	//
	// Initialize root
	//

	void initRoot()
	{
		auto node                     = derived().rootIndex();
		count_[node.pos][node.offset] = 0;
	}

	//
	// Fill
	//

	void fill(Index node, pos_t children)
	{
		count_[children].fill(count_[node.pos][node.offset]);
	}

	//
	// Clear
	//

	void clearImpl() { count_.resize(1); }

	void clearImpl(pos_t nodes) {}

	//
	// Shrink to fit
	//

	void shrinkToFitImpl() { count_.shrink_to_fit(); }

	//
	// Update node
	//

	void updateNode(Index node, pos_t children)
	{
		switch (countPropagationCriteria()) {
			case PropagationCriteria::MIN:
				count_[node.pos][node.offset] = std::ranges::min(count_[children]);
				return;
			case PropagationCriteria::MAX:
				count_[node.pos][node.offset] = std::ranges::max(count_[children]);
				return;
			case PropagationCriteria::MEAN:
				count_[node.pos][node.offset] = mean(children);
				return;
			case PropagationCriteria::FIRST:
				count_[node.pos][node.offset] = count_[children].front();
				return;
			case PropagationCriteria::NONE: return;
		}
	}

	[[nodiscard]] constexpr count_t mean(pos_t nodes) const
	{
		// FIXME: What happens if count_t is not unsigned integer?
		if constexpr (std::unsigned_integral<count_t>) {
			return std::accumulate(std::cbegin(count_[nodes]), std::cend(count_[nodes]),
			                       std::uint64_t(0)) /
			       N;
		} else {
			return std::accumulate(std::cbegin(count_[nodes]), std::cend(count_[nodes]), 0.0) /
			       N;
		}
	}

	//
	// Is prunable
	//

	[[nodiscard]] bool isPrunable(pos_t nodes) const
	{
		return std::all_of(std::cbegin(count_[nodes]) + 1, std::cend(count_[nodes]),
		                   [a = count_[nodes].front()](auto b) { return a == b; });
	}

	//
	// Memory node block
	//

	[[nodiscard]] static constexpr std::size_t memoryNodeBlock() noexcept
	{
		return N * sizeof(count_t);
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr MapType mapType() noexcept { return MapType::COUNT; }

	std::size_t serializedSize(std::ranges::input_range auto r) const
	{
		return std::ranges::size(r) * memoryNodeBlock();
	}

	void readNodes(ReadBuffer& in, std::ranges::input_range auto r)
	{
		for (auto const [pos, offsets] : r) {
			if (offsets.all()) {
				in.read(count_[pos].data(), memoryNodeBlock());
			} else {
				DataBlock<count_t, N> count;
				in.read(count.data(), memoryNodeBlock());
				for (offset_t i{}; N != i; ++i) {
					count_[pos][i] = offsets[i] ? count[i] : count_[pos][i];
				}
			}
		}
	}

	void writeNodes(WriteBuffer& out, std::ranges::input_range auto r) const
	{
		out.reserve(out.size() + serializedSize(r));
		for (auto pos : r) {
			out.write(count_[pos].data(), memoryNodeBlock());
		}
	}

 protected:
	// Data
	Container<DataBlock<count_t, N>> count_;

	// Propagation criteria
	PropagationCriteria prop_criteria_ = PropagationCriteria::MAX;

	template <class Derived2, std::size_t N2>
	friend class CountMap;
};

//
// Concepts
//

template <class Map>
concept IsCountMap = IsMapType<Map, MapType::COUNT>;
}  // namespace ufo

#endif  // UFO_MAP_COUNT_MAP_HPP