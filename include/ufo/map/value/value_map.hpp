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

#ifndef UFO_MAP_VALUE_MAP_HPP
#define UFO_MAP_VALUE_MAP_HPP

// UFO
#include <ufo/algorithm/algorithm.hpp>
#include <ufo/map/types.hpp>

// STL
#include <algorithm>
#include <functional>
#include <iostream>
#include <limits>
#include <ranges>
#include <utility>

namespace ufo
{
template <class Derived, std::size_t N>
class ValueMap
{
 public:
	//
	// Get value
	//

	[[nodiscard]] value_t value(Index node) const { return value_[node.pos][node.offset]; }

	[[nodiscard]] value_t value(Node node) const { return value(derived().index(node)); }

	[[nodiscard]] value_t value(Code code) const { return value(derived().index(code)); }

	[[nodiscard]] value_t value(Key key) const { return value(derived().index(key)); }

	[[nodiscard]] value_t value(Point coord, depth_t depth = 0) const
	{
		return value(derived().index(coord, depth));
	}

	[[nodiscard]] value_t value(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return value(derived().index(x, y, z, depth));
	}

	//
	// Set value
	//

	void setValue(Index node, value_t value)
	{
		derived().apply(
		    node, [this, value](Index node) { value_[node.pos][node.offset] = value; },
		    [this, value](pos_t pos) { value_[pos].fill(value); });
	}

	Node setValue(Node node, value_t value, bool propagate = true)
	{
		return derived().apply(
		    node, [this, value](Index node) { value_[node.pos][node.offset] = value; },
		    [this, value](pos_t pos) { value_[pos].fill(value); }, propagate);
	}

	Node setValue(Code code, value_t value, bool propagate = true)
	{
		return derived().apply(
		    code, [this, value](Index node) { value_[node.pos][node.offset] = value; },
		    [this, value](pos_t pos) { value_[pos].fill(value); }, propagate);
	}

	Node setValue(Key key, value_t value, bool propagate = true)
	{
		return setValue(derived().toCode(key), value, propagate);
	}

	Node setValue(Point coord, value_t value, bool propagate = true, depth_t depth = 0)
	{
		return setValue(derived().toCode(coord, depth), value, propagate);
	}

	Node setValue(coord_t x, coord_t y, coord_t z, value_t value, bool propagate = true,
	              depth_t depth = 0)
	{
		return setValue(derived().toCode(x, y, z, depth), value, propagate);
	}

	//
	// Update value
	//

	void updateValue(Index node, std::invocable<value_t> auto unary_op)
	{
		derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    value_[node.pos][node.offset] = unary_op(value_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : value_[pos]) {
				    e = unary_op(e);
			    }
		    });
	}

	void updateValue(Index node, std::invocable<Index, value_t> auto binary_op)
	{
		derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    value_[node.pos][node.offset] = binary_op(node, value_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (offset_t i{}; auto& e : value_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    });
	}

	Node updateValue(Node node, std::invocable<value_t> auto unary_op,
	                 bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    value_[node.pos][node.offset] = unary_op(value_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : value_[pos]) {
				    e = unary_op(e);
			    }
		    },
		    propagate);
	}

	Node updateValue(Node node, std::invocable<Index, value_t> auto binary_op,
	                 bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    value_[node.pos][node.offset] = binary_op(node, value_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (offset_t i{}; auto& e : value_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    },
		    propagate);
	}

	Node updateValue(Code code, std::invocable<value_t> auto unary_op,
	                 bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, unary_op](Index node) {
			    value_[node.pos][node.offset] = unary_op(value_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : value_[pos]) {
				    e = unary_op(e);
			    }
		    },
		    propagate);
	}

	Node updateValue(Code code, std::invocable<Index, value_t> auto binary_op,
	                 bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, binary_op](Index node) {
			    value_[node.pos][node.offset] = binary_op(node, value_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (offset_t i{}; auto& e : value_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    },
		    propagate);
	}

	Node updateValue(Key key, std::invocable<value_t> auto unary_op, bool propagate = true)
	{
		return updateValue(derived().toCode(key), unary_op, propagate);
	}

	Node updateValue(Key key, std::invocable<Index, value_t> auto binary_op,
	                 bool propagate = true)
	{
		return updateValue(derived().toCode(key), binary_op, propagate);
	}

	Node updateValue(Point coord, std::invocable<value_t> auto unary_op,
	                 bool propagate = true, depth_t depth = 0)
	{
		return updateValue(derived().toCode(coord, depth), unary_op, propagate);
	}

	Node updateValue(Point coord, std::invocable<Index, value_t> auto binary_op,
	                 bool propagate = true, depth_t depth = 0)
	{
		return updateValue(derived().toCode(coord, depth), binary_op, propagate);
	}

	Node updateValue(coord_t x, coord_t y, coord_t z, std::invocable<value_t> auto unary_op,
	                 bool propagate = true, depth_t depth = 0)
	{
		return updateValue(derived().toCode(x, y, z, depth), unary_op, propagate);
	}

	Node updateValue(coord_t x, coord_t y, coord_t z,
	                 std::invocable<Index, value_t> auto binary_op, bool propagate = true,
	                 depth_t depth = 0)
	{
		return updateValue(derived().toCode(x, y, z, depth), binary_op, propagate);
	}

	//
	// Propagation criteria
	//

	[[nodiscard]] constexpr PropagationCriteria valuePropagationCriteria() const noexcept
	{
		return prop_criteria_;
	}

	void setValuePropagationCriteria(PropagationCriteria prop_criteria,
	                                 bool                propagate = true) noexcept
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

	ValueMap() { value_.emplace_back(); }

	ValueMap(ValueMap const& other) = default;

	ValueMap(ValueMap&& other) = default;

	template <class Derived2>
	ValueMap(ValueMap<Derived2, N> const& other)
	    : value_(other.value_), prop_criteria_(other.prop_criteria_)
	{
	}

	template <class Derived2>
	ValueMap(ValueMap<Derived2, N>&& other)
	    : value_(std::move(other.value_)), prop_criteria_(std::move(other.prop_criteria_))
	{
	}

	//
	// Destructor
	//

	~ValueMap() = default;

	//
	// Assignment operator
	//

	ValueMap& operator=(ValueMap const& rhs) = default;

	ValueMap& operator=(ValueMap&& rhs) = default;

	template <class Derived2>
	ValueMap& operator=(ValueMap<Derived2, N> const& rhs)
	{
		value_         = rhs.value_;
		prop_criteria_ = rhs.prop_criteria_;
		return *this;
	}

	template <class Derived2>
	ValueMap& operator=(ValueMap<Derived2, N>&& rhs)
	{
		value_         = std::move(rhs.value_);
		prop_criteria_ = std::move(rhs.prop_criteria_);
		return *this;
	}

	//
	// Swap
	//

	void swap(ValueMap& other) noexcept
	{
		std::swap(value_, other.value_);
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
		value_.emplace_back();
		value_.back().fill(value_[node.pos][node.offset]);
	}

	//
	// Resize
	//

	void resize(std::size_t count)
	{
		// TODO: Implement
		value_.resize(count);
	}

	//
	// Reserve
	//

	void reserveImpl(std::size_t new_cap) { value_.reserve(new_cap); }

	//
	// Initialize root
	//

	void initRoot()
	{
		auto node                     = derived().rootIndex();
		value_[node.pos][node.offset] = 0;
	}

	//
	// Fill
	//

	void fill(Index node, pos_t children)
	{
		value_[children].fill(value_[node.pos][node.offset]);
	}

	//
	// Clear
	//

	void clearImpl() { value_.resize(1); }

	void clearImpl(pos_t nodes) {}

	//
	// Shrink to fit
	//

	void shrinkToFitImpl() { value_.shrink_to_fit(); }

	//
	// Update node
	//

	void updateNode(Index node, pos_t children)
	{
		switch (valuePropagationCriteria()) {
			case PropagationCriteria::MIN:
				value_[node.pos][node.offset] = std::ranges::min(value_[children]);
				return;
			case PropagationCriteria::MAX:
				value_[node.pos][node.offset] = std::ranges::max(value_[children]);
				return;
			case PropagationCriteria::MEAN:
				value_[node.pos][node.offset] = mean(children);
				return;
			case PropagationCriteria::FIRST:
				value_[node.pos][node.offset] = value_[children].front();
				return;
			case PropagationCriteria::NONE: return;
		}
	}

	[[nodiscard]] constexpr value_t mean(pos_t nodes) const
	{
		return static_cast<value_t>(
		    std::accumulate(std::cbegin(value_[nodes]), std::cend(value_[nodes]), 0.0) / N);
	}

	//
	// Is prunable
	//

	[[nodiscard]] bool isPrunable(pos_t nodes) const
	{
		return std::all_of(std::cbegin(value_[nodes]) + 1, std::cend(value_[nodes]),
		                   [a = value_[nodes].front()](auto b) { return a == b; });
	}

	//
	// Memory node block
	//

	[[nodiscard]] static constexpr std::size_t memoryNodeBlock() noexcept
	{
		return N * sizeof(value_t);
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr MapType mapType() noexcept { return MapType::VALUE; }

	constexpr std::size_t serializedSize(std::ranges::input_range auto r) const
	{
		return std::ranges::size(r) * memoryNodeBlock();
	}

	void readNodes(ReadBuffer& in, std::ranges::input_range auto r)
	{
		for (auto const [pos, offsets] : r) {
			if (offsets.all()) {
				in.read(value_[pos].data(), memoryNodeBlock());
			} else {
				DataBlock<value_t, N> value;
				in.read(value.data(), memoryNodeBlock());
				for (offset_t i{}; N != i; ++i) {
					value_[pos][i] = offsets[i] ? value[i] : value_[pos][i];
				}
			}
		}
	}

	void writeNodes(WriteBuffer& out, std::ranges::input_range auto r) const
	{
		for (auto pos : r) {
			out.write(value_[pos].data(), memoryNodeBlock());
		}
	}

 protected:
	// Data
	Container<DataBlock<value_t, N>> value_;

	// Propagation criteria
	PropagationCriteria prop_criteria_ = PropagationCriteria::MAX;

	template <class Derived2, std::size_t N2>
	friend class ValueMap;
};

//
// Concepts
//

template <class Map>
concept IsValueMap = IsMapType<Map, MapType::VALUE>;
}  // namespace ufo

#endif  // UFO_MAP_VALUE_MAP_HPP