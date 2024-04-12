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

#ifndef UFO_MAP_TIME_MAP_HPP
#define UFO_MAP_TIME_MAP_HPP

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
class TimeMap
{
 public:
	//
	// Get time
	//

	[[nodiscard]] time_t time(Index node) const { return time_[node.pos][node.offset]; }

	[[nodiscard]] time_t time(Node node) const { return time(derived().index(node)); }

	[[nodiscard]] time_t time(Code code) const { return time(derived().index(code)); }

	[[nodiscard]] time_t time(Key key) const { return time(derived().index(key)); }

	[[nodiscard]] time_t time(Point coord, depth_t depth = 0) const
	{
		return time(derived().index(coord, depth));
	}

	[[nodiscard]] time_t time(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return time(derived().index(x, y, z, depth));
	}

	//
	// Set time
	//

	void setTime(Index node, time_t value)
	{
		derived().apply(
		    node, [this, value](Index node) { time_[node.pos][node.offset] = value; },
		    [this, value](pos_t pos) { time_[pos].fill(value); });
	}

	Node setTime(Node node, time_t value, bool propagate = true)
	{
		return derived().apply(
		    node, [this, value](Index node) { time_[node.pos][node.offset] = value; },
		    [this, value](pos_t pos) { time_[pos].fill(value); }, propagate);
	}

	Node setTime(Code code, time_t value, bool propagate = true)
	{
		return derived().apply(
		    code, [this, value](Index node) { time_[node.pos][node.offset] = value; },
		    [this, value](pos_t pos) { time_[pos].fill(value); }, propagate);
	}

	Node setTime(Key key, time_t value, bool propagate = true)
	{
		return setTime(derived().toCode(key), value, propagate);
	}

	Node setTime(Point coord, time_t value, bool propagate = true, depth_t depth = 0)
	{
		return setTime(derived().toCode(coord, depth), value, propagate);
	}

	Node setTime(coord_t x, coord_t y, coord_t z, time_t value, bool propagate = true,
	             depth_t depth = 0)
	{
		return setTime(derived().toCode(x, y, z, depth), value, propagate);
	}

	//
	// Update time
	//

	void updateTime(Index node, std::invocable<time_t> auto unary_op)
	{
		derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    time_[node.pos][node.offset] = unary_op(time_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : time_[pos]) {
				    e = unary_op(e);
			    }
		    });
	}

	void updateTime(Index node, std::invocable<Index, time_t> auto binary_op)
	{
		derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    time_[node.pos][node.offset] = binary_op(node, time_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (offset_t i{}; auto& e : time_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    });
	}

	Node updateTime(Node node, std::invocable<time_t> auto unary_op, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    time_[node.pos][node.offset] = unary_op(time_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : time_[pos]) {
				    e = unary_op(e);
			    }
		    },
		    propagate);
	}

	Node updateTime(Node node, std::invocable<Index, time_t> auto binary_op,
	                bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    time_[node.pos][node.offset] = binary_op(node, time_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (offset_t i{}; auto& e : time_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    },
		    propagate);
	}

	Node updateTime(Code code, std::invocable<time_t> auto unary_op, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, unary_op](Index node) {
			    time_[node.pos][node.offset] = unary_op(time_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : time_[pos]) {
				    e = unary_op(e);
			    }
		    },
		    propagate);
	}

	Node updateTime(Code code, std::invocable<Index, time_t> auto binary_op,
	                bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, binary_op](Index node) {
			    time_[node.pos][node.offset] = binary_op(node, time_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (offset_t i{}; auto& e : time_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    },
		    propagate);
	}

	Node updateTime(Key key, std::invocable<time_t> auto unary_op, bool propagate = true)
	{
		return updateTime(derived().toCode(key), unary_op, propagate);
	}

	Node updateTime(Key key, std::invocable<Index, time_t> auto binary_op,
	                bool propagate = true)
	{
		return updateTime(derived().toCode(key), binary_op, propagate);
	}

	Node updateTime(Point coord, std::invocable<time_t> auto unary_op,
	                bool propagate = true, depth_t depth = 0)
	{
		return updateTime(derived().toCode(coord, depth), unary_op, propagate);
	}

	Node updateTime(Point coord, std::invocable<Index, time_t> auto binary_op,
	                bool propagate = true, depth_t depth = 0)
	{
		return updateTime(derived().toCode(coord, depth), binary_op, propagate);
	}

	Node updateTime(coord_t x, coord_t y, coord_t z, std::invocable<time_t> auto unary_op,
	                bool propagate = true, depth_t depth = 0)
	{
		return updateTime(derived().toCode(x, y, z, depth), unary_op, propagate);
	}

	Node updateTime(coord_t x, coord_t y, coord_t z,
	                std::invocable<Index, time_t> auto binary_op, bool propagate = true,
	                depth_t depth = 0)
	{
		return updateTime(derived().toCode(x, y, z, depth), binary_op, propagate);
	}

	//
	// Propagation criteria
	//

	[[nodiscard]] constexpr PropagationCriteria timePropagationCriteria() const noexcept
	{
		return prop_criteria_;
	}

	void setTimePropagationCriteria(PropagationCriteria prop_criteria,
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

	TimeMap() { time_.emplace_back(); }

	TimeMap(TimeMap const& other) = default;

	TimeMap(TimeMap&& other) = default;

	template <class Derived2>
	TimeMap(TimeMap<Derived2, N> const& other)
	    : time_(other.time_), prop_criteria_(other.prop_criteria_)
	{
	}

	template <class Derived2>
	TimeMap(TimeMap<Derived2, N>&& other)
	    : time_(std::move(other.time_)), prop_criteria_(std::move(other.prop_criteria_))
	{
	}

	//
	// Destructor
	//

	~TimeMap() = default;

	//
	// Assignment operator
	//

	TimeMap& operator=(TimeMap const& rhs) = default;

	TimeMap& operator=(TimeMap&& rhs) = default;

	template <class Derived2>
	TimeMap& operator=(TimeMap<Derived2, N> const& rhs)
	{
		time_          = rhs.time_;
		prop_criteria_ = rhs.prop_criteria_;
		return *this;
	}

	template <class Derived2>
	TimeMap& operator=(TimeMap<Derived2, N>&& rhs)
	{
		time_          = std::move(rhs.time_);
		prop_criteria_ = std::move(rhs.prop_criteria_);
		return *this;
	}

	//
	// Swap
	//

	void swap(TimeMap& other) noexcept
	{
		std::swap(time_, other.time_);
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
		time_.emplace_back();
		time_.back().fill(time_[node.pos][node.offset]);
	}

	//
	// Resize
	//

	void resize(std::size_t count)
	{
		// TODO: Implement
		time_.resize(count);
	}

	//
	// Reserve
	//

	void reserve(std::size_t new_cap) { time_.reserve(new_cap); }

	//
	// Initialize root
	//

	void initRoot()
	{
		auto node                    = derived().rootIndex();
		time_[node.pos][node.offset] = 0;
	}

	//
	// Fill
	//

	void fill(Index node, pos_t children)
	{
		time_[children].fill(time_[node.pos][node.offset]);
	}

	//
	// Clear
	//

	void clearImpl() { time_.resize(1); }

	void clearImpl(pos_t nodes) {}

	//
	// Shrink to fit
	//

	void shrinkToFit() { time_.shrink_to_fit(); }

	//
	// Update node
	//

	void updateNode(Index node, pos_t children)
	{
		switch (timePropagationCriteria()) {
			case PropagationCriteria::MIN:
				time_[node.pos][node.offset] = std::ranges::min(time_[children]);
				return;
			case PropagationCriteria::MAX:
				time_[node.pos][node.offset] = std::ranges::max(time_[children]);
				return;
			case PropagationCriteria::MEAN:
				time_[node.pos][node.offset] = mean(children);
				return;
			case PropagationCriteria::FIRST:
				time_[node.pos][node.offset] = time_[children].front();
				return;
			case PropagationCriteria::NONE: return;
		}
	}

	[[nodiscard]] constexpr time_t mean(pos_t nodes) const
	{
		return static_cast<time_t>(
		    std::accumulate(std::cbegin(time_[nodes]), std::cend(time_[nodes]), 0.0) / N);
	}

	//
	// Is prunable
	//

	[[nodiscard]] bool isPrunable(pos_t nodes) const
	{
		return std::all_of(std::cbegin(time_[nodes]) + 1, std::cend(time_[nodes]),
		                   [a = time_[nodes].front()](auto b) { return a == b; });
	}

	//
	// Memory node block
	//

	[[nodiscard]] static constexpr std::size_t memoryNodeBlock() noexcept
	{
		return N * sizeof(time_t);
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr MapType mapType() noexcept { return MapType::TIME; }

	constexpr std::size_t serializedSize(std::ranges::input_range auto r) const
	{
		return std::ranges::size(r) * memoryNodeBlock();
	}

	void readNodes(ReadBuffer& in, std::ranges::input_range auto r)
	{
		for (auto const [pos, offsets] : r) {
			if (offsets.all()) {
				in.read(time_[pos].data(), memoryNodeBlock());
			} else {
				DataBlock<time_t, N> time;
				in.read(time.data(), memoryNodeBlock());
				for (offset_t i{}; N != i; ++i) {
					time_[pos][i] = offsets[i] ? time[i] : time_[pos][i];
				}
			}
		}
	}

	void writeNodes(WriteBuffer& out, std::ranges::input_range auto r) const
	{
		for (auto pos : r) {
			out.write(time_[pos].data(), memoryNodeBlock());
		}
	}

 protected:
	// Data
	Container<DataBlock<time_t, N>> time_;

	// Propagation criteria
	PropagationCriteria prop_criteria_ = PropagationCriteria::MAX;

	template <class Derived2, std::size_t N2>
	friend class TimeMap;
};

//
// Concepts
//

template <class Map>
concept IsTimeMap = IsMapType<Map, MapType::TIME>;
}  // namespace ufo

#endif  // UFO_MAP_TIME_MAP_HPP