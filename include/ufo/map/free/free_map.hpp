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

#ifndef UFO_MAP_FREE_MAP_HPP
#define UFO_MAP_FREE_MAP_HPP

// UFO
#include <ufo/map/buffer.hpp>
#include <ufo/map/code.hpp>
#include <ufo/map/index.hpp>
#include <ufo/map/key.hpp>
#include <ufo/map/node.hpp>
#include <ufo/map/point.hpp>
#include <ufo/map/types.hpp>

// STL

namespace ufo
{
template <class Derived, std::size_t N>
class FreeMap
{
 public:
	//
	// Get free
	//

	[[nodiscard]] std::uint8_t free(Index node) const { return free_[node.pos][node.offset]; }

	[[nodiscard]] std::uint8_t free(Node node) const { return free(derived().index(node)); }

	[[nodiscard]] std::uint8_t free(Code code) const { return free(derived().index(code)); }

	[[nodiscard]] std::uint8_t free(Key key) const { return free(derived().index(key)); }

	[[nodiscard]] std::uint8_t free(Point coord, depth_t depth = 0) const
	{
		return free(derived().index(coord, depth));
	}

	[[nodiscard]] std::uint8_t free(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return free(derived().index(x, y, z, depth));
	}

	//
	// Set free
	//

	void setFree(Index node, std::uint8_t distance)
	{
		derived().apply(
		    node, [this, distance](Index node) { free_[node.pos][node.offset] = distance; },
		    [this, distance](pos_t pos) { free_[pos].fill(distance); });
	}

	Node setFree(Node node, std::uint8_t distance, bool propagate = true)
	{
		return derived().apply(
		    node, [this, distance](Index node) { free_[node.pos][node.offset] = distance; },
		    [this, distance](pos_t pos) { free_[pos].fill(distance); }, propagate);
	}

	Node setFree(Code code, std::uint8_t distance, bool propagate = true)
	{
		return derived().apply(
		    code, [this, distance](Index node) { free_[node.pos][node.offset] = distance; },
		    [this, distance](pos_t pos) { free_[pos].fill(distance); }, propagate);
	}

	Node setFree(Key key, std::uint8_t distance, bool propagate = true)
	{
		return setFree(derived().toCode(key), distance, propagate);
	}

	Node setFree(Point coord, std::uint8_t distance, bool propagate = true, depth_t depth = 0)
	{
		return setFree(derived().toCode(coord, depth), distance, propagate);
	}

	Node setFree(coord_t x, coord_t y, coord_t z, std::uint8_t distance, bool propagate = true,
	             depth_t depth = 0)
	{
		return setFree(derived().toCode(x, y, z, depth), distance, propagate);
	}

	//
	// Reset free
	//

	void resetFree(Index node)
	{
		derived().apply(
		    node, [this](Index node) { free_[node.pos][node.offset] = 0.0f; },
		    [this](pos_t pos) { free_[pos].fill(0.0f); });
	}

	Node resetFree(Node node, bool propagate = true)
	{
		return derived().apply(
		    node, [this](Index node) { free_[node.pos][node.offset] = 0.0f; },
		    [this](pos_t pos) { free_[pos].fill(0.0f); }, propagate);
	}

	Node resetFree(Code code, bool propagate = true)
	{
		return derived().apply(
		    code, [this](Index node) { free_[node.pos][node.offset] = 0.0f; },
		    [this](pos_t pos) { free_[pos].fill(0.0f); }, propagate);
	}

	Node resetFree(Key key, bool propagate = true)
	{
		return resetFree(derived().toCode(key), propagate);
	}

	Node resetFree(Point coord, bool propagate = true, depth_t depth = 0)
	{
		return resetFree(derived().toCode(coord, depth), propagate);
	}

	Node resetFree(coord_t x, coord_t y, coord_t z, bool propagate = true,
	               depth_t depth = 0)
	{
		return resetFree(derived().toCode(x, y, z, depth), propagate);
	}

	//
	// Update free
	//

	void updateFree(Index node, std::invocable<std::uint8_t> auto unary_op)
	{
		derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    free_[node.pos][node.offset] = unary_op(free_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : free_[pos]) {
				    e = unary_op(e);
			    }
		    });
	}

	void updateFree(Index node, std::invocable<Index, std::uint8_t> auto binary_op)
	{
		derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    free_[node.pos][node.offset] = binary_op(node, free_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (std::size_t i{}; auto& e : free_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    });
	}

	Node updateFree(Node node, std::invocable<std::uint8_t> auto unary_op, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    free_[node.pos][node.offset] = unary_op(free_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : free_[pos]) {
				    e = unary_op(e);
			    }
		    },
		    propagate);
	}

	Node updateFree(Node node, std::invocable<Index, std::uint8_t> auto binary_op,
	                bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    free_[node.pos][node.offset] = binary_op(node, free_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (std::size_t i{}; auto& e : free_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    },
		    propagate);
	}

	Node updateFree(Code code, std::invocable<std::uint8_t> auto unary_op, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, unary_op](Index node) {
			    free_[node.pos][node.offset] = unary_op(free_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : free_[pos]) {
				    e = unary_op(e);
			    }
		    },
		    propagate);
	}

	Node updateFree(Code code, std::invocable<Index, std::uint8_t> auto binary_op,
	                bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, binary_op](Index node) {
			    free_[node.pos][node.offset] = binary_op(node, free_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (std::size_t i{}; auto& e : free_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    },
		    propagate);
	}

	Node updateFree(Key key, std::invocable<std::uint8_t> auto unary_op, bool propagate = true)
	{
		return updateFree(derived().toCode(key), unary_op, propagate);
	}

	Node updateFree(Key key, std::invocable<Index, std::uint8_t> auto binary_op,
	                bool propagate = true)
	{
		return updateFree(derived().toCode(key), binary_op, propagate);
	}

	Node updateFree(Point coord, std::invocable<std::uint8_t> auto unary_op, bool propagate = true,
	                depth_t depth = 0)
	{
		return updateFree(derived().toCode(coord, depth), unary_op, propagate);
	}

	Node updateFree(Point coord, std::invocable<Index, std::uint8_t> auto binary_op,
	                bool propagate = true, depth_t depth = 0)
	{
		return updateFree(derived().toCode(coord, depth), binary_op, propagate);
	}

	Node updateFree(coord_t x, coord_t y, coord_t z, std::invocable<std::uint8_t> auto unary_op,
	                bool propagate = true, depth_t depth = 0)
	{
		return updateFree(derived().toCode(x, y, z, depth), unary_op, propagate);
	}

	Node updateFree(coord_t x, coord_t y, coord_t z,
	                std::invocable<Index, std::uint8_t> auto binary_op, bool propagate = true,
	                depth_t depth = 0)
	{
		return updateFree(derived().toCode(x, y, z, depth), binary_op, propagate);
	}

 protected:
	//
	// Constructors
	//

	FreeMap() { free_.emplace_back(); }

	FreeMap(FreeMap const& other) = default;

	FreeMap(FreeMap&& other) = default;

	template <class Derived2>
	FreeMap(FreeMap<Derived2, N> const& other) : free_(other.free_)
	{
	}

	template <class Derived2>
	FreeMap(FreeMap<Derived2, N>&& other) : free_(std::move(other.free_))
	{
	}

	//
	// Destructor
	//

	~FreeMap() = default;

	//
	// Assignment operator
	//

	FreeMap& operator=(FreeMap const& rhs) = default;

	FreeMap& operator=(FreeMap&& rhs) = default;

	template <class Derived2>
	FreeMap& operator=(FreeMap<Derived2, N> const& rhs)
	{
		free_ = rhs.free_;
		return *this;
	}

	template <class Derived2>
	FreeMap& operator=(FreeMap<Derived2, N>&& rhs)
	{
		free_ = std::move(rhs.free_);
		return *this;
	}

	//
	// Swap
	//

	void swap(FreeMap& other) noexcept { std::swap(free_, other.free_); }

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
		free_.emplace_back();
		free_.back().fill(free_[node.pos][node.offset]);
	}

	//
	// Resize
	//

	void resize(std::size_t count)
	{
		// TODO: Look at
		free_.resize(count);
	}

	//
	// Reserve
	//

	void reserveImpl(std::size_t new_cap) { free_.reserve(new_cap); }

	//
	// Initialize root
	//

	void initRoot()
	{
		auto node                    = derived().rootIndex();
		free_[node.pos][node.offset] = 0.0f;
	}

	//
	// Fill
	//

	void fill(Index node, pos_t children)
	{
		free_[children].fill(free_[node.pos][node.offset]);
	}

	//
	// Clear
	//

	void clearImpl()
	{
		// TODO: Look at
		free_.assign(1);
	}

	void clearImpl(pos_t nodes) {}

	//
	// Shrink to fit
	//

	void shrinkToFitImpl() { free_.shrink_to_fit(); }

	//
	// Update node
	//

	void updateNode(Index node, pos_t children)
	{
		free_[node.pos][node.offset] = std::ranges::min(free_[children]);
	}

	//
	// Is prunable
	//

	[[nodiscard]] bool isPrunable(pos_t nodes) const
	{
		return std::all_of(std::cbegin(free_[nodes]) + 1, std::cend(free_[nodes]),
		                   [a = free_[nodes].front()](auto b) { return a == b; });
	}

	//
	// Memory node block
	//

	[[nodiscard]] static constexpr std::size_t memoryNodeBlock() noexcept
	{
		return N * sizeof(std::uint8_t);
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr MapType mapType() noexcept { return MapType::FREE; }

	constexpr std::size_t serializedSize(std::ranges::input_range auto r) const
	{
		return std::ranges::size(r) * memoryNodeBlock();
	}

	void readNodes(ReadBuffer& in, std::ranges::input_range auto r)
	{
		// TODO: Implement
		// for (auto const [pos, offsets] : r) {
		// 	if (offsets.all()) {
		// 		in.read(free_[pos].data(), memoryNodeBlock());
		// 	} else {
		// 		DataBlock<free_t, N> free;
		// 		in.read(free.data(), memoryNodeBlock());
		// 		for (offset_t i{}; N != i; ++i) {
		// 			free_[pos][i] = offsets[i] ? free[i] : free_[pos][i];
		// 		}
		// 	}
		// }
	}

	void writeNodes(WriteBuffer& out, std::ranges::input_range auto r) const
	{
		for (auto pos : r) {
			out.write(&free_[pos], memoryNodeBlock());
		}
	}

	//
	// Dot file info
	//

	std::ostream& dotFileInfo(std::ostream& out, Index node) const
	{
		return out << "Free: " << free_[node.pos][node.offset];
	}

 protected:
	Container<DataBlock<std::uint8_t, N>> free_;

	template <class Derived2, std::size_t N2>
	friend class FreeMap;
};

//
// Concepts
//

template <class Map>
concept IsFreeMap = IsMapType<Map, MapType::FREE>;
}  // namespace ufo

#endif  // UFO_MAP_FREE_MAP_HPP