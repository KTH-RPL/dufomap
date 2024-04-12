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

#ifndef UFO_MAP_SEEN_FREE_MAP_HPP
#define UFO_MAP_SEEN_FREE_MAP_HPP

// UFO
#include <ufo/map/bit_set.hpp>
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
class SeenFreeMap
{
 public:
	//
	// Get seen free
	//

	[[nodiscard]] bool seenFree(Index node) const
	{
		return seen_free_[node.pos][node.offset];
	}

	[[nodiscard]] bool seenFree(Node node) const { return seenFree(derived().index(node)); }

	[[nodiscard]] bool seenFree(Code code) const { return seenFree(derived().index(code)); }

	[[nodiscard]] bool seenFree(Key key) const { return seenFree(derived().index(key)); }

	[[nodiscard]] bool seenFree(Point coord, depth_t depth = 0) const
	{
		return seenFree(derived().index(coord, depth));
	}

	[[nodiscard]] bool seenFree(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return seenFree(derived().index(x, y, z, depth));
	}

	//
	// Set seen free
	//

	void setSeenFree(Index node)
	{
		derived().apply(
		    node, [this](Index node) { seen_free_[node.pos].set(node.offset); },
		    [this](pos_t pos) { seen_free_[pos].set(); });
	}

	Node setSeenFree(Node node, bool propagate = true)
	{
		return derived().apply(
		    node, [this](Index node) { seen_free_[node.pos].set(node.offset); },
		    [this](pos_t pos) { seen_free_[pos].set(); }, propagate);
	}

	Node setSeenFree(Code code, bool propagate = true)
	{
		return derived().apply(
		    code, [this](Index node) { seen_free_[node.pos].set(node.offset); },
		    [this](pos_t pos) { seen_free_[pos].set(); }, propagate);
	}

	Node setSeenFree(Key key, bool propagate = true)
	{
		return setSeenFree(derived().toCode(key), propagate);
	}

	Node setSeenFree(Point coord, bool propagate = true, depth_t depth = 0)
	{
		return setSeenFree(derived().toCode(coord, depth), propagate);
	}

	Node setSeenFree(coord_t x, coord_t y, coord_t z, bool propagate = true,
	                 depth_t depth = 0)
	{
		return setSeenFree(derived().toCode(x, y, z, depth), propagate);
	}

	//
	// Reset seen free
	//

	void resetSeenFree(Index node)
	{
		derived().apply(
		    node, [this](Index node) { seen_free_[node.pos].reset(node.offset); },
		    [this](pos_t pos) { seen_free_[pos].reset(); });
	}

	Node resetSeenFree(Node node, bool propagate = true)
	{
		return derived().apply(
		    node, [this](Index node) { seen_free_[node.pos].reset(node.offset); },
		    [this](pos_t pos) { seen_free_[pos].reset(); }, propagate);
	}

	Node resetSeenFree(Code code, bool propagate = true)
	{
		return derived().apply(
		    code, [this](Index node) { seen_free_[node.pos].reset(node.offset); },
		    [this](pos_t pos) { seen_free_[pos].reset(); }, propagate);
	}

	Node resetSeenFree(Key key, bool propagate = true)
	{
		return resetSeenFree(derived().toCode(key), propagate);
	}

	Node resetSeenFree(Point coord, bool propagate = true, depth_t depth = 0)
	{
		return resetSeenFree(derived().toCode(coord, depth), propagate);
	}

	Node resetSeenFree(coord_t x, coord_t y, coord_t z, bool propagate = true,
	                   depth_t depth = 0)
	{
		return resetSeenFree(derived().toCode(x, y, z, depth), propagate);
	}

	//
	// Update seen free
	//

	void updateSeenFree(Index node, std::invocable<bool> auto unary_op)
	{
		derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    seen_free_[node.pos][node.offset] =
			        unary_op(std::as_const(seen_free_[node.pos])[node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (std::size_t i{}; N != i; ++i) {
				    seen_free_[pos][i] = unary_op(std::as_const(seen_free_[pos])[i]);
			    }
		    });
	}

	void updateSeenFree(Index node, std::invocable<Index, bool> auto binary_op)
	{
		derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    seen_free_[node.pos][node.offset] =
			        binary_op(node, std::as_const(seen_free_[node.pos])[node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (std::size_t i{}; N != i; ++i) {
				    seen_free_[pos][i] =
				        binary_op(Index(pos, i++), std::as_const(seen_free_[pos])[i]);
			    }
		    });
	}

	Node updateSeenFree(Node node, std::invocable<bool> auto unary_op,
	                    bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    seen_free_[node.pos][node.offset] =
			        unary_op(std::as_const(seen_free_[node.pos])[node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (std::size_t i{}; N != i; ++i) {
				    seen_free_[pos][i] = unary_op(std::as_const(seen_free_[pos])[i]);
			    }
		    },
		    propagate);
	}

	Node updateSeenFree(Node node, std::invocable<Index, bool> auto binary_op,
	                    bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    seen_free_[node.pos][node.offset] =
			        binary_op(node, std::as_const(seen_free_[node.pos])[node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (std::size_t i{}; N != i; ++i) {
				    seen_free_[pos][i] =
				        binary_op(Index(pos, i++), std::as_const(seen_free_[pos])[i]);
			    }
		    },
		    propagate);
	}

	Node updateSeenFree(Code code, std::invocable<bool> auto unary_op,
	                    bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, unary_op](Index node) {
			    seen_free_[node.pos][node.offset] =
			        unary_op(std::as_const(seen_free_[node.pos])[node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (std::size_t i{}; N != i; ++i) {
				    seen_free_[pos][i] = unary_op(std::as_const(seen_free_[pos])[i]);
			    }
		    },
		    propagate);
	}

	Node updateSeenFree(Code code, std::invocable<Index, bool> auto binary_op,
	                    bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, binary_op](Index node) {
			    seen_free_[node.pos][node.offset] =
			        binary_op(node, std::as_const(seen_free_[node.pos])[node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (std::size_t i{}; N != i; ++i) {
				    seen_free_[pos][i] =
				        binary_op(Index(pos, i++), std::as_const(seen_free_[pos])[i]);
			    }
		    },
		    propagate);
	}

	Node updateSeenFree(Key key, std::invocable<bool> auto unary_op, bool propagate = true)
	{
		return updateSeenFree(derived().toCode(key), unary_op, propagate);
	}

	Node updateSeenFree(Key key, std::invocable<Index, bool> auto binary_op,
	                    bool propagate = true)
	{
		return updateSeenFree(derived().toCode(key), binary_op, propagate);
	}

	Node updateSeenFree(Point coord, std::invocable<bool> auto unary_op,
	                    bool propagate = true, depth_t depth = 0)
	{
		return updateSeenFree(derived().toCode(coord, depth), unary_op, propagate);
	}

	Node updateSeenFree(Point coord, std::invocable<Index, bool> auto binary_op,
	                    bool propagate = true, depth_t depth = 0)
	{
		return updateSeenFree(derived().toCode(coord, depth), binary_op, propagate);
	}

	Node updateSeenFree(coord_t x, coord_t y, coord_t z, std::invocable<bool> auto unary_op,
	                    bool propagate = true, depth_t depth = 0)
	{
		return updateSeenFree(derived().toCode(x, y, z, depth), unary_op, propagate);
	}

	Node updateSeenFree(coord_t x, coord_t y, coord_t z,
	                    std::invocable<Index, bool> auto binary_op, bool propagate = true,
	                    depth_t depth = 0)
	{
		return updateSeenFree(derived().toCode(x, y, z, depth), binary_op, propagate);
	}

 protected:
	//
	// Constructors
	//

	SeenFreeMap() { seen_free_.emplace_back(); }

	SeenFreeMap(SeenFreeMap const& other) = default;

	SeenFreeMap(SeenFreeMap&& other) = default;

	template <class Derived2>
	SeenFreeMap(SeenFreeMap<Derived2, N> const& other) : seen_free_(other.seen_free_)
	{
	}

	template <class Derived2>
	SeenFreeMap(SeenFreeMap<Derived2, N>&& other) : seen_free_(std::move(other.seen_free_))
	{
	}

	//
	// Destructor
	//

	~SeenFreeMap() = default;

	//
	// Assignment operator
	//

	SeenFreeMap& operator=(SeenFreeMap const& rhs) = default;

	SeenFreeMap& operator=(SeenFreeMap&& rhs) = default;

	template <class Derived2>
	SeenFreeMap& operator=(SeenFreeMap<Derived2, N> const& rhs)
	{
		seen_free_ = rhs.seen_free_;
		return *this;
	}

	template <class Derived2>
	SeenFreeMap& operator=(SeenFreeMap<Derived2, N>&& rhs)
	{
		seen_free_ = std::move(rhs.seen_free_);
		return *this;
	}

	//
	// Swap
	//

	void swap(SeenFreeMap& other) noexcept { std::swap(seen_free_, other.seen_free_); }

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
		seen_free_.emplace_back(std::as_const(seen_free_[node.pos])[node.offset] ? -1 : 0);
	}

	//
	// Resize
	//

	void resize(std::size_t count)
	{
		// TODO: Look at
		seen_free_.resize(count);
	}

	//
	// Reserve
	//

	void reserveImpl(std::size_t new_cap) { seen_free_.reserve(new_cap); }

	//
	// Initialize root
	//

	void initRoot()
	{
		auto node = derived().rootIndex();
		seen_free_[node.pos].reset();
	}

	//
	// Fill
	//

	void fill(Index node, pos_t children)
	{
		seen_free_[children] =
		    BitSet<N>(std::as_const(seen_free_[node.pos])[node.offset] ? -1 : 0);
	}

	//
	// Clear
	//

	void clearImpl()
	{
		// TODO: Look at
		seen_free_.assign(1, BitSet<N>(0));
	}

	void clearImpl(pos_t nodes) {}

	//
	// Shrink to fit
	//

	void shrinkToFitImpl() { seen_free_.shrink_to_fit(); }

	//
	// Update node
	//

	void updateNode(Index node, pos_t children)
	{
		seen_free_[node.pos][node.offset] = seen_free_[children].all();
	}

	//
	// Is prunable
	//

	[[nodiscard]] bool isPrunable(pos_t nodes) const
	{
		return seen_free_[nodes].all() || seen_free_[nodes].none();
	}

	//
	// Memory node block
	//

	[[nodiscard]] static constexpr std::size_t memoryNodeBlock() noexcept
	{
		return sizeof(BitSet<N>);
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr MapType mapType() noexcept { return MapType::SEEN_FREE; }

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
			out.write(&seen_free_[pos], memoryNodeBlock());
		}
	}

	//
	// Dot file info
	//

	std::ostream& dotFileInfo(std::ostream& out, Index node) const
	{
		return out << "Seen free: " << seen_free_[node.pos][node.offset];
	}

 protected:
	Container<BitSet<N>> seen_free_;

	template <class Derived2, std::size_t N2>
	friend class SeenFreeMap;
};

//
// Concepts
//

template <class Map>
concept IsSeenFreeMap = IsMapType<Map, MapType::SEEN_FREE>;
}  // namespace ufo

#endif  // UFO_MAP_SEEN_FREE_MAP_HPP