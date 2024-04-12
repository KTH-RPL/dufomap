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

#ifndef UFO_MAP_LABEL_MAP_HPP
#define UFO_MAP_LABEL_MAP_HPP

// UFO
#include <ufo/map/buffer.hpp>
#include <ufo/map/code.hpp>
#include <ufo/map/index.hpp>
#include <ufo/map/key.hpp>
#include <ufo/map/node.hpp>
#include <ufo/map/point.hpp>
#include <ufo/map/types.hpp>

namespace ufo
{
template <class Derived, std::size_t N>
class LabelMap
{
 public:
	//
	// Get label
	//

	[[nodiscard]] label_t label(Index node) const { return label_[node.pos][node.offset]; }

	[[nodiscard]] label_t label(Node node) const { return label(derived().index(node)); }

	[[nodiscard]] label_t label(Code code) const { return label(derived().index(code)); }

	[[nodiscard]] label_t label(Key key) const { return label(derived().index(key)); }

	[[nodiscard]] label_t label(Point coord, depth_t depth = 0) const
	{
		return label(derived().index(coord, depth));
	}

	[[nodiscard]] label_t label(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return label(derived().index(x, y, z, depth));
	}

	//
	// Set label
	//

	void setLabel(Index node, label_t value)
	{
		derived().apply(
		    node, [this, value](Index node) { label_[node.pos][node.offset] = value; },
		    [this, value](pos_t pos) { label_[pos].fill(value); });
	}

	Node setLabel(Node node, label_t value, bool propagate = true)
	{
		return derived().apply(
		    node, [this, value](Index node) { label_[node.pos][node.offset] = value; },
		    [this, value](pos_t pos) { label_[pos].fill(value); }, propagate);
	}

	Node setLabel(Code code, label_t value, bool propagate = true)
	{
		return derived().apply(
		    code, [this, value](Index node) { label_[node.pos][node.offset] = value; },
		    [this, value](pos_t pos) { label_[pos].fill(value); }, propagate);
	}

	Node setLabel(Key key, label_t value, bool propagate = true)
	{
		return setLabel(derived().toCode(key), value, propagate);
	}

	Node setLabel(Point coord, label_t value, bool propagate = true, depth_t depth = 0)
	{
		return setLabel(derived().toCode(coord, depth), value, propagate);
	}

	Node setLabel(coord_t x, coord_t y, coord_t z, label_t value, bool propagate = true,
	              depth_t depth = 0)
	{
		return setLabel(derived().toCode(x, y, z, depth), value, propagate);
	}

	//
	// Update label
	//

	void updateLabel(Index node, std::invocable<label_t> auto unary_op)
	{
		derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    label_[node.pos][node.offset] = unary_op(label_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : label_[pos]) {
				    e = unary_op(e);
			    }
		    });
	}

	void updateLabel(Index node, std::invocable<Index, label_t> auto binary_op)
	{
		derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    label_[node.pos][node.offset] = binary_op(node, label_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (std::size_t i{}; auto& e : label_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    });
	}

	Node updateLabel(Node node, std::invocable<label_t> auto unary_op,
	                 bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    label_[node.pos][node.offset] = unary_op(label_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : label_[pos]) {
				    e = unary_op(e);
			    }
		    },
		    propagate);
	}

	Node updateLabel(Node node, std::invocable<Index, label_t> auto binary_op,
	                 bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    label_[node.pos][node.offset] = binary_op(node, label_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (std::size_t i{}; auto& e : label_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    },
		    propagate);
	}

	Node updateLabel(Code code, std::invocable<label_t> auto unary_op,
	                 bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, unary_op](Index node) {
			    label_[node.pos][node.offset] = unary_op(label_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : label_[pos]) {
				    e = unary_op(e);
			    }
		    },
		    propagate);
	}

	Node updateLabel(Code code, std::invocable<Index, label_t> auto binary_op,
	                 bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, binary_op](Index node) {
			    label_[node.pos][node.offset] = binary_op(node, label_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (std::size_t i{}; auto& e : label_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    },
		    propagate);
	}

	Node updateLabel(Key key, std::invocable<label_t> auto unary_op, bool propagate = true)
	{
		return updateLabel(derived().toCode(key), unary_op, propagate);
	}

	Node updateLabel(Key key, std::invocable<Index, label_t> auto binary_op,
	                 bool propagate = true)
	{
		return updateLabel(derived().toCode(key), binary_op, propagate);
	}

	Node updateLabel(Point coord, std::invocable<label_t> auto unary_op,
	                 bool propagate = true, depth_t depth = 0)
	{
		return updateLabel(derived().toCode(coord, depth), unary_op, propagate);
	}

	Node updateLabel(Point coord, std::invocable<Index, label_t> auto binary_op,
	                 bool propagate = true, depth_t depth = 0)
	{
		return updateLabel(derived().toCode(coord, depth), binary_op, propagate);
	}

	Node updateLabel(coord_t x, coord_t y, coord_t z, std::invocable<label_t> auto unary_op,
	                 bool propagate = true, depth_t depth = 0)
	{
		return updateLabel(derived().toCode(x, y, z, depth), unary_op, propagate);
	}

	Node updateLabel(coord_t x, coord_t y, coord_t z,
	                 std::invocable<Index, label_t> auto binary_op, bool propagate = true,
	                 depth_t depth = 0)
	{
		return updateLabel(derived().toCode(x, y, z, depth), binary_op, propagate);
	}

 protected:
	//
	// Constructors
	//

	LabelMap() { label_.emplace_back(); }

	LabelMap(LabelMap const& other) = default;

	LabelMap(LabelMap&& other) = default;

	template <class Derived2>
	LabelMap(LabelMap<Derived2, N> const& other) : label_(other.label_)
	{
	}

	template <class Derived2>
	LabelMap(LabelMap<Derived2, N>&& other) : label_(std::move(other.label_))
	{
	}

	//
	// Destructor
	//

	~LabelMap() = default;

	//
	// Assignment operator
	//

	LabelMap& operator=(LabelMap const& rhs) = default;

	LabelMap& operator=(LabelMap&& rhs) = default;

	template <class Derived2>
	LabelMap& operator=(LabelMap<Derived2, N> const& rhs)
	{
		label_ = rhs.label_;
		return *this;
	}

	template <class Derived2>
	LabelMap& operator=(LabelMap<Derived2, N>&& rhs)
	{
		label_ = std::move(rhs.label_);
		return *this;
	}

	//
	// Swap
	//

	void swap(LabelMap& other) noexcept { std::swap(label_, other.label_); }

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
		label_.emplace_back();
		label_.back().fill(label_[node.pos][node.offset]);
	}

	//
	// Resize
	//

	void resize(std::size_t count) { label_.resize(count, DataBlock<label_t, N>{}); }

	//
	// Reserve
	//

	void reserveImpl(std::size_t new_cap) { label_.reserve(new_cap); }

	//
	// Initialize root
	//

	void initRoot()
	{
		auto node                     = derived().rootIndex();
		label_[node.pos][node.offset] = 0;
	}

	//
	// Fill
	//

	void fill(Index node, pos_t children)
	{
		label_[children].fill(label_[node.pos][node.offset]);
	}

	//
	// Clear
	//

	void clearImpl() { label_.assign(1, 0); }

	void clearImpl(pos_t nodes) {}

	//
	// Shrink to fit
	//

	void shrinkToFitImpl() { label_.shrink_to_fit(); }

	//
	// Update node
	//

	void updateNode(Index node, pos_t children)
	{
		label_t l{};
		for (auto e : label_[children]) {
			l |= e;
		}
		label_[node.pos][node.offset] = l;
	}

	//
	// Is prunable
	//

	[[nodiscard]] bool isPrunable(pos_t nodes) const
	{
		return std::all_of(std::cbegin(label_[nodes]) + 1, std::cend(label_[nodes]),
		                   [a = label_[nodes].front()](auto b) { return a == b; });
	}

	//
	// Memory node block
	//

	[[nodiscard]] static constexpr std::size_t memoryNodeBlock() noexcept
	{
		return N * sizeof(label_t);
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr MapType mapType() noexcept { return MapType::LABEL; }

	constexpr std::size_t serializedSize(std::ranges::input_range auto r) const
	{
		return std::ranges::size(r) * memoryNodeBlock();
	}

	void readNodes(ReadBuffer& in, std::ranges::input_range auto r)
	{
		for (auto const [pos, offsets] : r) {
			if (offsets.all()) {
				in.read(label_[pos].data(), memoryNodeBlock());
			} else {
				DataBlock<label_t, N> label;
				in.read(label.data(), memoryNodeBlock());
				for (offset_t i{}; N != i; ++i) {
					label_[pos][i] = offsets[i] ? label[i] : label_[pos][i];
				}
			}
		}
	}

	void writeNodes(WriteBuffer& out, std::ranges::input_range auto r) const
	{
		out.reserve(out.size() + serializedSize(r));
		for (auto pos : r) {
			out.write(label_[pos].data(), memoryNodeBlock());
		}
	}

	//
	// Dot file info
	//

	std::ostream& dotFileInfo(std::ostream& out, Index node) const
	{
		return out << "Label: " << label_[node.pos][node.offset];
	}

 protected:
	Container<DataBlock<label_t, N>> label_;

	template <class Derived2, std::size_t N2>
	friend class LabelMap;
};

//
// Concepts
//

template <class Map>
concept IsLabelMap = IsMapType<Map, MapType::LABEL>;
}  // namespace ufo

#endif  // UFO_MAP_LABEL_MAP_HPP