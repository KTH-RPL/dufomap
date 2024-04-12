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

#ifndef UFO_MAP_COLOR_MAP_HPP
#define UFO_MAP_COLOR_MAP_HPP

// UFO
#include <ufo/map/io.hpp>
#include <ufo/map/node.hpp>
#include <ufo/map/point.hpp>
#include <ufo/map/types.hpp>

// STL
#include <algorithm>
#include <array>
#include <iostream>
#include <vector>

namespace ufo
{
template <class Derived, std::size_t N>
class ColorMap
{
 public:
	//
	// Get color
	//

	[[nodiscard]] Color color(Index node) const { return color_[node.pos][node.offset]; }

	[[nodiscard]] Color color(Node node) const { return color(derived().index(node)); }

	[[nodiscard]] Color color(Code code) const { return color(derived().index(code)); }

	[[nodiscard]] Color color(Key key) const { return color(derived().index(key)); }

	[[nodiscard]] Color color(Point coord, depth_t depth = 0) const
	{
		return color(derived().index(coord, depth));
	}

	[[nodiscard]] Color color(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return color(derived().index(x, y, z, depth));
	}

	//
	// Set color
	//

	void setColor(Index node, Color value)
	{
		derived().apply(
		    node, [this, value](Index node) { color_[node.pos][node.offset] = value; },
		    [this, value](pos_t pos) { color_[pos].fill(value); });
	}

	void setColor(Index node, color_t red, color_t green, color_t blue)
	{
		setColor(node, Color(red, green, blue));
	}

	Node setColor(Node node, Color value, bool propagate = true)
	{
		return derived().apply(
		    node, [this, value](Index node) { color_[node.pos][node.offset] = value; },
		    [this, value](pos_t pos) { color_[pos].fill(value); }, propagate);
	}

	Node setColor(Node node, color_t red, color_t green, color_t blue,
	              bool propagate = true)
	{
		return setColor(node, Color(red, green, blue), propagate);
	}

	Node setColor(Code code, Color value, bool propagate = true)
	{
		return derived().apply(
		    code, [this, value](Index node) { color_[node.pos][node.offset] = value; },
		    [this, value](pos_t pos) { color_[pos].fill(value); }, propagate);
	}

	Node setColor(Code code, color_t red, color_t green, color_t blue,
	              bool propagate = true)
	{
		return setColor(code, Color(red, green, blue), propagate);
	}

	Node setColor(Key key, Color value, bool propagate = true)
	{
		return setColor(derived().toCode(key), value, propagate);
	}

	Node setColor(Key key, color_t red, color_t green, color_t blue, bool propagate = true)
	{
		return setColor(derived().toCode(key), red, green, blue, propagate);
	}

	Node setColor(Point coord, Color value, bool propagate = true, depth_t depth = 0)
	{
		return setColor(derived().toCode(coord, depth), value, propagate);
	}

	Node setColor(Point coord, color_t red, color_t green, color_t blue,
	              bool propagate = true, depth_t depth = 0)
	{
		return setColor(derived().toCode(coord, depth), red, green, blue, propagate);
	}

	Node setColor(coord_t x, coord_t y, coord_t z, Color value, bool propagate = true,
	              depth_t depth = 0)
	{
		return setColor(derived().toCode(x, y, z, depth), value, propagate);
	}

	Node setColor(coord_t x, coord_t y, coord_t z, color_t red, color_t green, color_t blue,
	              bool propagate = true, depth_t depth = 0)
	{
		return setColor(derived().toCode(x, y, z, depth), red, green, blue, propagate);
	}

	//
	// Update color
	//

	void updateColor(Index node, std::invocable<Color> auto unary_op)
	{
		derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    color_[node.pos][node.offset] = unary_op(color_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : color_[pos]) {
				    e = unary_op(e);
			    }
		    });
	}

	void updateColor(Index node, std::invocable<Index, Color> auto binary_op)
	{
		derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    color_[node.pos][node.offset] = binary_op(node, color_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (offset_t i{}; auto& e : color_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    });
	}

	Node updateColor(Node node, std::invocable<Color> auto unary_op, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    color_[node.pos][node.offset] = unary_op(color_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : color_[pos]) {
				    e = unary_op(e);
			    }
		    },
		    propagate);
	}

	Node updateColor(Node node, std::invocable<Index, Color> auto binary_op,
	                 bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    color_[node.pos][node.offset] = binary_op(node, color_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (offset_t i{}; auto& e : color_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    },
		    propagate);
	}

	Node updateColor(Code code, std::invocable<Color> auto unary_op, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, unary_op](Index node) {
			    color_[node.pos][node.offset] = unary_op(color_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : color_[pos]) {
				    e = unary_op(e);
			    }
		    },
		    propagate);
	}

	Node updateColor(Code code, std::invocable<Index, Color> auto binary_op,
	                 bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, binary_op](Index node) {
			    color_[node.pos][node.offset] = binary_op(node, color_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (offset_t i{}; auto& e : color_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    },
		    propagate);
	}

	Node updateColor(Key key, std::invocable<Color> auto unary_op, bool propagate = true)
	{
		return updateColor(derived().toCode(key), unary_op, propagate);
	}

	Node updateColor(Key key, std::invocable<Index, Color> auto binary_op,
	                 bool propagate = true)
	{
		return updateColor(derived().toCode(key), binary_op, propagate);
	}

	Node updateColor(Point coord, std::invocable<Color> auto unary_op,
	                 bool propagate = true, depth_t depth = 0)
	{
		return updateColor(derived().toCode(coord, depth), unary_op, propagate);
	}

	Node updateColor(Point coord, std::invocable<Index, Color> auto binary_op,
	                 bool propagate = true, depth_t depth = 0)
	{
		return updateColor(derived().toCode(coord, depth), binary_op, propagate);
	}

	Node updateColor(coord_t x, coord_t y, coord_t z, std::invocable<Color> auto unary_op,
	                 bool propagate = true, depth_t depth = 0)
	{
		return updateColor(derived().toCode(x, y, z, depth), unary_op, propagate);
	}

	Node updateColor(coord_t x, coord_t y, coord_t z,
	                 std::invocable<Index, Color> auto binary_op, bool propagate = true,
	                 depth_t depth = 0)
	{
		return updateColor(derived().toCode(x, y, z, depth), binary_op, propagate);
	}

	//
	// Has color
	//

	[[nodiscard]] bool hasColor(Index node) const
	{
		return color_[node.pos][node.offset].isSet();
	}

	[[nodiscard]] bool hasColor(Node node) const { return hasColor(derived().index(node)); }

	[[nodiscard]] bool hasColor(Code code) const { return hasColor(derived().index(code)); }

	[[nodiscard]] bool hasColor(Key key) const { return hasColor(derived().index(key)); }

	[[nodiscard]] bool hasColor(Point coord, depth_t depth = 0) const
	{
		return hasColor(derived().index(coord, depth));
	}

	[[nodiscard]] bool hasColor(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return hasColor(derived().index(x, y, z, depth));
	}

	//
	// Clear color
	//

	void clearColor(Index index) { setColor(index, Color()); }

	void clearColor(Node node, bool propagate = true)
	{
		setColor(node, Color(), propagate);
	}

	void clearColor(Code code, bool propagate = true)
	{
		setColor(code, Color(), propagate);
	}

	void clearColor(Key key, bool propagate = true)
	{
		clearColor(derived().toCode(key), propagate);
	}

	void clearColor(Point coord, bool propagate = true, depth_t depth = 0)
	{
		clearColor(derived().toCode(coord, depth), propagate);
	}

	void clearColor(coord_t x, coord_t y, coord_t z, bool propagate = true,
	                depth_t depth = 0)
	{
		clearColor(derived().toCode(x, y, z, depth), propagate);
	}

 protected:
	//
	// Constructors
	//

	ColorMap() { color_.emplace_back(); }

	ColorMap(ColorMap const&) = default;

	ColorMap(ColorMap&&) = default;

	template <class Derived2>
	ColorMap(ColorMap<Derived2, N> const& other) : color_(other.color_)
	{
	}

	template <class Derived2>
	ColorMap(ColorMap<Derived2, N>&& other) : color_(std::move(other.color_))
	{
	}

	//
	//
	//

	~ColorMap() = default;

	//
	// Assignment operator
	//

	ColorMap& operator=(ColorMap const&) = default;

	ColorMap& operator=(ColorMap&&) = default;

	template <class Derived2>
	ColorMap& operator=(ColorMap<Derived2, N> const& rhs)
	{
		color_ = rhs.color_;
		return *this;
	}

	template <class Derived2>
	ColorMap& operator=(ColorMap<Derived2, N>&& rhs)
	{
		color_ = std::move(rhs.color_);
		return *this;
	}

	//
	// Swap
	//

	void swap(ColorMap& other) noexcept { std::swap(color_, other.color_); }

	//
	// Derived
	//

	[[nodiscard]] constexpr Derived& derived() { return *static_cast<Derived*>(this); }

	[[nodiscard]] constexpr Derived const& derived() const
	{
		return *static_cast<Derived const*>(this);
	}

	//
	// Initialize root
	//

	void initRoot()
	{
		auto node = derived().rootIndex();
		color_[node.pos][node.offset].clear();
	}

	//
	// Create node block
	//

	void createNodeBlock(Index node)
	{
		color_.emplace_back();
		color_.back().fill(color_[node.pos][node.offset]);
	}

	//
	// Resize
	//

	void resize(std::size_t count)
	{
		// TODO: Implement
		color_.resize(count);
	}

	//
	// Reserve
	//

	void reserveImpl(std::size_t new_cap) { color_.reserve(new_cap); }

	//
	// Fill
	//

	void fill(Index node, pos_t children)
	{
		color_[children].fill(color_[node.pos][node.offset]);
	}

	//
	// Clear
	//

	void clearImpl() { color_.resize(1); }

	void clearImpl(pos_t nodes) {}

	//
	// Shrink to fit
	//

	void shrinkToFitImpl() { color_.shrink_to_fit(); }

	//
	// Update node
	//

	void updateNode(Index node, pos_t children)
	{
		std::uint16_t red{}, green{}, blue{}, num{};
		for (std::size_t i{}; N != i; ++i) {
			red += color_[children][i].red;
			green += color_[children][i].green;
			blue += color_[children][i].blue;
			num += color_[children][i].isSet();
		}

		num                                 = num ? num : 1;
		color_[node.pos][node.offset].red   = static_cast<color_t>(red / num);
		color_[node.pos][node.offset].green = static_cast<color_t>(green / num);
		color_[node.pos][node.offset].blue  = static_cast<color_t>(blue / num);
	}

	//
	// Is collapsible
	//

	[[nodiscard]] bool isPrunable(pos_t nodes) const
	{
		return std::all_of(std::cbegin(color_[nodes]) + 1, std::cend(color_[nodes]),
		                   [a = color_[nodes].front()](auto b) { return a == b; });
	}

	//
	// Memory node block
	//

	[[nodiscard]] static constexpr std::size_t memoryNodeBlock() noexcept
	{
		return N * sizeof(Color);
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr MapType mapType() noexcept { return MapType::COLOR; }

	std::size_t serializedSize(std::ranges::input_range auto r) const
	{
		return std::ranges::size(r) * memoryNodeBlock();
	}

	void readNodes(ReadBuffer& in, std::ranges::input_range auto r)
	{
		for (auto const [pos, offsets] : r) {
			if (offsets.all()) {
				in.read(color_[pos].data(), memoryNodeBlock());
			} else {
				DataBlock<Color, N> color;
				in.read(color.data(), memoryNodeBlock());
				for (offset_t i{}; N != i; ++i) {
					color_[pos][i] = offsets[i] ? color[i] : color_[pos][i];
				}
			}
		}
	}

	void writeNodes(WriteBuffer& out, std::ranges::input_range auto r) const
	{
		out.reserve(out.size() + serializedSize(r));
		for (auto pos : r) {
			out.write(color_[pos].data(), memoryNodeBlock());
		}
	}

	//
	// Dot file info
	//

	std::ostream& dotFileInfo(std::ostream& out, Index node) const
	{
		return out << color_[node.pos][node.offset];
	}

 protected:
	// Data
	Container<DataBlock<Color, N>> color_;

	template <class Derived2, std::size_t N2>
	friend class ColorMap;
};

//
// Concepts
//

template <class Map>
concept IsColorMap = IsMapType<Map, MapType::COLOR>;
}  // namespace ufo

#endif  // UFO_MAP_COLOR_MAP_HPP