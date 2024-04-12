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

#ifndef UFO_MAP_POINT_MAP_HPP
#define UFO_MAP_POINT_MAP_HPP

// UFO
#include <ufo/map/buffer.hpp>
#include <ufo/map/code.hpp>
#include <ufo/map/index.hpp>
#include <ufo/map/key.hpp>
#include <ufo/map/node.hpp>
#include <ufo/map/point.hpp>
#include <ufo/map/types.hpp>

// STL
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace ufo
{
template <class Derived, std::size_t N>
class PointMap
{
 public:
	//
	// Get point
	//

	[[nodiscard]] Point point(Index node) const { return point_[node.pos][node.offset]; }

	[[nodiscard]] Point point(Node node) const { return point(derived().index(node)); }

	[[nodiscard]] Point point(Code code) const { return point(derived().index(code)); }

	[[nodiscard]] Point point(Key key) const { return point(derived().index(key)); }

	[[nodiscard]] Point point(Point coord, depth_t depth = 0) const
	{
		return point(derived().index(coord, depth));
	}

	[[nodiscard]] Point point(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return point(derived().index(x, y, z, depth));
	}

	//
	// Has point
	//

	[[nodiscard]] bool hasPoint(Index node) const
	{
		return !std::isnan(point(node).x) || !std::isnan(point(node).y) ||
		       !std::isnan(point(node).z);
	}

	[[nodiscard]] bool hasPoint(Node node) const { return hasPoint(derived().index(node)); }

	[[nodiscard]] bool hasPoint(Code code) const { return hasPoint(derived().index(code)); }

	[[nodiscard]] bool hasPoint(Key key) const { return hasPoint(derived().index(key)); }

	[[nodiscard]] bool hasPoint(Point coord, depth_t depth = 0) const
	{
		return hasPoint(derived().index(coord, depth));
	}

	[[nodiscard]] bool hasPoint(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return hasPoint(derived().index(x, y, z, depth));
	}

	//
	//
	//

	// TODO: Add methods

 protected:
	//
	// Constructors
	//

	PointMap()
	{
		point_.emplace_back();
		point_.back().fill(Point(std::numeric_limits<coord_t>::quiet_NaN(),
		                         std::numeric_limits<coord_t>::quiet_NaN(),
		                         std::numeric_limits<coord_t>::quiet_NaN()));
	}

	PointMap(PointMap const& other) = default;

	PointMap(PointMap&& other) = default;

	template <class Derived2>
	PointMap(PointMap<Derived2, N> const& other) : point_(other.point_)
	{
	}

	template <class Derived2>
	PointMap(PointMap<Derived2, N>&& other) : point_(std::move(other.point_))
	{
	}

	//
	// Destructor
	//

	~PointMap() = default;

	//
	// Assignment operator
	//

	PointMap& operator=(PointMap const& rhs) = default;

	PointMap& operator=(PointMap&& rhs) = default;

	template <class Derived2>
	PointMap& operator=(PointMap<Derived2, N> const& rhs)
	{
		point_ = rhs.point_;
		return *this;
	}

	template <class Derived2>
	PointMap& operator=(PointMap<Derived2, N>&& rhs)
	{
		point_ = std::move(rhs.point_);
		return *this;
	}

	//
	// Swap
	//

	void swap(PointMap& other) noexcept { std::swap(point_, other.point_); }

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

	void createNodeBlock(Index)
	{
		point_.emplace_back();
		point_.back().fill(Point(std::numeric_limits<coord_t>::quiet_NaN(),
		                         std::numeric_limits<coord_t>::quiet_NaN(),
		                         std::numeric_limits<coord_t>::quiet_NaN()));
	}

	//
	// Resize
	//

	void resize(std::size_t count)
	{
		point_.resize(count, Point(std::numeric_limits<coord_t>::quiet_NaN(),
		                           std::numeric_limits<coord_t>::quiet_NaN(),
		                           std::numeric_limits<coord_t>::quiet_NaN()));
	}

	//
	// Reserve
	//

	void reserveImpl(std::size_t new_cap) { point_.reserve(new_cap); }

	//
	// Initialize root
	//

	void initRoot()
	{
		auto node                     = derived().rootIndex();
		point_[node.pos][node.offset] = {std::numeric_limits<coord_t>::quiet_NaN(),
		                                 std::numeric_limits<coord_t>::quiet_NaN(),
		                                 std::numeric_limits<coord_t>::quiet_NaN()};
	}

	//
	// Fill
	//

	void fill(Index node, pos_t children)
	{
		point_[children].fill(Point(std::numeric_limits<coord_t>::quiet_NaN(),
		                            std::numeric_limits<coord_t>::quiet_NaN(),
		                            std::numeric_limits<coord_t>::quiet_NaN()));
	}

	//
	// Clear
	//

	void clearImpl()
	{
		point_.assign(1, Point(std::numeric_limits<coord_t>::quiet_NaN(),
		                       std::numeric_limits<coord_t>::quiet_NaN(),
		                       std::numeric_limits<coord_t>::quiet_NaN()));
	}

	void clearImpl(pos_t nodes) {}

	//
	// Shrink to fit
	//

	void shrinkToFitImpl() { point_.shrink_to_fit(); }

	//
	// Update node
	//

	void updateNode(Index node, pos_t children)
	{
		Point       mean;
		std::size_t num{};
		for (auto p : point_[children]) {
			if (!std::isnan(p.x) || !std::isnan(p.y) || !std::isnan(p.z)) {
				mean += p;
				++num;
			}
		}

		if (num) {
			point_[node.pos][node.offset] = mean / num;
		} else {
			point_[node.pos][node.offset] = {std::numeric_limits<coord_t>::quiet_NaN(),
			                                 std::numeric_limits<coord_t>::quiet_NaN(),
			                                 std::numeric_limits<coord_t>::quiet_NaN()};
		}
	}

	//
	// Is prunable
	//

	[[nodiscard]] bool isPrunable(pos_t nodes) const
	{
		return std::all_of(std::begin(point_[nodes]), std::end(point_[nodes]), [](auto p) {
			return std::isnan(p.x) && std::isnan(p.y) && std::isnan(p.z);
		});
	}

	//
	// Memory node block
	//

	[[nodiscard]] static constexpr std::size_t memoryNodeBlock() noexcept
	{
		return sizeof(DataBlock<Point, N>);
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr MapType mapType() noexcept { return MapType::POINT; }

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
			out.write(&point_[pos], memoryNodeBlock());
		}
	}

	//
	// Dot file info
	//

	std::ostream& dotFileInfo(std::ostream& out, Index node) const
	{
		return out << "Point: " << point(node);
	}

 protected:
	Container<DataBlock<Point, N>> point_;

	template <class Derived2, std::size_t N2>
	friend class PointMap;
};

//
// Concepts
//

template <class Map>
concept IsPointMap = IsMapType<Map, MapType::POINT>;
}  // namespace ufo

#endif  // UFO_MAP_POINT_MAP_HPP