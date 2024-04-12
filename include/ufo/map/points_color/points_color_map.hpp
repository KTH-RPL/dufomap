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

#ifndef UFO_MAP_POINTS_COLOR_MAP_HPP
#define UFO_MAP_POINTS_COLOR_MAP_HPP

// UFO
#include <ufo/container/small_vector.hpp>
#include <ufo/map/buffer.hpp>
#include <ufo/map/code.hpp>
#include <ufo/map/color/color.hpp>
#include <ufo/map/index.hpp>
#include <ufo/map/key.hpp>
#include <ufo/map/node.hpp>
#include <ufo/map/point.hpp>
#include <ufo/map/point_cloud.hpp>
#include <ufo/map/points/points_map.hpp>
#include <ufo/map/types.hpp>

// STL
#include <algorithm>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <numeric>
#include <ranges>
#include <type_traits>

namespace ufo
{
template <typename T, class Derived, std::size_t N>
class PointsColorMap
{
 private:
	using size_t = std::conditional_t<sizeof(T) <= sizeof(std::uint16_t), std::uint16_t,
	                                  std::uint32_t>;

	static constexpr coord_t POINTS_MAX =
	    static_cast<coord_t>(std::numeric_limits<T>::max());

	union Data {
		Data(std::size_t data = 0) : data(data) {}

		Data(Data const& rhs) : data(rhs.data) {}

		Data& operator=(Data const& other)
		{
			data = other.data;
			return *this;
		}

		~Data() {}

		std::size_t                                                          data{};
		SmallVector<color_t, 3 + 3 * sizeof(T) / sizeof(color_t), N, size_t> points;
	};

 public:
	//
	// Get points
	//

	[[nodiscard]] PointCloudColor points() const { return points(derived().rootIndex()); }

	[[nodiscard]] PointCloudColor points(Index node) const
	{
		PointCloudColor pc;
		if (PointsPropagationCriteria::NUM_POINTS == pointsPropagationCriteria()) {
			pc.reserve(numPoints(node));
		}

		auto hs     = static_cast<coord_t>(derived().size() / 2);
		auto center = derived().center(node);
		auto depth  = derived().depth(node);

		if (derived().isLeaf(node)) {
			if (!derived().isPureLeaf(node)) {
				return pc;
			}

			for (auto e : data_[node.pos].points.query(node.offset)) {
				T* p = reinterpret_cast<T*>(&e[3]);
				pc.emplace_back(center[0] + hs * p[0] / POINTS_MAX,
				                center[1] + hs * p[1] / POINTS_MAX,
				                center[2] + hs * p[2] / POINTS_MAX, e[0], e[1], e[2]);
			}
		} else {
			std::array<std::pair<Index, Point>, Derived::maxDepthLevels()> nodes;
			nodes[depth].first      = node;
			nodes[depth].second     = center;
			nodes[depth - 1].first  = derived().child(node, 0);
			nodes[depth - 1].second = derived().childCenter(
			    center, static_cast<coord_t>(derived().size(depth - 1) / 2), 0);

			for (depth_t d = depth - 1; depth != d;) {
				auto [n, c] = nodes[d];

				if (0 == d) {
					for (auto e : data_[n.pos].points.query(n.offset)) {
						T* p = reinterpret_cast<T*>(&e[3]);
						pc.emplace_back(c[0] + hs * p[0] / POINTS_MAX, c[1] + hs * p[1] / POINTS_MAX,
						                c[2] + hs * p[2] / POINTS_MAX, e[0], e[1], e[2]);
					}
				} else if (derived().isParent(n)) {
					--d;
					nodes[d].first  = derived().child(n, 0);
					nodes[d].second = derived().childCenter(c, static_cast<coord_t>(derived().size(d) / 2), 0);
					continue;
				}

				while (7 < ++nodes[d].first.offset) {
					++d;
				}
				if (depth <= d) {
					break;
				}
				nodes[d].second = derived().childCenter(
				    nodes[d + 1].second, static_cast<coord_t>(derived().size(d) / 2), nodes[d].first.offset);
			}
		}

		return pc;
	}

	[[nodiscard]] PointCloudColor points(Node node) const
	{
		return points(derived().index(node));
	}

	[[nodiscard]] PointCloudColor points(Code code) const
	{
		return points(derived().index(code));
	}

	[[nodiscard]] PointCloudColor points(Key key) const
	{
		return points(derived().index(key));
	}

	[[nodiscard]] PointCloudColor points(Point coord, depth_t depth = 0) const
	{
		return points(derived().index(coord, depth));
	}

	[[nodiscard]] PointCloudColor points(coord_t x, coord_t y, coord_t z,
	                                     depth_t depth = 0) const
	{
		return points(derived().index(x, y, z, depth));
	}

	//
	// Has points
	//

	[[nodiscard]] bool hasPoints() const { return hasPoints(derived().rootIndex()); }

	[[nodiscard]] bool hasPoints(Index node) const
	{
		if (derived().isParent(node)) {
			switch (pointsPropagationCriteria()) {
				case PointsPropagationCriteria::HAS_POINTS:
					return data_[node.pos].data & (std::size_t(1) << node.offset);
				case PointsPropagationCriteria::NUM_POINTS: {
					auto children = derived().children(node);
					return derived().isPureLeaves(children) ? !data_[children].points.empty()
					                                        : 0 != data_[children].data;
				}
				case PointsPropagationCriteria::NONE: {
					std::array<Index, Derived::maxDepthLevels()> nodes;
					nodes[1] = node;
					for (std::size_t i{1}; 0 != i;) {
						auto n = nodes[i];
						i -= 7 < ++nodes[i].offset;
						if (derived().isParent(n)) {
							nodes[++i] = derived().child(n, 0);
						} else if (derived().isPureLeaf(n)) {
							if (!data_[n.pos].points.empty()) {
								return true;
							} else {
								--i;
							}
						}
					}
				}
			}
		} else if (derived().isPureLeaf(node)) {
			return !data_[node.pos].points.empty(node.offset);
		}

		return false;
	}

	[[nodiscard]] bool hasPoints(Node node) const
	{
		return hasPoints(derived().index(node));
	}

	[[nodiscard]] bool hasPoints(Code code) const
	{
		return hasPoints(derived().index(code));
	}

	[[nodiscard]] bool hasPoints(Key key) const { return hasPoints(derived().index(key)); }

	[[nodiscard]] bool hasPoints(Point coord, depth_t depth = 0) const
	{
		return hasPoints(derived().index(coord, depth));
	}

	[[nodiscard]] bool hasPoints(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return hasPoints(derived().index(x, y, z, depth));
	}

	//
	// Num points
	//

	[[nodiscard]] std::size_t numPoints() const { return numPoints(derived().rootIndex()); }

	[[nodiscard]] std::size_t numPoints(Index node) const
	{
		if (derived().isParent(node)) {
			switch (pointsPropagationCriteria()) {
				case PointsPropagationCriteria::HAS_POINTS: {
					if (0 == (data_[node.pos].data & (std::size_t(1) << node.offset))) {
						return false;
					}
					std::size_t                                  num_points{};
					std::array<Index, Derived::maxDepthLevels()> nodes;
					nodes[1] = derived().child(node, 0);
					for (std::size_t i{1}; 0 != i;) {
						auto n = nodes[i];
						i -= 7 < ++nodes[i].offset;
						if (derived().isParent(n) &&
						    (data_[n.pos].data & (std::size_t(1) << n.offset))) {
							nodes[++i] = derived().child(n, 0);
						} else if (derived().isPureLeaf(n)) {
							num_points += data_[n.pos].points.size();
							--i;
						}
					}
					return num_points;
				}
				case PointsPropagationCriteria::NUM_POINTS: {
					auto children = derived().children(node);
					return derived().isPureLeaves(children) ? data_[children].points.size()
					                                        : data_[children].data;
				}
				case PointsPropagationCriteria::NONE: {
					std::size_t                                  num_points{};
					std::array<Index, Derived::maxDepthLevels()> nodes;
					nodes[1] = derived().child(node, 0);
					for (std::size_t i{1}; 0 != i;) {
						auto n = nodes[i];
						i -= 7 < ++nodes[i].offset;
						if (derived().isParent(n)) {
							nodes[++i] = derived().child(n, 0);
						} else if (derived().isPureLeaf(n)) {
							num_points += data_[n.pos].points.size();
							--i;
						}
					}
					return num_points;
				}
			}
		} else if (derived().isPureLeaf(node)) {
			return data_[node.pos].points.size(node.offset);
		}

		return 0;
	}

	[[nodiscard]] std::size_t numPoints(Node node) const
	{
		return numPoints(derived().index(node));
	}

	[[nodiscard]] std::size_t numPoints(Code code) const
	{
		return numPoints(derived().index(code));
	}

	[[nodiscard]] std::size_t numPoints(Key key) const
	{
		return numPoints(derived().index(key));
	}

	[[nodiscard]] std::size_t numPoints(Point coord, depth_t depth = 0) const
	{
		return numPoints(derived().index(coord, depth));
	}

	[[nodiscard]] std::size_t numPoints(coord_t x, coord_t y, coord_t z,
	                                    depth_t depth = 0) const
	{
		return numPoints(derived().index(x, y, z, depth));
	}

	//
	// Assign points
	//

	void assignPoints(Index node, PointColor point)
	{
		assert(derived().isPureLeaf(node));

		data_[node.pos].points.resize(node.offset, 1);

		auto  c  = derived().center(node);
		auto  hs = static_cast<coord_t>(derived().size() / 2);
		auto& e  = data_[node.pos].points(node.offset, 0);
		e[0]     = point.red;
		e[1]     = point.green;
		e[2]     = point.blue;
		T* p     = reinterpret_cast<T*>(&e[3]);
		p[0]     = static_cast<T>(POINTS_MAX * (point.x - c.x) / hs);
		p[1]     = static_cast<T>(POINTS_MAX * (point.y - c.y) / hs);
		p[2]     = static_cast<T>(POINTS_MAX * (point.z - c.z) / hs);
	}

	void assignPoints(Index node, std::input_iterator auto first,
	                  std::input_iterator auto last)
	{
		assert(derived().isPureLeaf(node));

		data_[node.pos].points.resize(node.offset, std::distance(first, last));

		auto c  = derived().center(node);
		auto hs = static_cast<coord_t>(derived().size() / 2);
		for (auto& e : data_[node.pos].points.query(node.offset)) {
			e[0] = first->red;
			e[1] = first->green;
			e[2] = first->blue;
			T* p = reinterpret_cast<T*>(&e[3]);
			p[0] = static_cast<T>(POINTS_MAX * (first->x - c.x) / hs);
			p[1] = static_cast<T>(POINTS_MAX * (first->y - c.y) / hs);
			p[2] = static_cast<T>(POINTS_MAX * (first->z - c.z) / hs);
			++first;
		}
	}

	void assignPoints(Index node, std::ranges::input_range auto points)
	{
		assignPoints(node, std::cbegin(points), std::cend(points));
	}

	//
	// Insert points
	//

	void insertPoints(Index node, PointColor point)
	{
		assert(derived().isPureLeaf(node));

		auto cur_size = data_[node.pos].points.size(node.offset);
		data_[node.pos].points.resize(node.offset, cur_size + 1);

		auto c   = derived().center(node);
		auto hs  = static_cast<coord_t>(derived().size() / 2);
		auto it  = data_[node.pos].points.begin(node.offset) + cur_size;
		(*it)[0] = point.red;
		(*it)[1] = point.green;
		(*it)[2] = point.blue;
		T* p     = reinterpret_cast<T*>(&(*it)[3]);
		p[0]     = static_cast<T>(POINTS_MAX * (point.x - c.x) / hs);
		p[1]     = static_cast<T>(POINTS_MAX * (point.y - c.y) / hs);
		p[2]     = static_cast<T>(POINTS_MAX * (point.z - c.z) / hs);
	}

	template <std::input_iterator I, std::sentinel_for<I> S>
	void insertPoints(Index node, I first, S last)
	{
		assert(derived().isPureLeaf(node));

		auto cur_size = data_[node.pos].points.size(node.offset);
		auto in_size  = std::distance(first, last);
		data_[node.pos].points.resize(node.offset, cur_size + in_size);

		auto c  = derived().center(node);
		auto hs = static_cast<coord_t>(derived().size() / 2);
		auto it = data_[node.pos].points.begin(node.offset) + cur_size;
		for (; first != last; ++first, ++it) {
			(*it)[0] = first->red;
			(*it)[1] = first->green;
			(*it)[2] = first->blue;
			T* p     = reinterpret_cast<T*>(&(*it)[3]);
			p[0]     = static_cast<T>(POINTS_MAX * (first->x - c.x) / hs);
			p[1]     = static_cast<T>(POINTS_MAX * (first->y - c.y) / hs);
			p[2]     = static_cast<T>(POINTS_MAX * (first->z - c.z) / hs);
		}
	}

	template <std::ranges::input_range R>
	void insertPoints(Index node, R&& points)
	{
		insertPoints(node, std::ranges::begin(points), std::ranges::end(points));
	}

	void insertPoints(PointColor point, bool propagate = true)
	{
		auto node = derived().createIndex(point);

		derived().setModified(node);

		insertPoints(node, point);

		if (propagate) {
			derived().propagateModified();
		}
	}

	template <std::input_iterator I, std::sentinel_for<I> S>
	void insertPoints(I first, S last, bool propagate = true)
	{
		// FIXME: Optimize

		CodeUnorderedMap<PointCloudColor> points;
		for (; first != last; ++first) {
			points[derived().toCode(*first)].push_back(*first);
		}

		for (auto const& [c, p] : points) {
			insertPoints(derived().createIndex(c), p);
		}

		if (propagate) {
			derived().propagateModified();
		}
	}

	void insertPoints(std::ranges::input_range auto points, bool propagate = true)
	{
		insertPoints(std::cbegin(points), std::cend(points), propagate);
	}

	//
	// Clear points
	//

	void clearPoints(bool propagate = true)
	{
		clearPoints(derived().rootCode(), propagate);
	}

	void clearPoints(Index node)
	{
		derived().apply(
		    node,
		    [this](Index node) {
			    if (derived().isPureLeaf(node)) {
				    data_[node.pos].points.clear(node.offset);
			    }
		    },
		    [this](pos_t nodes) { clearImpl(nodes); });
	}

	Node clearPoints(Node node, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this](Index node) {
			    if (derived().isPureLeaf(node)) {
				    data_[node.pos].points.clear(node.offset);
			    }
		    },
		    [this](pos_t nodes) { clearImpl(nodes); }, propagate);
	}

	Node clearPoints(Code code, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this](Index node) {
			    if (derived().isPureLeaf(node)) {
				    data_[node.pos].points.clear(node.offset);
			    }
		    },
		    [this](pos_t nodes) { clearImpl(nodes); }, propagate);
	}

	Node clearPoints(Key key, bool propagate = true)
	{
		return clearPoints(derived().toCode(key), propagate);
	}

	Node clearPoints(Point coord, bool propagate = true, depth_t depth = 0)
	{
		return clearPoints(derived().toCode(coord, depth), propagate);
	}

	Node clearPoints(coord_t x, coord_t y, coord_t z, bool propagate = true,
	                 depth_t depth = 0)
	{
		return clearPoints(derived().toCode(x, y, z, depth), propagate);
	}

	//
	// Erase points
	//

	void erasePoints(std::invocable<PointColor> auto unary_pred, bool propagate = true)
	{
		erasePoints(derived().rootCode(), unary_pred, propagate);
	}

	void erasePoints(std::invocable<Index, PointColor> auto binary_pred,
	                 bool                                   propagate = true)
	{
		erasePoints(derived().rootCode(), binary_pred, propagate);
	}

	void erasePoints(Index node, std::invocable<PointColor> auto unary_pred)
	{
		// TODO: Implement
	}

	void erasePoints(Index node, std::invocable<Index, PointColor> auto binary_pred)
	{
		// TODO: Implement
	}

	void erasePoints(Node node, std::invocable<PointColor> auto unary_pred,
	                 bool propagate = true)
	{
		// TODO: Implement
	}

	void erasePoints(Node node, std::invocable<Index, PointColor> auto binary_pred,
	                 bool propagate = true)
	{
		// TODO: Implement
	}

	void erasePoints(Code code, std::invocable<PointColor> auto unary_pred,
	                 bool propagate = true)
	{
		// TODO: Implement
	}

	void erasePoints(Code code, std::invocable<Index, PointColor> auto binary_pred,
	                 bool propagate = true)
	{
		// TODO: Implement
	}

	void erasePoints(Key key, std::invocable<PointColor> auto unary_pred,
	                 bool propagate = true)
	{
		erasePoints(derived().toCode(key), unary_pred, propagate);
	}

	void erasePoints(Key key, std::invocable<Index, PointColor> auto binary_pred,
	                 bool propagate = true)
	{
		erasePoints(derived().toCode(key), binary_pred, propagate);
	}

	void erasePoints(Point coord, std::invocable<PointColor> auto unary_pred,
	                 bool propagate = true, depth_t depth = 0)
	{
		erasePoints(derived().toCode(coord, depth), unary_pred, propagate);
	}

	void erasePoints(Point coord, std::invocable<Index, PointColor> auto binary_pred,
	                 bool propagate = true, depth_t depth = 0)
	{
		erasePoints(derived().toCode(coord, depth), binary_pred, propagate);
	}

	void erasePoints(coord_t x, coord_t y, coord_t z,
	                 std::invocable<PointColor> auto unary_pred, bool propagate = true,
	                 depth_t depth = 0)
	{
		erasePoints(derived().toCode(x, y, z, depth), unary_pred, propagate);
	}

	void erasePoints(coord_t x, coord_t y, coord_t z,
	                 std::invocable<Index, PointColor> auto binary_pred,
	                 bool propagate = true, depth_t depth = 0)
	{
		erasePoints(derived().toCode(x, y, z, depth), binary_pred, propagate);
	}

	//
	// Propagation criteria
	//

	[[nodiscard]] constexpr PointsPropagationCriteria pointsPropagationCriteria()
	    const noexcept
	{
		return prop_criteria_;
	}

	void setPointsPropagationCriteria(PointsPropagationCriteria prop_criteria,
	                                  bool                      propagate = true)
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

	PointsColorMap() { data_.emplace_back(0); }

	PointsColorMap(PointsColorMap const& other)
	    : data_(other.data_), prop_criteria_(other.prop_criteria_)
	{
		for (pos_t i{}; data_.size() != i; ++i) {
			if (derived().isPureLeaves(i)) {
				data_[i].data   = 0;
				data_[i].points = other.data_[i].points;
			}
		}
	}

	PointsColorMap(PointsColorMap&&) = default;

	template <class Derived2>
	PointsColorMap(PointsColorMap<T, Derived2, N> const& other)
	    : data_(other.data_), prop_criteria_(other.prop_criteria_)
	{
		for (pos_t i{}; data_.size() != i; ++i) {
			if (derived().isPureLeaves(i)) {
				data_[i].data   = 0;
				data_[i].points = other.data_[i].points;
			}
		}
	}

	template <class Derived2>
	PointsColorMap(PointsColorMap<T, Derived2, N>&& other)
	    : data_(std::move(other.data_)), prop_criteria_(std::move(other.prop_criteria_))
	{
	}

	//
	// Destructor
	//

	~PointsColorMap()
	{
		for (pos_t i{}; data_.size() != i; ++i) {
			clearImpl(i);
		}
	}

	//
	// Assignment operator
	//

	PointsColorMap& operator=(PointsColorMap const& rhs)
	{
		data_ = rhs.data_;
		for (pos_t i{}; data_.size() != i; ++i) {
			if (derived().isPureLeaves(i)) {
				data_[i].data   = 0;
				data_[i].points = rhs.data_[i].points;
			}
		}

		prop_criteria_ = rhs.prop_criteria_;
		return *this;
	}

	PointsColorMap& operator=(PointsColorMap&&) = default;

	template <class Derived2>
	PointsColorMap& operator=(PointsColorMap<T, Derived2, N> const& rhs)
	{
		data_ = rhs.data_;
		for (pos_t i{}; data_.size() != i; ++i) {
			if (derived().isPureLeaves(i)) {
				data_[i].data   = 0;
				data_[i].points = rhs.data_[i].points;
			}
		}

		prop_criteria_ = rhs.prop_criteria_;
		return *this;
	}

	template <class Derived2>
	PointsColorMap& operator=(PointsColorMap<T, Derived2, N>&& rhs)
	{
		data_          = std::move(rhs.data_);
		prop_criteria_ = std::move(rhs.prop_criteria_);
		return *this;
	}

	//
	// Swap
	//

	void swap(PointsColorMap& other) noexcept
	{
		std::swap(data_, other.data_);
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

	void createNodeBlock(Index node) { data_.emplace_back(0); }

	//
	// Resize
	//

	void resize(std::size_t count) { data_.resize(count, Data(0)); }

	//
	// Reserve
	//

	void reserveImpl(std::size_t new_cap) { data_.reserve(new_cap); }

	//
	// Initialize root
	//

	void initRoot() { data_[derived().rootIndex().pos].data = 0; }

	//
	// Fill
	//

	void fill(Index node, pos_t children) {}

	//
	// Clear
	//

	void clearImpl() { data_.assign(1, Data(0)); }

	void clearImpl(pos_t nodes)
	{
		if (derived().isPureLeaves(nodes)) {
			data_[nodes].points.clear();
		} else {
			data_[nodes].data = 0;
		}
	}

	//
	// Shrink to fit
	//

	void shrinkToFitImpl() { data_.shrink_to_fit(); }

	//
	// Update node
	//

	void updateNode(Index node, pos_t children)
	{
		switch (pointsPropagationCriteria()) {
			case PointsPropagationCriteria::HAS_POINTS: {
				std::size_t num{};
				for (offset_t i{}; N != i; ++i) {
					auto sibling = derived().sibling(node, i);
					num |= std::size_t(derived().isParent(sibling) &&
					                   0 != data_[derived().children(sibling)].data)
					       << i;
				}
				data_[node.pos].data = num;
				return;
			}
			case PointsPropagationCriteria::NUM_POINTS: {
				std::size_t num{};
				auto        pl = derived().isPureLeaves(children);
				for (offset_t i{}; N != i; ++i) {
					auto sibling = derived().sibling(node, i);
					if (derived().isParent(sibling)) {
						num += pl ? data_[derived().children(sibling)].points.size()
						          : data_[derived().children(sibling)].data;
					}
				}
				data_[node.pos].data = num;
				return;
			}
			case PointsPropagationCriteria::NONE: return;
		}
	}

	//
	// Is prunable
	//

	[[nodiscard]] bool isPrunable(pos_t nodes) const
	{
		return derived().isPureLeaves(nodes) ? data_[nodes].points.empty()
		                                     : 0 == data_[nodes].data;
	}

	//
	// Memory node block
	//

	[[nodiscard]] static constexpr std::size_t memoryNodeBlock() noexcept
	{
		return sizeof(Data);
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr MapType mapType() noexcept
	{
		if constexpr (std::same_as<std::int8_t, T>) {
			return MapType::POINTS_COLOR8;
		} else if constexpr (std::same_as<std::int16_t, T>) {
			return MapType::POINTS_COLOR16;
		} else if constexpr (std::same_as<std::int32_t, T>) {
			return MapType::POINTS_COLOR32;
		}
	}

	std::size_t serializedSize(std::ranges::input_range auto r) const
	{
		std::size_t s{};
		for (auto nodes : r) {
			if (derived().isPureLeaves(nodes)) {
				s += data_[nodes].points.serializedSize();
			}
		}
		return s;
	}

	void readNodes(ReadBuffer& in, std::ranges::input_range auto r)
	{
		for (auto const [pos, offsets] : r) {
			if (!derived().isPureLeaves(pos)) {
				continue;
			}

			if (offsets.all()) {
				data_[pos].points.read(in);
			} else {
				SmallVector<color_t, 3 + 3 * sizeof(T) / sizeof(color_t), N, size_t> points;
				points.read(in);
				for (offset_t i{}; N != i; ++i) {
					if (!offsets[i]) {
						continue;
					}
					data_[pos].points.resize(i, points.size(i));
					if (!points.empty(i)) {
						std::copy(points.begin(i), points.end(i), data_[pos].points.begin(i));
					}
				}
			}
		}
	}

	void writeNodes(WriteBuffer& out, std::ranges::input_range auto r) const
	{
		for (auto nodes : r) {
			if (derived().isPureLeaves(nodes)) {
				data_[nodes].points.write(out);
			}
		}
	}

	//
	// Dot file info
	//

	std::ostream& dotFileInfo(std::ostream& out, Index node) const
	{
		switch (pointsPropagationCriteria()) {
			case PointsPropagationCriteria::HAS_POINTS:
				if (derived().isPureLeaf(node)) {
					out << "Points: " << data_[node.pos].points.size(node.offset);
				} else {
					out << "Points: "
					    << ((data_[node.pos].data & (std::size_t(1) << node.offset))
					            ? "<font color='green'><b>Yes</b></font>"
					            : "<font color='red'>No</font>");
				}
				break;
			case PointsPropagationCriteria::NUM_POINTS:
				if (derived().isPureLeaf(node)) {
					out << "Points: " << data_[node.pos].points.size(node.offset);
				} else if (derived().isParent(node)) {
					auto c = derived().children(node);
					if (derived().isPureLeaves(c)) {
						out << "Points: " << data_[c].points.size();
					} else {
						out << "Points: " << data_[c].data;
					}
				} else {
					out << "Points: 0";
				}
				break;
			case PointsPropagationCriteria::NONE:
				if (derived().isPureLeaf(node)) {
					out << "Points: " << data_[node.pos].points.size(node.offset);
				} else {
					out << "Points: Unknown";
				}
				break;
		}
		return out;
	}

 protected:
	// Data
	Container<Data> data_;

	// Propagation criteria
	PointsPropagationCriteria prop_criteria_ = PointsPropagationCriteria::NUM_POINTS;

	template <typename T2, class Derived2, std::size_t N2>
	friend class PointsColorMap;
};

template <class Derived, std::size_t N>
using PointsColorMap8 = PointsColorMap<std::int8_t, Derived, N>;

template <class Derived, std::size_t N>
using PointsColorMap16 = PointsColorMap<std::int16_t, Derived, N>;

template <class Derived, std::size_t N>
using PointsColorMap32 = PointsColorMap<std::int32_t, Derived, N>;

//
// Concepts
//

template <class Map>
concept IsPointsColorMap =
    IsMapType<Map, MapType::POINTS_COLOR8> || IsMapType<Map, MapType::POINTS_COLOR16> ||
    IsMapType<Map, MapType::POINTS_COLOR32>;
}  // namespace ufo

#endif  // UFO_MAP_POINTS_COLOR_MAP_HPP