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

#ifndef UFO_MAP_OCTREE_NODE_HPP
#define UFO_MAP_OCTREE_NODE_HPP

// UFO
#include <ufo/geometry/aabc.hpp>
#include <ufo/geometry/point.hpp>
#include <ufo/map/code.hpp>
#include <ufo/map/index.hpp>
#include <ufo/map/types.hpp>

// STL
#include <utility>

namespace ufo
{
/*!
 * @brief A wrapper around a UFOMap inner/leaf node.
 *
 */
struct Node {
 public:
	//
	// Constructor
	//

	constexpr Node() = default;

	void swap(Node& other) noexcept
	{
		std::swap(code_, other.code_);
		std::swap(index_, other.index_);
	}

	/*!
	 * @brief Compare two nodes.
	 *
	 * @param lhs,rhs The nodes to compare.
	 * @return Whether the two nodes are equal.
	 */
	friend constexpr bool operator==(Node lhs, Node rhs) noexcept
	{
		return lhs.code_ == rhs.code_ && lhs.index_ == rhs.index_;
	}

	/*!
	 * @brief Compare two nodes.
	 *
	 * @param lhs,rhs The nodes to compare.
	 * @return Whether the two nodes are different.
	 */
	friend constexpr bool operator!=(Node lhs, Node rhs) noexcept { return !(lhs == rhs); }

	friend constexpr bool operator<(Node lhs, Node rhs) noexcept
	{
		return lhs.code_ < rhs.code_;
	}

	friend constexpr bool operator<=(Node lhs, Node rhs) noexcept
	{
		return lhs.code_ <= rhs.code_;
	}

	friend constexpr bool operator>(Node lhs, Node rhs) noexcept
	{
		return lhs.code_ > rhs.code_;
	}

	friend constexpr bool operator>=(Node lhs, Node rhs) noexcept
	{
		return lhs.code_ >= rhs.code_;
	}

	/*!
	 * @brief Get the code for the node.
	 *
	 * @return The code for the node.
	 */
	[[nodiscard]] constexpr Code code() const noexcept { return code_; }

	/*!
	 * @brief Get the depth of the node.
	 *
	 * @return The depth of the node.
	 */
	[[nodiscard]] constexpr depth_t depth() const noexcept { return code_.depth(); }

	/*!
	 * @brief Get the corresponding index.
	 *
	 * @note Use the octree that generated the node to read the data.
	 *
	 * @return The corresponding data.
	 */
	[[nodiscard]] constexpr Index index() const noexcept { return index_; }

 protected:
	constexpr Node(Code code, Index index) noexcept : code_(code), index_(index) {}

	[[nodiscard]] constexpr pos_t pos() const noexcept { return index_.pos; }

	/*!
	 * @brief Get the offset of the node (i.e., the child from the parent's
	 * perspective).
	 *
	 * @return The offset of the node.
	 */
	[[nodiscard]] constexpr offset_t offset() const noexcept { return index_.offset; }

	[[nodiscard]] constexpr offset_t offset(depth_t depth) const noexcept
	{
		return code_.offset(depth);
	}

 protected:
	// The code for the node
	Code code_;
	// The index of the node
	Index index_;

	template <class Derived>
	friend class Octree;

	friend class std::hash<Node>;
};

struct NodeBV : public Node {
 public:
	//
	// Constructor
	//

	constexpr NodeBV() = default;

	void swap(NodeBV& other)
	{
		std::swap(static_cast<Node&>(*this), static_cast<Node&>(other));
		std::swap(aabc_, other.aabc_);
	}

	friend constexpr bool operator==(NodeBV const& lhs, NodeBV const& rhs) noexcept
	{
		return static_cast<Node>(lhs) == static_cast<Node>(rhs);
	}

	friend constexpr bool operator!=(NodeBV const& lhs, NodeBV const& rhs) noexcept
	{
		return !(lhs == rhs);
	}

	/*!
	 * @brief The bounding volume of the node.
	 *
	 * @return The bounding volume of the node.
	 */
	[[nodiscard]] constexpr AABC boundingVolume() const noexcept { return aabc_; }

	/*!
	 * @brief The center coordinate of the node.
	 *
	 * @return The center coordinate of the node.
	 */
	[[nodiscard]] constexpr Point center() const noexcept { return aabc_.center; }

	/*!
	 * @brief The minimum coordinate of the node.
	 *
	 * @return The minimum coordinate of the node.
	 */
	[[nodiscard]] constexpr Point min() const noexcept { return aabc_.min(); }

	/*!
	 * @brief The maximum coordinate of the node.
	 *
	 * @return The maximum coordinte of the node.
	 */
	[[nodiscard]] constexpr Point max() const noexcept { return aabc_.max(); }

	/*!
	 * @brief Half the length of a side of the node.
	 *
	 * @return Half the length of a side of the node.
	 */
	[[nodiscard]] constexpr float halfSize() const noexcept { return aabc_.half_size; }

	/*!
	 * @brief The length of a side of the node.
	 *
	 * @return The length of a side of the node.
	 */
	[[nodiscard]] constexpr float size() const noexcept { return 2 * halfSize(); }

	/*!
	 * @brief The center x coordinate of the node.
	 *
	 * @return The center x coordinate of the node.
	 */
	[[nodiscard]] constexpr float x() const noexcept { return aabc_.center.x; }

	/*!
	 * @brief The center y coordinate of the node.
	 *
	 * @return The center y coordinate of the node.
	 */
	[[nodiscard]] constexpr float y() const noexcept { return aabc_.center.y; }

	/*!
	 * @brief The center z coordinate of the node.
	 *
	 * @return The center z coordinate of the node.
	 */
	[[nodiscard]] constexpr float z() const noexcept { return aabc_.center.z; }

 protected:
	constexpr NodeBV(Code code, Index index, AABC aabc) noexcept
	    : Node(code, index), aabc_(aabc)
	{
	}

	constexpr NodeBV(Node node, AABC aabc) noexcept : Node(node), aabc_(aabc) {}

 private:
	// The AABC for the node
	AABC aabc_;

	template <class Derived>
	friend class Octree;

	friend class std::hash<NodeBV>;
};

struct NearestNode : NodeBV {
	double const squared_distance;

	constexpr NearestNode(NodeBV const& node, double squared_distance)
	    : NodeBV(node), squared_distance(squared_distance)
	{
	}

	friend constexpr bool operator==(NearestNode const& a, NearestNode const& b)
	{
		return static_cast<NodeBV const&>(a) == static_cast<NodeBV const&>(b);
	}

	friend constexpr bool operator!=(NearestNode const& a, NearestNode const& b)
	{
		return !(a == b);
	}

	friend constexpr bool operator<(NearestNode const& a, NearestNode const& b)
	{
		return a.squared_distance < b.squared_distance;
	}

	friend constexpr bool operator<=(NearestNode const& a, NearestNode const& b)
	{
		return a.squared_distance <= b.squared_distance;
	}

	friend constexpr bool operator>(NearestNode const& a, NearestNode const& b)
	{
		return a.squared_distance > b.squared_distance;
	}

	friend constexpr bool operator>=(NearestNode const& a, NearestNode const& b)
	{
		return a.squared_distance >= b.squared_distance;
	}
};

}  // namespace ufo

namespace std
{
template <>
struct hash<ufo::Node> {
	std::size_t operator()(ufo::Node node) const { return hash<ufo::Code>()(node.code_); }
};

template <>
struct hash<ufo::NodeBV> {
	std::size_t operator()(ufo::NodeBV node) const { return hash<ufo::Code>()(node.code_); }
};
}  // namespace std

#endif  // UFO_MAP_OCTREE_NODE_HPP