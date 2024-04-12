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

#ifndef UFO_MAP_OCTREE_BASE_HPP
#define UFO_MAP_OCTREE_BASE_HPP

// UFO
#include <ufo/algorithm/algorithm.hpp>
#include <ufo/map/bit_set.hpp>
#include <ufo/map/code.hpp>
#include <ufo/map/index.hpp>
#include <ufo/map/io.hpp>
#include <ufo/map/key.hpp>
#include <ufo/map/node.hpp>
#include <ufo/map/octree/octree_iterator.hpp>
#include <ufo/map/octree/octree_predicate.hpp>
#include <ufo/map/point.hpp>
#include <ufo/map/predicate/predicate.hpp>
#include <ufo/map/predicate/spatial.hpp>
#include <ufo/map/types.hpp>
#include <ufo/math/util.hpp>
#include <ufo/util/iterator_wrapper.hpp>
#include <ufo/util/type_traits.hpp>

// STL
#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <cmath>
#include <concepts>
#include <fstream>
#include <functional>
#include <ios>
#include <iostream>
#include <iterator>
#include <memory>
#include <numeric>
#include <optional>
#include <ranges>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace ufo
{
// Utilizing curiously recurring template pattern (CRTP)
template <class Derived>
class Octree
{
 public:
	//
	// Tags
	//

	using const_iterator                       = OctreeIteratorWrapper<Derived, Node>;
	using const_query_iterator                 = const_iterator;
	using const_bounding_volume_iterator       = OctreeIteratorWrapper<Derived, NodeBV>;
	using const_bounding_volume_query_iterator = const_bounding_volume_iterator;
	using const_query_nearest_iterator = OctreeIteratorWrapper<Derived, NearestNode>;

	using Query        = IteratorWrapper<const_query_iterator>;
	using QueryBV      = IteratorWrapper<const_bounding_volume_query_iterator>;
	using QueryNearest = IteratorWrapper<const_query_nearest_iterator>;

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                       Octree                                        |
	|                                                                                     |
	**************************************************************************************/

	//
	// Order for Compute
	//

	/*!
	 * @brief Order the data in the octree for efficient compute. This is potentially an
	 * expensive call. Only use this if you are going to make many queries without altering
	 * the octree in between, as this essentially 'sorts' the octree and it is only ordered
	 * until the octree changes.
	 */
	void orderForCompute()
	{
		auto perm = sortPermutation(parent_code_);

		// TODO: Implement

		derived().applyPermutation(perm);

		applyPermutation(parent_code_, perm);
		applyPermutation(modified_, perm);

		resize(numNodes() / 8);
	}

	//
	// Reserve
	//

	void reserve(std::size_t new_cap)
	{
		children_.reserve(new_cap);
		parent_code_.reserve(new_cap);
		modified_.reserve(new_cap);
		derived().reserveImpl(new_cap);
	}

	//
	// Shrink to fit
	//

	void shrinkToFit()
	{
		children_.shrink_to_fit();
		parent_code_.shrink_to_fit();
		modified_.shrink_to_fit();
		derived().shrinkToFitImpl();
	}

	//
	// Clear
	//

	/*!
	 * @brief Erases the map. After this call, the map contains only the root node.
	 */
	void clear() { clear(size(), depthLevels()); }

	/*!
	 * @brief Erases the map and changes the leaf node size and the number of depth levels.
	 * After this call, the map contains only the root node.
	 *
	 * @param leaf_size The new leaf node size.
	 * @param depth_levels The new number of depth levels.
	 */
	void clear(node_size_t leaf_size, depth_t depth_levels)
	{
		resize(1);

		setNodeSizeAndDepthLevels(leaf_size, depth_levels);

		initRoot();
	}

	//
	// Depth levels
	//

	/*!
	 * @brief The number of depth levels the octree currently have.
	 *
	 * @return The number of depth levels the octree currently have.
	 */
	[[nodiscard]] constexpr depth_t depthLevels() const noexcept { return depth_levels_; }

	/*!
	 * @brief The minimum depth levels an octree can have.
	 *
	 * @return The minimum depth levels an octree can have.
	 */
	[[nodiscard]] static constexpr depth_t minDepthLevels() noexcept
	{
		// FIXME: Change to correct
		return 2;
	}

	/*!
	 * @brief The maximum depth levels an octree can have.
	 *
	 * @return The maximum depth levels an octree can have.
	 */
	[[nodiscard]] static constexpr depth_t maxDepthLevels() noexcept
	{
		// FIXME: Change to correct
		return 20;  // 21
	}

	//
	// Size
	//

	/*!
	 * @brief Get the node size at a specific depth.
	 *
	 * @param depth The depth.
	 * @return The node size at the depth.
	 */
	[[nodiscard]] constexpr node_size_t size(depth_t depth = 0) const
	{
		assert(rootDepth() >= depth);
		return node_size_[depth];
	}

	//
	// Volume
	//

	/*!
	 * @brief The volume of a node at a specific depth.
	 *
	 * @note Same as `size(depth) * size(depth) * size(depth)`.
	 *
	 * @param depth The depth.
	 * @return The volume of a node at the depth.
	 */
	[[nodiscard]] constexpr node_size_t volume(depth_t depth = 0) const
	{
		assert(rootDepth() >= depth);
		auto const s = size(depth);
		return s * s * s;
	}

	//
	// Center
	//

	/*!
	 * @return The center of the octree.
	 */
	[[nodiscard]] Point center() const { return Point(0, 0, 0); }

	//
	// Bounding volume
	//

	/*!
	 * @return Minimum bounding volume convering the whole octree.
	 */
	[[nodiscard]] AABC boundingVolume() const
	{
		return AABC(center(), size(rootDepth() - 1));
	}

	//
	// Inside
	//

	/*!
	 * @brief Check if a coordinate is inside the octree bounds.
	 *
	 * @param coord The coordinate.
	 * @return Whether the coordinate is inside the octree bounds.
	 */
	[[nodiscard]] constexpr bool isInside(Point coord) const
	{
		return isInside(coord.x, coord.y, coord.z);
	}

	/*!
	 * @brief Check if a coordinate is inside the octree bounds.
	 *
	 * @param x,y,z The coordinate.
	 * @return Whether the coordinate is inside the octree bounds.
	 */
	[[nodiscard]] constexpr bool isInside(coord_t x, coord_t y, coord_t z) const
	{
		coord_t const max = static_cast<coord_t>(size(rootDepth() - 1));
		coord_t const min = -max;
		return min <= x && min <= y && min <= z && max > x && max > y && max > z;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Leaf                                         |
	|                                                                                     |
	**************************************************************************************/

	//
	// Pure leaf
	//

	/*!
	 * @brief Check if a node is a pure leaf node (i.e., can never have children).
	 *
	 * @param node The node to check.
	 * @return Whether the node is a pure leaf node.
	 */
	[[nodiscard]] constexpr bool isPureLeaf(Index node) const
	{
		// NOTE: Should be 1 because it is the parent code of the node
		return 1 == parent_code_[node.pos].depth();
	}

	/*!
	 * @brief Check if a node is a pure leaf node (i.e., can never have children).
	 *
	 * @note Only have to check if the depth of the node is 0.
	 *
	 * @param node The node to check.
	 * @return Whether the node is a pure leaf node.
	 */
	[[nodiscard]] static constexpr bool isPureLeaf(Node node) noexcept
	{
		return 0 == node.depth();
	}

	/*!
	 * @brief Check if a node corresponding to a code is a pure leaf node (i.e., can never
	 * have children).
	 *
	 * @note Only have to check if the depth of the code is 0.
	 *
	 * @param code The code of the node to check.
	 * @return Whether the node is a pure leaf node.
	 */
	[[nodiscard]] static constexpr bool isPureLeaf(Code code) noexcept
	{
		return 0 == code.depth();
	}

	/*!
	 * @brief Check if a node corresponding to a key is a pure leaf node (i.e., can never
	 * have children).
	 *
	 * @note Only have to check if the depth of the key is 0.
	 *
	 * @param key The key of the node to check.
	 * @return Whether the node is a pure leaf node.
	 */
	[[nodiscard]] static constexpr bool isPureLeaf(Key key) noexcept
	{
		return 0 == key.depth();
	}

	/*!
	 * @brief Check if a node corresponding to a coordinate at a specified depth is a
	 * pure leaf node (i.e., can never have children).
	 *
	 * @note Only have to check if the depth is 0.
	 *
	 * @param coord The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is a pure leaf node.
	 */
	[[nodiscard]] static constexpr bool isPureLeaf(Point coord, depth_t depth = 0) noexcept
	{
		return 0 == depth;
	}

	/*!
	 * @brief Check if a node corresponding to a coordinate at a specified depth is a
	 * pure leaf node (i.e., can never have children).
	 *
	 * @note Only have to check if the depth is 0.
	 *
	 * @param x,y,z The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is a pure leaf node.
	 */
	[[nodiscard]] static constexpr bool isPureLeaf(coord_t x, coord_t y, coord_t z,
	                                               depth_t depth = 0) noexcept
	{
		return 0 == depth;
	}

	//
	// Leaf
	//

	/*!
	 * @brief Check if a node is a leaf node (i.e., has no children).
	 *
	 * @param node The node to check.
	 * @return Whether the node is a leaf node.
	 */
	[[nodiscard]] constexpr bool isLeaf(Index node) const
	{
		return NULL_POS == children_[node.pos][node.offset];
	}

	/*!
	 * @brief Check if a node is a leaf node (i.e., has no children).
	 *
	 * @param node The node to check.
	 * @return Whether the node is a leaf node.
	 */
	[[nodiscard]] constexpr bool isLeaf(Node node) const { return isLeaf(index(node)); }

	/*!
	 * @brief Check if a node corresponding to a code is a leaf node (i.e., has no
	 * children).
	 *
	 * @param code The code of the node to check.
	 * @return Whether the node is a leaf node.
	 */
	[[nodiscard]] constexpr bool isLeaf(Code code) const { return isLeaf(index(code)); }

	/*!
	 * @brief Check if a node corresponding to a key is a leaf node (i.e., has no children).
	 *
	 * @param key The key of the node to check.
	 * @return Whether the node is a leaf node.
	 */
	[[nodiscard]] constexpr bool isLeaf(Key key) const { return isLeaf(toCode(key)); }

	/*!
	 * @brief Check if a node corresponding to a coordinate at a specified depth is a leaf
	 * node (i.e., has no children).
	 *
	 * @param coord The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is a leaf node.
	 */
	[[nodiscard]] constexpr bool isLeaf(Point coord, depth_t depth = 0) const
	{
		return isLeaf(toCode(coord, depth));
	}

	/*!
	 * @brief Check if a node corresponding to a coordinate at a specified depth is a leaf
	 * node (i.e., has no children).
	 *
	 * @param x,y,z The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is a leaf node.
	 */
	[[nodiscard]] constexpr bool isLeaf(coord_t x, coord_t y, coord_t z,
	                                    depth_t depth = 0) const
	{
		return isLeaf(toCode(x, y, z, depth));
	}

	//
	// Parent
	//

	/*!
	 * @brief Check if a node is a parent (i.e., has children).
	 *
	 * @param node The node to check.
	 * @return Whether the node is a parent.
	 */
	[[nodiscard]] constexpr bool isParent(Index node) const { return !isLeaf(node); }

	/*!
	 * @brief Check if a node is a parent (i.e., has children).
	 *
	 * @param node The node to check.
	 * @return Whether the node is a parent.
	 */
	[[nodiscard]] constexpr bool isParent(Node node) const { return !isLeaf(node); }

	/*!
	 * @brief Check if a node corresponding to a code is a parent (i.e., has
	 * children).
	 *
	 * @param code The code of the node to check.
	 * @return Whether the node is a parent.
	 */
	[[nodiscard]] constexpr bool isParent(Code code) const { return !isLeaf(code); }

	/*!
	 * @brief Check if a node corresponding to a key is a parent (i.e., has
	 * children).
	 *
	 * @param key The key of the node to check.
	 * @return Whether the node is a parent.
	 */
	[[nodiscard]] constexpr bool isParent(Key key) const { return !isLeaf(key); }

	/*!
	 * @brief Check if a node corresponding to a coordinate at a specified depth is a parent
	 * (i.e., has children).
	 *
	 * @param coord The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is a parent.
	 */
	[[nodiscard]] constexpr bool isParent(Point coord, depth_t depth = 0) const
	{
		return !isLeaf(coord, depth);
	}

	/*!
	 * @brief Check if a node corresponding to a coordinate at a specified depth is a parent
	 * (i.e., has children).
	 *
	 * @param x,y,z The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is a parent.
	 */
	[[nodiscard]] constexpr bool isParent(coord_t x, coord_t y, coord_t z,
	                                      depth_t depth = 0) const
	{
		return !isLeaf(x, y, z, depth);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Exist                                        |
	|                                                                                     |
	**************************************************************************************/

	//
	// Exists
	//

	/*!
	 * @brief Check if a node at the index exists.
	 *
	 * @param node The index to check.
	 * @return Whether a node at the index exists.
	 */
	[[nodiscard]] constexpr bool exists(Index node) const
	{
		return parent_code_.size() > node.pos && INVALID_CODE != parent_code_[node.pos];
	}

	// TODO: Add comment
	[[nodiscard]] constexpr bool exists(Node node) const
	{
		return parent_code_[index(node).pos] == node.code().parent();
	}

	// TODO: Add comment
	[[nodiscard]] constexpr bool exists(Code code) const
	{
		return parent_code_[index(code).pos] == code.parent();
	}

	// TODO: Add comment
	[[nodiscard]] constexpr bool exists(Key key) const { return exists(toCode(key)); }

	// TODO: Add comment
	[[nodiscard]] constexpr bool exists(Point coord, depth_t depth = 0) const
	{
		return exists(toCode(coord, depth));
	}

	// TODO: Add comment
	[[nodiscard]] constexpr bool exists(coord_t x, coord_t y, coord_t z,
	                                    depth_t depth = 0) const
	{
		return exists(toCode(x, y, z, depth));
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Modified                                       |
	|                                                                                     |
	**************************************************************************************/

	//
	// Modified
	//

	[[nodiscard]] constexpr bool isModified(Index node) const
	{
		return modified_[node.pos][node.offset];
	}

	/*!
	 * @brief Check if the octree is in a modified state (i.e., at least one node has been
	 * modified).
	 *
	 * @return Whether the octree is in a modified state.
	 */
	[[nodiscard]] constexpr bool isModified() const { return isModified(rootIndex()); }

	/*!
	 * @brief Check if a node of the octree is in a modified state (i.e., the node
	 * or one of its children has been modified).
	 *
	 * @param node The node to check.
	 * @return Whether the node is in a modified state.
	 */
	[[nodiscard]] constexpr bool isModified(Node node) const
	{
		return isModified(index(node));
	}

	/*!
	 * @brief Check if a node corresponding to a code is in a modified state (i.e., the node
	 * or one of its children has been modified).
	 *
	 * @param code The code of the node to check.
	 * @return Whether the node is in a modified state.
	 */
	[[nodiscard]] constexpr bool isModified(Code code) const
	{
		return isModified(index(code));
	}

	/*!
	 * @brief Check if a node corresponding to a key is in a modified state (i.e., the node
	 * or one of its children has been modified).
	 *
	 * @param key The key of the node to check.
	 * @return Whether the node is in a modified state.
	 */
	[[nodiscard]] constexpr bool isModified(Key key) const
	{
		return isModified(toCode(key));
	}

	/*!
	 * @brief Check if a node corresponding to a coordinate at a specified depth is in a
	 * modified state (i.e., the node or one of its children has been modified).
	 *
	 * @param coord The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is in a modified state.
	 */
	[[nodiscard]] constexpr bool isModified(Point coord, depth_t depth = 0) const
	{
		return isModified(toCode(coord, depth));
	}

	/*!
	 * @brief Check if a node corresponding to a coordinate at a specified depth is in a
	 * modified state (i.e., the node or one of its children has been modified).
	 *
	 * @param x,y,z The coordinate of the node to check.
	 * @param depth The depth of the node to check.
	 * @return Whether the node is in a modified state.
	 */
	[[nodiscard]] constexpr bool isModified(coord_t x, coord_t y, coord_t z,
	                                        depth_t depth = 0) const
	{
		return isModified(toCode(x, y, z, depth));
	}

	//
	// Set modified
	//

	constexpr void setModified() { setModified(rootIndex()); }

	constexpr void setModified(Index node)
	{
		assert(modified_.size() > node.pos && 8 > node.offset);
		modified_[node.pos].set(node.offset);
		if (isParent(node)) {
			setModifiedRecurs(children(node));
		}
	}

	/*!
	 * @brief Set a node and all its children to the modified state. This will also set the
	 * parents of the node to the modified state.
	 *
	 * @param node The node.
	 */
	constexpr void setModified(Node node) { setModified(createIndex(node)); }

	/*!
	 * @brief Set a node, corresponding to a code, and all its children to the modified
	 * state. This will also set the parents of the node to the modified state.
	 *
	 * @param code The code of the node.
	 */
	constexpr void setModified(Code code) { setModified(createIndex(code)); }

	/*!
	 * @brief Set a node, corresponding to a key, and all its children to the modified
	 * state. This will also set the parents of the node to the modified state.
	 *
	 * @param key The key of the node.
	 */
	constexpr void setModified(Key key) { setModified(toCode(key)); }

	/*!
	 * @brief Set a node, corresponding to a coordinate and a specified depth, and all its
	 * children to the modified state. This will also set the parents of the node to the
	 * modified state.
	 *
	 * @param coord The coordinate of the node.
	 * @param depth The depth of the node.
	 */
	constexpr void setModified(Point coord, depth_t depth = 0)
	{
		setModified(toCode(coord, depth));
	}

	/*!
	 * @brief Set a node, corresponding to a coordinate and a specified depth, and all its
	 * children to the modified state. This will also set
	 * the parents of the node to the modified state.
	 *
	 * @param x,y,z The coordinate of the node.
	 * @param depth The depth of the node.
	 */
	constexpr void setModified(coord_t x, coord_t y, coord_t z, depth_t depth = 0)
	{
		setModified(toCode(x, y, z, depth));
	}

	//
	// Reset modified
	//

	constexpr void resetModified(Index node)
	{
		if (isParent(node) && isModified(node)) {
			resetModifiedRecurs(children(node));
		}
		modified_[node.pos].reset(node.offset);
	}

	/*!
	 * @brief Reset all nodes from the modified state.
	 *
	 * @note Only use this if you know what you are doing or if you do *not* plan to query
	 * for information in the octree. This function does not propagate information up the
	 * tree, hence queries will not function as expected. If you want to make queries then
	 * you should use 'propagateModified' instead.
	 */
	constexpr void resetModified() { resetModified(rootIndex()); }

	/*!
	 * @brief Reset a node and its children from the modified state.
	 *
	 * @note Only use this if you know what you are doing or if you do *not* plan to query
	 * for information in the octree. This function does not propagate information up the
	 * tree, hence queries will not function as expected. If you want to make queries then
	 * you should use 'propagateModified' instead.
	 *
	 * @param node The node.
	 */
	constexpr void resetModified(Node node)
	{
		// TODO: Implement
	}

	/*!
	 * @brief Reset a node, corresponding to a code, and its children from the modified
	 * state.
	 *
	 * @note Only use this if you know what you are doing or if you do *not* plan to query
	 * for information in the octree. This function does not propagate information up the
	 * tree, hence queries will not function as expected. If you want to make queries then
	 * you should use 'propagateModified' instead.
	 *
	 * @param code The code of the node.
	 */
	constexpr void resetModified(Code code)
	{
		// TODO: Implement
	}

	/*!
	 * @brief Reset a node, corresponding to a key, and its children from the modified
	 * state.
	 *
	 * @note Only use this if you know what you are doing or if you do *not* plan to query
	 * for information in the octree. This function does not propagate information up the
	 * tree, hence queries will not function as expected. If you want to make queries then
	 * you should use 'propagateModified' instead.
	 *
	 * @param key The key of the node.
	 */
	constexpr void resetModified(Key key) { resetModified(toCode(key)); }

	/*!
	 * @brief Reset a node, corresponding to a coordinate and a specified depth, and its
	 * children from the modified state.
	 *
	 * @note Only use this if you know what you are doing or if you do *not* plan to query
	 * for information in the octree. This function does not propagate information up the
	 * tree, hence queries will not function as expected. If you want to make queries then
	 * you should use 'propagateModified' instead.
	 *
	 * @param coord The coordinate of the node.
	 * @param depth The depth of the node.
	 */
	constexpr void resetModified(Point coord, depth_t depth = 0)
	{
		resetModified(toCode(coord, depth));
	}

	/*!
	 * @brief Reset a node, corresponding to a coordinate and a specified depth, and its
	 * children from the modified state.
	 *
	 * @note Only use this if you know what you are doing or if you do *not* plan to query
	 * for information in the octree. This function does not propagate information up the
	 * tree, hence queries will not function as expected. If you want to make queries then
	 * you should use 'propagateModified' instead.
	 *
	 * @param x,y,z The coordinate of the node.
	 * @param depth The depth of the node.
	 */
	constexpr void resetModified(coord_t x, coord_t y, coord_t z, depth_t depth = 0)
	{
		resetModified(toCode(x, y, z, depth));
	}

	//
	// Propagate
	//

	/*!
	 * @brief Propagate modified information up the octree.
	 *
	 * @param reset_modified Whether propagated node's modified state should be reset.
	 * @param prune Whether the tree should be pruned also.
	 */
	void propagateModified(bool reset_modified = true, bool prune = false)
	{
		propagateModified(rootIndex(), reset_modified, prune);
	}

	void propagateModified(Index node, bool reset_modified = true, bool prune = false)
	{
		if (!isModified(node)) {
			return;
		}

		if (isParent(node)) {
			if (reset_modified) {
				if (prune) {
					propagateModifiedRecurs<true, true>(children(node));
				} else {
					propagateModifiedRecurs<true, false>(children(node));
				}
			} else {
				if (prune) {
					propagateModifiedRecurs<false, true>(children(node));
				} else {
					propagateModifiedRecurs<false, false>(children(node));
				}
			}
			if (prune) {
				updateNode<true>(node);
			} else {
				updateNode<false>(node);
			}
		}

		modified_[node.pos][node.offset] = !reset_modified;
	}

	/*!
	 * @brief Propagate a node's children modified information up the octree.
	 *
	 * @param node The node.
	 * @param reset_modified Whether propagated node's modified state should be reset.
	 * @param prune Whether the tree should be pruned also.
	 */
	void propagateModified(Node node, bool reset_modified = true, bool prune = false)
	{
		propagateModified(index(node), reset_modified, prune);
	}

	/*!
	 * @brief Propagate a node's, corresponding to a code, children modified information up
	 * the octree.
	 *
	 * @param code The code of the node.
	 * @param reset_modified Whether propagated node's modified state should be reset.
	 * @param prune Whether the tree should be pruned also.
	 */
	void propagateModified(Code code, bool reset_modified = true, bool prune = false)
	{
		propagateModified(index(code), reset_modified, prune);
	}

	/*!
	 * @brief Propagate a node's, corresponding to a key, children modified information up
	 * the octree.
	 *
	 * @param key The key of the node.
	 * @param reset_modified Whether propagated node's modified state should be reset.
	 * @param prune Whether the tree should be pruned also.
	 */
	void propagateModified(Key key, bool reset_modified = true, bool prune = false)
	{
		propagateModified(toCode(key), reset_modified, prune);
	}

	/*!
	 * @brief Propagate a node's, corresponding to a coordinate and a specified depth,
	 * children modified information up the octree.
	 *
	 * @param coord The coordinate of the node.
	 * @param reset_modified Whether propagated node's modified state should be reset.
	 * @param prune Whether the tree should be pruned also.
	 * @param depth The depth of the node.
	 */
	void propagateModified(Point coord, bool reset_modified = true, bool prune = false,
	                       depth_t depth = 0)
	{
		propagateModified(toCode(coord, depth), reset_modified, prune);
	}

	/*!
	 * @brief Propagate a node's, corresponding to a coordinate and a specified depth,
	 * children modified information up the octree.
	 *
	 * @param x,y,z The coordinate of the node.
	 * @param reset_modified Whether propagated node's modified state should be reset.
	 * @param prune Whether the tree should be pruned also.
	 * @param depth The depth of the node.
	 */
	void propagateModified(coord_t x, coord_t y, coord_t z, bool reset_modified = true,
	                       bool prune = false, depth_t depth = 0)
	{
		propagateModified(toCode(x, y, z, depth), reset_modified, prune);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Prune                                        |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comment
	[[nodiscard]] bool isPrunable(Index node)
	{
		return isLeaf(node) || derived().isPrunable(children(node));
	}

	void prune() { prune(rootIndex()); }

	// TODO: Add comment
	void prune(Index node)
	{
		// TODO: Implement recursive stuff

		// if (isPrunable(node)) {
		// 	deleteChildren(node);
		// }
	}

	void prune(Node node)
	{
		Index n = index(node);
		if (depth(n) == node.depth()) {
			prune(node);
		}
	}

	void prune(Code code)
	{
		Index node = index(code);
		if (depth(node) == code.depth()) {
			prune(node);
		}
	}

	void prune(Point coord, depth_t depth = 0)
	{
		Index node = index(coord, depth);
		if (this->depth(node) == depth) {
			prune(node);
		}
	}

	void prune(coord_t x, coord_t y, coord_t z, depth_t depth = 0)
	{
		Index node = index(x, y, z, depth);
		if (this->depth(node) == depth) {
			prune(node);
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Root                                         |
	|                                                                                     |
	**************************************************************************************/

	//
	// Is
	//

	// TODO: Add comment
	[[nodiscard]] bool isRoot(Index node) const { return rootIndex() == node; }

	/*!
	 * @brief Check if a node is the root node.
	 *
	 * @param node The node.
	 * @return Whether the node is the root node.
	 */
	[[nodiscard]] bool isRoot(Node node) const { return isRoot(node.code()); }

	/*!
	 * @brief Check if a node, corresponding to a code, is the root node.
	 *
	 * @param code The code of the node.
	 * @return Whether the node is the root node.
	 */
	[[nodiscard]] bool isRoot(Code code) const { return rootCode() == code; }

	/*!
	 * @brief Check if a node, corresponding to a key, is the root node.
	 *
	 * @param key The key of the node.
	 * @return Whether the node is the root node.
	 */
	[[nodiscard]] bool isRoot(Key key) const { return isRoot(toCode(key)); }

	/*!
	 * @brief Check if a node, corresponding to a coordinate at a specific depth, is the
	 * root node.
	 *
	 * @param coord The coordinate of the node.
	 * @param depth The depth of the node.
	 * @return Whether the node is the root node.
	 */
	[[nodiscard]] bool isRoot(Point coord, depth_t depth = 0) const
	{
		return isRoot(toCode(coord, depth));
	}

	/*!
	 * @brief Check if a node, corresponding to a coordinate at a specific depth, is the
	 * root node.
	 *
	 * @param x,y,z The coordinate of the node.
	 * @param depth The depth of the node.
	 * @return Whether the node is the root node.
	 */
	[[nodiscard]] bool isRoot(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return isRoot(toCode(x, y, z, depth));
	}

	//
	// Get
	//

	/*!
	 * @brief Get the root node index.
	 *
	 * @return The root node index.
	 */
	[[nodiscard]] constexpr Index rootIndex() const noexcept { return Index(0, 0); }

	/*!
	 * @brief Get the root node.
	 *
	 * @return The root node.
	 */
	[[nodiscard]] constexpr Node rootNode() const noexcept
	{
		return Node(rootCode(), rootIndex());
	}

	/*!
	 * @brief Get the root node with bounding volume.
	 *
	 * @return The root node with bounding volume.
	 */
	[[nodiscard]] constexpr NodeBV rootNodeBV() const noexcept
	{
		return NodeBV(rootNode(), rootBoundingVolume());
	}

	/*!
	 * @brief Get the code for the root node.
	 *
	 * @return The root node code.
	 */
	[[nodiscard]] constexpr Code rootCode() const noexcept { return Code(0, rootDepth()); }

	/*!
	 * @brief Get the depth of the root node.
	 *
	 * @return The depth of the root node.
	 */
	[[nodiscard]] constexpr depth_t rootDepth() const noexcept { return depthLevels() - 1; }

	//
	// Size
	//

	/*!
	 * @brief Get the size of the root node.
	 *
	 * @note This is the same as `size()`.
	 *
	 * @return The size of the root node.
	 */
	[[nodiscard]] constexpr node_size_t rootSize() const { return size(rootDepth()); }

	//
	// Center
	//

	/*!
	 * @brief Get the center of the root node.
	 *
	 * @note This is the same as `center()`.
	 *
	 * @return The center of the root node.
	 */
	[[nodiscard]] constexpr Point rootCenter() const noexcept { return center(); }

	//
	// Bounding volume
	//

	/*!
	 * @brief Get the bounding volume of the root node.
	 *
	 * @note This is the same as `boundingVolmue()`.
	 *
	 * @return The bounding volume of the root node.
	 */
	[[nodiscard]] constexpr AABC rootBoundingVolume() const noexcept
	{
		return boundingVolume();
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Node                                         |
	|                                                                                     |
	**************************************************************************************/

	//
	// Size
	//

	/*!
	 * @brief Get the size of a node.
	 *
	 * @param node The node.
	 * @return The size of the node.
	 */
	[[nodiscard]] constexpr node_size_t size(Node node) const { return size(node.depth()); }

	//
	// Center
	//

	/*!
	 * @brief The center of a node.
	 *
	 * @param node The node.
	 * @return The center of the node.
	 */
	[[nodiscard]] Point center(Node node) const { return toCoord(node.code()); }

	/*!
	 * @brief The center of a node.
	 *
	 * @param node The node.
	 * @return The center of the node.
	 */
	[[nodiscard]] Point center(NodeBV const& node) const { return node.center(); }

	//
	// Bounding volume
	//

	/*!
	 * @brief Bounding volume for a node.
	 *
	 * @param node The node
	 * @return Bounding volume for the node.
	 */
	[[nodiscard]] AABC boundingVolume(Node node) const
	{
		return AABC(center(node), size(node) / 2);
	}

	/*!
	 * @brief Bounding volume for a node.
	 *
	 * @param node The node
	 * @return Bounding volume for the node.
	 */
	[[nodiscard]] AABC boundingVolume(NodeBV const& node) const
	{
		return node.boundingVolume();
	}

	//
	// Create
	//

	[[nodiscard]] Node createNode(Node node) const
	{
		// TODO: Implement correct
		return isCorrect(node) ? setModified(node), node : createNode(node.code());
	}

	[[nodiscard]] Node createNode(Code code) const { return Node(code, createIndex(code)); }

	[[nodiscard]] Node createNode(Key key) const { return createNode(toCode(key)); }

	[[nodiscard]] Node createNode(Point coord, depth_t depth = 0) const
	{
		return createNode(toCode(coord, depth));
	}

	[[nodiscard]] Node createNode(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return createNode(toCode(x, y, z, depth));
	}

	//
	// At
	//

	[[nodiscard]] std::optional<Node> at(Index node) const
	{
		return children_.size() > node.pos ? std::optional<Node>(operator()(node))
		                                   : std::nullopt;
	}

	/*!
	 * @brief Get the node corresponding to a code with bounds checking.
	 *
	 * @note The node can be higher up the tree than the specified depth. This happens if
	 * the node at a higher depth has no children. If it is neccessary that the node is at
	 * the specified depth, then the corresponding 'createNodeChecked' function can be used.
	 * The data inside the nodes returned by this function and 'createNodeChecked' will be
	 * the same, so it is only neccessary to use 'createNodeChecked' if you intend to alter
	 * what the node stores.
	 *
	 * @param code The code.
	 * @return The node.
	 */
	[[nodiscard]] std::optional<Node> at(Code code) const
	{
		return rootDepth() >= code.depth() ? std::optional<Node>(operator()(code))
		                                   : std::nullopt;
	}

	/*!
	 * @brief Get the node corresponding to a key with bounds checking.
	 *
	 * @note The node can be higher up the tree than the specified depth. This happens if
	 * the node at a higher depth has no children. If it is neccessary that the node is at
	 * the specified depth, then the corresponding 'createNodeChecked' function can be used.
	 * The data inside the nodes returned by this function and 'createNodeChecked' will be
	 * the same, so it is only neccessary to use 'createNodeChecked' if you intend to alter
	 * what the node stores.
	 *
	 * @param key The key.
	 * @return The node.
	 */
	[[nodiscard]] std::optional<Node> at(Key key) const { return at(toCode(key)); }

	/*!
	 * @brief Get the node corresponding to a coordinate at a specific depth with bounds
	 * checking.
	 *
	 * @note The node can be higher up the tree than the specified depth. This happens if
	 * the node at a higher depth has no children. If it is neccessary that the node is at
	 * the specified depth, then the corresponding 'createNodeChecked' function can be used.
	 * The data inside the nodes returned by this function and 'createNodeChecked' will be
	 * the same, so it is only neccessary to use 'createNodeChecked' if you intend to alter
	 * what the node stores.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The node.
	 */
	[[nodiscard]] std::optional<Node> at(Point coord, depth_t depth = 0) const
	{
		auto code = toCodeChecked(coord, depth);
		return code ? std::optional<Node>(*code) : std::nullopt;
	}

	/*!
	 * @brief Get the node corresponding to a coordinate at a specific depth with bounds
	 * checking.
	 *
	 * @note The node can be higher up the tree than the specified depth. This happens if
	 * the node at a higher depth has no children. If it is neccessary that the node is at
	 * the specified depth, then the corresponding 'createNodeChecked' function can be used.
	 * The data inside the nodes returned by this function and 'createNodeChecked' will be
	 * the same, so it is only neccessary to use 'createNodeChecked' if you intend to alter
	 * what the node stores.
	 *
	 * @param x,y,z The coordinate.
	 * @param depth The depth.
	 * @return The node.
	 */
	[[nodiscard]] std::optional<Node> at(coord_t x, coord_t y, coord_t z,
	                                     depth_t depth = 0) const
	{
		return at(Point(x, y, z), depth);
	}

	//
	// Function call operator
	//

	[[nodiscard]] Node operator()(Index node) const
	{
		assert(INVALID_CODE != parent_code_[node.pos]);
		return Node(parent_code_[node.pos].child(node.offset), node);
	}

	// TODO: Add comment
	[[nodiscard]] Node operator()(Node node) const
	{
		return Node(node.code(), index(node));
	}

	/*!
	 * @brief Get the node corresponding to a code.
	 *
	 * @note The node can be higher up the tree than the specified depth. This happens if
	 * the node at a higher depth has no children. If it is neccessary that the node is at
	 * the specified depth, then the corresponding 'createNode' function can be used. The
	 * data inside the nodes returned by this function and 'createNode' will be the same, so
	 * it is only neccessary to use 'createNode' if you intend to alter what the node
	 * stores.
	 *
	 * @param code The code.
	 * @return The node.
	 */
	[[nodiscard]] Node operator()(Code code) const { return Node(code, index(code)); }

	/*!
	 * @brief Get the node corresponding to a key.
	 *
	 * @note The node can be higher up the tree than the specified depth. This happens if
	 * the node at a higher depth has no children. If it is neccessary that the node is at
	 * the specified depth, then the corresponding 'createNode' function can be used. The
	 * data inside the nodes returned by this function and 'createNode' will be the same, so
	 * it is only neccessary to use 'createNode' if you intend to alter what the node
	 * stores.
	 *
	 * @param key The key.
	 * @return The node.
	 */
	[[nodiscard]] Node operator()(Key key) const { return operator()(toCode(key)); }

	/*!
	 * @brief Get the node corresponding to a coordinate at a specific depth.
	 *
	 * @note The node can be higher up the tree than the specified depth. This happens if
	 * the node at a higher depth has no children. If it is neccessary that the node is at
	 * the specified depth, then the corresponding 'createNode' function can be used. The
	 * data inside the nodes returned by this function and 'createNode' will be the same, so
	 * it is only neccessary to use 'createNode' if you intend to alter what the node
	 * stores.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The node.
	 */
	[[nodiscard]] Node operator()(Point coord, depth_t depth = 0) const
	{
		return operator()(toCode(coord, depth));
	}

	/*!
	 * @brief Get the node corresponding to a coordinate at a specific depth.
	 *
	 * @note The node can be higher up the tree than the specified depth. This happens if
	 * the node at a higher depth has no children. If it is neccessary that the node is at
	 * the specified depth, then the corresponding 'createNode' function can be used. The
	 * data inside the nodes returned by this function and 'createNode' will be the same, so
	 * it is only neccessary to use 'createNode' if you intend to alter what the node
	 * stores.
	 *
	 * @param x,y,z The coordinate.
	 * @param depth The depth.
	 * @return The node.
	 */
	[[nodiscard]] Node operator()(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return operator()(toCode(x, y, z, depth));
	}

	//
	// Sibling
	//

	/*!
	 * @brief Get the sibling of a node.
	 *
	 * @param node The node.
	 * @param sibling_index The index of the sibling.
	 * @return The sibling.
	 */
	[[nodiscard]] Node sibling(Node node, offset_t sibling_index) const
	{
		assert(8 > sibling_index);
		return Node(node.code().sibling(sibling_index),
		            exists(node) ? node.index().sibling(sibling_index) : node.index());
	}

	/*!
	 * @brief Get the sibling of a node.
	 *
	 * @param node The node.
	 * @param sibling_index The index of the sibling.
	 * @return The sibling.
	 */
	[[nodiscard]] NodeBV sibling(NodeBV const& node, offset_t sibling_index) const
	{
		assert(8 > sibling_index);
		AABC aabc(siblingCenter(node.center(), node.halfSize(), node.offset(), sibling_index),
		          node.halfSize());
		return NodeBV(sibling(static_cast<Node>(node), sibling_index), aabc);
	}

	/*!
	 * @brief Get the sibling of a node with bounds checking.
	 *
	 * @param node The node.
	 * @param sibling_index The index of the sibling.
	 * @return The sibling.
	 */
	[[nodiscard]] Node siblingChecked(Node node, offset_t sibling_index) const
	{
		if (!isRoot(node)) {
			throw std::out_of_range("Node has no siblings");
		} else if (7 < sibling_index) {
			throw std::out_of_range("sibling_index out of range");
		}
		return sibling(node, sibling_index);
	}

	/*!
	 * @brief Get the sibling of a node with bounds checking.
	 *
	 * @param node The node.
	 * @param sibling_index The index of the sibling.
	 * @return The sibling.
	 */
	[[nodiscard]] NodeBV siblingChecked(NodeBV node, offset_t sibling_index) const
	{
		if (!isRoot(node)) {
			throw std::out_of_range("Node has no siblings");
		} else if (7 < sibling_index) {
			throw std::out_of_range("sibling_index out of range");
		}
		return sibling(node, sibling_index);
	}

	//
	// Child
	//

	/*!
	 * @brief Get a child of a node.
	 *
	 * @param node The node.
	 * @param child_index The index of the child.
	 * @return The child.
	 */
	[[nodiscard]] Node child(Node node, offset_t child_index) const
	{
		assert(8 > child_index);
		return Node(node.code().child(child_index),
		            isParent(node) ? child(node.index(), child_index) : node.index());
	}

	/*!
	 * @brief Get a child of a node.
	 *
	 * @param node The node.
	 * @param child_index The index of the child.
	 * @return The child.
	 */
	[[nodiscard]] NodeBV child(NodeBV const& node, offset_t child_index) const
	{
		assert(8 > child_index);
		auto const child_half_size = node.halfSize() / 2;
		AABC       child_aabc(childCenter(node.center(), child_half_size, child_index),
		                      child_half_size);

		return NodeBV(child(static_cast<Node>(node), child_index), child_aabc);
	}

	/*!
	 * @brief Get a child of a node with bounds checking.
	 *
	 * @param node The node.
	 * @param child_index The index of the child.
	 * @return The child.
	 */
	[[nodiscard]] Node childChecked(Node node, offset_t child_index) const
	{
		if (isLeaf(node)) {
			throw std::out_of_range("Node has no children");
		} else if (7 < child_index) {
			throw std::out_of_range("child_index out of range");
		} else {
			return child(node, child_index);
		}
	}

	/*!
	 * @brief Get a child of a node with bounds checking.
	 *
	 * @param node The node.
	 * @param child_index The index of the child.
	 * @return The child.
	 */
	[[nodiscard]] NodeBV childChecked(NodeBV const& node, offset_t child_index) const
	{
		if (isLeaf(node)) {
			throw std::out_of_range("Node has no children");
		} else if (7 < child_index) {
			throw std::out_of_range("child_index out of range");
		} else {
			return child(node, child_index);
		}
	}

	//
	// Parent
	//

	/*!
	 * @brief Get the parent of a node.
	 *
	 * @param node The node.
	 * @return The parent.
	 */
	[[nodiscard]] Node parent(Node node) const
	{
		assert(!isRoot(node));
		return operator()(node.code().parent());
	}

	/*!
	 * @brief Get the parent of a node.
	 *
	 * @param node The node.
	 * @return The parent.
	 */
	[[nodiscard]] NodeBV parent(NodeBV const& node) const
	{
		assert(!isRoot(node));
		AABC bv(parentCenter(node.center(), node.halfSize(), node.offset()), node.size());

		return NodeBV(parent(static_cast<Node>(node)), bv);
	}

	/*!
	 * @brief Get the parent of a node with bounds checking.
	 *
	 * @param node The node.
	 * @return The parent.
	 */
	[[nodiscard]] Node parentChecked(Node node) const
	{
		if (rootDepth() <= node.depth()) {
			throw std::out_of_range("Node has no parent");
		}
		return parent(node);
	}

	/*!
	 * @brief Get the parent of a node with bounds checking.
	 *
	 * @param node The node.
	 * @return The parent.
	 */
	[[nodiscard]] NodeBV parentChecked(NodeBV const& node) const
	{
		if (rootDepth() <= node.depth()) {
			throw std::out_of_range("Node has no parent");
		}
		return parent(node);
	}

	//
	// NodeBV
	//

	/*!
	 * @brief Convert a node to a node with bounding volume.
	 *
	 * @param node The node.
	 * @return The node with bounding volume.
	 */
	[[nodiscard]] NodeBV toNodeBV(Node node) const
	{
		return NodeBV(node, boundingVolume(node));
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Index                                        |
	|                                                                                     |
	**************************************************************************************/

	//
	// Index
	//

	[[nodiscard]] constexpr Index index(Node node) const
	{
		if (!isValid(node)) {
			return index(node.code());
		} else if (isCorrect(node) || isLeaf(node.index())) {
			return node.index();
		} else {
			return index(node.code(), node.index(), parent_code_[node.pos()].depth() - 1);
		}
	}

	[[nodiscard]] constexpr Index index(Code code) const
	{
		return index(code, rootIndex(), rootDepth());
	}

	[[nodiscard]] constexpr Index index(Key key) const { return index(toCode(key)); }

	[[nodiscard]] constexpr Index index(Point coord, depth_t depth = 0) const
	{
		return index(toCode(coord, depth));
	}

	[[nodiscard]] constexpr Index index(coord_t x, coord_t y, coord_t z,
	                                    depth_t depth = 0) const
	{
		return index(toCode(x, y, z, depth));
	}

	//
	// Depth
	//

	[[nodiscard]] depth_t depth(Index node) const
	{
		return parent_code_[node.pos].depth() - 1;
	}

	//
	// Center
	//

	[[nodiscard]] Point center(Index node) const
	{
		return toCoord(parent_code_[node.pos].child(node.offset));
	}

	//
	// Size
	//

	/*!
	 * @brief Get the size of a node.
	 *
	 * @param node The node.
	 * @return The size of the node.
	 */
	[[nodiscard]] constexpr node_size_t size(Index node) const { return size(depth(node)); }

	//
	// Bounding volume
	//

	/*!
	 * @brief Bounding volume for a node.
	 *
	 * @param node The node
	 * @return Bounding volume for the node.
	 */
	[[nodiscard]] AABC boundingVolume(Index node) const
	{
		return AABC(center(node), size(node) / 2);
	}

	//
	// Create
	//

	// IMPROVE: Make it so this requires that it is derived from CodeOrIndex
	template <std::input_iterator I, std::sentinel_for<I> S>
	void createIndicesFromCodes(I first, S last)
	{
		std::array<Index, maxDepthLevels()> nodes;
		auto                                depth = rootDepth();
		nodes[depth]                              = rootIndex();
		for (auto prev_code = rootCode(); first != last; ++first) {
			Code const c = first->code;
			for (; !Code::equalAtDepth(prev_code, c, depth); ++depth) {
			}

			prev_code = c;

			for (auto min_depth = c.depth(); min_depth < depth; --depth) {
				if (isLeaf(nodes[depth])) {
					// IMPROVE: Seems to be faster to have the code inline here
					// createChildren(nodes[depth]);

					if (free_children_.empty()) {
						children_[nodes[depth].pos][nodes[depth].offset] =
						    static_cast<pos_t>(children_.size());
						children_.push_back({NULL_POS, NULL_POS, NULL_POS, NULL_POS, NULL_POS,
						                     NULL_POS, NULL_POS, NULL_POS});
						parent_code_.emplace_back(
						    parent_code_[nodes[depth].pos].child(nodes[depth].offset));
						modified_.emplace_back(isModified(nodes[depth]) ? -1 : 0);
						derived().createNodeBlock(nodes[depth]);
					} else {
						children_[nodes[depth].pos][nodes[depth].offset] = free_children_.top();
						free_children_.pop();
						fill(nodes[depth], children(nodes[depth]));
					}
				}
				modified_[nodes[depth].pos].set(nodes[depth].offset);
				nodes[depth - 1] = child(nodes[depth], c.offset(depth - 1));
			}

			first->index = nodes[depth];
		}
	}

	// IMPROVE: Make it so this requires that it is derived from CodeOrIndex
	template <std::ranges::input_range R>
	void createIndicesFromCodes(R& r)
	{
		createIndicesFromCodes(std::ranges::begin(r), std::ranges::end(r));
	}

	[[nodiscard]] Index createIndex(Node node)
	{
		auto index = node.index();
		if (isCorrect(node)) {
			return isModified(index) ? index : createIndex(node.code());
		} else if (isValid(node) && isModified(index)) {
			for (depth_t d{depth(index)}, min_d{node.depth()}; min_d < d; --d) {
				createChildren(index);
				modified_[index.pos].set(index.offset);
				index = child(index, node.offset(d - 1));
			}
			return index;
		} else {
			return createIndex(node.code());
		}
	}

	[[nodiscard]] std::vector<Index> createIndex(std::vector<Code> const& codes)
	{
		std::vector<Index> indices;
		indices.reserve(codes.size());

		std::array<Index, maxDepthLevels()> nodes;
		auto                                depth = rootDepth();
		nodes[depth]                              = rootIndex();

		auto prev_code = rootCode();
		for (auto c : codes) {
			for (; !Code::equalAtDepth(prev_code, c, depth); ++depth) {
			}

			prev_code = c;

			for (auto min_depth = c.depth(); min_depth < depth; --depth) {
				createChildren(nodes[depth]);
				modified_[nodes[depth].pos].set(nodes[depth].offset);
				nodes[depth - 1] = child(nodes[depth], c.offset(depth - 1));
			}

			indices.push_back(nodes[depth]);
		}

		return indices;
	}

	[[nodiscard]] Index createIndex(Code code)
	{
		Index node = rootIndex();
		for (depth_t d{rootDepth()}, min_d{code.depth()}; min_d < d; --d) {
			createChildren(node);
			modified_[node.pos].set(node.offset);
			node = child(node, code.offset(d - 1));
		}
		return node;
	}

	[[nodiscard]] Index createIndex(Key key) { return createIndex(toCode(key)); }

	[[nodiscard]] Index createIndex(Point coord, depth_t depth = 0)
	{
		return createIndex(toCode(coord, depth));
	}

	[[nodiscard]] Index createIndex(coord_t x, coord_t y, coord_t z, depth_t depth = 0)
	{
		return createIndex(toCode(x, y, z, depth));
	}

	[[nodiscard]] Index createChild(Index node, offset_t c)
	{
		createChildren(node);
		modified_[node.pos].set(node.offset);
		return child(node, c);
	}

	// TODO: Add comment
	void createChildren(Index node)
	{
		if (isParent(node)) {
			return;
		}

		if (free_children_.empty()) {
			children_[node.pos][node.offset] = static_cast<pos_t>(children_.size());
			children_.push_back({NULL_POS, NULL_POS, NULL_POS, NULL_POS, NULL_POS, NULL_POS,
			                     NULL_POS, NULL_POS});
			parent_code_.emplace_back(parent_code_[node.pos].child(node.offset));
			modified_.emplace_back(isModified(node) ? -1 : 0);
			derived().createNodeBlock(node);
		} else {
			children_[node.pos][node.offset] = free_children_.top();
			free_children_.pop();
			fill(node, children(node));
		}
	}

	//
	// Child
	//

	[[nodiscard]] constexpr Index child(Index node, offset_t c) const
	{
		assert(8 > c);
		assert(isParent(node));
		return {children(node), c};
	}

	//
	// Sibling
	//

	[[nodiscard]] Index sibling(Index node, offset_t s) const
	{
		assert(8 > s);
		return {node.pos, s};
	}

	//
	// Parent
	//

	[[nodiscard]] Index parent(Index node) const
	{
		assert(!isRoot(node));
		return index(parent_code_[node.pos]);
	}

	// TODO: Implement

	/**************************************************************************************
	|                                                                                     |
	|                                     Conversion                                      |
	|                                                                                     |
	**************************************************************************************/

	//
	// Code
	//

	// TODO: Change name of this function
	[[nodiscard]] constexpr Code toCode(Index node) noexcept
	{
		return parent_code_[node.pos].child(node.offset);
	}

	/*!
	 * @brief Convert a key to a code.
	 *
	 * @param key The key to convert.
	 * @return The code corresponding to the key.
	 */
	[[nodiscard]] static constexpr Code toCode(Key key) noexcept
	{
		assert(maxDepthLevels() >= key.depth());
		return Code(key);
	}

	/*!
	 * @brief Convert a coordinate at a specific depth to a code.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The code corresponding to the coordinate at the specific depth.
	 */
	[[nodiscard]] constexpr Code toCode(Point coord, depth_t depth = 0) const noexcept
	{
		return toCode(toKey(coord, depth));
	}

	/*!
	 * @brief Convert a coordinate at a specific depth to a code.
	 *
	 * @param x,y,z The coordinate.
	 * @param depth The depth.
	 * @return The code corresponding to the coordinate at the specific depth.
	 */
	[[nodiscard]] constexpr Code toCode(coord_t x, coord_t y, coord_t z,
	                                    depth_t depth = 0) const noexcept
	{
		return toCode(toKey(x, y, z, depth));
	}

	/*!
	 * @brief Convert a coordinate at a specific depth to a code with bounds check.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The code corresponding to the coordinate at the specific depth.
	 */
	[[nodiscard]] constexpr std::optional<Code> toCodeChecked(
	    Point coord, depth_t depth = 0) const noexcept
	{
		auto key = toKeyChecked(coord, depth);
		return key ? std::optional<Code>(toCode(*key)) : std::nullopt;
	}

	/*!
	 * @brief Convert a coordinate at a specific depth to a code with bounds check.
	 *
	 * @param x,y,z The coordinate.
	 * @param depth The depth.
	 * @return The code corresponding to the coordinate at the specific depth.
	 */
	[[nodiscard]] constexpr std::optional<Code> toCodeChecked(
	    coord_t x, coord_t y, coord_t z, depth_t depth = 0) const noexcept
	{
		return toCodeChecked(Point(x, y, z), depth);
	}

	//
	// Key
	//

	/*!
	 * @brief Convert a code to a key.
	 *
	 * @param code The code to convert.
	 * @return The key corresponding to the code.
	 */
	[[nodiscard]] static constexpr Key toKey(Code code) noexcept { return code; }

	/*!
	 * @brief Convert a coordinate at a specific depth to a key.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The corresponding key.
	 */
	[[nodiscard]] constexpr Key toKey(Point coord, depth_t depth = 0) const noexcept
	{
		assert(isInside(coord));
		assert(rootDepth() >= depth);
		return Key(toKey(coord.x, depth), toKey(coord.y, depth), toKey(coord.z, depth),
		           depth);
	}

	/*!
	 * @brief Convert a coordinate at a specific depth to a key.
	 *
	 * @param x,y,z The coordinate.
	 * @param depth The depth.
	 * @return The corresponding key.
	 */
	[[nodiscard]] constexpr Key toKey(coord_t x, coord_t y, coord_t z,
	                                  depth_t depth = 0) const noexcept
	{
		assert(isInside(x, y, z));
		assert(rootDepth() >= depth);
		return Key(toKey(x, depth), toKey(y, depth), toKey(z, depth), depth);
	}

	/*!
	 * @brief Convert a coordinate at a specific depth to a key with bounds check.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The corresponding key.
	 */
	[[nodiscard]] constexpr std::optional<Key> toKeyChecked(
	    Point coord, depth_t depth = 0) const noexcept
	{
		return rootDepth() >= depth && isInside(coord)
		           ? std::optional<Key>(toKey(coord, depth))
		           : std::nullopt;
	}

	/*!
	 * @brief Convert a coordinate at a specific depth to a key with bounds check.
	 *
	 * @param x,y,z The coordinate.
	 * @param depth The depth.
	 * @return The corresponding key.
	 */
	[[nodiscard]] constexpr std::optional<Key> toKeyChecked(
	    coord_t x, coord_t y, coord_t z, depth_t depth = 0) const noexcept
	{
		return toKeyChecked(Point(x, y, z), depth);
	}

	//
	// Coordinate
	//

	/*!
	 * @brief Convert a code to a coordinate.
	 *
	 * @param code The code.
	 * @return The corresponding coordinate.
	 */
	[[nodiscard]] constexpr Point toCoord(Code code) const noexcept
	{
		return toCoord(toKey(code));
	}

	/*!
	 * @brief Convert a key to a coordinate.
	 *
	 * @param key The key.
	 * @return The corresponding coordinate.
	 */
	[[nodiscard]] constexpr Point toCoord(Key key) const noexcept
	{
		return Point(toCoord(key[0], key.depth()), toCoord(key[1], key.depth()),
		             toCoord(key[2], key.depth()));
	}

	/*!
	 * @brief Convert a code to a coordinate with bounds check.
	 *
	 * @param Code The code.
	 * @return The corresponding coordinate.
	 */
	[[nodiscard]] constexpr std::optional<Point> toCoordChecked(Code code) const noexcept
	{
		return toCoordChecked(toKey(code));
	}

	/*!
	 * @brief Convert a key to a coordinate with bounds check.
	 *
	 * @param key The key.
	 * @return The corresponding coordinate.
	 */
	[[nodiscard]] constexpr std::optional<Point> toCoordChecked(Key key) const noexcept
	{
		return rootDepth() >= key.depth() ? std::optional<Point>(toCoord(key)) : std::nullopt;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Traverse                                       |
	|                                                                                     |
	**************************************************************************************/

	/*!
	 * @brief Depth first traversal of the octree, starting at the root node. The function
	 * 'f' will be called for each node traverse. If 'f' returns true then the children of
	 * the node will also be traverse, otherwise they will not.
	 *
	 * @param f The callback function to be called for each node traversed.
	 */
	void traverse(std::invocable<Node> auto f, bool only_exists = true) const
	{
		traverse(rootNode(), f, only_exists);
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the root node. The function
	 * 'f' will be called for each node traverse. If 'f' returns true then the children of
	 * the node will also be traverse, otherwise they will not.
	 *
	 * @param f The callback function to be called for each node traversed.
	 */
	// void traverse(std::invocable<NodeBV> auto f, bool only_exists = true) const
	// {
	// 	traverse(rootNode(), f, only_exists);
	// }

	/*!
	 * @brief Depth first traversal of the octree, starting at the node. The function 'f'
	 * will be called for each node traverse. If 'f' returns true then the children of the
	 * node will also be traverse, otherwise they will not.
	 *
	 * @param node The node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 */
	void traverse(Node node, std::invocable<Node> auto f, bool only_exists = true) const
	{
		std::array<Node, maxDepthLevels()> nodes;
		nodes[0] = node;

		if (only_exists) {
			if (!exists(node)) {
				return;
			}
			for (int depth{}; 0 <= depth;) {
				node        = nodes[depth];
				auto offset = nodes[depth].offset();
				if (7 > offset) {
					nodes[depth] = sibling(nodes[depth], offset + 1);
				} else {
					--depth;
				}
				if (f(node) && isParent(node)) {
					nodes[++depth] = child(node, 0);
				}
			}
		} else {
			for (int depth{}; 0 <= depth;) {
				node        = nodes[depth];
				auto offset = nodes[depth].offset();
				if (7 > offset) {
					nodes[depth] = sibling(nodes[depth], offset + 1);
				} else {
					--depth;
				}
				if (f(node) && !isPureLeaf(node)) {
					nodes[++depth] = child(node, 0);
				}
			}
		}
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node. The function 'f'
	 * will be called for each node traverse. If 'f' returns true then the children of the
	 * node will also be traverse, otherwise they will not.
	 *
	 * @param node The node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 */
	void traverse(Node node, std::invocable<NodeBV> auto f, bool only_exists = true) const
	{
		std::array<NodeBV, maxDepthLevels()> nodes;
		nodes[0] = toNodeBV(node);

		if (only_exists) {
			if (!exists(node)) {
				return;
			}

			for (int depth{}; 0 <= depth;) {
				node        = nodes[depth];
				auto offset = nodes[depth].offset();
				if (7 > offset) {
					nodes[depth] = sibling(nodes[depth], offset + 1);
				} else {
					--depth;
				}
				if (f(node) && isParent(node)) {
					nodes[++depth] = child(node, 0);
				}
			}
		} else {
			for (int depth{}; 0 <= depth;) {
				node        = nodes[depth];
				auto offset = nodes[depth].offset();
				if (7 > offset) {
					nodes[depth] = sibling(nodes[depth], offset + 1);
				} else {
					--depth;
				}
				if (f(node) && !isPureLeaf(node)) {
					nodes[++depth] = child(node, 0);
				}
			}
		}
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node corresponding to the
	 * code. The function 'f' will be called for each node traverse. If 'f' returns true
	 * then the children of the node will also be traverse, otherwise they will not.
	 *
	 * @param code The code to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 */
	void traverse(Code code, std::invocable<Node> auto f, bool only_exists = true) const
	{
		traverse(operator()(code), f, only_exists);
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node corresponding to the
	 * code. The function 'f' will be called for each node traverse. If 'f' returns true
	 * then the children of the node will also be traverse, otherwise they will not.
	 *
	 * @param code The code to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 */
	void traverse(Code code, std::invocable<NodeBV> auto f, bool only_exists = true) const
	{
		traverse(operator()(code), f, only_exists);
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node corresponding to the
	 * key. The function 'f' will be called for each node traverse. If 'f' returns true
	 * then the children of the node will also be traverse, otherwise they will not.
	 *
	 * @param key The key to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 */
	void traverse(Key key, std::invocable<Node> auto f, bool only_exists = true) const
	{
		traverse(toCode(key), f, only_exists);
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node corresponding to the
	 * key. The function 'f' will be called for each node traverse. If 'f' returns true
	 * then the children of the node will also be traverse, otherwise they will not.
	 *
	 * @param key The key to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 */
	void traverse(Key key, std::invocable<NodeBV> auto f, bool only_exists = true) const
	{
		traverse(toCode(key), f, only_exists);
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node corresponding to the
	 * coordinate at a specified depth. The function 'f' will be called for each node
	 * traverse. If 'f' returns true then the children of the node will also be traverse,
	 * otherwise they will not.
	 *
	 * @param coord The coord to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 * @param depth The depth of the node.
	 */
	void traverse(Point coord, std::invocable<Node> auto f, bool only_exists = true,
	              depth_t depth = 0) const
	{
		traverse(toCode(coord, depth), f, only_exists);
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node corresponding to the
	 * coordinate at a specified depth. The function 'f' will be called for each node
	 * traverse. If 'f' returns true then the children of the node will also be traverse,
	 * otherwise they will not.
	 *
	 * @param coord The coord to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 * @param depth The depth of the node.
	 */
	void traverse(Point coord, std::invocable<NodeBV> auto f, bool only_exists = true,
	              depth_t depth = 0) const
	{
		traverse(toCode(coord, depth), f, only_exists);
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node corresponding to the
	 * coordinate at a specified depth. The function 'f' will be called for each node
	 * traverse. If 'f' returns true then the children of the node will also be traverse,
	 * otherwise they will not.
	 *
	 * @param x,y,z The coord to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 * @param depth The depth of the node.
	 */
	void traverse(coord_t x, coord_t y, coord_t z, std::invocable<Node> auto f,
	              bool only_exists = true, depth_t depth = 0) const
	{
		traverse(toCode(x, y, z, depth), f, only_exists);
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node corresponding to the
	 * coordinate at a specified depth. The function 'f' will be called for each node
	 * traverse. If 'f' returns true then the children of the node will also be traverse,
	 * otherwise they will not.
	 *
	 * @param x,y,z The coord to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 * @param depth The depth of the node.
	 */
	void traverse(coord_t x, coord_t y, coord_t z, std::invocable<NodeBV> auto f,
	              bool only_exists = true, depth_t depth = 0) const
	{
		traverse(toCode(x, y, z, depth), f, only_exists);
	}

	/*! TODO: Update info for all nearest
	 * @brief Traverse the octree in the orderDepth first traversal of the octree, starting
	 * at the root node. The function 'f' will be called for each node traverse. If 'f'
	 * returns true then the children of the node will also be traverse, otherwise they
	 * will not.
	 *
	 * @param f The callback function to be called for each node traversed.
	 */
	template <class Geometry>
	void traverseNearest(Geometry const& g, std::invocable<NearestNode> auto f,
	                     bool only_exists = true) const
	{
		traverseNearest(rootNode(), g, f, only_exists);
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node. The function 'f'
	 * will be called for each node traverse. If 'f' returns true then the children of the
	 * node will also be traverse, otherwise they will not.
	 *
	 * @param node The node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 */
	template <class Geometry>
	void traverseNearest(Node node, Geometry const& g, std::invocable<NearestNode> auto f,
	                     bool only_exists = true) const
	{
		std::priority_queue<NearestNode, std::vector<NearestNode>, std::greater<NearestNode>>
		       nodes;
		NodeBV nbv = toNodeBV(node);
		nodes.emplace(nbv, squaredDistance(nbv.boundingVolume(), g));

		if (only_exists) {
			if (!exists(node)) {
				return;
			}
			while (!nodes.empty()) {
				auto n_d = nodes.top();
				nodes.pop();

				if (f(n_d) && isParent(n_d)) {
					for (offset_t i{}; 8 != i; ++i) {
						auto c = child(n_d, i);
						nodes.emplace(c, squaredDistance(c.boundingVolume(), g));
					}
				}
			}
		} else {
			while (!nodes.empty()) {
				auto n_d = nodes.top();
				nodes.pop();

				if (f(n_d) && !isPureLeaf(n_d)) {
					for (offset_t i{}; 8 != i; ++i) {
						auto c = child(n_d, i);
						nodes.emplace(c, squaredDistance(c.boundingVolume(), g));
					}
				}
			}
		}
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node corresponding to the
	 * code. The function 'f' will be called for each node traverse. If 'f' returns true
	 * then the children of the node will also be traverse, otherwise they will not.
	 *
	 * @param code The code to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 */
	template <class Geometry>
	void traverseNearest(Code code, Geometry const& g, std::invocable<NearestNode> auto f,
	                     bool only_exists = true) const
	{
		traverseNearest(operator()(code), g, f, only_exists);
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node corresponding to the
	 * key. The function 'f' will be called for each node traverse. If 'f' returns true
	 * then the children of the node will also be traverse, otherwise they will not.
	 *
	 * @param key The key to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 */
	template <class Geometry>
	void traverseNearest(Key key, Geometry const& g, std::invocable<NearestNode> auto f,
	                     bool only_exists = true) const
	{
		traverseNearest(toCode(key), g, f, only_exists);
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node corresponding to the
	 * coordinate at a specified depth. The function 'f' will be called for each node
	 * traverse. If 'f' returns true then the children of the node will also be traverse,
	 * otherwise they will not.
	 *
	 * @param coord The coord to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 * @param depth The depth of the node.
	 */
	template <class Geometry>
	void traverseNearest(Point coord, Geometry const& g, std::invocable<NearestNode> auto f,
	                     bool only_exists = true, depth_t depth = 0) const
	{
		traverseNearest(toCode(coord, depth), g, f, only_exists);
	}

	/*!
	 * @brief Depth first traversal of the octree, starting at the node corresponding to the
	 * coordinate at a specified depth. The function 'f' will be called for each node
	 * traverse. If 'f' returns true then the children of the node will also be traverse,
	 * otherwise they will not.
	 *
	 * @param x,y,z The coord to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 * @param depth The depth of the node.
	 */
	template <class Geometry>
	void traverseNearest(coord_t x, coord_t y, coord_t z, Geometry const& g,
	                     std::invocable<NearestNode> auto f, bool only_exists = true,
	                     depth_t depth = 0) const
	{
		traverseNearest(toCode(x, y, z, depth), g, f, only_exists);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Query                                        |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comments

	//
	// Query
	//

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] Query query(Predicate&& predicate, bool only_exists = true,
	                          bool early_stopping = false) const
	{
		return query(rootNode(), std::forward<Predicate>(predicate), only_exists,
		             early_stopping);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] Query query(Node node, Predicate&& predicate, bool only_exists = true,
	                          bool early_stopping = false) const
	{
		return Query(
		    beginQuery(node, std::forward<Predicate>(predicate), only_exists, early_stopping),
		    endQuery());
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] Query query(Code code, Predicate&& predicate, bool only_exists = true,
	                          bool early_stopping = false) const
	{
		return query(operator()(code), std::forward<Predicate>(predicate), only_exists,
		             early_stopping);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] Query query(Key key, Predicate&& predicate, bool only_exists = true,
	                          bool early_stopping = false) const
	{
		return query(toCode(key), std::forward<Predicate>(predicate), only_exists,
		             early_stopping);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] Query query(Point coord, depth_t depth, Predicate&& predicate,
	                          bool only_exists = true, bool early_stopping = false) const
	{
		return query(toCode(coord, depth), std::forward<Predicate>(predicate), only_exists,
		             early_stopping);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] Query query(coord_t x, coord_t y, coord_t z, depth_t depth,
	                          Predicate&& predicate, bool only_exists = true,
	                          bool early_stopping = false) const
	{
		return query(toCode(x, y, z, depth), std::forward<Predicate>(predicate), only_exists,
		             early_stopping);
	}

	//
	// Query bounding volume
	//

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] QueryBV queryBV(Predicate&& predicate, bool only_exists = true,
	                              bool early_stopping = false) const
	{
		return queryBV(rootNode(), std::forward<Predicate>(predicate), only_exists,
		               early_stopping);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] QueryBV queryBV(Node node, Predicate&& predicate, bool only_exists = true,
	                              bool early_stopping = false) const
	{
		return QueryBV(beginQueryBV(node, std::forward<Predicate>(predicate), only_exists,
		                            early_stopping),
		               endQueryBV());
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] QueryBV queryBV(Code code, Predicate&& predicate, bool only_exists = true,
	                              bool early_stopping = false) const
	{
		return queryBV(operator()(code), std::forward<Predicate>(predicate), only_exists,
		               early_stopping);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] QueryBV queryBV(Key key, Predicate&& predicate, bool only_exists = true,
	                              bool early_stopping = false) const
	{
		return queryBV(toCode(key), std::forward<Predicate>(predicate), only_exists,
		               early_stopping);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] QueryBV queryBV(Point coord, depth_t depth, Predicate&& predicate,
	                              bool only_exists    = true,
	                              bool early_stopping = false) const
	{
		return queryBV(toCode(coord, depth), std::forward<Predicate>(predicate), only_exists,
		               early_stopping);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] QueryBV queryBV(coord_t x, coord_t y, coord_t z, depth_t depth,
	                              Predicate&& predicate, bool only_exists = true,
	                              bool early_stopping = false) const
	{
		return queryBV(toCode(x, y, z, depth), std::forward<Predicate>(predicate),
		               only_exists, early_stopping);
	}

	//
	// Query nearest
	//

	template <class Geometry, class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] QueryNearest queryNearest(Geometry&& geometry, Predicate&& predicate,
	                                        bool only_exists    = true,
	                                        bool early_stopping = false) const
	{
		return queryNearest(rootNode(), std::forward<Geometry>(geometry),
		                    std::forward<Predicate>(predicate), only_exists, early_stopping);
	}

	template <class Geometry, class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] QueryNearest queryNearest(Node node, Geometry&& geometry,
	                                        Predicate&& predicate, bool only_exists = true,
	                                        bool early_stopping = false) const
	{
		return QueryNearest(beginQueryNearest(node, std::forward<Geometry>(geometry),
		                                      std::forward<Predicate>(predicate), only_exists,
		                                      early_stopping),
		                    endQueryNearest());
	}

	template <class Geometry, class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] QueryNearest queryNearest(Code code, Geometry&& geometry,
	                                        Predicate&& predicate, bool only_exists = true,
	                                        bool early_stopping = false) const
	{
		return queryNearest(operator()(code), std::forward<Geometry>(geometry),
		                    std::forward<Predicate>(predicate), only_exists, early_stopping);
	}

	template <class Geometry, class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] QueryNearest queryNearest(Key key, Geometry&& geometry,
	                                        Predicate&& predicate, bool only_exists = true,
	                                        bool early_stopping = false) const
	{
		return queryNearest(toCode(key), std::forward<Geometry>(geometry),
		                    std::forward<Predicate>(predicate), only_exists, early_stopping);
	}

	template <class Geometry, class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] QueryNearest queryNearest(Point coord, depth_t depth, Geometry&& geometry,
	                                        Predicate&& predicate, bool only_exists = true,
	                                        bool early_stopping = false) const
	{
		return queryNearest(toCode(coord, depth), std::forward<Geometry>(geometry),
		                    std::forward<Predicate>(predicate), only_exists, early_stopping);
	}

	template <class Geometry, class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] QueryNearest queryNearest(coord_t x, coord_t y, coord_t z, depth_t depth,
	                                        Geometry&& geometry, Predicate&& predicate,
	                                        bool only_exists    = true,
	                                        bool early_stopping = false) const
	{
		return queryNearest(toCode(x, y, z, depth), std::forward<Geometry>(geometry),
		                    std::forward<Predicate>(predicate), only_exists, early_stopping);
	}

	//
	// Query to output
	//

	template <class Predicate, class OutputIt>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	OutputIt query(Predicate&& predicate, OutputIt d_first, bool only_exists = true,
	               bool early_stopping = false) const
	{
		return query(rootNode(), std::forward<Predicate>(predicate), d_first, only_exists,
		             early_stopping);
	}

	template <class Predicate, class OutputIt>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	OutputIt query(Node node, Predicate&& predicate, OutputIt d_first,
	               bool only_exists = true, bool early_stopping = false) const
	{
		if constexpr (std::is_same_v<typename OutputIt::value_type, Node>) {
			return std::ranges::copy(
			    query(node, std::forward<Predicate>(predicate), only_exists, early_stopping),
			    d_first);
		} else {
			return std::ranges::copy(
			    queryBV(node, std::forward<Predicate>(predicate), only_exists, early_stopping),
			    d_first);
		}
	}

	template <class Predicate, class OutputIt>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	OutputIt query(Code code, Predicate&& predicate, OutputIt d_first,
	               bool only_exists = true, bool early_stopping = false) const
	{
		return query(operator()(code), std::forward<Predicate>(predicate), d_first,
		             only_exists, early_stopping);
	}

	template <class Predicate, class OutputIt>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	OutputIt query(Key key, Predicate&& predicate, OutputIt d_first,
	               bool only_exists = true, bool early_stopping = false) const
	{
		return query(toCode(key), std::forward<Predicate>(predicate), d_first, only_exists,
		             early_stopping);
	}

	template <class Predicate, class OutputIt>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	OutputIt query(Point coord, depth_t depth, Predicate&& predicate, OutputIt d_first,
	               bool only_exists = true, bool early_stopping = false) const
	{
		return query(toCode(coord, depth), std::forward<Predicate>(predicate), d_first,
		             only_exists, early_stopping);
	}

	template <class Predicate, class OutputIt>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	OutputIt query(coord_t x, coord_t y, coord_t z, depth_t depth, Predicate&& predicate,
	               OutputIt d_first, bool only_exists = true,
	               bool early_stopping = false) const
	{
		return query(toCode(x, y, z, depth), std::forward<Predicate>(predicate), d_first,
		             only_exists, early_stopping);
	}

	template <class Predicate, class OutputIt>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	OutputIt queryK(std::size_t k, Predicate&& predicate, OutputIt d_first,
	                bool only_exists = true, bool early_stopping = false) const
	{
		return queryK(rootNode(), k, std::forward<Predicate>(predicate), d_first, only_exists,
		              early_stopping);
	}

	template <class Predicate, class OutputIt>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	OutputIt queryK(Node node, std::size_t k, Predicate&& predicate, OutputIt d_first,
	                bool only_exists = true, bool early_stopping = false) const
	{
		if constexpr (std::is_same_v<typename OutputIt::value_type, Node>) {
			return std::ranges::copy(
			    query(node, std::forward<Predicate>(predicate), only_exists, early_stopping) |
			        std::views::take(k),
			    d_first);
		} else {
			return std::ranges::copy(
			    queryBV(node, std::forward<Predicate>(predicate), only_exists, early_stopping) |
			        std::views::take(k),
			    d_first);
		}
	}

	template <class Predicate, class OutputIt>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	OutputIt queryK(Code code, std::size_t k, Predicate&& predicate, OutputIt d_first,
	                bool only_exists = true, bool early_stopping = false) const
	{
		return queryK(operator()(code), k, std::forward<Predicate>(predicate), d_first,
		              only_exists, early_stopping);
	}

	template <class Predicate, class OutputIt>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	OutputIt queryK(Key key, std::size_t k, Predicate&& predicate, OutputIt d_first,
	                bool only_exists = true, bool early_stopping = false) const
	{
		return queryK(toCode(key), k, std::forward<Predicate>(predicate), d_first,
		              only_exists, early_stopping);
	}

	template <class Predicate, class OutputIt>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	OutputIt queryK(Point coord, depth_t depth, std::size_t k, Predicate&& predicate,
	                OutputIt d_first, bool only_exists = true,
	                bool early_stopping = false) const
	{
		return queryK(toCode(coord, depth), k, std::forward<Predicate>(predicate), d_first,
		              only_exists, early_stopping);
	}

	template <class Predicate, class OutputIt>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	OutputIt queryK(coord_t x, coord_t y, coord_t z, depth_t depth, std::size_t k,
	                Predicate&& predicate, OutputIt d_first, bool only_exists = true,
	                bool early_stopping = false) const
	{
		return queryK(toCode(x, y, z, depth), k, std::forward<Predicate>(predicate), d_first,
		              only_exists, early_stopping);
	}

	template <class Geometry, class Predicate, class OutputIt>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	OutputIt queryNearest(Geometry&& geometry, Predicate&& predicate, OutputIt d_first,
	                      double epsilon = 0.0, bool only_exists = true,
	                      bool early_stopping = false) const
	{
		return queryNearest(rootNode(), std::forward<Geometry>(geometry),
		                    std::forward<Predicate>(predicate), d_first, epsilon, only_exists,
		                    early_stopping);
	}

	template <class Geometry, class Predicate, class OutputIt>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	OutputIt queryNearest(Node node, Geometry&& geometry, Predicate&& predicate,
	                      OutputIt d_first, double epsilon = 0.0, bool only_exists = true,
	                      bool early_stopping = false) const
	{
		return std::ranges::copy(queryNearest(node, std::forward<Geometry>(geometry),
		                                      std::forward<Predicate>(predicate), epsilon,
		                                      only_exists, early_stopping),
		                         d_first);
	}

	template <class Geometry, class Predicate, class OutputIt>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	OutputIt queryNearest(Code code, Geometry&& geometry, Predicate&& predicate,
	                      OutputIt d_first, double epsilon = 0.0, bool only_exists = true,
	                      bool early_stopping = false) const
	{
		return queryNearest(operator()(code), std::forward<Geometry>(geometry),
		                    std::forward<Predicate>(predicate), d_first, epsilon, only_exists,
		                    early_stopping);
	}

	template <class Geometry, class Predicate, class OutputIt>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	OutputIt queryNearest(Key key, Geometry&& geometry, Predicate&& predicate,
	                      OutputIt d_first, double epsilon = 0.0, bool only_exists = true,
	                      bool early_stopping = false) const
	{
		return queryNearest(toCode(key), std::forward<Geometry>(geometry),
		                    std::forward<Predicate>(predicate), d_first, epsilon, only_exists,
		                    early_stopping);
	}

	template <class Geometry, class Predicate, class OutputIt>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	OutputIt queryNearest(Point coord, depth_t depth, Geometry&& geometry,
	                      Predicate&& predicate, OutputIt d_first, double epsilon = 0.0,
	                      bool only_exists = true, bool early_stopping = false) const
	{
		return queryNearest(toCode(coord, depth), std::forward<Geometry>(geometry),
		                    std::forward<Predicate>(predicate), d_first, epsilon, only_exists,
		                    early_stopping);
	}

	template <class Geometry, class Predicate, class OutputIt>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	OutputIt queryNearest(coord_t x, coord_t y, coord_t z, depth_t depth,
	                      Geometry&& geometry, Predicate&& predicate, OutputIt d_first,
	                      double epsilon = 0.0, bool only_exists = true,
	                      bool early_stopping = false) const
	{
		return queryNearest(toCode(x, y, z, depth), std::forward<Geometry>(geometry),
		                    std::forward<Predicate>(predicate), d_first, epsilon, only_exists,
		                    early_stopping);
	}

	template <class Geometry, class Predicate, class OutputIt>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	OutputIt queryNearestK(std::size_t k, Geometry&& geometry, Predicate&& predicate,
	                       OutputIt d_first, double epsilon = 0.0, bool only_exists = true,
	                       bool early_stopping = false) const
	{
		return queryNearestK(rootNode(), k, std::forward<Geometry>(geometry),
		                     std::forward<Predicate>(predicate), d_first, epsilon,
		                     only_exists, early_stopping);
	}

	template <class Geometry, class Predicate, class OutputIt>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	OutputIt queryNearestK(Node node, std::size_t k, Geometry&& geometry,
	                       Predicate&& predicate, OutputIt d_first, double epsilon = 0.0,
	                       bool only_exists = true, bool early_stopping = false) const
	{
		return std::ranges::copy(queryNearest(node, std::forward<Geometry>(geometry),
		                                      std::forward<Predicate>(predicate), epsilon,
		                                      only_exists, early_stopping) |
		                             std::views::take(k),
		                         d_first);
	}

	template <class Geometry, class Predicate, class OutputIt>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	OutputIt queryNearestK(Code code, std::size_t k, Geometry&& geometry,
	                       Predicate&& predicate, OutputIt d_first, double epsilon = 0.0,
	                       bool only_exists = true, bool early_stopping = false) const
	{
		return queryNearestK(operator()(code), k, std::forward<Geometry>(geometry),
		                     std::forward<Predicate>(predicate), d_first, epsilon,
		                     only_exists, early_stopping);
	}

	template <class Geometry, class Predicate, class OutputIt>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	OutputIt queryNearestK(Key key, std::size_t k, Geometry&& geometry,
	                       Predicate&& predicate, OutputIt d_first, double epsilon = 0.0,
	                       bool only_exists = true, bool early_stopping = false) const
	{
		return queryNearestK(toCode(key), k, std::forward<Geometry>(geometry),
		                     std::forward<Predicate>(predicate), d_first, epsilon,
		                     only_exists, early_stopping);
	}

	template <class Geometry, class Predicate, class OutputIt>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	OutputIt queryNearestK(Point coord, depth_t depth, std::size_t k, Geometry&& geometry,
	                       Predicate&& predicate, OutputIt d_first, double epsilon = 0.0,
	                       bool only_exists = true, bool early_stopping = false) const
	{
		return queryNearestK(toCode(coord, depth), k, std::forward<Geometry>(geometry),
		                     std::forward<Predicate>(predicate), d_first, epsilon,
		                     only_exists, early_stopping);
	}

	template <class Geometry, class Predicate, class OutputIt>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	OutputIt queryNearestK(coord_t x, coord_t y, coord_t z, depth_t depth, std::size_t k,
	                       Geometry&& geometry, Predicate&& predicate, OutputIt d_first,
	                       double epsilon = 0.0, bool only_exists = true,
	                       bool early_stopping = false) const
	{
		return queryNearestK(toCode(x, y, z, depth), k, std::forward<Geometry>(geometry),
		                     std::forward<Predicate>(predicate), d_first, epsilon,
		                     only_exists, early_stopping);
	}

	//
	// Query iterator
	//

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] const_query_iterator beginQuery(Predicate&& predicate,
	                                              bool        only_exists    = true,
	                                              bool        early_stopping = false) const
	{
		return beginQuery(rootNode(), std::forward<Predicate>(predicate), only_exists,
		                  early_stopping);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] const_query_iterator beginQuery(Node node, Predicate&& predicate,
	                                              bool only_exists    = true,
	                                              bool early_stopping = false) const
	{
		if (only_exists) {
			if (early_stopping) {
				if constexpr (pred::contains_spatial_predicate_v<std::decay_t<Predicate>>) {
					return const_query_iterator(new OctreeIterator<Node, true, true, Derived,
					                                               NodeBV, std::decay_t<Predicate>>(
					    &derived(), toNodeBV(node), std::forward<Predicate>(predicate)));
				} else {
					return const_query_iterator(new OctreeIterator<Node, true, true, Derived, Node,
					                                               std::decay_t<Predicate>>(
					    &derived(), node, std::forward<Predicate>(predicate)));
				}
			} else {
				if constexpr (pred::contains_spatial_predicate_v<std::decay_t<Predicate>>) {
					return const_query_iterator(new OctreeIterator<Node, true, false, Derived,
					                                               NodeBV, std::decay_t<Predicate>>(
					    &derived(), toNodeBV(node), std::forward<Predicate>(predicate)));
				} else {
					return const_query_iterator(new OctreeIterator<Node, true, false, Derived, Node,
					                                               std::decay_t<Predicate>>(
					    &derived(), node, std::forward<Predicate>(predicate)));
				}
			}
		} else {
			if (early_stopping) {
				if constexpr (pred::contains_spatial_predicate_v<std::decay_t<Predicate>>) {
					return const_query_iterator(new OctreeIterator<Node, false, true, Derived,
					                                               NodeBV, std::decay_t<Predicate>>(
					    &derived(), toNodeBV(node), std::forward<Predicate>(predicate)));
				} else {
					return const_query_iterator(new OctreeIterator<Node, false, true, Derived, Node,
					                                               std::decay_t<Predicate>>(
					    &derived(), node, std::forward<Predicate>(predicate)));
				}
			} else {
				if constexpr (pred::contains_spatial_predicate_v<std::decay_t<Predicate>>) {
					return const_query_iterator(new OctreeIterator<Node, false, false, Derived,
					                                               NodeBV, std::decay_t<Predicate>>(
					    &derived(), toNodeBV(node), std::forward<Predicate>(predicate)));
				} else {
					return const_query_iterator(new OctreeIterator<Node, false, false, Derived,
					                                               Node, std::decay_t<Predicate>>(
					    &derived(), node, std::forward<Predicate>(predicate)));
				}
			}
		}
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] const_query_iterator beginQuery(Code code, Predicate&& predicate,
	                                              bool only_exists    = true,
	                                              bool early_stopping = false) const
	{
		return beginQuery(operator()(code), std::forward<Predicate>(predicate), only_exists,
		                  early_stopping);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] const_query_iterator beginQuery(Key key, Predicate&& predicate,
	                                              bool only_exists    = true,
	                                              bool early_stopping = false) const
	{
		return beginQuery(toCode(key), std::forward<Predicate>(predicate), only_exists,
		                  early_stopping);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] const_query_iterator beginQuery(Point coord, depth_t depth,
	                                              Predicate&& predicate,
	                                              bool        only_exists    = true,
	                                              bool        early_stopping = false) const
	{
		return beginQuery(toCode(coord, depth), std::forward<Predicate>(predicate),
		                  only_exists, early_stopping);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] const_query_iterator beginQuery(coord_t x, coord_t y, coord_t z,
	                                              depth_t depth, Predicate&& predicate,
	                                              bool only_exists    = true,
	                                              bool early_stopping = false) const
	{
		return beginQuery(toCode(x, y, z, depth), std::forward<Predicate>(predicate),
		                  only_exists, early_stopping);
	}

	[[nodiscard]] const_query_iterator endQuery() const
	{
		return const_query_iterator(
		    new OctreeIterator<Node, true, true, Derived>(&derived()));
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] const_bounding_volume_query_iterator beginQueryBV(
	    Predicate&& predicate, bool only_exists = true, bool early_stopping = false) const
	{
		return beginQueryBV(rootNode(), std::forward<Predicate>(predicate), only_exists,
		                    early_stopping);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] const_bounding_volume_query_iterator beginQueryBV(
	    Node node, Predicate&& predicate, bool only_exists = true,
	    bool early_stopping = false) const
	{
		if (only_exists) {
			if (early_stopping) {
				return const_bounding_volume_query_iterator(
				    new OctreeIterator<NodeBV, true, true, Derived, NodeBV, Predicate>(
				        &derived(), toNodeBV(node), std::forward<Predicate>(predicate)));
			} else {
				return const_bounding_volume_query_iterator(
				    new OctreeIterator<NodeBV, true, false, Derived, NodeBV, Predicate>(
				        &derived(), toNodeBV(node), std::forward<Predicate>(predicate)));
			}
		} else {
			if (early_stopping) {
				return const_bounding_volume_query_iterator(
				    new OctreeIterator<NodeBV, false, true, Derived, NodeBV, Predicate>(
				        &derived(), toNodeBV(node), std::forward<Predicate>(predicate)));
			} else {
				return const_bounding_volume_query_iterator(
				    new OctreeIterator<NodeBV, false, false, Derived, NodeBV, Predicate>(
				        &derived(), toNodeBV(node), std::forward<Predicate>(predicate)));
			}
		}
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] const_bounding_volume_query_iterator beginQueryBV(
	    Code code, Predicate&& predicate, bool only_exists = true,
	    bool early_stopping = false) const
	{
		return beginQueryBV(operator()(code), std::forward<Predicate>(predicate), only_exists,
		                    early_stopping);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] const_bounding_volume_query_iterator beginQueryBV(
	    Key key, Predicate&& predicate, bool only_exists = true,
	    bool early_stopping = false) const
	{
		return beginQueryBV(toCode(key), std::forward<Predicate>(predicate), only_exists,
		                    early_stopping);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] const_bounding_volume_query_iterator beginQueryBV(
	    Point coord, depth_t depth, Predicate&& predicate, bool only_exists = true,
	    bool early_stopping = false) const
	{
		return beginQueryBV(toCode(coord, depth), std::forward<Predicate>(predicate),
		                    only_exists, early_stopping);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] const_bounding_volume_query_iterator beginQueryBV(
	    coord_t x, coord_t y, coord_t z, depth_t depth, Predicate&& predicate,
	    bool only_exists = true, bool early_stopping = false) const
	{
		return beginQueryBV(toCode(x, y, z, depth), std::forward<Predicate>(predicate),
		                    only_exists, early_stopping);
	}

	[[nodiscard]] const_bounding_volume_query_iterator endQueryBV() const
	{
		return const_bounding_volume_query_iterator(
		    new OctreeIterator<NodeBV, true, true, Derived, NodeBV>(&derived()));
	}

	template <class Geometry, class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] const_query_nearest_iterator beginQueryNearest(
	    Geometry&& geometry, Predicate&& predicate, double epsilon = 0.0,
	    bool only_exists = true, bool early_stopping = false) const
	{
		return beginQueryNearest(rootNode(), std::forward<Geometry>(geometry),
		                         std::forward<Predicate>(predicate), epsilon, only_exists,
		                         early_stopping);
	}

	template <class Geometry, class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] const_query_nearest_iterator beginQueryNearest(
	    Node node, Geometry&& geometry, Predicate&& predicate, double epsilon = 0.0,
	    bool only_exists = true, bool early_stopping = false) const
	{
		if (only_exists) {
			if (early_stopping) {
				return const_query_nearest_iterator(
				    new OctreeIteratorNearest<true, true, Derived, Geometry, Predicate>(
				        &derived(), toNodeBV(node), std::forward<Geometry>(geometry),
				        std::forward<Predicate>(predicate), epsilon));
			} else {
				return const_query_nearest_iterator(
				    new OctreeIteratorNearest<true, false, Derived, Geometry, Predicate>(
				        &derived(), toNodeBV(node), std::forward<Geometry>(geometry),
				        std::forward<Predicate>(predicate), epsilon));
			}
		} else {
			if (early_stopping) {
				return const_query_nearest_iterator(
				    new OctreeIteratorNearest<false, true, Derived, Geometry, Predicate>(
				        &derived(), toNodeBV(node), std::forward<Geometry>(geometry),
				        std::forward<Predicate>(predicate), epsilon));
			} else {
				return const_query_nearest_iterator(
				    new OctreeIteratorNearest<false, false, Derived, Geometry, Predicate>(
				        &derived(), toNodeBV(node), std::forward<Geometry>(geometry),
				        std::forward<Predicate>(predicate), epsilon));
			}
		}
	}

	template <class Geometry, class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] const_query_nearest_iterator beginQueryNearest(
	    Code code, Geometry&& geometry, Predicate&& predicate, double epsilon = 0.0,
	    bool only_exists = true, bool early_stopping = false) const
	{
		return beginQueryNearest(operator()(code), std::forward<Geometry>(geometry),
		                         std::forward<Predicate>(predicate), epsilon, only_exists,
		                         early_stopping);
	}

	template <class Geometry, class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] const_query_nearest_iterator beginQueryNearest(
	    Key key, Geometry&& geometry, Predicate&& predicate, double epsilon = 0.0,
	    bool only_exists = true, bool early_stopping = false) const
	{
		return beginQueryNearest(toCode(key), std::forward<Geometry>(geometry),
		                         std::forward<Predicate>(predicate), epsilon, only_exists,
		                         early_stopping);
	}

	template <class Geometry, class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] const_query_nearest_iterator beginQueryNearest(
	    Point coord, depth_t depth, Geometry&& geometry, Predicate&& predicate,
	    double epsilon = 0.0, bool only_exists = true, bool early_stopping = false) const
	{
		return beginQueryNearest(toCode(coord, depth), std::forward<Geometry>(geometry),
		                         std::forward<Predicate>(predicate), epsilon, only_exists,
		                         early_stopping);
	}

	template <class Geometry, class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] const_query_nearest_iterator beginQueryNearest(
	    coord_t x, coord_t y, coord_t z, depth_t depth, Geometry&& geometry,
	    Predicate&& predicate, double epsilon = 0.0, bool only_exists = true,
	    bool early_stopping = false) const
	{
		return beginQueryNearest(toCode(x, y, z, depth), std::forward<Geometry>(geometry),
		                         std::forward<Predicate>(predicate), epsilon, only_exists,
		                         early_stopping);
	}

	[[nodiscard]] const_query_nearest_iterator endQueryNearest() const
	{
		return const_query_nearest_iterator(new OctreeIteratorNearest<true, true, Derived>());
	}

	//
	// "Normal" iterator
	//

	[[nodiscard]] const_iterator begin(bool only_exists    = true,
	                                   bool early_stopping = false) const
	{
		return begin(rootNode(), only_exists, early_stopping);
	}

	[[nodiscard]] const_iterator begin(Node node, bool only_exists = true,
	                                   bool early_stopping = false) const
	{
		return beginQuery(node, pred::Leaf{}, only_exists, early_stopping);
	}

	[[nodiscard]] const_iterator begin(Code code, bool only_exists = true,
	                                   bool early_stopping = false) const
	{
		return begin(operator()(code), only_exists, early_stopping);
	}

	[[nodiscard]] const_iterator begin(Key key, bool only_exists = true,
	                                   bool early_stopping = false) const
	{
		return begin(toCode(key), only_exists, early_stopping);
	}

	[[nodiscard]] const_iterator begin(Point coord, depth_t depth, bool only_exists = true,
	                                   bool early_stopping = false) const
	{
		return begin(toCode(coord, depth), only_exists, early_stopping);
	}

	[[nodiscard]] const_iterator begin(coord_t x, coord_t y, coord_t z, depth_t depth,
	                                   bool only_exists    = true,
	                                   bool early_stopping = false) const
	{
		return begin(toCode(x, y, z, depth), only_exists, early_stopping);
	}

	[[nodiscard]] const_iterator end() const { return endQuery(); }

	[[nodiscard]] const_bounding_volume_iterator beginBV(bool only_exists    = true,
	                                                     bool early_stopping = false) const
	{
		return beginBV(rootNode(), only_exists, early_stopping);
	}

	[[nodiscard]] const_iterator beginBV(Node node, bool only_exists = true,
	                                     bool early_stopping = false) const
	{
		return beginQueryBV(node, pred::Leaf{}, only_exists, early_stopping);
	}

	[[nodiscard]] const_iterator beginBV(Code code, bool only_exists = true,
	                                     bool early_stopping = false) const
	{
		return beginBV(operator()(code), only_exists, early_stopping);
	}

	[[nodiscard]] const_iterator beginBV(Key key, bool only_exists = true,
	                                     bool early_stopping = false) const
	{
		return beginBV(toCode(key), only_exists, early_stopping);
	}

	[[nodiscard]] const_iterator beginBV(Point coord, depth_t depth,
	                                     bool only_exists    = true,
	                                     bool early_stopping = false) const
	{
		return beginBV(toCode(coord, depth), only_exists, early_stopping);
	}

	[[nodiscard]] const_iterator beginBV(coord_t x, coord_t y, coord_t z, depth_t depth,
	                                     bool only_exists    = true,
	                                     bool early_stopping = false) const
	{
		return beginBV(toCode(x, y, z, depth), only_exists, early_stopping);
	}

	[[nodiscard]] const_bounding_volume_iterator endBV() const { return endQueryBV(); }

	/**************************************************************************************
	|                                                                                     |
	|                                         I/O                                         |
	|                                                                                     |
	**************************************************************************************/

	//
	// Read
	//

	void read(std::filesystem::path const& path, mt_t map_types = MapType::ALL,
	          bool propagate = true)
	{
		std::ifstream file;
		file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
		file.imbue(std::locale());
		file.open(path, std::ios_base::in | std::ios_base::binary);

		read(file, map_types, propagate);
	}

	void read(std::istream& in, mt_t map_types = MapType::ALL, bool propagate = true)
	{
		readData(in, readHeader(in), map_types, propagate);
	}

	void read(ReadBuffer& in, mt_t map_types = MapType::ALL, bool propagate = true)
	{
		auto header = readHeader(in);
		readData(in, header, map_types, propagate);
	}

	void readData(std::istream& in, FileHeader const& header, mt_t map_types = MapType::ALL,
	              bool propagate = true)
	{
		if (size() != header.leaf_size || depthLevels() != header.depth_levels) {
			clear(header.leaf_size, header.depth_levels);
		}

		auto nodes = readNodes(in);
		derived().readNodes(in, nodes, header.compressed, map_types);

		if (propagate) {
			propagateModified();
		}
	}

	void readData(ReadBuffer& in, FileHeader const& header, mt_t map_types = MapType::ALL,
	              bool propagate = true)
	{
		if (size() != header.leaf_size || depthLevels() != header.depth_levels) {
			clear(header.leaf_size, header.depth_levels);
		}

		auto nodes = readNodes(in);
		derived().readNodes(in, nodes, header.compressed, map_types);

		if (propagate) {
			propagateModified();
		}
	}

	//
	// Write
	//

	void write(std::filesystem::path const& path, depth_t min_depth = 0,
	           bool compress = false, mt_t map_types = MapType::ALL,
	           int compression_acceleration_level = 1, int compression_level = 0) const
	{
		write(path, pred::Exists(), min_depth, compress, map_types,
		      compression_acceleration_level, compression_level);
	}

	void write(std::ostream& out, depth_t min_depth = 0, bool compress = false,
	           mt_t map_types = MapType::ALL, int compression_acceleration_level = 1,
	           int compression_level = 0) const
	{
		write(out, pred::Exists(), min_depth, compress, map_types,
		      compression_acceleration_level, compression_level);
	}

	void write(WriteBuffer& out, depth_t min_depth = 0, bool compress = false,
	           mt_t map_types = MapType::ALL, int compression_acceleration_level = 1,
	           int compression_level = 0) const
	{
		write(out, pred::Exists(), min_depth, compress, map_types,
		      compression_acceleration_level, compression_level);
	}

	Buffer write(depth_t min_depth = 0, bool compress = false,
	             mt_t map_types = MapType::ALL, int compression_acceleration_level = 1,
	             int compression_level = 0) const
	{
		return write(pred::Exists(), min_depth, compress, map_types,
		             compression_acceleration_level, compression_level);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	void write(std::filesystem::path const& path, Predicate&& predicate,
	           depth_t min_depth = 0, bool compress = false, mt_t map_types = MapType::ALL,
	           int compression_acceleration_level = 1, int compression_level = 0) const
	{
		std::ofstream file;
		file.exceptions(std::ofstream::failbit | std::ofstream::badbit);
		file.imbue(std::locale());
		file.open(path, std::ios_base::out | std::ios_base::binary);

		write(file, std::forward<Predicate>(predicate), min_depth, compress, map_types,
		      compression_acceleration_level, compression_level);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	void write(std::ostream& out, Predicate&& predicate, depth_t min_depth = 0,
	           bool compress = false, mt_t map_types = MapType::ALL,
	           int compression_acceleration_level = 1, int compression_level = 0) const
	{
		auto [tree_structure, nodes] = data(pred::Leaf() && pred::DepthMin(min_depth) &&
		                                    std::forward<Predicate>(predicate));
		write(out, tree_structure, nodes, compress, map_types, compression_acceleration_level,
		      compression_level);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	void write(WriteBuffer& out, Predicate&& predicate, depth_t min_depth = 0,
	           bool compress = false, mt_t map_types = MapType::ALL,
	           int compression_acceleration_level = 1, int compression_level = 0) const
	{
		auto [tree_structure, nodes] = data(pred::Leaf() && pred::DepthMin(min_depth) &&
		                                    std::forward<Predicate>(predicate));
		write(out, tree_structure, nodes, compress, map_types, compression_acceleration_level,
		      compression_level);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	Buffer write(Predicate&& predicate, depth_t min_depth = 0, bool compress = false,
	             mt_t map_types = MapType::ALL, int compression_acceleration_level = 1,
	             int compression_level = 0) const
	{
		Buffer buffer;
		write(buffer, predicate, min_depth, compress, map_types,
		      compression_acceleration_level, compression_level);
		return buffer;
	}

	void writeModified(std::filesystem::path const& filename, bool reset_modified = true,
	                   bool propagate = true, bool compress = false,
	                   mt_t map_types                     = MapType::ALL,
	                   int compression_acceleration_level = 1, int compression_level = 0)
	{
		std::ofstream file;
		file.exceptions(std::ofstream::failbit | std::ofstream::badbit);
		file.imbue(std::locale());
		file.open(filename, std::ios_base::out | std::ios_base::binary);

		writeModified(file, reset_modified, propagate, compress, map_types,
		              compression_acceleration_level, compression_level);
	}

	void writeModified(std::ostream& out, bool reset_modified = true, bool propagate = true,
	                   bool compress = false, mt_t map_types = MapType::ALL,
	                   int compression_acceleration_level = 1, int compression_level = 0)
	{
		auto [tree, indices] = modifiedData(reset_modified, propagate);
		write(out, tree, indices, compress, map_types, compression_acceleration_level,
		      compression_level);
	}

	void writeModified(WriteBuffer& out, bool reset_modified = true, bool propagate = true,
	                   bool compress = false, mt_t map_types = MapType::ALL,
	                   int compression_acceleration_level = 1, int compression_level = 0)
	{
		auto [tree, indices] = modifiedData(reset_modified, propagate);
		write(out, tree, indices, compress, map_types, compression_acceleration_level,
		      compression_level);
	}

	Buffer writeModified(bool reset_modified = true, bool propagate = true,
	                     bool compress = false, mt_t map_types = MapType::ALL,
	                     int compression_acceleration_level = 1, int compression_level = 0)
	{
		Buffer buffer;
		writeModified(buffer, reset_modified, propagate, compress, map_types,
		              compression_acceleration_level, compression_level);
		return buffer;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                    Save dot file                                    |
	|                                                                                     |
	**************************************************************************************/

	void saveDotFile(Index node, std::filesystem::path const& path, depth_t min_depth = 0,
	                 mt_t map_types = MapType::ALL) const
	{
		saveDotFile(node, path, pred::Exists(), min_depth, map_types);
	}

	void saveDotFile(std::filesystem::path const& path, depth_t min_depth = 0,
	                 mt_t map_types = MapType::ALL) const
	{
		saveDotFile(rootIndex(), path, min_depth, map_types);
	}

	void saveDotFile(Node node, std::filesystem::path const& path, depth_t min_depth = 0,
	                 mt_t map_types = MapType::ALL) const
	{
		saveDotFile(index(node), path, min_depth, map_types);
	}

	void saveDotFile(Code code, std::filesystem::path const& path, depth_t min_depth = 0,
	                 mt_t map_types = MapType::ALL) const
	{
		saveDotFile(index(code), path, min_depth, map_types);
	}

	void saveDotFile(Key key, std::filesystem::path const& path, depth_t min_depth = 0,
	                 mt_t map_types = MapType::ALL) const
	{
		saveDotFile(index(key), path, min_depth, map_types);
	}

	void saveDotFile(Point coord, std::filesystem::path const& path, depth_t min_depth = 0,
	                 mt_t map_types = MapType::ALL, depth_t depth = 0) const
	{
		saveDotFile(index(coord, depth), path, min_depth, map_types);
	}

	void saveDotFile(coord_t x, coord_t y, coord_t z, std::filesystem::path const& path,
	                 depth_t min_depth = 0, mt_t map_types = MapType::ALL,
	                 depth_t depth = 0) const
	{
		saveDotFile(index(x, y, z, depth), path, min_depth, map_types);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	void saveDotFile(Index node, std::filesystem::path const& path, Predicate&& predicate,
	                 depth_t min_depth = 0, mt_t map_types = MapType::ALL) const
	{
		std::ofstream file;
		file.exceptions(std::ofstream::failbit | std::ofstream::badbit);
		file.imbue(std::locale());
		file.open(path, std::ios_base::out | std::ios_base::binary);

		saveDotFile(node, file, std::forward<Predicate>(predicate), min_depth, map_types);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	void saveDotFile(std::filesystem::path const& path, Predicate&& predicate,
	                 depth_t min_depth = 0, mt_t map_types = MapType::ALL) const
	{
		saveDotFile(rootIndex(), path, std::forward<Predicate>(predicate), min_depth,
		            map_types);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	void saveDotFile(Node node, std::filesystem::path const& path, Predicate&& predicate,
	                 depth_t min_depth = 0, mt_t map_types = MapType::ALL) const
	{
		saveDotFile(index(node), path, std::forward<Predicate>(predicate), min_depth,
		            map_types);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	void saveDotFile(Code code, std::filesystem::path const& path, Predicate&& predicate,
	                 depth_t min_depth = 0, mt_t map_types = MapType::ALL) const
	{
		saveDotFile(index(code), path, std::forward<Predicate>(predicate), min_depth,
		            map_types);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	void saveDotFile(Key key, std::filesystem::path const& path, Predicate&& predicate,
	                 depth_t min_depth = 0, mt_t map_types = MapType::ALL) const
	{
		saveDotFile(index(key), path, std::forward<Predicate>(predicate), min_depth,
		            map_types);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	void saveDotFile(Point coord, std::filesystem::path const& path, Predicate&& predicate,
	                 depth_t min_depth = 0, mt_t map_types = MapType::ALL,
	                 depth_t depth = 0) const
	{
		saveDotFile(index(coord, depth), path, std::forward<Predicate>(predicate), min_depth,
		            map_types);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	void saveDotFile(coord_t x, coord_t y, coord_t z, std::filesystem::path const& path,
	                 Predicate&& predicate, depth_t min_depth = 0,
	                 mt_t map_types = MapType::ALL, depth_t depth = 0) const
	{
		saveDotFile(index(x, y, z, depth), path, std::forward<Predicate>(predicate),
		            min_depth, map_types);
	}

	void saveDotFile(Index node, std::ostream& out, depth_t min_depth = 0,
	                 mt_t map_types = MapType::ALL) const
	{
		saveDotFile(node, out, pred::Exists(), min_depth, map_types);
	}

	void saveDotFile(std::ostream& out, depth_t min_depth = 0,
	                 mt_t map_types = MapType::ALL) const
	{
		saveDotFile(rootIndex(), out, min_depth, map_types);
	}

	void saveDotFile(Node node, std::ostream& out, depth_t min_depth = 0,
	                 mt_t map_types = MapType::ALL) const
	{
		saveDotFile(index(node), out, min_depth, map_types);
	}

	void saveDotFile(Code code, std::ostream& out, depth_t min_depth = 0,
	                 mt_t map_types = MapType::ALL) const
	{
		saveDotFile(index(code), out, min_depth, map_types);
	}

	void saveDotFile(Key key, std::ostream& out, depth_t min_depth = 0,
	                 mt_t map_types = MapType::ALL) const
	{
		saveDotFile(index(key), out, min_depth, map_types);
	}

	void saveDotFile(Point coord, std::ostream& out, depth_t min_depth = 0,
	                 mt_t map_types = MapType::ALL, depth_t depth = 0) const
	{
		saveDotFile(index(coord, depth), out, min_depth, map_types);
	}

	void saveDotFile(coord_t x, coord_t y, coord_t z, std::ostream& out,
	                 depth_t min_depth = 0, mt_t map_types = MapType::ALL,
	                 depth_t depth = 0) const
	{
		saveDotFile(index(x, y, z, depth), out, min_depth, map_types);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	void saveDotFile(Index node, std::ostream& out, Predicate&& predicate,
	                 depth_t min_depth = 0, mt_t map_types = MapType::ALL) const
	{
		auto d = depth(node);

		if (d < min_depth) {
			return;
		}

		out << std::boolalpha << "graph UFOMap {\n";
		out << "fontname=\"Helvetica,Arial,sans-serif\"\n";
		out << "node [fontname=\"Helvetica,Arial,sans-serif\" shape=box color=lightblue2 "
		       "style=filled]\n";
		out << "edge [fontname=\"Helvetica,Arial,sans-serif\"]\n";

		std::string id = std::to_string((node.pos << 3) | node.offset);
		auto        m  = isModified(node);
		out << id << " [";
		if (isLeaf(node)) {
			if (m) {
				out << "label=<Modified: <font color='green'><b>" << m << "</b></font>";
			} else {
				out << "label=<Modified: <font color='red'>" << m << "</font>";
			}
			derived().dotFileInfo(out, node, map_types);
			out << ">]\n}";
			return;
		} else {
			out << "shape=octagon ";
			if (m) {
				out << "label=<Modified: <font color='green'><b>" << m << "</b></font>";
			} else {
				out << "label=<Modified: <font color='red'><b>" << m << "</b></font>";
			}
			derived().dotFileInfo(out, node, map_types);
			out << "> color=darkorange1]\n";
		}

		if (d - 1 >= min_depth) {
			saveDotFileRecurs(node, id, d - 1, out, predicate, min_depth, map_types);
		}

		out << "}";
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	void saveDotFile(std::ostream& out, Predicate&& predicate, depth_t min_depth = 0,
	                 mt_t map_types = MapType::ALL) const
	{
		saveDotFile(rootIndex(), out, std::forward<Predicate>(predicate), min_depth,
		            map_types);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	void saveDotFile(Node node, std::ostream& out, Predicate&& predicate,
	                 depth_t min_depth = 0, mt_t map_types = MapType::ALL) const
	{
		saveDotFile(index(node), out, std::forward<Predicate>(predicate), min_depth,
		            map_types);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	void saveDotFile(Code code, std::ostream& out, Predicate&& predicate,
	                 depth_t min_depth = 0, mt_t map_types = MapType::ALL) const
	{
		saveDotFile(index(code), out, std::forward<Predicate>(predicate), min_depth,
		            map_types);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	void saveDotFile(Key key, std::ostream& out, Predicate&& predicate,
	                 depth_t min_depth = 0, mt_t map_types = MapType::ALL) const
	{
		saveDotFile(index(key), out, std::forward<Predicate>(predicate), min_depth,
		            map_types);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	void saveDotFile(Point coord, std::ostream& out, Predicate&& predicate,
	                 depth_t min_depth = 0, mt_t map_types = MapType::ALL,
	                 depth_t depth = 0) const
	{
		saveDotFile(index(coord, depth), out, std::forward<Predicate>(predicate), min_depth,
		            map_types);
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	void saveDotFile(coord_t x, coord_t y, coord_t z, std::ostream& out,
	                 Predicate&& predicate, depth_t min_depth = 0,
	                 mt_t map_types = MapType::ALL, depth_t depth = 0) const
	{
		saveDotFile(index(x, y, z, depth), out, std::forward<Predicate>(predicate), min_depth,
		            map_types);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Statistics                                      |
	|                                                                                     |
	**************************************************************************************/

	/*!
	 * @return Number of nodes in the octree.
	 */
	[[nodiscard]] std::size_t numNodes() const
	{
		return 8 * (children_.size() - free_children_.size());
	}

	/*!
	 * @brief This is lower bound memory usage of a node.
	 *
	 * @note Additional data accessed by pointers inside a node are not counted.
	 *
	 * @return Memory usage of a single node.
	 */
	[[nodiscard]] static constexpr std::size_t memoryNode() noexcept(
	    noexcept(Derived::memoryNodeBlock()))
	{
		return Derived::memoryNodeBlock() / 8;
	}

	/*!
	 * @brief Lower bound memory usage for all nodes.
	 *
	 * @note Does not account for pointed to data inside nodes.
	 *
	 * @return Memory usage of the octree.
	 */
	[[nodiscard]] std::size_t memoryUsage() const { return numNodes() * memoryNode(); }

	/*!
	 * @return Number of allocated nodes.
	 */
	[[nodiscard]] std::size_t numNodesAllocated() const { return 8 * children_.size(); }

	/*!
	 * @brief Lower bound memory usage for all allocated nodes.
	 *
	 * @note Does not account for pointed to data inside nodes.
	 *
	 * @return Memory usage of the allocated octree.
	 */
	[[nodiscard]] std::size_t memoryUsageAllocated() const
	{
		return numNodesAllocated() * memoryNode();
	}

 protected:
	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comments

	Octree(node_size_t leaf_node_size = 0.1, depth_t depth_levels = 17)

	{
		setNodeSizeAndDepthLevels(leaf_node_size, depth_levels);

		children_.push_back(
		    {NULL_POS, NULL_POS, NULL_POS, NULL_POS, NULL_POS, NULL_POS, NULL_POS, NULL_POS});
		parent_code_.emplace_back(rootCode().parent());
		modified_.emplace_back(0);
	}

	Octree(Octree const& other)
	    : depth_levels_(other.depth_levels_)
	    , half_max_value_(other.half_max_value_)
	    , children_(other.children_)
	    , free_children_(other.free_children_)
	    , parent_code_(other.parent_code_)
	    , modified_(other.modified_)
	    , node_size_(other.node_size_)
	    , node_size_factor_(other.node_size_factor_)
	{
	}

	Octree(Octree&& other)
	    : depth_levels_(std::move(other.depth_levels_))
	    , half_max_value_(std::move(other.half_max_value_))
	    , children_(std::move(other.children_))
	    , free_children_(std::move(other.free_children_))
	    , parent_code_(std::move(other.parent_code_))
	    , modified_(std::move(other.modified_))
	    , node_size_(std::move(other.node_size_))
	    , node_size_factor_(std::move(other.node_size_factor_))
	{
	}

	template <class Derived2>
	Octree(Octree<Derived2> const& other)
	    : depth_levels_(other.depth_levels_)
	    , half_max_value_(other.half_max_value_)
	    , children_(other.children_)
	    , free_children_(other.free_children_)
	    , parent_code_(other.parent_code_)
	    , modified_(other.modified_)
	    , node_size_(other.node_size_)
	    , node_size_factor_(other.node_size_factor_)
	{
	}

	template <class Derived2>
	Octree(Octree<Derived2>&& other)
	    : depth_levels_(std::move(other.depth_levels_))
	    , half_max_value_(std::move(other.half_max_value_))
	    , children_(std::move(other.children_))
	    , free_children_(std::move(other.free_children_))
	    , parent_code_(std::move(other.parent_code_))
	    , modified_(std::move(other.modified_))
	    , node_size_(std::move(other.node_size_))
	    , node_size_factor_(std::move(other.node_size_factor_))
	{
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Destructor                                      |
	|                                                                                     |
	**************************************************************************************/

	~Octree() {}

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comments

	Octree& operator=(Octree const& rhs)
	{
		depth_levels_     = rhs.depth_levels_;
		half_max_value_   = rhs.half_max_value_;
		children_         = rhs.children_;
		free_children_    = rhs.free_children_;
		parent_code_      = rhs.parent_code_;
		modified_         = rhs.modified_;
		node_size_        = rhs.node_size_;
		node_size_factor_ = rhs.node_size_factor_;
		return *this;
	}

	Octree& operator=(Octree&& rhs)
	{
		depth_levels_     = std::move(rhs.depth_levels_);
		half_max_value_   = std::move(rhs.half_max_value_);
		children_         = std::move(rhs.children_);
		free_children_    = std::move(rhs.free_children_);
		parent_code_      = std::move(rhs.parent_code_);
		modified_         = std::move(rhs.modified_);
		node_size_        = std::move(rhs.node_size_);
		node_size_factor_ = std::move(rhs.node_size_factor_);
		return *this;
	}

	template <class Derived2>
	Octree& operator=(Octree<Derived2> const& rhs)
	{
		depth_levels_     = rhs.depth_levels_;
		half_max_value_   = rhs.half_max_value_;
		children_         = rhs.children_;
		free_children_    = rhs.free_children_;
		parent_code_      = rhs.parent_code_;
		modified_         = rhs.modified_;
		node_size_        = rhs.node_size_;
		node_size_factor_ = rhs.node_size_factor_;
		return *this;
	}

	template <class Derived2>
	Octree& operator=(Octree<Derived2>&& rhs)
	{
		depth_levels_     = std::move(rhs.depth_levels_);
		half_max_value_   = std::move(rhs.half_max_value_);
		children_         = std::move(rhs.children_);
		free_children_    = std::move(rhs.free_children_);
		parent_code_      = std::move(rhs.parent_code_);
		modified_         = std::move(rhs.modified_);
		node_size_        = std::move(rhs.node_size_);
		node_size_factor_ = std::move(rhs.node_size_factor_);
		return *this;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                         Swap                                        |
	|                                                                                     |
	**************************************************************************************/

	void swap(Octree& other)
	{
		std::swap(depth_levels_, other.depth_levels_);
		std::swap(half_max_value_, other.half_max_value_);
		std::swap(children_, other.children_);
		std::swap(free_children_, other.free_children_);
		std::swap(parent_code_, other.parent_code_);
		std::swap(modified_, other.modified_);
		std::swap(node_size_, other.node_size_);
		std::swap(node_size_factor_, other.node_size_factor_);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Derived                                       |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comments

	[[nodiscard]] constexpr Derived& derived() { return *static_cast<Derived*>(this); }

	[[nodiscard]] constexpr Derived const& derived() const
	{
		return *static_cast<Derived const*>(this);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Octree                                        |
	|                                                                                     |
	**************************************************************************************/

	void setNodeSizeAndDepthLevels(node_size_t const leaf_node_size,
	                               depth_t const     depth_levels)
	{
		if (minDepthLevels() > depth_levels || maxDepthLevels() < depth_levels) {
			throw std::invalid_argument("depth_levels have to be in range [" +
			                            std::to_string(+minDepthLevels()) + ".." +
			                            std::to_string(+maxDepthLevels()) + "], '" +
			                            std::to_string(+depth_levels) + "' was supplied.");
		}

		depth_levels_   = depth_levels;
		half_max_value_ = key_t(1) << (depth_levels - 2);  // TODO: Correct?

		// For increased precision
		for (int i{0}; auto& ns : node_size_) {
			ns = std::ldexp(leaf_node_size, i++);
		}

		std::ranges::transform(node_size_, std::begin(node_size_factor_),
		                       [](auto const n) { return 1.0 / n; });
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Root                                         |
	|                                                                                     |
	**************************************************************************************/

	/*!
	 * @brief Initilize the root node.
	 */
	void initRoot()
	{
		children_[0].fill(NULL_POS);
		modified_[0].reset();
		derived().initRoot();
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        TODO                                         |
	|                                                                                     |
	**************************************************************************************/

	void fill(Index node, pos_t children)
	{
		parent_code_[children] = parent_code_[node.pos].child(node.offset);
		if (isModified(node)) {
			modified_[children].set();
		} else {
			modified_[children].reset();
		}
		derived().fill(node, children);
	}

	void resize(std::size_t new_size)
	{
		children_.resize(new_size);
		free_children_ = {};
		parent_code_.resize(new_size);
		modified_.resize(new_size);
		derived().resize(new_size);
	}

	void clearImpl(pos_t nodes)
	{
		// NOTE: Important that derived is cleared first in case they use parent code
		derived().clearImpl(nodes);
		parent_code_[nodes] = INVALID_CODE;
		// TODO: Implement
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Apply                                        |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comments

	void apply(Index node, std::invocable<Index> auto f, std::invocable<pos_t> auto f2)
	{
		if (isLeaf(node)) {
			f(node);
		} else if (allLeaf(children(node))) {
			f2(children(node));
		} else {
			// Recursive
			// auto pos = children(node);
			// for (offset_t i{}; 8 != i; ++i) {
			// 	apply(Index(pos, i), f, f2);
			// }

			// Iterative
			std::array<Index, maxDepthLevels()> nodes;
			nodes[1] = child(node, 0);
			for (std::size_t i{1}; 0 != i;) {
				node = nodes[i];
				i -= 7 < ++nodes[i].offset;
				if (isLeaf(node)) {
					f(node);
				} else if (allLeaf(children(node))) {
					f2(children(node));
				} else {
					nodes[++i] = child(node, 0);
				}
			}
		}
	}

	void apply(Index node, std::invocable<Index> auto f, std::invocable<pos_t> auto f2,
	           bool propagate)
	{
		setModified(node);

		if (isLeaf(node)) {
			f(node);
		} else {
			applyRecurs(node, f, f2);
		}

		if (propagate) {
			propagateModified();
		}
	}

	void applyRecurs(Index node, std::invocable<Index> auto f,
	                 std::invocable<pos_t> auto f2)
	{
		setAllModified(children(node));
		if (allLeaf(children(node))) {
			f2(children(node));
		} else {
			// FIXME: Compare iterative and recursive version

			// // Recursive
			// node.pos = children(node);
			// for (offset_t i{}; 8 != i; ++i) {
			// 	node.offset = i;
			// 	if (isLeaf(node)) {
			// 		f(node);
			// 	} else {
			// 		applyRecurs(node, f, f2);
			// 	}
			// }

			// Iterative
			std::array<Index, maxDepthLevels()> nodes;
			nodes[1] = child(node, 0);
			for (std::size_t i{1}; 0 != i;) {
				node = nodes[i];
				i -= 7 < ++nodes[i].offset;
				if (isLeaf(node)) {
					f(node);
				} else {
					setAllModified(children(node));
					if (allLeaf(children(node))) {
						f2(children(node));
					} else {
						nodes[++i] = child(node, 0);
					}
				}
			}
		}
	}

	Node apply(Node node, std::invocable<Index> auto f, std::invocable<pos_t> auto f2,
	           bool propagate)
	{
		Index index = createIndex(node);
		apply(index, f, f2, propagate);
		return {node.code(), index};
	}

	Node apply(Code code, std::invocable<Index> auto f, std::invocable<pos_t> auto f2,
	           bool propagate)
	{
		Index index = createIndex(code);
		apply(index, f, f2, propagate);
		return {code, index};
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Recurs                                       |
	|                                                                                     |
	**************************************************************************************/

	void recurs(Index node, std::invocable<Index> auto f,
	            std::invocable<pos_t> auto f2) const
	{
		if (isLeaf(node)) {
			f(node);
		} else if (allLeaf(children(node))) {
			f2(children(node));
		} else {
			std::array<Index, maxDepthLevels()> nodes;
			nodes[1] = child(node, 0);
			for (std::size_t i{1}; 0 != i;) {
				node = nodes[i];
				i -= 7 < ++nodes[i].offset;
				if (isLeaf(node)) {
					f(node);
				} else if (allLeaf(children(node))) {
					f2(children(node));
				} else {
					nodes[++i] = child(node, 0);
				}
			}
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Conversion                                      |
	|                                                                                     |
	**************************************************************************************/

	//
	// Key
	//

	/*!
	 * @brief Convert a coordinate at a specific depth to a key.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The corresponding key.
	 */
	[[nodiscard]] constexpr key_t toKey(coord_t coord, depth_t depth = 0) const
	{
		assert(rootDepth() >= depth);
		assert(-static_cast<coord_t>(size(rootDepth() - 1)) <= coord &&
		       static_cast<coord_t>(size(rootDepth() - 1)) > coord);
		// FIXME: Make it possible to make a cloud of coordinates
		return static_cast<key_t>(
		           std::floor(static_cast<node_size_t>(coord) / node_size_[0])) +
		       half_max_value_;
		// key_t val = static_cast<key_t>(
		//     std::floor(static_cast<coord_t>(node_size_factor_[0]) * coord));
		// return val + half_max_value_;
	}

	/*!
	 * @brief Convert a coordinate at a specific depth to a key with bounds check.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The corresponding key.
	 */
	[[nodiscard]] constexpr std::optional<key_t> toKeyChecked(coord_t coord,
	                                                          depth_t depth = 0) const
	{
		auto min = -size(rootDepth() - 1);
		auto max = -min;
		return min <= coord && max > coord && rootDepth() >= depth
		           ? std::optional<key_t>(toKey(coord, depth))
		           : std::nullopt;
	}

	//
	// Coordinate
	//

	/*!
	 * @brief Convert a key to a coordinate at a specific depth.
	 *
	 * @param key The key.
	 * @param depth The depth of the coordinate.
	 * @return The corresponding coordinate.
	 */
	[[nodiscard]] constexpr coord_t toCoord(key_t key, depth_t depth = 0) const
	{
		assert(rootDepth() >= depth);
		assert(2 * half_max_value_ >= key);
		constexpr auto sub64 = std::minus<std::int_fast64_t>{};
		return rootDepth() == depth
		           ? coord_t{}
		           : (std::floor(static_cast<coord_t>(
		                             sub64(static_cast<std::int_fast64_t>(key),
		                                   static_cast<std::int_fast64_t>(half_max_value_))) /
		                         static_cast<coord_t>(1U << depth)) +
		              coord_t(0.5)) *
		                 static_cast<coord_t>(size(depth));
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Leaf                                         |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] bool isPureLeaves(pos_t pos) const
	{
		return 1 == parent_code_[pos].depth();
	}

	/**************************************************************************************
	|                                                                                     |
	|                                    Access nodes                                     |
	|                                                                                     |
	**************************************************************************************/

	//
	// Correct
	//

	// TODO: Add comment
	[[nodiscard]] constexpr bool isCorrect(Node node) const
	{
		assert(parent_code_.size() > node.pos());
		return parent_code_[node.pos()] == node.code().parent();
	}

	//
	// Valid
	//

	// TODO: Add comment
	[[nodiscard]] constexpr bool isValid(Node node) const
	{
		assert(parent_code_.size() > node.pos());
		depth_t depth = parent_code_[node.pos()].depth();
		return depth > node.depth() &&
		       Code::equalAtDepth(parent_code_[node.pos()], node.code(), depth);
	}

	//
	// Get node
	//

	// TODO: Add comment
	[[nodiscard]] constexpr Index index(Code code, Index node, depth_t depth) const
	{
		depth_t min_depth = code.depth();
		while (min_depth < depth && isParent(node)) {
			node = child(node, code.offset(--depth));
		}
		return node;
	}

	//
	// Get children
	//

	// TODO: Add comment
	[[nodiscard]] constexpr pos_t children(Index node) const
	{
		return children_[node.pos][node.offset];
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Center                                        |
	|                                                                                     |
	**************************************************************************************/

	//
	// Child center
	//

	// TODO: Add comment
	[[nodiscard]] static constexpr Point childCenter(Point    parent_center,
	                                                 coord_t  child_half_size,
	                                                 offset_t child)
	{
		assert(8 > child);
		parent_center[0] += child & offset_t(1) ? child_half_size : -child_half_size;
		parent_center[1] += child & offset_t(2) ? child_half_size : -child_half_size;
		parent_center[2] += child & offset_t(4) ? child_half_size : -child_half_size;
		return parent_center;
	}

	//
	// Sibling center
	//

	// TODO: Add comment
	[[nodiscard]] static constexpr Point siblingCenter(Point center, coord_t half_size,
	                                                   offset_t index,
	                                                   offset_t sibling_index)
	{
		assert(8 > sibling_index);
		offset_t const temp = index ^ sibling_index;
		coord_t const  size = 2 * half_size;
		center[0] +=
		    temp & offset_t(1) ? (sibling_index & offset_t(1) ? size : -size) : coord_t{};
		center[1] +=
		    temp & offset_t(2) ? (sibling_index & offset_t(2) ? size : -size) : coord_t{};
		center[2] +=
		    temp & offset_t(4) ? (sibling_index & offset_t(4) ? size : -size) : coord_t{};
		return center;
	}

	//
	// Parent center
	//

	// TODO: Add comment
	[[nodiscard]] static constexpr Point parentCenter(Point    child_center,
	                                                  coord_t  child_half_size,
	                                                  offset_t child_index)
	{
		assert(8 > child_index);
		child_center[0] -= child_index & offset_t(1) ? child_half_size : -child_half_size;
		child_center[1] -= child_index & offset_t(2) ? child_half_size : -child_half_size;
		child_center[2] -= child_index & offset_t(4) ? child_half_size : -child_half_size;
		return child_center;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Leaf                                         |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comment
	[[nodiscard]] bool allLeaf(pos_t pos) const
	{
		assert(children_.size() > pos);
		return std::ranges::all_of(children_[pos], [](auto e) { return NULL_POS == e; });
	}

	// TODO: Add comment
	[[nodiscard]] bool anyLeaf(pos_t pos) const
	{
		assert(children_.size() > pos);
		return std::ranges::any_of(children_[pos], [](auto e) { return NULL_POS == e; });
	}

	// TODO: Add comment
	[[nodiscard]] bool noneLeaf(pos_t pos) const
	{
		assert(children_.size() > pos);
		return std::ranges::none_of(children_[pos], [](auto e) { return NULL_POS == e; });
	}

	// TODO: Add comment
	[[nodiscard]] bool allParent(pos_t pos) const { return noneLeaf(pos); }

	// TODO: Add comment
	[[nodiscard]] bool anyParent(pos_t pos) const { return !allLeaf(pos); }

	// TODO: Add comment
	[[nodiscard]] bool noneParent(pos_t pos) const { return allLeaf(pos); }

	/**************************************************************************************
	|                                                                                     |
	|                                      Modified                                       |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Implement

	// TODO: Add comment
	[[nodiscard]] bool allModified(pos_t pos) const
	{
		assert(modified_.size() > pos);
		return modified_[pos].all();
	}

	// TODO: Add comment
	[[nodiscard]] bool anyModified(pos_t pos) const
	{
		assert(modified_.size() > pos);
		return modified_[pos].any();
	}

	// TODO: Add comment
	[[nodiscard]] bool noneModified(pos_t pos) const
	{
		assert(modified_.size() > pos);
		return modified_[pos].none();
	}

	//
	// Set all modified
	//

	void setAllModified(pos_t nodes)
	{
		assert(modified_.size() > nodes);

		modified_[nodes].set();
	}

	//
	// Set modified
	//

	// TODO: Add comment
	void setModifiedRecurs(pos_t nodes)
	{
		assert(modified_.size() > nodes);

		modified_[nodes].set();

		// // FIXME: Make iterative
		// for (offset_t i = 0; 8 != i; ++i) {
		// 	Index node{nodes, i};
		// 	if (isParent(node)) {
		// 		setModifiedRecurs(children(node));
		// 	}
		// }

		std::array<Index, maxDepthLevels()> node;
		node[1].pos    = nodes;
		node[1].offset = 0;
		for (std::size_t i{1}; 0 != i;) {
			auto n = node[i];
			i -= 7 < ++node[i].offset;
			if (isParent(n)) {
				node[++i] = child(n, 0);
				modified_[node[i].pos].set();
			}
		}
	}

	//
	// Reset modified
	//

	// TODO: Add comment
	void resetModifiedRecurs(pos_t nodes)
	{
		assert(modified_.size() > nodes);

		// // FIXME: Make iterative
		// for (offset_t i = 0; 8 != i; ++i) {
		// 	Index node{nodes, i};
		// 	if (isModified(node) && isModified(node)) {
		// 		resetModifiedRecurs(children(node));
		// 	}
		// }

		// modified_[nodes].reset();

		std::array<Index, maxDepthLevels()> node;
		node[0].pos    = nodes;
		node[0].offset = 0;
		for (int depth{}; 0 <= depth;) {
			auto n = node[depth];
			depth -= 7 < ++node[depth].offset;
			if (isModified(n) && isParent(n)) {
				node[++depth] = child(n, 0);
			}
			if (7 == n.offset) {
				modified_[n.pos].reset();
			}
		}
	}

	//
	// Set parents modified
	//

	// TODO: Add comment
	void setParentsModified(Code code)
	{
		setParentsModified(rootIndex(), rootDepth(), code);
	}

	// TODO: Add comment
	void setParentsModified(Index node, depth_t d, Code c)
	{
		for (depth_t min_d = c.depth() + 1; min_d < d && isParent(node);) {
			modified_[node.pos].set(node.offset);
			node = child(node, c.offset(--d));
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Propagate                                      |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comment
	template <bool Prune>
	void updateNode(Index node)
	{
		derived().updateNode(node, children(node));
		if constexpr (Prune) {
			if (isPrunable(node)) {
				deleteChildren(node);
			}
		}
	}

	// TODO: Add comment
	template <bool ResetModified, bool Prune>
	void propagateModifiedRecurs(pos_t nodes)
	{
		// FIXME: Make iterative
		for (offset_t i = 0; 8 != i; ++i) {
			Index node{nodes, i};
			if (isModified(node) && isParent(node)) {
				propagateModifiedRecurs<ResetModified, Prune>(children(node));
				updateNode<Prune>(node);
			}
		}

		if constexpr (ResetModified) {
			modified_[nodes].reset();
		}

		// std::array<Index, maxDepthLevels()> node;
		// node[1].pos    = nodes;
		// node[1].offset = 0;
		// for (std::size_t i{1}; 0 != i;) {
		// 	auto n = node[i];
		// 	if (8 > n.offset) {
		// 		++node[i].offset;
		// 		if (isModified(n) && isParent(n)) {
		// 			node[++i] = child(n, 0);
		// 		}
		// 	} else {
		// 		for (offset_t i{}; 8 != i; ++i) {
		// 			Index x{n.pos, i};
		// 			if (isModified(x) && isParent(x)) {
		// 				updateNode<Prune>(x);
		// 			}
		// 		}
		// 		if constexpr (ResetModified) {
		// 			modified_[n.pos].reset();
		// 		}
		// 		--i;
		// 	}
		// }
	}

	/**************************************************************************************
	|                                                                                     |
	|                                 Create/delete nodes                                 |
	|                                                                                     |
	**************************************************************************************/

	//
	// Delete
	//

	// TODO: Add comment
	void deleteChildren(Index node)
	{
		if (isLeaf(node)) {
			return;
		}

		// FIXME: Check if iterative faster

		// for (offset_t i = 0; 8 != i; ++i) {
		// 	auto c = child(node, i);
		// 	deleteChildren(c);
		// 	clear(children(node));
		// }

		std::array<Index, maxDepthLevels()> nodes;
		nodes[1] = child(node, 0);
		clearImpl(nodes[1].pos);
		for (std::size_t i{1}; 0 != i;) {
			auto c = nodes[i];
			i -= 7 < ++nodes[i].offset;
			if (isParent(c)) {
				nodes[++i] = child(c, 0);
				clearImpl(nodes[i].pos);
				free_children_.push(children(c));
				children_[c.pos][c.offset] = NULL_POS;
			}
		}

		free_children_.push(children(node));
		children_[node.pos][node.offset] = NULL_POS;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                         I/O                                         |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] FileOptions fileOptions(bool const compress) const
	{
		FileOptions options;
		options.compressed   = compress;
		options.leaf_size    = size();
		options.depth_levels = depthLevels();
		return options;
	}

	[[nodiscard]] std::vector<std::pair<pos_t, BitSet<8>>> readNodes(std::istream& in)
	{
		auto tree      = readTreeStructure(in);
		auto num_nodes = readNum(in);
		return retrieveNodes(tree, num_nodes);
	}

	[[nodiscard]] std::vector<std::pair<pos_t, BitSet<8>>> readNodes(ReadBuffer& in)
	{
		auto tree      = readTreeStructure(in);
		auto num_nodes = readNum(in);
		return retrieveNodes(tree, num_nodes);
	}

	[[nodiscard]] std::unique_ptr<BitSet<8>[]> readTreeStructure(std::istream& in)
	{
		std::uint64_t num  = readNum(in);
		auto          tree = std::make_unique<BitSet<8>[]>(num);
		in.read(reinterpret_cast<char*>(tree.get()),
		        static_cast<std::streamsize>(num *
		                                     sizeof(typename decltype(tree)::element_type)));
		return tree;
	}

	[[nodiscard]] std::unique_ptr<BitSet<8>[]> readTreeStructure(ReadBuffer& in)
	{
		std::uint64_t num  = readNum(in);
		auto          tree = std::make_unique<BitSet<8>[]>(num);
		in.read(tree.get(), num * sizeof(typename decltype(tree)::element_type));
		return tree;
	}

	[[nodiscard]] std::uint64_t readNum(std::istream& in)
	{
		std::uint64_t num;
		in.read(reinterpret_cast<char*>(&num), sizeof(num));
		return num;
	}

	[[nodiscard]] std::uint64_t readNum(ReadBuffer& in)
	{
		std::uint64_t num;
		in.read(&num, sizeof(num));
		return num;
	}

	[[nodiscard]] std::vector<std::pair<pos_t, BitSet<8>>> retrieveNodes(
	    std::unique_ptr<BitSet<8>[]> const& tree, std::uint64_t num_nodes)
	{
		std::vector<std::pair<pos_t, BitSet<8>>> nodes;
		nodes.reserve(num_nodes);
		BitSet<8> const* ptr = tree.get();  // TODO: Look into
		retrieveNodesRecurs(rootIndex().pos, ptr, nodes);
		return nodes;
	}

	void retrieveNodesRecurs(pos_t pos, BitSet<8> const*& tree,
	                         std::vector<std::pair<pos_t, BitSet<8>>>& nodes)
	{
		// FIXME: Write iterative

		BitSet<8> const valid_return = *tree++;
		BitSet<8> const valid_inner  = *tree++;

		if (valid_return.any()) {
			nodes.emplace_back(pos, valid_return);
		}

		if (valid_inner.any()) {
			for (offset_t i = 0; 8 != i; ++i) {
				if (valid_inner[i]) {
					createChildren(Index(pos, i));
					retrieveNodesRecurs(children(Index(pos, i)), tree, nodes);
				}
			}
		}

		modified_[pos] |= valid_return | valid_inner;
	}

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	[[nodiscard]] std::pair<std::vector<BitSet<8>>, std::vector<pos_t>> data(
	    Predicate const& predicate) const
	{
		std::vector<BitSet<8>> tree;
		std::vector<pos_t>     indices;

		std::conditional_t<pred::contains_spatial_predicate_v<Predicate>, NodeBV, Node>
		    root_node;
		if constexpr (pred::contains_spatial_predicate_v<Predicate>) {
			root_node = rootNodeBV();
		} else {
			root_node = rootNode();
		}

		bool valid_return =
		    pred::ValueCheck<Predicate>::apply(predicate, derived(), root_node);
		bool valid_inner = !valid_return && pred::InnerCheck<Predicate>::apply(
		                                        predicate, derived(), root_node);

		tree.emplace_back(valid_return ? 1U : 0U);
		tree.emplace_back(valid_inner ? 1U : 0U);

		if (valid_return) {
			indices.push_back(rootIndex().pos);
		} else if (valid_inner) {
			dataRecurs(child(root_node, 0), predicate, tree, indices);
			if (indices.empty()) {
				tree.clear();
			}
		}

		return {std::move(tree), std::move(indices)};
	}

	template <class Predicate, class NodeType>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	void dataRecurs(NodeType const& node, Predicate const& predicate,
	                std::vector<BitSet<8>>& tree, std::vector<pos_t>& indices) const
	{
		// FIXME: Write iterative

		BitSet<8> valid_return;
		BitSet<8> valid_inner;
		for (offset_t i = 0; 8 != i; ++i) {
			auto s = sibling(node, i);

			if (pred::ValueCheck<Predicate>::apply(predicate, derived(), s)) {
				valid_return.set(i);
			} else if (pred::InnerCheck<Predicate>::apply(predicate, derived(), s)) {
				valid_inner.set(i);
			}
		}

		tree.push_back(valid_return);
		tree.push_back(valid_inner);

		auto cur_tree_size    = tree.size();
		auto cur_indices_size = indices.size();

		if (valid_return.any()) {
			indices.push_back(node.pos());
		}

		if (valid_inner.any()) {
			for (offset_t i = 0; 8 != i; ++i) {
				if (valid_inner[i]) {
					auto s = sibling(node, i);
					dataRecurs(child(s, 0), predicate, tree, indices);
				}
			}
		}

		if (indices.size() == cur_indices_size) {
			tree.resize(cur_tree_size);
			tree[tree.size() - 1] = 0;
			tree[tree.size() - 2] = 0;
		}
	}

	template <bool Propagate>
	[[nodiscard]] std::pair<std::vector<BitSet<8>>, std::vector<pos_t>> modifiedData()
	{
		std::vector<BitSet<8>> modified_tree;
		std::vector<pos_t>     modified_indices;

		modified_tree.reserve(modified_tree_size_);
		modified_indices.reserve(modified_indices_size_);

		modifiedDataRecurs(rootIndex(), modified_tree, modified_indices);

		if (modified_indices.empty()) {
			modified_tree.clear();
		}

		modified_tree_size_    = std::max(modified_tree_size_, modified_tree.size());
		modified_indices_size_ = std::max(modified_indices_size_, modified_indices.size());

		return {std::move(modified_tree), std::move(modified_indices)};
	}

	template <bool Propagate>
	void modifiedDataRecurs(pos_t pos, std::vector<BitSet<8>>& modified_tree,
	                        std::vector<pos_t>& modified_indices)
	{
		// FIXME: Write iterative

		BitSet<8> leaves(static_cast<BitSet<8>::T>(isLeaf(Index(pos, 0))) |
		                 (static_cast<BitSet<8>::T>(isLeaf(Index(pos, 1)) << 1)) |
		                 (static_cast<BitSet<8>::T>(isLeaf(Index(pos, 2)) << 2)) |
		                 (static_cast<BitSet<8>::T>(isLeaf(Index(pos, 3)) << 3)) |
		                 (static_cast<BitSet<8>::T>(isLeaf(Index(pos, 4)) << 4)) |
		                 (static_cast<BitSet<8>::T>(isLeaf(Index(pos, 5)) << 5)) |
		                 (static_cast<BitSet<8>::T>(isLeaf(Index(pos, 6)) << 6)) |
		                 (static_cast<BitSet<8>::T>(isLeaf(Index(pos, 7)) << 7)));

		BitSet<8> const valid_return = leaves & modified_[pos];
		BitSet<8> const valid_inner  = ~leaves & modified_[pos];

		modified_tree.push_back(valid_return);
		modified_tree.push_back(valid_inner);

		auto cur_tree_size    = modified_tree.size();
		auto cur_indices_size = modified_indices.size();

		if (valid_return.any()) {
			modified_indices.push_back(pos);
		}

		if (valid_inner.any()) {
			for (offset_t i = 0; 8 != i; ++i) {
				if (valid_inner[i]) {
					modifiedDataRecurs(child(pos, i));
				}
			}
		}

		if constexpr (Propagate) {
			// TODO: propagate(pos, valid_inner);
		}

		modified_[pos].reset();

		if (modified_indices.size() == cur_indices_size) {
			modified_tree.resize(cur_tree_size);
			modified_tree[modified_tree.size() - 1] = 0;
			modified_tree[modified_tree.size() - 2] = 0;
		}
	}

	void write(std::ostream& out, std::vector<BitSet<8>> const& tree,
	           std::vector<pos_t> const& nodes, bool compress, mt_t map_types,
	           int compression_acceleration_level, int compression_level) const
	{
		writeHeader(out, fileOptions(compress));
		writeTreeStructure(out, tree);
		writeNumNodes(out, nodes.size());
		writeNodes(out, nodes, compress, map_types, compression_acceleration_level,
		           compression_level);
	}

	void write(WriteBuffer& out, std::vector<BitSet<8>> const& tree,
	           std::vector<pos_t> const& nodes, bool compress, mt_t map_types,
	           int compression_acceleration_level, int compression_level) const
	{
		writeHeader(out, fileOptions(compress));
		writeTreeStructure(out, tree);
		writeNumNodes(out, nodes.size());
		writeNodes(out, nodes, compress, map_types, compression_acceleration_level,
		           compression_level);
	}

	void writeTreeStructure(std::ostream& out, std::vector<BitSet<8>> const& tree) const
	{
		std::uint64_t num = tree.size();
		out.write(reinterpret_cast<char const*>(&num), sizeof(num));
		out.write(reinterpret_cast<char const*>(tree.data()),
		          static_cast<std::streamsize>(
		              num * sizeof(typename std::decay_t<decltype(tree)>::value_type)));
	}

	void writeTreeStructure(WriteBuffer& out, std::vector<BitSet<8>> const& tree) const
	{
		std::uint64_t num = tree.size();
		out.write(&num, sizeof(num));
		out.write(tree.data(),
		          num * sizeof(typename std::decay_t<decltype(tree)>::value_type));
	}

	void writeNumNodes(std::ostream& out, std::uint64_t num_nodes) const
	{
		out.write(reinterpret_cast<char const*>(&num_nodes), sizeof(num_nodes));
	}

	void writeNumNodes(WriteBuffer& out, std::uint64_t num_nodes) const
	{
		out.write(&num_nodes, sizeof(num_nodes));
	}

	void writeNodes(std::ostream& out, std::ranges::input_range auto r, bool const compress,
	                mt_t const map_types, int const compression_acceleration_level,
	                int const compression_level) const
	{
		derived().writeNodes(out, r, compress, map_types, compression_acceleration_level,
		                     compression_level);
	}

	void writeNodes(WriteBuffer& out, std::ranges::input_range auto r, bool const compress,
	                mt_t const map_types, int const compression_acceleration_level,
	                int const compression_level) const
	{
		derived().writeNodes(out, r, compress, map_types, compression_acceleration_level,
		                     compression_level);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                    Save dot file                                    |
	|                                                                                     |
	**************************************************************************************/

	template <class Predicate>
	  requires pred::Predicate<Predicate, Derived, NodeBV>
	void saveDotFileRecurs(Index node, std::string const& id, depth_t depth,
	                       std::ostream& out, Predicate const& predicate, depth_t min_depth,
	                       mt_t map_types) const
	{
		for (ufo::offset_t i{}; 8 != i; ++i) {
			auto        c        = child(node, i);
			bool        p        = isParent(c);
			std::string child_id = std::to_string((c.pos << 3) | c.offset);
			out << child_id << " [";
			if (p) {
				out << "shape=octagon ";
			}
			auto m = isModified(c);
			if (m) {
				out << "label=<Modified: <font color='green'><b>" << m << "</b></font>";
			} else {
				out << "label=<Modified: <font color='red'><b>" << m << "</b></font>";
			}
			derived().dotFileInfo(out, c, map_types);
			out << "> " << (p ? "color=darkorange1" : "") << "]\n";
			out << id << " -- " << child_id << '\n';
			if (p && depth - 1 >= min_depth) {
				saveDotFileRecurs(c, child_id, depth - 1, out, predicate, min_depth, map_types);
			}
		}
	}

 protected:
	// The number of depth levels
	depth_t depth_levels_;
	// The maximum coordinate value the octree can store
	key_t half_max_value_;

	// Node indices
	Container<DataBlock<pos_t, 8>> children_;
	// Free node indices
	std::stack<pos_t> free_children_;

	// Parent code for the nodes
	Container<Code> parent_code_;

	// Indicates wheter a node has been modified
	Container<BitSet<8>> modified_;

	// Stores the node size at a given depth, where the depth is the index
	std::array<node_size_t, maxDepthLevels()> node_size_;
	// Reciprocal of the node size at a given depth, where the depth is the index
	std::array<node_size_t, maxDepthLevels()> node_size_factor_;

	//
	// Store for performance
	//

	std::size_t modified_tree_size_    = 0;
	std::size_t modified_indices_size_ = 0;

	//
	// Friends
	//

	friend Derived;

	template <class Derived2>
	friend class Octree;
};

}  // namespace ufo

#endif  // UFO_MAP_OCTREE_BASE_HPP