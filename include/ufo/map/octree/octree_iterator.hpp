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

#ifndef UFO_MAP_OCTREE_ITERATOR_HPP
#define UFO_MAP_OCTREE_ITERATOR_HPP

// UFO
#include <ufo/geometry/bounding_volume.hpp>
#include <ufo/geometry/minimum_distance.hpp>
#include <ufo/geometry/point.hpp>
#include <ufo/map/code.hpp>
#include <ufo/map/node.hpp>
#include <ufo/map/octree/octree_predicate.hpp>
#include <ufo/map/predicate/predicate.hpp>
#include <ufo/map/types.hpp>

// STL
#include <cstddef>   // For std::ptrdiff_t
#include <iterator>  // For std::forward_iterator_tag
#include <memory>
#include <queue>
#include <set>
#include <stack>
#include <type_traits>
// #include <utility>

namespace ufo
{
template <class Tree, typename T>
class OctreeIteratorBase
{
 public:
	// Tags
	using iterator_category = std::forward_iterator_tag;
	using difference_type   = std::ptrdiff_t;
	using value_type        = T;
	using pointer           = value_type*;
	using reference         = value_type&;
	using const_pointer     = value_type const*;
	using const_reference   = value_type const&;

	constexpr OctreeIteratorBase(Tree const* tree) : tree_(tree) {}

	virtual ~OctreeIteratorBase() {}

	virtual void next() = 0;

	virtual OctreeIteratorBase* copy() = 0;

	// virtual reference data() = 0;
	virtual const_reference data() const = 0;

	constexpr Tree const* tree() const { return tree_; }

	virtual bool equal(OctreeIteratorBase const& other) const = 0;

	virtual std::size_t status() const = 0;

 protected:
	template <class NodeType>
	[[nodiscard]] constexpr bool isPureLeaf(NodeType const& node) const
	{
		return tree_->isPureLeaf(node);
	}

	template <class NodeType>
	[[nodiscard]] constexpr bool isParent(NodeType const& node) const
	{
		return tree_->isParent(node.index());
	}

	template <class NodeType>
	[[nodiscard]] constexpr NodeType child(NodeType const& node, offset_t child) const
	{
		return tree_->child(node, child);
	}

	template <class NodeType>
	[[nodiscard]] constexpr NodeType sibling(NodeType const& node, offset_t sibling) const
	{
		return tree_->sibling(node, sibling);
	}

	template <class NodeType>
	[[nodiscard]] constexpr bool exists(NodeType const& node) const
	{
		return tree_->exists(node.index());
	}

	template <bool OnlyExists, class NodeType, class Predicates>
	[[nodiscard]] constexpr bool validInner(NodeType const&   node,
	                                        Predicates const& predicates) const
	{
		if constexpr (OnlyExists) {
			return isParent(node) &&
			       pred::InnerCheck<Predicates>::apply(predicates, *tree_, node);
		} else {
			return !isPureLeaf(node) &&
			       pred::InnerCheck<Predicates>::apply(predicates, *tree_, node);
		}
	}

	template <class NodeType, class Predicates>
	[[nodiscard]] constexpr bool validReturn(NodeType const&   node,
	                                         Predicates const& predicates) const
	{
		return pred::ValueCheck<Predicates>::apply(predicates, *tree_, node);
	}

 protected:
	// The UFOMap
	Tree const* tree_;
};

template <class Tree, typename T>
class OctreeIteratorWrapper
{
 private:
	using Base = OctreeIteratorBase<Tree, T>;

 public:
	// Tags
	using const_pointer     = typename Base::const_pointer;
	using const_reference   = typename Base::const_reference;
	using difference_type   = typename Base::difference_type;
	using iterator_category = typename Base::iterator_category;
	using pointer           = typename Base::pointer;
	using reference         = typename Base::reference;
	using value_type        = typename Base::value_type;

	OctreeIteratorWrapper(Base* it_base) : it_base_(it_base) {}

	OctreeIteratorWrapper(OctreeIteratorWrapper const& other)
	    : it_base_(other.it_base_->copy())
	{
	}

	OctreeIteratorWrapper& operator++()
	{
		it_base_->next();
		return *this;
	}

	OctreeIteratorWrapper operator++(int)
	{
		OctreeIteratorWrapper result(it_base_->copy());
		++(*this);
		return result;
	}

	// pointer operator->() { return &(it_base_->data()); }

	// reference operator*() { return it_base_->data(); }

	const_pointer operator->() const { return &(it_base_->data()); }

	const_reference operator*() const { return it_base_->data(); }

	friend bool operator==(OctreeIteratorWrapper const& lhs,
	                       OctreeIteratorWrapper const& rhs)
	{
		return lhs.it_base_->equal(*(rhs.it_base_));
	}

	friend bool operator!=(OctreeIteratorWrapper const& lhs,
	                       OctreeIteratorWrapper const& rhs)
	{
		return !(lhs == rhs);
	}

 private:
	std::unique_ptr<Base> it_base_;
};

template <class BaseNodeType, bool OnlyExists, bool EarlyStopping, class Tree,
          class NodeType = Node, pred::Predicate<Tree, NodeType> Predicates = pred::TRUE>
class OctreeIterator final : public OctreeIteratorBase<Tree, BaseNodeType>
{
 private:
	static constexpr bool OnlyLeavesOrFixedDepth =
	    pred::contains_always_predicate_v<pred::PureLeaf, Predicates> ||
	    pred::contains_always_predicate_v<pred::DepthE, Predicates> || EarlyStopping;

	using Base = OctreeIteratorBase<Tree, BaseNodeType>;

 public:
	// Tags
	using typename Base::const_pointer;
	using typename Base::const_reference;
	using typename Base::difference_type;
	using typename Base::iterator_category;
	using typename Base::pointer;
	using typename Base::reference;
	using typename Base::value_type;

	OctreeIterator(Tree const* tree) : Base(tree) {}

	OctreeIterator(Tree const* tree, NodeType const& root, Predicates const& predicates)
	    : Base(tree), predicates_(predicates)
	{
		init(root);
	}

	void next() override
	{
		return_index_ -= return_index_ ? 1 : 0;

		// Skip forward to next valid return node
		while (0 == return_index_ && inner_index_) {
			auto current = this->child(inner_nodes_[--inner_index_], 7);

			// Go down the tree
			for (offset_t i{8}; 0 != i;) {
				current = this->sibling(current, --i);

				if constexpr (OnlyLeavesOrFixedDepth) {
					if (this->validReturn(current, predicates_)) {
						return_nodes_[return_index_++] = current;
					} else if (this->template validInner<OnlyExists>(current, predicates_)) {
						inner_nodes_[inner_index_++] = current;
					}
				} else {
					if (this->validReturn(current, predicates_)) {
						return_nodes_[return_index_++] = current;
					}
					if (this->template validInner<OnlyExists>(current, predicates_)) {
						inner_nodes_[inner_index_++] = current;
					}
				}
			}
		}
	}

	OctreeIterator* copy() override { return new OctreeIterator(*this); }

	// reference data() override { return node_; }

	const_reference data() const override { return return_nodes_[return_index_ - 1]; }

	bool equal(Base const& other) const override
	{
		return status() == other.status();
		// return other.tree() == this->tree() && other.status() == status() &&
		//        (!return_nodes_.empty() && other.data() == data());
	}

	std::size_t status() const override { return inner_index_ + return_index_; }

 private:
	void init(NodeType const& node)
	{
		if constexpr (OnlyExists) {
			if (!this->exists(node)) {
				return;
			}
		}

		if constexpr (OnlyLeavesOrFixedDepth) {
			if (this->validReturn(node, predicates_)) {
				return_nodes_[return_index_++] = node;
			} else if (this->template validInner<OnlyExists>(node, predicates_)) {
				inner_nodes_[inner_index_++] = node;
				next();
			}
		} else {
			if (this->validReturn(node, predicates_)) {
				return_nodes_[return_index_++] = node;
				if (this->template validInner<OnlyExists>(node, predicates_)) {
					inner_nodes_[inner_index_++] = node;
				}
			} else if (this->template validInner<OnlyExists>(node, predicates_)) {
				inner_nodes_[inner_index_++] = node;
				next();
			}
		}
	}

 private:
	Predicates const predicates_{};  // Predicates that nodes has to fulfill

	std::array<NodeType, 8 * Tree::maxDepthLevels()>
	                        inner_nodes_;   // To be processed inner nodes
	std::array<NodeType, 8> return_nodes_;  // To be processed return nodes
	int                     inner_index_{};
	int                     return_index_{};
};

template <bool OnlyExists, bool EarlyStopping, class Tree, class Geometry = Point,
          pred::Predicate<Tree, NearestNode> Predicates = pred::TRUE>
class OctreeIteratorNearest final : public OctreeIteratorBase<Tree, NearestNode>
{
 private:
	static constexpr bool OnlyLeavesOrFixedDepth =
	    pred::contains_always_predicate_v<pred::PureLeaf, Predicates> ||
	    pred::contains_always_predicate_v<pred::DepthE, Predicates> || EarlyStopping;

	using Base = OctreeIteratorBase<Tree, NearestNode>;

 public:
	// Tags
	using typename Base::const_pointer;
	using typename Base::const_reference;
	using typename Base::difference_type;
	using typename Base::iterator_category;
	using typename Base::pointer;
	using typename Base::reference;
	using typename Base::value_type;

 private:
	using Queue =
	    std::priority_queue<value_type, std::vector<value_type>, std::greater<value_type>>;

 public:
	OctreeIteratorNearest(Tree const* tree) : Base(tree) {}

	OctreeIteratorNearest(Tree const* tree, NodeBV const& root, Geometry const& geometry,
	                      Predicates const& predicates, double epsilon = 0.0)
	    : Base(tree), predicates_(predicates), geometry_(geometry), epsilon_(epsilon)
	{
		init(root);
	}

	void next() override
	{
		if (!return_nodes_.empty()) {
			return_nodes_.pop();
		}

		// Skip forward to next valid return node
		while (!inner_nodes_.empty()) {
			if (!return_nodes_.empty() && return_nodes_.top() <= inner_nodes_.top()) {
				return;
			}

			auto current = this->child(inner_nodes_.top().node, 0);
			inner_nodes_.pop();

			for (offset_t idx{}; 8 != idx; ++idx) {
				current = this->sibling(current, idx);

				if constexpr (OnlyLeavesOrFixedDepth) {
					if (this->validReturn(current, predicates_)) {
						return_nodes_.emplace(current, squaredDistance(current));
					} else if (this->template validInner<OnlyExists>(current, predicates_)) {
						inner_nodes_.emplace(current, squaredDistance(current) + epsilon_);
					}
				} else {
					if (this->validReturn(current, predicates_)) {
						auto dist_sq = squaredDistance(current);
						return_nodes_.emplace(current, dist_sq);
						if (this->template validInner<OnlyExists>(current, predicates_)) {
							inner_nodes_.emplace(current, dist_sq + epsilon_);
						}
					} else if (this->template validInner<OnlyExists>(current, predicates_)) {
						inner_nodes_.emplace(current, squaredDistance(current) + epsilon_);
					}
				}
			}
		}
	}

	OctreeIteratorNearest* copy() override { return new OctreeIteratorNearest(*this); }

	// reference data() override { return return_nodes_.top(); }

	const_reference data() const override { return return_nodes_.top(); }

	bool equal(Base const& other) const override
	{
		return other.tree() == this->tree() && other.status() == status() &&
		       (!return_nodes_.empty() && other.data() == data());
	}

	std::size_t status() const override
	{
		return inner_nodes_.size() + return_nodes_.size();
	}

 private:
	double squaredDistance(NodeBV const& node) const
	{
		return squaredDistance(node.boundingVolume(), geometry_);
	}

	void init(NodeBV const& node)
	{
		if (OnlyExists) {
			if (!this->exists(node)) {
				return;
			}
		}

		if constexpr (OnlyLeavesOrFixedDepth) {
			if (this->validReturn(node, predicates_)) {
				return_nodes_.emplace(node, squaredDistance(node));
			} else if (this->template validInner<OnlyExists>(node, predicates_)) {
				std::vector<value_type> container;
				container.reserve(256);
				return_nodes_ = std::priority_queue<value_type, std::vector<value_type>,
				                                    std::greater<value_type>>(
				    std::greater<value_type>(), std::move(container));
				inner_nodes_ = return_nodes_;

				inner_nodes_.emplace(node, squaredDistance(node) + epsilon_);
				next();
			}
		} else {
			if (this->validReturn(node, predicates_)) {
				std::vector<value_type> container;
				container.reserve(256);
				return_nodes_ = std::priority_queue<value_type, std::vector<value_type>,
				                                    std::greater<value_type>>(
				    std::greater<value_type>(), std::move(container));
				inner_nodes_ = return_nodes_;

				auto const dist_sq = squaredDistance(node);

				return_nodes_.emplace(node, dist_sq);
				if (this->template validInner<OnlyExists>(node, predicates_)) {
					inner_nodes_.emplace(node, dist_sq + epsilon_);
				}
			} else if (this->template validInner<OnlyExists>(node, predicates_)) {
				std::vector<value_type> container;
				container.reserve(256);
				return_nodes_ = std::priority_queue<value_type, std::vector<value_type>,
				                                    std::greater<value_type>>(
				    std::greater<value_type>(), std::move(container));
				inner_nodes_ = return_nodes_;

				inner_nodes_.emplace(node, squaredDistance(node) + epsilon_);
				next();
			}
		}
	}

 private:
	Predicates const predicates_;    // Predicates that nodes has to fulfill
	Geometry const   geometry_;      // Geometry to find nearest to
	double const     epsilon_;       // Epsilon for approximate search
	Queue            inner_nodes_;   // To be processed inner nodes
	Queue            return_nodes_;  // To be processed return nodes
};
}  // namespace ufo

#endif  // UFO_MAP_OCTREE_ITERATOR_HPP
