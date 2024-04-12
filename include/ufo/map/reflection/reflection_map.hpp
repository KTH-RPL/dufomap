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

#ifndef UFO_MAP_REFLECTION_MAP_HPP
#define UFO_MAP_REFLECTION_MAP_HPP

// UFO
#include <ufo/map/reflection/reflection.hpp>
#include <ufo/map/types.hpp>

// STL
#include <algorithm>
#include <limits>
#include <ranges>

namespace ufo
{
template <class Derived, std::size_t N>
class ReflectionMap
{
 public:
	//
	// Get reflection
	//

	[[nodiscard]] Reflection reflection(Index node) const
	{
		return reflection_[node.pos][node.offset];
	}

	[[nodiscard]] Reflection reflection(Node node) const
	{
		return reflection(derived().index(node));
	}

	[[nodiscard]] Reflection reflection(Code code) const
	{
		return reflection(derived().index(code));
	}

	[[nodiscard]] Reflection reflection(Key key) const
	{
		return reflection(derived().index(key));
	}

	[[nodiscard]] Reflection reflection(Point coord, depth_t depth = 0) const
	{
		return reflection(derived().index(coord, depth));
	}

	[[nodiscard]] Reflection reflection(coord_t x, coord_t y, coord_t z,
	                                    depth_t depth = 0) const
	{
		return reflection(derived().index(x, y, z, depth));
	}

	//
	// Get reflectiveness
	//

	[[nodiscard]] reflection_t reflectiveness(Index node) const
	{
		return reflection(node).reflectiveness();
	}

	[[nodiscard]] reflection_t reflectiveness(Node node) const
	{
		return reflectiveness(derived().index(node));
	}

	[[nodiscard]] reflection_t reflectiveness(Code code) const
	{
		return reflectiveness(derived().index(code));
	}

	[[nodiscard]] reflection_t reflectiveness(Key key) const
	{
		return reflectiveness(derived().index(key));
	}

	[[nodiscard]] reflection_t reflectiveness(Point coord, depth_t depth = 0) const
	{
		return reflectiveness(derived().index(coord, depth));
	}

	[[nodiscard]] reflection_t reflectiveness(coord_t x, coord_t y, coord_t z,
	                                          depth_t depth = 0) const
	{
		return reflectiveness(derived().index(x, y, z, depth));
	}

	//
	// Get hits
	//

	[[nodiscard]] count_t hits(Index node) const { return reflection(node).hits; }

	[[nodiscard]] count_t hits(Node node) const { return hits(derived().index(node)); }

	[[nodiscard]] count_t hits(Code code) const { return hits(derived().index(code)); }

	[[nodiscard]] count_t hits(Key key) const { return hits(derived().index(key)); }

	[[nodiscard]] count_t hits(Point coord, depth_t depth = 0) const
	{
		return hits(derived().index(coord, depth));
	}

	[[nodiscard]] count_t hits(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return hits(derived().index(x, y, z, depth));
	}

	//
	// Get misses
	//

	[[nodiscard]] count_t misses(Index node) const { return reflection(node).misses; }

	[[nodiscard]] count_t misses(Node node) const { return misses(derived().index(node)); }

	[[nodiscard]] count_t misses(Code code) const { return misses(derived().index(code)); }

	[[nodiscard]] count_t misses(Key key) const { return misses(derived().index(key)); }

	[[nodiscard]] count_t misses(Point coord, depth_t depth = 0) const
	{
		return misses(derived().index(coord, depth));
	}

	[[nodiscard]] count_t misses(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return misses(derived().index(x, y, z, depth));
	}

	//
	// Set reflection
	//

	void setReflection(Index node, Reflection value)
	{
		derived().apply(
		    node, [this, value](Index node) { reflection_[node.pos][node.offset] = value; },
		    [this, value](pos_t pos) { reflection_[pos].fill(value); });
	}

	Node setReflection(Node node, Reflection value, bool propagate = true)
	{
		return derived().apply(
		    node, [this, value](Index node) { reflection_[node.pos][node.offset] = value; },
		    [this, value](pos_t pos) { reflection_[pos].fill(value); }, propagate);
	}

	Node setReflection(Code code, Reflection value, bool propagate = true)
	{
		return derived().apply(
		    code, [this, value](Index node) { reflection_[node.pos][node.offset] = value; },
		    [this, value](pos_t pos) { reflection_[pos].fill(value); }, propagate);
	}

	Node setReflection(Key key, Reflection value, bool propagate = true)
	{
		return setReflection(derived().toCode(key), value, propagate);
	}

	Node setReflection(Point coord, Reflection value, bool propagate = true,
	                   depth_t depth = 0)
	{
		return setReflection(derived().toCode(coord, depth), value, propagate);
	}

	Node setReflection(coord_t x, coord_t y, coord_t z, Reflection value,
	                   bool propagate = true, depth_t depth = 0)
	{
		return setReflection(derived().toCode(x, y, z, depth), value, propagate);
	}

	//
	// Set hits
	//

	void setHits(Index node, count_t value)
	{
		return derived().apply(
		    node,
		    [this, value](Index node) { reflection_[node.pos][node.offset].hits = value; },
		    [this, value](pos_t pos) {
			    for (auto& r : reflection_[pos]) {
				    r.hits = value;
			    }
		    });
	}

	Node setHits(Node node, count_t value, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, value](Index node) { reflection_[node.pos][node.offset].hits = value; },
		    [this, value](pos_t pos) {
			    for (auto& r : reflection_[pos]) {
				    r.hits = value;
			    }
		    },
		    propagate);
	}

	Node setHits(Code code, count_t value, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, value](Index node) { reflection_[node.pos][node.offset].hits = value; },
		    [this, value](pos_t pos) {
			    for (auto& r : reflection_[pos]) {
				    r.hits = value;
			    }
		    },
		    propagate);
	}

	Node setHits(Key key, count_t value, bool propagate = true)
	{
		return setHits(derived().toCode(key), value, propagate);
	}

	Node setHits(Point coord, count_t value, bool propagate = true, depth_t depth = 0)
	{
		return setHits(derived().toCode(coord, depth), value, propagate);
	}

	Node setHits(coord_t x, coord_t y, coord_t z, count_t value, bool propagate = true,
	             depth_t depth = 0)
	{
		return setHits(derived().toCode(x, y, z, depth), value, propagate);
	}

	//
	// Set misses
	//

	void setMisses(Index node, count_t value)
	{
		return derived().apply(
		    node,
		    [this, value](Index node) { reflection_[node.pos][node.offset].misses = value; },
		    [this, value](pos_t pos) {
			    for (auto& r : reflection_[pos]) {
				    r.misses = value;
			    }
		    });
	}

	Node setMisses(Node node, count_t value, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, value](Index node) { reflection_[node.pos][node.offset].misses = value; },
		    [this, value](pos_t pos) {
			    for (auto& r : reflection_[pos]) {
				    r.misses = value;
			    }
		    },
		    propagate);
	}

	Node setMisses(Code code, count_t value, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, value](Index node) { reflection_[node.pos][node.offset].misses = value; },
		    [this, value](pos_t pos) {
			    for (auto& r : reflection_[pos]) {
				    r.misses = value;
			    }
		    },
		    propagate);
	}

	Node setMisses(Key key, count_t value, bool propagate = true)
	{
		return setMisses(derived().toCode(key), value, propagate);
	}

	Node setMisses(Point coord, count_t value, bool propagate = true, depth_t depth = 0)
	{
		return setMisses(derived().toCode(coord, depth), value, propagate);
	}

	Node setMisses(coord_t x, coord_t y, coord_t z, count_t value, bool propagate = true,
	               depth_t depth = 0)
	{
		return setMisses(derived().toCode(x, y, z, depth), value, propagate);
	}

	//
	// Update reflection
	//

	void updateReflection(Index node, int hits_change, int misses_change)
	{
		derived().apply(
		    node,
		    [this, hits_change, misses_change](Index node) {
			    reflection_[node.pos][node.offset].hits += hits_change;
			    reflection_[node.pos][node.offset].misses += misses_change;
		    },
		    [this, hits_change, misses_change](pos_t pos) {
			    for (auto& e : reflection_[pos]) {
				    e.hits += hits_change;
				    e.misses += misses_change;
			    }
		    });
	}

	void updateReflection(Index node, std::invocable<Reflection> auto unary_op)
	{
		derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    reflection_[node.pos][node.offset] =
			        unary_op(reflection_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : reflection_[pos]) {
				    e = unary_op(e);
			    }
		    });
	}

	void updateReflection(Index node, std::invocable<Index, Reflection> auto binary_op)
	{
		derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    reflection_[node.pos][node.offset] =
			        binary_op(node, reflection_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (std::size_t i{}; auto& e : reflection_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    });
	}

	Node updateReflection(Node node, int hits_change, int misses_change,
	                      bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, hits_change, misses_change](Index node) {
			    reflection_[node.pos][node.offset].hits += hits_change;
			    reflection_[node.pos][node.offset].misses += misses_change;
		    },
		    [this, hits_change, misses_change](pos_t pos) {
			    for (auto& e : reflection_[pos]) {
				    e.hits += hits_change;
				    e.misses += misses_change;
			    }
		    },
		    propagate);
	}

	Node updateReflection(Node node, std::invocable<Reflection> auto unary_op,
	                      bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    reflection_[node.pos][node.offset] =
			        unary_op(reflection_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : reflection_[pos]) {
				    e = unary_op(e);
			    }
		    },
		    propagate);
	}

	Node updateReflection(Node node, std::invocable<Index, Reflection> auto binary_op,
	                      bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    reflection_[node.pos][node.offset] =
			        binary_op(node, reflection_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (std::size_t i{}; auto& e : reflection_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    },
		    propagate);
	}

	Node updateReflection(Code code, int hits_change, int misses_change,
	                      bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, hits_change, misses_change](Index node) {
			    reflection_[node.pos][node.offset].hits += hits_change;
			    reflection_[node.pos][node.offset].misses += misses_change;
		    },
		    [this, hits_change, misses_change](pos_t pos) {
			    for (auto& e : reflection_[pos]) {
				    e.hits += hits_change;
				    e.misses += misses_change;
			    }
		    },
		    propagate);
	}

	Node updateReflection(Code code, std::invocable<Reflection> auto unary_op,
	                      bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, unary_op](Index node) {
			    reflection_[node.pos][node.offset] =
			        unary_op(reflection_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : reflection_[pos]) {
				    e = unary_op(e);
			    }
		    },
		    propagate);
	}

	Node updateReflection(Code code, std::invocable<Index, Reflection> auto binary_op,
	                      bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, binary_op](Index node) {
			    reflection_[node.pos][node.offset] =
			        binary_op(node, reflection_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (std::size_t i{}; auto& e : reflection_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    },
		    propagate);
	}

	Node updateReflection(Key key, int hits_change, int misses_change,
	                      bool propagate = true)
	{
		return updateReflection(derived().toCode(key), hits_change, misses_change, propagate);
	}

	Node updateReflection(Key key, std::invocable<Reflection> auto unary_op,
	                      bool propagate = true)
	{
		return updateReflection(derived().toCode(key), unary_op, propagate);
	}

	Node updateReflection(Key key, std::invocable<Index, Reflection> auto binary_op,
	                      bool propagate = true)
	{
		return updateReflection(derived().toCode(key), binary_op, propagate);
	}

	Node updateReflection(Point coord, int hits_change, int misses_change,
	                      bool propagate = true, depth_t depth = 0)
	{
		return updateReflection(derived().toCode(coord, depth), hits_change, misses_change,
		                        propagate);
	}

	Node updateReflection(Point coord, std::invocable<Reflection> auto unary_op,
	                      bool propagate = true, depth_t depth = 0)
	{
		return updateReflection(derived().toCode(coord, depth), unary_op, propagate);
	}

	Node updateReflection(Point coord, std::invocable<Index, Reflection> auto binary_op,
	                      bool propagate = true, depth_t depth = 0)
	{
		return updateReflection(derived().toCode(coord, depth), binary_op, propagate);
	}

	Node updateReflection(coord_t x, coord_t y, coord_t z, int hits_change,
	                      int misses_change, bool propagate = true, depth_t depth = 0)
	{
		return updateReflection(derived().toCode(x, y, z, depth), hits_change, misses_change,
		                        propagate);
	}

	Node updateReflection(coord_t x, coord_t y, coord_t z,
	                      std::invocable<Reflection> auto unary_op, bool propagate = true,
	                      depth_t depth = 0)
	{
		return updateReflection(derived().toCode(x, y, z, depth), unary_op, propagate);
	}

	Node updateReflection(coord_t x, coord_t y, coord_t z,
	                      std::invocable<Index, Reflection> auto binary_op,
	                      bool propagate = true, depth_t depth = 0)
	{
		return updateReflection(derived().toCode(x, y, z, depth), binary_op, propagate);
	}

	//
	// Propagation criteria
	//

	[[nodiscard]] constexpr PropagationCriteria reflectionPropagationCriteria()
	    const noexcept
	{
		return prop_criteria_;
	}

	constexpr void setReflectionPropagationCriteria(PropagationCriteria prop_criteria,
	                                                bool propagate = true) noexcept
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

	ReflectionMap() { reflection_.emplace_back(); }

	ReflectionMap(ReflectionMap const& other) = default;

	ReflectionMap(ReflectionMap&& other) = default;

	template <class Derived2>
	ReflectionMap(ReflectionMap<Derived2, N> const& other)
	    : reflection_(other.reflection_), prop_criteria_(other.prop_criteria_)
	{
	}

	template <class Derived2>
	ReflectionMap(ReflectionMap<Derived2, N>&& other)
	    : reflection_(std::move(other.reflection_))
	    , prop_criteria_(std::move(other.prop_criteria_))
	{
	}

	//
	// Destructor
	//

	~ReflectionMap() = default;

	//
	// Assignment operator
	//

	ReflectionMap& operator=(ReflectionMap const& rhs) = default;

	ReflectionMap& operator=(ReflectionMap&& rhs) = default;

	template <class Derived2>
	ReflectionMap& operator=(ReflectionMap<Derived2, N> const& rhs)
	{
		reflection_    = rhs.reflection_;
		prop_criteria_ = rhs.prop_criteria_;
		return *this;
	}

	template <class Derived2>
	ReflectionMap& operator=(ReflectionMap<Derived2, N>&& rhs)
	{
		reflection_    = std::move(rhs.reflection_);
		prop_criteria_ = std::move(rhs.prop_criteria_);
		return *this;
	}

	//
	// Swap
	//

	void swap(ReflectionMap& other) noexcept
	{
		std::swap(reflection_, other.reflection_);
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
		reflection_.emplace_back();
		reflection_.back().fill(reflection_[node.pos][node.offset]);
	}

	//
	// Resize
	//

	void resize(std::size_t count)
	{
		// TODO: Implement
		reflection_.resize(count);
	}

	//
	// Reserve
	//

	void reserveImpl(std::size_t new_cap) { reflection_.reserve(new_cap); }

	//
	// Initialize root
	//

	void initRoot()
	{
		auto node                                 = derived().rootIndex();
		reflection_[node.pos][node.offset].hits   = 0;
		reflection_[node.pos][node.offset].misses = 0;
	}

	//
	// Fill
	//

	void fill(Index node, pos_t children)
	{
		reflection_[children].fill(reflection_[node.pos][node.offset]);
	}

	//
	// Clear
	//

	void clearImpl() { reflection_.resize(1); }

	void clearImpl(pos_t nodes) {}

	//
	// Shrink to fit
	//

	void shrinkToFitImpl() { reflection_.shrink_to_fit(); }

	//
	// Update node
	//

	void updateNode(Index node, pos_t children)
	{
		// TODO: Add sum?
		// TODO: What does 'MIN', 'MAX', and 'MEAN' mean?
		switch (reflectionPropagationCriteria()) {
			case PropagationCriteria::MIN:
				reflection_[node.pos][node.offset] = min(children);
				return;
			case PropagationCriteria::MAX:
				reflection_[node.pos][node.offset] = max(children);
				return;
			case PropagationCriteria::MEAN:
				reflection_[node.pos][node.offset] = mean(children);
				return;
			case PropagationCriteria::FIRST:
				reflection_[node.pos][node.offset] = reflection_[children].front();
				return;
			case PropagationCriteria::NONE: return;
		}
	}

	[[nodiscard]] constexpr Reflection min(pos_t nodes) const
	{
		Reflection res(std::numeric_limits<count_t>::max(),
		               std::numeric_limits<count_t>::max());
		for (auto r : reflection_[nodes]) {
			res.hits   = std::min(res.hits, r.hits);
			res.misses = std::min(res.misses, r.misses);
		}
		return res;
	}

	[[nodiscard]] constexpr Reflection max(pos_t nodes) const
	{
		Reflection res(std::numeric_limits<count_t>::lowest(),
		               std::numeric_limits<count_t>::lowest());
		for (auto r : reflection_[nodes]) {
			res.hits   = std::max(res.hits, r.hits);
			res.misses = std::max(res.misses, r.misses);
		}
		return res;
	}

	[[nodiscard]] constexpr Reflection mean(pos_t nodes) const
	{
		count_t hits{}, misses{};
		for (auto r : reflection_[nodes]) {
			hits += r.hits;
			misses += r.misses;
		}
		return Reflection(hits / static_cast<count_t>(N), misses / static_cast<count_t>(N));
	}

	//
	// Is prunable
	//

	[[nodiscard]] bool isPrunable(pos_t nodes) const
	{
		// TODO: Use floor(log2(X))?
		return std::all_of(std::cbegin(reflection_[nodes]) + 1, std::cend(reflection_[nodes]),
		                   [a = reflection_[nodes].front()](auto b) { return a == b; });
	}

	//
	// Memory node block
	//

	[[nodiscard]] static constexpr std::size_t memoryNodeBlock() noexcept
	{
		return N * sizeof(Reflection);
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr MapType mapType() noexcept
	{
		return MapType::REFLECTION;
	}

	constexpr std::size_t serializedSize(std::ranges::input_range auto r) const
	{
		return std::ranges::size(r) * memoryNodeBlock();
	}

	void readNodes(ReadBuffer& in, std::ranges::input_range auto r)
	{
		for (auto const [pos, offsets] : r) {
			if (offsets.all()) {
				in.read(reflection_[pos].data(), memoryNodeBlock());
			} else {
				DataBlock<reflection_t, N> reflection;
				in.read(reflection.data(), memoryNodeBlock());
				for (offset_t i{}; N != i; ++i) {
					reflection_[pos][i] = offsets[i] ? reflection[i] : reflection_[pos][i];
				}
			}
		}
	}

	void writeNodes(WriteBuffer& out, std::ranges::input_range auto r) const
	{
		out.reserve(out.size() + serializedSize(r));
		for (auto pos : r) {
			out.write(reflection_[pos].data(), memoryNodeBlock());
		}
	}

	//
	// Dot file info
	//

	std::ostream& dotFileInfo(std::ostream& out, Index node) const
	{
		return out << reflection_[node.pos][node.offset];
	}

 protected:
	// Data
	Container<DataBlock<Reflection, N>> reflection_;

	// Propagation criteria
	PropagationCriteria prop_criteria_ = PropagationCriteria::MAX;

	template <class Derived2, std::size_t N2>
	friend class ReflectionMap;
};

//
// Concepts
//

template <class Map>
concept IsReflectionMap = IsMapType<Map, MapType::REFLECTION>;
}  // namespace ufo

#endif  // UFO_MAP_REFLECTION_MAP_HPP