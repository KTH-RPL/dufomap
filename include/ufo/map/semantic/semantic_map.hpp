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

#ifndef UFO_MAP_SEMANTIC_MAP_HPP
#define UFO_MAP_SEMANTIC_MAP_HPP

// UFO
#include <ufo/map/buffer.hpp>
#include <ufo/map/code.hpp>
#include <ufo/map/index.hpp>
#include <ufo/map/key.hpp>
#include <ufo/map/node.hpp>
#include <ufo/map/point.hpp>
#include <ufo/map/semantic/semantic.hpp>
#include <ufo/map/types.hpp>

// STL
#include <limits>

namespace ufo
{
enum class SemanticPropagationCriteria { MIN, MAX, NONE };

template <class Derived, std::size_t N>
class SemanticMap
{
 public:
	//
	// Get semantic
	//

	[[nodiscard]] Semantic semantic(Index node) const
	{
		return semantic_[node.pos][node.offset];
	}

	[[nodiscard]] Semantic semantic(Node node) const
	{
		return semantic(derived().index(node));
	}

	[[nodiscard]] Semantic semantic(Code code) const
	{
		return semantic(derived().index(code));
	}

	[[nodiscard]] Semantic semantic(Key key) const
	{
		return semantic(derived().index(key));
	}

	[[nodiscard]] Semantic semantic(Point coord, depth_t depth = 0) const
	{
		return semantic(derived().index(coord, depth));
	}

	[[nodiscard]] Semantic semantic(coord_t x, coord_t y, coord_t z,
	                                depth_t depth = 0) const
	{
		return semantic(derived().index(x, y, z, depth));
	}

	//
	// Set semantic
	//

	void setSemantic(Index node, Semantic value)
	{
		derived().apply(
		    node, [this, value](Index node) { semantic_[node.pos][node.offset] = value; },
		    [this, value](pos_t pos) { semantic_[pos].fill(value); });
	}

	Node setSemantic(Node node, Semantic value, bool propagate = true)
	{
		return derived().apply(
		    node, [this, value](Index node) { semantic_[node.pos][node.offset] = value; },
		    [this, value](pos_t pos) { semantic_[pos].fill(value); }, propagate);
	}

	Node setSemantic(Code code, Semantic value, bool propagate = true)
	{
		return derived().apply(
		    code, [this, value](Index node) { semantic_[node.pos][node.offset] = value; },
		    [this, value](pos_t pos) { semantic_[pos].fill(value); }, propagate);
	}

	Node setSemantic(Key key, Semantic value, bool propagate = true)
	{
		return setSemantic(derived().toCode(key), value, propagate);
	}

	Node setSemantic(Point coord, Semantic value, bool propagate = true, depth_t depth = 0)
	{
		return setSemantic(derived().toCode(coord, depth), value, propagate);
	}

	Node setSemantic(coord_t x, coord_t y, coord_t z, Semantic value, bool propagate = true,
	                 depth_t depth = 0)
	{
		return setSemantic(derived().toCode(x, y, z, depth), value, propagate);
	}

	//
	// Update semantic
	//

	void updateSemantic(Index node, std::invocable<Semantic> auto unary_op)
	{
		derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    semantic_[node.pos][node.offset] = unary_op(semantic_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : semantic_[pos]) {
				    e = unary_op(e);
			    }
		    });
	}

	void updateSemantic(Index node, std::invocable<Index, Semantic> auto binary_op)
	{
		derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    semantic_[node.pos][node.offset] =
			        binary_op(node, semantic_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (std::size_t i{}; auto& e : semantic_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    });
	}

	Node updateSemantic(Node node, std::invocable<Semantic> auto unary_op,
	                    bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    semantic_[node.pos][node.offset] = unary_op(semantic_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : semantic_[pos]) {
				    e = unary_op(e);
			    }
		    },
		    propagate);
	}

	Node updateSemantic(Node node, std::invocable<Index, Semantic> auto binary_op,
	                    bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    semantic_[node.pos][node.offset] =
			        binary_op(node, semantic_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (std::size_t i{}; auto& e : semantic_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    },
		    propagate);
	}

	Node updateSemantic(Code code, std::invocable<Semantic> auto unary_op,
	                    bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, unary_op](Index node) {
			    semantic_[node.pos][node.offset] = unary_op(semantic_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (auto& e : semantic_[pos]) {
				    e = unary_op(e);
			    }
		    },
		    propagate);
	}

	Node updateSemantic(Code code, std::invocable<Index, Semantic> auto binary_op,
	                    bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, binary_op](Index node) {
			    semantic_[node.pos][node.offset] =
			        binary_op(node, semantic_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (std::size_t i{}; auto& e : semantic_[pos]) {
				    e = binary_op(Index(pos, i++), e);
			    }
		    },
		    propagate);
	}

	Node updateSemantic(Key key, std::invocable<Semantic> auto unary_op,
	                    bool propagate = true)
	{
		return updateSemantic(derived().toCode(key), unary_op, propagate);
	}

	Node updateSemantic(Key key, std::invocable<Index, Semantic> auto binary_op,
	                    bool propagate = true)
	{
		return updateSemantic(derived().toCode(key), binary_op, propagate);
	}

	Node updateSemantic(Point coord, std::invocable<Semantic> auto unary_op,
	                    bool propagate = true, depth_t depth = 0)
	{
		return updateSemantic(derived().toCode(coord, depth), unary_op, propagate);
	}

	Node updateSemantic(Point coord, std::invocable<Index, Semantic> auto binary_op,
	                    bool propagate = true, depth_t depth = 0)
	{
		return updateSemantic(derived().toCode(coord, depth), binary_op, propagate);
	}

	Node updateSemantic(coord_t x, coord_t y, coord_t z,
	                    std::invocable<Semantic> auto unary_op, bool propagate = true,
	                    depth_t depth = 0)
	{
		return updateSemantic(derived().toCode(x, y, z, depth), unary_op, propagate);
	}

	Node updateSemantic(coord_t x, coord_t y, coord_t z,
	                    std::invocable<Index, Semantic> auto binary_op,
	                    bool propagate = true, depth_t depth = 0)
	{
		return updateSemantic(derived().toCode(x, y, z, depth), binary_op, propagate);
	}

	//
	// Propagation criteria
	//

	[[nodiscard]] constexpr SemanticPropagationCriteria semanticPropagationCriteria()
	    const noexcept
	{
		return prop_criteria_;
	}

	constexpr void setSemanticPropagationCriteria(SemanticPropagationCriteria prop_criteria,
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

	SemanticMap() { semantic_.emplace_back(); }

	SemanticMap(SemanticMap const& other) = default;

	SemanticMap(SemanticMap&& other) = default;

	template <class Derived2>
	SemanticMap(SemanticMap<Derived2, N> const& other)
	    : semantic_(other.semantic_), prop_criteria_(other.prop_criteria_)
	{
	}

	template <class Derived2>
	SemanticMap(SemanticMap<Derived2, N>&& other)
	    : semantic_(std::move(other.semantic_))
	    , prop_criteria_(std::move(other.prop_criteria_))
	{
	}

	//
	// Destructor
	//

	~SemanticMap() = default;

	//
	// Assignment operator
	//

	SemanticMap& operator=(SemanticMap const& rhs) = default;

	SemanticMap& operator=(SemanticMap&& rhs) = default;

	template <class Derived2>
	SemanticMap& operator=(SemanticMap<Derived2, N> const& rhs)
	{
		semantic_      = rhs.semantic_;
		prop_criteria_ = rhs.prop_criteria_;
		return *this;
	}

	template <class Derived2>
	SemanticMap& operator=(SemanticMap<Derived2, N>&& rhs)
	{
		semantic_      = std::move(rhs.semantic_);
		prop_criteria_ = std::move(rhs.prop_criteria_);
		return *this;
	}

	//
	// Swap
	//

	void swap(SemanticMap& other) noexcept
	{
		std::swap(semantic_, other.semantic_);
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
		semantic_.emplace_back();
		semantic_.back().fill(semantic_[node.pos][node.offset]);
	}

	//
	// Resize
	//

	void resize(std::size_t count) { semantic_.resize(count, Semantic(0, 0)); }

	//
	// Reserve
	//

	void reserveImpl(std::size_t new_cap) { semantic_.reserve(new_cap); }

	//
	// Initialize root
	//

	void initRoot()
	{
		auto node                              = derived().rootIndex();
		semantic_[node.pos][node.offset].label = 0;
		semantic_[node.pos][node.offset].value = 0;
	}

	//
	// Fill
	//

	void fill(Index node, pos_t children)
	{
		semantic_[children].fill(semantic_[node.pos][node.offset]);
	}

	//
	// Clear
	//

	void clearImpl() { semantic_.assign(1, Semantic(0, 0)); }

	void clearImpl(pos_t nodes) {}

	//
	// Shrink to fit
	//

	void shrinkToFitImpl() { semantic_.shrink_to_fit(); }

	//
	// Update node
	//

	void updateNode(Index node, pos_t children)
	{
		switch (semanticPropagationCriteria()) {
			case SemanticPropagationCriteria::MIN: {
				Semantic s(0, std::numeric_limits<value_t>::max());
				for (auto e : semantic_[children]) {
					s.label |= e.label;
					s.value = std::min(s.value, e.value);
				}
				semantic_[node.pos][node.offset] = s;
			}
				return;
			case SemanticPropagationCriteria::MAX: {
				Semantic s(0, std::numeric_limits<value_t>::lowest());
				for (auto e : semantic_[children]) {
					s.label |= e.label;
					s.value = std::max(s.value, e.value);
				}
				semantic_[node.pos][node.offset] = s;
			}
				return;
			case SemanticPropagationCriteria::NONE: return;
		}
	}

	//
	// Is prunable
	//

	[[nodiscard]] bool isPrunable(pos_t nodes) const
	{
		return std::all_of(std::cbegin(semantic_[nodes]) + 1, std::cend(semantic_[nodes]),
		                   [a = semantic_[nodes].front()](auto b) { return a == b; });
	}

	//
	// Memory node block
	//

	[[nodiscard]] static constexpr std::size_t memoryNodeBlock() noexcept
	{
		return N * sizeof(Semantic);
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr MapType mapType() noexcept { return MapType::SEMANTIC; }

	constexpr std::size_t serializedSize(std::ranges::input_range auto r) const
	{
		return std::ranges::size(r) * memoryNodeBlock();
	}

	void readNodes(ReadBuffer& in, std::ranges::input_range auto r)
	{
		for (auto const [pos, offsets] : r) {
			if (offsets.all()) {
				in.read(semantic_[pos].data(), memoryNodeBlock());
			} else {
				DataBlock<Semantic, N> semantic;
				in.read(semantic.data(), memoryNodeBlock());
				for (offset_t i{}; N != i; ++i) {
					semantic_[pos][i] = offsets[i] ? semantic[i] : semantic_[pos][i];
				}
			}
		}
	}

	void writeNodes(WriteBuffer& out, std::ranges::input_range auto r) const
	{
		for (auto pos : r) {
			out.write(semantic_[pos].data(), memoryNodeBlock());
		}
	}

	//
	// Dot file info
	//

	std::ostream& dotFileInfo(std::ostream& out, Index node) const
	{
		return out << "Semantic: " << semantic_[node.pos][node.offset];
	}

 protected:
	Container<DataBlock<Semantic, N>> semantic_;

	// Propagation criteria
	SemanticPropagationCriteria prop_criteria_ = SemanticPropagationCriteria::MAX;

	template <class Derived2, std::size_t N2>
	friend class SemanticMap;
};

//
// Concepts
//

template <class Map>
concept IsSemanticMap = IsMapType<Map, MapType::SEMANTIC>;
}  // namespace ufo

#endif  // UFO_MAP_SEMANTIC_MAP_HPP