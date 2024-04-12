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

#ifndef UFO_MAP_OCCUPANCY_MAP_HPP
#define UFO_MAP_OCCUPANCY_MAP_HPP

// UFO
#include <ufo/geometry/aabb.hpp>
#include <ufo/map/io.hpp>
#include <ufo/map/node.hpp>
#include <ufo/map/point.hpp>
#include <ufo/map/predicate/satisfies.hpp>
#include <ufo/map/types.hpp>
#include <ufo/math/logit.hpp>
#include <ufo/util/type_traits.hpp>

// STL
#include <cstdint>
#include <iostream>
#include <limits>
#include <type_traits>

namespace ufo
{
template <class Derived, std::size_t N>
class OccupancyMap
{
 public:
	//
	// Get occupancy logit
	//

	[[nodiscard]] logit_t occupancyLogit(Index node) const
	{
		return logit_[node.pos][node.offset];
	}

	[[nodiscard]] logit_t occupancyLogit(Node node) const
	{
		return occupancyLogit(derived().index(node));
	}

	[[nodiscard]] logit_t occupancyLogit(Code code) const
	{
		return occupancyLogit(derived().index(code));
	}

	[[nodiscard]] logit_t occupancyLogit(Key key) const
	{
		return occupancyLogit(derived().index(key));
	}

	[[nodiscard]] logit_t occupancyLogit(Point coord, depth_t depth = 0) const
	{
		return occupancyLogit(derived().index(coord, depth));
	}

	[[nodiscard]] logit_t occupancyLogit(coord_t x, coord_t y, coord_t z,
	                                     depth_t depth = 0) const
	{
		return occupancyLogit(derived().index(x, y, z, depth));
	}

	//
	// Get occupancy
	//

	[[nodiscard]] occupancy_t occupancy(Index node) const
	{
		return toOccupancyProbability(occupancyLogit(node));
	}

	[[nodiscard]] occupancy_t occupancy(Node node) const
	{
		return occupancy(derived().index(node));
	}

	[[nodiscard]] occupancy_t occupancy(Code code) const
	{
		return occupancy(derived().index(code));
	}

	[[nodiscard]] occupancy_t occupancy(Key key) const
	{
		return occupancy(derived().index(key));
	}

	[[nodiscard]] occupancy_t occupancy(Point coord, depth_t depth = 0) const
	{
		return occupancy(derived().index(coord, depth));
	}

	[[nodiscard]] occupancy_t occupancy(coord_t x, coord_t y, coord_t z,
	                                    depth_t depth = 0) const
	{
		return occupancy(derived().index(x, y, z, depth));
	}

	//
	// Set occupancy logit
	//

	void setOccupancyLogit(Index node, logit_t value)
	{
		BitSet<8> u = isUnknown(value) ? -1 : 0;
		BitSet<8> f = isFree(value) ? -1 : 0;
		BitSet<8> o = isOccupied(value) ? -1 : 0;

		derived().apply(
		    node,
		    [this, value, u = u.any(), f = f.any(), o = o.any()](Index node) {
			    logit_[node.pos][node.offset]             = value;
			    contains_unknown_[node.pos][node.offset]  = u;
			    contains_free_[node.pos][node.offset]     = f;
			    contains_occupied_[node.pos][node.offset] = o;
		    },
		    [this, value, u, f, o](pos_t pos) {
			    logit_[pos].fill(value);
			    contains_unknown_[pos]  = u;
			    contains_free_[pos]     = f;
			    contains_occupied_[pos] = o;
		    });
	}

	Node setOccupancyLogit(Node node, logit_t value, bool propagate = true)
	{
		BitSet<8> u = isUnknown(value) ? -1 : 0;
		BitSet<8> f = isFree(value) ? -1 : 0;
		BitSet<8> o = isOccupied(value) ? -1 : 0;

		return derived().apply(
		    node,
		    [this, value, u = u.any(), f = f.any(), o = o.any()](Index node) {
			    logit_[node.pos][node.offset]             = value;
			    contains_unknown_[node.pos][node.offset]  = u;
			    contains_free_[node.pos][node.offset]     = f;
			    contains_occupied_[node.pos][node.offset] = o;
		    },
		    [this, value, u, f, o](pos_t pos) {
			    logit_[pos].fill(value);
			    contains_unknown_[pos]  = u;
			    contains_free_[pos]     = f;
			    contains_occupied_[pos] = o;
		    },
		    propagate);
	}

	Node setOccupancyLogit(Code code, logit_t value, bool propagate = true)
	{
		BitSet<8> u = isUnknown(value) ? -1 : 0;
		BitSet<8> f = isFree(value) ? -1 : 0;
		BitSet<8> o = isOccupied(value) ? -1 : 0;

		return derived().apply(
		    code,
		    [this, value, u = u.any(), f = f.any(), o = o.any()](Index node) {
			    logit_[node.pos][node.offset]             = value;
			    contains_unknown_[node.pos][node.offset]  = u;
			    contains_free_[node.pos][node.offset]     = f;
			    contains_occupied_[node.pos][node.offset] = o;
		    },
		    [this, value, u, f, o](pos_t pos) {
			    logit_[pos].fill(value);
			    contains_unknown_[pos]  = u;
			    contains_free_[pos]     = f;
			    contains_occupied_[pos] = o;
		    },
		    propagate);
	}

	Node setOccupancyLogit(Key key, logit_t value, bool propagate = true)
	{
		return setOccupancyLogit(derived().toCode(key), value, propagate);
	}

	Node setOccupancyLogit(Point coord, logit_t value, bool propagate = true,
	                       depth_t depth = 0)
	{
		return setOccupancyLogit(derived().toCode(coord, depth), value, propagate);
	}

	Node setOccupancyLogit(coord_t x, coord_t y, coord_t z, logit_t value,
	                       bool propagate = true, depth_t depth = 0)
	{
		return setOccupancyLogit(derived().toCode(x, y, z, depth), value, propagate);
	}

	//
	// Set occupancy
	//

	void setOccupancy(Index node, occupancy_t value)
	{
		return setOccupancyLogit(node, toOccupancyLogit(value));
	}

	Node setOccupancy(Node node, occupancy_t value, bool propagate = true)
	{
		return setOccupancyLogit(node, toOccupancyLogit(value), propagate);
	}

	Node setOccupancy(Code code, occupancy_t value, bool propagate = true)
	{
		return setOccupancyLogit(code, toOccupancyLogit(value), propagate);
	}

	Node setOccupancy(Key key, occupancy_t value, bool propagate = true)
	{
		return setOccupancy(derived().toCode(key), value, propagate);
	}

	Node setOccupancy(Point coord, occupancy_t value, bool propagate = true,
	                  depth_t depth = 0)
	{
		return setOccupancy(derived().toCode(coord, depth), value, propagate);
	}

	Node setOccupancy(coord_t x, coord_t y, coord_t z, occupancy_t value,
	                  bool propagate = true, depth_t depth = 0)
	{
		return setOccupancy(derived().toCode(x, y, z, depth), value, propagate);
	}

	//
	// Update occupancy logit
	//

	void updateOccupancyLogit(Index node, int change)
	{
		derived().apply(
		    node,
		    [this, change](Index node) {
			    logit_[node.pos][node.offset] =
			        std::clamp(logit_[node.pos][node.offset] + change,
			                   static_cast<int>(std::numeric_limits<logit_t>::min()),
			                   static_cast<int>(std::numeric_limits<logit_t>::max()));

			    contains_unknown_[node.pos][node.offset] =
			        isUnknown(logit_[node.pos][node.offset]);
			    contains_free_[node.pos][node.offset] = isFree(logit_[node.pos][node.offset]);
			    contains_occupied_[node.pos][node.offset] =
			        isOccupied(logit_[node.pos][node.offset]);
		    },
		    [this, change](pos_t pos) {
			    for (offset_t i{}; N != i; ++i) {
				    logit_[pos][i] =
				        std::clamp(logit_[pos][i] + change,
				                   static_cast<int>(std::numeric_limits<logit_t>::min()),
				                   static_cast<int>(std::numeric_limits<logit_t>::max()));

				    contains_unknown_[pos][i]  = isUnknown(logit_[pos][i]);
				    contains_free_[pos][i]     = isFree(logit_[pos][i]);
				    contains_occupied_[pos][i] = isOccupied(logit_[pos][i]);
			    }
		    });
	}

	void updateOccupancyLogit(Index node, std::invocable<logit_t> auto unary_op)
	{
		derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    logit_[node.pos][node.offset] = unary_op(logit_[node.pos][node.offset]);

			    contains_unknown_[node.pos][node.offset] =
			        isUnknown(logit_[node.pos][node.offset]);
			    contains_free_[node.pos][node.offset] = isFree(logit_[node.pos][node.offset]);
			    contains_occupied_[node.pos][node.offset] =
			        isOccupied(logit_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (offset_t i{}; N != i; ++i) {
				    logit_[pos][i] = unary_op(logit_[pos][i]);

				    contains_unknown_[pos][i]  = isUnknown(logit_[pos][i]);
				    contains_free_[pos][i]     = isFree(logit_[pos][i]);
				    contains_occupied_[pos][i] = isOccupied(logit_[pos][i]);
			    }
		    });
	}

	void updateOccupancyLogit(Index node, std::invocable<Index, logit_t> auto binary_op)
	{
		derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    logit_[node.pos][node.offset] = binary_op(node, logit_[node.pos][node.offset]);

			    contains_unknown_[node.pos][node.offset] =
			        isUnknown(logit_[node.pos][node.offset]);
			    contains_free_[node.pos][node.offset] = isFree(logit_[node.pos][node.offset]);
			    contains_occupied_[node.pos][node.offset] =
			        isOccupied(logit_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (offset_t i{}; N != i; ++i) {
				    logit_[pos][i] = binary_op(Index(pos, i), logit_[pos][i]);

				    contains_unknown_[pos][i]  = isUnknown(logit_[pos][i]);
				    contains_free_[pos][i]     = isFree(logit_[pos][i]);
				    contains_occupied_[pos][i] = isOccupied(logit_[pos][i]);
			    }
		    });
	}

	Node updateOccupancyLogit(Node node, int change, bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, change](Index node) {
			    logit_[node.pos][node.offset] =
			        std::clamp(logit_[node.pos][node.offset] + change,
			                   static_cast<int>(std::numeric_limits<logit_t>::min()),
			                   static_cast<int>(std::numeric_limits<logit_t>::max()));

			    contains_unknown_[node.pos][node.offset] =
			        isUnknown(logit_[node.pos][node.offset]);
			    contains_free_[node.pos][node.offset] = isFree(logit_[node.pos][node.offset]);
			    contains_occupied_[node.pos][node.offset] =
			        isOccupied(logit_[node.pos][node.offset]);
		    },
		    [this, change](pos_t pos) {
			    for (offset_t i{}; N != i; ++i) {
				    logit_[pos][i] =
				        std::clamp(logit_[pos][i] + change,
				                   static_cast<int>(std::numeric_limits<logit_t>::min()),
				                   static_cast<int>(std::numeric_limits<logit_t>::max()));

				    contains_unknown_[pos][i]  = isUnknown(logit_[pos][i]);
				    contains_free_[pos][i]     = isFree(logit_[pos][i]);
				    contains_occupied_[pos][i] = isOccupied(logit_[pos][i]);
			    }
		    },
		    propagate);
	}

	Node updateOccupancyLogit(Node node, std::invocable<logit_t> auto unary_op,
	                          bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    logit_[node.pos][node.offset] = unary_op(logit_[node.pos][node.offset]);

			    contains_unknown_[node.pos][node.offset] =
			        isUnknown(logit_[node.pos][node.offset]);
			    contains_free_[node.pos][node.offset] = isFree(logit_[node.pos][node.offset]);
			    contains_occupied_[node.pos][node.offset] =
			        isOccupied(logit_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (offset_t i{}; N != i; ++i) {
				    logit_[pos][i] = unary_op(logit_[pos][i]);

				    contains_unknown_[pos][i]  = isUnknown(logit_[pos][i]);
				    contains_free_[pos][i]     = isFree(logit_[pos][i]);
				    contains_occupied_[pos][i] = isOccupied(logit_[pos][i]);
			    }
		    },
		    propagate);
	}

	Node updateOccupancyLogit(Node node, std::invocable<Index, logit_t> auto binary_op,
	                          bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    logit_[node.pos][node.offset] = binary_op(node, logit_[node.pos][node.offset]);

			    contains_unknown_[node.pos][node.offset] =
			        isUnknown(logit_[node.pos][node.offset]);
			    contains_free_[node.pos][node.offset] = isFree(logit_[node.pos][node.offset]);
			    contains_occupied_[node.pos][node.offset] =
			        isOccupied(logit_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (offset_t i{}; N != i; ++i) {
				    logit_[pos][i] = binary_op(Index(pos, i), logit_[pos][i]);

				    contains_unknown_[pos][i]  = isUnknown(logit_[pos][i]);
				    contains_free_[pos][i]     = isFree(logit_[pos][i]);
				    contains_occupied_[pos][i] = isOccupied(logit_[pos][i]);
			    }
		    },
		    propagate);
	}

	Node updateOccupancyLogit(Code code, int change, bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, change](Index node) {
			    logit_[node.pos][node.offset] =
			        std::clamp(logit_[node.pos][node.offset] + change,
			                   static_cast<int>(std::numeric_limits<logit_t>::min()),
			                   static_cast<int>(std::numeric_limits<logit_t>::max()));

			    contains_unknown_[node.pos][node.offset] =
			        isUnknown(logit_[node.pos][node.offset]);
			    contains_free_[node.pos][node.offset] = isFree(logit_[node.pos][node.offset]);
			    contains_occupied_[node.pos][node.offset] =
			        isOccupied(logit_[node.pos][node.offset]);
		    },
		    [this, change](pos_t pos) {
			    for (offset_t i{}; N != i; ++i) {
				    logit_[pos][i] =
				        std::clamp(logit_[pos][i] + change,
				                   static_cast<int>(std::numeric_limits<logit_t>::min()),
				                   static_cast<int>(std::numeric_limits<logit_t>::max()));

				    contains_unknown_[pos][i]  = isUnknown(logit_[pos][i]);
				    contains_free_[pos][i]     = isFree(logit_[pos][i]);
				    contains_occupied_[pos][i] = isOccupied(logit_[pos][i]);
			    }
		    },
		    propagate);
	}

	Node updateOccupancyLogit(Code code, std::invocable<logit_t> auto unary_op,
	                          bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, unary_op](Index node) {
			    logit_[node.pos][node.offset] = unary_op(logit_[node.pos][node.offset]);

			    contains_unknown_[node.pos][node.offset] =
			        isUnknown(logit_[node.pos][node.offset]);
			    contains_free_[node.pos][node.offset] = isFree(logit_[node.pos][node.offset]);
			    contains_occupied_[node.pos][node.offset] =
			        isOccupied(logit_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (offset_t i{}; N != i; ++i) {
				    logit_[pos][i] = unary_op(logit_[pos][i]);

				    contains_unknown_[pos][i]  = isUnknown(logit_[pos][i]);
				    contains_free_[pos][i]     = isFree(logit_[pos][i]);
				    contains_occupied_[pos][i] = isOccupied(logit_[pos][i]);
			    }
		    },
		    propagate);
	}

	Node updateOccupancyLogit(Code code, std::invocable<Index, logit_t> auto binary_op,
	                          bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, binary_op](Index node) {
			    logit_[node.pos][node.offset] = binary_op(node, logit_[node.pos][node.offset]);

			    contains_unknown_[node.pos][node.offset] =
			        isUnknown(logit_[node.pos][node.offset]);
			    contains_free_[node.pos][node.offset] = isFree(logit_[node.pos][node.offset]);
			    contains_occupied_[node.pos][node.offset] =
			        isOccupied(logit_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (offset_t i{}; N != i; ++i) {
				    logit_[pos][i] = binary_op(Index(pos, i), logit_[pos][i]);

				    contains_unknown_[pos][i]  = isUnknown(logit_[pos][i]);
				    contains_free_[pos][i]     = isFree(logit_[pos][i]);
				    contains_occupied_[pos][i] = isOccupied(logit_[pos][i]);
			    }
		    },
		    propagate);
	}

	Node updateOccupancyLogit(Key key, int change, bool propagate = true)
	{
		return updateOccupancyLogit(derived().toCode(key), change, propagate);
	}

	Node updateOccupancyLogit(Key key, std::invocable<logit_t> auto unary_op,
	                          bool propagate = true)
	{
		return updateOccupancyLogit(derived().toCode(key), unary_op, propagate);
	}

	Node updateOccupancyLogit(Key key, std::invocable<Index, logit_t> auto binary_op,
	                          bool propagate = true)
	{
		return updateOccupancyLogit(derived().toCode(key), binary_op, propagate);
	}

	Node updateOccupancyLogit(Point coord, int change, bool propagate = true,
	                          depth_t depth = 0)
	{
		return updateOccupancyLogit(derived().toCode(coord, depth), change, propagate);
	}

	Node updateOccupancyLogit(Point coord, std::invocable<logit_t> auto unary_op,
	                          bool propagate = true, depth_t depth = 0)
	{
		return updateOccupancyLogit(derived().toCode(coord, depth), unary_op, propagate);
	}

	Node updateOccupancyLogit(Point coord, std::invocable<Index, logit_t> auto binary_op,
	                          bool propagate = true, depth_t depth = 0)
	{
		return updateOccupancyLogit(derived().toCode(coord, depth), binary_op, propagate);
	}

	Node updateOccupancyLogit(coord_t x, coord_t y, coord_t z, int change,
	                          bool propagate = true, depth_t depth = 0)
	{
		return updateOccupancyLogit(derived().toCode(x, y, z, depth), change, propagate);
	}

	Node updateOccupancyLogit(coord_t x, coord_t y, coord_t z,
	                          std::invocable<logit_t> auto unary_op, bool propagate = true,
	                          depth_t depth = 0)
	{
		return updateOccupancyLogit(derived().toCode(x, y, z, depth), unary_op, propagate);
	}

	Node updateOccupancyLogit(coord_t x, coord_t y, coord_t z,
	                          std::invocable<Index, logit_t> auto binary_op,
	                          bool propagate = true, depth_t depth = 0)
	{
		return updateOccupancyLogit(derived().toCode(x, y, z, depth), binary_op, propagate);
	}

	//
	// Update occupancy
	//

	void updateOccupancy(Index node, occupancy_t change)
	{
		updateOccupancyLogit(node, toOccupancyLogit(change));
	}

	void updateOccupancy(Index node, std::invocable<occupancy_t> auto unary_op)
	{
		derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    logit_[node.pos][node.offset] = toOccupancyLogit(
			        unary_op(toOccupancyProbability(logit_[node.pos][node.offset])));

			    contains_unknown_[node.pos][node.offset] =
			        isUnknown(logit_[node.pos][node.offset]);
			    contains_free_[node.pos][node.offset] = isFree(logit_[node.pos][node.offset]);
			    contains_occupied_[node.pos][node.offset] =
			        isOccupied(logit_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (offset_t i{}; N != i; ++i) {
				    logit_[pos][i] =
				        toOccupancyLogit(unary_op(toOccupancyProbability(logit_[pos][i])));

				    contains_unknown_[pos][i]  = isUnknown(logit_[pos][i]);
				    contains_free_[pos][i]     = isFree(logit_[pos][i]);
				    contains_occupied_[pos][i] = isOccupied(logit_[pos][i]);
			    }
		    });
	}

	void updateOccupancy(Index node, std::invocable<Index, occupancy_t> auto binary_op)
	{
		derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    logit_[node.pos][node.offset] = toOccupancyLogit(
			        binary_op(node, toOccupancyProbability(logit_[node.pos][node.offset])));

			    contains_unknown_[node.pos][node.offset] =
			        isUnknown(logit_[node.pos][node.offset]);
			    contains_free_[node.pos][node.offset] = isFree(logit_[node.pos][node.offset]);
			    contains_occupied_[node.pos][node.offset] =
			        isOccupied(logit_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (offset_t i{}; N != i; ++i) {
				    logit_[pos][i] = toOccupancyLogit(
				        binary_op(Index(pos, i), toOccupancyProbability(logit_[pos][i])));

				    contains_unknown_[pos][i]  = isUnknown(logit_[pos][i]);
				    contains_free_[pos][i]     = isFree(logit_[pos][i]);
				    contains_occupied_[pos][i] = isOccupied(logit_[pos][i]);
			    }
		    });
	}

	Node updateOccupancy(Node node, occupancy_t change, bool propagate = true)
	{
		return updateOccupancyLogit(node, toOccupancyLogit(change), propagate);
	}

	Node updateOccupancy(Node node, std::invocable<occupancy_t> auto unary_op,
	                     bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, unary_op](Index node) {
			    logit_[node.pos][node.offset] = toOccupancyLogit(
			        unary_op(toOccupancyProbability(logit_[node.pos][node.offset])));

			    contains_unknown_[node.pos][node.offset] =
			        isUnknown(logit_[node.pos][node.offset]);
			    contains_free_[node.pos][node.offset] = isFree(logit_[node.pos][node.offset]);
			    contains_occupied_[node.pos][node.offset] =
			        isOccupied(logit_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (offset_t i{}; N != i; ++i) {
				    logit_[pos][i] =
				        toOccupancyLogit(unary_op(toOccupancyProbability(logit_[pos][i])));

				    contains_unknown_[pos][i]  = isUnknown(logit_[pos][i]);
				    contains_free_[pos][i]     = isFree(logit_[pos][i]);
				    contains_occupied_[pos][i] = isOccupied(logit_[pos][i]);
			    }
		    },
		    propagate);
	}

	Node updateOccupancy(Node node, std::invocable<Index, occupancy_t> auto binary_op,
	                     bool propagate = true)
	{
		return derived().apply(
		    node,
		    [this, binary_op](Index node) {
			    logit_[node.pos][node.offset] = toOccupancyLogit(
			        binary_op(node, toOccupancyProbability(logit_[node.pos][node.offset])));

			    contains_unknown_[node.pos][node.offset] =
			        isUnknown(logit_[node.pos][node.offset]);
			    contains_free_[node.pos][node.offset] = isFree(logit_[node.pos][node.offset]);
			    contains_occupied_[node.pos][node.offset] =
			        isOccupied(logit_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (offset_t i{}; N != i; ++i) {
				    logit_[pos][i] = toOccupancyLogit(
				        binary_op(Index(pos, i), toOccupancyProbability(logit_[pos][i])));

				    contains_unknown_[pos][i]  = isUnknown(logit_[pos][i]);
				    contains_free_[pos][i]     = isFree(logit_[pos][i]);
				    contains_occupied_[pos][i] = isOccupied(logit_[pos][i]);
			    }
		    },
		    propagate);
	}

	Node updateOccupancy(Code code, occupancy_t change, bool propagate = true)
	{
		return updateOccupancyLogit(code, toOccupancyLogit(change), propagate);
	}

	Node updateOccupancy(Code code, std::invocable<occupancy_t> auto unary_op,
	                     bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, unary_op](Index node) {
			    logit_[node.pos][node.offset] = toOccupancyLogit(
			        unary_op(toOccupancyProbability(logit_[node.pos][node.offset])));

			    contains_unknown_[node.pos][node.offset] =
			        isUnknown(logit_[node.pos][node.offset]);
			    contains_free_[node.pos][node.offset] = isFree(logit_[node.pos][node.offset]);
			    contains_occupied_[node.pos][node.offset] =
			        isOccupied(logit_[node.pos][node.offset]);
		    },
		    [this, unary_op](pos_t pos) {
			    for (offset_t i{}; N != i; ++i) {
				    logit_[pos][i] =
				        toOccupancyLogit(unary_op(toOccupancyProbability(logit_[pos][i])));

				    contains_unknown_[pos][i]  = isUnknown(logit_[pos][i]);
				    contains_free_[pos][i]     = isFree(logit_[pos][i]);
				    contains_occupied_[pos][i] = isOccupied(logit_[pos][i]);
			    }
		    },
		    propagate);
	}

	Node updateOccupancy(Code code, std::invocable<Index, occupancy_t> auto binary_op,
	                     bool propagate = true)
	{
		return derived().apply(
		    code,
		    [this, binary_op](Index node) {
			    logit_[node.pos][node.offset] = toOccupancyLogit(
			        binary_op(node, toOccupancyProbability(logit_[node.pos][node.offset])));

			    contains_unknown_[node.pos][node.offset] =
			        isUnknown(logit_[node.pos][node.offset]);
			    contains_free_[node.pos][node.offset] = isFree(logit_[node.pos][node.offset]);
			    contains_occupied_[node.pos][node.offset] =
			        isOccupied(logit_[node.pos][node.offset]);
		    },
		    [this, binary_op](pos_t pos) {
			    for (offset_t i{}; N != i; ++i) {
				    logit_[pos][i] = toOccupancyLogit(
				        binary_op(Index(pos, i), toOccupancyProbability(logit_[pos][i])));

				    contains_unknown_[pos][i]  = isUnknown(logit_[pos][i]);
				    contains_free_[pos][i]     = isFree(logit_[pos][i]);
				    contains_occupied_[pos][i] = isOccupied(logit_[pos][i]);
			    }
		    },
		    propagate);
	}

	Node updateOccupancy(Key key, occupancy_t change, bool propagate = true)
	{
		return updateOccupancy(derived().toCode(key), change, propagate);
	}

	Node updateOccupancy(Key key, std::invocable<occupancy_t> auto unary_op,
	                     bool propagate = true)
	{
		return updateOccupancy(derived().toCode(key), unary_op, propagate);
	}

	Node updateOccupancy(Key key, std::invocable<Index, occupancy_t> auto binary_op,
	                     bool propagate = true)
	{
		return updateOccupancy(derived().toCode(key), binary_op, propagate);
	}

	Node updateOccupancy(Point coord, occupancy_t change, bool propagate = true,
	                     depth_t depth = 0)
	{
		return updateOccupancy(derived().toCode(coord, depth), change, propagate);
	}

	Node updateOccupancy(Point coord, std::invocable<occupancy_t> auto unary_op,
	                     bool propagate = true, depth_t depth = 0)
	{
		return updateOccupancy(derived().toCode(coord, depth), unary_op, propagate);
	}

	Node updateOccupancy(Point coord, std::invocable<Index, occupancy_t> auto binary_op,
	                     bool propagate = true, depth_t depth = 0)
	{
		return updateOccupancy(derived().toCode(coord, depth), binary_op, propagate);
	}

	Node updateOccupancy(coord_t x, coord_t y, coord_t z, occupancy_t change,
	                     bool propagate = true, depth_t depth = 0)
	{
		return updateOccupancy(derived().toCode(x, y, z, depth), change, propagate);
	}

	Node updateOccupancy(coord_t x, coord_t y, coord_t z,
	                     std::invocable<occupancy_t> auto unary_op, bool propagate = true,
	                     depth_t depth = 0)
	{
		return updateOccupancy(derived().toCode(x, y, z, depth), unary_op, propagate);
	}

	Node updateOccupancy(coord_t x, coord_t y, coord_t z,
	                     std::invocable<Index, occupancy_t> auto binary_op,
	                     bool propagate = true, depth_t depth = 0)
	{
		return updateOccupancy(derived().toCode(x, y, z, depth), binary_op, propagate);
	}

	//
	// Get occupancy state
	//

	[[nodiscard]] OccupancyState occupancyState(Index node) const
	{
		auto occ = occupancyLogit(node);
		if (isUnknown(occ)) {
			return OccupancyState::UNKNOWN;
		} else if (isFree(occ)) {
			return OccupancyState::FREE;
		} else {
			return OccupancyState::OCCUPIED;
		}
	}

	[[nodiscard]] OccupancyState occupancyState(Node node) const
	{
		return occupancyState(derived().index(node));
	}

	[[nodiscard]] OccupancyState occupancyState(Code code) const
	{
		return occupancyState(derived().index(code));
	}

	[[nodiscard]] OccupancyState occupancyState(Key key) const
	{
		return occupancyState(derived().index(key));
	}

	[[nodiscard]] OccupancyState occupancyState(Point coord, depth_t depth = 0) const
	{
		return occupancyState(derived().index(coord, depth));
	}

	[[nodiscard]] OccupancyState occupancyState(coord_t x, coord_t y, coord_t z,
	                                            depth_t depth = 0) const
	{
		return occupancyState(derived().index(x, y, z, depth));
	}

	//
	// Contains occupancy state
	//

	[[nodiscard]] bool containsOccupancyState(Index node, OccupancyState state) const
	{
		switch (state) {
			case OccupancyState::UNKNOWN: return containsUnknown(node);
			case OccupancyState::FREE: return containsFree(node);
			case OccupancyState::OCCUPIED: return containsOccupied(node);
		}
	}

	[[nodiscard]] bool containsOccupancyState(Node node, OccupancyState state) const
	{
		return containsOccupancyState(derived().index(node), state);
	}

	[[nodiscard]] bool containsOccupancyState(Code code, OccupancyState state) const
	{
		return containsOccupancyState(derived().index(code), state);
	}

	[[nodiscard]] bool containsOccupancyState(Key key, OccupancyState state) const
	{
		return containsOccupancyState(derived().index(key), state);
	}

	[[nodiscard]] bool containsOccupancyState(Point coord, OccupancyState state,
	                                          depth_t depth = 0) const
	{
		return containsOccupancyState(derived().index(coord, depth), state);
	}

	[[nodiscard]] bool containsOccupancyState(coord_t x, coord_t y, coord_t z,
	                                          OccupancyState state, depth_t depth = 0) const
	{
		return containsOccupancyState(derived().index(x, y, z, depth), state);
	}

	//
	// Is unknown
	//

	[[nodiscard]] bool isUnknown(Index node) const
	{
		return isUnknown(occupancyLogit(node));
	}

	[[nodiscard]] bool isUnknown(Node node) const
	{
		return isUnknown(derived().index(node));
	}

	[[nodiscard]] bool isUnknown(Code code) const
	{
		return isUnknown(derived().index(code));
	}

	[[nodiscard]] bool isUnknown(Key key) const { return isUnknown(derived().index(key)); }

	[[nodiscard]] bool isUnknown(Point coord, depth_t depth = 0) const
	{
		return isUnknown(derived().index(coord, depth));
	}

	[[nodiscard]] bool isUnknown(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return isUnknown(derived().index(x, y, z, depth));
	}

	//
	// Is free
	//

	[[nodiscard]] bool isFree(Index node) const { return isFree(occupancyLogit(node)); }

	[[nodiscard]] bool isFree(Node node) const { return isFree(derived().index(node)); }

	[[nodiscard]] bool isFree(Code code) const { return isFree(derived().index(code)); }

	[[nodiscard]] bool isFree(Key key) const { return isFree(derived().index(key)); }

	[[nodiscard]] bool isFree(Point coord, depth_t depth = 0) const
	{
		return isFree(derived().index(coord, depth));
	}

	[[nodiscard]] bool isFree(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return isFree(derived().index(x, y, z, depth));
	}

	//
	// Is occupied
	//

	[[nodiscard]] bool isOccupied(Index node) const
	{
		return isOccupied(occupancyLogit(node));
	}

	[[nodiscard]] bool isOccupied(Node node) const
	{
		return isOccupied(derived().index(node));
	}

	[[nodiscard]] bool isOccupied(Code code) const
	{
		return isOccupied(derived().index(code));
	}

	[[nodiscard]] bool isOccupied(Key key) const
	{
		return isOccupied(derived().index(key));
	}

	[[nodiscard]] bool isOccupied(Point coord, depth_t depth = 0) const
	{
		return isOccupied(derived().index(coord, depth));
	}

	[[nodiscard]] bool isOccupied(coord_t x, coord_t y, coord_t z, depth_t depth = 0) const
	{
		return isOccupied(derived().index(x, y, z, depth));
	}

	//
	// Contains unknown
	//

	[[nodiscard]] bool containsUnknown(Index node) const
	{
		return contains_unknown_[node.pos][node.offset];
		// return derived().isLeaf(node) ? isUnknown(node)
		//                               : contains_unknown_[node.pos][node.offset];
	}

	[[nodiscard]] bool containsUnknown(Node node) const
	{
		return containsUnknown(derived().index(node));
	}

	[[nodiscard]] bool containsUnknown(Code code) const
	{
		return containsUnknown(derived().index(code));
	}

	[[nodiscard]] bool containsUnknown(Key key) const
	{
		return containsUnknown(derived().index(key));
	}

	[[nodiscard]] bool containsUnknown(Point coord, depth_t depth = 0) const
	{
		return containsUnknown(derived().index(coord, depth));
	}

	[[nodiscard]] bool containsUnknown(coord_t x, coord_t y, coord_t z,
	                                   depth_t depth = 0) const
	{
		return containsUnknown(derived().index(x, y, z, depth));
	}

	//
	// Contains free
	//

	[[nodiscard]] bool containsFree(Index node) const
	{
		return contains_free_[node.pos][node.offset];
		// return derived().isLeaf(node) ? isFree(node) :
		// contains_free_[node.pos][node.offset];
	}

	[[nodiscard]] bool containsFree(Node node) const
	{
		return containsFree(derived().index(node));
	}

	[[nodiscard]] bool containsFree(Code code) const
	{
		return containsFree(derived().index(code));
	}

	[[nodiscard]] bool containsFree(Key key) const
	{
		return containsFree(derived().index(key));
	}

	[[nodiscard]] bool containsFree(Point coord, depth_t depth = 0) const
	{
		return containsFree(derived().index(coord, depth));
	}

	[[nodiscard]] bool containsFree(coord_t x, coord_t y, coord_t z,
	                                depth_t depth = 0) const
	{
		return containsFree(derived().index(x, y, z, depth));
	}

	//
	// Contains occupied
	//

	[[nodiscard]] bool containsOccupied(Index node) const
	{
		return contains_occupied_[node.pos][node.offset];
		// return derived().isLeaf(node) ? isOccupied(node)
		//                               : contains_occupied_[node.pos][node.offset];
	}

	[[nodiscard]] bool containsOccupied(Node node) const
	{
		return containsOccupied(derived().index(node));
	}

	[[nodiscard]] bool containsOccupied(Code code) const
	{
		return containsOccupied(derived().index(code));
	}

	[[nodiscard]] bool containsOccupied(Key key) const
	{
		return containsOccupied(derived().index(key));
	}

	[[nodiscard]] bool containsOccupied(Point coord, depth_t depth = 0) const
	{
		return containsOccupied(derived().index(coord, depth));
	}

	[[nodiscard]] bool containsOccupied(coord_t x, coord_t y, coord_t z,
	                                    depth_t depth = 0) const
	{
		return containsOccupied(derived().index(x, y, z, depth));
	}

	//
	// Sensor model
	//

	[[nodiscard]] constexpr occupancy_t occupancyClampingThres() const noexcept
	{
		return probability(occupancyClampingThresLogit());
	}

	[[nodiscard]] constexpr double occupancyClampingThresLogit() const noexcept
	{
		return clamping_thres_logit_;
	}

	[[nodiscard]] constexpr occupancy_t occupiedThres() const noexcept
	{
		return toOccupancyProbability(occupiedThresLogit());
	}

	[[nodiscard]] constexpr logit_t occupiedThresLogit() const noexcept
	{
		return occupied_thres_logit_;
	}

	[[nodiscard]] constexpr occupancy_t freeThres() const noexcept
	{
		return toOccupancyProbability(freeThresLogit());
	}

	[[nodiscard]] constexpr logit_t freeThresLogit() const noexcept
	{
		return free_thres_logit_;
	}

	//
	// Probability <-> logit
	//

	[[nodiscard]] constexpr logit_t toOccupancyLogit(occupancy_t probability) const
	{
		return logit<logit_t>(static_cast<double>(probability),
		                      -occupancyClampingThresLogit(), occupancyClampingThresLogit());
	}

	template <class logit_t>
	[[nodiscard]] constexpr occupancy_t toOccupancyProbability(logit_t logit) const
	{
		if constexpr (std::is_floating_point_v<logit_t>) {
			return probability(logit);
		} else {
			return probability(logit, -occupancyClampingThresLogit(),
			                   occupancyClampingThresLogit());
		}
	}

	template <class occupancy_t>
	[[nodiscard]] constexpr logit_t toOccupancyChangeLogit(occupancy_t probability) const
	{
		if constexpr (std::is_floating_point_v<logit_t>) {
			return logit(probability);
		} else {
			return logitChangeValue<logit_t>(static_cast<double>(probability),
			                                 -occupancyClampingThresLogit(),
			                                 occupancyClampingThresLogit());
		}
	}

	//
	// Set sensor model
	//

	void setOccupancyThres(occupancy_t occupied_thres, occupancy_t free_thres,
	                       bool propagate = true)
	{
		// FIXME: Should add a warning that these are very computational expensive to
		// call since the whole tree has to be updated

		setOccupancyThresLogit(toOccupancyLogit(occupied_thres), toOccupancyLogit(free_thres),
		                       propagate);
	}

	// FIXME: Look at
	void setOccupancyThresLogit(logit_t occupied_thres, logit_t free_thres,
	                            bool propagate = true)
	{
		// FIXME: Should add a warning that these are very computational expensive to
		// call since the whole tree has to be updated

		occupied_thres_logit_ = occupied_thres;
		free_thres_logit_     = free_thres;

		derived().setModified();

		if (propagate) {
			derived().propagateModified();
		}
	}

	void setOccupancyClampingThres(occupancy_t probability)
	{
		clamping_thres_logit_ = logit(probability);
	}

	//
	// Propagation criteria
	//

	[[nodiscard]] constexpr PropagationCriteria occupancyPropagationCriteria()
	    const noexcept
	{
		return prop_criteria_;
	}

	constexpr void setOccupancyPropagationCriteria(
	    PropagationCriteria occupancy_prop_criteria, bool propagate = true) noexcept
	{
		if (prop_criteria_ == occupancy_prop_criteria) {
			return;
		}

		prop_criteria_ = occupancy_prop_criteria;

		// Set all inner nodes to modified
		// FIXME: Possible to optimize this to only set the ones with children
		derived().setModified();

		if (propagate) {
			derived().propagateModified();
		}
	}

 protected:
	//
	// Constructors
	//

	OccupancyMap()
	{
		logit_.emplace_back();
		contains_unknown_.emplace_back();
		contains_free_.emplace_back();
		contains_occupied_.emplace_back();

		contains_unknown_.front()  = isUnknown(0) ? -1 : 0;
		contains_free_.front()     = isFree(0) ? -1 : 0;
		contains_occupied_.front() = isOccupied(0) ? -1 : 0;
	}

	OccupancyMap(OccupancyMap const& other) = default;

	OccupancyMap(OccupancyMap&& other) = default;

	template <class Derived2>
	OccupancyMap(OccupancyMap<Derived2, N> const& other)
	    : logit_(other.logit_)
	    , contains_unknown_(other.contains_unknown_)
	    , contains_free_(other.contains_free_)
	    , contains_occupied_(other.contains_occupied_)
	    , clamping_thres_logit_(other.clamping_thres_logit_)
	    , occupied_thres_logit_(other.occupied_thres_logit_)
	    , free_thres_logit_(other.free_thres_logit_)
	    , prop_criteria_(other.prop_criteria_)
	{
	}

	template <class Derived2>
	OccupancyMap(OccupancyMap<Derived2, N>&& other)
	    : logit_(std::move(other.logit_))
	    , contains_unknown_(std::move(other.contains_unknown_))
	    , contains_free_(std::move(other.contains_free_))
	    , contains_occupied_(std::move(other.contains_occupied_))
	    , clamping_thres_logit_(std::move(other.clamping_thres_logit_))
	    , occupied_thres_logit_(std::move(other.occupied_thres_logit_))
	    , free_thres_logit_(std::move(other.free_thres_logit_))
	    , prop_criteria_(std::move(other.prop_criteria_))
	{
	}

	//
	// Destructor
	//

	~OccupancyMap() = default;

	//
	// Assignment operator
	//

	OccupancyMap& operator=(OccupancyMap const& rhs) = default;

	OccupancyMap& operator=(OccupancyMap&& rhs) = default;

	template <class Derived2>
	OccupancyMap& operator=(OccupancyMap<Derived2, N> const& rhs)
	{
		logit_                = rhs.logit_;
		contains_unknown_     = rhs.contains_unknown_;
		contains_free_        = rhs.contains_free_;
		contains_occupied_    = rhs.contains_occupied_;
		clamping_thres_logit_ = rhs.clamping_thres_logit_;
		occupied_thres_logit_ = rhs.occupied_thres_logit_;
		free_thres_logit_     = rhs.free_thres_logit_;
		prop_criteria_        = rhs.prop_criteria_;
		return *this;
	}

	template <class Derived2>
	OccupancyMap& operator=(OccupancyMap<Derived2, N>&& rhs)
	{
		logit_                = std::move(rhs.logit_);
		contains_unknown_     = std::move(rhs.contains_unknown_);
		contains_free_        = std::move(rhs.contains_free_);
		contains_occupied_    = std::move(rhs.contains_occupied_);
		clamping_thres_logit_ = std::move(rhs.clamping_thres_logit_);
		occupied_thres_logit_ = std::move(rhs.occupied_thres_logit_);
		free_thres_logit_     = std::move(rhs.free_thres_logit_);
		prop_criteria_        = std::move(rhs.prop_criteria_);
		return *this;
	}

	//
	// Swap
	//

	void swap(OccupancyMap& other) noexcept
	{
		std::swap(logit_, other.logit_);
		std::swap(contains_unknown_, other.contains_unknown_);
		std::swap(contains_free_, other.contains_free_);
		std::swap(contains_occupied_, other.contains_occupied_);
		std::swap(clamping_thres_logit_, other.clamping_thres_logit_);
		std::swap(occupied_thres_logit_, other.occupied_thres_logit_);
		std::swap(free_thres_logit_, other.free_thres_logit_);
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
		logit_.emplace_back();
		contains_unknown_.emplace_back();
		contains_free_.emplace_back();
		contains_occupied_.emplace_back();

		logit_.back().fill(logit_[node.pos][node.offset]);
		contains_unknown_.back()  = isUnknown(logit_[node.pos][node.offset]) ? -1 : 0;
		contains_free_.back()     = isFree(logit_[node.pos][node.offset]) ? -1 : 0;
		contains_occupied_.back() = isOccupied(logit_[node.pos][node.offset]) ? -1 : 0;
	}

	//
	// Resize
	//

	void resize(std::size_t count)
	{
		// TODO: Implement
		logit_.resize(count);
		contains_unknown_.resize(count);
		contains_free_.resize(count);
		contains_occupied_.resize(count);
	}

	//
	// Reserve
	//

	void reserveImpl(std::size_t new_cap)
	{
		logit_.reserve(new_cap);
		contains_unknown_.reserve(new_cap);
		contains_free_.reserve(new_cap);
		contains_occupied_.reserve(new_cap);
	}

	//
	// Initialize root
	//

	void initRoot()
	{
		auto node                                 = derived().rootIndex();
		logit_[node.pos][node.offset]             = 0;
		contains_unknown_[node.pos][node.offset]  = isUnknown(0);
		contains_free_[node.pos][node.offset]     = isFree(0);
		contains_occupied_[node.pos][node.offset] = isOccupied(0);
	}

	//
	// Fill
	//

	void fill(Index node, pos_t children)
	{
		logit_[children].fill(logit_[node.pos][node.offset]);
		contains_unknown_[children]  = isUnknown(logit_[node.pos][node.offset]) ? -1 : 0;
		contains_free_[children]     = isFree(logit_[node.pos][node.offset]) ? -1 : 0;
		contains_occupied_[children] = isOccupied(logit_[node.pos][node.offset]) ? -1 : 0;
	}

	//
	// Clear
	//

	void clearImpl()
	{
		logit_.resize(1);
		contains_unknown_.resize(1);
		contains_free_.resize(1);
		contains_occupied_.resize(1);
	}

	void clearImpl(pos_t nodes) {}

	//
	// Shrink to fit
	//

	void shrinkToFit()
	{
		logit_.shrink_to_fit();
		contains_unknown_.shrink_to_fit();
		contains_free_.shrink_to_fit();
		contains_occupied_.shrink_to_fit();
	}

	//
	// Update node
	//

	void updateNode(Index node, pos_t children)
	{
		switch (occupancyPropagationCriteria()) {
			case PropagationCriteria::MIN:
				logit_[node.pos][node.offset] = std::ranges::min(logit_[children]);
				break;
			case PropagationCriteria::MAX:
				logit_[node.pos][node.offset] = std::ranges::max(logit_[children]);
				break;
			case PropagationCriteria::MEAN:
				logit_[node.pos][node.offset] = mean(children);
				break;
			case PropagationCriteria::FIRST:
				logit_[node.pos][node.offset] = logit_[children].front();
				break;
			case PropagationCriteria::NONE: break;
		}

		contains_unknown_[node.pos][node.offset]  = contains_unknown_[children].any();
		contains_free_[node.pos][node.offset]     = contains_free_[children].any();
		contains_occupied_[node.pos][node.offset] = contains_occupied_[children].any();

		// contains_unknown_[node.pos][node.offset] =
		//     contains_unknown_[children].any() || containsUnknown(children);
		// contains_free_[node.pos][node.offset] =
		//     contains_free_[children].any() || containsFree(children);
		// contains_occupied_[node.pos][node.offset] =
		//     contains_occupied_[children].any() || containsOccupied(children);
	}

	[[nodiscard]] constexpr logit_t mean(pos_t nodes) const
	{
		return std::accumulate(std::cbegin(logit_[nodes]), std::cend(logit_[nodes]), 0) / N;
	}

	// [[nodiscard]] bool containsUnknown(pos_t nodes) const
	// {
	// 	int r{};
	// 	for (offset_t i{}; N != i; ++i) {
	// 		Node node(nodes, i);
	// 		r += derived().isLeaf(node) && isUnknown(node);
	// 	}
	// 	return r;
	// }

	// [[nodiscard]] bool containsFree(pos_t nodes) const
	// {
	// 	int r{};
	// 	for (offset_t i{}; N != i; ++i) {
	// 		Node node(nodes, i);
	// 		r += derived().isLeaf(node) && isFree(node);
	// 	}
	// 	return r;
	// }

	// [[nodiscard]] bool containsOccupied(pos_t nodes) const
	// {
	// 	int r{};
	// 	for (offset_t i{}; N != i; ++i) {
	// 		Node node(nodes, i);
	// 		r += derived().isLeaf(node) && isOccupied(node);
	// 	}
	// 	return r;
	// }

	//
	// Is prunable
	//

	[[nodiscard]] bool isPrunable(pos_t nodes) const
	{
		return std::all_of(std::cbegin(logit_[nodes]) + 1, std::cend(logit_[nodes]),
		                   [a = logit_[nodes].front()](auto b) { return a == b; });
	}

	//
	// Is
	//

	[[nodiscard]] constexpr bool isUnknown(logit_t const logit) const noexcept
	{
		return !isFree(logit) && !isOccupied(logit);
	}

	[[nodiscard]] constexpr bool isFree(logit_t const logit) const noexcept
	{
		return freeThresLogit() > logit;
	}

	[[nodiscard]] constexpr bool isOccupied(logit_t const logit) const noexcept
	{
		return occupiedThresLogit() < logit;
	}

	//
	// Memory node block
	//

	[[nodiscard]] static constexpr std::size_t memoryNodeBlock() noexcept
	{
		return N * sizeof(logit_t);
	}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] static constexpr MapType mapType() noexcept { return MapType::OCCUPANCY; }

	constexpr std::size_t serializedSize(std::ranges::input_range auto r) const
	{
		return std::ranges::size(r) * memoryNodeBlock();
	}

	void readNodes(ReadBuffer& in, std::ranges::input_range auto r)
	{
		for (auto const [pos, offsets] : r) {
			if (offsets.all()) {
				in.read(logit_[pos].data(), memoryNodeBlock());
			} else {
				DataBlock<logit_t, N> logit;
				in.read(logit.data(), memoryNodeBlock());
				for (offset_t i{}; N != i; ++i) {
					logit_[pos][i] = offsets[i] ? logit[i] : logit_[pos][i];
				}
			}
		}
	}

	void writeNodes(WriteBuffer& out, std::ranges::input_range auto r) const
	{
		out.reserve(out.size() + serializedSize(r));
		for (auto pos : r) {
			out.write(logit_[pos].data(), memoryNodeBlock());
		}
	}

 protected:
	Container<DataBlock<logit_t, 8>> logit_;
	Container<BitSet<8>>             contains_unknown_;
	Container<BitSet<8>>             contains_free_;
	Container<BitSet<8>>             contains_occupied_;

	double clamping_thres_logit_ = logit(0.971);

	logit_t occupied_thres_logit_ = toOccupancyLogit(0.5);  // Threshold for occupied
	logit_t free_thres_logit_     = toOccupancyLogit(0.5);  // Threshold for free

	// TODO: Maybe add lookup tables?

	// Propagation criteria
	PropagationCriteria prop_criteria_ = PropagationCriteria::MAX;

	template <class Derived2, std::size_t N2>
	friend class OccupancyMap;
};

//
// Concepts
//

template <class Map>
concept IsOccupancyMap = IsMapType<Map, MapType::OCCUPANCY>;
}  // namespace ufo

#endif  // UFO_MAP_OCCUPANCY_MAP_HPP