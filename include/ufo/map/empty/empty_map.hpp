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

#ifndef UFO_MAP_EMPTY_MAP_HPP
#define UFO_MAP_EMPTY_MAP_HPP

// UFO
#include <ufo/map/index.hpp>
#include <ufo/map/io.hpp>
#include <ufo/map/types.hpp>

// STL
#include <cstdint>
#include <iostream>
#include <ranges>

namespace ufo
{
template <std::uint64_t Num, class Derived, std::size_t N>
class EmptyMap
{
 protected:
	//
	// Constructors
	//

	constexpr EmptyMap() = default;

	constexpr EmptyMap(EmptyMap const&) = default;

	constexpr EmptyMap(EmptyMap&&) = default;

	template <class Derived2>
	constexpr EmptyMap(EmptyMap<Num, Derived2, N> const&)
	{
	}

	template <class Derived2>
	constexpr EmptyMap(EmptyMap<Num, Derived2, N>&&)
	{
	}

	//
	// Destructor
	//

	~EmptyMap() = default;

	//
	// Assignment operator
	//

	constexpr EmptyMap& operator=(EmptyMap const&) = default;

	constexpr EmptyMap& operator=(EmptyMap&&) = default;

	template <class Derived2>
	constexpr EmptyMap& operator=(EmptyMap<Num, Derived2, N> const&)
	{
		return *this;
	}

	template <class Derived2>
	constexpr EmptyMap& operator=(EmptyMap<Num, Derived2, N>&&)
	{
		return *this;
	}

	//
	// Swap
	//

	void swap(EmptyMap&) noexcept {}

	//
	// Create node block
	//

	static constexpr void createNodeBlock(Index) noexcept {}

	//
	// Resize
	//

	static constexpr void resize(std::size_t) noexcept {}

	//
	// Reserve
	//

	static constexpr void reserveImpl(std::size_t) noexcept {}

	//
	// Initialize root
	//

	static constexpr void initRoot() noexcept {}

	//
	// Fill
	//

	static constexpr void fill(Index, pos_t) noexcept {}

	//
	// Clear
	//

	static constexpr void clearImpl() noexcept {}

	static constexpr void clearImpl(pos_t) noexcept {}

	//
	// Shrink to fit
	//

	static constexpr void shrinkToFitImpl() noexcept {}

	//
	// Update node
	//

	static constexpr void updateNode(Index, pos_t) noexcept {}

	//
	// Is prunable
	//

	[[nodiscard]] static constexpr bool isPrunable(pos_t) noexcept { return true; }

	//
	// Memory node block
	//

	[[nodiscard]] static constexpr std::size_t memoryNodeBlock() noexcept { return 0; }

	//
	// Input/ouput (read/write)
	//

	static constexpr MapType mapType() noexcept { return MapType::NONE; }

	static constexpr std::size_t serializedSize(std::ranges::input_range auto) noexcept
	{
		return 0;
	}

	static constexpr void readNodes(ReadBuffer&, std::ranges::input_range auto) noexcept {}

	static constexpr void writeNodes(WriteBuffer&, std::ranges::input_range auto) noexcept
	{
	}

	//
	// Dot file info
	//

	static std::ostream& dotFileInfo(std::ostream& out, Index) { return out; }
};

//
// Concepts
//

template <class Map>
concept IsEmptyMap = IsMapType<Map, MapType::NONE>;
}  // namespace ufo

#endif  // UFO_MAP_EMPTY_MAP_HPP