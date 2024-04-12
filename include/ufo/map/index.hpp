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

#ifndef UFO_MAP_INDEX_HPP
#define UFO_MAP_INDEX_HPP

// UFO
#include <ufo/map/bit_set.hpp>
#include <ufo/map/types.hpp>

// STL
#include <algorithm>
#include <ostream>

namespace ufo
{
struct Index {
	pos_t    pos{NULL_POS};
	offset_t offset{0};

	constexpr Index() noexcept = default;

	constexpr Index(pos_t pos, offset_t offset) noexcept : pos(pos), offset(offset) {}

	void swap(Index& other) noexcept
	{
		std::swap(pos, other.pos);
		std::swap(offset, other.offset);
	}

	constexpr bool operator==(Index rhs) const
	{
		return pos == rhs.pos && offset == rhs.offset;
	}

	constexpr bool operator!=(Index rhs) const { return !(operator==(rhs)); }

	constexpr Index sibling(offset_t offset) const { return {pos, offset}; }
};
}  // namespace ufo

namespace std
{
inline ostream& operator<<(ostream& out, ufo::Index index)
{
	return out << "pos: " << +index.pos << " offset: " << +index.offset;
}

inline void swap(ufo::Index& lhs, ufo::Index& rhs) noexcept(noexcept(lhs.swap(rhs)))
{
	lhs.swap(rhs);
}

template <>
struct hash<ufo::Index> {
	std::size_t operator()(ufo::Index index) const
	{
		return (static_cast<std::uint64_t>(index.pos) << 3) | index.offset;
	}
};
}  // namespace std

#endif  // UFO_MAP_INDEX_HPP