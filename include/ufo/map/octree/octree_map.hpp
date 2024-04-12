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

#ifndef UFO_MAP_HPP
#define UFO_MAP_HPP

// UFO
#include <ios>
#include <ufo/map/octree/octree.hpp>

namespace ufo
{
// NOTE: Remove this when it is possible to friend varidic number of classes
#define REPEAT_2(M, N)   M(N) M(N + 1)
#define REPEAT_4(M, N)   REPEAT_2(M, N) REPEAT_2(M, N + 2)
#define REPEAT_8(M, N)   REPEAT_4(M, N) REPEAT_4(M, N + 4)
#define REPEAT_16(M, N)  REPEAT_8(M, N) REPEAT_8(M, N + 8)
#define REPEAT_32(M, N)  REPEAT_16(M, N) REPEAT_16(M, N + 16)
#define REPEAT_64(M, N)  REPEAT_32(M, N) REPEAT_32(M, N + 32)
#define REPEAT_128(M, N) REPEAT_64(M, N) REPEAT_64(M, N + 64)

// All your base are belong to us
template <template <class, std::size_t> class... Maps>
class OctreeMap
    : public Octree<OctreeMap<Maps...>>
    , public Maps<OctreeMap<Maps...>, 8>...
{
 protected:
	//
	// Friends
	//

	friend Octree<OctreeMap>;
#define FRIEND(N)                                                       \
	friend std::tuple_element_t<std::min(static_cast<std::size_t>(N + 1), \
	                                     sizeof...(Maps)),                \
	                            std::tuple<void, Maps<OctreeMap, 8>...>>;
	REPEAT_128(FRIEND, 0)

 public:
	//
	// Constructors
	//

	OctreeMap(node_size_t leaf_node_size = 0.1, depth_t depth_levels = 17)
	    : Octree<OctreeMap>(leaf_node_size, depth_levels)
	{
	}

	OctreeMap(std::filesystem::path const& file) : OctreeMap(0.1, 17)
	{
		Octree<OctreeMap>::read(file);
	}

	OctreeMap(std::istream& in) : OctreeMap(0.1, 17) { Octree<OctreeMap>::read(in); }

	OctreeMap(ReadBuffer& in) : OctreeMap(0.1, 17) { Octree<OctreeMap>::read(in); }

	OctreeMap(OctreeMap const& other) = default;

	OctreeMap(OctreeMap&& other) = default;

	template <template <class, std::size_t> class... Maps2>
	OctreeMap(OctreeMap<Maps2...> const& other) : Octree<OctreeMap>(other)
	{
		(initilize<Maps<OctreeMap, 8>>(other), ...);
		(init<Maps<OctreeMap, 8>, OctreeMap<Maps2...>::mapType()>(
		     Octree<OctreeMap>::numNodesAllocated() / 8),
		 ...);
	}

	template <template <class, std::size_t> class... Maps2>
	OctreeMap(OctreeMap<Maps2...>&& other) : Octree<OctreeMap>(other)
	{
		(initilize<Maps<OctreeMap, 8>>(std::move(other)), ...);
		(init<Maps<OctreeMap, 8>, OctreeMap<Maps2...>::mapType()>(
		     Octree<OctreeMap>::numNodesAllocated() / 8),
		 ...);
	}

	OctreeMap& operator=(OctreeMap const& rhs) = default;

	OctreeMap& operator=(OctreeMap&& rhs) = default;

	template <template <class, std::size_t> class... Maps2>
	OctreeMap& operator=(OctreeMap<Maps2...> const& rhs)
	{
		Octree<OctreeMap>::operator=(rhs);
		(initilize<Maps<OctreeMap, 8>>(rhs), ...);
		(init<Maps<OctreeMap, 8>, OctreeMap<Maps2...>::mapType()>(
		     Octree<OctreeMap>::numNodesAllocated() / 8),
		 ...);
		return *this;
	}

	template <template <class, std::size_t> class... Maps2>
	OctreeMap& operator=(OctreeMap<Maps2...>&& rhs)
	{
		Octree<OctreeMap>::operator=(rhs);
		(initilize<Maps<OctreeMap, 8>>(std::move(rhs)), ...);
		(init<Maps<OctreeMap, 8>, OctreeMap<Maps2...>::mapType()>(
		     Octree<OctreeMap>::numNodesAllocated() / 8),
		 ...);
		return *this;
	}

	//
	// Swap
	//

	void swap(OctreeMap& other) noexcept(noexcept(
	    Octree<OctreeMap>::swap(other)) && noexcept((Maps<OctreeMap, 8>::swap(other), ...)))
	{
		Octree<OctreeMap>::swap(other);
		(Maps<OctreeMap, 8>::swap(other), ...);
	}

	//
	// Map type
	//

	[[nodiscard]] static constexpr mt_t mapType() noexcept
	{
		return (Maps<OctreeMap, 8>::mapType() | ...);
	}

 protected:
	//
	// Init
	//

	template <class Map, mt_t MT>
	void init(std::size_t size)
	{
		if constexpr (0 == (MT & Map::mapType())) {
			Map::resize(1);
			Map::initRoot();
			Map::resize(size);
		}
	}

	//
	// Create node block
	//

	void createNodeBlock(Index node) { (Maps<OctreeMap, 8>::createNodeBlock(node), ...); }

	//
	// Apply Permutation
	//

	void applyPermutation(Permutation const& perm)
	{
		(Maps<OctreeMap, 8>::applyPermutation(perm), ...);
	}

	//
	// Resize
	//

	void resize(std::size_t new_size) { (Maps<OctreeMap, 8>::resize(new_size), ...); }

	//
	// Reserve
	//

	void reserveImpl(std::size_t new_cap)
	{
		(Maps<OctreeMap, 8>::reserveImpl(new_cap), ...);
	}

	//
	// Initialize root
	//

	void initRoot() { (Maps<OctreeMap, 8>::initRoot(), ...); }

	//
	// Fill
	//

	void fill(Index node, pos_t children)
	{
		(Maps<OctreeMap, 8>::fill(node, children), ...);
	}

	//
	// Clear
	//

	void clearImpl(pos_t nodes) { (Maps<OctreeMap, 8>::clearImpl(nodes), ...); }

	//
	// Shrink to fit
	//

	void shrinkToFitImpl() { (Maps<OctreeMap, 8>::shrinkToFit(), ...); }

	//
	// Update node
	//

	void updateNode(Index node, pos_t children)
	{
		(Maps<OctreeMap, 8>::updateNode(node, children), ...);
	}

	//
	// Is prunable
	//

	[[nodiscard]] bool isPrunable(pos_t nodes) const
	{
		return (Maps<OctreeMap, 8>::isPrunable(nodes) && ...);
	}

	//
	// Memory node block
	//

	[[nodiscard]] static constexpr std::size_t memoryNodeBlock() noexcept
	{
		return (Maps<OctreeMap, 8>::memoryNodeBlock() + ...);
	}

	[[nodiscard]] std::size_t serializedSize(std::ranges::input_range auto r, bool compress,
	                                         mt_t data) const
	{
		return (serializedSize<Maps<OctreeMap, 8>>(r, compress, data) + ...);
	}

	void readNodes(std::istream& in, std::ranges::input_range auto r, bool const compressed,
	               mt_t map_types)
	{
		auto cur_pos = in.tellg();
		in.seekg(0, std::ios_base::end);
		auto end_pos = in.tellg();
		in.seekg(cur_pos);

		map_types &= mapType();

		Buffer buf;
		Buffer compress_buf;
		while (in.tellg() != end_pos && in.good()) {
			MapType       mt;
			std::uint64_t data_size;

			in.read(reinterpret_cast<char*>(&mt), sizeof(mt));
			in.read(reinterpret_cast<char*>(&data_size), sizeof(data_size));

			if (mt & map_types) {
				(readNodes<Maps<OctreeMap, 8>>(in, buf, compress_buf, r, mt, data_size,
				                               compressed) ||
				 ...);
			} else {
				// Skip forward
				in.seekg(static_cast<std::istream::off_type>(data_size), std::istream::cur);
			}
		}
	}

	void readNodes(ReadBuffer& in, std::ranges::input_range auto r, bool const compressed,
	               mt_t map_types)
	{
		map_types &= mapType();

		Buffer compress_buf;
		while (in.readIndex() < in.size()) {
			MapType       mt;
			std::uint64_t data_size;

			in.read(&mt, sizeof(mt));
			in.read(&data_size, sizeof(data_size));

			std::uint64_t next_index = in.readIndex() + data_size;

			if (mt & map_types) {
				(readNodes<Maps<OctreeMap, 8>>(in, compress_buf, r, mt, data_size, compressed) ||
				 ...);
			}

			// Skip forward
			in.setReadIndex(next_index);
		}
	}

	void writeNodes(std::ostream& out, std::ranges::input_range auto r, bool const compress,
	                mt_t const map_types, int const compression_acceleration_level,
	                int const compression_level) const
	{
		Buffer buf;
		(writeNodes<Maps<OctreeMap, 8>>(out, buf, r, compress, map_types,
		                                compression_acceleration_level, compression_level),
		 ...);
	}

	void writeNodes(WriteBuffer& out, std::ranges::input_range auto r, bool const compress,
	                mt_t const map_types, int const compression_acceleration_level,
	                int const compression_level) const
	{
		out.reserve(out.size() + serializedSize(r, compress, map_types));
		(writeNodes<Maps<OctreeMap, 8>>(out, r, compress, map_types,
		                                compression_acceleration_level, compression_level),
		 ...);
	}

	//
	// Dot file info
	//

	std::ostream& dotFileInfo(std::ostream& out, Index node, mt_t map_types) const
	{
		(dotFileInfo<Maps<OctreeMap, 8>>(out, node, map_types), ...);
		return out;
	}

 private:
	//
	// Initilize
	//

	template <class Map, class Map2>
	void initilize(Map2 const& other)
	{
		if constexpr (Map::mapType() & Map2::mapType()) {
			Map::operator=(other);
		}
	}

	template <class Map, class Map2>
	void initlize(Map2&& other)
	{
		if constexpr (Map::mapType() & Map2::mapType()) {
			Map::operator=(std::forward<Map2>(other));
		}
	}

	//
	// Input/output (read/write)
	//

	template <class Map>
	std::size_t serializedSize(std::ranges::input_range auto r, bool compress,
	                           mt_t map_types) const
	{
		if (Map::mapType() & map_types) {
			return 0;
		}

		if (compress) {
			return sizeof(MapType) + sizeof(std::uint64_t) + sizeof(std::uint64_t) +
			       maxSizeCompressed(Map::serializedSize(r));
		} else {
			return sizeof(MapType) + sizeof(std::uint64_t) + Map::serializedSize(r);
		}
	}

	template <class Map>
	bool readNodes(std::istream& in, Buffer& buf, Buffer& compress_buf,
	               std::ranges::input_range auto r, MapType const mt,
	               uint64_t const data_size, bool const compressed)
	{
		if (Map::mapType() != mt) {
			return false;
		}

		buf.clear();
		// TODO: Implement better (should probably be resize?)
		buf.reserve(data_size);
		in.read(reinterpret_cast<char*>(buf.data()), static_cast<std::streamsize>(data_size));
		return readNodes(buf, compress_buf, r, mt, data_size, compressed);
	}

	template <class Map>
	bool readNodes(ReadBuffer& in, Buffer& compress_buf, std::ranges::input_range auto r,
	               MapType const mt, uint64_t const data_size, bool const compressed)
	{
		if (Map::mapType() != mt) {
			return false;
		}

		if (compressed) {
			compress_buf.clear();

			std::uint64_t uncompressed_size;
			in.read(&uncompressed_size, sizeof(uncompressed_size));

			decompressData(in, compress_buf, uncompressed_size);

			Map::readNodes(compress_buf, r);
		} else {
			Map::readNodes(in, r);
		}

		return true;
	}

	template <class Map>
	void writeNodes(std::ostream& out, Buffer& buf, std::ranges::input_range auto r,
	                bool const compress, mt_t const map_types,
	                int const compression_acceleration_level,
	                int const compression_level) const
	{
		if (0 == (Map::mapType() & map_types)) {
			return;
		}

		buf.clear();
		writeNodes(buf, r, compress, compression_acceleration_level, compression_level);

		if (!buf.empty()) {
			out.write(reinterpret_cast<char const*>(buf.data()),
			          static_cast<std::streamsize>(buf.size()));
		}
	}

	template <class Map>
	void writeNodes(WriteBuffer& out, std::ranges::input_range auto r, bool const compress,
	                mt_t const map_types, int const compression_acceleration_level,
	                int const compression_level) const
	{
		constexpr MapType mt = Map::mapType();
		if constexpr (MapType::NONE == mt) {
			return;
		}

		if (0 == (mt & map_types)) {
			return;
		}

		out.write(&mt, sizeof(mt));

		std::uint64_t size;
		auto          size_index = out.writeIndex();
		out.setWriteIndex(size_index + sizeof(size));

		if (compress) {
			Buffer data;
			data.reserve(Map::serializedSize(r));
			Map::writeNodes(data, r);

			compressData(data, out, compression_acceleration_level, compression_level);
		} else {
			Map::writeNodes(out, r);
		}

		auto cur_index = out.writeIndex();
		size           = cur_index - (size_index + sizeof(size));
		out.setWriteIndex(size_index);
		out.write(&size, sizeof(size));
		out.setWriteIndex(cur_index);
	}

	//
	// Dot file info
	//

	template <class Map>
	std::ostream& dotFileInfo(std::ostream& out, Index node, mt_t map_types) const
	{
		if (0 == (Map::mapType() & map_types)) {
			return out;
		}

		out << "<br/>";

		return Map::dotFileInfo(out, node);
	}
};
}  // namespace ufo

#endif  // UFO_MAP_HPP