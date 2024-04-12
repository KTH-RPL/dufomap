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

#ifndef UFO_CONTAINER_SMALL_VECTOR_HPP
#define UFO_CONTAINER_SMALL_VECTOR_HPP

// UFO
#include <ufo/map/buffer.hpp>
#include <ufo/map/index.hpp>
#include <ufo/map/types.hpp>
#include <ufo/util/iterator_wrapper.hpp>

// STL
#include <cstdlib>
#include <iterator>
#include <memory>
#include <numeric>
#include <type_traits>
#include <utility>

namespace ufo
{
template <typename T, std::size_t T_NUM, std::size_t N, typename SIZE_T = T>
class SmallVector
{
	// TODO: Make it possible to have smaller SIZE_T than T
	static constexpr std::size_t SIZES_SIZE   = sizeof(SIZE_T) / sizeof(T);
	static constexpr std::size_t SIZES_OFFSET = N * SIZES_SIZE;

 public:
	using size_type       = SIZE_T;
	using difference_type = std::ptrdiff_t;
	using reference       = std::conditional_t<1 == T_NUM, T &, std::array<T, T_NUM> &>;
	using const_reference =
	    std::conditional_t<1 == T_NUM, T const &, std::array<T, T_NUM> const &>;
	using pointer = std::conditional_t<1 == T_NUM, T *, std::array<T, T_NUM> *>;
	using const_pointer =
	    std::conditional_t<1 == T_NUM, T const *, std::array<T, T_NUM> const *>;
	using iterator = std::conditional_t<1 == T_NUM, T *, std::array<T, T_NUM> *>;
	using const_iterator =
	    std::conditional_t<1 == T_NUM, T const *, std::array<T, T_NUM> const *>;
	using reverse_iterator       = std::reverse_iterator<iterator>;
	using const_reverse_iterator = std::reverse_iterator<const_iterator>;
	using Query                  = IteratorWrapper<iterator>;
	using ConstQuery             = IteratorWrapper<const_iterator>;
	using ReverseQuery           = IteratorWrapper<reverse_iterator>;
	using ReverseConstQuery      = IteratorWrapper<const_reverse_iterator>;

 public:
	//
	// Constructors
	//

	//
	// Destructor
	//

	//
	// Assignment operator
	//

	//
	// iterators
	//

	[[nodiscard]] iterator begin() { return begin(0); }

	[[nodiscard]] const_iterator begin() const { return begin(0); }

	[[nodiscard]] const_iterator cbegin() const { return cbegin(); }

	[[nodiscard]] iterator end() { return end(N - 1); }

	[[nodiscard]] const_iterator end() const { return end(N - 1); }

	[[nodiscard]] const_iterator cend() const { return end(); }

	[[nodiscard]] iterator begin(std::size_t pos)
	{
		return reinterpret_cast<iterator>(data_ ? data_.get() + offset(pos) : nullptr);
	}

	[[nodiscard]] const_iterator begin(std::size_t pos) const
	{
		return reinterpret_cast<const_iterator>(data_ ? data_.get() + offset(pos) : nullptr);
	}

	[[nodiscard]] const_iterator cbegin(std::size_t pos) const { return begin(pos); }

	[[nodiscard]] iterator end(std::size_t pos)
	{
		return reinterpret_cast<iterator>(data_ ? data_.get() + offset(pos + 1) : nullptr);
	}

	[[nodiscard]] const_iterator end(std::size_t pos) const
	{
		return reinterpret_cast<const_iterator>(data_ ? data_.get() + offset(pos + 1)
		                                              : nullptr);
	}

	[[nodiscard]] const_iterator cend(std::size_t pos) const { return end(pos); }

	[[nodiscard]] reverse_iterator rbegin() { return std::make_reverse_iterator(end()); }

	[[nodiscard]] const_iterator rbegin() const
	{
		return std::make_reverse_iterator(end());
	}

	[[nodiscard]] const_iterator crbegin() const { return crbegin(); }

	[[nodiscard]] iterator rend() { return std::make_reverse_iterator(begin()); }

	[[nodiscard]] const_iterator rend() const
	{
		return std::make_reverse_iterator(begin());
	}

	[[nodiscard]] const_iterator crend() const { return rend(); }

	[[nodiscard]] iterator rbegin(std::size_t pos)
	{
		return std::make_reverse_iterator(end(pos));
	}

	[[nodiscard]] const_iterator rbegin(std::size_t pos) const
	{
		return std::make_reverse_iterator(end(pos));
	}

	[[nodiscard]] const_iterator crbegin(std::size_t pos) const { return rbegin(pos); }

	[[nodiscard]] iterator rend(std::size_t pos)
	{
		return std::make_reverse_iterator(begin(pos));
	}

	[[nodiscard]] const_iterator rend(std::size_t pos) const
	{
		return std::make_reverse_iterator(begin(pos));
	}

	[[nodiscard]] const_iterator crend(std::size_t pos) const { return rend(pos); }

	//
	// Query
	//

	[[nodiscard]] Query query() { return {begin(), end()}; }

	[[nodiscard]] Query query(std::size_t pos) { return {begin(pos), end(pos)}; }

	[[nodiscard]] ConstQuery query() const { return {begin(), end()}; }

	[[nodiscard]] ConstQuery query(std::size_t pos) const { return {begin(pos), end(pos)}; }

	[[nodiscard]] ReverseQuery rquery() { return {rbegin(), rend()}; }

	[[nodiscard]] ReverseQuery rquery(std::size_t pos) { return {rbegin(pos), rend(pos)}; }

	[[nodiscard]] ReverseConstQuery rquery() const { return {rbegin(), rend()}; }

	[[nodiscard]] ReverseConstQuery rquery(std::size_t pos) const
	{
		return {rbegin(pos), rend(pos)};
	}

	//
	// Data
	//

	T *data() { return data(0); }

	T const *data() const { return data(0); }

	T *data(std::size_t pos) { return data_.get() + offset(pos); }

	T const *data(std::size_t pos) const { return data_.get() + offset(pos); }

	//
	// Operator()()
	//

	reference operator()(std::size_t idx) { return operator()(0, idx); }

	const_reference operator()(std::size_t idx) const { return operator()(0, idx); }

	reference operator()(std::size_t pos, std::size_t idx)
	{
		return reinterpret_cast<reference>(data_[offset(pos) + T_NUM * idx]);
	}

	const_reference operator()(std::size_t pos, std::size_t idx) const
	{
		return reinterpret_cast<const_reference>(data_[offset(pos) + T_NUM * idx]);
	}

	//
	// Clear
	//

	void clear()
	{
		if (data_) {
			data_.reset();
		}
	}

	void clear(std::size_t pos)
	{
		if (size() == size(pos)) {
			clear();
		} else {
			resize(pos);
		}
	}

	//
	// Empty
	//

	[[nodiscard]] bool empty() const { return nullptr == data_; }

	[[nodiscard]] bool empty(std::size_t pos) const { return 0 == size(pos); }

	//
	// Size
	//

	[[nodiscard]] std::size_t size() const
	{
		auto s = sizes();
		return std::accumulate(std::cbegin(s), std::cend(s), std::size_t(0));
	}

	[[nodiscard]] std::size_t size(std::size_t pos) const
	{
		return data_ ? *(reinterpret_cast<size_type const *>(data_.get()) + pos) : 0;
	}

	[[nodiscard]] std::array<size_type, N> sizes() const
	{
		if (empty()) {
			return std::array<size_type, N>{};
		}

		size_type const *it = reinterpret_cast<size_type const *>(data_.get());

		std::array<size_type, N> s;
		std::copy(it, it + N, s.data());
		return s;
	}

	//
	// Allocated size
	//

	// [[nodiscard]] std::size_t allocatedSize() const
	// {
	// 	return SIZES_OFFSET + T_NUM * size();
	// }

	// [[nodiscard]] std::size_t allocatedSize(std::size_t pos) const
	// {
	// 	return SIZES_OFFSET + T_NUM * size(pos);
	// }

	//
	// Offset
	//

	[[nodiscard]] std::size_t offset(std::size_t pos) const
	{
		size_type const *it = reinterpret_cast<size_type const *>(data_.get());
		return data_ ? SIZES_OFFSET + T_NUM * std::accumulate(it, it + pos, std::size_t(0))
		             : 0;
	}

	//
	// Resize
	//

	void resize(std::size_t count)
	{
		if (0 == count) {
			clear();
			return;
		} else if (size() == count) {
			return;
		}

		T *p_cur = data_.release();
		T *p_new = allocate(p_cur, count);

		if (!p_new) {
			data_.reset(p_cur);
			throw std::bad_alloc();
		}

		data_.reset(p_new);

		auto it = reinterpret_cast<SIZE_T *>(data_.get());
		*it     = static_cast<SIZE_T>(count);
		std::fill(it + 1, it + N, SIZE_T(0));
	}

	void resize(std::size_t pos, std::size_t count)
	{
		auto cur_sizes = sizes();
		auto cur_size =
		    std::accumulate(std::cbegin(cur_sizes), std::cend(cur_sizes), std::size_t(0));
		auto cur_count = cur_sizes[pos];

		if (cur_count == count) {
			return;
		} else if (0 == count && cur_size == cur_count) {
			clear();
			return;
		}

		if (count < cur_count && N - 1 != pos) {
			std::copy(std::to_address(begin(pos + 1)), std::to_address(end()),
			          std::to_address(begin(pos) + count));
		}

		bool was_empty = empty();

		T *p_cur = data_.release();
		T *p_new = allocate(p_cur, cur_size + count - cur_count);

		if (!p_new) {
			data_.reset(p_cur);
			throw std::bad_alloc();
		}

		data_.reset(p_new);

		if (was_empty) {
			auto it = reinterpret_cast<SIZE_T *>(data_.get());
			std::fill(it, it + N, SIZE_T(0));
		} else {
			if (count > cur_count && N - 1 != pos) {
				std::copy_backward(std::to_address(begin(pos + 1)), std::to_address(end()),
				                   std::to_address(end() + (count - cur_count)));
			}
		}

		reinterpret_cast<SIZE_T &>(data_[SIZES_SIZE * pos]) = static_cast<SIZE_T>(count);
	}

	//
	// Memory usage
	//

	[[nodiscard]] std::size_t memoryUsage()
	{
		return empty() ? 0 : N * sizeof(SIZE_T) + size() * T_NUM * sizeof(T);
	}

	[[nodiscard]] std::size_t memoryUsage(std::size_t pos)
	{
		return empty(pos) ? 0 : size(pos) * T_NUM * sizeof(T);
	}

	//
	// Input/output (read/write)
	//

	std::size_t serializedSize() const { return sizeof(std::uint32_t) + memoryUsage(); }

	void read(ReadBuffer &in)
	{
		std::uint32_t s;
		in.read(&s, sizeof(s));
		resize(s);
		if (s) {
			in.read(data_.get(), memoryUsage());
		}
	}

	void read(ReadBuffer &in, std::size_t pos)
	{
		std::uint32_t s;
		in.read(&s, sizeof(s));
		resize(pos, s);
		if (s) {
			in.read(data(pos), memoryUsage(pos));
		}
	}

	void write(WriteBuffer &out)
	{
		std::uint32_t s = size();
		out.write(&s, sizeof(s));
		if (s) {
			out.write(data_.get(), memoryUsage());
		}
	}

	void write(WriteBuffer &out, std::size_t pos)
	{
		std::uint32_t s = size(pos);
		out.write(&s, sizeof(s));
		if (s) {
			out.write(data(pos), memoryUsage(pos));
		}
	}

 protected:
	//
	// Allocate
	//

	T *allocate(T *p, std::size_t s)
	{
		return static_cast<T *>(realloc(p, N * sizeof(SIZE_T) + s * T_NUM * sizeof(T)));
	}

 protected:
	struct free_delete {
		void operator()(void *x) { free(x); }
	};

	std::unique_ptr<T[], free_delete> data_;
};

//
// Specialization for pair
//

// template <typename T1, typename T2, std::size_t N, typename size_type>
// class SmallVector<std::pair<T1, T2>, 1, N, size_type>
// {
//  public:
// 	//
// 	// TODO: Implement
// 	//

//  protected:
// 	//
// 	// TODO: Implement
// 	//

//  protected:
// 	struct free_delete {
// 		void operator()(void *x) { free(x); }
// 	};

// 	std::uniquelhs.ptr_<std::pair<T1, T2>[], free_delete> data_;
// };
}  // namespace ufo

namespace std
{
template <typename T, size_t T_NUM, size_t N, typename SIZE_T = T>
ostream &operator<<(ostream &out, ufo::SmallVector<T, T_NUM, N, SIZE_T> const &vec)
{
	for (size_t i{}; N != i; ++i) {
		if (vec.empty(i)) {
			out << "[] ";
		} else {
			out << "[";
			for (auto const &e : vec.query(i)) {
				if constexpr (1 == T_NUM) {
					out << +e << ' ';
				} else {
					out << '(';
					for (auto x : e) {
						out << +x << ' ';
					}
					out << "\b) ";
				}
			}
			out << "\b] ";
		}
	}

	out << '\b';

	return out;
}
}  // namespace std

#endif  // UFO_CONTAINER_SMALL_VECTOR_HPP