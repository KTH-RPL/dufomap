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

#ifndef UFO_MAP_CODE_HPP
#define UFO_MAP_CODE_HPP

// UFO
#include <ufo/map/key.hpp>
#include <ufo/map/types.hpp>

// STL
#include <immintrin.h>

#include <cassert>
#include <cmath>
#include <functional>
#include <list>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>

namespace ufo
{
/*!
 * @brief A code is a single value for indexing a specific node in an octree at
 * a specific depth
 *
 * @details Morton codes are used in UFOMap to increase performance when
 * accessing the octree
 *
 */
class Code
{
 public:
	constexpr Code() = default;

	// TODO: Set most significant none used bits to 0
	explicit constexpr Code(code_t code, depth_t depth = 0)
	    : code_(code >> 3 * depth << (3 * depth + 5) |
	            static_cast<code_t>(~depth & 0b11111))
	{
	}

	// TODO: Set most significant none used bits to 0
	Code(Key key) : code_(toCode(key) << 5 | static_cast<code_t>(~key.depth() & 0b11111)) {}

	/*!
	 * @brief Get the corresponding key to code
	 *
	 * @return The corresponding key to code
	 */
	operator Key() const noexcept
	{
#if defined(__BMI2__)
		return Key(_pext_u64(code_, 0x4924924924924920), _pext_u64(code_, 0x9249249249249240),
		           _pext_u64(code_, 0x2492492492492480), depth());
#else
		return Key(toKey(0), toKey(1), toKey(2), depth());
#endif
	}

	constexpr bool operator==(Code rhs) const { return code_ == rhs.code_; }

	constexpr bool operator!=(Code rhs) const { return !(*this == rhs); }

	constexpr bool operator<(Code rhs) const { return code_ < rhs.code_; }

	constexpr bool operator<=(Code rhs) const { return code_ <= rhs.code_; }

	constexpr bool operator>(Code rhs) const { return code_ > rhs.code_; }

	constexpr bool operator>=(Code rhs) const { return code_ >= rhs.code_; }

	static constexpr bool equalAtDepth(Code lhs, Code rhs, depth_t depth)
	{
		return (lhs.code_ >> (3 * depth + 5)) == (rhs.code_ >> (3 * depth + 5));
	}

	/*!
	 * @brief Return the code at a specified depth
	 *
	 * @param depth The depth of the code
	 * @return Code The code at the specified depth
	 */
	constexpr Code toDepth(depth_t depth) const { return Code(code_ >> 5, depth); }

	/*!
	 * @brief Converts a key to a code
	 *
	 * @param key The key to convert
	 * @return uint64_t The code corresponding to the key
	 */
	static code_t toCode(Key key)
	{
#if defined(__BMI2__)
		return _pdep_u64(key[0], 0x9249249249249249) | _pdep_u64(key[1], 0x2492492492492492) |
		       _pdep_u64(key[2], 0x4924924924924924);
#else
		return splitBy3(key[0]) | (splitBy3(key[1]) << 1) | (splitBy3(key[2]) << 2);
#endif
	}

	/*!
	 * @brief Get the key component from a code
	 *
	 * @param code The code to generate the key component from
	 * @param index The index of the key component
	 * @return The key component value
	 */
	static key_t toKey(Code code, std::size_t index)
	{
		assert(3 > index);
		return get3Bits(code.code_ >> (index + 5));
	}

	/*!
	 * @brief Get the key component from this code
	 *
	 * @param index The index of the key component
	 * @return The key component value
	 */
	key_t toKey(offset_t index) const
	{
		assert(3 > index);
		return get3Bits(code_ >> (index + 5));
	}

	/*!
	 * @brief Get the offset at a specific depth for this code.
	 *
	 * @param depth The depth the index is requested for.
	 * @return The offset at the specified depth.
	 */
	constexpr offset_t offset(depth_t depth) const
	{
		return (code_ >> (3 * depth + 5)) & 0b111;
	}

	// TODO: Rename?
	constexpr offset_t offset() const { return offset(depth()); }

	constexpr Code parent(depth_t parent_depth) const
	{
		assert(parent_depth >= depth());  // TODO: Fix
		return Code(code_ >> 5, parent_depth);
	}

	// TODO: Rename?
	constexpr Code parent() const { return parent(depth() + 1); }

	/*!
	 * @brief Get the code of a specific child to this code
	 *
	 * @param index The index of the child
	 * @return Code The child code
	 */
	constexpr Code child(offset_t index) const
	{
		depth_t d  = depth();
		depth_t cd = d - 1;
		// FIXME: Throw error?
		return 0 != d ? Code((code_ >> 5) | (static_cast<code_t>(index) << (3 * cd)), cd)
		              : *this;
	}

	/*!
	 * @brief Get the code of a specific sibling to this code
	 *
	 * @param index The index of the sibling
	 * @return Code The sibling code
	 */
	inline Code sibling(std::size_t index) const
	{
		depth_t d  = depth();
		code_t  sc = (code_ >> (5 + 3 * (d + 1))) << (3 * (d + 1));
		return Code(sc + (static_cast<code_t>(index) << (3 * d)), d);
	}

	/*!
	 * @brief Get the code
	 *
	 * @return code_t The code
	 */
	constexpr code_t code() const noexcept { return code_ >> 5; }

	constexpr code_t raw() const noexcept { return code_; }

	/*!
	 * @brief Get the depth that this code is specified at
	 *
	 * @return depth_t The depth this code is specified at
	 */
	constexpr depth_t depth() const noexcept { return ~code_ & 0b11111; }

 private:
	static code_t splitBy3(key_t a)
	{
#if defined(__BMI2__)
		return _pdep_u64(static_cast<code_t>(a), 0x9249249249249249);
#else
		code_t code = static_cast<code_t>(a) & 0x1fffff;
		code        = (code | code << 32) & 0x1f00000000ffff;
		code        = (code | code << 16) & 0x1f0000ff0000ff;
		code        = (code | code << 8) & 0x100f00f00f00f00f;
		code        = (code | code << 4) & 0x10c30c30c30c30c3;
		code        = (code | code << 2) & 0x1249249249249249;
		return code;
#endif
	}

	static key_t get3Bits(code_t code)
	{
#if defined(__BMI2__)
		return static_cast<key_t>(_pext_u64(code, 0x9249249249249249));
#else
		code_t a = code & 0x1249249249249249;
		a        = (a ^ (a >> 2)) & 0x10c30c30c30c30c3;
		a        = (a ^ (a >> 4)) & 0x100f00f00f00f00f;
		a        = (a ^ (a >> 8)) & 0x1f0000ff0000ff;
		a        = (a ^ (a >> 16)) & 0x1f00000000ffff;
		a        = (a ^ a >> 32) & 0x1fffff;
		return static_cast<key_t>(a);
#endif
	}

 private:
	// The Morton code
	code_t code_ = 0;

	friend class std::hash<Code>;
};

static constexpr Code INVALID_CODE(std::numeric_limits<code_t>::max(), 0);
}  // namespace ufo

namespace std
{
template <>
struct hash<ufo::Code> {
	std::size_t operator()(ufo::Code code) const { return code.code_; }
};
}  // namespace std

namespace ufo
{
using CodeSet          = std::set<Code>;
using CodeUnorderedSet = std::unordered_set<Code>;
template <typename T>
using CodeMap = std::map<Code, T>;
template <typename T>
using CodeUnorderedMap = std::unordered_map<Code, T>;

// class CodeSet
// {
//  public:
// 	CodeSet(unsigned int power = 18) : power_(power)
// 	{
// 		num_buckets_ = size_t(1) << power_;
// 		data_.resize(num_buckets_);
// 	}

// 	struct CodeSetIterator {
// 		CodeSetIterator(CodeSet const* set = nullptr) : set_(set)
// 		{
// 			if (!set_) {
// 				return;
// 			}

// 			if (set_->data_.empty()) {
// 				set_ = nullptr;
// 			} else {
// 				outer_iter_     = set_->data_.begin();
// 				outer_iter_end_ = set_->data_.end();
// 				while (outer_iter_ != outer_iter_end_ && outer_iter_->empty()) {
// 					++outer_iter_;
// 				}
// 				if (outer_iter_ == outer_iter_end_) {
// 					set_ = nullptr;
// 				} else {
// 					inner_iter_     = outer_iter_->begin();
// 					inner_iter_end_ = outer_iter_->end();
// 				}
// 			}
// 		}

// 		Code const& operator*() const { return *inner_iter_; }

// 		// Postfix increment
// 		CodeSetIterator operator++(int)
// 		{
// 			CodeSetIterator result = *this;
// 			++(*this);
// 			return result;
// 		}

// 		// Prefix increment
// 		CodeSetIterator& operator++()
// 		{
// 			++inner_iter_;
// 			if (inner_iter_ == inner_iter_end_) {
// 				++outer_iter_;
// 				while (outer_iter_ != outer_iter_end_ && outer_iter_->empty()) {
// 					++outer_iter_;
// 				}
// 				if (outer_iter_ == outer_iter_end_) {
// 					set_ = nullptr;
// 				} else {
// 					inner_iter_     = outer_iter_->begin();
// 					inner_iter_end_ = outer_iter_->end();
// 				}
// 			}
// 			return *this;
// 		}

// 		bool operator==(CodeSetIterator const& rhs) const { return (rhs.set_ == set_); }

// 		bool operator!=(CodeSetIterator const& rhs) const { return (rhs.set_ != set_); }

// 	 private:
// 		CodeSet const*                                 set_;
// 		std::vector<std::vector<Code>>::const_iterator outer_iter_;
// 		std::vector<std::vector<Code>>::const_iterator outer_iter_end_;
// 		std::vector<Code>::const_iterator              inner_iter_;
// 		std::vector<Code>::const_iterator              inner_iter_end_;
// 		// typename decltype(CodeSet::data_)::const_iterator outer_iter_;
// 		// typename decltype(CodeSet::data_)::const_iterator outer_iter_end_;
// 		// typename decltype(CodeSet::data_)::value_type::const_iterator inner_iter_;
// 		// typename decltype(CodeSet::data_)::value_type::const_iterator inner_iter_end_;
// 	};

// 	std::pair<int, bool> insert(Code const& value)
// 	{
// 		size_t hash = getBucket(value);
// 		if (std::any_of(std::execution::seq, data_[hash].begin(), data_[hash].end(),
// 		                [&value](auto const& elem) { return value == elem; })) {
// 			return std::make_pair(0, false);  // TODO: Fix
// 		}

// 		++size_;

// 		if (load_factor() > max_load_factor() && power_ < MAX_POWER) {
// 			rehash(num_buckets_ * 2);
// 			hash = getBucket(value);
// 		}

// 		data_[hash].push_back(value);

// 		return std::make_pair(0, true);  // TOOD: Fix
// 	}

// 	void clear()
// 	{
// 		std::for_each(std::execution::seq, data_.begin(), data_.end(),
// 		              [](auto& bucket) { bucket.clear(); });
// 		size_ = 0;
// 	}

// 	bool empty() const noexcept { return 0 == size_; }

// 	size_t size() const noexcept { return size_; }

// 	size_t bucket_count() const noexcept { return num_buckets_; }

// 	unsigned int bucket_count_power() const noexcept { return power_; }

// 	float load_factor() const { return size_ / ((float)num_buckets_); }

// 	float max_load_factor() const { return max_load_factor_; }

// 	void max_load_factor(float max_load_factor)
// 	{
// 		max_load_factor_ = max_load_factor;

// 		if (load_factor() > max_load_factor_ && power_ < MAX_POWER) {
// 			rehash(num_buckets_ * 2);
// 		}
// 	}

// 	void rehash(std::size_t count)
// 	{
// 		std::size_t  min_count = std::max((float)count, size() / max_load_factor());
// 		unsigned int power =
// 		    std::max(power_, std::min((unsigned int)std::log2(min_count) + 1, MAX_POWER));

// 		if (power_ == power) {
// 			return;
// 		}

// 		power_       = power;
// 		num_buckets_ = std::size_t(1) << power_;

// 		decltype(data_) new_data;
// 		new_data.resize(num_buckets_);

// 		for (Code const& value : *this) {
// 			new_data[getBucket(value)].push_back(value);
// 		}

// 		data_.swap(new_data);
// 	}

// 	void reserve(std::size_t count)
// 	{
// 		power_ = std::max(power_, std::min((unsigned int)std::log2(count) + 1, MAX_POWER));
// 		data_.reserve(std::size_t(1) << power_);
// 	}

// 	CodeSetIterator begin() const { return CodeSetIterator(this); }

// 	CodeSetIterator end() const { return CodeSetIterator(); }

// 	void swap(CodeSet& other)
// 	{
// 		data_.swap(other.data_);
// 		std::swap(power_, other.power_);
// 		std::swap(num_buckets_, other.num_buckets_);
// 		std::swap(size_, other.size_);
// 		std::swap(max_load_factor_, other.max_load_factor_);
// 	}

// 	using const_iterator = CodeSetIterator;

//  private:
// 	std::size_t getBucket(Code const& key) const
// 	{
// 		unsigned int offset = 3 * key.getDepth();
// 		unsigned int modder = (num_buckets_ - 1) << offset;
// 		return (Code::Hash()(key) & modder) >> offset;
// 	}

//  private:
// 	std::vector<std::vector<Code>> data_;
// 	unsigned int                   power_;
// 	std::size_t                    num_buckets_;
// 	std::size_t                    size_            = 0;
// 	float                          max_load_factor_ = 1.0;

// 	inline static const unsigned int MAX_POWER = 28;

// 	friend struct CodeSetIterator;
// };

// template <class T>
// class CodeMap
// {
//  public:
// 	CodeMap(unsigned int power = 18) : power_(power)
// 	{
// 		num_buckets_ = std::size_t(1) << power_;  // std::pow(2, power_);
// 		data_.resize(num_buckets_);
// 	}

// 	struct CodeMapIterator {
// 		CodeMapIterator(const CodeMap* map = nullptr) : map_(map)
// 		{
// 			if (nullptr == map_) {
// 				return;
// 			}

// 			if (map_->data_.empty()) {
// 				map_ = nullptr;
// 			} else {
// 				outer_iter_     = map_->data_.begin();
// 				outer_iter_end_ = map_->data_.end();
// 				while (outer_iter_ != outer_iter_end_ && outer_iter_->empty()) {
// 					++outer_iter_;
// 				}
// 				if (outer_iter_ == outer_iter_end_) {
// 					map_ = nullptr;
// 				} else {
// 					inner_iter_     = outer_iter_->begin();
// 					inner_iter_end_ = outer_iter_->end();
// 				}
// 			}
// 		}

// 		const std::pair<Code, T>& operator*() const
// 		{
// 			return *inner_iter_;  // map_->data_[outer_index_][inner_index_];
// 		}

// 		std::pair<Code, T> operator*()
// 		{
// 			return *inner_iter_;  // map_->data_[outer_index_][inner_index_];
// 		}

// 		// Postfix increment
// 		CodeMapIterator operator++(int)
// 		{
// 			CodeMapIterator result = *this;
// 			++(*this);
// 			return result;
// 		}

// 		// Prefix increment
// 		CodeMapIterator& operator++()
// 		{
// 			++inner_iter_;
// 			if (inner_iter_ == inner_iter_end_) {
// 				++outer_iter_;
// 				while (outer_iter_ != outer_iter_end_ && outer_iter_->empty()) {
// 					++outer_iter_;
// 				}
// 				if (outer_iter_ == outer_iter_end_) {
// 					map_ = nullptr;
// 				} else {
// 					inner_iter_     = outer_iter_->begin();
// 					inner_iter_end_ = outer_iter_->end();
// 				}
// 			}
// 			return *this;
// 		}

// 		bool operator==(const CodeMapIterator& rhs) const { return (rhs.map_ == map_); }

// 		bool operator!=(const CodeMapIterator& rhs) const { return (rhs.map_ != map_); }

// 	 private:
// 		const CodeMap*                                                      map_;
// 		typename std::vector<std::list<std::pair<Code, T>>>::const_iterator outer_iter_;
// 		typename std::vector<std::list<std::pair<Code, T>>>::const_iterator outer_iter_end_;
// 		typename std::list<std::pair<Code, T>>::const_iterator              inner_iter_;
// 		typename std::list<std::pair<Code, T>>::const_iterator              inner_iter_end_;
// 		// typename decltype(CodeMap<T>::data_)::const_iterator outer_iter_;
// 		// typename decltype(CodeMap<T>::data_)::const_iterator outer_iter_end_;
// 		// typename decltype(CodeMap<T>::data_)::value_type::const_iterator inner_iter_;
// 		// typename decltype(CodeMap<T>::data_)::value_type::const_iterator inner_iter_end_;
// 	};

// 	T& operator[](Code const& key)
// 	{
// 		std::size_t hash = getBucket(key);
// 		auto        it   = std::find_if(data_[hash].begin(), data_[hash].end(),
// 		                                [&key](const auto& elem) { return key == elem.first;
// }); 		if (it != data_[hash].end()) { 			return it->second;
// 		}

// 		++size_;

// 		if (load_factor() > max_load_factor() && power_ < MAX_POWER) {
// 			rehash(num_buckets_ * 2);
// 			hash = getBucket(key);
// 		}

// 		return std::get<1>(data_[hash].emplace_front(key, T()));  // TODO: How to
// 		                                                          // call default?
// 	}

// 	std::pair<int, bool> try_emplace(Code const& key,
// 	                                 const T&    value)  // TODO: Fix
// 	{
// 		std::size_t hash = getBucket(key);
// 		if (std::any_of(data_[hash].begin(), data_[hash].end(),
// 		                [&key](const auto& elem) { return key == elem.first; })) {
// 			return std::make_pair(0, false);  // TODO: Fix
// 		}

// 		++size_;

// 		if (load_factor() > max_load_factor() && power_ < MAX_POWER) {
// 			rehash(num_buckets_ * 2);
// 			hash = getBucket(key);
// 		}

// 		data_[hash].emplace_front(key, value);

// 		return std::make_pair(0, true);  // TODO: Fix
// 	}

// 	void clear()
// 	{
// 		std::for_each(data_.begin(), data_.end(), [](auto& bucket) { bucket.clear(); });
// 		size_ = 0;
// 	}

// 	bool empty() const { return 0 == size_; }

// 	std::size_t size() { return size_; }

// 	std::size_t bucket_count() const { return num_buckets_; }

// 	unsigned int bucket_count_power() const { return power_; }

// 	float load_factor() const
// 	{
// 		return static_cast<float>(size_) / static_cast<float>(num_buckets_);
// 	}

// 	float max_load_factor() const { return max_load_factor_; }

// 	void max_load_factor(float max_load_factor)
// 	{
// 		max_load_factor_ = max_load_factor;

// 		if (load_factor() > max_load_factor_ && power_ < MAX_POWER) {
// 			rehash(num_buckets_ * 2);
// 		}
// 	}

// 	void rehash(std::size_t count)
// 	{
// 		std::size_t min_count = std::max(
// 		    count, static_cast<std::size_t>(static_cast<float>(size()) /
// max_load_factor())); 		unsigned int power = std::max( 		    power_,
// std::min(static_cast<unsigned int>(std::log2(min_count)) + 1, MAX_POWER));

// 		if (power_ == power) {
// 			return;
// 		}

// 		power_       = power;
// 		num_buckets_ = std::size_t(1) << power_;

// 		decltype(data_) new_data;
// 		new_data.resize(num_buckets_);

// 		for (const auto& [key, value] : *this) {
// 			new_data[getBucket(key)].emplace_front(key, value);
// 		}

// 		data_.swap(new_data);
// 		// fprintf(stderr, "\n\nRehash, new power: %u\n\n", power_);
// 	}

// 	void reserve(std::size_t count)
// 	{
// 		power_ = std::max(
// 		    power_, std::min(static_cast<unsigned int>(std::log2(count)) + 1, MAX_POWER));
// 		data_.reserve(std::size_t(1) << power_);
// 	}

// 	CodeMapIterator begin() const { return CodeMapIterator(this); }

// 	CodeMapIterator end() const { return CodeMapIterator(); }

// 	void swap(CodeMap<T>& other)
// 	{
// 		data_.swap(other.data_);
// 		std::swap(power_, other.power_);
// 		std::swap(num_buckets_, other.num_buckets_);
// 		std::swap(size_, other.size_);
// 		std::swap(max_load_factor_, other.max_load_factor_);
// 	}

//  private:
// 	std::size_t getBucket(Code const& key) const
// 	{
// 		unsigned int offset = 3 * key.depth();
// 		unsigned int modder = static_cast<unsigned int>((num_buckets_ - 1) << offset);
// 		return (key.code() & modder) >> offset;
// 	}

//  private:
// 	std::vector<std::list<std::pair<Code, T>>> data_;
// 	unsigned int                               power_;
// 	std::size_t                                num_buckets_;
// 	std::size_t                                size_            = 0;
// 	float                                      max_load_factor_ = 1.0;

// 	inline static const unsigned int MAX_POWER = 28;

// 	friend struct CodeMapIterator;
// };
}  // namespace ufo

#endif  // UFO_MAP_CODE_HPP