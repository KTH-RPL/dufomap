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

#ifndef UFO_GEOMETRY_MINIMUM_DISTANCE_HPP
#define UFO_GEOMETRY_MINIMUM_DISTANCE_HPP

// UFO
#include <ufo/geometry/bounding_volume.hpp>

namespace ufo
{
//
// AABB
//

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(AABB const& a, AABB const& b) noexcept
{
	Point a_min = a.min();
	Point a_max = a.max();
	Point b_min = b.min();
	Point b_max = b.max();

	float result = 0;
	for (size_t i = 0; i != 3; ++i) {
		float delta = std::fdim(a_min[i], b_max[i]) + std::fdim(b_min[i], a_max[i]);
		result += delta * delta;

		// if (a_min[i] > b_max[i]) {
		// 	float delta = b_max[i] - a_min[i];
		// 	result += delta * delta;
		// } else if (b_min[i] > a_max[i]) {
		// 	float delta = a_max[i] - b_min[i];
		// 	result += delta * delta;
		// }
		// else the projection intervals overlap.
	}

	return result;
}

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(AABB const& a, AABB const& b) noexcept
{
	return std::sqrt(squaredDistance(a, b));
}

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(AABB const& a, AABC b) noexcept
{
	Point a_min = a.min();
	Point a_max = a.max();
	Point b_min = b.min();
	Point b_max = b.max();

	float result = 0;
	for (size_t i = 0; i != 3; ++i) {
		float delta = std::fdim(a_min[i], b_max[i]) + std::fdim(b_min[i], a_max[i]);
		result += delta * delta;

		// if (a_min[i] > b_max[i]) {
		// 	float delta = b_max[i] - a_min[i];
		// 	result += delta * delta;
		// } else if (b_min[i] > a_max[i]) {
		// 	float delta = a_max[i] - b_min[i];
		// 	result += delta * delta;
		// }
		// else the projection intervals overlap.
	}

	return result;
}

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(AABB const& a, AABC b) noexcept
{
	return std::sqrt(squaredDistance(a, b));
}

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(AABB const& a, Frustum const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(AABB const& a, Frustum const& b) noexcept
// {
// 	// FIXME: Enable
// 	return std::sqrt(squaredDistance(a, b));
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(AABB const& a, LineSegment const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(AABB const& a, LineSegment const& b) noexcept
// {
// 	// FIXME: Enable
// 	return std::sqrt(squaredDistance(a, b));
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(AABB const& a, OBB const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(AABB const& a, OBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return std::sqrt(squaredDistance(a, b));
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(AABB const& a, Plane const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(AABB const& a, Plane const& b) noexcept
// {
// 	// FIXME: Enable
// 	return std::sqrt(squaredDistance(a, b));
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(AABB const& a, Point b) noexcept
{
	return b.squaredDistance(Point::clamp(b, a.min(), a.max()));
}

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(AABB const& a, Point b) noexcept
{
	return std::sqrt(squaredDistance(a, b));
}

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(AABB const& a, Ray const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(AABB const& a, Ray const& ray)
// {
// 	// FIXME: Enable
// 	return std::sqrt(squaredDistance(a, b));
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(AABB const& a, Sphere const& b) noexcept
{
	return std::fdim(distance(a, b.center), b.radius);
}

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(AABB const& a, Sphere const& b) noexcept
{
	// FIXME: Implement better
	auto dist = distance(a, b);
	return dist * dist;
}

//
// AABC
//

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(AABC a, AABB const& b) noexcept
{
	return squaredDistance(b, a);
}

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(AABC a, AABB const& b) noexcept { return distance(b, a); }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(AABC a, AABC b) noexcept
{
	// FIXME: Is this actually correct?
	// Point a;
	// for (size_t i = 0; i != 3; ++i) {
	// 	a[i] = std::max(0.0f, (aabc_1.center[i] - aabc_1.half_size) -
	// 	                          (aabc_2.center[i] + aabc_2.half_size));
	// }
	// float result = a.squaredNorm();
	// for (size_t i = 0; i != 3; ++i) {
	// 	a[i] = std::max(0.0f, (aabc_2.center[i] - aabc_2.half_size) -
	// 	                          (aabc_1.center[i] + aabc_1.half_size));
	// }
	// return result + a.squaredNorm();

	float hs     = a.half_size + b.half_size;
	float result = 0;
	for (size_t i = 0; i != 3; ++i) {
		float tmp = std::fdim(std::abs(a.center[i] - b.center[i]), hs);
		result += tmp * tmp;
	}
	return result;

	// float res = 0.0f;
	// for (size_t i = 0; i != 3; ++i) {
	// 	float tmp = std::max(0.0f, std::max((aabc_1.center[i] - aabc_1.half_size) -
	// 	                                        (aabc_2.center[i] + aabc_2.half_size),
	// 	                                    (aabc_2.center[i] - aabc_2.half_size) -
	// 	                                        (aabc_1.center[i] + aabc_1.half_size)));
	// 	res += tmp * tmp;
	// }
	// return res;

	// Point a = aabc_1.min() - aabc_2.max();
	// Point b = aabc_2.min() - aabc_1.max();

	// float result = 0.0;
	// for (size_t i = 0; i != 3; ++i) {
	// 	float d_1 = std::max(0.0f, a[i]);
	// 	float d_2 = std::max(0.0f, b[i]);
	// 	result += (d_1 * d_1) + (d_2 * d_2);
	// }

	// return result;
}

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(AABC a, AABC b) noexcept
{
	return std::sqrt(squaredDistance(a, b));
}

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(AABC a, Frustum const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(AABC a, Frustum const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(AABC a, LineSegment const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(AABC a, LineSegment const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(AABC a, OBB const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(AABC a, OBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(AABC a, Plane const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(AABC a, Plane const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(AABC a, Point b) noexcept
{
	return b.squaredDistance(Point::clamp(b, a.min(), a.max()));
}

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(AABC a, Point b) noexcept
{
	return b.distance(Point::clamp(b, a.min(), a.max()));
}

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(AABC a, Ray const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(AABC a, Ray const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(AABC a, Sphere const& b) noexcept
{
	return std::fdim(distance(a, b.center), b.radius);
}

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(AABC a, Sphere const& b) noexcept
{
	// FIXME: Implement better
	auto dist = distance(a, b);
	return dist * dist;
}

//
// Frustum
//

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Frustum const& a, AABB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Frustum const& a, AABB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Frustum const& a, AABC b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Frustum const& a, AABC b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Frustum const& a, Frustum const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Frustum const& a, Frustum const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Frustum const& a, LineSegment const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Frustum const& a, LineSegment const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Frustum const& a, OBB const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Frustum const& a, OBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Frustum const& a, Plane const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Frustum const& a, Plane const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Frustum const& a, Point b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Frustum const& a, Point b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Frustum const& a, Ray const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Frustum const& a, Ray const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Frustum const& a, Sphere const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Frustum const& a, Sphere const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

//
// Line segment
//

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(LineSegment const& a, AABB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(LineSegment const& a, AABB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(LineSegment const& a, AABC b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(LineSegment const& a, AABC b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(LineSegment const& a, Frustum const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(LineSegment const& a, Frustum const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(LineSegment const& a, LineSegment const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(LineSegment const& a, LineSegment const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(LineSegment const& a, OBB const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(LineSegment const& a, OBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(LineSegment const& a, Plane const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(LineSegment const& a, Plane const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(LineSegment const& a, Point b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(LineSegment const& a, Point b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(LineSegment const& a, Ray const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(LineSegment const& a, Ray const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(LineSegment const& a, Sphere const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(LineSegment const& a, Sphere const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

//
// OBB
//

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(OBB const& a, AABB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(OBB const& a, AABB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(OBB const& a, AABC b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(OBB const& a, AABC b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(OBB const& a, Frustum const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(OBB const& a, Frustum const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(OBB const& a, AABC b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(OBB const& a, LineSegment const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(OBB const& a, OBB const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(OBB const& a, OBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(OBB const& a, Plane const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(OBB const& a, Plane const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(OBB const& a, Point b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(OBB const& a, Point b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(OBB const& a, Ray const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(OBB const& a, Ray const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(OBB const& a, Sphere const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(OBB const& a, Sphere const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

//
// Plane
//

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Plane const& a, AABB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Plane const& a, AABB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Plane const& a, AABC b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Plane const& a, AABC b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Plane const& a, Frustum const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Plane const& a, Frustum const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Plane const& a, LineSegment const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Plane const& a, LineSegment const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Plane const& a, OBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Plane const& a, OBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Plane const& a, Plane const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Plane const& a, Plane const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Plane const& a, Point b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Plane const& a, Point b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Plane const& a, Ray const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Plane const& a, Ray const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Plane const& a, Sphere const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Plane const& a, Sphere const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

//
// Point
//

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(Point a, AABB const& b) noexcept
{
	return squaredDistance(b, a);
}

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(Point a, AABB const& b) noexcept { return distance(b, a); }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(Point a, AABC b) noexcept
{
	return squaredDistance(b, a);
}

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(Point a, AABC b) noexcept { return distance(b, a); }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Point a, Frustum const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Point a, Frustum const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Point a, LineSegment const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Point a, LineSegment const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Point a, OBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Point a, OBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Point a, Plane const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Point a, Plane const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(Point a, Point b) noexcept
{
	return a.squaredDistance(b);
}

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(Point a, Point b) noexcept { return a.distance(b); }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Point a, Ray const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Point a, Ray const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(Point a, Sphere const& b) noexcept
{
	return std::fdim(a.distance(b.center), b.radius);
}

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(Point a, Sphere const& b) noexcept
{
	auto dist = distance(a, b);
	return dist * dist;
}

//
// Ray
//

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Ray const& a, AABB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Ray const& a, AABB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Ray const& a, AABC b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Ray const& a, AABC b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Ray const& a, Frustum const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Ray const& a, Frustum const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Ray const& a, LineSegment const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Ray const& a, LineSegment const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Ray const& a, OBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Ray const& a, OBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Ray const& a, Plane const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Ray const& a, Plane const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Ray const& a, Point b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Ray const& a, Point b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Ray const& a, Ray const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Ray const& a, Ray const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Ray const& a, AABB const& b) noexcept
// {
// 	// TODO: Implement
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Ray const& a, Sphere const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(a, b);
// }

//
// Sphere
//

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(Sphere const& a, AABB const& b) noexcept
{
	return squaredDistance(b, a);
}

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(Sphere const& a, AABB const& b) noexcept
{
	return distance(b, a);
}

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(Sphere const& a, AABC b) noexcept
{
	return squaredDistance(b, a);
}

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(Sphere const& a, AABC b) noexcept { return distance(b, a); }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Sphere const& a, Frustum const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Sphere const& a, Frustum const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Sphere const& a, LineSegment const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Sphere const& a, LineSegment const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Sphere const& a, OBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Sphere const& a, OBB const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Sphere const& a, Plane const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Sphere const& a, Plane const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(Sphere const& a, Point b) noexcept
{
	return distance(b, a);
}

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(Sphere const& a, Point b) noexcept { return distance(b, a); }

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
// constexpr float squaredDistance(Sphere const& a, Ray const& b) noexcept
// {
// 	// FIXME: Enable
// 	return squaredDistance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
// constexpr float distance(Sphere const& a, Ray const& b) noexcept
// {
// 	// FIXME: Enable
// 	return distance(b, a);
// }

/*!
 * @brief Computes the minimum distance between a and b.
 *
 * @note If only the relative distance is of importance, then the squared distance is
 * recommended to use since it is generally faster to compute.
 *
 * @param a,b Geometry objects.
 * @return The minimum distance between a and b.
 */
constexpr float distance(Sphere const& a, Sphere const& b) noexcept
{
	return std::fdim(a.center.distance(b.center), a.radius + b.radius);
}

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
constexpr float squaredDistance(Sphere const& a, Sphere const& b) noexcept
{
	auto dist = distance(a, b);
	return dist * dist;
}

}  // namespace ufo

#endif  // UFO_GEOMETRY_MINIMUM_DISTANCE_HPP