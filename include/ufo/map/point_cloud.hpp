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

#ifndef UFO_MAP_POINT_CLOUD_HPP
#define UFO_MAP_POINT_CLOUD_HPP

// UFO
#include <cstring>
#include <initializer_list>
#include <numeric>
#include <ufo/map/color/color.hpp>
#include <ufo/map/intensity/intensity_map.hpp>
#include <ufo/map/point.hpp>
#include <ufo/map/types.hpp>
#include <ufo/math/pose6.hpp>

// LZF
#include <liblzf/lzf.h>

// STL
#include <algorithm>
#include <concepts>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <type_traits>
#include <utility>
#include <vector>

// STL parallel
#ifdef UFO_PARALLEL
#include <execution>
#endif

namespace ufo
{

template <class... Types>
struct CloudElement : Types... {
	constexpr CloudElement() = default;

	constexpr CloudElement(Types const&... types) : Types(types)... {}
};

template <class... Types>
using Cloud = std::vector<CloudElement<Types...>>;

using PointCloud      = Cloud<Point>;
using PointCloudColor = Cloud<Point, Color>;

/*!
 * @brief Transform each point in the point cloud
 *
 * @param transform The transformation to be applied to each point
 */
template <class InputIt, typename T>
void applyTransform(InputIt first, InputIt last, Pose6<T> const& transform)
{
	std::for_each(first, last,
	              [t = transform.translation, r = transform.rotation.rotMatrix()](auto& p) {
		              auto const x = p.x;
		              auto const y = p.y;
		              auto const z = p.z;
		              p.x          = r[0] * x + r[1] * y + r[2] * z + t.x;
		              p.y          = r[3] * x + r[4] * y + r[5] * z + t.y;
		              p.z          = r[6] * x + r[7] * y + r[8] * z + t.z;
	              });
}

template <class PointCloud, typename T>
void applyTransform(PointCloud& cloud, Pose6<T> const& transform)
{
	applyTransform(std::begin(cloud), std::end(cloud), transform);
}

template <class PointCloud>
void removeNaN(PointCloud& cloud)
{
	std::erase_if(cloud, [](auto const& point) {
		return std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z);
	});
}

template <class PointCloud>
void filterDistance(PointCloud& cloud, Point origin, float max_distance)
{
	float sqrt_dist = max_distance * max_distance;
	std::erase_if(cloud, [origin, sqrt_dist](auto const& point) {
		return origin.squaredDistance(point) > sqrt_dist;
	});
}

template <class PointCloud>
void readPointCloudXYZ(std::filesystem::path const& file, PointCloud& cloud)
{
	std::ifstream f;
	f.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	f.imbue(std::locale());
	f.open(file);

	cloud.clear();

	std::string line;
	while (std::getline(f, line)) {
		float x, y, z;

		std::istringstream iss(line);
		if (!(iss >> x >> y >> z)) {
			// TODO: Error
		}

		cloud.emplace_back(x, y, z);
	}
}

template <class PointCloud>
void readPointCloudXYZRGB(std::filesystem::path const& file, PointCloud& cloud)
{
	std::ifstream f;
	f.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	f.imbue(std::locale());
	f.open(file);

	cloud.clear();

	std::string line;
	while (std::getline(f, line)) {
		float x, y, z, r, g, b;

		std::istringstream iss(line);
		if (!(iss >> x >> y >> z >> r >> g >> b)) {
			// TODO: Error
		}

		cloud.emplace_back(x, y, z);
		if constexpr (IsColor<PointCloud>) {
			cloud.back().red   = std::numeric_limits<color_t>::max() * r;
			cloud.back().green = std::numeric_limits<color_t>::max() * g;
			cloud.back().blue  = std::numeric_limits<color_t>::max() * b;
		}
	}
}

template <class PointCloud>
void readPointCloudXYZI(std::filesystem::path const& file, PointCloud& cloud)
{
	std::ifstream f;
	f.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	f.imbue(std::locale());
	f.open(file);

	cloud.clear();

	std::string line;
	while (std::getline(f, line)) {
		float x, y, z, i;

		std::istringstream iss(line);
		if (!(iss >> x >> y >> z >> i)) {
			// TODO: Error
		}

		cloud.emplace_back(x, y, z);
		if constexpr (IsIntensity<PointCloud>) {
			cloud.back().intensity = i;
		}
	}
}

template <class PointCloud>
void readPointCloudXYZIRGB(std::filesystem::path const& file, PointCloud& cloud)
{
	std::ifstream f;
	f.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	f.imbue(std::locale());
	f.open(file);

	cloud.clear();

	std::string line;
	while (std::getline(f, line)) {
		float x, y, z, i, r, g, b;

		std::istringstream iss(line);
		if (!(iss >> x >> y >> z >> i >> r >> g >> b)) {
			// TODO: Error
		}

		cloud.emplace_back(x, y, z);
		if constexpr (IsColor<PointCloud>) {
			cloud.back().red   = std::numeric_limits<color_t>::max() * r;
			cloud.back().green = std::numeric_limits<color_t>::max() * g;
			cloud.back().blue  = std::numeric_limits<color_t>::max() * b;
		}
		if constexpr (IsIntensity<PointCloud>) {
			cloud.back().intensity = i;
		}
	}
}

template <class PointCloud>
void readPointCloudPLY(std::filesystem::path const& file, PointCloud& cloud)
{
	// TODO: Implement
}

namespace impl
{
enum class PCDName { UNKNOWN, X, Y, Z, INTENSITY, RGB, LABEL, VALUE };

enum class PCDType { I, U, F };

enum class PCDDataType { ASCII, BINARY, BINARY_COMPRESSED };

struct PCDPointField {
	PCDName     name{PCDName::UNKNOWN};
	std::size_t size;
	std::size_t count{1};
	PCDType     type;
};

struct PCDHeader {
 public:
	std::vector<PCDPointField> fields;
	std::size_t                width;
	std::size_t                height;
	PCDDataType                datatype;
};

std::string toLower(std::string str)
{
	std::transform(std::begin(str), std::end(str), std::begin(str),
	               [](auto c) { return std::tolower(c); });
	return str;
}

std::vector<std::string> splitString(std::string const& str, std::string const& splitters)
{
	std::vector<std::string> result;
	auto it = std::find_if_not(std::cbegin(str), std::cend(str), [&splitters](auto c) {
		return std::any_of(std::cbegin(splitters), std::cend(splitters),
		                   [c](auto e) { return c == e; });
	});
	for (auto last = std::cend(str); it != last;) {
		auto cur = it;
		it       = std::find_if(it, last, [&splitters](auto c) {
      return std::any_of(std::cbegin(splitters), std::cend(splitters),
			                         [c](auto e) { return c == e; });
    });
		result.emplace_back(cur, it);
		it = std::find_if_not(it, last, [&splitters](auto c) {
			return std::any_of(std::cbegin(splitters), std::cend(splitters),
			                   [c](auto e) { return c == e; });
		});
	}
	return result;
}

template <typename T>
T unpackASCIIPCDElement(std::string const& str, PCDType type, std::size_t size)
{
	if constexpr (std::same_as<Color, T>) {
		if (4 != size) {
			return Color();
		}

		std::uint8_t data[3];
		switch (type) {
			case PCDType::I: {
				std::int32_t v = std::stol(str);
				std::memcpy(data, &v, 3);
			}
			case PCDType::U: {
				std::uint32_t v = std::stoul(str);
				std::memcpy(data, &v, 3);
			}
			case PCDType::F: {
				float v = std::stof(str);
				std::memcpy(data, &v, 3);
			}
		}
		return Color(data[2], data[1], data[0]);
	} else {
		switch (type) {
			case PCDType::I: return static_cast<T>(std::stoll(str));
			case PCDType::U: return static_cast<T>(std::stoull(str));
			case PCDType::F: return static_cast<T>(std::stod(str));
		}
	}

	return T();
}

template <typename T>
T unpackBinaryPCDElement(char const* data, PCDType type, std::size_t size)
{
	if constexpr (std::same_as<Color, T>) {
		if (4 != size) {
			return Color();
		}

		std::uint8_t d[3];
		std::memcpy(d, data, 3);
		return Color(data[2], data[1], data[0]);
	} else {
		switch (type) {
			case PCDType::I:
				switch (size) {
					case 1: {
						std::int8_t d;
						std::memcpy(&d, data, sizeof(d));
						return static_cast<T>(d);
					}
					case 2: {
						std::int16_t d;
						std::memcpy(&d, data, sizeof(d));
						return static_cast<T>(d);
					}
					case 4: {
						std::int32_t d;
						std::memcpy(&d, data, sizeof(d));
						return static_cast<T>(d);
					}
					case 8: {
						std::int64_t d;
						std::memcpy(&d, data, sizeof(d));
						return static_cast<T>(d);
					}
					default: return T();
				}
			case PCDType::U:
				switch (size) {
					case 1: {
						std::uint8_t d;
						std::memcpy(&d, data, sizeof(d));
						return static_cast<T>(d);
					}
					case 2: {
						std::uint16_t d;
						std::memcpy(&d, data, sizeof(d));
						return static_cast<T>(d);
					}
					case 4: {
						std::uint32_t d;
						std::memcpy(&d, data, sizeof(d));
						return static_cast<T>(d);
					}
					case 8: {
						std::uint64_t d;
						std::memcpy(&d, data, sizeof(d));
						return static_cast<T>(d);
					}
					default: return T();
				}
			case PCDType::F:
				switch (size) {
					case 4: {
						float d;
						std::memcpy(&d, data, sizeof(d));
						return static_cast<T>(d);
					}
					case 8: {
						double d;
						std::memcpy(&d, data, sizeof(d));
						return static_cast<T>(d);
					}
					default: return T();
				}
		}
	}

	return T();
}
}  // namespace impl

template <class PointCloud>
void readPointCloudPCD(std::filesystem::path const& filename, PointCloud& cloud,
                       Pose6f& viewpoint)
{
	std::ifstream file;
	file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	file.imbue(std::locale());
	file.open(filename, std::ios_base::in | std::ios_base::binary);

	impl::PCDHeader header;

	std::string line;
	while (std::getline(file, line)) {
		if ('#' == line[0]) {
			// Skip comments
			continue;
		} else if (line.starts_with("VERSION")) {
			// Do not care about the version
			continue;
		} else if (line.starts_with("FIELDS")) {
			std::istringstream iss(line.substr(7));
			std::string        s;
			std::size_t        index{};
			while (iss >> s) {
				if (header.fields.size() == index) {
					header.fields.emplace_back();
				}

				s = impl::toLower(s);

				if ("x" == s) {
					header.fields[index].name = impl::PCDName::X;
				} else if ("y" == s) {
					header.fields[index].name = impl::PCDName::Y;
				} else if ("z" == s) {
					header.fields[index].name = impl::PCDName::Z;
				} else if ("intensity" == s) {
					header.fields[index].name = impl::PCDName::INTENSITY;
				} else if ("rgb" == s) {
					header.fields[index].name = impl::PCDName::RGB;
				} else if ("label" == s) {
					header.fields[index].name = impl::PCDName::LABEL;
				} else if ("value" == s) {
					header.fields[index].name = impl::PCDName::VALUE;
				} else {
					header.fields[index].name = impl::PCDName::UNKNOWN;
				}
				++index;
			}
		} else if (line.starts_with("SIZE")) {
			std::istringstream iss(line.substr(5));
			std::size_t        s;
			std::size_t        index{};
			while (iss >> s) {
				if (header.fields.size() == index) {
					header.fields.emplace_back();
				}

				header.fields[index].size = s;
				++index;
			}
		} else if (line.starts_with("TYPE")) {
			std::istringstream iss(line.substr(5));
			char               s;
			std::size_t        index{};
			while (iss >> s) {
				if (header.fields.size() == index) {
					header.fields.emplace_back();
				}

				if ('I' == s) {
					header.fields[index].type = impl::PCDType::I;
				} else if ('U' == s) {
					header.fields[index].type = impl::PCDType::U;
				} else if ('F' == s) {
					header.fields[index].type = impl::PCDType::F;
				}
				++index;
			}
		} else if (line.starts_with("COUNT")) {
			std::istringstream iss(line.substr(6));
			std::size_t        s;
			std::size_t        index{};
			while (iss >> s) {
				if (header.fields.size() == index) {
					header.fields.emplace_back();
				}

				header.fields[index].count = s;
				++index;
			}
		} else if (line.starts_with("WIDTH")) {
			std::istringstream iss(line.substr(6));
			if (!(iss >> header.width)) {
				// TODO: Error
			}
		} else if (line.starts_with("HEIGHT")) {
			std::istringstream iss(line.substr(7));
			if (!(iss >> header.height)) {
				// TODO: Error
			}
		} else if (line.starts_with("VIEWPOINT")) {
			std::istringstream iss(line.substr(10));
			if (!(iss >> viewpoint.x() >> viewpoint.y() >> viewpoint.z() >> viewpoint.qw() >>
			      viewpoint.qx() >> viewpoint.qy() >> viewpoint.qz())) {
				// TODO: Error
			}
		} else if (line.starts_with("POINTS")) {
			// Skip since we can get this from width * height
		} else if (line.starts_with("DATA")) {
			auto data = line.substr(5);  // FIXME: Trim
			if ("ascii" == data) {
				header.datatype = impl::PCDDataType::ASCII;
			} else if ("binary" == data) {
				header.datatype = impl::PCDDataType::BINARY;
			} else if ("binary_compressed" == data) {
				header.datatype = impl::PCDDataType::BINARY_COMPRESSED;
			} else {
				// TODO: Handle error
			}
			break;  // Everything after is data
		}
	}

	// TODO: Check header

	std::size_t point_size{};
	for (auto const& f : header.fields) {
		point_size += f.size * f.count;
	}

	cloud.clear();
	cloud.resize(header.width * header.height);

	switch (header.datatype) {
		case impl::PCDDataType::ASCII: {
			for (auto& p : cloud) {
				if (!std::getline(file, line)) {
					// TODO: Handle error
				}

				auto s = impl::splitString(line, "\t\r\n ");

				if (s.size() != header.fields.size()) {
					// TODO: Handle error
				}

				for (std::size_t i{}; auto const& field : header.fields) {
					if (1 != field.count) {
						i += field.count;
						continue;  // Cannot handle anything other than one count
					}

					switch (field.name) {
						case impl::PCDName::X:
							p.x = impl::unpackASCIIPCDElement<coord_t>(s[i], field.type, field.size);
							break;
						case impl::PCDName::Y:
							p.y = impl::unpackASCIIPCDElement<coord_t>(s[i], field.type, field.size);
							break;
						case impl::PCDName::Z:
							p.z = impl::unpackASCIIPCDElement<coord_t>(s[i], field.type, field.size);
							break;
						case impl::PCDName::RGB:
							if constexpr (IsColor<PointCloud>) {
								static_cast<Color&>(p) =
								    impl::unpackASCIIPCDElement<Color>(s[i], field.type, field.size);
							}
							break;
						case impl::PCDName::INTENSITY:
							if constexpr (IsIntensity<PointCloud>) {
								p.intensity = impl::unpackASCIIPCDElement<intensity_t>(s[i], field.type,
								                                                       field.size);
							}
							break;
						case impl::PCDName::LABEL:
							if constexpr (IsLabel<PointCloud>) {
								p.label =
								    impl::unpackASCIIPCDElement<label_t>(s[i], field.type, field.size);
							}
							break;
						case impl::PCDName::VALUE:
							if constexpr (IsValue<PointCloud>) {
								p.value =
								    impl::unpackASCIIPCDElement<value_t>(s[i], field.type, field.size);
							}
							break;
						case impl::PCDName::UNKNOWN: break;
					}

					i += field.count;
				}
			}
			break;
		}
		case impl::PCDDataType::BINARY: {
			std::unique_ptr<char[]> buffer(new char[point_size]);
			for (auto& p : cloud) {
				if (!file.read(buffer.get(), point_size)) {
					// TODO: Handle error
				}

				for (auto data = buffer.get(); auto const& field : header.fields) {
					if (1 != field.count) {
						data += field.count * field.size;
						continue;  // Cannot handle anything other than one count
					}

					switch (field.name) {
						case impl::PCDName::X:
							p.x = impl::unpackBinaryPCDElement<coord_t>(data, field.type, field.size);
							break;
						case impl::PCDName::Y:
							p.y = impl::unpackBinaryPCDElement<coord_t>(data, field.type, field.size);
							break;
						case impl::PCDName::Z:
							p.z = impl::unpackBinaryPCDElement<coord_t>(data, field.type, field.size);
							break;
						case impl::PCDName::RGB:
							if constexpr (IsColor<PointCloud>) {
								static_cast<Color&>(p) =
								    impl::unpackBinaryPCDElement<Color>(data, field.type, field.size);
							}
							break;
						case impl::PCDName::INTENSITY:
							if constexpr (IsIntensity<PointCloud>) {
								p.intensity = impl::unpackBinaryPCDElement<intensity_t>(data, field.type,
								                                                        field.size);
							}
							break;
						case impl::PCDName::LABEL:
							if constexpr (IsLabel<PointCloud>) {
								p.label =
								    impl::unpackBinaryPCDElement<label_t>(data, field.type, field.size);
							}
							break;
						case impl::PCDName::VALUE:
							if constexpr (IsValue<PointCloud>) {
								p.value =
								    impl::unpackBinaryPCDElement<value_t>(data, field.type, field.size);
							}
							break;
						case impl::PCDName::UNKNOWN: break;
					}

					data += field.count * field.size;
				}
			}
			break;
		}
		case impl::PCDDataType::BINARY_COMPRESSED: {
			// TODO: Implement
			break;
		}
	}
}

template <class PointCloud>
void readPointCloudPCD(std::filesystem::path const& file, PointCloud& cloud)
{
	Pose6f pose;
	readPointCloudPCD(file, cloud, pose);
}

template <class PointCloud>
void readPointCloudPTS(std::filesystem::path const& filename, PointCloud& cloud)
{
	std::ifstream file;
	file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	file.imbue(std::locale());
	file.open(filename);

	std::string line;
	if (!std::getline(file, line)) {
		// TODO: Handle error
	}

	std::istringstream iss(line);

	std::size_t num_points;
	if (!(iss >> num_points)) {
		// TODO: Handle error
	}

	cloud.clear();
	cloud.resize(num_points);

	for (auto& p : cloud) {
		if (!std::getline(file, line)) {
			// TODO: Handle error
		}

		iss = line;

		if (!(iss >> p.x >> p.y >> p.z)) {
			// TODO: Handle error
		}

		if constexpr (IsIntensity<PointCloud>) {
			if (!(iss >> p.intensity)) {
				// TODO: Handle error
			}
		}

		if constexpr (IsColor<PointCloud>) {
			if (!(iss >> p.red >> p.green >> p.blue)) {
				// TODO: Handle error
			}
		}
	}
}

template <class PointCloud>
void readPointCloud(std::filesystem::path const& file, PointCloud& cloud)
{
	// FIXME: Make lower case
	auto ext = file.extension().string();
	if (".xyz" == ext) {
		return readPointCloudXYZ(file, cloud);
	} else if (".xyzi" == ext) {
		return readPointCloudXYZI(file, cloud);
	} else if (".xyzrgb" == ext) {
		return readPointCloudXYZRGB(file, cloud);
	} else if (".xyzirgb" == ext) {
		return readPointCloudXYZIRGB(file, cloud);
	} else if (".ply" == ext) {
		return readPointCloudPLY(file, cloud);
	} else if (".pcd" == ext) {
		return readPointCloudPCD(file, cloud);
	} else if (".pts" == ext) {
		return readPointCloudPTS(file, cloud);
	} else {
		// TODO: Cannot read file format
	}
}

template <class PointCloud>
void writePointCloudXYZ(std::filesystem::path const& file, PointCloud const& cloud)
{
	std::ofstream f;
	f.exceptions(std::ofstream::failbit | std::ofstream::badbit);
	f.imbue(std::locale());
	f.open(file, std::ios_base::out | std::ios_base::binary);

	f << std::fixed << std::setprecision(10);

	for (auto const& p : cloud) {
		f << p.x << ' ' << p.y << ' ' << p.z << '\n';
	}
}

template <class PointCloud>
void writePointCloudXYZI(std::filesystem::path const& file, PointCloud const& cloud)
{
	std::ofstream f;
	f.exceptions(std::ofstream::failbit | std::ofstream::badbit);
	f.imbue(std::locale());
	f.open(file, std::ios_base::out | std::ios_base::binary);

	f << std::fixed << std::setprecision(10);

	for (auto const& p : cloud) {
		f << p.x << ' ' << p.y << ' ' << p.z;
		if constexpr (IsIntensity<PointCloud>) {
			f << ' ' << p.intensity << '\n';
		} else {
			f << " 0\n";
		}
	}
}

template <class PointCloud>
void writePointCloudXYZRGB(std::filesystem::path const& file, PointCloud const& cloud)
{
	std::ofstream f;
	f.exceptions(std::ofstream::failbit | std::ofstream::badbit);
	f.imbue(std::locale());
	f.open(file, std::ios_base::out | std::ios_base::binary);

	f << std::fixed << std::setprecision(10);

	for (auto const& p : cloud) {
		f << p.x << ' ' << p.y << ' ' << p.z;
		if constexpr (IsColor<PointCloud>) {
			f << ' '
			  << (static_cast<double>(p.red) /
			      static_cast<double>(std::numeric_limits<color_t>::max()))
			  << ' '
			  << (static_cast<double>(p.green) /
			      static_cast<double>(std::numeric_limits<color_t>::max()))
			  << ' '
			  << (static_cast<double>(p.blue) /
			      static_cast<double>(std::numeric_limits<color_t>::max()))
			  << '\n';
		} else {
			f << " 0.0 0.0 0.0\n";
		}
	}
}

template <class PointCloud>
void writePointCloudXYZIRGB(std::filesystem::path const& file, PointCloud const& cloud)
{
	std::ofstream f;
	f.exceptions(std::ofstream::failbit | std::ofstream::badbit);
	f.imbue(std::locale());
	f.open(file, std::ios_base::out | std::ios_base::binary);

	f << std::fixed << std::setprecision(10);

	for (auto const& p : cloud) {
		f << p.x << ' ' << p.y << ' ' << p.z << ' ';
		if constexpr (IsIntensity<PointCloud>) {
			f << p.intensity;
		} else {
			f << '0';
		}
		if constexpr (IsColor<PointCloud>) {
			f << ' '
			  << (static_cast<double>(p.red) /
			      static_cast<double>(std::numeric_limits<color_t>::max()))
			  << ' '
			  << (static_cast<double>(p.green) /
			      static_cast<double>(std::numeric_limits<color_t>::max()))
			  << ' '
			  << (static_cast<double>(p.blue) /
			      static_cast<double>(std::numeric_limits<color_t>::max()))
			  << '\n';
		} else {
			f << " 0.0 0.0 0.0\n";
		}
	}
}

template <class PointCloud>
void writePointCloudPLY(std::filesystem::path const& file, PointCloud const& cloud)
{
	// TODO: Implement
}

template <class PointCloud>
void writePointCloudPCD(std::filesystem::path const& file, PointCloud const& cloud,
                        Pose6f const& viewpoint = Pose6f(), bool ascii = false,
                        bool compressed = false)
{
	std::ofstream f;
	f.exceptions(std::ofstream::failbit | std::ofstream::badbit);
	f.imbue(std::locale());
	f.open(file, std::ios_base::out | std::ios_base::binary);

	f << "# .PCD v0.7 - Point Cloud Data file format\n";
	f << "VERSION 0.7\n";
	f << "FIELDS x y z";
	if constexpr (IsColor<PointCloud>) {
		f << " rgb";
	}
	if constexpr (IsIntensity<PointCloud>) {
		f << " intensity";
	}
	if constexpr (IsLabel<PointCloud>) {
		f << " label";
	}
	if constexpr (IsValue<PointCloud>) {
		f << " value";
	}
	f << '\n';

	f << "SIZE 4 4 4";
	if constexpr (IsColor<PointCloud>) {
		f << " 4";
	}
	if constexpr (IsIntensity<PointCloud>) {
		f << " 4";
	}
	if constexpr (IsLabel<PointCloud>) {
		f << " 4";
	}
	if constexpr (IsValue<PointCloud>) {
		f << " 4";
	}
	f << '\n';

	f << "TYPE F F F";
	if constexpr (IsColor<PointCloud>) {
		f << " F";
	}
	if constexpr (IsIntensity<PointCloud>) {
		f << " F";
	}
	if constexpr (IsLabel<PointCloud>) {
		f << " U";
	}
	if constexpr (IsValue<PointCloud>) {
		f << " F";
	}
	f << '\n';

	f << "COUNT 1 1 1";
	if constexpr (IsColor<PointCloud>) {
		f << " 1";
	}
	if constexpr (IsIntensity<PointCloud>) {
		f << " 1";
	}
	if constexpr (IsLabel<PointCloud>) {
		f << " 1";
	}
	if constexpr (IsValue<PointCloud>) {
		f << " 1";
	}
	f << '\n';

	f << "WIDTH " << cloud.size() << '\n';
	f << "HEIGHT 1\n";
	// clang-format off
	f << "VIEWPOINT " << viewpoint.x()  << ' ' 
										<< viewpoint.y()  << ' ' 
										<< viewpoint.z()  << ' ' 
										<< viewpoint.qw() << ' ' 
										<< viewpoint.qx() << ' ' 
										<< viewpoint.qy() << ' ' 
										<< viewpoint.qz() << '\n';
	// clang-format on
	f << "POINTS " << cloud.size() << '\n';

	if (ascii) {
		f << "DATA ascii\n";
		f << std::fixed << std::setprecision(10);
		for (auto const& p : cloud) {
			f << p.x << ' ' << p.y << ' ' << p.z;
			if constexpr (IsColor<PointCloud>) {
				std::uint32_t rgb = (p.red << 16) | (p.green << 8) | p.blue;
				f << ' ' << rgb;
			}
			if constexpr (IsIntensity<PointCloud>) {
				f << ' ' << p.intensity;
			}
			if constexpr (IsLabel<PointCloud>) {
				f << ' ' << p.label;
			}
			if constexpr (IsValue<PointCloud>) {
				f << ' ' << p.value;
			}
			f << '\n';
		}
	} else if (compressed) {
		f << "DATA binary_compressed\n";

		// std::unique_ptr<char[]> buffer(new char[buffer_size]);
		// std::unique_ptr<char[]> buffer_compressed(new char[buffer_size * 2]);

		// lzf_compress()
		// TODO: Implement
	} else {
		f << "DATA binary\n";
		// TODO: Will this always be correct?
		std::copy(reinterpret_cast<char const*>(cloud.data()),
		          reinterpret_cast<char const*>(cloud.data() + cloud.size()),
		          std::ostream_iterator<char>(f));
	}
}

template <class PointCloud>
void writePointCloudPTS(std::filesystem::path const& file, PointCloud const& cloud)
{
	std::ofstream f;
	f.exceptions(std::ofstream::failbit | std::ofstream::badbit);
	f.imbue(std::locale());
	f.open(file, std::ios_base::out | std::ios_base::binary);

	f << std::fixed << std::setprecision(10);

	f << cloud.size() << '\n';

	// x y z intensity red green blue
	// It is not specified what intensity should be (see
	// https://www.danielgm.net/cc/forum/viewtopic.php?t=1307)
	for (auto const& p : cloud) {
		f << p.x << ' ' << p.y << ' ' << p.z << ' ';
		if constexpr (IsIntensity<PointCloud>) {
			f << p.intensity;
		} else {
			f << '0';
		}
		if constexpr (IsColor<PointCloud>) {
			f << ' ' << +p.red << ' ' << +p.green << ' ' << +p.blue << '\n';
		} else {
			f << " 0 0 0\n";
		}
	}
}

template <class PointCloud>
void writePointCloud(std::filesystem::path const& file, PointCloud const& cloud)
{
	// FIXME: Make lower case
	auto ext = file.extension().string();
	if (".xyz" == ext) {
		return writePointCloudXYZ(file, cloud);
	} else if (".xyzi" == ext) {
		return writePointCloudXYZI(file, cloud);
	} else if (".xyzrgb" == ext) {
		return writePointCloudXYZRGB(file, cloud);
	} else if (".xyzirgb" == ext) {
		return writePointCloudXYZIRGB(file, cloud);
	} else if (".ply" == ext) {
		return writePointCloudPLY(file, cloud);
	} else if (".pcd" == ext) {
		return writePointCloudPCD(file, cloud);
	} else if (".pts" == ext) {
		return writePointCloudPTS(file, cloud);
	} else {
		// TODO: Cannot read file format
	}
}

#ifdef UFO_PARALLEL
/*!
 * @brief Transform each point in the point cloud
 *
 * @param transform The transformation to be applied to each point
 */
template <class ExecutionPolicy, class InputIt, typename T>
  requires std::is_execution_policy_v<std::remove_cvref_t<ExecutionPolicy>>
void applyTransform(ExecutionPolicy&& policy, InputIt first, InputIt last,
                    Pose6<T> const& transform)
{
	std::for_each(std::forward<ExecutionPolicy>(policy), first, last,
	              [t = transform.translation, r = transform.rotation.rotMatrix()](auto& p) {
		              auto const x = p.x;
		              auto const y = p.y;
		              auto const z = p.z;
		              p.x          = r[0] * x + r[1] * y + r[2] * z + t.x;
		              p.y          = r[3] * x + r[4] * y + r[5] * z + t.y;
		              p.z          = r[6] * x + r[7] * y + r[8] * z + t.z;
	              });
}

template <class ExecutionPolicy, class PointCloud, typename T>
  requires std::is_execution_policy_v<std::remove_cvref_t<ExecutionPolicy>>
void applyTransform(ExecutionPolicy&& policy, PointCloud& cloud,
                    Pose6<T> const& transform)
{
	applyTransform(std::forward<ExecutionPolicy>(policy), std::begin(cloud),
	               std::end(cloud), transform);
}

template <class ExecutionPolicy, class PointCloud>
  requires std::is_execution_policy_v<std::remove_cvref_t<ExecutionPolicy>>
void removeNaN(ExecutionPolicy&& policy, PointCloud& cloud)
{
	auto it = std::remove_if(std::forward<ExecutionPolicy>(policy), std::begin(cloud),
	                         std::end(cloud), [](auto const& point) {
		                         return std::isnan(point.x) || std::isnan(point.y) ||
		                                std::isnan(point.z);
	                         });
	cloud.erase(it, std::end(cloud));
}

template <class ExecutionPolicy, class PointCloud>
  requires std::is_execution_policy_v<std::remove_cvref_t<ExecutionPolicy>>
void filterDistance(ExecutionPolicy&& policy, PointCloud& cloud, Point origin,
                    float max_distance)
{
	float sqrt_dist = max_distance * max_distance;
	auto  it = std::remove_if(std::forward<ExecutionPolicy>(policy), std::begin(cloud),
	                          std::end(cloud), [origin, sqrt_dist](auto const& point) {
                             return origin.squaredDistance(point) > sqrt_dist;
                           });
	cloud.erase(it, std::end(cloud));
}
#endif
}  // namespace ufo

#endif  // UFO_MAP_POINT_CLOUD_HPP