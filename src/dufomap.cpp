// UFO
#include <ufo/map/integration/integration.hpp>
#include <ufo/map/integration/integration_parameters.hpp>
#include <ufo/map/node.hpp>
#include <ufo/map/point.hpp>
#include <ufo/map/point_cloud.hpp>
#include <ufo/map/points/points_predicate.hpp>
#include <ufo/map/predicate/satisfies.hpp>
#include <ufo/map/predicate/spatial.hpp>
#include <ufo/map/types.hpp>
#include <ufo/map/ufomap.hpp>
#include <ufo/math/pose6.hpp>
#include <ufo/util/timing.hpp>

// TOML
#include "toml.hpp"
#include "indicators.hpp"

// STL
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <future>
#include <ios>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

#ifdef UFO_PARALLEL
// STL
#include <execution>
#endif

struct Dataset {
	std::size_t first = 0;
	std::size_t last  = -1;
	std::size_t num   = -1;
};

struct Map {
	ufo::node_size_t resolution = 0.1;  // In meters
	ufo::depth_t     levels     = 17;   // Levels of the octree
};

struct Clustering {
	bool         cluster      = false;
	float        max_distance = 0.2f;
	std::size_t  min_points   = 0;
	ufo::depth_t depth        = 0;
};

struct Printing {
	bool verbose = true;
	bool debug   = false;
};

struct Output {
	std::string filename   = "output";
	bool        has_color  = false;
	bool        raycasting = false;
};

struct Config {
	Dataset                dataset;
	Map                    map;
	ufo::IntegrationParams integration;
	bool                   propagate = false;
	Clustering             clustering;
	Printing               printing;
	Output                 output;

	void read(toml::table tbl)
	{
		map.resolution = read(tbl["important"]["resolution"], map.resolution);
		
		dataset.first = read(tbl["dataset"]["first"], dataset.first);
		dataset.last  = read(tbl["dataset"]["last"], dataset.last);
		dataset.num   = read(tbl["dataset"]["num"], dataset.num);
		map.levels     = read(tbl["map"]["levels"], map.levels);

		auto dsm = read(tbl["integration"]["down_sampling_method"], std::string("none"));
		integration.down_sampling_method =
		    "none" == dsm
		        ? ufo::DownSamplingMethod::NONE
		        : ("centroid" == dsm ? ufo::DownSamplingMethod::CENTROID
		                             : ("uniform" == dsm ? ufo::DownSamplingMethod::UNIFORM
		                                                 : ufo::DownSamplingMethod::CENTER));
		integration.hit_depth = read(tbl["integration"]["hit_depth"], integration.hit_depth);
		integration.miss_depth =
		    read(tbl["integration"]["miss_depth"], integration.miss_depth);
		integration.min_range = read(tbl["integration"]["min_range"], integration.min_range);
		integration.max_range = read(tbl["integration"]["max_range"], integration.max_range);
		integration.inflate_unknown =
		    read(tbl["important"]["inflate_unknown"], integration.inflate_unknown);
		integration.inflate_unknown_compensation =
		    read(tbl["integration"]["inflate_unknown_compensation"],
		         integration.inflate_unknown_compensation);
		integration.ray_passthrough_hits = read(tbl["integration"]["ray_passthrough_hits"],
		                                        integration.ray_passthrough_hits);
		integration.inflate_hits_dist =
		    read(tbl["important"]["inflate_hits_dist"], integration.inflate_hits_dist);
		integration.ray_casting_method = read(tbl["integration"]["simple_ray_casting"], true)
		                                     ? ufo::RayCastingMethod::SIMPLE
		                                     : ufo::RayCastingMethod::PROPER;
		integration.simple_ray_casting_factor =
		    read(tbl["integration"]["simple_ray_casting_factor"],
		         integration.simple_ray_casting_factor);
		integration.parallel = tbl["integration"]["parallel"].value_or(integration.parallel);
		integration.num_threads =
		    read(tbl["integration"]["num_threads"], integration.num_threads);
		integration.only_valid =
		    read(tbl["integration"]["only_valid"], integration.only_valid);
			
		propagate = read(tbl["integration"]["propagate"], propagate);

		clustering.cluster = read(tbl["clustering"]["cluster"], clustering.cluster);
		clustering.max_distance =
		    read(tbl["clustering"]["max_distance"], clustering.max_distance);
		clustering.min_points = read(tbl["clustering"]["min_points"], clustering.min_points);
		clustering.depth      = read(tbl["clustering"]["depth"], clustering.depth);

		printing.verbose = read(tbl["printing"]["verbose"], printing.verbose);
		printing.debug   = read(tbl["printing"]["debug"], printing.debug);

		output.filename   = read(tbl["output"]["filename"], output.filename);
		output.has_color  = read(tbl["output"]["has_color"], output.has_color);
		output.raycasting = read(tbl["output"]["raycasting"], output.raycasting);
	}

	void save() const
	{
		// TODO: Implement
	}

 private:
	template <typename T>
	std::remove_cvref_t<T> read(toml::node_view<toml::node> node, T&& default_value)
	{
		// if (!node.is_value()) {
		// 	node.as_array()->push_back("MISSING");
		// 	missing_config = true;
		// 	std::cout << node << '\n';
		// 	return default_value;
		// }

		return node.value_or(default_value);
	}

 private:
	bool missing_config{false};
	bool wrong_config{false};
};

std::ostream& operator<<(std::ostream& out, Config const& config)
{
	out << "Config\n";

	out << "\tDataset\n";
	out << "\t\tFirst: " << config.dataset.first << '\n';
	out << "\t\tLast:  ";
	if (-1 == config.dataset.last) {
		out << -1 << '\n';
	} else {
		out << config.dataset.last << '\n';
	}
	out << "\t\tNum:   ";
	if (-1 == config.dataset.num) {
		out << -1 << '\n';
	} else {
		out << config.dataset.num << '\n';
	}

	out << "\tMap\n";
	out << "\t\tResolution: " << config.map.resolution << '\n';
	out << "\t\tLevels:     " << +config.map.levels << '\n';

	out << "\tIntegration\n";
	out << "\t\tDown sampling method:        "
	    << (ufo::DownSamplingMethod::NONE == config.integration.down_sampling_method
	            ? "none"
	            : (ufo::DownSamplingMethod::CENTER ==
	                       config.integration.down_sampling_method
	                   ? "center"
	                   : (ufo::DownSamplingMethod::CENTROID ==
	                              config.integration.down_sampling_method
	                          ? "centroid"
	                          : "uniform")))
	    << '\n';
	out << "\t\tHit depth:                   " << +config.integration.hit_depth << '\n';
	out << "\t\tMiss depth:                  " << +config.integration.miss_depth << '\n';
	out << "\t\tMin range:                   " << config.integration.min_range << '\n';
	out << "\t\tMax range:                   " << config.integration.max_range << '\n';
	out << "\t\tInflate unknown              " << config.integration.inflate_unknown
	    << '\n';
	out << "\t\tInflate unknown compensation "
	    << config.integration.inflate_unknown_compensation << '\n';
	out << "\t\tRay passthrough hits         " << config.integration.ray_passthrough_hits
	    << '\n';
	out << "\t\tInflate hits dist            " << config.integration.inflate_hits_dist
	    << '\n';
	out << "\t\tEarly stop distance:         " << config.integration.early_stop_distance
	    << '\n';
	out << "\t\tSimple ray casting:          " << std::boolalpha
	    << (ufo::RayCastingMethod::SIMPLE == config.integration.ray_casting_method ? true
	                                                                               : false)
	    << '\n';
	out << "\t\tSimple ray casting factor:   "
	    << config.integration.simple_ray_casting_factor << '\n';
	out << "\t\tParallel:                    " << config.integration.parallel << '\n';
	out << "\t\tNum threads:                 " << config.integration.num_threads << '\n';
	out << "\t\tOnly valid:                  " << config.integration.only_valid << '\n';
	out << "\t\tSliding window size:         " << config.integration.sliding_window_size
	    << '\n';
	out << "\t\tPropagate:                   " << std::boolalpha << config.propagate
	    << '\n';

	out << "\tClustering\n";
	out << "\t\tCluster:      " << std::boolalpha << config.clustering.cluster << '\n';
	out << "\t\tMax distance: " << config.clustering.max_distance << '\n';
	out << "\t\tMin points:   " << config.clustering.min_points << '\n';
	out << "\t\tDepth:        " << +config.clustering.depth << '\n';

	out << "\tPrinting\n";
	out << "\t\tVerbose: " << std::boolalpha << config.printing.verbose << '\n';
	out << "\t\tDebug:   " << std::boolalpha << config.printing.debug << '\n';

	out << "\tOutput\n";
	out << "\t\tFilename:   " << config.output.filename << '\n';
	out << "\t\tHas color:  " << std::boolalpha << config.output.has_color << '\n';
	out << "\t\tRaycasting: " << config.output.raycasting << '\n';

	return out;
}

Config readConfig(std::filesystem::path path)
{
	Config config;
	for (;;) {
		if (std::filesystem::exists(path)) {
			toml::table tbl;
			try {
				tbl = toml::parse_file((path).string());
			} catch (toml::parse_error const& err) {
				std::cerr << "Configuration parsing failed:\n" << err << '\n';
				exit(1);
			}

			config.read(tbl);
			if (config.printing.verbose) {
				std::cout << "Found: " << (path) << '\n';
			}

			break;
		}
		if (!path.has_parent_path()) {
			std::cout << "Did not find configuration file, using default.\n";
			break;
		}
		path = path.parent_path();
	}

	if (config.printing.verbose) {
		std::cout << config << '\n';
	}

	return config;
}

ufo::Color randomColor()
{
	static std::random_device                          rd;
	static std::mt19937                                gen(rd());
	static std::uniform_int_distribution<ufo::color_t> dis(0, -1);
	return {dis(gen), dis(gen), dis(gen)};
}

template <class Map>
void cluster(Map& map, Clustering const& clustering)
{
	std::unordered_set<ufo::Node> seen;
	std::vector<ufo::Sphere>      queue;

	auto depth        = clustering.depth;
	auto max_distance = clustering.max_distance;
	auto min_points   = clustering.min_points;

	ufo::label_t l{1};
	for (auto node : map.query(ufo::pred::Leaf(depth) && ufo::pred::SeenFree() &&
	                           ufo::pred::HitsMin(1) && ufo::pred::Label(0))) {
		if (map.label(node.index())) {  // FIXME: This is because how the iterator works
			continue;
		}

		seen = {node};
		queue.assign(1, ufo::Sphere(map.center(node), max_distance));

		map.setLabel(node, l, false);

		while (!queue.empty()) {
			auto p = ufo::pred::Intersects(queue);
			queue.clear();
			for (auto const& node : map.query(
			         ufo::pred::Leaf(depth) && ufo::pred::SeenFree() && ufo::pred::HitsMin(1) &&
			         ufo::pred::Label(0) && std::move(p) &&
			         ufo::pred::Satisfies([&seen](auto n) { return seen.insert(n).second; }))) {
				queue.emplace_back(map.center(node), max_distance);
				map.setLabel(node, l, false);
			}
		}

		if (seen.size() < min_points) {
			for (auto e : seen) {
				if (l == map.label(e)) {
					map.setLabel(e, -1, false);
				}
			}
		}

		l += seen.size() >= min_points;

		map.propagateModified();  // FIXME: Should this be here?
	}
}

template <class PointCloud>
void filterminDistance(PointCloud& cloud, ufo::Point origin, float min_distance)
{
	float sqrt_dist = min_distance * min_distance;
	std::erase_if(cloud, [origin, sqrt_dist](auto const& point) {
		return origin.squaredDistance(point) < sqrt_dist;
	});
}

int main(int argc, char* argv[])
{
	if (1 >= argc) {
		std::cout << "[ERROR] Please running by: " << argv[0] << " [pcd_folder] [optional: config_file_path]";
		return 0;
	}

	std::filesystem::path path(argv[1]);
	std::string config_file_path;
	if (argc > 2)
		config_file_path = argv[2];
	else
		config_file_path = std::string(argv[1]) + "/dufomap.toml";

	auto config = readConfig(std::filesystem::path(config_file_path));
	std::cout << "[LOG] Step 1: Successfully read configuration from: " << config_file_path <<std::endl;
	ufo::Map<ufo::MapType::SEEN_FREE | ufo::MapType::REFLECTION | ufo::MapType::LABEL> map(
	    config.map.resolution, config.map.levels);
	map.reserve(100'000'000);

	std::vector<std::filesystem::path> pcds;
	for (const auto& entry : std::filesystem::directory_iterator(path / "pcd")) {
		if (!entry.is_regular_file()) {
			continue;
		}
		std::size_t i = std::stoul(entry.path().stem());
		if (config.dataset.first <= i && config.dataset.last >= i) {
			pcds.push_back(entry.path().filename());
		}
	}
	std::ranges::sort(pcds);
	// std::cout << config << std::endl;
	pcds.resize(std::min(pcds.size(), config.dataset.num));

	ufo::Timing timing;
	timing.start("Total");

	ufo::PointCloudColor cloud_acc;

	std::cout << "[LOG] Step 2: Starting Processing data from: " << path << '\n';
	indicators::show_console_cursor(false);
	indicators::BlockProgressBar bar{
		indicators::option::BarWidth{50},
		indicators::option::Start{"["},
		indicators::option::End{"]"},
		indicators::option::PrefixText{"[LOG] Running dufomap "},
		indicators::option::ForegroundColor{indicators::Color::white},
		indicators::option::ShowElapsedTime{true},
		indicators::option::ShowRemainingTime{true},
		indicators::option::FontStyles{std::vector<indicators::FontStyle>{indicators::FontStyle::bold}}
	};
	
	for (std::size_t i{}; std::string filename : pcds) {
		bar.set_progress(100 * i / pcds.size());
		++i;
		timing.setTag("Total " + std::to_string(i) + " of " + std::to_string(pcds.size()) +
		              " (" + std::to_string(100 * i / pcds.size()) + "%)");

		ufo::PointCloudColor cloud;
		ufo::Pose6f          viewpoint;
		timing[1].start("Read");
		ufo::readPointCloudPCD(path / "pcd" / filename, cloud, viewpoint);
		timing[1].stop();

		// NOTE(Qingwen): lots of user facing about ego points inside data, we filterminDistance around ego agent.
		filterminDistance(cloud, viewpoint.translation, config.integration.min_range);

		cloud_acc.insert(std::end(cloud_acc), std::cbegin(cloud), std::cend(cloud));

		ufo::insertPointCloud(map, cloud, viewpoint.translation, config.integration,
		                      config.propagate);

		if (config.printing.verbose) {
			timing[2] = config.integration.timing;
			timing.print(true, true, 2, 4);
		}
	}
	indicators::show_console_cursor(true);
	std::cout << "\033[0m\n[LOG] Step 3: Finished Processing data. Start saving map... " << std::endl;
	// bar.is_completed();
	if (!config.propagate) {
		timing[3].start("Propagate");
		map.propagateModified();
		timing[3].stop();
	}

	timing[4].start("Cluster");
	if (config.clustering.cluster) {
		cluster(map, config.clustering);
	}
	timing[4].stop();

	timing[5].start("Query");
	ufo::PointCloudColor cloud_static;

	for (auto& p : cloud_acc) {
		if (!map.seenFree(p))
			cloud_static.push_back(p);
	}

	timing[5].stop();

	timing[6].start("write");
	ufo::writePointCloudPCD(path / (config.output.filename + ".pcd"), cloud_static);
	timing[6].stop();
	timing.stop();

	timing[2] = config.integration.timing;
	timing.print(true, true, 2, 4);
	std::cout << "[LOG]: Finished! ^v^.. Clean output map with " << cloud_static.size() << " points save in " << path / (config.output.filename + ".pcd") << '\n';
}