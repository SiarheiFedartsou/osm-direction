#include <iomanip>
#include <iostream>
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <fstream>
#include <nlohmann/json.hpp>
#include <osmium/geom/haversine.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>
#include <osmium/index/map/flex_mem.hpp>
#include <osmium/io/any_input.hpp>
#include <osmium/visitor.hpp>
#include <osmium/util/progress_bar.hpp>

struct Coordinate {
  double x{};
  double y{};

  osmium::geom::Coordinates asOSMCoord() const {
    return osmium::geom::Coordinates(x, y);
  }
};

using ObjectID = unsigned long long;

struct WayNode {
  ObjectID id;
  Coordinate coordinate;
};

enum class OnewayDirection { None, Forward, Backward };

struct Way {
  std::vector<WayNode> nodes;
  OnewayDirection oneway_direction = OnewayDirection::None;
};

enum class Direction {
    Forward,
    Backward,
};

struct ProposedChange {
    ObjectID node_id;
    Coordinate node_coordinate;
    Direction direction;
    double vis_bearing_deg{};
};

struct DataCollector : public osmium::handler::Handler {
  void way(const osmium::Way &way) {
    bool has_stop_sign = false;

    Way internal_way;

    const char *oneway = way.tags()["oneway"];
    if (oneway) {
      if (strcmp(oneway, "yes") == 0) {
        internal_way.oneway_direction = OnewayDirection::Forward;
      } else if (strcmp(oneway, "-1") == 0) {
        internal_way.oneway_direction = OnewayDirection::Backward;
      }
    }
    for (const auto &node : way.nodes()) {
      ++intersections[node.ref()];
      if (stop_signs.contains(node.ref())) {
        has_stop_sign = true;
      }
      internal_way.nodes.push_back(
          {static_cast<ObjectID>(node.ref()),
           {node.location().lon(), node.location().lat()}});
    }
    if (has_stop_sign) {
      ways_with_stop_signs.emplace_back(std::move(internal_way));
    }
  }

  void node(const osmium::Node &node) {
    const char *highway = node.tags()["highway"];
    if (highway && strcmp(highway, "stop") == 0) {
      if (!node.tags()["direction"] && !node.tags()["stop:direction"]) {
        stop_signs.insert(node.id());
      }
    }
  }

  std::unordered_map<ObjectID, size_t> intersections;
  std::unordered_set<ObjectID> stop_signs;
  std::vector<Way> ways_with_stop_signs;
};

std::vector<ProposedChange> GenerateChanges(const DataCollector &data) {
  std::vector<ProposedChange> changes;
  for (const auto &way : data.ways_with_stop_signs) {

    if (way.oneway_direction == OnewayDirection::None) {
      double distance_from_intersection_or_stop_sign = 0;
      bool after_stop_sign = false;
      ObjectID current_stop_sign = 0;
      Coordinate current_stop_sign_coordinate;
      double distance_to_prev_intersection = 0;

      // TODO: what if there is no intersection before/after the stop sign?
      for (size_t index = 0; index < way.nodes.size(); ++index) {
        const auto &node = way.nodes[index];
        if (index > 0) {
          distance_from_intersection_or_stop_sign +=
              osmium::geom::haversine::distance(
                  way.nodes[index - 1].coordinate.asOSMCoord(),
                  node.coordinate.asOSMCoord());
        }

        bool is_intersection = data.intersections.at(node.id) > 1;
        bool is_stop_sign =
            !is_intersection && data.stop_signs.contains(node.id);
        if (is_stop_sign) {
          after_stop_sign = true;
          current_stop_sign = node.id;
          current_stop_sign_coordinate = node.coordinate;
          distance_to_prev_intersection =
              distance_from_intersection_or_stop_sign;
          distance_from_intersection_or_stop_sign = 0;
        }

        if (is_intersection) {
          if (after_stop_sign) {
            ProposedChange change;
            change.node_coordinate = current_stop_sign_coordinate;
            change.node_id = current_stop_sign;

            const auto distance_to_next_intersection =
                distance_from_intersection_or_stop_sign;
            if (distance_to_next_intersection < distance_to_prev_intersection) {
              change.direction = Direction::Forward;
            } else {
              change.direction = Direction::Backward;
            }
            after_stop_sign = false;
            distance_from_intersection_or_stop_sign = 0;
            changes.push_back(change);
          } else {
            distance_from_intersection_or_stop_sign = 0;
          }
        }
      }
    } else {
        for (size_t index = 0; index < way.nodes.size(); ++index) {
            const auto &node = way.nodes[index];
            bool is_intersection = data.intersections.at(node.id) > 1;
            bool is_stop_sign =
                !is_intersection && data.stop_signs.contains(node.id);
            if (is_stop_sign) {
                ProposedChange change;
                change.node_id = node.id;
                change.node_coordinate = node.coordinate;
                if (way.oneway_direction == OnewayDirection::Forward) {
                    change.direction = Direction::Forward;
                } else if (way.oneway_direction == OnewayDirection::Backward) {
                    change.direction = Direction::Backward;
                } else {
                    assert(false && "unreachable");
                }
                changes.push_back(change);
            }
        }
    }
  }
  return changes;
}

void RenderChangesToGeoJSON(const std::vector<ProposedChange>& changes, const std::string& filename) {
    using json = nlohmann::json;
    std::ofstream file(filename);

    for (const auto& change : changes) {
        json j_change;
        j_change["type"] = "FeatureCollection";
        j_change["features"] = json::array();

        json j_stop_sign;
        j_stop_sign["type"] = "Feature";
        j_stop_sign["geometry"] = json::object();
        j_stop_sign["geometry"]["type"] = "Point";
        j_stop_sign["geometry"]["coordinates"] = json::array();
        j_stop_sign["geometry"]["coordinates"][0] = change.node_coordinate.x;
        j_stop_sign["geometry"]["coordinates"][1] = change.node_coordinate.y;
        j_stop_sign["properties"] = json::object();
        j_stop_sign["properties"]["osmid"] = change.node_id;
        j_stop_sign["properties"]["direction"] = change.direction == Direction::Forward ? "forward" : "backward";
        j_change["features"].push_back(j_stop_sign);
  
        json j_cooperative_work = json::object();
        j_cooperative_work["meta"] = json::object();
        j_cooperative_work["meta"]["version"] = 2;
        j_cooperative_work["meta"]["type"] = 1;
            
        json j_operation = json::object();
        j_operation["operationType"] = "modifyElement";
        j_operation["data"] = json::object();
        j_operation["data"]["id"] = "node/" + std::to_string(change.node_id);
        j_operation["data"]["operations"] = json::array();
        j_operation["data"]["operations"].push_back(json::object());
        j_operation["data"]["operations"][0]["operation"] = "setTags";
        j_operation["data"]["operations"][0]["data"] = json::object();
        j_operation["data"]["operations"][0]["data"]["direction"] = change.direction == Direction::Forward ? "forward" : "backward";


        j_cooperative_work["operations"] = json::array();
        j_cooperative_work["operations"].push_back(j_operation);

        j_change["cooperativeWork"] = std::move(j_cooperative_work);

        constexpr char RS = static_cast<char>(0x1E);
        file << RS << j_change.dump() << '\n';
    }
}

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " OSMFILE\n";
    return 1;
  }

  try {
    osmium::io::Reader reader{argv[1], osmium::osm_entity_bits::node |
                                           osmium::osm_entity_bits::way};
    osmium::ProgressBar progress{reader.file_size(), osmium::isatty(2)};

    using Index = osmium::index::map::FlexMem<osmium::unsigned_object_id_type,
                                              osmium::Location>;
    using LocationHandler = osmium::handler::NodeLocationsForWays<Index>;

    Index index;
    LocationHandler location_handler{index};

    DataCollector data_collector;

    while (osmium::memory::Buffer buffer = reader.read()) {
        osmium::apply(buffer, location_handler, data_collector);
        progress.update(reader.offset());
    }

    progress.done();

    
    auto changes = GenerateChanges(data_collector);
    RenderChangesToGeoJSON(changes, "changes.geojson");
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
