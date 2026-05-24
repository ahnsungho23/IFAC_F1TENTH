#include "readwrite_global_waypoints.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nlohmann/json.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace global_planner
{
using json = nlohmann::json;

namespace
{
std::string get_json_path(const std::string & map_dir) //////////global_waypoints.json" 경로 문자열을 만듬.
{
  return map_dir + "/global_waypoints.json";
}

int32_t get_i32(const json & j, const char * key, int32_t fallback = 0)
{
  if (!j.contains(key)) {
    return fallback;
  }
  return j.at(key).get<int32_t>();
}

double get_f64(const json & j, const char * key, double fallback = 0.0)
{
  if (!j.contains(key)) {
    return fallback;
  }
  return j.at(key).get<double>();
}

std::string get_str(const json & j, const char * key, const std::string & fallback = "")
{
  if (!j.contains(key)) {
    return fallback;
  }
  return j.at(key).get<std::string>();
}

void from_json(const json & j, builtin_interfaces::msg::Time & t)
{
  t.sec = get_i32(j, "sec");
  if (j.contains("nanosec")) {
    t.nanosec = j.at("nanosec").get<uint32_t>();
  } else if (j.contains("nsecs")) {
    t.nanosec = j.at("nsecs").get<uint32_t>();
  } else {
    t.nanosec = 0U;
  }
}

json to_json_time(const builtin_interfaces::msg::Time & t)
{
  return json{{"sec", t.sec}, {"nanosec", t.nanosec}};
}

void from_json(const json & j, std_msgs::msg::Header & h)
{
  if (j.contains("stamp")) {
    from_json(j.at("stamp"), h.stamp);
  }
  h.frame_id = get_str(j, "frame_id");
}

json to_json_header(const std_msgs::msg::Header & h)
{
  return json{{"stamp", to_json_time(h.stamp)}, {"frame_id", h.frame_id}};
}

void from_json(const json & j, geometry_msgs::msg::Point & p)
{
  p.x = get_f64(j, "x");
  p.y = get_f64(j, "y");
  p.z = get_f64(j, "z");
}

json to_json_point(const geometry_msgs::msg::Point & p)
{
  return json{{"x", p.x}, {"y", p.y}, {"z", p.z}};
}

void from_json(const json & j, geometry_msgs::msg::Quaternion & q)
{
  q.x = get_f64(j, "x");
  q.y = get_f64(j, "y");
  q.z = get_f64(j, "z");
  q.w = get_f64(j, "w", 1.0);
}

json to_json_quat(const geometry_msgs::msg::Quaternion & q)
{
  return json{{"x", q.x}, {"y", q.y}, {"z", q.z}, {"w", q.w}};
}

void from_json(const json & j, geometry_msgs::msg::Pose & p)
{
  if (j.contains("position")) {
    from_json(j.at("position"), p.position);
  }
  if (j.contains("orientation")) {
    from_json(j.at("orientation"), p.orientation);
  }
}

json to_json_pose(const geometry_msgs::msg::Pose & p)
{
  return json{{"position", to_json_point(p.position)}, {"orientation", to_json_quat(p.orientation)}};
}

void from_json(const json & j, geometry_msgs::msg::Vector3 & v)
{
  v.x = get_f64(j, "x");
  v.y = get_f64(j, "y");
  v.z = get_f64(j, "z");
}

json to_json_vec3(const geometry_msgs::msg::Vector3 & v)
{
  return json{{"x", v.x}, {"y", v.y}, {"z", v.z}};
}

void from_json(const json & j, std_msgs::msg::ColorRGBA & c)
{
  c.r = static_cast<float>(get_f64(j, "r"));
  c.g = static_cast<float>(get_f64(j, "g"));
  c.b = static_cast<float>(get_f64(j, "b"));
  c.a = static_cast<float>(get_f64(j, "a"));
}

json to_json_color(const std_msgs::msg::ColorRGBA & c)
{
  return json{{"r", c.r}, {"g", c.g}, {"b", c.b}, {"a", c.a}};
}

void from_json(const json & j, f110_msgs::msg::Wpnt & w)
{
  w.id = get_i32(j, "id");
  w.s_m = get_f64(j, "s_m");
  w.d_m = get_f64(j, "d_m");
  w.x_m = get_f64(j, "x_m");
  w.y_m = get_f64(j, "y_m");
  w.d_right = get_f64(j, "d_right");
  w.d_left = get_f64(j, "d_left");
  w.psi_rad = get_f64(j, "psi_rad");
  w.kappa_radpm = get_f64(j, "kappa_radpm");
  w.vx_mps = get_f64(j, "vx_mps");
  w.ax_mps2 = get_f64(j, "ax_mps2");
}

json to_json_wpnt(const f110_msgs::msg::Wpnt & w)
{
  return json{
    {"id", w.id}, {"s_m", w.s_m}, {"d_m", w.d_m}, {"x_m", w.x_m}, {"y_m", w.y_m},
    {"d_right", w.d_right}, {"d_left", w.d_left}, {"psi_rad", w.psi_rad},
    {"kappa_radpm", w.kappa_radpm}, {"vx_mps", w.vx_mps}, {"ax_mps2", w.ax_mps2}
  };
}

void from_json(const json & j, f110_msgs::msg::WpntArray & arr)
{
  if (j.contains("header")) {
    from_json(j.at("header"), arr.header);
  }
  arr.wpnts.clear();
  if (j.contains("wpnts") && j.at("wpnts").is_array()) {
    for (const auto & jw : j.at("wpnts")) {
      f110_msgs::msg::Wpnt w;
      from_json(jw, w);
      arr.wpnts.push_back(w);
    }
  }
}

json to_json_wpnt_array(const f110_msgs::msg::WpntArray & arr)
{
  json jw = json::array();
  for (const auto & w : arr.wpnts) {
    jw.push_back(to_json_wpnt(w));
  }
  return json{{"header", to_json_header(arr.header)}, {"wpnts", jw}};
}

void from_json(const json & j, visualization_msgs::msg::Marker & m)
{
  if (j.contains("header")) {
    from_json(j.at("header"), m.header);
  }
  m.ns = get_str(j, "ns");
  m.id = get_i32(j, "id");
  m.type = get_i32(j, "type");
  m.action = get_i32(j, "action");
  if (j.contains("pose")) {
    from_json(j.at("pose"), m.pose);
  }
  if (j.contains("scale")) {
    from_json(j.at("scale"), m.scale);
  }
  if (j.contains("color")) {
    from_json(j.at("color"), m.color);
  }
  if (j.contains("lifetime")) {
    m.lifetime.sec = get_i32(j.at("lifetime"), "sec");
    if (j.at("lifetime").contains("nanosec")) {
      m.lifetime.nanosec = j.at("lifetime").at("nanosec").get<uint32_t>();
    } else if (j.at("lifetime").contains("nsecs")) {
      m.lifetime.nanosec = j.at("lifetime").at("nsecs").get<uint32_t>();
    }
  }
  m.frame_locked = j.contains("frame_locked") ? j.at("frame_locked").get<bool>() : false;

  m.points.clear();
  if (j.contains("points") && j.at("points").is_array()) {
    for (const auto & jp : j.at("points")) {
      geometry_msgs::msg::Point p;
      from_json(jp, p);
      m.points.push_back(p);
    }
  }

  m.colors.clear();
  if (j.contains("colors") && j.at("colors").is_array()) {
    for (const auto & jc : j.at("colors")) {
      std_msgs::msg::ColorRGBA c;
      from_json(jc, c);
      m.colors.push_back(c);
    }
  }

  m.text = get_str(j, "text");
  m.mesh_resource = get_str(j, "mesh_resource");
  m.mesh_use_embedded_materials =
    j.contains("mesh_use_embedded_materials") ? j.at("mesh_use_embedded_materials").get<bool>() : false;
}

json to_json_marker(const visualization_msgs::msg::Marker & m)
{
  json points = json::array();
  for (const auto & p : m.points) {
    points.push_back(to_json_point(p));
  }
  json colors = json::array();
  for (const auto & c : m.colors) {
    colors.push_back(to_json_color(c));
  }

  return json{
    {"header", to_json_header(m.header)},
    {"ns", m.ns},
    {"id", m.id},
    {"type", m.type},
    {"action", m.action},
    {"pose", to_json_pose(m.pose)},
    {"scale", to_json_vec3(m.scale)},
    {"color", to_json_color(m.color)},
    {"lifetime", {{"sec", m.lifetime.sec}, {"nanosec", m.lifetime.nanosec}}},
    {"frame_locked", m.frame_locked},
    {"points", points},
    {"colors", colors},
    {"text", m.text},
    {"mesh_resource", m.mesh_resource},
    {"mesh_use_embedded_materials", m.mesh_use_embedded_materials}
  };
}

void from_json(const json & j, visualization_msgs::msg::MarkerArray & arr)
{
  arr.markers.clear();
  if (!j.contains("markers") || !j.at("markers").is_array()) {
    return;
  }
  for (const auto & jm : j.at("markers")) {
    visualization_msgs::msg::Marker m;
    from_json(jm, m);
    arr.markers.push_back(m);
  }
}

json to_json_marker_array(const visualization_msgs::msg::MarkerArray & arr)
{
  json out = json::array();
  for (const auto & m : arr.markers) {
    out.push_back(to_json_marker(m));
  }
  return json{{"markers", out}};
}
}  // namespace

bool read_global_waypoints(const std::string & map_dir, GlobalWaypointBundle & out_bundle, std::string & error_msg)
/////////파일이 열리는지 확인
//파일 내용이 비어있지 않은지 확인
//둘 다 통과하면 true 반환
{
  std::ifstream file(get_json_path(map_dir));
  if (!file.is_open()) {
    error_msg = "Could not open global_waypoints.json in map_dir=" + map_dir;
    return false;
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  const auto body = buffer.str();
  if (body.empty()) {
    error_msg = "global_waypoints.json exists but is empty in map_dir=" + map_dir;
    return false;
  }

  try {
    const auto j = json::parse(body);

    out_bundle.map_info_str.data = j.at("map_info_str").at("data").get<std::string>();
    out_bundle.est_lap_time.data = j.at("est_lap_time").at("data").get<float>();
    from_json(j.at("centerline_markers"), out_bundle.centerline_markers);
    from_json(j.at("centerline_waypoints"), out_bundle.centerline_waypoints);
    from_json(j.at("global_traj_markers_iqp"), out_bundle.global_traj_markers_iqp);
    from_json(j.at("global_traj_wpnts_iqp"), out_bundle.global_traj_wpnts_iqp);
    from_json(j.at("global_traj_markers_sp"), out_bundle.global_traj_markers_sp);
    from_json(j.at("global_traj_wpnts_sp"), out_bundle.global_traj_wpnts_sp);
    from_json(j.at("trackbounds_markers"), out_bundle.trackbounds_markers);
  } catch (const std::exception & e) {
    error_msg = "Failed parsing global_waypoints.json: " + std::string(e.what());
    return false;
  }

  error_msg.clear();
  return true;
}

bool write_global_waypoints(const std::string & map_dir, const GlobalWaypointBundle & bundle, std::string & error_msg)
//////////write_global_waypoint실제 데이터 직렬화는 안 함.
//파일을 append 모드로 열 수 있는지만 확인
//열리면 아무것도 쓰지 않고 true 반환
//실패하면 error_msg 설정 후 false
//bundle도 (void)bundle;로 무시됨  --------파일 접근 가능 여부 체크용 함수
{
  std::ofstream file(get_json_path(map_dir));
  if (!file.is_open()) {
    error_msg = "Could not open global_waypoints.json for write in map_dir=" + map_dir;
    return false;
  }

  json j;
  j["map_info_str"] = {{"data", bundle.map_info_str.data}};
  j["est_lap_time"] = {{"data", bundle.est_lap_time.data}};
  j["centerline_markers"] = to_json_marker_array(bundle.centerline_markers);
  j["centerline_waypoints"] = to_json_wpnt_array(bundle.centerline_waypoints);
  j["global_traj_markers_iqp"] = to_json_marker_array(bundle.global_traj_markers_iqp);
  j["global_traj_wpnts_iqp"] = to_json_wpnt_array(bundle.global_traj_wpnts_iqp);
  j["global_traj_markers_sp"] = to_json_marker_array(bundle.global_traj_markers_sp);
  j["global_traj_wpnts_sp"] = to_json_wpnt_array(bundle.global_traj_wpnts_sp);
  j["trackbounds_markers"] = to_json_marker_array(bundle.trackbounds_markers);

  file << j.dump(2);
  error_msg.clear();
  return true;
}

}  // namespace global_planner
