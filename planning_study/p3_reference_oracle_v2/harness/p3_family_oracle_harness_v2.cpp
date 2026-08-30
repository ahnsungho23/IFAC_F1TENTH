// Audit-only deterministic evaluator for the frozen production P3 family.
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "local_planning/raceline_spline_planner.hpp"
#include "local_planning/research_instrumentation.hpp"

namespace {
using local_planning::EgoFrenetState;
using local_planning::P3ShadowCandidateTrace;
using local_planning::PlanningResearchCycle;
using local_planning::RacelineSplineParameters;
using local_planning::RacelineSplinePlanner;

struct Event {
  std::string id, bag, context;
  std::uint64_t callback{0};
  std::size_t invocation{0};
  EgoFrenetState ego;
  f110_msgs::msg::WpntArray reference;
  std::vector<f110_msgs::msg::Obstacle> obstacles;
};
struct Request {
  std::string phase;
  bool relaxed{false}, left{false};
  double target{0}, middle{0}, entry{0}, exit{0};
};

std::vector<std::string> split(const std::string & line) {
  std::vector<std::string> out;
  std::size_t begin=0;
  for (;;) {
    const auto end=line.find('\t',begin);
    out.push_back(line.substr(begin,end==std::string::npos?end:end-begin));
    if (end==std::string::npos) return out;
    begin=end+1;
  }
}
double num(const std::string & s) {
  std::size_t n=0; const double x=std::stod(s,&n);
  if(n!=s.size()||!std::isfinite(x)) throw std::runtime_error("bad number "+s);
  return x;
}
long long integer(const std::string & s) {
  std::size_t n=0; const auto x=std::stoll(s,&n);
  if(n!=s.size()) throw std::runtime_error("bad integer "+s);
  return x;
}

RacelineSplineParameters params() {
  RacelineSplineParameters p;
  p.detection_lookahead_m=15.0; p.obstacle_cluster_gap_m=0.8;
  p.obstacle_longitudinal_padding_m=0.0; p.vehicle_length_m=0.56;
  p.vehicle_half_width_m=0.15; p.safety_margin_m=0.08;
  p.obstacle_reserve_from_lut=false; p.tracking_error_reserve_m=0.0;
  p.tracking_error_lut_speed_bins_mps={0,1,1.6,2.2,2.9,4.5,7};
  p.tracking_error_lut_curvature_bins_radpm={0,0.1,0.2,0.35,0.46};
  p.tracking_error_lut_values_m={
    .095,.095,.095,.130,.130,.100,.165,.165,.165,.165,
    .100,.165,.165,.165,.165,.145,.165,.190,.230,.230,
    .275,.275,.390,.390,.390,.275,.275,.390,.390,.390,
    .275,.275,.390,.390,.390};
  p.avoidance_velocity_limit_speed_bins_mps={0,1,2,3,4,5,6,7,8,9};
  p.avoidance_velocity_limit_lateral_accel_mps2={7.6,7.6,7.6,7.6,7,7,7,6.5,6.5,6.5};
  p.avoidance_velocity_limit_accel_mps2={3.7,3.7,3.7,3.7,3.7,3.47,3.33,3,3,3};
  p.avoidance_velocity_limit_decel_mps2={2,2,2,2,2,2,2,2,2,2};
  p.longitudinal_launch_speed_floor_mps=1.0;
  p.handoff_speed_shaping_enable=true;
  p.confirmed_obstacle_speed_envelope_enable=true;
  p.confirmed_speed_post_hold_distance_m=1.0;
  p.confirmed_speed_response_delay_sec=.15;
  p.analytic_path_geometry_enable=true;
  p.avoidance_minimum_speed_mps=1.0; p.wall_safety_margin_m=.04;
  p.fallback_track_half_width_m=1.5; p.margin_pass_speed_cap_mps=2.0;
  p.approach_feasibility_decel_mps2=2.0; p.approach_feasibility_decel_max_mps2=3.5;
  p.profile_feasibility_decel_mps2=3.5; p.commitment_retention_reserve_fraction=.5;
  p.localization_reserve_m=0.0;
  p.pre_apex_distances_m={11.442220427651225,7.628146951767484,3.814073475883742};
  p.post_apex_distances_m={2.059509950005119,4.119019900010238,6.178529850015357};
  p.entry_transition_fractions={.5145810930150512,.75,1.0};
  p.transition_distance_scales={.4971684162574945,.6991537701867223,3.698773101198193};
  p.outside_line_transition_scale=.4060036444074003;
  p.maximum_exit_length_m=0; p.post_merge_lookahead_m=5; p.post_merge_min_time_sec=1;
  p.merge_ramp_min_length_m=0; p.merge_ramp_time_sec=0;
  p.minimum_target_offset_m=.15; p.maximum_target_offset_m=1.5;
  p.target_d_candidate_count=5; p.maximum_lateral_slope=.8;
  p.entry_discontinuity_min_budget_m=.2; p.entry_continuity_baseline_m=.5;
  p.maximum_curvature_radpm=1.316266519079011; p.maximum_curvature_rate_radpm2=20;
  p.control_wheelbase_m=.33; p.control_max_steering_left_rad=.410;
  p.control_max_steering_right_rad=.361;
  p.control_understeer_gradient_left_rad_per_mps2=.014;
  p.control_understeer_gradient_right_rad_per_mps2=.019;
  p.control_max_steering_rate_radps=20;
  p.safe_stop_buffer_m=2.6; p.safe_stop_deceleration_mps2=1.8;
  p.raw_slowdown_post_hold_distance_m=1; p.minimum_path_points=8;
  p.safe_stop_escape_check_enable=true; p.safe_stop_escape_retreat_step_m=.3;
  p.safe_stop_escape_max_retreats=8;
  return p;
}

Event readEvent(const std::filesystem::path & path) {
  std::ifstream in(path); std::string line;
  if(!in||!std::getline(in,line)||line!="P3_ORACLE_EVENT_V1")
    throw std::runtime_error("unsupported event");
  Event e; e.reference.header.frame_id="map";
  while(std::getline(in,line)) {
    const auto t=split(line);
    if(t[0]=="EVENT"&&t.size()==11) {
      e.id=t[1]; e.bag=t[2]; e.callback=integer(t[3]); e.invocation=integer(t[4]);
      e.context=t[5]; e.ego.s=num(t[6]); e.ego.d=num(t[7]); e.ego.speed=num(t[8]);
    } else if(t[0]=="W"&&t.size()==12) {
      f110_msgs::msg::Wpnt w; w.id=integer(t[1]); w.s_m=num(t[2]); w.d_m=num(t[3]);
      w.x_m=num(t[4]); w.y_m=num(t[5]); w.d_right=num(t[6]); w.d_left=num(t[7]);
      w.psi_rad=num(t[8]); w.kappa_radpm=num(t[9]); w.vx_mps=num(t[10]);
      w.ax_mps2=num(t[11]); e.reference.wpnts.push_back(w);
    } else if(t[0]=="O"&&t.size()==12) {
      f110_msgs::msg::Obstacle o; o.id=integer(t[1]); o.s_center=num(t[2]);
      o.s_start=num(t[3]); o.s_end=num(t[4]); o.d_right=num(t[5]); o.d_left=num(t[6]);
      o.size=num(t[7]); o.s_var=num(t[8]); o.d_var=num(t[9]);
      o.is_static=integer(t[10])!=0; o.is_visible=integer(t[11])!=0;
      o.d_center=.5*(o.d_right+o.d_left); e.obstacles.push_back(o);
    } else if(t[0]=="END_EVENT") break;
  }
  if(e.id.empty()||e.reference.wpnts.empty()) throw std::runtime_error("incomplete event");
  return e;
}

std::vector<Request> readRequests(const std::filesystem::path & path) {
  std::ifstream in(path); std::string line;
  if(!in||!std::getline(in,line)||line!="P3_ORACLE_REQUESTS_V1")
    throw std::runtime_error("unsupported requests");
  std::vector<Request> out;
  while(std::getline(in,line)) {
    const auto t=split(line); if(t.size()!=8||t[0]!="Q") continue;
    out.push_back({t[1],integer(t[2])!=0,integer(t[3])!=0,
      num(t[4]),num(t[5]),num(t[6]),num(t[7])});
  }
  return out;
}
std::string clean(std::string s) { for(char &c:s) if(c=='\t'||c=='\n'||c=='\r') c=' '; return s; }

void context(const Event &e,const RacelineSplinePlanner &p) {
  std::cout<<"CONTEXT\tevent_id\tgate\tvalid\toutside_is_left\tside\tside_valid\tcluster_start\tcluster_end\tminimum_target\tmaximum_target\treason\n";
  for(bool relaxed:{false,true}) {
    const auto c=p.inspectP3OracleContext(e.ego,e.obstacles,relaxed);
    for(const auto &o:c.visible) {
      std::cout<<"VISIBLE\t"<<e.id<<'\t'<<(relaxed?"RELAXED":"STRICT")<<'\t'
        <<o.id<<'\t'<<o.start<<'\t'<<o.end<<'\t'<<o.center<<'\t'
        <<o.d_right<<'\t'<<o.d_left<<'\n';
    }
    for(bool left:{false,true}) {
      const auto &d=left?c.left:c.right;
      std::cout<<"CONTEXT\t"<<e.id<<'\t'<<(relaxed?"RELAXED":"STRICT")<<'\t'
        <<c.valid<<'\t'<<c.outside_is_left<<'\t'<<(left?"LEFT":"RIGHT")<<'\t'
        <<d.valid<<'\t'<<d.cluster_start<<'\t'<<d.cluster_end<<'\t'
        <<d.minimum_target<<'\t'<<d.maximum_target<<'\t'
        <<clean(d.valid?c.reason:d.reason)<<'\n';
    }
  }
}

void candidate(const Event&e,std::size_t i,const Request&q,const P3ShadowCandidateTrace&t) {
  const auto&v=t.validation;
  std::cout<<"CANDIDATE\t"<<e.id<<'\t'<<i<<'\t'<<q.phase<<'\t'
    <<(q.relaxed?"RELAXED":"STRICT")<<'\t'<<(q.left?"LEFT":"RIGHT")<<'\t'
    <<q.target<<'\t'<<q.middle<<'\t'<<q.entry<<'\t'<<q.exit;
  for(double z:t.knot_stations) std::cout<<'\t'<<z;
  std::cout<<'\t'<<t.point_count<<'\t'<<t.validator_executed<<'\t'<<t.hard_valid<<'\t'
    <<clean(t.rejection_reason)<<'\t'<<v.first_failure_kind<<'\t'<<v.failure_waypoint_index
    <<'\t'<<v.failure_obstacle_id<<'\t'<<v.minimum_center_track_margin_m
    <<'\t'<<t.minimum_track_margin_m<<'\t'<<t.minimum_obstacle_margin_m
    <<'\t'<<t.waypoint0_center_track_margin_m
    <<'\t'<<t.waypoint0_footprint_track_margin_m
    <<'\t'<<t.peak_lateral_slope<<'\t'<<t.lateral_slope_margin
    <<'\t'<<v.peak_positive_curvature_radpm<<'\t'<<v.peak_negative_curvature_radpm
    <<'\t'<<t.minimum_curvature_margin_radpm<<'\t'<<t.peak_curvature_rate_radpm2
    <<'\t'<<t.curvature_rate_margin_radpm2<<'\t'<<t.ego_braking_distance_deficit_m
    <<'\t'<<t.velocity_loss<<'\t'<<t.minimum_normalized_safety_slack
    <<'\t'<<t.global_path_deviation_m<<'\t'<<t.exit_reaches_next_obstacle
    <<'\t'<<t.path_digest<<'\t'<<clean(v.failure_footprint_side)
    <<'\t'<<v.failure_heading_relative_rad<<'\t'<<v.failure_corner_protrusion_m<<'\n';
}

void writePath(const std::filesystem::path&path,const Event&e,const Request&q,
 const P3ShadowCandidateTrace&t) {
  std::ofstream o(path); o<<std::setprecision(17)
   <<"event_id\tphase\tgate\tside\td_target\td_mid\tentry_scale\texit_scale\tindex\ts_m\td_m\tx_m\ty_m\tpsi_rad\tkappa_radpm\tvx_mps\tax_mps2\n";
  for(std::size_t i=0;i<t.path.wpnts.size();++i){const auto&w=t.path.wpnts[i];
   o<<e.id<<'\t'<<q.phase<<'\t'<<(q.relaxed?"RELAXED":"STRICT")<<'\t'
    <<(q.left?"LEFT":"RIGHT")<<'\t'<<q.target<<'\t'<<q.middle<<'\t'<<q.entry
    <<'\t'<<q.exit<<'\t'<<i<<'\t'<<w.s_m<<'\t'<<w.d_m<<'\t'<<w.x_m
    <<'\t'<<w.y_m<<'\t'<<w.psi_rad<<'\t'<<w.kappa_radpm<<'\t'<<w.vx_mps
    <<'\t'<<w.ax_mps2<<'\n';}
}
}

int main(int argc,char**argv){
 if(argc<3||argc>4){std::cerr<<"usage: harness EVENT REQUESTS [PATH]\n";return 2;}
 try{
  const auto e=readEvent(argv[1]); const auto qs=readRequests(argv[2]);
  RacelineSplinePlanner p(params()); std::string error;
  if(!p.setReference(e.reference,&error)) throw std::runtime_error(error);
  std::cout<<std::setprecision(17); context(e,p);
  PlanningResearchCycle research_cycle;
  research_cycle.obstacle_sequence=1U;
  research_cycle.source_epoch=2U;
  research_cycle.reference_generation=1U;
  p.setActiveResearchCycle(&research_cycle);
  const auto baseline=p.evaluateP3Shadow(e.ego,e.obstacles,0,0,1,"ORACLE_PARITY");
  p.setActiveResearchCycle(nullptr);
  std::cout<<"BASELINE\t"<<e.id<<'\t'<<baseline.would_recover<<'\t'
    <<baseline.candidate_count<<'\t'<<baseline.hard_validator_call_count<<'\t'
    <<baseline.hard_valid_count<<'\t'<<clean(baseline.failure_classification)<<'\n';
  std::cout<<"BASELINE_CANDIDATE\tevent_id\tindex\tgenerator_stage\tside\td_target\td_mid\tentry_scale\texit_scale\thard_valid\trejection_reason\tpath_digest\n";
  for(std::size_t i=0;i<baseline.candidates.size();++i){const auto&t=baseline.candidates[i];
    std::cout<<"BASELINE_CANDIDATE\t"<<e.id<<'\t'<<i<<'\t'<<t.generator_stage<<'\t'
      <<(t.go_left?"LEFT":"RIGHT")<<'\t'<<t.d_target<<'\t'<<t.d_mid<<'\t'
      <<t.entry_scale<<'\t'<<t.exit_scale<<'\t'<<t.hard_valid<<'\t'
      <<clean(t.rejection_reason)<<'\t'<<t.path_digest<<'\n';}
  std::cout<<"BASELINE_CONSTRUCTED_CANDIDATE\tevent_id\tindex\tgenerator_stage\tside\td_target\td_mid\td_probe\ts_probe\tentry_scale\texit_scale\treturned_by_policy\tdiscarded_side\thard_valid\tpath_digest\n";
  for(std::size_t i=0;i<baseline.research_all_candidates.size();++i){const auto&t=baseline.research_all_candidates[i];
    std::cout<<"BASELINE_CONSTRUCTED_CANDIDATE\t"<<e.id<<'\t'<<i<<'\t'<<t.generator_stage<<'\t'
      <<(t.go_left?"LEFT":"RIGHT")<<'\t'<<t.d_target<<'\t'<<t.d_mid<<'\t'
      <<t.d_probe<<'\t'<<t.s_probe<<'\t'<<t.entry_scale<<'\t'<<t.exit_scale<<'\t'
      <<t.returned_by_policy<<'\t'<<t.discarded_side<<'\t'<<t.hard_valid<<'\t'
      <<t.path_digest<<'\n';}
  std::cout<<"CANDIDATE\tevent_id\trequest_index\tphase\tgate\tside\td_target\td_mid\tentry_scale\texit_scale\tz0\tz1\tz2\tz3\tz4\tpoint_count\tvalidator_executed\thard_valid\tfirst_failure_reason\tfirst_failure_enum\tfailure_waypoint_index\tfailure_obstacle_id\tcenter_track_margin_m\tfootprint_track_margin_m\tobstacle_margin_m\twaypoint0_center_track_margin_m\twaypoint0_footprint_track_margin_m\tpeak_lateral_slope\tlateral_slope_margin\tpeak_positive_curvature_radpm\tpeak_negative_curvature_radpm\tsigned_curvature_margin_radpm\tpeak_curvature_rate_radpm2\tcurvature_rate_margin_radpm2\tbraking_deficit_m\tvelocity_loss\tminimum_normalized_safety_slack\tglobal_path_deviation_m\texit_reaches_next_obstacle\tpath_digest\tfailure_footprint_side\tfailure_heading_relative_rad\tfailure_corner_protrusion_m\n";
  for(std::size_t i=0;i<qs.size();++i){const auto&q=qs[i];
   const auto t=p.evaluateP3OracleCandidate(e.ego,e.obstacles,q.left,q.target,q.middle,q.entry,q.exit,q.relaxed);
   candidate(e,i,q,t); if(argc==4&&qs.size()==1) writePath(argv[3],e,q,t);}
  return 0;
 }catch(const std::exception&e){std::cerr<<"P3 oracle harness failed: "<<e.what()<<'\n';return 1;}
}
