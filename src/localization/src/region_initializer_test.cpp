// Offline state-machine tests: no rclcpp initialization, nodes, TF or ROS traffic.
#include "localization/region_initializer.h"
#include <iostream>
#include <stdexcept>
using namespace kiss_matcher;
namespace {
void check(bool value, const char *message) { if(!value) throw std::runtime_error(message); }
Eigen::Matrix4d pose(double x=0,double y=0,double z=0,double roll=0,double pitch=0,double yaw=0) {
  Eigen::Matrix4d t=Eigen::Matrix4d::Identity();
  t.block<3,3>(0,0)=(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())).toRotationMatrix();
  t.block<3,1>(0,3)=Eigen::Vector3d(x,y,z);return t;
}
LocalizationResult result(const Eigen::Matrix4d &t,double overlap=95) {
  LocalizationResult r;r.valid=r.converged=r.fine_ran=true;r.overlap_percent=overlap;
  r.fine_inliers=100;r.inlier_rmse=0.01;r.target_T_source=t;return r;
}
constexpr std::int64_t sec=1000000000LL;
}
int main(int argc,char **argv) {
 try {
  check(argc==2 || argc==3,"supply synthetic profile and optional initial-reference profile");
  InitializerConfig config;config.confirm_min_dt=1;config.confirm_timeout=10;
  RegionInitializer gate(config);gate.load(argv[1],"map","base_link");
  const auto odom=pose(4,-2,1,0.21,-0.17,0.4);
  gate.begin(1,10*sec,odom);
  check(gate.guesses().size()==2,"all competing seeds must remain searchable");
  const Eigen::Matrix4d map_T_odom=odom.inverse();
  check((gate.guesses()[0].second-map_T_odom).norm()<1e-10,"six DOF initial map->odom conversion");
  auto candidate=[&](const Eigen::Matrix4d &t,const Eigen::Matrix4d &o,double score=95) {
    return gate.evaluate("candidate","seed_vgicp",result(t,score),o,std::nullopt);
  };
  std::vector<InitializationCandidate> list={candidate(map_T_odom,odom),candidate(map_T_odom,odom,94)};
  auto d=gate.decide(list,10*sec,odom);
  check(d.clusters==1 && !d.ready && d.confirmations==1,"same mode seeds merge and wait");
  check(gate.waitForQuery(11*sec)==1,"temporary query wait preserves first confirmation");
  d=gate.decide(list,10*sec,odom);
  check(!d.ready && d.confirmations==1,"repeated stamp cannot confirm");
  const auto moving_odom=odom*pose(0.8,0.1,0,0,0,0.08);
  gate.begin(1,12*sec,moving_odom);
  check((gate.guesses()[0].second-map_T_odom).norm()<1e-10,"moving window must keep first odom reference");
  list={candidate(map_T_odom,moving_odom)};d=gate.decide(list,12*sec,moving_odom);
  check(d.ready && d.confirmations==2,"moving second window confirms same map->odom");

  gate.resetConfirmation();gate.begin(1,13*sec,odom);
  list={candidate(map_T_odom,odom)};d=gate.decide(list,13*sec,odom);
  check(d.confirmations==1,"fresh first window before wait timeout");
  check(gate.waitForQuery(24*sec)==0,"waiting must expire confirmation after timeout");
  gate.begin(1,25*sec,odom);d=gate.decide(list,25*sec,odom);
  check(d.confirmations==1 && !d.ready,"expired wait cannot confirm with only one new window");
  check(gate.waitForQuery(24*sec)==0,"backward clock during wait clears confirmation");

  gate.reset();gate.begin(1,20*sec,odom);
  list={candidate(map_T_odom,odom),candidate(pose(10)*map_T_odom,odom,93)};
  d=gate.decide(list,20*sec,odom);
  check(!d.ready && d.clusters==2 && d.reason=="initialization_ambiguous","different high-score modes must reject");
  check(d.confirmations==0,"ambiguity cannot accumulate confirmations");
  list={candidate(map_T_odom,odom)};d=gate.decide(list,22*sec,odom);
  check(d.confirmations==1,"ambiguity clears previous confirmation");
  gate.begin(1,40*sec,odom);list={candidate(map_T_odom,odom)};d=gate.decide(list,40*sec,odom);
  check(!d.ready && d.confirmations==1 && gate.referenceStamp()==40*sec,"timeout resets references and confirmation");
  gate.begin(1,5*sec,odom);list={candidate(map_T_odom,odom)};d=gate.decide(list,5*sec,odom);
  check(!d.ready && d.confirmations==1 && gate.referenceStamp()==5*sec,"backward time resets references");

  auto bad=candidate(pose(0,0,0,0.9)*map_T_odom,odom);
  check(!bad.eligible && bad.reason=="region_attitude_rejected","region attitude constraint");
  bad=candidate(pose(0,0,5)*map_T_odom,odom);
  check(!bad.eligible && bad.reason=="region_position_rejected","wrong floor constraint");
  bad=gate.evaluate("bad","seed_vgicp",result(pose(2)*map_T_odom),odom,map_T_odom);
  check(!bad.eligible && bad.reason=="seed_correction_exceeded","seed basin correction limit");
  auto r=result(map_T_odom);r.converged=false;
  check(!gate.evaluate("bad","kiss_full",r,odom,std::nullopt).eligible,"KISS must obey convergence gate");
  r=result(map_T_odom);r.inlier_rmse=1;
  check(!gate.evaluate("bad","kiss_full",r,odom,std::nullopt).eligible,"KISS must obey residual gate");
  const auto library_pose=pose(0,0,5)*map_T_odom;
  check(gate.evaluate("kf","keyframe_vgicp",result(library_pose),odom,library_pose,false).eligible,
      "dynamic reference candidate replaces entry box without changing pose");
  r=result(library_pose);r.converged=false;
  check(!gate.evaluate("kf","keyframe_vgicp",r,odom,library_pose,false).eligible,"dynamic mode keeps convergence protection");
  check(!gate.evaluate("kf","keyframe_vgicp",result(pose(2)*library_pose),odom,library_pose,false).eligible,
      "dynamic mode keeps seed correction limit");
  gate.reset();gate.begin(-1,50*sec,odom);
  list={candidate(map_T_odom,odom)};d=gate.decide(list,50*sec,odom);
  check(!d.ready && d.confirmations==1,"full-map KISS still requires confirmation");
  gate.begin(-1,52*sec,moving_odom);list={candidate(map_T_odom,moving_odom)};d=gate.decide(list,52*sec,moving_odom);
  check(d.ready,"full-map KISS confirms through same gate");
  bool invalid=false;try {gate.begin(1,0,odom);}catch(const std::invalid_argument&){invalid=true;}
  check(invalid,"zero snapshot time cannot silently become latest TF");
  invalid=false;try {gate.begin(1,60*sec,Eigen::Matrix4d::Zero());}catch(const std::invalid_argument&){invalid=true;}
  check(invalid,"invalid odom transform must reject");
  // A TF lookup failure is a node concern. Its handler calls this exact reset.
  gate.reset();gate.begin(1,61*sec,odom);list={candidate(map_T_odom,odom)};d=gate.decide(list,61*sec,odom);
  check(!d.ready && d.confirmations==1,"external TF-failure reset discards confirmation history");
  if (argc==3) {
    RegionInitializer anchor(config);anchor.load(argv[2],"map","base_link");
    anchor.begin(1,100*sec,odom);
    const auto moved=odom*pose(0.8,1.2,4.5,0.1,0.2,0.4);
    anchor.begin(1,102*sec,moved);
    auto good=anchor.evaluate("kiss_region","kiss_region",result(map_T_odom),moved,std::nullopt);
    check(good.eligible,"upstairs KISS must pass when back-projected starting pose is inside bounds");
    check(good.map_T_base(2,3)>4,"current output must retain upstairs pose, not reference pose");
    auto seeded=anchor.evaluate("seed","seed_vgicp",result(map_T_odom),moved,map_T_odom);
    check(seeded.eligible,"propagated seed must pass reference bounds");
    auto wrong=anchor.evaluate("kiss_full","kiss_full",result(pose(0,0,3)*map_T_odom),moved,std::nullopt);
    check(!wrong.eligible && wrong.reason=="region_position_rejected","wrong-floor starting pose still rejected after motion");
    std::vector<InitializationCandidate> a={good};
    d=anchor.decide(a,102*sec,moved);check(!d.ready && d.confirmations==1,"still needs second window");
    anchor.resetConfirmation();
    check(anchor.referenceStamp()==100*sec,"expired input must preserve starting reference");
    anchor.begin(1,104*sec,moved);
    a={anchor.evaluate("kiss_full","kiss_full",result(map_T_odom),moved,std::nullopt)};
    d=anchor.decide(a,104*sec,moved);check(!d.ready && d.confirmations==1,"expired result cannot retain confirmation credit");
    anchor.begin(1,106*sec,moved);a={anchor.evaluate("kiss_full","kiss_full",result(map_T_odom),moved,std::nullopt)};
    d=anchor.decide(a,106*sec,moved);check(d.ready,"fresh consistent pair can confirm upstairs");
    anchor.begin(1,130*sec,moved);
    check(anchor.referenceStamp()==100*sec,"long confirmation gap must not redefine entry anchor");
    a={anchor.evaluate("kiss_full","kiss_full",result(map_T_odom),moved,std::nullopt)};
    d=anchor.decide(a,130*sec,moved);check(!d.ready && d.confirmations==1,"long gap clears confirmations");
    anchor.begin(1,90*sec,moved);
    check(anchor.referenceStamp()==90*sec,"backward clock must discard stale anchor");
    anchor.reset();anchor.begin(1,140*sec,moved);
    check(anchor.referenceStamp()==140*sec,"explicit TF/epoch reset must discard anchor");
    std::cout << "PASS: reference bounds permit upstairs seed/KISS, reject displaced starting floor, preserve current pose, clear expired confirmations, retain anchor across timeout and reset on epoch\n";
  }
  std::cout << "PASS: nonzeroRP/changed odom, all-seed propagation, clustering, ambiguity, repeated stamps, moving confirmation, timeout, attitude/floor, KISS gate, invalid TF inputs and reset\n";
  return 0;
 } catch(const std::exception &e) {std::cerr << e.what()<<'\n';return 1;}
}
