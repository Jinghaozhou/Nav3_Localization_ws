#pragma once
// Experimental position-group screening. Coarse scores are ranking only, never acceptance.
#include <algorithm>
#include <numeric>
#include <set>
#include <pcl/filters/voxel_grid.h>
#include "localization/region_initializer.h"
namespace kiss_matcher {
struct SearchGroup { std::vector<size_t> members, representatives; double score=-1; };
inline std::vector<SearchGroup> positionGroups(
    const std::vector<std::pair<std::string,Eigen::Matrix4d>>& seeds,
    const Eigen::Matrix4d& odom, double radius=1.0) {
  std::vector<SearchGroup> groups; std::vector<size_t> order(seeds.size());std::iota(order.begin(),order.end(),0);
  // Stable map order, independent of descriptor retrieval rank.
  std::sort(order.begin(),order.end(),[&](size_t a,size_t b){return seeds[a].first<seeds[b].first;});
  auto pose=[&](size_t i)->Eigen::Matrix4d{return seeds[i].second*odom;};
  for(auto i:order) {
    const auto p=pose(i);int chosen=-1;double best=radius;
    for(size_t g=0;g<groups.size();++g) {
      const auto c=pose(groups[g].members.front());double d=(p.block<3,1>(0,3)-c.block<3,1>(0,3)).norm();
      if(d<=best && std::abs(p(2,3)-c(2,3))<=.5){best=d;chosen=g;}
    }
    if(chosen<0){groups.push_back({});chosen=groups.size()-1;}groups[chosen].members.push_back(i);
  }
  for(auto &g:groups) {
    auto medoid=g.members.front();double best=1e100;
    for(auto i:g.members){double sum=0;for(auto j:g.members)sum+=(pose(i).block<3,1>(0,3)-pose(j).block<3,1>(0,3)).squaredNorm();if(sum<best){best=sum;medoid=i;}}
    g.representatives.push_back(medoid);
    for(auto i:g.members){bool different=true;for(auto j:g.representatives)if(initializationRotationDegrees(pose(i).block<3,3>(0,0).transpose()*pose(j).block<3,3>(0,0))<45)different=false;if(different)g.representatives.push_back(i);}
  }
  return groups;
}
struct HierarchicalOutput {std::vector<InitializationCandidate> candidates; size_t groups=0, screened=0, refined=0;bool expanded=false, unresolved_support=false, low_confidence=false;};
inline HierarchicalOutput hierarchicalSearch(
    const pcl::PointCloud<LocalizationPoint>& source,const pcl::PointCloud<LocalizationPoint>& target,
    const std::vector<std::pair<std::string,Eigen::Matrix4d>>& seeds,
    const Eigen::Matrix4d& odom,std::int64_t stamp,RegionInitializer& initializer,
    LocalizationRegistration& coarse,LocalizationRegistration& fine,double confidence_overlap=80.0) {
  HierarchicalOutput out;auto groups=positionGroups(seeds,odom);out.groups=groups.size();
  auto sample=[](const auto &cloud){pcl::PointCloud<LocalizationPoint> result;pcl::VoxelGrid<LocalizationPoint> grid;grid.setLeafSize(.25,.25,.25);grid.setInputCloud(cloud.makeShared());grid.filter(result);return result;};
  const auto cs=sample(source),ct=sample(target);
  for(size_t g=0;g<groups.size();++g)for(auto i:groups[g].representatives){
    const auto r=coarse.alignWithPrior(cs,ct,seeds[i].second,initializer.config().crop_margin);
    auto c=initializer.evaluate(seeds[i].first,"hierarchy_screen",r,odom,seeds[i].second,false);
    // Nonconvergence at the low iteration budget must not eliminate a group.
    const auto d=(seeds[i].second.inverse()*r.target_T_source).eval();
    if(r.target_T_source.allFinite() && std::isfinite(r.inlier_rmse) && r.fine_inliers>=20 && d.block<3,1>(0,3).norm()<=2 && initializationRotationDegrees(d.block<3,3>(0,0))<=45)
      groups[g].score=std::max(groups[g].score,r.overlap_percent);
    c.eligible=false;c.reason="ranking_only_group_"+std::to_string(g);out.candidates.push_back(c);++out.screened;
  }
  std::vector<size_t> ranked(groups.size());std::iota(ranked.begin(),ranked.end(),0);
  std::stable_sort(ranked.begin(),ranked.end(),[&](auto a,auto b){return groups[a].score>groups[b].score;});
  std::set<size_t> selected;
  for(size_t rank=0;rank<ranked.size();++rank)if(rank<3 || groups[ranked[rank]].score>=groups[ranked.front()].score-5)selected.insert(ranked[rank]);
  if(ranked.empty() || groups[ranked.front()].score<0){for(auto g:ranked)selected.insert(g);out.expanded=true;}
  auto refine=[&](size_t g){for(auto i:groups[g].members){auto r=fine.alignWithPrior(source,target,seeds[i].second,initializer.config().crop_margin);out.candidates.push_back(initializer.evaluate(seeds[i].first,"hierarchy_fine",r,odom,seeds[i].second,false));++out.refined;}};
  for(auto g:ranked)if(selected.count(g))refine(g);
  // Probe on a copy: do not advance the live two-window state during search.
  auto probe=initializer;auto rows=out.candidates;auto decision=probe.decide(rows,stamp,odom);
  const bool usable=std::any_of(rows.begin(),rows.end(),[](const auto &c){return c.eligible;});
  double best_eligible=-1;
  for(const auto &c:rows)if(c.eligible)best_eligible=std::max(best_eligible,c.registration.overlap_percent);
  // Low-confidence screening must not hide a competing staircase (recorded counterexample).
  if(!usable || best_eligible<confidence_overlap || decision.reason=="initialization_ambiguous"){
    out.expanded=true;for(auto g:ranked)if(!selected.count(g))refine(g);
  }
  // A strong geometrical fit beyond seed support cannot be accepted, but it also
  // cannot be silently discarded to accept a weaker, different staircase.
  double best=-1;int winner=-1;
  for(size_t i=0;i<out.candidates.size();++i)if(out.candidates[i].eligible && out.candidates[i].registration.overlap_percent>best){best=out.candidates[i].registration.overlap_percent;winner=i;}
  if(winner>=0)for(const auto &c:out.candidates) {
    const auto &r=c.registration;
    if(c.stage!="hierarchy_fine" || c.eligible || !r.valid || !r.converged || r.fine_inliers<static_cast<size_t>(initializer.config().min_fine_inliers) || r.inlier_rmse>initializer.config().max_inlier_rmse || r.overlap_percent<best-5)continue;
    const auto &w=out.candidates[winner].map_T_base;
    if((w.block<3,1>(0,3)-c.map_T_base.block<3,1>(0,3)).norm()>.5 || initializationRotationDegrees(w.block<3,3>(0,0).transpose()*c.map_T_base.block<3,3>(0,0))>10)out.unresolved_support=true;
  }
  out.low_confidence=winner>=0 && best<confidence_overlap;
  if(out.unresolved_support || out.low_confidence)for(auto &c:out.candidates)if(c.eligible){c.eligible=false;c.reason=out.unresolved_support?"hierarchy_unresolved_competitor":"hierarchy_low_confidence";}
  return out;
}
}
