// Read-only offline experiment: no ROS node, publishers, TF, or map updates.
#include <fstream>
#include <chrono>
#include <yaml-cpp/yaml.h>
#include <pcl/io/pcd_io.h>
#include "localization/hierarchical_search.h"
int main(int argc,char**argv){
 if(argc!=7)return 2;using namespace kiss_matcher;using Cloud=pcl::PointCloud<LocalizationPoint>;
 Cloud source,target; pcl::PointCloud<pcl::PointXYZ> xyz;
 if(pcl::io::loadPCDFile(argv[1],xyz)<0||pcl::io::loadPCDFile(argv[2],target)<0)return 3;
 for(const auto &p:xyz){LocalizationPoint q;q.x=p.x;q.y=p.y;q.z=p.z;q.intensity=0;source.push_back(q);}
 std::vector<std::pair<std::string,Eigen::Matrix4d>> seeds;
 for(const auto &s:YAML::LoadFile(argv[3])){Eigen::Matrix4d t;for(int i=0;i<4;++i)for(int j=0;j<4;++j)t(i,j)=s["pose"][i][j].as<double>();seeds.emplace_back(s["id"].as<std::string>(),t);}
 InitializerConfig ic;RegionInitializer init(ic);init.load(argv[4],"map","base_link");init.begin(2,1000000000,Eigen::Matrix4d::Identity());
 LocalizationConfig cfg;cfg.min_inliers=7;cfg.overlap_threshold=60;cfg.num_threads=4;
 LocalizationRegistration fine(cfg,rclcpp::get_logger("hierarchy_replay"));cfg.voxel_resolution=.25;cfg.max_num_iter=16;LocalizationRegistration coarse(cfg,rclcpp::get_logger("hierarchy_screen"));
 auto start=std::chrono::steady_clock::now();HierarchicalOutput result;
 if(std::string(argv[6])=="hierarchy") result=hierarchicalSearch(source,target,seeds,Eigen::Matrix4d::Identity(),1000000000,init,coarse,fine);
 else for(const auto &[id,t]:seeds){auto r=fine.alignWithPrior(source,target,t,ic.crop_margin);result.candidates.push_back(init.evaluate(id,"exhaustive_fine",r,Eigen::Matrix4d::Identity(),t,false));++result.refined;}
 auto decision=init.decide(result.candidates,1000000000,Eigen::Matrix4d::Identity());
 YAML::Emitter o;o<<YAML::BeginMap<<YAML::Key<<"ms"<<YAML::Value<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()<<YAML::Key<<"groups"<<YAML::Value<<result.groups<<YAML::Key<<"screened"<<YAML::Value<<result.screened<<YAML::Key<<"refined"<<YAML::Value<<result.refined<<YAML::Key<<"expanded"<<YAML::Value<<result.expanded<<YAML::Key<<"low_confidence"<<YAML::Value<<result.low_confidence<<YAML::Key<<"unresolved_support"<<YAML::Value<<result.unresolved_support<<YAML::Key<<"reason"<<YAML::Value<<(result.unresolved_support?"hierarchy_unresolved_competitor":(result.low_confidence?"hierarchy_low_confidence":decision.reason))<<YAML::Key<<"selected"<<YAML::Value<<decision.selected<<YAML::Key<<"candidates"<<YAML::Value<<YAML::BeginSeq;
 for(auto &c:result.candidates){const auto &r=c.registration;o<<YAML::BeginMap<<YAML::Key<<"id"<<YAML::Value<<c.id<<YAML::Key<<"stage"<<YAML::Value<<c.stage<<YAML::Key<<"eligible"<<YAML::Value<<c.eligible<<YAML::Key<<"reason"<<YAML::Value<<c.reason<<YAML::Key<<"overlap"<<YAML::Value<<r.overlap_percent<<YAML::Key<<"rmse"<<YAML::Value<<(std::isfinite(r.inlier_rmse)?r.inlier_rmse:-1)<<YAML::Key<<"pose"<<YAML::Value<<YAML::BeginSeq;for(int i=0;i<4;i++){o<<YAML::Flow<<YAML::BeginSeq;for(int j=0;j<4;j++)o<<r.target_T_source(i,j);o<<YAML::EndSeq;}o<<YAML::EndSeq<<YAML::EndMap;}
 o<<YAML::EndSeq<<YAML::EndMap;std::ofstream(argv[5])<<o.c_str()<<"\n";
}
