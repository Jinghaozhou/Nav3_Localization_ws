#include <stdexcept>
#include <iostream>
#include "localization/hierarchical_search.h"
int main(){using namespace kiss_matcher;std::vector<std::pair<std::string,Eigen::Matrix4d>> s;
 auto add=[&](std::string id,double x,double z,double yaw){Eigen::Matrix4d m=Eigen::Matrix4d::Identity();m(0,3)=x;m(2,3)=z;m.block<3,3>(0,0)=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();s.emplace_back(id,m);};
 add("a",0,0,0);add("b",.4,0,0);add("c",.2,0,1.57);add("d",0,3,0);
 auto g=positionGroups(s,Eigen::Matrix4d::Identity());
 if(g.size()!=2 || g[0].members.size()!=3 || g[0].representatives.size()!=2 || g[1].members.size()!=1)throw std::runtime_error("position/orientation groups");
 std::reverse(s.begin(),s.end());auto h=positionGroups(s,Eigen::Matrix4d::Identity());
 if(h.size()!=g.size() || s[h[0].representatives[0]].first!="c")throw std::runtime_error("stable medoid independent of rank");
 if(!positionGroups({},Eigen::Matrix4d::Identity()).empty())throw std::runtime_error("empty groups");
 std::cout<<"PASS spatial separation, medoid, orientation diversity, rank invariance, empty input\n";
}
