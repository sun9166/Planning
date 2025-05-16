
#include "common/toolbox/path_restriction/include/path_restriction.h"
#include "common/toolbox/collision_detect/include/collision_detect.h"
#include "src/application/execution/map_engine/interface/freespace/include/freespace_check.h"
#include "src/application/execution/map_engine/core/bitstar/include/boundary_constrain.h"
// #include "ivlocmsg/ivmsglocpos.h"
// #include "ivpathplan/pathpoint.h"
#include "src/application/execution/adjust_pose/include/adjust_pose.h"
#include "src/application/execution/free_drive/include/freedrive.h"
#include "src/application/execution/dummy/include/dummy_pathplan.h"


// using namespace acu::pathplan;

PathRestriction::PathRestriction():
_length_threshold(6.0),
_width_threshold(2.0),
_width_resolution(0.1),
_width_limit(1.0),
_last_left_restrict(_width_limit),
_last_right_restrict(_width_limit),
_initial_left(0.46),
_initial_right(0.46)
{
  ros::NodeHandle nh;
  nh.param("Enable_Path_Restriction", _if_enable, false);
}


/*
assiagn restrictions for pathpoints and path
input path needed to be assiagned 
output int: (0: sucess, -1: failed)
*/
int PathRestriction::AssignPathRestriction(PathData& path_no_restriction)
{
  int rtvalue = -1;
  if(!_if_enable) {return rtvalue;}
  // auto& path_no_restriction = path;
  auto& path_no_restriction_path = path_no_restriction.path;
  if(path_no_restriction_path.empty()) {return rtvalue;}

  float length_threshold = _width_threshold;
  DataPool* DP = DataPool::Instance();
  // //TEMP
  // float sum_dis_threshold = (DP->GetMainDataPtr()->speedplan_param.stop_dis_fs > _length_threshold) ? _length_threshold :
  //                            DP->GetMainDataPtr()->speedplan_param.stop_dis_fs;
  // //TEMP
  float sum_dis_threshold = _length_threshold;
  AINFO<<"sum_dis_threshold is: "<<sum_dis_threshold;

  // BoundaryConstrain *boundary_constrain;//in the namespace pathplan instead of ivpathplan.
  // boundary_constrain = new BoundaryConstrain();
  // boundary_constrain->Use_Basemap_tool = true;
  // boundary_constrain->totallength = length_threshold;
  // boundary_constrain->CheckPartsNum_start = 9;
  // boundary_constrain->CheckPartsNum_end = 10;


  CoordinationTransform *coor_transform = CoordinationTransform::instance();
  FreespaceCheck *pfreespace_check = FreespaceCheck::instance();

  Site ego_site;
  ego_site.xg = DP->GetMainDataPtr()->loc_perception.localization.xg;
  ego_site.yg = DP->GetMainDataPtr()->loc_perception.localization.yg;
  ego_site.globalangle = DP->GetMainDataPtr()->loc_perception.localization.yaw;

  float sum_dis = 0.0;
  auto iter = path_no_restriction_path.begin();

  int left_half = 1;
  int right_half = 2;
  CarModel car_model_tmp;
  car_model_tmp.front_over_hang = 1.35;
  car_model_tmp.back_over_hang = 0.3;
  car_model_tmp.half_wheel = 0.46;
  // float carmodel_check_start = car_model_tmp.half_wheel;
  // std::cout<<"AssignPathRestriction by: "<<_initial_left<<" | "<<_initial_right<<std::endl;
  std::map<float, std::list<Site>::iterator> left_restrict_map, right_restrict_map;
  for(; std::next(iter)!=path_no_restriction_path.end() && iter!=path_no_restriction_path.end(); ++iter)
  {
    auto iter_next = std::next(iter);
    sum_dis += std::hypot(iter_next->y - iter->y, iter_next->x - iter->x);
    if(sum_dis>sum_dis_threshold) {break;}

    geometry_msgs::Point32 collision_point_left, collision_point_right;
    float dis2obs_left = std::numeric_limits<float>::max();
    float dis2obs_right = std::numeric_limits<float>::max();
    
    car_model_tmp.half_wheel = 0.46;//restore
    for(float h = _initial_left; h <= length_threshold; h += _width_resolution)
    {
      car_model_tmp.half_wheel = h;
      bool collision_check_left = ObjectCollisionDetect::CollisionDetection(
                          *iter,
                          DP->GetMainDataPtr()->loc_perception.perception_detail_data.sorted_objs,
                          car_model_tmp,
                          -1,
                          collision_point_left,
                          left_half);
      if(collision_check_left)
      { 
        Site collision_point_gccs, pathpoint_gccs;
        Site collision_point_vcs, collision_point_pathpoint_vcs;

        collision_point_vcs.x = collision_point_left.x;
        collision_point_vcs.y = collision_point_left.y;
        collision_point_vcs.angle = collision_point_left.z;//QUE

        coor_transform->VCS2GCCS(ego_site, *iter, pathpoint_gccs);
        coor_transform->VCS2GCCS(ego_site, collision_point_vcs, collision_point_gccs);
        coor_transform->GCCS2VCS(pathpoint_gccs, collision_point_gccs, collision_point_pathpoint_vcs);

        dis2obs_left = collision_point_pathpoint_vcs.y;
        // dis2obs_left = h;
        break;
      }

      bool fs_check = pfreespace_check->MODLFSCheck(*iter, car_model_tmp.front_over_hang,
                                                    car_model_tmp.back_over_hang,
                                                    car_model_tmp.half_wheel, 
                                                    true, 1);
      if(!fs_check)
      {
        dis2obs_left = h;
        break;
      }
    }

    car_model_tmp.half_wheel = 0.46;//restore
    for(float h = _initial_right; h <= length_threshold; h += _width_resolution)
    {
      car_model_tmp.half_wheel = h;
      bool collision_check_right = ObjectCollisionDetect::CollisionDetection(
                            *iter,
                            DP->GetMainDataPtr()->loc_perception.perception_detail_data.sorted_objs,
                            car_model_tmp,
                            -1,
                            collision_point_right,
                            right_half);
      if(collision_check_right)
      { 
        Site collision_point_gccs, pathpoint_gccs;
        Site collision_point_vcs, collision_point_pathpoint_vcs;

        collision_point_vcs.x = collision_point_right.x;
        collision_point_vcs.y = collision_point_right.y;
        collision_point_vcs.angle = collision_point_right.z;//QUE

        coor_transform->VCS2GCCS(ego_site, *iter, pathpoint_gccs);
        coor_transform->VCS2GCCS(ego_site, collision_point_vcs, collision_point_gccs);
        coor_transform->GCCS2VCS(pathpoint_gccs, collision_point_gccs, collision_point_pathpoint_vcs);

        dis2obs_right = std::fabs(collision_point_pathpoint_vcs.y);
        // dis2obs_right = h; 
        break;
      }
      bool fs_check = pfreespace_check->MODLFSCheck(*iter, car_model_tmp.front_over_hang,
                                                    car_model_tmp.back_over_hang,
                                                    car_model_tmp.half_wheel, 
                                                    true, 2);
      if(!fs_check)
      {
        dis2obs_right = h;
        break;
      }
    }

    // //using boundary_constrain
    // ivlocmsg::ivmsglocpos pathpoint;
    // pathpoint.xg = iter->xg;
    // pathpoint.yg = iter->yg;
    // pathpoint.angle = iter->globalangle;
    // auto dis2fs = boundary_constrain->CalcDis2FS(pathpoint, false, _initial_left, _initial_right);
    
    // float dis2fs_left = (dis2fs.first.first<0 || dis2fs.first.first>1e3) ? length_threshold : dis2fs.first.first;
    // float dis2fs_right = (dis2fs.second.first<0 || dis2fs.second.first>1e3) ? length_threshold : dis2fs.second.first;

    // iter->pathpoint_restrict.restrict_left = (dis2fs_left < dis2obs_left) ? dis2fs_left : dis2obs_left;
    // iter->pathpoint_restrict.restrict_right = (dis2fs_right < dis2obs_right) ? dis2fs_right : dis2obs_right;
    // //using boundary_constrain ends

    iter->pathpoint_restrict.restrict_left = dis2obs_left;
    iter->pathpoint_restrict.restrict_right = dis2obs_right;


    left_restrict_map.insert(std::make_pair(iter->pathpoint_restrict.restrict_left, iter));
    right_restrict_map.insert(std::make_pair(iter->pathpoint_restrict.restrict_right, iter));

    if((dis2obs_left + dis2obs_right) < 0.2 + 0.51*2)
    {
      DP->GetMainDataPtr()->speedplan_param.stop_dis_AC = sum_dis;
      AINFO<<"break AssignPathRestriction by stop_dis_AC: "<<sum_dis;
      break;
    }
  }
  // delete boundary_constrain;

  if(left_restrict_map.empty() || right_restrict_map.empty())
  {
    AERROR<<"left_restrict_map or right_restrict_map is empty, returned.";
    return rtvalue;
  }

  // //TEMP
  // //assiagn the first value whose velocity is not zero, for cutting up// 
  // DP->GetMainDataPtr()->motion_path.left_restrict = 0.0;
  // DP->GetMainDataPtr()->motion_path.right_restrict = 0.0;
  // auto iter_left = left_restrict_map.begin();
  // for(; iter_left!=left_restrict_map.end(); ++iter_left)
  // {
  //   if(iter_left->second->velocity > 0.0)
  //   {
  //     DP->GetMainDataPtr()->motion_path.left_restrict = iter_left->first;
  //     break;
  //   }
  // }
  // auto iter_right = right_restrict_map.begin();
  // for(; iter_right!=right_restrict_map.end(); ++iter_right)
  // {
  //   if(iter_right->second->velocity > 0.0)
  //   {
  //     DP->GetMainDataPtr()->motion_path.right_restrict = iter_right->first;
  //     break;
  //   }
  // }
  // //TEMP
  path_no_restriction.left_restrict = left_restrict_map.begin()->first;
  path_no_restriction.right_restrict = right_restrict_map.begin()->first;
  AINFO<<"restricts are: "<<path_no_restriction.left_restrict<<" | "
       <<path_no_restriction.right_restrict;

  SetLastResrictLeft(path_no_restriction.left_restrict);
  SetLastResrictRight(path_no_restriction.right_restrict); 

  rtvalue = 0;
  return rtvalue;
}


/*
generate boudaries for path with path's restrictions
input path with assiagned restrictions
output int: (0: sucess, -1: failed)
*/
int PathRestriction::GeneratePathRestrictionLine(const std::list<Site>& path_restriction) const
{
  int rtvalue = -1;
  if(!_if_enable) {return rtvalue;}
  if(path_restriction.empty()) 
  {
    AERROR<<"path's empty, returned.";
    return rtvalue;
  }
  DataPool* DP = DataPool::Instance();
  
  // auto move_dis_left = DP->GetMainDataPtr()->motion_path.left_restrict;
  // auto move_dis_right = DP->GetMainDataPtr()->motion_path.right_restrict;
  // std::cout<<"_last_left_restrict: "<<_last_left_restrict<<", _last_right_restrict: "<<_last_right_restrict<<std::endl;

  auto move_dis_left = _last_left_restrict;
  auto move_dis_right = _last_right_restrict;

  move_dis_left = (move_dis_left > 1.0) ? 1.0 : move_dis_left;
  move_dis_right = (move_dis_right > 1.0) ? 1.0 : move_dis_right;

  AINFO<<"move_dis: "<<move_dis_left<<" | "<<move_dis_right;
  // std::cout<<"move_dis: "<<move_dis_left<<" | "<<move_dis_right<<std::endl;

  float length_threshold = _width_threshold;
  if(move_dis_left<0.0 || move_dis_left>length_threshold || 
     move_dis_right<0.0 || move_dis_right>length_threshold)
  {
    AERROR<<"path's left_restrict or right_restrict error, returned.";
    // std::cout<<"path's left_restrict or right_restrict error, returned."<<std::endl;

    return rtvalue;
  }

  std::list<Site> path_to_filt = path_restriction;
  if(-1 == FiltPathByAngle(path_to_filt, path_restriction, move_dis_left, move_dis_right))
  {
    AERROR<<"FiltPathByAngle fails, returned.";
    // std::cout<<"FiltPathByAngle fails, returned."<<std::endl;

    return rtvalue;
  }

  PathData left_restriction_line, right_restriction_line;
  auto iter = path_to_filt.begin();
  float sum_dis = 0.0;
  for(;iter!=path_to_filt.end() && std::next(iter)!=path_to_filt.end(); ++iter)
  {
    const auto& p = *iter;
    // //TEMP
    // if(p.velocity<=0)
    // {
    //   AINFO<<"cutting up by velocity: "<<p.velocity;
    //   break;      
    // }
    // //TEMP
    // AINFO<<"checking: "<<p.x<<", "<<p.y<<", "
    //      <<p.pathpoint_restrict.restrict_left<<", "<<p.pathpoint_restrict.restrict_right;

    //should use sum_dis cuz if un-necessary in the beginning, the pathpoint_restrict is not assiagned yet.
    auto iter_next = std::next(iter);
    sum_dis += std::hypot(iter_next->x - iter->x, iter_next->y - iter->y);
    if(sum_dis > _length_threshold) {break;}

    Site left_restriction_pt, right_restriction_pt;
    /*-----------------------------assiagn by triangle------------------------------*/
    //xg, yg, etc. are not assiagned!
    left_restriction_pt.x = p.x - move_dis_left * std::sin(p.angle * 3.14 / 180.0);
    left_restriction_pt.y = p.y + move_dis_left * std::cos(p.angle * 3.14 / 180.0);
    left_restriction_pt.angle = p.angle;

    right_restriction_pt.x = p.x + move_dis_right * std::sin(p.angle * 3.14 / 180.0);
    right_restriction_pt.y = p.y - move_dis_right * std::cos(p.angle * 3.14 / 180.0);
    right_restriction_pt.angle = p.angle;
    /*==============================================================================*/

    /*-----------------------------assiagn by vector--------------------------------*/
    // Site unit_vector(std::next(iter)->x - p.x, std::next(iter)->y - p.y);
    // float unit_vetor_mold = std::hypot(unit_vector.x, unit_vector.y);
    // if(unit_vetor_mold < 1e-5) {continue;}
    // unit_vector.x /= unit_vetor_mold;
    // unit_vector.y /= unit_vetor_mold;
    
    // Site unit_normal_vector_left(-unit_vector.y, unit_vector.x);
    // Site unit_normal_vector_right(unit_vector.y, -unit_vector.x);

    // left_restriction_pt.x = p.x + move_dis_left * unit_normal_vector_left.x;
    // left_restriction_pt.y = p.y + move_dis_left * unit_normal_vector_left.y;
    // left_restriction_pt.angle = p.angle;

    // right_restriction_pt.x = p.x + move_dis_right * unit_normal_vector_right.x;
    // right_restriction_pt.y = p.y + move_dis_right * unit_normal_vector_right.y;
    // right_restriction_pt.angle = p.angle;    
    /*==============================================================================*/

    left_restriction_line.path.emplace_back(left_restriction_pt);
    right_restriction_line.path.emplace_back(right_restriction_pt);

    rtvalue = 0;
  }
  
  //No interpolation or filter is used!
  DP->GetMainDataPtr()->paths.left_restriction_line = left_restriction_line;
  DP->GetMainDataPtr()->paths.right_restriction_line = right_restriction_line;
  auto rt = GenerateFrontCornerLine(
            left_restriction_line, right_restriction_line, 
            DP->GetMainDataPtr()->paths.left_restriction_line_front, 
            DP->GetMainDataPtr()->paths.right_restriction_line_front);
  AINFO<<"returned with size: "<<left_restriction_line.path.size()<<", "<<right_restriction_line.path.size();
  AINFO<<"GenerateFrontCornerLine with size: "<<DP->GetMainDataPtr()->paths.left_restriction_line_front.path.size()
       <<", "<<DP->GetMainDataPtr()->paths.right_restriction_line_front.path.size()<<", returns "<<rt;

  // std::cout<<"returned with size: "<<left_restriction_line.path.size()<<", "<<right_restriction_line.path.size()<<std::endl;
  // std::cout<<"GenerateFrontCornerLine with size: "<<DP->GetMainDataPtr()->paths.left_restriction_line_front.path.size()
  //      <<", "<<DP->GetMainDataPtr()->paths.right_restriction_line_front.path.size()<<", returns "<<rt<<std::endl;

  return rtvalue;
}



/*
remove points with un-qualified angles
input path_ori with assiagned restrictions, move_dis_left(right) move-distance to the left(right)
output int: (0: sucess, -1: failed), path: filted path
*/
int PathRestriction::FiltPathByAngle(std::list<Site>& path, const std::list<Site>& path_ori, 
                    const float& move_dis_left, const float& move_dis_right) const
{
  //direction: 0 left, 1: right
  int rtvalue = -1;
  path = path_ori;
  if(path.size()<5) 
  {
    AWARN<<"path to filt is too short";
    return rtvalue;
  }
  int inter_num = 1;
  auto iter = path.begin();
  auto iter_next = iter;

  while(iter!=path.end() && std::next(iter)!=path.end())
  {
    iter_next = std::next(iter);
    auto iter_angle = NormalizeAngle(iter_next->angle - iter->angle);

    auto angle_threshold_left = (move_dis_left > 1e-3) ? (0.03/move_dis_left)*180.0/3.14 : std::numeric_limits<float>::max();
    auto angle_threshold_right = (move_dis_right > 1e-3) ? (0.03/move_dis_right)*180.0/3.14 : std::numeric_limits<float>::max();

    float angle_threshold = (iter_angle > 0) ? angle_threshold_left : angle_threshold_right;

    while(iter_next!=path.end() && fabs(iter_angle) > angle_threshold*inter_num)
    {
      ++inter_num;
      iter_next = path.erase(iter_next);
    }
    inter_num = 1;
    iter = iter_next;
  }
  rtvalue = 0;
  return rtvalue;
}


/*
generate parallel paths
input base_path generated paths based on, 
      direction move direction, move_dis move distance
output int: (0: sucess, -1: failed), rt_path returned path
*/
int PathRestriction::GenerateParallelPath(const std::list<Site>& base_path, std::list<Site>& rt_path,
                                          const int& direction, const float& move_dis) const
{
  int rtvalue = -1;
  if(base_path.size() < 5)
  {
    AWARN<<"base_path too short, returned.";
    return rtvalue;
  }

  if(direction != 1 && direction != -1)
  {
    AWARN<<"move direction invalid: "<<direction<< ", returned.";
    return rtvalue;
  }

  if(move_dis <= 1e-3)
  {
    AWARN<<"move_dis too small: "<<move_dis<< ", returned.";
    return rtvalue;
  }

  float move_dis_left = 0.0, move_dis_right = 0.0;
  if(direction == 1)
  {//left
    move_dis_left = move_dis;
  }
  else if(direction == -1)
  {//right
    move_dis_right = move_dis;
  }

  rt_path.clear();
  std::list<Site> filted_path;

  if(-1 == FiltPathByAngle(filted_path, base_path, move_dis_left, move_dis_right))
  {
    AWARN<<"FiltPathByAngle failed, returned.";
    return rtvalue;
  }
  else if(filted_path.size() < 5)
  {
    AWARN<<"FiltPathByAngle result too short: "<<filted_path.size() <<", returned.";
    return rtvalue;   
  }

  DataPool* DP = DataPool::Instance();
  float sum_dis = 0.0;
  // //TEMP
  // float sum_dis_threshold = (DP->GetMainDataPtr()->speedplan_param.stop_dis_fs > _length_threshold) ? _length_threshold :
  //                            DP->GetMainDataPtr()->speedplan_param.stop_dis_fs;
  // //TEMP
  float sum_dis_threshold = _length_threshold;
  auto iter = filted_path.begin();
  for(;iter!=filted_path.end() && std::next(iter)!=filted_path.end(); ++iter)
  {
    auto iter_next = std::next(iter);
    sum_dis += std::hypot(iter_next->x - iter->x, iter_next->y - iter->y);
    if(sum_dis > sum_dis_threshold) {break;}
    if(iter->velocity <= 0)
    {
      AINFO<<"cutting up by velocity: "<<iter->velocity;
      break;      
    }
    Site p;
    if(direction == 1)
    {//left
      p.x = iter->x - move_dis * std::sin(iter->angle * 3.14 / 180.0);
      p.y = iter->y + move_dis * std::cos(iter->angle * 3.14 / 180.0);
      p.angle = iter->angle;
      rt_path.emplace_back(p);
    }
    else if(direction == -1)
    {
      p.x = iter->x + move_dis * std::sin(iter->angle * 3.14 / 180.0);
      p.y = iter->y - move_dis * std::cos(iter->angle * 3.14 / 180.0);
      p.angle = iter->angle;
      rt_path.emplace_back(p);

    }
  }
  rtvalue = 0;
  return rtvalue;
}


/*
get parallel distance by collision-detect & fs
input base_path generated paths based on
output int: (0: sucess, -1: failed), dis_pair returned distance pair
*/
int PathRestriction::GenerateParallelDistance(const std::list<Site>& base_path, std::pair<float, float>& dis_pair) const
{
  int rtvalue = -1;
  if(base_path.size() < 5)
  {
    AWARN<<"base_path too short, returned.";
    return rtvalue;
  }

  if(_width_limit <= 1e-3)
  {
    AWARN<<"width_limit too short: "<< _width_limit <<", returned.";
    return rtvalue;
  }

  if(_width_resolution <= 1e-3)
  {
    AWARN<<"width_resolution too short: "<< _width_resolution <<", returned.";
    return rtvalue;
  }
  else if(_width_resolution > _width_limit)
  {
    AWARN<<"width_resolution is bigger than width_limit: "<< _width_resolution
         <<" > "<<_width_limit <<", returned.";
    return rtvalue;    
  }

  float dis_left = _width_limit, dis_right = _width_limit;

  DataPool* DP = DataPool::Instance();
  //use objs WITHOUT ultrasonic
  auto objset = DP->GetMainDataPtr()->loc_perception.perception_detail_data.sorted_objs;

  auto pfreespace_check = FreespaceCheck::instance();
  auto *cor_trans_ = CoordinationTransform::instance();

  geometry_msgs::Point32 collision_point_left, collision_point_right;

  int left_half = 1, right_half = 2;
  CarModel car_model_tmp;
  car_model_tmp.front_over_hang = 0.3;
  car_model_tmp.back_over_hang = 0.3;

  Site ego_site;
  ego_site.xg = DP->GetMainDataPtr()->loc_perception.localization.xg;
  ego_site.yg = DP->GetMainDataPtr()->loc_perception.localization.yg;
  ego_site.globalangle = DP->GetMainDataPtr()->loc_perception.localization.yaw;

  for(float i = _width_resolution; i <= _width_limit; i += _width_resolution)
  {//check left parallel path
    std::list<Site> para_path_left;
    if(-1 == GenerateParallelPath(base_path, para_path_left, 1, i))
    {
      AERROR<<"GenerateParallelPath failed, with para_path_left: "<<i;
      return rtvalue;
    }
    

    car_model_tmp.half_wheel = i;
    // AINFO<<"checking..."<<car_model_tmp.half_wheel;
    float sum_dis = 0.0;
    auto iter = para_path_left.begin();
    bool find_left_boundary = false;
    for(;iter!=para_path_left.end() && std::next(iter)!=para_path_left.end(); ++iter)
    {
      // AINFO<<"checking: "<<iter->x<<", "<<iter->y<<", "<<iter->angle;
      auto iter_next = std::next(iter);
      sum_dis += std::hypot(iter_next->x - iter->x, iter_next->y - iter->y);
      if(sum_dis > _length_threshold) {break;}
      bool collision_check_right = ObjectCollisionDetect::CollisionDetection(
                                   *iter, objset, car_model_tmp, -1,
                                   collision_point_right, right_half);
      if(collision_check_right)
      {
        dis_left = i;
        find_left_boundary = true;
        // AINFO<<"collision_check_right obj: "<<collision_check_right;

        break;
      }

      Site gccs;
      cor_trans_->VCS2GCCS(ego_site, *iter, gccs);
      if(!pfreespace_check->GCCSFSCheck(gccs, true))
      {
        dis_left = i;
        find_left_boundary = true;
        break;
      }
    }

    DP->GetMainDataPtr()->paths.left_restriction_line.path = para_path_left;
    if(find_left_boundary) {break;}
  }

  for(float j = _width_resolution; j <= _width_limit; j += _width_resolution)
  {//check right parallel path
    std::list<Site> para_path_right;
    if(-1 == GenerateParallelPath(base_path, para_path_right, -1, j))
    {
      AERROR<<"GenerateParallelPath failed, with para_path_right: "<<j;
      return rtvalue;
    }

    car_model_tmp.half_wheel = j;
    // AINFO<<"checking..."<<car_model_tmp.half_wheel;
    float sum_dis = 0.0;
    auto iter = para_path_right.begin();
    bool find_right_boundary = false;
    for(;iter!=para_path_right.end() && std::next(iter)!=para_path_right.end(); ++iter)
    {
      // AINFO<<"checking: "<<iter->x<<", "<<iter->y<<", "<<iter->angle;
      auto iter_next = std::next(iter);
      sum_dis += std::hypot(iter_next->x - iter->x, iter_next->y - iter->y);
      if(sum_dis > _length_threshold) {break;}
      bool collision_check_left = ObjectCollisionDetect::CollisionDetection(
                                   *iter, objset, car_model_tmp, -1,
                                   collision_point_left, left_half);
      if(collision_check_left)
      {
        dis_right = j;
        find_right_boundary = true;
        // AINFO<<"collision_check_left obj: "<<collision_check_left;

        break;
      }

      Site gccs;
      cor_trans_->VCS2GCCS(ego_site, *iter, gccs);
      if(!pfreespace_check->GCCSFSCheck(gccs, true))
      {
        dis_left = j;
        find_right_boundary = true;

        break;
      }
    }

    DP->GetMainDataPtr()->paths.right_restriction_line.path = para_path_right;
    if(find_right_boundary) {break;}
  }

  dis_pair.first = dis_left;
  dis_pair.second = dis_right;
  rtvalue = 0;
  return rtvalue;
}

/*
check whther necessary to re-plan the restricts
input path original path(motion_path or pure pursuit path)
output int: (0. don't replan, set the Resricts respectively
            -1. check failed restore InitialValue, replan, 
             1. smaller side failed,  restore InitialValue, replan. 
             2. larger side failed, set the InitialValue by the smaller side, replan.
             3. both side passes, set the InitialValue respectively, replan)
*/
int PathRestriction::CheckNecessity(std::list<Site>& path)
{
  int rtvalue = -1;
  if(path.size() < 5)
  {
    return rtvalue;
  }
  ResetInitialValue();

  DataPool* DP = DataPool::Instance();
  if(DP->GetMainDataPtr()->speedplan_param.stop_dis_fs < _length_threshold)
  {
    return 1;
  }

  //in case of _width_limit is setted by user outside, and is smaller than the default value
  //cuz _last_left_restrict and _last_right_restrict are initialized by default value
  _last_left_restrict = (_last_left_restrict > _width_limit) ? _width_limit : _last_left_restrict;
  _last_right_restrict = (_last_right_restrict > _width_limit) ? _width_limit : _last_right_restrict;

  double dis_obs_except_ultra = numeric_limits<double>::max();
  geometry_msgs::Point32 collision_cell_except_ultra;
  double collision_cell_except_ultra_height;
  
  double dis_out_of_fs = numeric_limits<double>::max();
  Site collision_site_fs;

  //for the second round check(if the left is smaller, right side should be check in the second round)
  int which_half_later_check = (_last_left_restrict < _last_right_restrict) ? 2 : 1;

  CarModel car_model_tmp;
  car_model_tmp.front_over_hang = 1.35;
  car_model_tmp.back_over_hang = 0.3;
  car_model_tmp.half_wheel = std::min(_last_left_restrict, _last_right_restrict);
  //use the smaller move-dis for whole-car-model check
  bool result_obs_except_ultra_min 
       = ObjectCollisionDetect::RoadCollisionDetection_dis<geometry::Site, std::list<geometry::Site>>(
                                path,
                                dis_obs_except_ultra,
                                DP->GetMainDataPtr()->loc_perception.perception_detail_data.sorted_objs,
                                car_model_tmp,
                                collision_cell_except_ultra,
                                collision_cell_except_ultra_height, _length_threshold);

  FreespaceCheck *pfreespace_check = FreespaceCheck::instance();
  bool is_infreesapce_min 
       = pfreespace_check->MODLPathDTGFSCheck_dis(
                           path,
                           collision_site_fs,
                           dis_out_of_fs,
                           car_model_tmp.front_over_hang,
                           car_model_tmp.back_over_hang,
                           car_model_tmp.half_wheel, _length_threshold);

  if(result_obs_except_ultra_min || !is_infreesapce_min)
  {//the smaller side failed
    // std::cout<<"should replan by min check.....: "<<car_model_tmp.half_wheel<<std::endl;
    // std::cout<<"collision_cell_except_ultra: "<<collision_cell_except_ultra.x<<", "<<collision_cell_except_ultra.y<<std::endl;
    // std::cout<<"collision_site_fs: "<<collision_site_fs.x<<", "<<collision_site_fs.y<<std::endl;
    
    rtvalue = 1;
  }
  else
  {//if the smaller side checking passed, check the larger side
   //use the larger move-dis for half-car-model check\

    //start next checking by the smaller one is reasonable at least
    SetInitialLeft(std::min(_last_left_restrict, _last_right_restrict));
    SetInitialRight(std::min(_last_left_restrict, _last_right_restrict));

    car_model_tmp.half_wheel = std::max(_last_left_restrict, _last_right_restrict);
    bool result_obs_except_ultra_max 
       = ObjectCollisionDetect::RoadCollisionDetection_dis<geometry::Site, std::list<geometry::Site>>(
                                path,
                                dis_obs_except_ultra,
                                DP->GetMainDataPtr()->loc_perception.perception_detail_data.sorted_objs,
                                car_model_tmp,
                                collision_cell_except_ultra,
                                collision_cell_except_ultra_height, _length_threshold, which_half_later_check);

    FreespaceCheck *pfreespace_check = FreespaceCheck::instance();
    bool is_infreesapce_max 
         = pfreespace_check->MODLPathDTGFSCheck_dis(
                             path,
                             collision_site_fs,
                             dis_out_of_fs,
                             car_model_tmp.front_over_hang,
                             car_model_tmp.back_over_hang,
                             car_model_tmp.half_wheel, _length_threshold, which_half_later_check);
    if(result_obs_except_ultra_max || !is_infreesapce_max)
    {//the smaller side failed
      // std::cout<<"should replan by max check.....: "<<car_model_tmp.half_wheel<<std::endl;
      rtvalue = 2;
    }
    else
    {//neither side collids, need NOT replan
      //set the larger side's initial checking value
      if(which_half_later_check == 2)
      {
        SetInitialRight(std::max(_last_left_restrict, _last_right_restrict));
      }
      else
      {
        SetInitialLeft(std::min(_last_left_restrict, _last_right_restrict));
      }
      rtvalue = 3;

      car_model_tmp.half_wheel = _width_limit;
      bool result_obs_except_ultra_max_upperlimit 
           = ObjectCollisionDetect::RoadCollisionDetection_dis<geometry::Site, std::list<geometry::Site>>(
                                    path,
                                    dis_obs_except_ultra,
                                    DP->GetMainDataPtr()->loc_perception.perception_detail_data.sorted_objs,
                                    car_model_tmp,
                                    collision_cell_except_ultra,
                                    collision_cell_except_ultra_height, _length_threshold);
      FreespaceCheck *pfreespace_check = FreespaceCheck::instance();
      bool is_infreesapce_max_upperlimit 
           = pfreespace_check->MODLPathDTGFSCheck_dis(
                               path,
                               collision_site_fs,
                               dis_out_of_fs,
                               car_model_tmp.front_over_hang,
                               car_model_tmp.back_over_hang,
                               car_model_tmp.half_wheel, _length_threshold);
      if(!result_obs_except_ultra_max_upperlimit && is_infreesapce_max_upperlimit)
      {//no need to replan, and set the last result to upper limit
        // std::cout<<"should NOT replan"<<std::endl;
        SetLastResrictLeft(_width_limit);
        SetLastResrictRight(_width_limit); 
        rtvalue = 0;
      }
    }
  }

  return rtvalue;
}


/*
generate front corner points' lines using the  restrict lines.
input ll left-line(restrict), rl right-line(restrict)
output int: (0: sucess, -1: failed), rll(rrl) retured left(right) front corner points line
*/
int PathRestriction::GenerateFrontCornerLine(const PathData& ll, const PathData& rl,
                                             PathData& rll, PathData& rrl) const
{
  int rtvalue = -1;
  if(ll.path.size()<5 && rl.path.size()<5) {return rtvalue;}

  rll.clear();
  rrl.clear();
  float car_len = 1.35;

  if(ll.path.size() > 5) 
  {
    for(const auto& p : ll.path)
    {
      Site rll_pt(p.x + car_len*std::cos(p.angle*3.14/180.0),
                  p.y + car_len*std::sin(p.angle*3.14/180.0));
      rll.path.emplace_back(rll_pt);
    }
    rtvalue = 1;
  }

  if(rl.path.size() > 5) 
  {
    for(const auto& p : rl.path)
    {
      Site rrl_pt(p.x + car_len*std::cos(p.angle*3.14/180.0),
                  p.y + car_len*std::sin(p.angle*3.14/180.0));
      rrl.path.emplace_back(rrl_pt);
    }
    rtvalue = (rtvalue == 1) ? 0 : 2;
  }

  return rtvalue;
}


/*
check the availability by current behavior state
input None
output bool:(true available)
*/
bool PathRestriction::CheckAvailabilitybyState()
{
  bool rtvalue = false;
  DataPool* DP = DataPool::Instance();
  auto crt_border_state = DP->GetMainDataRef().border_behavior_fsm_info.border_behavior_status.status;
  auto crt_normal_state = DP->GetMainDataRef().behavior_fsm_info.behavior_status.status;

  bool check_border = crt_border_state!=eBorderBehaviorStatus::BORDER_SMOOTH &&
                      crt_border_state!=eBorderBehaviorStatus::BORDER_DR_CLEAN &&
                      crt_border_state!=eBorderBehaviorStatus::BORDER_CUT_OUT &&
                      crt_border_state!=eBorderBehaviorStatus::BORDER_DR_BORDER_SMOOTH_STAYSTILL &&
                      crt_border_state!=eBorderBehaviorStatus::BORDER_DR_BORDER_CLEAN_STAYSTILL &&
                      crt_border_state!=eBorderBehaviorStatus::BORDER_DR_BORDER_CUTOUT_STAYSTILL &&
                      crt_border_state!=eBorderBehaviorStatus::BORDER_DR_BORDER_SMOOTH_BACK_FOLLOW &&
                      crt_border_state!=eBorderBehaviorStatus::BORDER_DR_BORDER_CLENAN_BACK_FOLLOW &&
                      crt_border_state!=eBorderBehaviorStatus::BORDER_DR_BORDER_CUTOUT_BACK_FOLLOW &&
                      crt_border_state!=eBorderBehaviorStatus::BORDER_DR_BACKFOLLOWING &&
                      crt_border_state!=eBorderBehaviorStatus::BORDER_DR_BACK_STAYSTILL;
  bool check_normal = true;

  auto crt_pathplanptr_state = DP->GetMainDataRef().pathplan_ptr->pathplan_type_.type;

  bool check_result = (crt_pathplanptr_state == ePathplanType::BOADER_CLEAN) ? check_border : check_normal;
   
  if(check_result) {rtvalue = true;}
  return rtvalue;
}


/*
check the availability by current behavior state for releasing
input None
output bool:(true available)
*/
bool PathRestriction::CheckAvailabilitybyStateForReleasing()
{
  bool rtvalue = false;
  DataPool* DP = DataPool::Instance();
  auto crt_border_state = DP->GetMainDataRef().border_behavior_fsm_info.border_behavior_status.status;
  auto crt_normal_state = DP->GetMainDataRef().behavior_fsm_info.behavior_status.status;

  bool check_border = crt_border_state == eBorderBehaviorStatus::BORDER_LOCAL_BACKFOLLOWING;
  bool check_normal = crt_normal_state == eBehaviorStatus::BACKFOLLOWING;

  auto crt_pathplanptr_state = DP->GetMainDataRef().pathplan_ptr->pathplan_type_.type;

  bool check_result = (crt_pathplanptr_state == ePathplanType::BOADER_CLEAN) ? check_border : check_normal;
   
  if(check_result) {rtvalue = true;}
  return rtvalue;
}