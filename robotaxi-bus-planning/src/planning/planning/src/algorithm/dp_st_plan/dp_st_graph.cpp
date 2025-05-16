/**
 * @file dp_st_graph.cpp
 **/


#include <algorithm>
#include <limits>
#include <string>
#include <utility>
////#include "ros/ros.h"

#include "pnc_point.pb.h"
#include "dp_st_graph.h"
#include "common/math/vec2d.h"
#include "common/math/math_utils.h"
#include "common/base/log/include/log.h"
#include "src/execution/motionplan/common/datatype.h"
#include "src/execution/motionplan/common/planning_gflags.h"
#include "src/execution/motionplan/behavior_parser/behavior_parser.h"


namespace acu {
namespace planning {

using acu::common::ErrorCode;
using acu::common::SpeedPoint;
using acu::common::Status;

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

bool CheckOverlapOnDpStGraph(const std::vector<const StBoundary*>& boundaries,
                             const StGraphPoint& p1, const StGraphPoint& p2) {
  const common::math::LineSegment2d seg(p1.point(), p2.point());
  for (const auto* boundary : boundaries) {
    if (boundary->boundary_type() == StBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }
    if (boundary->HasOverlap(seg)) {
      return true;
    }
  }
  return false;
}
}  // namespace

DpStGraph::DpStGraph(const StGraphData& st_graph_data,
                     const DpStSpeedConfig& dp_config,
                     const std::vector<const Obstacle*>& obstacles,
                     const common::TrajectoryPoint& init_point,
                     const SLBoundary& adc_sl_boundary)
    : st_graph_data_(st_graph_data),
      dp_st_speed_config_(dp_config),
      obstacles_(obstacles),
      init_point_(init_point),
      dp_st_cost_(dp_config, obstacles, init_point_),
      adc_sl_boundary_(adc_sl_boundary) {
  dp_st_speed_config_.set_total_path_length(
      std::fmin(dp_st_speed_config_.total_path_length(),
                st_graph_data_.path_data_length()));
  unit_s_ = dp_st_speed_config_.total_path_length() /
            (dp_st_speed_config_.matrix_dimension_s() - 1);
  unit_t_ = dp_st_speed_config_.total_time() /
            (dp_st_speed_config_.matrix_dimension_t() - 1);
}

Status DpStGraph::Search(SpeedInfo* const speed_data) {
  constexpr double kBounadryEpsilon = 1e-2;
  for (const auto& boundary : st_graph_data_.st_boundaries()) {
    if (boundary->boundary_type() == StBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }
    if (boundary->IsPointInBoundary({0.0, 0.0}) ||
        (std::fabs(boundary->min_t()) < kBounadryEpsilon &&
         std::fabs(boundary->min_s()) < kBounadryEpsilon)) {
      if (boundary->speed() > st_graph_data_.cruise_speed() ||
           boundary->speed() > init_point_.v()) {
        const std::string msg = boundary->id() + " boundary.min_t and min_s less than  1e-2.";
        AWARN_IF(FLAGS_enable_debug_motion)<<  msg;
        return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg); 
      }
      std::vector<SpeedPoint> speed_profile;
      double t = 0.0;
      for (int i = 0; i <= dp_st_speed_config_.matrix_dimension_t();//matrix_dimension_t : 8s
           ++i, t += unit_t_) {
        SpeedPoint speed_point;
        speed_point.set_s(0.0);
        speed_point.set_t(t);
        speed_point.set_v(0.0);
        speed_point.set_a(0.0);
        speed_profile.emplace_back(speed_point);
      }
      *speed_data = SpeedInfo(speed_profile);
      AWARN_IF(FLAGS_enable_debug_motion)<< "there are some obstacles closed to ego car" 
            <<", id = "<<boundary->id() <<" boundary.min_t and min_s less than  1e-2 ,STOP! ";
      const std::string msg = boundary->id() + " boundary.min_t and min_s less than  1e-2.";      
      return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);       
    }
  }

  if (st_graph_data_.st_boundaries().empty()) {
    std::vector<SpeedPoint> speed_profile;
    double s = 0.0;
    double t = 0.0;
    for (int i = 0; i <= dp_st_speed_config_.matrix_dimension_t() &&
                    i <= dp_st_speed_config_.matrix_dimension_s();
         ++i, t += unit_t_, s += unit_s_) {
      SpeedPoint speed_point;
      speed_point.set_s(s);
      speed_point.set_t(t);
      const double v_default = unit_s_ / unit_t_;
      speed_point.set_v(v_default);
      speed_point.set_a(0.0);
      speed_profile.emplace_back(std::move(speed_point));
    }
    *speed_data = SpeedInfo(std::move(speed_profile));
    return Status::OK();
  }

  //初始化代价表cost_table_
  if (!InitCostTable().ok()) {
    const std::string msg = "Initialize cost table failed.";
    AWARN_IF(FLAGS_enable_debug_motion)<<  msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }
  //计算代价表cost_table_中所有节点的总代价
  if (!CalculateTotalCost().ok()) {
    const std::string msg = "Calculate total cost failed.";
    AWARN_IF(FLAGS_enable_debug_motion)<<  msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }
  //通过计算得到的代价表cost_table_中的所有节点的总代价，找到代价最小的节点，获取速度数据
  if (!RetrieveSpeedProfile(speed_data).ok()) {
    const std::string msg = "Retrieve best speed profile failed.";
    AWARN_IF(FLAGS_enable_debug_motion)<<  msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }
  return Status::OK();
}

//初始化代价表
Status DpStGraph::InitCostTable() {
  //通过配置文件读取s和t的维数
  uint32_t dim_s = dp_st_speed_config_.matrix_dimension_s();
  uint32_t dim_t = dp_st_speed_config_.matrix_dimension_t();
  
  //维数检查，要求＞2
  DCHECK_GT(dim_s, 2);
  DCHECK_GT(dim_t, 2);

  //生成代价表cost_table_，行为dim_t，列为dim_s，所有节点均初始化为：StGraphPoint(),8个std::vector<StGraphPoint>(dim_s, StGraphPoint())
  cost_table_ = std::vector<std::vector<StGraphPoint>>(
      dim_t, std::vector<StGraphPoint>(dim_s, StGraphPoint()));

  //注：cost_table_[i][j] = STPoint(0.0+j*unit_s_, 0.0+ i *unit_t_);
  double curr_t = 0.0;
  uint32_t cost_table_size = cost_table_.size();
  for (uint32_t i = 0; i < cost_table_size; ++i, curr_t += unit_t_) {
    auto& cost_table_i = cost_table_[i];
    double curr_s = 0.0;
    uint32_t cost_table_i_size = cost_table_i.size();
    for (uint32_t j = 0; j < cost_table_i_size; ++j, curr_s += unit_s_) {
      cost_table_i[j].Init(i, j, STPoint(curr_s, curr_t));
      //对每个点进行初始化，得到StGraphPoint 类型对象的成员变量index_t_ = i，index_s_ = j;point_ = STPoint(curr_s, curr_t);
    }
  }
  return Status::OK();
}

//计算总代价
Status DpStGraph::CalculateTotalCost() {
  // col and row are for STGraph
  // t corresponding to col //横坐标
  // s corresponding to row //纵坐标
  size_t next_highest_row = 0;
  size_t next_lowest_row = 0;
  //对于第一、二，直至最后一个时间采样值，循环计算不同距离采样值上的代价
  //注意：每个时间采样值上的距离采样值数目是不一样的，例如：
  //第一个时间采样值为起点，在该点上只能有一个距离采样值：0，否则
  //代价表cost_table_就不正确，正常情况下，第二个时间采样值上的距离采样值数目会＞1，不然就是匀速前进
  // AINFO_IF(FLAGS_enable_debug_motion)<< "cost_table_.size() :"<<cost_table_.size();
  size_t cost_table_size = cost_table_.size();
  for (size_t c = 0; c < cost_table_size; ++c) { //c为st图的一列
    size_t highest_row = 0; //最高行，即最大加速度情形下所允许的最大距离采样值
    auto lowest_row = cost_table_.back().size() - 1;//最低行，即最大减速度情形下所允许的最小距离采样值//最后一列的点的个数，及s的个数，就是规划的s的上下限
    int count = static_cast<int>(next_highest_row) -
                static_cast<int>(next_lowest_row) + 1; //规划所有s方向上的离散点的个数
    
    //对于时间采样值c上的不同距离采样值r:next_lowest_row<=4<=next_highest_row
    //计算抵达节点（c,r）的最小总代价
    if (count > 0) {
      // AINFO_IF(FLAGS_enable_debug_motion)<<"next_lowest_row :"<<next_lowest_row<<",next_highest_row:"<<next_highest_row;
      for (size_t r = next_lowest_row; r <= next_highest_row; ++r) {
        auto msg = std::make_shared<StGraphMessage>(c, r);
        CalculateCostAt(msg);
        // AINFO_IF(FLAGS_enable_debug_motion)<<"cost_table_["<<c<<"]["<<r<<"] total_cost is "<<cost_table_[msg->c][msg->r].total_cost();
      }
    }

    //给定时间采样c的情形下，
    //更新最高行（即最大采样距离值）和最低行（即最小采样距离值）
    for (size_t r = next_lowest_row; r <= next_highest_row; ++r) {
      const auto& cost_cr = cost_table_[c][r];
      if (cost_cr.total_cost() < std::numeric_limits<double>::infinity()) {
        size_t h_r = 0;
        size_t l_r = 0;
        // AINFO_IF(FLAGS_enable_debug_motion)<<"No.["<<r<<"] cost is selected";
        //获取从当前点往后搜索的s的范围
        GetRowRange(cost_cr, &h_r, &l_r);
        highest_row = std::max(highest_row, h_r);
        lowest_row = std::min(lowest_row, static_cast<size_t>(l_r));
      }
    }
    next_highest_row = highest_row;
    next_lowest_row = lowest_row;
  }
  return Status::OK();
}

//基于当前ST图上的点point,获取下一个允许的
//最高行（即最大采样距离）和最低行（即最小采样距离值）
void DpStGraph::GetRowRange(const StGraphPoint& point, size_t* next_highest_row,
                            size_t* next_lowest_row) {
  double v0 = 0.0;
  if (!point.pre_point()) {//如果该点的pre_point为空指针，则将初始点的速度赋值给v0
    //起始点速度
    v0 = init_point_.v();
  } else {
    //其他点的速度 v = delta_s/delta_t;
    v0 = (point.index_s() - point.pre_point()->index_s()) * unit_s_ / unit_t_;
     // AINFO_IF(FLAGS_enable_debug_motion)<<"pre_point s :"<<point.pre_point()->index_s();
  }
  // AINFO_IF(FLAGS_enable_debug_motion)<<"v0 :"<<v0<<",point.index_s():"<<point.index_s();
  //cost_table_中最后一个时间采样值所包含的距离采样值数目
  const auto max_s_size = cost_table_.back().size() - 1;//应该为100
  const double speed_coeff = unit_t_ * unit_t_;
  //最大加速度情形下所允许的最大距离
  //@pqg
  double max_acceleration = max_acceleration_;
  double max_deceleration = max_deceleration_;
  const double delta_s_upper_bound =
         v0 * unit_t_ + 0.5 * max_acceleration * speed_coeff;//s = vt+1/2*a*t*t
  *next_highest_row =
      point.index_s() + static_cast<int>(delta_s_upper_bound / unit_s_);//得到以最大加速度加速上去，对应的下一个点的s值（换算成以单位s为计算的离散s）
  if (*next_highest_row >= max_s_size) {
    *next_highest_row = max_s_size;
  }
  // AINFO_IF(FLAGS_enable_debug_motion)<<"*next_highest_row :"<<*next_highest_row;
  
  const double delta_s_lower_bound = std::fmax(
         0.0, v0 * unit_t_ + 0.5 * max_deceleration * speed_coeff);
  *next_lowest_row =
      point.index_s() + static_cast<int>(delta_s_lower_bound / unit_s_);
  if (*next_lowest_row > max_s_size) { //限幅
    *next_lowest_row = max_s_size;
  } else if (*next_lowest_row < 0) {
    *next_lowest_row = 0;
  }
}

//使用动态规划算法计算(c,r)点的总代价
//c:时间坐标，即横坐标
//r：距离坐标，即纵坐标
void DpStGraph::CalculateCostAt(const std::shared_ptr<StGraphMessage>& msg) {
  const uint32_t c = msg->c;
  const uint32_t r = msg->r;
  auto& cost_cr = cost_table_[c][r]; //是一个StGraphPoint类型
  //1.获取并设置当前点的障碍物代价
  if (st_graph_data_.st_boundaries().empty()) {
    cost_cr.SetObstacleCost(0.0);//无障碍物时障碍物的cost设置为零
  } else {
    cost_cr.SetObstacleCost(dp_st_cost_.GetObstacleCost(cost_cr)); //将代价存入cost_table_[c][r]的obstacle_cost_成员变量中
  }
  
  // AINFO_IF(FLAGS_enable_debug_motion)<<"cost_table_["<<c<<"]["<<r<<"] obstacle_cost is "<<cost_cr.obstacle_cost();
  //当前点的障碍物代价无穷大，直接返回
  if (cost_cr.obstacle_cost() > std::numeric_limits<double>::max()) {
    return;
  }
  
  //初始代价
  const auto& cost_init = cost_table_[0][0];
  //第一个时间采样值为c（即时间t）==0,因此r(即距离值)必须为零，表示是起点，代价值自然设置为零
  if (c == 0) {
    DCHECK_EQ(r, 0) << "Incorrect. Row should be 0 with col = 0. row: " << r;
    cost_cr.SetTotalCost(0.0);
    return;
  }
  
  //获取速度限制条件，前面通过speed_limit_decider.GetSpeedLimits()获取，包括地图限速，曲率限制（包括向心加速度，向心加加速度的限制），路旁障碍物
  double speed_limit =
      st_graph_data_.speed_limit().GetSpeedLimitByS(unit_s_ * r);
  
  //第二个时间采样
  if (c == 1) {
    //加速度或者减速度超出范围，返回
    const double acc = (r * unit_s_ / unit_t_ - init_point_.v()) / unit_t_;
    // AINFO_IF(FLAGS_enable_debug_motion)<<"acc :"<<acc<<",current point v:"<<r * unit_s_ / unit_t_;
    if (acc < dp_st_speed_config_.max_deceleration() ||
        acc > dp_st_speed_config_.max_acceleration()) {
      AWARN_IF(FLAGS_enable_debug_motion)<<  "is not satisfy max_deceleration or max_acceleration";
      return;
    }
    
    //当前点与前序节点的连线穿过障碍物的st_boundary时，该节点不可取，返回，其total_cost保留初始值（无穷大值）
    if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                cost_init)) {
      AWARN_IF(FLAGS_enable_debug_motion)<<  "OverlapOnDpStGraph ";
      return;
    }
    //设置当前点的代价值
    cost_cr.SetTotalCost(cost_cr.obstacle_cost() + cost_init.total_cost() +
                         CalculateEdgeCostForSecondCol(r, speed_limit));
    // AINFO_IF(FLAGS_enable_debug_motion)<<"cost_table_["<<c<<"]["<<r<<"] cost_init.total_cost is "<<cost_init.total_cost() 
        //                      <<", link cost :"<< CalculateEdgeCostForSecondCol(r, speed_limit);
    //设置其前序节点为起点
    cost_cr.SetPrePoint(cost_init);
    return;
  }

  constexpr double kSpeedRangeBuffer = 0.20;
  //允许的最大距离采样差值
  const uint32_t max_s_diff =
      static_cast<uint32_t>( st_graph_data_.cruise_speed() * //FLAGS_planning_upper_speed_limit *
                            (1 + kSpeedRangeBuffer) * unit_t_ / unit_s_);  //s = v*t
  //确定搜索前序节点时的最小距离采样值
  const uint32_t r_low = (max_s_diff < r ? r - max_s_diff : 0);
  //上一个时间采样值下不同采样距离的代价数组
  const auto& pre_col = cost_table_[c - 1];

  //第三个时间采样值
  if (c == 2) {
    for (uint32_t r_pre = r_low; r_pre <= r; ++r_pre) {
      //从距离采样值r_pre到r所需的加速度
      const double acc =
          (r * unit_s_ - 2 * r_pre * unit_s_) / (unit_t_ * unit_t_);
      //若加速度越界，则忽略该距离采样值
      if (acc < dp_st_speed_config_.max_deceleration() ||
          acc > dp_st_speed_config_.max_acceleration()) {
        continue;
      }
      //当前点与前一时间采样值上的节点有重叠，忽略该距离采样值，返回进入计算下一个节点
      if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                  pre_col[r_pre])) {
        continue;
      }
      
      //计算当前节点（c,r）的代价值
      const double cost = cost_cr.obstacle_cost() +//当前点的障碍物cost
                          pre_col[r_pre].total_cost() +//前一个点的cost
                          CalculateEdgeCostForThirdCol(r, r_pre, speed_limit);//其他连接cost,包括速度，加速度，加加速度
      //若新代价值比节点（c,r）的原有代价更小，则更新当前节点（c,r）的总代价值
      if (cost < cost_cr.total_cost()) {
        cost_cr.SetTotalCost(cost);
        // AINFO_IF(FLAGS_enable_debug_motion)<<"cost_table_["<<c<<"]["<<r<<"] cost_init.total_cost is "<<pre_col[r_pre].total_cost() 
                //                  <<", link cost :"<< CalculateEdgeCostForThirdCol(r, r_pre, speed_limit);
        cost_cr.SetPrePoint(pre_col[r_pre]);
      }
    }
    return;
  }

  //其他的时间采样值
  for (uint32_t r_pre = r_low; r_pre <= r; ++r_pre) {

    //若节点（c-1,r_pre）上的总代价无穷大或前一次时间采样c-1上没有前序节点，忽略该节点
    if (std::isinf(pre_col[r_pre].total_cost()) ||
        pre_col[r_pre].pre_point() == nullptr) {
      continue;
    }
    
    //从r_pre抵达r所需的加速度
    const double curr_a = (cost_cr.index_s() * unit_s_ +
                           pre_col[r_pre].pre_point()->index_s() * unit_s_ -
                           2 * pre_col[r_pre].index_s() * unit_s_) /
                          (unit_t_ * unit_t_);
    //@pqg                       
    double max_acceleration = max_acceleration_;
    double max_deceleration = max_deceleration_;
 
    if (curr_a < dp_st_speed_config_.max_deceleration() ||
          curr_a > dp_st_speed_config_.max_acceleration()) { 
      continue;
    }
    if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                pre_col[r_pre])) {
      continue;
    }

    //上上次距离采样值
    uint32_t r_prepre = pre_col[r_pre].pre_point()->index_s();
    //ST图上的上上个点
    const StGraphPoint& prepre_graph_point = cost_table_[c - 2][r_prepre];
    //若上上个节点总代价无穷大，忽略
    if (std::isinf(prepre_graph_point.total_cost())) {
      continue;
    }

    //若上上个节点没有前序节点，忽略
    if (!prepre_graph_point.pre_point()) {
      continue;
    }

    //上上上个节点
    const STPoint& triple_pre_point = prepre_graph_point.pre_point()->point();
    //上上个节点
    const STPoint& prepre_point = prepre_graph_point.point();
    //上个节点
    const STPoint& pre_point = pre_col[r_pre].point();
    //当前节点
    const STPoint& curr_point = cost_cr.point();
    //计算从上上上个节点、上上个节点，上个节点与当前节点之间的最小连接代价
    double cost = cost_cr.obstacle_cost() + pre_col[r_pre].total_cost() +
                  CalculateEdgeCost(triple_pre_point, prepre_point, pre_point,
                                    curr_point, speed_limit);

    //更新代价
    if (cost < cost_cr.total_cost()) {
      // AINFO_IF(FLAGS_enable_debug_motion)<<"cost_table_["<<c<<"]["<<r<<"] other_cost is "<<cost-cost_cr.obstacle_cost();
      cost_cr.SetTotalCost(cost);
      cost_cr.SetPrePoint(pre_col[r_pre]);
    }
  }
}

//获取代价值最小的速度数据
Status DpStGraph::RetrieveSpeedProfile(SpeedInfo* const speed_data) {
  double min_cost = std::numeric_limits<double>::infinity();//最小代价值
  const StGraphPoint* best_end_point = nullptr;//最优终点（即包含最小代价值的末节点）
  // AINFO_IF(FLAGS_enable_debug_motion)<<"cost_table_ size :"<<cost_table_.size();
  // cost_table_.back()存储的是最后一个时间采样值上不同距离的代价值
  for (const StGraphPoint& cur_point : cost_table_.back()) {
    if (!std::isinf(cur_point.total_cost()) &&
        cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
    }
  }

  // 遍历cost_table_中的每一行，即第一个、第二个、......、最后一个时间采样值上的
  // 的back元素对应的代价值，将这些节点与现有最优终点best_end_point比较，
  // 不断更新最小代价值min_cost和最优终点best_end_point，
  // 直至找到全局最优的终点
  for (const auto& row : cost_table_) {
    const StGraphPoint& cur_point = row.back();
    if (!std::isinf(cur_point.total_cost()) &&
        cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
    }
  }

  // 寻找最优终点失败
  if (best_end_point == nullptr) {
    const std::string msg = "Fail to find the best feasible trajectory.";
    AWARN_IF(FLAGS_enable_debug_motion)<<  msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }

  //设置最优终点的速度数据，并找出器连接的倒数第二个、倒数第三个直至第一个时间节点
  //分别设置这些时间节点的速度数据
  std::vector<SpeedPoint> speed_profile;
  const StGraphPoint* cur_point = best_end_point;
  while (cur_point != nullptr) {
    // AINFO_IF(FLAGS_enable_debug_motion)<<"s:"<<cur_point->index_s()<<",t:"<<cur_point->index_t()
        //       <<",total cost: "<<cur_point->total_cost();
    SpeedPoint speed_point;
    speed_point.set_s(cur_point->point().s());
    speed_point.set_t(cur_point->point().t());
    speed_profile.emplace_back(speed_point);
    cur_point = cur_point->pre_point();
  }
  //将速度数据按起点到终点的顺序重新排序
  std::reverse(speed_profile.begin(), speed_profile.end());
 
  //若速度数据中起始点的时间t或者距离s大于编译器定义的最小双精度浮点数（即起始点的时间t或者距离s不为零），则视结果为错误输出
  constexpr double kEpsilon = std::numeric_limits<double>::epsilon();
  if (speed_profile.front().t() > kEpsilon ||
      speed_profile.front().s() > kEpsilon) {
    const std::string msg = "Fail to retrieve speed profile.";
    AWARN_IF(FLAGS_enable_debug_motion)<< msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }

  size_t speed_profile_size = speed_profile.size();
  for (size_t i = 0; i  < speed_profile_size; ++i) {
    if (i == 0) speed_profile[i].set_v(init_point_.v());
    else {
      const double v = (speed_profile[i].s() - speed_profile[i - 1].s()) /
                     (speed_profile[i].t() - speed_profile[i - 1].t() + 1e-3);
      speed_profile[i].set_v(v);
      // AINFO_IF(FLAGS_enable_debug_motion)<<"i:"<<i<<",v:"<<v<<",s:"<<speed_profile[i].s();
    } 
  }

  *speed_data = SpeedInfo(speed_profile);
  return Status::OK();
}

//计算第一、二、三、四个节点之间的连接代价
double DpStGraph::CalculateEdgeCost(const STPoint& first, const STPoint& second,
                                    const STPoint& third, const STPoint& forth,
                                    const double speed_limit) {
  return dp_st_cost_.GetSpeedCost(third, forth, speed_limit) +
         dp_st_cost_.GetAccelCostByThreePoints(second, third, forth) +
         dp_st_cost_.GetJerkCostByFourPoints(first, second, third, forth);
}

//为第二列（即第三个时间采样值上的代价数值）计算连接代价
//输入参数为速度限制及r列数值
double DpStGraph::CalculateEdgeCostForSecondCol(const uint32_t row,
                                                const double speed_limit) {
  double init_speed = init_point_.v();//前面规划初始点的速度
  double init_acc = init_point_.a();//前面规划初始点的加速度
  const STPoint& pre_point = cost_table_[0][0].point();//前一个ST点
  const STPoint& curr_point = cost_table_[1][row].point();//当前需处理的ST点
  return dp_st_cost_.GetSpeedCost(pre_point, curr_point, speed_limit) +//获取速度COST
         dp_st_cost_.GetAccelCostByTwoPoints(init_speed, pre_point,//获取两点间加速度COST
                                             curr_point) +
         dp_st_cost_.GetJerkCostByTwoPoints(init_speed, init_acc, pre_point,//获取两点见的dda的cost
                                            curr_point);
}

//为第三列（即第四个时间采样值上的代价数值）计算连接代价
double DpStGraph::CalculateEdgeCostForThirdCol(const uint32_t curr_row,
                                               const uint32_t pre_row,
                                               const double speed_limit) {
  double init_speed = init_point_.v();
  const STPoint& first = cost_table_[0][0].point();
  const STPoint& second = cost_table_[1][pre_row].point();
  const STPoint& third = cost_table_[2][curr_row].point();
  return dp_st_cost_.GetSpeedCost(second, third, speed_limit) +
         dp_st_cost_.GetAccelCostByThreePoints(first, second, third) +
         dp_st_cost_.GetJerkCostByThreePoints(init_speed, first, second, third);
}

}  // namespace planning
}  // namespace acu
