#ifndef RUNNABLE_ENTRY_INCLUDE_PLANNING_H_
#define RUNNABLE_ENTRY_INCLUDE_PLANNING_H_

#include <functional>
#include <thread>
#include <iostream>
#include <future>
#include <memory>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sched.h>
#include <mutex>
#include <thread>
#include <string>

#include "datapool/include/common_typedef.h"
#include "datapool/include/data_pool.h"
#include "common/base/communication/include/communication_base.h"
#include "common/base/time/include/rate.h"
#include "common/base/log/include/log.h"
#include "common/base/thread_pool/include/thread_pool.h"
#include "common/base/config/include/param_config_manager.h"
#include "common/common_define/error_code_define.h"
#include "common/mapcheck/geotool/include/coordtransform.h"
#include "common/toolbox/pursuit_interface/include/pursuit_interface.h"
#include "common/util/util.h"
#include "common/math/math_utils.h"
#include "src/execution/task/include/TaskContext.h"
#include "src/execution/business/include/dummy_business.h"
#include "src/execution/cognition/struct_cognition/struct_cognition.h"
#include "src/execution/behaviorplan/struct_behaviorplan/struct_decision.h"
#include "src/execution/behaviorplan/behavior_base/behavior_context_base.h"
#include "src/execution/behaviorplan/dummy_behavior/include/dummy_behavior.h"
#include "src/execution/motionplan/motionplan.h"
#include "sensor_imu_msgs.pb.h"
#include "mapengine_msgs.pb.h"
#include "vehicle_perception_msgs.pb.h"
#include "control_msgs.pb.h"
#include "prediction_msgs.pb.h" 
#include "planning_msgs.pb.h" 

#include "Timer.h"
#include "Acu_dds.h"
#include "Acu_zmq.h"
#include "acu_shm.h"

#define PLANNING_THREAD_NUM 10

namespace acu {
namespace planning {
class Planning {
  public:
    Planning();
    ~Planning();
    
    int Init();
    int Start();
    
    acu::zmq::Acu_zmq Acu_zmq_;
    acu::zmq::Acu_zmq Acu_zmq_1,Acu_zmq_2,Acu_zmq_3; 
    string topic;
    
  private:
    void CallBack_shm(acu::shm::CCommonTopicMsg *data);
    void CallBack_zmq_rep_datachange(acu::zmq::CCommonTopicMsg *data,acu::zmq::CCommonTopicMsg *rep);
    void CallBack_zmq(acu::zmq::CCommonTopicMsg *data, int size);
    void CallBack_dds(void* data, uintmax_t size);
    void CallBack_timer(void);

    int Process();
  protected:
    int InitDataPool();
    int InitThreads();
    int InitVars();
    int InitMotionplan();
    int GetParamConfig();
  protected:
    DataPool *data_pool_;
    
    void ResetStructPlanning();
    void CheckEnvironment();
    void TaskProcess();
    void CognitionProcess();
    void BusinessProcess();
    void BehaviorProcess();
    void UnstructBehaviorProcess();
    void StructBehaviorProcess();
    void MotionplanProcess();
    void PathplanProcess();
    void SpeedplanProcess();
    void CheckFault();
    void CutReversePath();
    void TurningLight(const PathData &local_path, const double &velocity, int &result_turning);
    int TurningPath(const PathData &local_path, double aimdist, double delaydist);

    //dds callback and pub 

    sensor_imu_msgs::SensorImu imu_msg_;
    vehicle_perception_msgs::VehiclePerception veh_perception_msg_;
    control_msgs::ControlFeedback control_feedbach_msg_;
    mapengine_msgs::Navigation navigation_msg_;
    mapengine_msgs::NavMission navmission_msg_;
    prediction_msgs::PredictionObjects prediction_msg_;
    void CallbackImuDdS(void* data, uintmax_t size);        //定位 tpimu
    void CallbackPerceptionDds(void* data, uintmax_t size); //感知
    void CallbackControlBoxDdS(void* data, uintmax_t size); //控制反馈
    void CallbackMapengineDds(void* data, uintmax_t size);  //地图引擎实时导航
    void CallbackPredictionDds(void* data, uintmax_t size); //预测
    void CallbackNavmissionDds(void* data, uintmax_t size); //地图引擎导航任务

    void CallBackZmqRepTaskCmd(acu::zmq::CCommonTopicMsg *data,acu::zmq::CCommonTopicMsg *rep);
    void CallBackZmqRepBehaviorLimit(acu::zmq::CCommonTopicMsg *data,acu::zmq::CCommonTopicMsg *rep);
    void CallBackZmqRepInquireStatus(acu::zmq::CCommonTopicMsg *data,acu::zmq::CCommonTopicMsg *rep);

    void CallBackInquireStatusDDS (void* data, uintmax_t size);
    void CallBackTaskCmdDDS (void* data, uintmax_t size);

    //int CallbackTaskCmd(const planning_api_msgs::PlanningCmd &input,
    //                  planning_api_msgs::PlanningResult &output);
    //int CallbackBehaviorLimit(const planning_api_msgs::BehaviorLimitCmd& input,
    //                        planning_api_msgs::PlanningResult& output);
    //int CallbackInquireStatus(const std_msgs::UInt8 &input,
    //                        planning_api_msgs::PlanningStatus& output);


    planning_msgs::Trajectory planing_msg_; //tppathplan
    planning_debug_msgs::PlanningDebug planning_debug_msg_; //planningdebug
    planning_msgs::BusinessFeedback business_msgs_; //tptask_fb 
    void PublishPlanningMsg();
    void PublishPlanningDebugMsg();
    void PublishBusinessMsg();  

    void FiltLocalizationVelocity(double& velocity);

    planning_msgs::Trajectory PathToMsgMotionPath (const PathData &local_path) {
    planning_msgs::Trajectory msg_path;
    for (auto &p : local_path.path) {
      planning_msgs::TrajectoryPoint point;
      point.set_x(p.x);
      point.set_y(p.y);
      point.set_xg(p.xg);
      point.set_yg(p.yg);
      point.set_angle(p.angle);
      point.set_angleglobal(p.globalangle);
      point.set_velocity(p.velocity);
      point.set_curvature(p.curvature);
      point.set_length(p.length);
      point.set_a(p.index);
      point.set_t(p.origin_index);
      point.set_direction(p.reverse == false ? 0 : 1);
      msg_path.add_points()->CopyFrom(point);
    }
    return msg_path;
  }
  //end dds
  inline double GetNowTimeSec(){
       // 获取当前时间点
       auto now = std::chrono::system_clock::now();
       // 转换为time_t类型
       std::time_t now_c = std::chrono::system_clock::to_time_t(now);
       // 转换为自UNIX纪元以来的秒数
       double now_seconds = static_cast<double>(now_c);
       return now_seconds;
  }
  inline double GetNowTimeMs(){
      // 获取当前时间点
      auto now = std::chrono::system_clock::now();
      // 计算自UNIX纪元以来的微秒数
      auto now_ms = std::chrono::time_point_cast<std::chrono::microseconds>(now).time_since_epoch().count();
      return static_cast<double>(now_ms);
  }
  protected:
    TaskContext task_context_;
    //UnstructCognition unstruct_cognition_;
    //std::shared_ptr<acu::motionplan::SpeedplanBase> speed_plan_ptr_;
    
    std::shared_ptr<acu::planning::StructDecision> struct_decision_ptr_;
    std::shared_ptr<acu::planning::MotionPlan> struct_motionplan_ptr_;
    std::shared_ptr<acu::planning::StructCognition> struct_cognition_ptr_;
    bool is_init_;
    int turning_count_;
    string version_;
    int imu_data_size_;
    int map_data_size_;
    bool callback_perception_flag_ = false;
    double perception_transfer_time_ = 0.0;
    bool callback_prediction_flag_ = false;
    double prediction_transfer_time_ = 0.0;
};
}  // namespace planning
}  // namespace acu
#endif  // RUNNABLE_ENTRY_INCLUDE_PLANNING_H_
