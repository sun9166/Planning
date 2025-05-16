
#include "planning_node.h"

namespace acu {
namespace planning {

acu::dds::Acu_dds* Acu_ddsPtr = new acu::dds::Acu_dds();

Planning::Planning() {}

Planning::~Planning() {}

int Planning::Init() {
  
  //imu_data_size_ = 80;
  //map_data_size_ = 6;
  imu_data_size_ = 1;
  map_data_size_ = 1;

  auto DP = acu::planning::DataPool::Instance()->GetMainDataPtr();
  DP->mapengine_data_list.clear();
  DP->mapengine_data_list.reserve(map_data_size_);
  DP->loc_perception.loc_data_list.clear();
  DP->loc_perception.loc_data_list.reserve(imu_data_size_);

  is_init_ = false;
  turning_count_ = 0;
  RETURN_VAL_IF(InitDataPool() == -1, -1);
  RETURN_VAL_IF(InitVars() == -1, -1);
  RETURN_VAL_IF(GetParamConfig() == -1, -1);
  
  //communication_->Init();
  //init publist and subscribe
  
  std::function<void(void* data, uintmax_t size)> f_imu_dds = 
                                 std::bind(&Planning::CallbackImuDdS, this, std::placeholders::_1,std::placeholders::_2);
  std::function<void(void* data, uintmax_t size)> f_perception_dds = 
                                 std::bind(&Planning::CallbackPerceptionDds, this, std::placeholders::_1,std::placeholders::_2);
  std::function<void(void* data, uintmax_t size)> f_controlbox_dds = 
                                 std::bind(&Planning::CallbackControlBoxDdS, this, std::placeholders::_1,std::placeholders::_2);
  std::function<void(void* data, uintmax_t size)> f_mapengine_dds = 
                                 std::bind(&Planning::CallbackMapengineDds, this, std::placeholders::_1,std::placeholders::_2);      
  std::function<void(void* data, uintmax_t size)> f_prediction_dds = 
                                 std::bind(&Planning::CallbackPredictionDds, this, std::placeholders::_1,std::placeholders::_2);
  std::function<void(void* data, uintmax_t size)> f_navimison_dds = 
                                 std::bind(&Planning::CallbackNavmissionDds, this, std::placeholders::_1,std::placeholders::_2);
  Acu_ddsPtr->Acu_dds_InitSubscribe(t_dds_sub[0].topic_name,f_imu_dds);       // tpimu
  Acu_ddsPtr->Acu_dds_InitSubscribe(t_dds_sub[1].topic_name,f_perception_dds);// tpperception
  Acu_ddsPtr->Acu_dds_InitSubscribe(t_dds_sub[2].topic_name,f_controlbox_dds);// tpcontrolfeedback
  Acu_ddsPtr->Acu_dds_InitSubscribe(t_dds_sub[3].topic_name,f_mapengine_dds); // mapengine/tpnavigation
  Acu_ddsPtr->Acu_dds_InitSubscribe(t_dds_sub[4].topic_name,f_prediction_dds);// tpprediction
  Acu_ddsPtr->Acu_dds_InitSubscribe(t_dds_sub[5].topic_name,f_navimison_dds); // mapengine/tpnavmission
  
  Acu_ddsPtr->Acu_dds_InitPublish(t_dds_pub[0].topic_name); // tppathplan
  Acu_ddsPtr->Acu_dds_InitPublish(t_dds_pub[1].topic_name); // tptask_fb
  Acu_ddsPtr->Acu_dds_InitPublish(t_dds_pub[2].topic_name); // planningdebug

  
  std::function<void(void* data, uintmax_t size)> f_taskcmd_dds = 
                                 std::bind(&Planning::CallBackTaskCmdDDS, this, std::placeholders::_1,std::placeholders::_2);
  Acu_ddsPtr->Acu_dds_InitSubscribe("PlanningCmd",f_taskcmd_dds); 
  Acu_ddsPtr->Acu_dds_InitPublish("PlanningResult"); 
  

    planning_msgs::PlanningCmd input;
  planning_msgs::PlanningStatus output;
  std::function<void(void* data, uintmax_t size)> f_inquire_status_dds = 
                                 std::bind(&Planning::CallBackInquireStatusDDS, this, std::placeholders::_1,std::placeholders::_2);
  Acu_ddsPtr->Acu_dds_InitSubscribe("CheckPlanningStatus",f_inquire_status_dds); 
  Acu_ddsPtr->Acu_dds_InitPublish("PlanningStatus"); 
  
  std::cout << "start init zmq" << std::endl;

  if (t_zmq_rep[0].topic_name!="") {
      std::function<void(acu::zmq::CCommonTopicMsg *data,acu::zmq::CCommonTopicMsg *rep)> f_zmq_rep_task_cmd = 
                              std::bind(&Planning::CallBackZmqRepTaskCmd, this, std::placeholders::_1,std::placeholders::_2);
      //Acu_zmq_1.DoRegisterServiceServer(t_zmq_rep[0].topic_name,f_zmq_rep_task_cmd); // planning/service/TaskCmd
  }
  if (t_zmq_rep[1].topic_name!="") {
      std::function<void(acu::zmq::CCommonTopicMsg *data,acu::zmq::CCommonTopicMsg *rep)> f_zmq_rep_behavior_limit = 
                              std::bind(&Planning::CallBackZmqRepBehaviorLimit, this, std::placeholders::_1,std::placeholders::_2);
      //Acu_zmq_2.DoRegisterServiceServer(t_zmq_rep[1].topic_name,f_zmq_rep_behavior_limit); // planning/service/BehaviorLimit
  }
  if (t_zmq_rep[2].topic_name!="") {
      std::function<void(acu::zmq::CCommonTopicMsg *data,acu::zmq::CCommonTopicMsg *rep)> f_zmq_rep_inquire_status = 
                              std::bind(&Planning::CallBackZmqRepInquireStatus, this, std::placeholders::_1,std::placeholders::_2);
      //Acu_zmq_3.DoRegisterServiceServer(t_zmq_rep[2].topic_name,f_zmq_rep_inquire_status); // planning/service/InquireStatus
  }

//

  InitThreads();
  struct_cognition_ptr_->Init();
  struct_decision_ptr_->Init();
  RETURN_VAL_IF(struct_motionplan_ptr_->Init() == -1, -1);
  if (!FLAGS_task_enable) {
    data_pool_->GetMainDataRef().task_content.command = eCommand::COMMAND_START;
    data_pool_->GetMainDataRef().task_content.SetTaskStatus();
    data_pool_->GetMainDataRef().task_content.SetCommandInfo();
  } 
  is_init_ = true;
  return 0;
}


void Planning::CallBack_timer(void)        // timer callback
{
    //std::cout << "Timer expired!" << std::endl;
    /********         add user code         *************/
  //std::lock_guard<std::mutex> lock(acu::planning::DataPool::Instance()->mutex_);
  AERROR <<"-------------------callback timer-------------------------------";
  Process();
  AERROR <<"-------------------Process Process-------------------------------";
  PublishPlanningMsg();
  PublishPlanningDebugMsg();
  PublishBusinessMsg(); 

    
}

int Planning::Start() {

   std::function<void()> f = std::bind(&Planning::CallBack_timer, this);
   acu::Timer::Timer timer(t_dds_pub[0].topic_period, f);               //        t_dds_tx[0].topic_period is send cycle     
   timer.start(); 
   std::shared_ptr<COriginalBytes> data =std::make_shared<COriginalBytes>();
   char buf[100],buf1[256];
   buf[0] = 1;
   buf[1] = 2;
   buf[2] = 3;
   buf[3] = 4;
   data->set_data(buf,100);
   std::shared_ptr<COriginalBytes> repdata =std::make_shared<COriginalBytes>();
   repdata->set_data(buf1,256);
   while(true) {
     // if(t_zmq_pub[0].topic_name!="")
     // {
     //    Acu_zmq_.DoPublish(t_zmq_pub[0].topic_name.c_str(),data);                  //   zmq pub
     // }
     // if(t_zmq_req[0].topic_name!=""&&Acu_zmq_2.zmq_communication.req_connected==true)
     // {
     //     Acu_zmq_2.DoService_req_transmit(t_zmq_req[0].topic_name.c_str(),0,1000000000,data,repdata);
     //     printf("zmq repdata .............................................................\n");
     //     printf("repdata[0] = %x \n",buf1[0]);
     //     printf("repdata[1] = %x \n",buf1[1]);
     //     printf("repdata[2] = %x \n",buf1[2]);
     // }
     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
     //buf[4]++;        
   }
   return 0;
}

int Planning::Process() {

  //TIME_MONITOR_INIT(planning_main);int Planning::Init() {

  //std::lock_guard<std::mutex> lock(acu::planning::DataPool::Instance()->mutex_);
  auto DP = acu::planning::DataPool::Instance()->GetMainDataPtr();
  if (!is_init_) {
    std::cout <<"planning not init!!!"<<std::endl;
    return 0;
  }
  AINFO << "===================planning Start======================";
  double star_t = acu::common::NodeTime::Now().ToSecond();
  DP->debug_planning_msg.header.set_time_stamp(acu::common::NodeTime::Now().ToSecond());
  CheckEnvironment();
  TaskProcess();
  DP->debug_planning_msg.cognition.set_time_stamp(acu::common::NodeTime::Now().ToSecond());
   AINFO << "===================CognitionProcess Start====================== ";
  CognitionProcess();//认知构建 
   AINFO << "===================CognitionProcess End====================== ";
  if (FLAGS_task_enable) {
    BusinessProcess();
    if (DP->business_ptr->GetBusinessType().type == eBusinessType::STRUCT_BUSINESS &&
      DP->business_ptr->IsInit() && DP->cognition_info.maptype == eMapType::STRUCTURE_ROAD) {
      AINFO << "=============[MAIN PROCESS]:STRUCT===================";
      DP->debug_planning_msg.decision.set_time_stamp(acu::common::NodeTime::Now().ToSecond());
      StructBehaviorProcess();//行为决策
      DP->debug_planning_msg.motionplan_debug.set_time_stamp(acu::common::NodeTime::Now().ToSecond());
      struct_motionplan_ptr_->Process();//运动规划
    } else {
      AWARN << "==============[MAIN PROCESS]:DUMMY, map:===============" << (int)DP->cognition_info.maptype
          << ",business:" << DP->business_ptr->GetBusinessType().type_str;
      DP->motion_path.path.clear();
      ResetStructPlanning();
    }
  } else {
    StructBehaviorProcess();
    struct_motionplan_ptr_->Process();
  }   
  //communication_->Publish();
  double delta_t = acu::common::NodeTime::Now().ToSecond() - star_t;
  DP->debug_planning_msg.time_cost_ms = 1000.0 * delta_t;
  AINFO << "===================planning End====================== "<<delta_t;
  if (delta_t > 0.08) {
    AINFO << "time is more than 80 ms";
  }
  return 0;
}

int Planning::InitDataPool() {
  data_pool_ = acu::planning::DataPool::Instance();
  RETURN_VAL_IF_NULL(data_pool_, -1);
  data_pool_->GetMainDataRef().cognition_info.maptype = eMapType::UNSTRUCTURE_ROAD;
  return 0;
}


int Planning::InitVars() {
  //communication_ = std::make_shared<Rosbridge>();
 // speed_plan_ptr_ = std::make_shared<acu::motionplan::UnstructSpeedplan>();
  struct_decision_ptr_ = std::make_shared<acu::planning::StructDecision>();
  struct_motionplan_ptr_ = std::make_shared<acu::planning::MotionPlan>();
  struct_cognition_ptr_ = std::make_shared<acu::planning::StructCognition>();
  //RETURN_VAL_IF_NULL(communication_, -1);
 // RETURN_VAL_IF_NULL(speed_plan_ptr_, -1);
  RETURN_VAL_IF_NULL(struct_decision_ptr_, -1);
  RETURN_VAL_IF_NULL(struct_motionplan_ptr_, -1);
  return 0;
}

int Planning::GetParamConfig() {
  std::string src_path = "/map/work/";
  //if (acu::common::util::GetSrcPath(src_path) == false) {
  //  return -1;
  //}
  std::string top_config_file = src_path + "config/car_select.conf";
  ParamConfigManager param_config_manager;
  param_config_manager.Reset();
  if (param_config_manager.ModuleConfigLoad(top_config_file) == false) {
    AERROR << "cannot load config file " << top_config_file;
    return -1;
  }
  ModelConfig *pmodel_config = new ModelConfig();
  param_config_manager.GetModelConfig("car_select", &pmodel_config);
  std::string current_carname, car_type;
  if (pmodel_config->GetValue("current_carname", &current_carname) == false) {
    AERROR << "cannot get param current_carname";
    return -1;
  }
  if (pmodel_config->GetValue("car_type", &car_type) == false) {
    AERROR << "cannot get param car_type";
    return -1;
  }
  delete pmodel_config;
  AINFO<<"current_carname "<<current_carname;

  pmodel_config = new ModelConfig();
  param_config_manager.Reset();
  if (param_config_manager.ModuleConfigLoad(src_path + "config/" + current_carname + "/vehicle_param.conf") == false) {
    AERROR << "cannot load config file " << src_path + "config/" + current_carname + "/vehicle_param.conf";
    return -1;
  }

  param_config_manager.GetModelConfig("vehicle_interface", &pmodel_config);
  auto DP = acu::planning::DataPool::Instance()->GetMainDataPtr();
  DP->config_info.car_type = car_type;
  if (DP->config_info.car_model.LoadVeicleParamsFromModel(pmodel_config) == false) {
    AERROR << "cannot get planning's vechile param";
    return -1;
  }
  delete pmodel_config;


  pmodel_config = new ModelConfig();
  param_config_manager.Reset();
  if (param_config_manager.ModuleConfigLoad(src_path + "config/" + current_carname + "/planning.conf") == false) {
    AERROR << "cannot load config file " << src_path + "config/" + current_carname + "/planning.conf";
    return -1;
  }
  param_config_manager.GetModelConfig("behaviorconfig", &pmodel_config);
  if (DP->config_info.behavior_config.LoadBehviorConfigParamsFromModel(pmodel_config) == false) {
    AERROR << "cannot get behaviorconfig param";
    return -1;
  }
  param_config_manager.GetModelConfig("speedplan", &pmodel_config);
  if (DP->config_info.speedplan_config.LoadSpeedplanConfigParamsFromModel(pmodel_config) == false) {
    AERROR << "cannot get speedplan config param";
    return -1;
  }

  // param_config_manager.GetModelConfig("unstruct_motion_config", &pmodel_config);
  // if (speed_plan_ptr_->LoadConfigFromModel(pmodel_config) == false) {
  //   AERROR << "cannot get unstruct_motion_config param";
  //   return -1;
  // }
  param_config_manager.GetModelConfig("business", &pmodel_config);
  if (DP->config_info.business_config.LoadBusinessConfigParamsFromModel(pmodel_config) == false) {
    AERROR << "cannot get behaviorconfig param";
    //return -1;
  }
  


  delete pmodel_config;


  AINFO << "load planning param OK";
  return 0;
}

int Planning::InitThreads() {
  auto thread_pool = ThreadPool::Instance();
  RETURN_VAL_IF_NULL(thread_pool, -1);
  thread_pool->Init(PLANNING_THREAD_NUM);
  usleep(1000);
  return 0;
}

void Planning::ResetStructPlanning() {
  struct_cognition_ptr_->Reset();
  struct_motionplan_ptr_->Reset();
	struct_decision_ptr_->ResetData();
  return;
}

void Planning::CheckEnvironment() {  
  //auto monitor_api = acu::common::MonitorApi::Instance();
  // 1.check mapengine delay
  double current_time = acu::common::NodeTime::Now().ToSecond();
  // AERROR<<"------------monitor----------";
  auto A = &DataPool::Instance()->GetMainDataRef().mapengine_data;
  if (A->mapengine_status.is_init) {
    if (current_time - A->mapengine_status.time > 3.0) {
      //monitor_api->SetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_1_ERROR, 
                        //"MAPENGINE CUT OFF:" + std::to_string(fabs(A->mapengine_status.time)) 
                        //+ "," + std::to_string(fabs(current_time)));
      //monitor_api->SetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_0_ERROR, 
      //                  "MAPENGINE DELAY:" + std::to_string(fabs(A->mapengine_status.time)) 
      //                  + "," + std::to_string(fabs(current_time)));
      AERROR<<"MAPENGINE DELAY.";
      A->mapengine_status.is_valid = false;
    }
    else if (current_time - A->mapengine_status.time > 0.5) {
      // monitor_api->SetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_0_ERROR, 
      //                   "MAPENGINE DELAY:" + std::to_string(fabs(A->mapengine_status.time)) 
      //                   + "," + std::to_string(fabs(current_time)));
      A->mapengine_status.is_valid = false;
      AERROR<<"mapengine_status delay.";
    }
    else if (A->mapengine_status.Recover()) {
      //monitor_api->ResetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_1_ERROR);
      //monitor_api->ResetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_0_ERROR);
    }
  }
  else {
    //monitor_api->ResetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_1_ERROR);
    //monitor_api->ResetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_0_ERROR);
  }

  auto B = &DataPool::Instance()->GetMainDataRef().loc_perception.perception_data;
  if (B->perception_status.is_init) {
    if (current_time - B->perception_status.time > 3.0) {
      //monitor_api->SetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_3_ERROR, 
     //                 "PERCEPTION CUT OFF:" + std::to_string(fabs(B->perception_status.time)) 
      //                + "," + std::to_string(fabs(current_time)));
     // monitor_api->SetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_2_ERROR, 
      //                "PERCEPTION DELAY:" + std::to_string(fabs(B->perception_status.time)) 
      ////                + "," + std::to_string(fabs(current_time)));
      B->perception_status.is_valid = false;
      AERROR<<"PERCEPTION CUT OFF.";
    }
    else if (current_time - B->perception_status.time > 0.5) {
      //monitor_api->SetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_2_ERROR, 
      //               "PERCEPTION DELAY:" + std::to_string(fabs(B->perception_status.time)) 
      //                + "," + std::to_string(fabs(current_time)));
      B->perception_status.is_valid = false;
      AERROR<<"PERCEPTION DELAY.";
    }
    else if (B->perception_status.Recover()) {
     // monitor_api->ResetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_3_ERROR);
      ////monitor_api->ResetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_2_ERROR);
    }
  }
  else {
    // monitor_api->ResetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_3_ERROR);
    // monitor_api->ResetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_2_ERROR);
  }

  auto C = &DataPool::Instance()->GetMainDataRef().loc_perception.localization_data;
  if (C->imu_status.is_init) {
    if (current_time - C->imu_status.time > 3.0) {
      // monitor_api->SetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_5_ERROR, 
      //                               "IMU CUT OFF:" + std::to_string(fabs(C->imu_status.time)) 
      //                               + "," + std::to_string(fabs(current_time)));
      // monitor_api->SetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_4_ERROR, 
      //                               "IMU DELAY:" + std::to_string(fabs(C->imu_status.time)) 
      //                               + "," + std::to_string(fabs(current_time)));
      C->imu_status.is_valid = false;
      AERROR<<"IMU CUT OFF.";
    }
    else if (current_time - C->imu_status.time > 0.15) {
      //monitor_api->SetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_4_ERROR, 
      //                            "IMU DELAY:" + std::to_string(fabs(C->imu_status.time)) 
      //                            + "," + std::to_string(fabs(current_time)));
      C->imu_status.is_valid = false;
      AERROR<<"IMU DELAY.";
    }
    else if (C->imu_status.Recover()) {
      //monitor_api->ResetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_5_ERROR);
     // monitor_api->ResetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_4_ERROR);
    }
  }
  else {
    //monitor_api->ResetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_5_ERROR);
   // monitor_api->ResetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_4_ERROR);
  }
  
  auto D = &DataPool::Instance()->GetMainDataRef().drive_status;
  if (D->canbus_status.is_init) {
    if (current_time - D->canbus_status.time > 3.0) {
      // monitor_api->SetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_7_ERROR, 
      //                         "CANBUS CUT OFF:" + std::to_string(fabs(D->canbus_status.time)) 
      //                         + "," + std::to_string(fabs(current_time)));
      // monitor_api->SetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_6_ERROR, 
      //                         "CANBUS DELAY:" + std::to_string(fabs(D->canbus_status.time)) 
      //                         + "," + std::to_string(fabs(current_time)));
      D->canbus_status.is_valid = false;
      AERROR<<"IMU CUT OFF.";
    }
    else if (current_time - D->canbus_status.time > 0.5) {
      // monitor_api->SetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_6_ERROR, 
      //                         "CANBUS DELAY:" + std::to_string(fabs(D->canbus_status.time)) 
      //                         + "," + std::to_string(fabs(current_time)));
      D->canbus_status.is_valid = false;
      AERROR<<"IMU DELAY.";
    }
    else if (D->canbus_status.Recover()) {
      //monitor_api->ResetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_7_ERROR);
      //monitor_api->ResetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_6_ERROR);
    }
  }
  else {
    //monitor_api->ResetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_7_ERROR);
    //monitor_api->ResetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_6_ERROR);
  }

  auto E = &DataPool::Instance()->GetMainDataRef().prediction_data;
  if (E->prediction_status.is_init) {
    if (current_time - E->prediction_status.time > 3.0) {
      //monitor_api->SetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_9_ERROR, 
      //                "PREDICTION CUT OFF:" + std::to_string(fabs(E->prediction_status.time)) 
       //               + "," + std::to_string(fabs(current_time)));
      //monitor_api->SetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_8_ERROR, 
      //                "PREDICTION DELAY:" + std::to_string(fabs(E->prediction_status.time)) 
      //                + "," + std::to_string(fabs(current_time)));
      E->prediction_status.is_valid = false;
      AERROR<<"PREDICTION CUT OFF.";
    }
    else if (current_time - E->prediction_status.time > 0.5) {
      //monitor_api->SetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_8_ERROR, 
      //                "PREDICTION DELAY:" + std::to_string(fabs(E->prediction_status.time)) 
      //                + "," + std::to_string(fabs(current_time)));
      E->prediction_status.is_valid = false;
      AERROR<<"PREDICTION DELAY.";
    }
    else if (E->prediction_status.Recover()) {
      //monitor_api->ResetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_9_ERROR);
      //monitor_api->ResetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_8_ERROR);
    }
  }
  else {
    //monitor_api->ResetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_9_ERROR);
    //monitor_api->ResetFaultInfo(ros::Time::now().toSec(), DFPLANNING_PLANNING_SOFTWARE_8_ERROR);
  }
  // monitor_api->Print();
  // AERROR<<"------------monitor----------";
}

void Planning::TaskProcess() {
  AINFO << "[TaskProcess]------------------";
  task_context_.PullData();
  task_context_.ScanTaskEvent();
  task_context_.PushData();
  /*if (DataPool::Instance()->GetMainDataRef().business_ptr != nullptr) {
    AINFO << "TaskProcess:" << DataPool::Instance()->GetMainDataRef().business_ptr->GetBusinessType().type_str;
  } else {
    AINFO << "TaskProcess_Business_empty";
  } */
}

void Planning::CognitionProcess() {
  AINFO << "[CognitionProcess]------------------";
  auto DP = DataPool::Instance();
  // if (DP->GetMainDataRef().cognition_info.maptype == eMapType::UNSTRUCTURE_ROAD) {
  //   AINFO_IF(FLAGS_log_enable) << "[Cogniton]:Unstruct Process";
  //   unstruct_cognition_.PullData();
  //   unstruct_cognition_.Process();
  //   unstruct_cognition_.PushData();
  //   DP->GetMainDataRef().debug_planning_msg.cognition_msg.SetDebugStateMsg(
  //     "Unstruct Process", 3);
  // }
  if (DP->GetMainDataRef().cognition_info.maptype == eMapType::STRUCTURE_ROAD) {
    AINFO_IF(FLAGS_log_enable)<< "[Cogniton]:struct Process";
    struct_cognition_ptr_->PullData();
    struct_cognition_ptr_->ProcessData();
    struct_cognition_ptr_->PushData();
    DP->GetMainDataRef().debug_planning_msg.cognition_msg.SetDebugStateMsg(
      "Struct Process", 1);         
  }

  if (DP->GetMainDataRef().business_ptr != nullptr) {
    //AINFO << "[Cognition]map:" << (int)DP->GetMainDataRef().cognition_info.maptype
    //        << ",business:" << DP->GetMainDataRef().business_ptr->GetBusinessType().type_str;
  } else {
   // AINFO << "[Cognition]Business_empty";
    //DP->GetMainDataRef().debug_planning_msg.cognition_msg.SetDebugStateMsg(
    // "null", 0);
  }
}

void Planning::BusinessProcess() {
  AINFO << "[BusinessProcess]------------------";
  auto DP = DataPool::Instance();
  if (DP->GetMainDataRef().business_ptr == nullptr) {
    DP->GetMainDataRef().business_ptr = std::make_shared<DummyBusiness>();
  }
  DP->GetMainDataRef().business_ptr->PullData();
  if (!DP->GetMainDataRef().business_ptr->IsInit()) {
    AINFO << "[BUSINESS]:INIT-" << DP->GetMainDataRef().business_ptr->GetBusinessType().type_str;
    DP->GetMainDataRef().business_ptr->Init();
    AINFO << "[BUSINESS]:Init() continue";
    ResetStructPlanning();
  } else {
    AINFO << "[BUSINESS]:PROCESS";
    DP->GetMainDataRef().business_ptr->Process();
  }
  DP->GetMainDataRef().business_ptr->PushData();
}


void Planning::StructBehaviorProcess() {
  AINFO << "[StructBehaviorProcess]------------------";
  auto DP = DataPool::Instance();
  struct_decision_ptr_->PullData();
  struct_decision_ptr_->Process();
  struct_decision_ptr_->PushData();
}



void Planning::TurningLight(const PathData &local_path, const double &velocity, int &result_turning) {
  if (local_path.path.size() < 5) {
    result_turning = 0;
    return;
  }
  int kAimDistance = 1.3;
  int turning = 0;
  int turnIndex = 0;
  int turnValue[5] = {0, 0, 0, 0, 0};
  double delay_dis = velocity * 2.0 > 0 ? velocity * 2.0 : 0;
  // step 1: get 5 points in the raw path, calculate each turning value,
  //         then find the biggest turning value
  for (int i = 0; i < 5; i++) {
    turnValue[i] = (5-i) * TurningPath(local_path, i / 2.0 + kAimDistance, delay_dis);
    if (fabs(turnValue[i]) > turning) {
      turning = std::fabs(turnValue[i]);
      turnIndex = i;
    }
    if (turnValue[i] != 0) break;
  }
  // step 2: count delay according to the biggest turning value
  if (turnValue[turnIndex] > 0) {
    turning_count_++;
  } else if (turnValue[turnIndex] < 0) {
    turning_count_--;
  } else {
    turning_count_ = 0;
  }
  // step 3: get the status of the turning lamp according to the count 
  if (turning_count_ > 10) {
    result_turning = 2;  // right
    turning_count_ = 10;
  } else if (turning_count_ < -10) {
    result_turning = 1;  // left
    turning_count_ = -10;
  } else {
    result_turning = 0;
  }
  return;
}

int Planning::TurningPath(const PathData &local_path, double aimdist, double delaydist) {
  int turning = 0;
  int nearnum = 0;
  double startdis = 0.0;
  geometry::SiteVec vec_local_path;
  for (auto single_point : local_path.path) {
    vec_local_path.push_back(single_point);
  }
  for (int i = 0; i < vec_local_path.size()-2; i++) {
    double single_dis = std::hypot(vec_local_path[i].x-vec_local_path[i+1].x,
                                   vec_local_path[i].y-vec_local_path[i+1].y);
    startdis += single_dis;
    if (startdis > delaydist) {
      nearnum = i;
      break;
    }
  }

  geometry_msgs::Point32 aimpoint;
  for (int i = nearnum; i < vec_local_path.size() - 2; i++) {
    aimpoint.x = vec_local_path.at(i).x;
    aimpoint.y = vec_local_path.at(i).y;
    aimpoint.z = vec_local_path.at(i).angle;
    if (std::hypot(vec_local_path[i].x - vec_local_path[nearnum].x,
                   vec_local_path[i].y - vec_local_path[nearnum].y) > aimdist) {
      break;
    }
  }
  float eta = atan2(aimpoint.y - vec_local_path[nearnum].y,
                    aimpoint.x - vec_local_path[nearnum].x);
  if (eta > 7.0 * 3.14 / 180) {
    turning = -1;
  } else if (eta < -7.0 * 3.14 / 180) {
    turning = 1;
  }
  return turning;
}

void Planning::CallbackImuDdS(void* data, uintmax_t size){
  if (size > 0 && data != nullptr) {
    std::lock_guard<std::mutex> lock(acu::planning::DataPool::Instance()->mutex_);
    AERROR<<"----------callback imu ------------------";
    imu_msg_.ParseFromArray(data, size);
    auto DP = acu::planning::DataPool::Instance()->GetMainDataPtr();
    if ((int)DP->config_info.car_model.sync_type == 0 && imu_data_size_ != 2) {
     imu_data_size_ = 1;
     map_data_size_ = 1;
    }
    auto dp_locpercep = &DP->loc_perception;
    LocalizationData temp_localization_data;
    dp_locpercep->localization_data.imu_status.CheckInit(imu_msg_.header().time_statistics().sending_timestamp());
    temp_localization_data.time = imu_msg_.header().time_stamp();
    temp_localization_data.xg = imu_msg_.pose().pose_euler().position().x().variable();
    temp_localization_data.yg = imu_msg_.pose().pose_euler().position().y().variable();
    temp_localization_data.yaw = imu_msg_.pose().pose_euler().rotation().yaw().variable();
    temp_localization_data.velocity = imu_msg_.v().variable();
    FiltLocalizationVelocity(temp_localization_data.velocity);
    temp_localization_data.vx = imu_msg_.twist().velocity().x().variable();
    temp_localization_data.vy = imu_msg_.twist().velocity().y().variable();
    temp_localization_data.yawrate = imu_msg_.twist().angular_velocity().z().variable();
    /**********dr*********/
    temp_localization_data.loc_xg_dr = imu_msg_.dr_pose_and_time().pose_euler().position().x().variable();
    temp_localization_data.loc_yg_dr = imu_msg_.dr_pose_and_time().pose_euler().position().y().variable();
    temp_localization_data.loc_yaw_dr = imu_msg_.dr_pose_and_time().pose_euler().rotation().yaw().variable();
    /**********dr*********/
    if (dp_locpercep->loc_data_list.size() >= imu_data_size_) {
      dp_locpercep->loc_data_list.pop_back();
    }
    if (!dp_locpercep->loc_data_list.empty() && 
        dp_locpercep->loc_data_list.front().time > temp_localization_data.time) {
      AERROR<<"Imu time wrong!!!!!";
    }
    dp_locpercep->loc_data_list.insert(dp_locpercep->loc_data_list.begin(), temp_localization_data);  
    //AERROR << "imu xg= "<<  temp_localization_data.xg << " ,yg= " <<temp_localization_data.yg
           //<< " ,yaw= "<< temp_localization_data.yaw ;  
  }
}

void Planning::CallbackPerceptionDds(void* data, uintmax_t size) {
  if (data != nullptr && size > 0) {
    std::lock_guard<std::mutex> lock(acu::planning::DataPool::Instance()->mutex_);
    veh_perception_msg_.ParseFromArray(data, size);
    //planning_debug_msg_.mutable_perception()->CopyFrom(veh_perception_msg_.mutable_obstacle_info()->objs());
    callback_perception_flag_ = true;
    perception_transfer_time_ = 1000 * (acu::common::NodeTime::Now().ToSecond()- veh_perception_msg_.header().time_statistics().sending_timestamp());
    auto dp_ci = &acu::planning::DataPool::Instance()->GetMainDataPtr()->loc_perception;
    dp_ci->perception_data.perception_status.CheckInit(veh_perception_msg_.header().time_statistics().sending_timestamp());
    dp_ci->perception_data.timestamp = veh_perception_msg_.header().time_statistics().sending_timestamp();
    dp_ci->perception_data.objects.clear();
    for (int i = 0; i < veh_perception_msg_.obstacle_info().objs().size(); i++) {
      CallbackObject single_object;
      single_object.id = veh_perception_msg_.obstacle_info().objs()[i].id();
      single_object.x = veh_perception_msg_.obstacle_info().objs()[i].x();
      single_object.y = veh_perception_msg_.obstacle_info().objs()[i].y();
      single_object.z = veh_perception_msg_.obstacle_info().objs()[i].z();
      single_object.vxrel = veh_perception_msg_.obstacle_info().objs()[i].vxrel();
      single_object.vyrel = veh_perception_msg_.obstacle_info().objs()[i].vyrel();
      single_object.xabs = veh_perception_msg_.obstacle_info().objs()[i].xabs();
      single_object.yabs = veh_perception_msg_.obstacle_info().objs()[i].yabs();
      single_object.vxabs = veh_perception_msg_.obstacle_info().objs()[i].vxabs();
      single_object.vyabs = veh_perception_msg_.obstacle_info().objs()[i].vyabs();
      single_object.width = veh_perception_msg_.obstacle_info().objs()[i].width();
      single_object.length = veh_perception_msg_.obstacle_info().objs()[i].length();
      single_object.height = veh_perception_msg_.obstacle_info().objs()[i].height();
      single_object.global_angle = veh_perception_msg_.obstacle_info().objs()[i].heading();
      single_object.speed = veh_perception_msg_.obstacle_info().objs()[i].speed();
      single_object.type = (int)veh_perception_msg_.obstacle_info().objs()[i].type();
      single_object.source = (int)veh_perception_msg_.obstacle_info().objs()[i].source();
      single_object.confidence = veh_perception_msg_.obstacle_info().objs()[i].confidence();
      single_object.age = veh_perception_msg_.obstacle_info().objs()[i].age();
      single_object.moving_status = veh_perception_msg_.obstacle_info().objs()[i].moving_status(); 
      for (int j = 0; j < veh_perception_msg_.obstacle_info().objs()[i].cells().size(); j++)  {
        ObjectCell cell;
        cell.idc = veh_perception_msg_.obstacle_info().objs()[i].cells()[j].idc();
        cell.x = veh_perception_msg_.obstacle_info().objs()[i].cells()[j].x();
        cell.y = veh_perception_msg_.obstacle_info().objs()[i].cells()[j].y();
        cell.xg = veh_perception_msg_.obstacle_info().objs()[i].cells()[j].xg();
        cell.yg = veh_perception_msg_.obstacle_info().objs()[i].cells()[j].yg();
        single_object.cells.push_back(cell);
      }
      dp_ci->perception_data.objects.push_back(single_object);
    }
    dp_ci->perception_data.cells.clear();
    for (int i = 0; i < veh_perception_msg_.obstacle_info().cells().size(); i++) {
      ObjectCell cell;
      cell.idc = veh_perception_msg_.obstacle_info().cells()[i].idc();
      cell.x = veh_perception_msg_.obstacle_info().cells()[i].x();
      cell.y = veh_perception_msg_.obstacle_info().cells()[i].y();
      cell.xg = veh_perception_msg_.obstacle_info().cells()[i].xg();
      cell.yg = veh_perception_msg_.obstacle_info().cells()[i].yg();
      dp_ci->perception_data.cells.push_back(cell);
    }

    if (FLAGS_use_calmcar_tfl) {
      //use calmcar perception traffic lights
      dp_ci->perception_data.traffic_lights.clear();
      for (int i = 0; i < veh_perception_msg_.environment_info().traffic_light().traffic_light().size(); i++) {
        TrafficLight single_traffic_light;
        single_traffic_light.color = (eLightColor)veh_perception_msg_.environment_info().traffic_light().traffic_light()[i].color();
        single_traffic_light.state = (eLightState)veh_perception_msg_.environment_info().traffic_light().traffic_light()[i].flicker();
        if (eLightColor::UNKNOWN_COLOR == single_traffic_light.color) continue;
        //黄闪灯、绿闪灯、长黑灯转为绿灯
        if (eLightState::FLICKER_GREEN  ==  single_traffic_light.state || 
            eLightState::FLICKER_YELLOW ==  single_traffic_light.state ||
            eLightState::LENGTH_BLACK   ==  single_traffic_light.state) {
          single_traffic_light.color = eLightColor::GREEN;
        }
        single_traffic_light.id = veh_perception_msg_.environment_info().traffic_light().traffic_light()[i].id();
        single_traffic_light.confidence = veh_perception_msg_.environment_info().traffic_light().traffic_light()[i].confidence();
        single_traffic_light.tracking_time = veh_perception_msg_.environment_info().traffic_light().traffic_light()[i].tracking_time();
        single_traffic_light.number = veh_perception_msg_.environment_info().traffic_light().traffic_light()[i].number();
        single_traffic_light.type = (eLightType)veh_perception_msg_.environment_info().traffic_light().traffic_light()[i].light_type();
        if (eLightType::TYPE_UNKNOWN == single_traffic_light.type)
          single_traffic_light.type = eLightType::TYPE_CIRCLE;
        single_traffic_light.id =std::to_string((int)single_traffic_light.type);
        dp_ci->perception_data.traffic_lights.push_back(single_traffic_light);
      }
    } else {
        dp_ci->perception_data.traffic_lights.clear();
        for (int i = 0; i < veh_perception_msg_.environment_info().traffic_light().traffic_light().size(); i++) {
          TrafficLight single_traffic_light;
          single_traffic_light.color = (eLightColor)veh_perception_msg_.environment_info().traffic_light().traffic_light()[i].color();
          single_traffic_light.state = (eLightState)veh_perception_msg_.environment_info().traffic_light().traffic_light()[i].flicker();
          //绿灯或黑灯下绿闪当做绿灯使用 add by ly
          if ( ( single_traffic_light.color == eLightColor::GREEN || 
                single_traffic_light.color == eLightColor::BLACK )
               && single_traffic_light.state ==eLightState::FLICKER_GREEN) {
            single_traffic_light.color = eLightColor::GREEN;
          }
          if (single_traffic_light.state == eLightState::LENGTH_YELLOW || 
              single_traffic_light.state == eLightState::LENGTH_BLACK) {
            single_traffic_light.color = eLightColor::GREEN;
          }
          single_traffic_light.id = veh_perception_msg_.environment_info().traffic_light().traffic_light()[i].id();
          single_traffic_light.confidence = veh_perception_msg_.environment_info().traffic_light().traffic_light()[i].confidence();
          single_traffic_light.tracking_time = veh_perception_msg_.environment_info().traffic_light().traffic_light()[i].tracking_time();
          single_traffic_light.number = veh_perception_msg_.environment_info().traffic_light().traffic_light()[i].number();
          if (3 == veh_perception_msg_.environment_info().traffic_light().traffic_light()[i].light_cycle().size()){
            single_traffic_light.is_v2x_traffic_light = veh_perception_msg_.environment_info().traffic_light().traffic_light()[i].using_obu();
            single_traffic_light.green_period = veh_perception_msg_.environment_info().traffic_light().traffic_light()[i].light_cycle()[0];
            single_traffic_light.yellow_period = veh_perception_msg_.environment_info().traffic_light().traffic_light()[i].light_cycle()[1];
            single_traffic_light.red_period = veh_perception_msg_.environment_info().traffic_light().traffic_light()[i].light_cycle()[2];
          }
          //
          dp_ci->perception_data.traffic_lights.push_back(single_traffic_light);
        }
    }
  } 

}
void Planning::CallbackControlBoxDdS(void* data, uintmax_t size) {
  if (size > 0 && data != nullptr) {
    control_feedbach_msg_.ParseFromArray(data, size);
    auto dp_can = &acu::planning::DataPool::Instance()->GetMainDataPtr()->drive_status;
    dp_can->canbus_status.CheckInit(control_feedbach_msg_.header().time_stamp());
    dp_can->timestamp = control_feedbach_msg_.header().time_stamp();
    dp_can->steer_angle = control_feedbach_msg_.vehicle_info().steerangle();
    dp_can->velocity = control_feedbach_msg_.vehicle_info().speed();
    dp_can->shiftlvlposition = control_feedbach_msg_.vehicle_info().shift_position();
    dp_can->epb_status = control_feedbach_msg_.vehicle_info().epb_status();
    dp_can->control_mode = control_feedbach_msg_.vehicle_info().current_drive_mode();
    dp_can->vehicle_accel = control_feedbach_msg_.vehicle_info().vehicle_accel();
    dp_can->brake_state =  control_feedbach_msg_.brake_state();
    dp_can->on_accpedal =  control_feedbach_msg_.vehicle_info().on_accpedal();
    AERROR << "brake_state " << dp_can->brake_state <<" dp_can->on_accpedal: " <<dp_can->on_accpedal;
    dp_can->steer_left_down =  control_feedbach_msg_.vehicle_info().steer_left_down();
    dp_can->SetDriveStatus();
  }

}
void Planning::CallbackMapengineDds(void* data, uintmax_t size){
  //std::lock_guard<std::mutex> lock(mutex_);
  if (size > 0 && data != nullptr) {
      std::lock_guard<std::mutex> lock(acu::planning::DataPool::Instance()->mutex_);
    navigation_msg_.ParseFromArray(data, size);
    auto DP = acu::planning::DataPool::Instance()->GetMainDataPtr();
    DP->mapengine_data.Reset();
    DP->mapengine_data.map_info_data.alllinelists.clear();
    DP->mapengine_data.mapengine_status.CheckInit(navigation_msg_.header().time_statistics().sending_timestamp());
    DP->mapengine_data.time = navigation_msg_.header().time_stamp();
    DP->mapengine_data.map_info_data.maptype = (eMapType)navigation_msg_.map_type();
    DP->cognition_info.SetMapType(DP->mapengine_data.map_info_data.maptype);
    DP->mapengine_data.map_info_data.dis2missionpoint = navigation_msg_.map_info().dis2endpoint(); 
    DP->mapengine_data.loc_data.Set(navigation_msg_.imu_data());
    DP->mapengine_data.last_loc_data.Set(navigation_msg_.last_imu_data());
    FiltLocalizationVelocity(DP->mapengine_data.loc_data.velocity);
    FiltLocalizationVelocity(DP->mapengine_data.last_loc_data.velocity);
    DP->mapengine_data.correction_loc_data.Set(navigation_msg_.correct_imu_data());
    DP->mapengine_data.last_correct_loc_data.Set(navigation_msg_.last_correct_imu_data());
    AERROR << "navigation_msg_ " << navigation_msg_.map_info().alllinelists_size();
    for (int i = 0; i < navigation_msg_.map_info().alllinelists().size(); i++) {
      //AERROR<< " i " <<i;
      MapEngineLineList temp_linelist;
      temp_linelist.SetMsg(navigation_msg_.map_info().alllinelists(i));
      temp_linelist.mission.Set(navigation_msg_.map_info().endpoint());
      temp_linelist.mission.is_park = !DP->config_info.behavior_config.enable_struct_pull_over;
      temp_linelist.dis2missionpoint = navigation_msg_.map_info().dis2endpoint(); 
      DP->mapengine_data.map_info_data.alllinelists.push_back(temp_linelist);
    }

    DP->mapengine_data.last_vision_data.clear();
    DP->mapengine_data.vision_data.clear();
    for (auto &last : navigation_msg_.last_vision_info()) {
      FuncInfo temp_func(last);
      DP->mapengine_data.last_vision_data.push_back(temp_func);
    }
    for (auto &current : navigation_msg_.vision_info()) {
      FuncInfo temp_func(current);
      DP->mapengine_data.vision_data.push_back(temp_func);
    }
    DP->mapengine_data.map_info_data.revlinelist.SetMsg(navigation_msg_.map_info().revlinelist());
    DP->mapengine_data.map_info_data.index = navigation_msg_.map_info().current_line_index();
    if (DP->mapengine_data_list.size() >= map_data_size_) {
      DP->mapengine_data_list.pop_back();
    }
    DP->mapengine_data_list.insert(DP->mapengine_data_list.begin(), DP->mapengine_data);
    for (auto &data : DP->mapengine_data_list) {
      data.map_info_data.task_change = navigation_msg_.map_info().change_task();
    }
    AERROR_IF(navigation_msg_.map_info().change_task())<<"CallbackMapengine change_task.";
  }
}
void Planning::CallbackPredictionDds(void* data, uintmax_t size){
  if (size > 0 && data != nullptr) {
    std::lock_guard<std::mutex> lock(acu::planning::DataPool::Instance()->mutex_);
    prediction_msg_.ParseFromArray(data, size);
    //AERROR <<"------------prediction_obstacle---------size "<< prediction_msg_.prediction_obstacle_size();
    //if (prediction_msg_.prediction_obstacle_size()== 0) return;
    
    auto DP = acu::planning::DataPool::Instance()->GetMainDataPtr();
    callback_prediction_flag_ = true;
    prediction_transfer_time_ = 1000 * (acu::common::NodeTime::Now().ToSecond() - prediction_msg_.header().time_statistics().sending_timestamp());
    DP->prediction_data.Reset();
    DP->prediction_data.prediction_status.CheckInit(prediction_msg_.header().time_statistics().sending_timestamp());
    DP->prediction_data.time = prediction_msg_.header().time_statistics().sending_timestamp();
    DP->prediction_data.perception_time = prediction_msg_.perception_objects().header().time_stamp();

    for (auto &obj : prediction_msg_.prediction_obstacle()) {
      PredictionObject prediction_obj;
      prediction_obj.time_stamp = obj.timestamp();
      prediction_obj.id = obj.id();
      prediction_obj.type = obj.type();
      prediction_obj.priority = obj.priority();
      prediction_obj.predicted_period = obj.predicted_period();
      prediction_obj.is_static = obj.is_static();
      prediction_obj.is_ultra_static = (obj.is_ultra_static() == 1);
      prediction_obj.speed = obj.speed();
      prediction_obj.obj_lane_id = obj.current_lane_id();
      for (auto &trajectory : obj.trajectories()) {
        PredictionTrajectory prediction_trajectory;
        prediction_trajectory.probability = trajectory.probability();
        prediction_trajectory.intentbylane = trajectory.intentbylane();
        for(auto &point : trajectory.points()) {
          PredictionPoint prediction_point;
          prediction_point.x = point.x();
          prediction_point.y = point.y();
          prediction_point.angle = point.angle();
          prediction_point.xg = point.xg();
          prediction_point.yg = point.yg();
          prediction_point.globalangle = point.globalangle();
          prediction_point.v = point.v();
          prediction_point.a = point.a();
          prediction_point.t = point.t();
          prediction_point.lane_id = point.lane_id();
          prediction_trajectory.points.push_back(prediction_point);
        }
        for (auto &lane_id : trajectory.lane_ids()) {
          if (prediction_trajectory.lane_ids.empty()) {
            prediction_trajectory.lane_ids.push_back(lane_id);
          }
          else if (prediction_trajectory.lane_ids.back() != lane_id) {
            prediction_trajectory.lane_ids.push_back(lane_id);
          }
        }
        prediction_obj.trajectories.push_back(prediction_trajectory);
      }
      DP->prediction_data.prediction_objects.push_back(prediction_obj);
    }
  }
}

void Planning::CallbackNavmissionDds(void* data, uintmax_t size){
  if (size > 0 && data != nullptr) {
    navmission_msg_.ParseFromArray(data, size);
    AERROR<<"CallbackNavmissionDds: " << navmission_msg_.header().module_name();
    auto DP = acu::planning::DataPool::Instance()->GetMainDataPtr();
    DP->task_content.mission_type_nav = navmission_msg_.type();
  }

}

void Planning::PublishPlanningMsg() {
  auto DP = acu::planning::DataPool::Instance()->GetMainDataPtr();
  if (DP->task_content.command == eCommand::COMMAND_STOP) {
    if (std::fabs(DP->loc_perception.localization_data.velocity) < 0.1 || 
      DP->drive_status.control_mode == 1) {
      DP->task_content.SetCommandInfo();
    }
  }
  /* ------------------------------ publish tppathplan------------------------------------*/
  planing_msg_.mutable_header()->set_time_stamp(DP->motion_path.time_stamp);
  double current_time = acu::common::NodeTime::Now().ToSecond();
  planing_msg_.mutable_header()->mutable_time_statistics()->set_sending_timestamp(current_time);
  planing_msg_.set_control_accuracy(DP->motion_path.control_accuracy);
  if (DP->motion_path.path.empty()) {
    planing_msg_.set_direction(0); 
  } else {
    planing_msg_.set_direction((DP->motion_path.path.front().reverse == false)? 0 : 1);
  }
  planing_msg_.mutable_lane_ids()->Clear();
  if (!DP->motion_path.lane_ids.empty()) {//@pqg add for prediction
    for (auto& id : DP->motion_path.lane_ids) {
      planing_msg_.add_lane_ids(id);
    }
  }
  planing_msg_.set_lane_type(DP->motion_path.path_type == "UTurn" ? 1 : 0);//为了横向针对掉头路段做特殊处理，这里暂时处理为非掉头就是直道。
  if (DP->task_exe_result == eTaskExeResult::SUCCESS 
      || DP->task_content.command_info == eCommand::COMMAND_STOP
      || DP->task_content.command_info == eCommand::COMMAND_SUSPEND) {
    planing_msg_.set_gear_position_request(1);
  } else {
    planing_msg_.set_gear_position_request(0);
  }
  planing_msg_.set_senario_type(DP->motion_path.senario_type);
  planing_msg_.set_steeringangle_rate_max(DP->motion_path.steeringangle_rate_max);
  planing_msg_.mutable_points()->Clear();
  planning_msgs::TrajectoryPoint temp;
  if (!DP->motion_path.path.empty()) {
    for (auto &p : DP->motion_path.path) {
      temp.set_x(p.x);
      temp.set_y(p.y);
      temp.set_xg(p.xg);
      temp.set_yg(p.yg);
      temp.set_xg_dr(p.dr_x);
      temp.set_yg_dr(p.dr_y);
      temp.set_angle(p.angle);
      temp.set_angleglobal(p.globalangle);
      temp.set_angleglobal_dr(p.dr_angle);
      temp.set_velocity(p.velocity);
      temp.set_a(p.a);
      temp.set_t(p.t);
      temp.set_curvature(p.curvature);
      temp.set_length(p.length);// - DP->motion_path.path.front().length;
      temp.set_direction(planing_msg_.direction());
      planing_msg_.add_points()->CopyFrom(temp);
      //if (p.a< -1.0)
      //AERROR<< "paht x: " << p.x << " y: "<< p.y << " ,t: "<< p.t << " ,v: " << p.velocity;
    }
  }

  AERROR<< "pathplan size "<< planing_msg_.points_size();
  DP->loc_perception.localization_data.SetMsg(*planing_msg_.mutable_imu_loc_data());
  DP->loc_perception.correct_localization.SetMsg(*planing_msg_.mutable_correct_loc_data());
  
  calmcar::dds::SerializedData Data1(planing_msg_.ByteSize());
  planing_msg_.SerializeToArray(Data1.buffer(), Data1.size());
  Acu_ddsPtr->Acu_dds_Publish(t_dds_pub[0].topic_name,std::move(Data1));      //   dds topic pub
  //pub_path_.publish(planing_msg_);
  
}
void Planning::PublishPlanningDebugMsg() {
  /* ----------------------------- publish planning_debug-------------------------------*/
  double plan_pub_time = acu::common::NodeTime::Now().ToSecond();
  planning_debug_msg_.mutable_header()->mutable_time_statistics()->set_sending_timestamp(plan_pub_time);
  //planning_debug_msg_.mutable_header()->set_version("20241205");
  //planning_debug_msg_.mutable_header()->set_module_name("Planning");
  common_msgs::TimeStatus time_status;
  auto DP = acu::planning::DataPool::Instance()->GetMainDataPtr();
  planning_debug_msg_.mutable_main_stream_msg()->set_str(DP->debug_planning_msg.main_stream_msg.str);
  planning_debug_msg_.mutable_main_stream_msg()->set_value(DP->debug_planning_msg.main_stream_msg.value);
  planning_debug_msg_.mutable_task_fsm_condition()->set_str(DP->debug_planning_msg.task_fsm_condition.str);
  planning_debug_msg_.mutable_task_fsm_condition()->set_value(DP->debug_planning_msg.task_fsm_condition.value);
  planning_debug_msg_.mutable_task_fsm_state()->set_str(DP->debug_planning_msg.task_fsm_state.str);
  planning_debug_msg_.mutable_task_fsm_state()->set_value(DP->debug_planning_msg.task_fsm_state.value);
  planning_debug_msg_.mutable_cognition_msg()->set_str(DP->debug_planning_msg.cognition_msg.str);
  planning_debug_msg_.mutable_cognition_msg()->set_value(DP->debug_planning_msg.cognition_msg.value);
  planning_debug_msg_.mutable_business_msg()->set_str(DP->debug_planning_msg.business_msg.str);
  planning_debug_msg_.mutable_business_msg()->set_value(DP->debug_planning_msg.business_msg.value);

  planning_debug_msg_.set_time_cost_ms(DP->debug_planning_msg.time_cost_ms);
  //planning_debug_msg_.mutable_business()->CopyFrom(DP->debug_planning_msg.business_debug);
  planning_debug_msg_.mutable_cognition()->CopyFrom(DP->debug_planning_msg.cognition);
  planning_debug_msg_.mutable_decision()->CopyFrom(DP->debug_planning_msg.decision);
  planning_debug_msg_.mutable_motionplan()->CopyFrom(DP->debug_planning_msg.motionplan_debug);
  planning_debug_msg_.mutable_locpose()->set_time_stamp(DP->cognition_info.struct_env_info.vehicle_info.localization.time_stamp);
  planning_debug_msg_.mutable_locpose()->set_xg(DP->cognition_info.struct_env_info.vehicle_info.localization.xg);
  planning_debug_msg_.mutable_locpose()->set_yg(DP->cognition_info.struct_env_info.vehicle_info.localization.yg);
  planning_debug_msg_.mutable_locpose()->set_yaw(DP->cognition_info.struct_env_info.vehicle_info.localization.global_angle);
  GeoTool geotool;
  PointVCS rel_point;
  PointGCCS ego_point, object_point;
  ego_point.xg = planning_debug_msg_.locpose().xg();
  ego_point.yg = planning_debug_msg_.locpose().yg();
  ego_point.angle = planning_debug_msg_.locpose().yaw();
  planning_debug_msg_.mutable_prediction()->mutable_points()->Clear();
  for (auto& object : DP->prediction_bk_data.prediction_objects) {
    for (auto& prediction : object.trajectories) {
      planning_msgs::Trajectory trajectory;
      planning_msgs::TrajectoryPoint trajectory_point;
      for (auto& point : prediction.points) {
        trajectory_point.set_xg(point.xg);
        trajectory_point.set_yg(point.yg);
        trajectory_point.set_angleglobal(point.globalangle);
        trajectory_point.set_t(point.t);
        object_point.xg = point.xg;
        object_point.yg = point.yg;
        geotool.GCCS2VCS(ego_point, object_point, rel_point);
        trajectory_point.set_x(rel_point.x);
        trajectory_point.set_y(rel_point.y);
        trajectory.add_points()->CopyFrom(trajectory_point);
      }
      planning_debug_msg_.mutable_prediction()->add_points()->CopyFrom(trajectory);
    }
  }

  
  calmcar::dds::SerializedData Data1(planning_debug_msg_.ByteSize());
  planning_debug_msg_.SerializeToArray(Data1.buffer(), Data1.size());
  Acu_ddsPtr->Acu_dds_Publish(t_dds_pub[2].topic_name,std::move(Data1));      //   dds topic pub
  //pub_debug_.publish(planning_debug_msg_);
  //DP->debug_planning_msg.Reset();
  //return 0;

}
void Planning::PublishBusinessMsg() {
    /* ------------------------------ publish tptask_fb------------------------------------*/
  auto DP = acu::planning::DataPool::Instance()->GetMainDataPtr();
  business_msgs_.mutable_header()->set_time_stamp(DP->motion_path.time_stamp);
  business_msgs_.mutable_header()->mutable_time_statistics()->set_sending_timestamp(acu::common::NodeTime::Now().ToSecond());
  business_msgs_.mutable_business()->set_direction((DP->motion_path.path.front().reverse == false)? 0 : 1);
  business_msgs_.mutable_business()->set_turning(DP->turning);
  business_msgs_.mutable_business()->set_impassable_flag(DP->impassable_flag);
  AWARN<< "DP->impassable_flag: "<< DP->impassable_flag  <<" , "<<business_msgs_.business().impassable_flag();
  business_msgs_.mutable_reference_lane_ids()->Clear();
  for(const auto& lane : DP->debug_planning_msg.cognition.reference_lane_ids()){
    business_msgs_.add_reference_lane_ids(lane);
  }
  business_msgs_.mutable_reference_target_ids()->Clear();
  for(const auto& lane : DP->debug_planning_msg.cognition.reference_target_ids()){
    business_msgs_.add_reference_target_ids(lane);
  }
  business_msgs_.set_decision_id(DP->debug_planning_msg.decision.target_line());
  business_msgs_.set_path_in_current(DP->debug_planning_msg.cognition.in_current());
  business_msgs_.set_is_passable((int)DP->debug_planning_msg.decision.is_passable());
  business_msgs_.mutable_passable_lane_ids()->Clear();
  AERROR << "passable_lane_ids: " << (int)business_msgs_.is_passable() << " , DP " 
                                  <<(int)DP->debug_planning_msg.decision.is_passable(); 
  for(const auto& lane : DP->debug_planning_msg.decision.passable_lane_ids()){
    business_msgs_.add_passable_lane_ids(lane);
    AWARN<< " , "<< lane;
  }
  business_msgs_.mutable_black_road_ids()->Clear();
  for(const auto& lane : DP->debug_planning_msg.decision.black_road_ids()){
    business_msgs_.add_black_road_ids(lane);
  }

  //pub_business_.publish(pub_business_msg_);
  calmcar::dds::SerializedData Data1(business_msgs_.ByteSize());
  business_msgs_.SerializeToArray(Data1.buffer(), Data1.size());
  Acu_ddsPtr->Acu_dds_Publish(t_dds_pub[1].topic_name,std::move(Data1));      //   dds topic pub

  DP->debug_planning_msg.Reset();

}
void Planning::FiltLocalizationVelocity(double& velocity) {
  if (DataPool::Instance()->GetMainDataPtr()->drive_status.shiftlvlposition == 1) {
    double temp_v = -velocity;
    if(temp_v < 0.0) temp_v = 0.0;
    velocity = temp_v;
  } else if (DataPool::Instance()->GetMainDataPtr()->drive_status.shiftlvlposition == 3) {
    double temp_v = velocity;
    if(temp_v < 0.0) temp_v = 0.0;
    velocity = temp_v;
  } else {
    // shift_position == 0 for parking or shift_position == 2 for neutral gear, keep origin velocity.
    // velocity = velocity;
  }
}

void Planning::CallBackZmqRepTaskCmd (acu::zmq::CCommonTopicMsg *data,acu::zmq::CCommonTopicMsg *rep){
  planning_msgs::PlanningCmd input;
  planning_msgs::PlanningResult output;
  input.ParseFromArray(data->message_content, data->topic_header.message_size);
  
  auto DP = acu::planning::DataPool::Instance()->GetMainDataPtr();
  DP->task_content.mission_id = input.mission_id();
  DP->task_content.mission_type = (eMissionType)(input.mission_type()); // 可能有问题，注意对应顺序
  DP->task_content.command = (eCommand)(input.mission_command());  // 可能有问题，注意对应顺序
  output.set_result(0);
  output.set_addition_message("CallbackTaskCmd doing.");
  DP->task_content.SetTaskStatus();
  if (DP->task_content.command == eCommand::COMMAND_STOP) {
    if (std::fabs(DP->loc_perception.localization_data.velocity) < 0.1) {
      DP->task_content.SetCommandInfo();
    }
  } else {
    DP->task_content.SetCommandInfo();
  }
  int byte_size = output.ByteSize();
  rep->topic_header.message_size = byte_size;
  rep->topic_header.seq = data->topic_header.seq;
  memcpy(rep->topic_header.message_type,data->topic_header.message_type,sizeof(data->topic_header.message_type));
  output.SerializeToArray(rep->message_content, byte_size);
  // char* tmp_buffer = new char[byte_size];
  // output.SerializeToArray(tmp_buffer, byte_size);
  // for (size_t i = 0; i <  byte_size; i++) {
  //   rep->message_content[i] = tmp_buffer[i];
  // }
  // delete[] tmp_buffer;
  AINFO<<"CallbackTaskCmd id "<<input.mission_id()<<" mission_type "
       <<(int)input.mission_type()<<" mission_command "<<(int)DP->task_content.command;
 // return ;
}

void Planning::CallBackZmqRepBehaviorLimit(acu::zmq::CCommonTopicMsg *data,acu::zmq::CCommonTopicMsg *rep){

  planning_msgs::BehaviorLimitCmd input;
  planning_msgs::PlanningResult output;//PlanningResult;
  input.ParseFromArray(data->message_content, data->topic_header.message_size);

  auto DP = acu::planning::DataPool::Instance()->GetMainDataPtr();
  DP->task_content.speed_limit = input.speed_limit();
  if (input.behaviorlimit_size() > 3) {
    output.set_result(-1);
    output.set_addition_message("size of behaviorlimit input is not 3.");
    return;//return -1;
  }
  if (input.behaviorlimit().size() >= 2) {
    DP->config_info.behavior_config.enable_struct_obs_avoid = (input.behaviorlimit()[0] != 1);
    DP->config_info.behavior_config.enable_struct_free_lanechange = (input.behaviorlimit()[1] != 1);
    if (input.behaviorlimit().size() == 3) {
      DP->config_info.behavior_config.enable_struct_pull_over = (input.behaviorlimit()[2] != 1);
    }
  }
  
  if (input.behaviorexecutethreshold().size() > 3) {
    output.set_result(-1);
    output.set_addition_message("size of behaviorexecutethreshold input is not 3.");
    return ;//return -1;
  }
  if (input.behaviorexecutethreshold().size() == 3) {
    DP->config_info.behavior_config.following_radicalness = input.behaviorexecutethreshold()[0];
    DP->config_info.behavior_config.avoid_radicalness = input.behaviorexecutethreshold()[1];
    DP->config_info.behavior_config.lanechange_radicalness = input.behaviorexecutethreshold()[2];
  }
  output.set_result(0);
  output.set_addition_message("BehaviorLimit success.");

  // 平行驾驶
  DP->task_content.brake_command = input.brake_cmd();
  DP->task_content.direction_command = input.lane_change_cmd();  
  AINFO << "CallbackBehaviorLimit brake_command " << (int)input.brake_cmd()
        << " direction_command " << (int)input.lane_change_cmd(); 
  int byte_size = output.ByteSize();
  rep->topic_header.message_size = byte_size;
  rep->topic_header.seq = data->topic_header.seq;
  memcpy(rep->topic_header.message_type,data->topic_header.message_type,sizeof(data->topic_header.message_type));
  output.SerializeToArray(rep->message_content, byte_size);
  // char* tmp_buffer = new char[byte_size];
  // output.SerializeToArray(tmp_buffer, byte_size);
  // for (size_t i = 0; i <  byte_size; i++) {
  //   rep->message_content[i] = tmp_buffer[i];
  // }
  // delete[] tmp_buffer;


  //return 0;

}
void Planning::CallBackZmqRepInquireStatus (acu::zmq::CCommonTopicMsg *data,acu::zmq::CCommonTopicMsg *rep) {

  planning_msgs::PlanningCmd input;
  planning_msgs::PlanningStatus output;
  input.ParseFromArray(data->message_content, data->topic_header.message_size);

  auto DP = acu::planning::DataPool::Instance()->GetMainDataPtr();
  if (DP->task_exe_result == eTaskExeResult::SUCCESS) {
    output.set_status(3);
    DP->task_content.SetTaskStatus();
    DP->task_content.SetCommandInfo();
    AWARN<<"CallbackInquireStatus task success, reset.";
  } else if (DP->task_exe_result == eTaskExeResult::FAILURE) {
    output.set_status(2);
  } else {
    if (DP->task_content.command_info == eCommand::COMMAND_STOP) {
      output.set_status(0);
    } else if (DP->task_content.command_info == eCommand::COMMAND_SUSPEND) {
        output.set_status(4);
    } else {
      if (DP->drive_status.control_mode == 1) {// 非自动状态下
        if (DP->ready_state == 0 && 
            (DP->debug_planning_msg.motionplan_debug.is_new_path() && 
            DP->debug_planning_msg.motionplan_debug.path_data_size() > 1 &&
            DP->motion_path.path.size() > 2 || DP->drive_status.velocity < 0.2)) {
          output.set_status(1);
        } else {
          output.set_status(5);
        }
      } else {
        output.set_status(1);
      }
    }
  } 
  output.set_mission_id(DP->task_content.mission_id);
  output.set_mission_type((int)DP->task_content.mission_type);
  output.set_speed_limit(DP->task_content.speed_limit);
  output.set_brake_status(DP->task_content.brake_command);
  output.set_lane_change_status(DP->task_content.direction_command);
  output.mutable_lane_change_feedback()->set_result(DP->debug_planning_msg.decision.lc_status());
  output.mutable_lane_change_feedback()->set_addition_message(DP->debug_planning_msg.decision.cmd_feedback());
  AINFO<<"CallbackInquireStatus status "<<output.status();
  output.mutable_behaviorlimit()->Clear();
  int order = (DP->config_info.behavior_config.enable_struct_obs_avoid)? 2 : 1;
  output.add_behaviorlimit(order);
  order = (DP->config_info.behavior_config.enable_struct_free_lanechange)? 2 : 1;
  output.add_behaviorlimit(order);
  output.mutable_behaviorexecutethreshold()->Clear();
  output.add_behaviorexecutethreshold(DP->config_info.behavior_config.following_radicalness);
  output.add_behaviorexecutethreshold(DP->config_info.behavior_config.avoid_radicalness);
  output.add_behaviorexecutethreshold(DP->config_info.behavior_config.lanechange_radicalness);

  int byte_size = output.ByteSize();
  rep->topic_header.message_size = byte_size;
  rep->topic_header.seq = data->topic_header.seq;
  memcpy(rep->topic_header.message_type,data->topic_header.message_type,sizeof(data->topic_header.message_type));
  output.SerializeToArray(rep->message_content, byte_size);
  // char* tmp_buffer = new char[byte_size];
  // output.SerializeToArray(tmp_buffer, byte_size);
  // for (size_t i = 0; i <  byte_size; i++) {
  //   rep->message_content[i] = tmp_buffer[i];
  // }
  // delete[] tmp_buffer;

}

void Planning::CallBackTaskCmdDDS (void* data, uintmax_t size){

  AINFO <<"------------------------CallBackTaskCmdDDS----------------------------";
  if (size < 1 || data == nullptr) return; 
  planning_msgs::PlanningCmd input;
  planning_msgs::PlanningResult output;
  input.ParseFromArray(data, size);
  auto DP = acu::planning::DataPool::Instance()->GetMainDataPtr();
  DP->task_content.mission_id = input.mission_id();
  DP->task_content.mission_type = (eMissionType)(input.mission_type()); // 可能有问题，注意对应顺序
  DP->task_content.command = (eCommand)(input.mission_command());  // 可能有问题，注意对应顺序
  output.set_result(0);
  output.set_addition_message("CallbackTaskCmd doing.");
  DP->task_content.SetTaskStatus();
  if (DP->task_content.command == eCommand::COMMAND_STOP) {
    if (std::fabs(DP->loc_perception.localization_data.velocity) < 0.1) {
      DP->task_content.SetCommandInfo();
    }
  } else {
    DP->task_content.SetCommandInfo();
  }
  calmcar::dds::SerializedData resutl_data(output.ByteSize());
  output.SerializeToArray(resutl_data.buffer(), resutl_data.size());
  Acu_ddsPtr->Acu_dds_Publish("PlanningResult",std::move(resutl_data));      //   dds topic pub
  AINFO<<"CallbackTaskCmd id "<<input.mission_id()<<" mission_type "
       <<(int)input.mission_type()<<" mission_command "<<(int)DP->task_content.command;
 // return ;

}

void Planning::CallBackInquireStatusDDS (void* data, uintmax_t size) {
  AINFO <<"------------------------CallBackInquireStatusDDS----------------------------";
  if (size < 1 || data == nullptr) return;
  planning_msgs::PlanningCmd input;
  planning_msgs::PlanningStatus output;
  input.ParseFromArray(data, size);
  auto DP = acu::planning::DataPool::Instance()->GetMainDataPtr();
  if (DP->task_exe_result == eTaskExeResult::SUCCESS) {
    output.set_status(3);
    DP->task_content.SetTaskStatus();
    DP->task_content.SetCommandInfo();
    AWARN<<"CallbackInquireStatus task success, reset.";
  } else if (DP->task_exe_result == eTaskExeResult::FAILURE) {
    output.set_status(2);
  } else {
    if (DP->task_content.command_info == eCommand::COMMAND_STOP) {
      output.set_status(0);
    } else if (DP->task_content.command_info == eCommand::COMMAND_SUSPEND) {
        output.set_status(4);
    } else {
      if (DP->drive_status.control_mode == 1) {// 非自动状态下
        if (DP->ready_state == 0 && 
            (DP->debug_planning_msg.motionplan_debug.is_new_path() && 
            DP->debug_planning_msg.motionplan_debug.path_data_size() > 1 &&
            DP->motion_path.path.size() > 2 || DP->drive_status.velocity < 0.2)) {
          output.set_status(1);
        } else {
          output.set_status(5);
        }
      } else {
        output.set_status(1);
      }
    }
  } 
  output.set_mission_id(DP->task_content.mission_id);
  output.set_mission_type((int)DP->task_content.mission_type);
  output.set_speed_limit(DP->task_content.speed_limit);
  output.set_brake_status(DP->task_content.brake_command);
  output.set_lane_change_status(DP->task_content.direction_command);
  output.mutable_lane_change_feedback()->set_result(DP->debug_planning_msg.decision.lc_status());
  output.mutable_lane_change_feedback()->set_addition_message(DP->debug_planning_msg.decision.cmd_feedback());
  AINFO<<"CallbackInquireStatus status "<<output.status();
  output.mutable_behaviorlimit()->Clear();
  int order = (DP->config_info.behavior_config.enable_struct_obs_avoid)? 2 : 1;
  output.add_behaviorlimit(order);
  order = (DP->config_info.behavior_config.enable_struct_free_lanechange)? 2 : 1;
  output.add_behaviorlimit(order);
  output.mutable_behaviorexecutethreshold()->Clear();
  output.add_behaviorexecutethreshold(DP->config_info.behavior_config.following_radicalness);
  output.add_behaviorexecutethreshold(DP->config_info.behavior_config.avoid_radicalness);
  output.add_behaviorexecutethreshold(DP->config_info.behavior_config.lanechange_radicalness);
  calmcar::dds::SerializedData resutl_data(output.ByteSize());
  output.SerializeToArray(resutl_data.buffer(), resutl_data.size());
  Acu_ddsPtr->Acu_dds_Publish("PlanningStatus",std::move(resutl_data));  

}

} // namespace planning
} // namespace acu
