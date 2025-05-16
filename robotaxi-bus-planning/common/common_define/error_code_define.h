#ifndef __ERROR_CODE_DEFINE_H__
#define __ERROR_CODE_DEFINE_H__
// clang-format off

/*--------------------------Driver module error codes start from here-----------------------------------------------*/
#define DFPERCEPTION_LIDAR_HARDWARE_0_ERROR          0x0000           // top lidar failed to sync
#define DFPERCEPTION_LIDAR_HARDWARE_1_ERROR          0x0001           // front lidar failed to sync
#define DFPERCEPTION_LIDAR_HARDWARE_2_ERROR          0x0002           // left lidar failed to sync
#define DFPERCEPTION_LIDAR_HARDWARE_3_ERROR          0x0003           // right lidar failed to sync
#define DFPERCEPTION_LIDAR_HARDWARE_4_ERROR          0x0004           // rear lidar failed to sync
#define DFPERCEPTION_LIDAR_HARDWARE_5_ERROR          0x0005           // top lidar time is not the same as system time
#define DFPERCEPTION_LIDAR_HARDWARE_6_ERROR          0x0006           // front lidar time is not the same as system time
#define DFPERCEPTION_LIDAR_HARDWARE_7_ERROR          0x0007           // left lidar time is not the same as system time
#define DFPERCEPTION_LIDAR_HARDWARE_8_ERROR          0x0008           // right lidar time is not the same as system time
#define DFPERCEPTION_LIDAR_HARDWARE_9_ERROR          0x0009           // rear lidar time is not the same as system time

#define DFPCICAN_HARDWARE_0_ERROR                    0x0200           // read mcu data failed!
#define DFPCICAN_HARDWARE_1_ERROR                    0x0201           // can busoff!

#define DFPERCEPTION_VISION_HARDWARE_4_ERROR         0x0300           // No.0 surround view camera init fail!
#define DFPERCEPTION_VISION_HARDWARE_5_ERROR         0x0301           // No.0 surround view camera no data or out of time limit!
#define DFPERCEPTION_VISION_HARDWARE_6_ERROR         0x0302           // No.1 surround view camera init fail!
#define DFPERCEPTION_VISION_HARDWARE_7_ERROR         0x0303           // No.1 surround view camera no data or out of time limit!
#define DFPERCEPTION_VISION_HARDWARE_8_ERROR         0x0304           // No.2 surround view camera init fail!
#define DFPERCEPTION_VISION_HARDWARE_9_ERROR         0x0305           // No.2 surround view camera no data or out of time limit!
#define DFPERCEPTION_VISION_HARDWARE_10_ERROR        0x0306           // No.3 surround view camera init fail!
#define DFPERCEPTION_VISION_HARDWARE_11_ERROR        0x0307           // No.3 surround view camera no data or out of time limit!

#define DFPERCEPTION_ULTRASONIC_HARDWARE_0_ERROR     0x0400           // ultrasonicraw(id 0) no data!
#define DFPERCEPTION_ULTRASONIC_HARDWARE_1_ERROR     0x0401           // ultrasonicraw(id 1) no data!
#define DFPERCEPTION_ULTRASONIC_HARDWARE_2_ERROR     0x0402           // ultrasonicraw(id 2) no data!
#define DFPERCEPTION_ULTRASONIC_HARDWARE_3_ERROR     0x0403           // ultrasonicraw(id 3) no data!
#define DFPERCEPTION_ULTRASONIC_HARDWARE_4_ERROR     0x0404           // ultrasonicraw(id 4) no data!
#define DFPERCEPTION_ULTRASONIC_HARDWARE_5_ERROR     0x0405           // ultrasonicraw(id 5) no data!
#define DFPERCEPTION_ULTRASONIC_HARDWARE_6_ERROR     0x0406           // ultrasonicraw(id 6) no data!
#define DFPERCEPTION_ULTRASONIC_HARDWARE_7_ERROR     0x0407           // ultrasonicraw(id 7) no data!
#define DFPERCEPTION_ULTRASONIC_HARDWARE_8_ERROR     0x0408           // ultrasonicraw(id 8) no data!
#define DFPERCEPTION_ULTRASONIC_HARDWARE_9_ERROR     0x0409           // ultrasonicraw(id 9) no data!
#define DFPERCEPTION_ULTRASONIC_HARDWARE_10_ERROR    0x040a           // ultrasonicraw(id 10) no data!
#define DFPERCEPTION_ULTRASONIC_HARDWARE_11_ERROR    0x040b           // ultrasonicraw(id 11) no data!

//--------------------------------------------------------------------------------------------------------------------


/*--------------------------Perception module error codes start from here-------------------------------------------*/
// 2022-04-27 update
#define DFPERCEPTION_LIDAR_DATA_TIMEOUT_0_ERROR		  0x1000           // top lidar data timeout 
#define DFPERCEPTION_LIDAR_DATA_TIMEOUT_1_ERROR		  0x1001           // front lidar data timeout 
#define DFPERCEPTION_LIDAR_DATA_TIMEOUT_2_ERROR		  0x1002           // left lidar data timeout 
#define DFPERCEPTION_LIDAR_DATA_TIMEOUT_3_ERROR		  0x1003           // right lidar data timeout 
#define DFPERCEPTION_LIDAR_DATA_TIMEOUT_4_ERROR		  0x1004           // rear lidar data timeout 
#define DFPERCEPTION_LIDAR_DATA_DEFICIENCY_0_ERROR  0x1005           // top lidar ring dificiency!
#define DFPERCEPTION_LIDAR_DATA_DEFICIENCY_1_ERROR  0x1006           // front lidar ring dificiency!
#define DFPERCEPTION_LIDAR_DATA_DEFICIENCY_2_ERROR  0x1007           // left lidar ring dificiency!
#define DFPERCEPTION_LIDAR_DATA_DEFICIENCY_3_ERROR  0x1008           // right lidar ring dificiency!
#define DFPERCEPTION_LIDAR_DATA_DEFICIENCY_4_ERROR  0x1009           // back lidar ring dificiency!
#define DFPERCEPTION_CAMERA_DATA_TIMEOUT_0_ERROR	  0x1100           // front_h60 data timeout 
#define DFPERCEPTION_CAMERA_DATA_TIMEOUT_1_ERROR	  0x1101           // tpvisionobjects data timeout 
#define DFPERCEPTION_CAMERA_DATA_TIMEOUT_2_ERROR	  0x1102           // tpvisionsegments data timeout 
#define DFPERCEPTION_CAMERA_DATA_TIMEOUT_3_ERROR	  0x1103           // tpvisionlandet data timeout 
#define DFPERCEPTION_CAMERA_DATA_TIMEOUT_4_ERROR	  0x1104           // tptrafficlight data timeout 
#define DFPERCEPTION_RADAR_DATA_TIMEOUT_0_ERROR		  0x1200           // radar 0 data timeout 
#define DFPERCEPTION_RADAR_DATA_TIMEOUT_1_ERROR		  0x1201           // radar 1 data timeout 
#define DFPERCEPTION_ULTRA_DATA_TIMEOUT_0_ERROR		  0x1300           // ultrasonic 0 data timeout (reserved)
#define DFPERCEPTION_ULTRA_DATA_TIMEOUT_1_ERROR		  0x1301           // ultrasonic 1 data timeout (reserved)
#define DFPERCEPTION_ULTRA_DATA_TIMEOUT_2_ERROR		  0x1302           // ultrasonic 2 data timeout (reserved)
#define DFPERCEPTION_ULTRA_DATA_TIMEOUT_3_ERROR		  0x1303           // ultrasonic 3 data timeout (reserved)

#define DFPERCEPTION_LIDAR_PROCESS_0_ERROR			0x1010           // lidar sofeware fault: failed to load config file
#define DFPERCEPTION_LIDAR_PROCESS_1_ERROR			0x1011           // lidar sofeware fault: lidar and camera out of sync
#define DFPERCEPTION_LIDAR_PROCESS_2_ERROR			0x1012           // lidar sofeware fault: lidar-camera fusion func skipped  
#define DFPERCEPTION_CAMERA_PROCESS_0_ERROR			0x1110           // camera sofeware fault: failed to load config file
#define DFPERCEPTION_CAMERA_PROCESS_1_ERROR			0x1111           // camera sofeware fault: failed to load model file
#define DFPERCEPTION_CAMERA_PROCESS_2_ERROR			0x1112           // camera sofeware fault: failed to load calibration file
#define DFPERCEPTION_CAMERA_PROCESS_3_ERROR			0x1113           // camera sofeware fault: config camera not equal to the camera used

#define DFPERCEPTION_RADAR_PROCESS_0_ERROR			0x1210           // radar sofeware fault (reserved)
#define DFPERCEPTION_ULTRA_PROCESS_0_ERROR			0x1310           // ultrasonic sofeware fault (reserved)
#define DFPERCEPTION_FUSION_PROCESS_0_ERROR			0x1410           // fusion sofeware fault (reserved)
#define DFPERCEPTION_MAIN_PROCESS_0_ERROR			  0x1510           // main_thread sofeware fault: localization data timeout
#define DFPERCEPTION_MAIN_PROCESS_1_ERROR			  0x1511           // main_thread sofeware fault: ego is out of basemap
#define DFPERCEPTION_MAIN_PROCESS_2_ERROR			  0x1512           // main_thread sofeware fault: lidar process thread timeout
#define DFPERCEPTION_MAIN_PROCESS_3_ERROR			  0x1513           // main_thread sofeware fault: camera process thread timeout (reserved)
#define DFPERCEPTION_MAIN_PROCESS_4_ERROR			  0x1514           // main_thread sofeware fault: radar process thread timeout (reserved)
#define DFPERCEPTION_MAIN_PROCESS_5_ERROR			  0x1515           // main_thread sofeware fault: fusion process thread timeout (reserved)
#define DFPERCEPTION_PERCEPTION_SYS_INIT_ERROR	0x8002			 // perception initialization failed
//--------------------------------------------------------------------------------------------------------------------

/*--------------------------Localization module error codes start from here------------------------------------------*/
#define DFLOCALIZATION_IMU_HARDWARE_0_ERROR          0x2000           // BBox open nabbox serial port fail！
#define DFLOCALIZATION_IMU_HARDWARE_1_ERROR          0x2001           // BBox read nabox seoial fail!
#define DFLOCALIZATION_IMU_HARDWARE_2_ERROR          0x2002           // The time bewtween navbox and bbox is abnormal
#define DFLOCALIZATION_IMU_SYS_INIT_ERROR			       0x8000			  // imu initialization failed
#define DFLOCALIZATION_NAVBOX_HARDWARE_3_ERROR       0x2003           // IMU communication is error!
#define DFLOCALIZATION_NAVBOX_HARDWARE_4_ERROR       0x2004           // Imu gyro is abnormal!
#define DFLOCALIZATION_NAVBOX_HARDWARE_5_ERROR       0x2005           // Imu acc is abnormal!
#define DFLOCALIZATION_NAVBOX_HARDWARE_6_ERROR       0x2006           // Imu temperature is abnormal!
#define DFLOCALIZATION_NAVBOX_HARDWARE_7_ERROR       0x2007           // GPS communication is error!
#define DFLOCALIZATION_NAVBOX_HARDWARE_8_ERROR       0x2008           // GPS data is abnormal!
#define DFLOCALIZATION_NAVBOX_HARDWARE_9_ERROR       0x2009           // Antenna of GPS A is not connected!
#define DFLOCALIZATION_NAVBOX_HARDWARE_A_ERROR       0x200a           // Antenna of GPS B is not connected!
#define DFLOCALIZATION_NAVBOX_HARDWARE_B_ERROR       0x200b           // Odom communication is error!
#define DFLOCALIZATION_NAVBOX_HARDWARE_C_ERROR       0x200c           // Odom left wheel speed is abnormal
#define DFLOCALIZATION_NAVBOX_HARDWARE_D_ERROR       0x200d           // Odom right wheel speed is abnormal
#define DFLOCALIZATION_NAVBOX_HARDWARE_E_ERROR       0x200e           // Odom gear is abnormal
#define DFLOCALIZATION_NAVBOX_HARDWARE_F_ERROR       0x200f           // Lidar communication is error!
#define DFLOCALIZATION_NAVBOX_HARDWARE_G_ERROR       0x20a0           // Lidar data is abnormal
#define DFLOCALIZATION_NAVBOX_HARDWARE_H_ERROR       0x20a1           // 4G module is abnormal

#define DFLOCALIZATION_LOCPOSE_0_ERROR               0x2010           // Locaization is fail!
#define DFLOCALIZATION_LIDAR_SOFTWARE_0_ERROR        0x2011           //Input cloud is empty or invalid
#define DFLOCALIZATION_LIDAR_SOFTWARE_1_ERROR        0x2012           //Lidar matching keeps initial
#define DFLOCALIZATION_LIDAR_SOFTWARE_2_ERROR        0x2013           //Lidar matching params can not be read 
#define DFLOCALIZATION_LIDAR_SOFTWARE_3_ERROR        0x2014           //Map route is not existing
#define DFLOCALIZATION_LIDAR_SOFTWARE_4_ERROR        0x2015           //Map is received but contents absent
#define DFLOCALIZATION_LIDAR_SOFTWARE_5_ERROR        0x2016           //The params for map loading is wrong
#define DFLOCALIZATION_LIDAR_SOFTWARE_6_ERROR        0x2017           //GMM high resolution map is error
#define DFLOCALIZATION_LIDAR_SYS_INIT_ERROR		       0x8001			  //Lidarmatching initialization failed
#define DFLOCALIZATION_LOCPOSE_7_ERROR               0x2018           //Navbox assemble params check fail
#define DFLOCALIZATION_LOCPOSE_8_ERROR               0x2019           //Navbox baseline calibration params check fail
#define DFLOCALIZATION_LOCPOSE_9_ERROR               0x201a           //Navbox odom calibration params check fail
#define DFLOCALIZATION_NAVBOX_SOFTWARE_A_ERROR       0x201b                 // navbox not getting lidar data!
#define DFLOCALIZATION_NAVBOX_SOFTWARE_B_ERROR       0x201c           //Imu acc initial error is large
#define DFLOCALIZATION_NAVBOX_SOFTWARE_C_ERROR       0x201d           //GPS differential age over time
#define DFLOCALIZATION_NAVBOX_SOFTWARE_D_ERROR       0x201e           //GPS data is over time
#define DFLOCALIZATION_NAVBOX_SOFTWARE_E_ERROR       0x201f           //Lidar data is over time
#define DFLOCALIZATION_NAVBOX_SOFTWARE_F_ERROR       0x20b0           //Odom is over time 
#define DFLOCALIZATION_NAVBOX_SOFTWARE_G_ERROR       0x20b1           //Fusion error is large and dr time is over time 
#define DFLOCALIZATION_NAVBOX_SOFTWARE_H_ERROR       0x20b2           //GPS heading is abnormal
#define DFLOCALIZATION_NAVBOX_SOFTWARE_I_ERROR       0x20b3           //location is abnormal
#define DFLOCALIZATION_FUSION_SYS_INIT_ERROR		     0x8009			      // fusion initialization failed
#define DFLOCALIZATION_LOCPOSE_A_ERROR               0x20b4           // lidar matching is not ready and gps is not avilable
#define DFLOCALIZATION_LIDAR_SOFTWARE_WARNING        0x20b5           // Localization is not stable but not error yet. level 2
#define DFLOCALIZATION_NAVBOX_SOFTWARE_WARNING       0x20b6           // fusion localization has error,but not large. level 2
//--------------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------------------------

/*--------------------------Missionplan module error codes start from here------------------------------------------*/
// #define DFPLANNING_MISSIONPLAN_SOFTWARE_0_ERROR      0x2000           // routeplan no data or out of time limit!
// #define DFPLANNING_MISSIONPLAN_SOFTWARE_1_ERROR      0x2001           // statuspredict out of range!
#define DFPLANNING_PLANNING_SOFTWARE_0_ERROR         0x3010           // mapengine data delay
#define DFPLANNING_PLANNING_SOFTWARE_1_ERROR         0x3011           // mapengine data cut out
#define DFPLANNING_PLANNING_SOFTWARE_2_ERROR         0x3012           // perception data delay           
#define DFPLANNING_PLANNING_SOFTWARE_3_ERROR         0x3013           // perception data cut out
#define DFPLANNING_PLANNING_SOFTWARE_4_ERROR         0x3014           // imu data delay
#define DFPLANNING_PLANNING_SOFTWARE_5_ERROR         0x3015           // imu data cut out
#define DFPLANNING_PLANNING_SOFTWARE_6_ERROR         0x3016           // canbus data delay
#define DFPLANNING_PLANNING_SOFTWARE_7_ERROR         0x3017           // canbus data cut out
#define DFPLANNING_PLANNING_SOFTWARE_8_ERROR         0x3018           // prediction data delay
#define DFPLANNING_PLANNING_SOFTWARE_9_ERROR         0x3019           // prediction data cut out
#define DFPLANNING_PLANNING_SOFTWARE_10_ERROR        0x301A           // map api wrong 
#define DFPLANNING_PLANNING_SYS_INIT_ERROR			     0x8005			  // planning initialization failed
#define DFPLANNING_PREDICTION_SYS_INIT_ERROR		     0x8004			  // prediction initialization failed
// #define DFPLANNING_PLANNING_SOFTWARE_11_ERROR        0x301B           // imu data cut out
//--------------------------------------------------------------------------------------------------------------------



/*--------------------------mapengine module error codes start from here-------------------------------------------*/
#define DFMAPENGINE_SOFTWARE_0_ERROR				 0x4010			  // imu data delay
#define DFMAPENGINE_SOFTWARE_1_ERROR         0x4011			  // imu data cut out
#define DFMAPENGINE_SOFTWARE_2_ERROR				 0x4012			  // lane routeplan failed
#define DFMAPENGINE_SOFTWARE_10_ERROR        0x401A			  // map api wrong 
#define DFMAPENGINE_SYS_INIT_ERROR			 		 0x8003			  // mapengine initialization failed
//--------------------------------------------------------------------------------------------------------------------


/*--------------------------Control module error codes start from here-------------------------------------------*/
#define DFCONTROL_HARDWARE_0_ERROR                   0x5000           // cannot shiftposition fault
#define DFCONTROL_HARDWARE_1_ERROR                   0x5001           // spi data delay
#define DFCONTROL_HARDWARE_2_ERROR                   0x5002           //eps feedback error
#define DFCONTROL_HARDWARE_3_ERROR                   0x5003
#define DFCONTROL_HARDWARE_4_ERROR                   0x5004
#define DFCONTROL_HARDWARE_5_ERROR                   0x5005

#define DFCONTROL_SOFTWARE_0_ERROR                   0x5010           // receive planning topic delay
#define DFCONTROL_SOFTWARE_1_ERROR                   0x5011           // receive localization topic delay
#define DFCONTROL_SOFTWARE_2_ERROR                   0x5012           // receive perception topic delay
#define DFCONTROL_SOFTWARE_3_ERROR                   0x5013           // receive canbus topic delay
#define DFCONTROL_SOFTWARE_4_ERROR                   0x5014           // lat error big
#define DFCONTROL_SYS_INIT_ERROR					           0x8006			  // control initialization failed

//--------------------------------------------------------------------------------------------------------------------


#define DFVEHICLE_EPS_0_ERROR     0x7000
#define DFVEHICLE_EPS_1_ERROR     0x7001

#define DFVEHICLE_BRAKE_0_ERROR   0x7010
#define DFVEHICLE_BRAKE_1_ERROR   0x7011

#define DFVEHICLE_DRIVE_0_ERROR   0x7020

#define DFVEHICLE_EPB_0_ERROR     0x7030
#define DFVEHICLE_EPB_1_ERROR     0x7031

#define DFVEHICLE_POWER_0_ERROR   0x7040
#define DFVEHICLE_POWER_1_ERROR   0x7041
#define DFVEHICLE_POWER_2_ERROR   0x7042
#define DFVEHICLE_POWER_3_ERROR   0x7043

#define DFVEHICLE_CPU_0_ERROR     0x7050
#define DFVEHICLE_VCU_1_ERROR     0x7051


/*--------------------------Businee module error codes start from here----------------------------------------------*/
#define DFMONITOR_APP_CPU_HGIH_ERROR                 0x6000              // cpu of nodes high!
#define DFMONITOR_CLOUDSERVICE_CPU_HGIH_ERROR        0x6001
#define DFMONITOR_OBJECTFUSION_CPU_HGIH_ERROR        0x6002
#define DFMONITOR_LOCFUSION_CPU_HGIH_ERROR           0x6003
// clang-format off
//--------------------------------------------------------------------------------------------------------------------

#endif
