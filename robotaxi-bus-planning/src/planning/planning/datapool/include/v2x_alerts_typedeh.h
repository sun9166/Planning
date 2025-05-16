#ifndef DATAPOOL_INCLUDE_V2X_ALERTS_TYPEDEF_H_
#define DATAPOOL_INCLUDE_V2X_ALERTS_TYPEDEF_H_
namespace acu {
namespace planning {

struct ForwardCollisionAlert {
    bool is_alert_exist;
    int  forward_risk_vehicle_id;
    std::string vin;
    double dis_to_forward_vehicle;

    ForwardCollisionAlert(){
        is_alert_exist = false;
        forward_risk_vehicle_id = -1;
        dis_to_forward_vehicle = -1000.0;
    }
};

struct IntersectionCollisionAlert {
    bool is_alert_exist;
    int  intersection_risk_vehicle_id;
    std::string vin;
    IntersectionCollisionAlert(){
        is_alert_exist = false;
        intersection_risk_vehicle_id = -1;
    }
};

struct EmergencyBrakeAlert {
    bool is_alert_exist;
    int  emergency_brake_vehicle_id;
    std::string vin;
    EmergencyBrakeAlert(){
        is_alert_exist = false;
        emergency_brake_vehicle_id = -1;
    }
};

struct AbnormalVehicleAlert {
    bool is_alert_exist;
    int  abnormal_vehicle_id;
    std::string vin;
    AbnormalVehicleAlert(){
        is_alert_exist = false;
        abnormal_vehicle_id = -1;
    }
};

struct RunRedLightWarning {
    bool is_alert_exist;

    RunRedLightWarning(){
        is_alert_exist = false;
    }
};

struct GreenWaveSpeedAlert {
    bool is_alert_exist;
    double dis_to_light;
    double lower_speed_limit;
    double higher_speed_limit;

    GreenWaveSpeedAlert() {
        is_alert_exist = false;
        dis_to_light = -1000.0;
        lower_speed_limit = -1000.0;
        higher_speed_limit = -1000.0;
    }
};

typedef struct v2xAlertsData {
    ForwardCollisionAlert forward_collision_alert;
    IntersectionCollisionAlert intersection_collision_alert;
    EmergencyBrakeAlert emergency_brake_alert;
    AbnormalVehicleAlert abnormal_vehicle_alert;
    RunRedLightWarning run_redLight_warning;
    GreenWaveSpeedAlert green_wave_speed_alert;
 
} v2xAlertsData;



}
}

#endif