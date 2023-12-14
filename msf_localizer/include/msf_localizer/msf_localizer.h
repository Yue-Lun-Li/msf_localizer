#ifndef MSF_LOCALIZER_H_INCLUDE
#define MSF_LOCALIZER_H_INCLUDE

#include <ros/ros.h>

#include <string>
#include <mutex>
#include <cmath>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <autoware_msgs/RPY.h>
#include <ublox_msgs/NavATT.h>
#include <ublox_msgs/NavPVT.h>
#include <autoware_can_msgs/CANInfo.h>
#include <std_msgs/Float32.h>
#include <autoware_msgs/NDTStat.h>
#include <msf_localizer/Msf_state.h>

#include <boost/optional.hpp>
#include <Eigen/Eigen>

#include <msf_localizer/mec_transformation.h>
#include <msf_localizer/mec_color.h>

class Extended_Kalman_filter{
public:
    Extended_Kalman_filter(ros::NodeHandle nh, ros::NodeHandle private_nh);
    ~Extended_Kalman_filter();
    void init();
    void run();
    mec_transformation::llh_InitTypeDef llh0;
    
private:
    // _name -> mean private function or variable
    // name_ -> mean parameter contanior receiving param from launch
    
    // variables
    mec_transformation::llh_InitTypeDef _llhA; // body llh WGS84
    mec_transformation::ned_InitTypeDef _nedA; // body ned based on navigation-frame (EE-building)
    mec_transformation::enu_InitTypeDef _enuA; // body enu based on navigation-frame (EE-building)
    mec_transformation::xyz_InitTypeDef _level_arm; // antenna tf based on body-frame
    mec_transformation::xyz_InitTypeDef _acc_ned;
    mec_transformation::xyz_InitTypeDef _gps_vel_ned;
    mec_transformation::rpy_InitTypeDef _rpy;
    mec_transformation::rpy_InitTypeDef _gps_rpy;

    ros::Time _last_time;
    double _dt; // duration time
    double _time_cost; // ms
    double _ndt_weight;
    double _can_vel_ms;
    double _previous_yaw;

    // flag 
    bool _gps_initial_or_not;
    bool _gps_rpy_updated_or_not;
    bool _gps_info_initial_or_not;
    bool _x_state_initial_or_not;
    bool _ndt_iteration;
    bool _ndt_score;
    bool _ndt_sat;
    bool _ndt_time_cost;
    bool _gps_sat;
    bool _ndt_msf_switch;
    bool _gps_covariance;

    // Subscriber
    ros::Subscriber _imu_sub;
    ros::Subscriber _gps_sub;
    ros::Subscriber _gps_rpy_sub;
    ros::Subscriber _gps_info_sub;
    ros::Subscriber _can_sub;
    ros::Subscriber _ndt_sub;
    ros::Subscriber _ndt_rpy_sub;
    ros::Subscriber _ndt_rel_sub;
    ros::Subscriber _ndt_stat_sub;

    // Publisher
    ros::Publisher _msf_localizer_pub;
    ros::Publisher _msf_state_pub;
    ros::Publisher _msf_fix_pub;

    // ros variables
    ros::NodeHandle _nh;
    ros::NodeHandle _private_nh;

    // STL

    // Callback function
    void _imu_callback(const sensor_msgs::Imu::ConstPtr input);
    void _gps_callback(const sensor_msgs::NavSatFix::ConstPtr input);
    void _gps_rpy_callback(const ublox_msgs::NavATT::ConstPtr input);
    void _gps_info_callback(const ublox_msgs::NavPVT::ConstPtr input);
    void _can_callback(const autoware_can_msgs::CANInfo::ConstPtr input);
    void _ndt_callback(const geometry_msgs::PoseStamped::ConstPtr input);
    void _ndt_rpy_callback(const autoware_msgs::RPY::ConstPtr input);
    void _ndt_rel_callback(const std_msgs::Float32::ConstPtr input);
    void _ndt_stat_callback(const autoware_msgs::NDTStat::ConstPtr input);

    // Paramter contanior
    std::string _imu_topic_;
    std::string _gps_topic_;
    std::string _gps_rpy_topic_;
    std::string _gps_info_topic_;
    std::string _can_topic_;
    std::string _ndt_topic_;
    std::string _ndt_rpy_topic_;
    std::string _ndt_rel_topic_;
    std::string _ndt_stat_topic_;
    std::string _msf_output_pose_topic_;
    std::string _msf_output_state_topic_;
    std::string _msf_output_fix_topic_;

    bool _gps_mode_;
    bool _ndt_mode_;
    bool _can_mode_;
    bool _tf_map_2_base_link_;
    bool _tf_map_2_msf_link_;
    double _antenna_tf_x_;
    double _antenna_tf_y_;
    double _antenna_tf_z_;
    double _imu_bias_angular_vel_x_;
    double _imu_bias_angular_vel_y_;
    double _imu_bias_angular_vel_z_;
    double _imu_bias_linear_acc_x_;
    double _imu_bias_linear_acc_y_;
    double _imu_bias_linear_acc_z_;
    double _ndt_compesated_threshold_;
    double _ndt_reliability_threshold_;
    double _ndt_iteration_threshold_;
    double _ndt_time_cost_threshold_;
    double _ndt_score_threshold_;
    double _state_check_threshold_;
    double _gps_covariance_threshold_;
    int _ndt_gps_sat_threshold_;


    // define martix size by parameter
    // int _matrix_m_;
    // int _matrix_n_;
    // int _matrix_l_;

    // msgs
    sensor_msgs::Imu _imu_data;
    sensor_msgs::NavSatFix _gps_data;
    ublox_msgs::NavATT _gps_rpy_data;
    ublox_msgs::NavPVT _gps_info_data;
    autoware_can_msgs::CANInfo _can_data;
    geometry_msgs::PoseStamped _ndt_data;
    autoware_msgs::RPY _ndt_rpy_data;
    std_msgs::Float32 _ndt_rel_data;
    autoware_msgs::NDTStat _ndt_stat_data;
    geometry_msgs::PoseStamped _msf_data;
    msf_localizer::Msf_state _msf_state_data;
    sensor_msgs::NavSatFix _msf_fix_data;

    // matrix definition
    Eigen::VectorXd _x_state; // default 15*1, m*1
    Eigen::MatrixXd _F_transfer; // default 15*15, m*m
    Eigen::MatrixXd _P_state_cov; // default 15*15, m*m
    Eigen::MatrixXd _Q_system_noise_cov; // default 15*15, m*m
    Eigen::MatrixXd _R_measurement_cov; // default 9*9, n*n
    Eigen::MatrixXd _H_measurement; // default 9*15, n*m
    Eigen::MatrixXd _K_gain; // default 15*9, m*n
    Eigen::VectorXd _gps_measurement; // default 9*1, n*1

    // member function
    void _Prediction();
    void _X_matrix_updated();
    void _F_matrix_updated();
    void _Q_matrix_updated();
    void _R_matrix_updated(bool ndt_switch);
    void _H_matrix_updated();
    void _GPS_matrix_updated();
    void _Measurement(Eigen::VectorXd & x_state_predicted);
    void _Publisher();
    void _state_check();
};

#endif