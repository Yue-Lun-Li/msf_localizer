#include <msf_localizer/msf_localizer.h>

extern std::mutex ekf_mutex;

Extended_Kalman_filter::Extended_Kalman_filter(ros::NodeHandle nh, ros::NodeHandle private_nh){
    this->_nh = nh;
    this->_private_nh = private_nh;
    this->_x_state.resize(15);
    this->_F_transfer.resize(15,15);
    this->_P_state_cov.resize(15,15);
    this->_Q_system_noise_cov.resize(15,15);
    this->_R_measurement_cov.resize(9,9);
    this->_H_measurement.resize(9,15);
    this->_K_gain.resize(15,9);
    this->_gps_measurement.resize(9);
    ROS_INFO_STREAM(MAGENTA<<" EKF created successfully ! ");
}

Extended_Kalman_filter::~Extended_Kalman_filter(){
    ROS_INFO_STREAM(CYAN<<" EKF Shutdown successfully ! ");
}

void Extended_Kalman_filter::init(){    
    // parameters
    this->_private_nh.param("IMU_topic",
        this->_imu_topic_,std::string("/ublox_f9k/imu_means"));
    this->_private_nh.param("GPS_topic",
        this->_gps_topic_,std::string("/ublox_f9k/fix"));
    this->_private_nh.param("GPS_RPY_topic",
        this->_gps_rpy_topic_,std::string("/ublox_f9k/navatt"));
    this->_private_nh.param("GPS_Info_topic",
        this->_gps_info_topic_,std::string("/ublox_f9k/navpvt"));
    this->_private_nh.param("CAN_topic",
        this->_can_topic_,std::string("/can_info"));
    this->_private_nh.param("NDT_topic",
        this->_ndt_topic_,std::string("/ndt_pose"));
    this->_private_nh.param("NDT_RPY_topic",
        this->_ndt_rpy_topic_,std::string("/ndt/rpy"));
    this->_private_nh.param("NDT_Reliability_topic",
        this->_ndt_rel_topic_,std::string("/ndt_reliability"));
    this->_private_nh.param("NDT_Status_topic",
        this->_ndt_stat_topic_,std::string("/ndt_stat"));
    
    this->_private_nh.param("MSF_output_pose_topic",
        this->_msf_output_pose_topic_,std::string("/msf_localizer/msf_pose"));
    this->_private_nh.param("MSF_output_state_topic",
        this->_msf_output_state_topic_,std::string("/msf_localizer/msf_state"));
    this->_private_nh.param("MSF_output_fix_topic",
        this->_msf_output_fix_topic_,std::string("/msf_localizer/fix"));

    this->_private_nh.param("gps_mode",this->_gps_mode_,true);
    this->_private_nh.param("ndt_mode",this->_ndt_mode_,true);
    this->_private_nh.param("can_mode",this->_can_mode_,true);

    this->_private_nh.param("tf_map_2_base_link",this->_tf_map_2_base_link_,true);
    this->_private_nh.param("tf_map_2_msf_link",this->_tf_map_2_msf_link_,true);

    this->_private_nh.param("antenna_tf_x",this->_antenna_tf_x_,0.29);
    this->_private_nh.param("antenna_tf_y",this->_antenna_tf_y_,0.0);
    this->_private_nh.param("antenna_tf_z",this->_antenna_tf_z_,1.565);

    this->_private_nh.param("imu_bias_angular_vel_x",this->_imu_bias_angular_vel_x_,0.0023);
    this->_private_nh.param("imu_bias_angular_vel_y",this->_imu_bias_angular_vel_y_,-0.00062705);
    this->_private_nh.param("imu_bias_angular_vel_z",this->_imu_bias_angular_vel_z_,0.00089237);
    this->_private_nh.param("imu_bias_linear_acc_x",this->_imu_bias_linear_acc_x_,-0.3347);
    this->_private_nh.param("imu_bias_linear_acc_y",this->_imu_bias_linear_acc_y_,0.0);
    this->_private_nh.param("imu_bias_linear_acc_z",this->_imu_bias_linear_acc_z_,-0.0931);

    this->_private_nh.param("ndt_compesated_threshold",this->_ndt_compesated_threshold_,1.0);
    this->_private_nh.param("ndt_reliability_threshold",this->_ndt_reliability_threshold_,30.0);
    this->_private_nh.param("ndt_iteration_threshold",this->_ndt_iteration_threshold_,10.0);
    this->_private_nh.param("ndt_score_threshold",this->_ndt_score_threshold_,90.0);
    this->_private_nh.param("ndt_time_cost_threshold",this->_ndt_time_cost_threshold_,100.0);
    this->_private_nh.param("ndt_gps_sat_threshold",this->_ndt_gps_sat_threshold_,10);
    this->_private_nh.param("state_check_threshold",this->_state_check_threshold_,3.0);
    this->_private_nh.param("gps_covariance_threshold",this->_gps_covariance_threshold_,0.09);

    ROS_INFO_STREAM(" Parameters received successfully ! ");

    // variables & matrices initialize 
    this->llh0 = {22.99665875,120.222584889,98.211};
    this->_level_arm = {this->_antenna_tf_x_,this->_antenna_tf_y_,this->_antenna_tf_z_};
    this->_rpy = {0,0,0};
    this->_ndt_weight = 1.0;
    
    // setup matrices value
    this->_x_state.setZero();
    this->_F_transfer.setIdentity();
    // ROS_INFO_STREAM(" F matrix : "<< this->_F_transfer);
    this->_P_state_cov.setIdentity();
    this->_Q_system_noise_cov.setIdentity();
    this->_Q_matrix_updated();
    this->_R_measurement_cov.setIdentity();
    this->_R_matrix_updated(true);
    this->_H_measurement.setZero();
    this->_H_matrix_updated();
    this->_K_gain.setZero();
    this->_gps_measurement.setZero();
    ROS_INFO_STREAM(" Variables & matrices initialized successfully ! ");

    // make sure shutdown every sub
    this->_imu_sub.shutdown();
    this->_gps_sub.shutdown();
    this->_gps_rpy_sub.shutdown();
    this->_gps_info_sub.shutdown();
    this->_can_sub.shutdown();
    this->_ndt_sub.shutdown();
    this->_ndt_rpy_sub.shutdown();
    this->_ndt_rel_sub.shutdown();
    this->_ndt_stat_sub.shutdown();
    ROS_INFO_STREAM(" Subscribers initialized successfully ! ");

    // initial flag
    this->_gps_initial_or_not = false;
    this->_gps_rpy_updated_or_not = false;
    this->_gps_info_initial_or_not = false;
    this->_x_state_initial_or_not = false;
    this->_ndt_iteration  = false;
    this->_ndt_sat = false;
    this->_ndt_score = false;
    this->_ndt_time_cost = false;
    this->_gps_sat = false;
    this->_ndt_msf_switch = false;
    this->_gps_covariance = false;
    
    ROS_INFO_STREAM(" Flags initialized successfully ! ");
    ROS_INFO_STREAM(MAGENTA<<" EKF initialized successfully ! ");
};

void Extended_Kalman_filter::run(){
    // publisher
    this->_msf_localizer_pub = this->_nh.advertise<geometry_msgs::PoseStamped>
        (this->_msf_output_pose_topic_.c_str(),1000);
    this->_msf_state_pub = this->_nh.advertise<msf_localizer::Msf_state>
        (this->_msf_output_state_topic_.c_str(),1000);
    this->_msf_fix_pub = this->_nh.advertise<sensor_msgs::NavSatFix>
        (this->_msf_output_fix_topic_.c_str(),1000);
    
    // subscriber
    this->_imu_sub = this->_nh.subscribe(this->_imu_topic_.c_str(),1000,
        &Extended_Kalman_filter::_imu_callback,this);
    if (this->_can_mode_ == true){
        this->_can_sub = this->_nh.subscribe(this->_can_topic_.c_str(),1000,
            &Extended_Kalman_filter::_can_callback,this);
    }
    //if (this->_gps_mode_ == true){
        this->_gps_sub = this->_nh.subscribe(this->_gps_topic_.c_str(),1000,
            &Extended_Kalman_filter::_gps_callback,this);
        this->_gps_rpy_sub = this->_nh.subscribe(this->_gps_rpy_topic_.c_str(),1000,
            &Extended_Kalman_filter::_gps_rpy_callback,this);
        this->_gps_info_sub = this->_nh.subscribe(this->_gps_info_topic_.c_str(),1000,
            &Extended_Kalman_filter::_gps_info_callback,this);
    //}
    if (this->_ndt_mode_ == true){
        this->_ndt_sub = this->_nh.subscribe(this->_ndt_topic_.c_str(),1000,
            &Extended_Kalman_filter::_ndt_callback,this);
        this->_ndt_rel_sub = this->_nh.subscribe(this->_ndt_rel_topic_.c_str(),1000,
            &Extended_Kalman_filter::_ndt_rel_callback,this);
        this->_ndt_stat_sub = this->_nh.subscribe(this->_ndt_stat_topic_.c_str(),1000,
            &Extended_Kalman_filter::_ndt_stat_callback,this);
    }
    ROS_INFO_STREAM(CYAN<<" EKF : ready ! ");
};

// Callback function
void Extended_Kalman_filter::_imu_callback(const sensor_msgs::Imu::ConstPtr input){
    ROS_INFO_STREAM(BOLDWHITE <<"------------------------------------------------------");
    ekf_mutex.lock();
    static ros::Time start_time,end_time;
    static Eigen::Matrix4d rotation_matrix;
    static Eigen::Vector4d acc_b;
    static Eigen::Vector4d acc_ned;

    start_time = ros::Time::now();

    // remove bias
    this->_imu_data = *input;
    // ROS_INFO_STREAM(GREEN <<" EKF : IMU updated ! ");
    this->_imu_data.linear_acceleration.x -= this->_imu_bias_linear_acc_x_;
    this->_imu_data.linear_acceleration.y -= this->_imu_bias_linear_acc_y_;
    this->_imu_data.linear_acceleration.z -= this->_imu_bias_linear_acc_z_;
    this->_imu_data.angular_velocity.x -= this->_imu_bias_angular_vel_x_;
    this->_imu_data.angular_velocity.y -= this->_imu_bias_angular_vel_y_;
    this->_imu_data.angular_velocity.z -= this->_imu_bias_angular_vel_z_;
    
    // make imu-frame be NED (down)
    this->_imu_data.linear_acceleration.x *= -1; 
    this->_imu_data.angular_velocity.y *= -1;
    this->_imu_data.angular_velocity.z *= -1;
    
    // transfer acc from body-frame to NED && remove gravity acc
    acc_b << this->_imu_data.linear_acceleration.x, 
             this->_imu_data.linear_acceleration.y,
             this->_imu_data.linear_acceleration.z,
             1; 
    rotation_matrix = mec_transformation::rotation_matrix_eular(this->_rpy);
    acc_ned = rotation_matrix*acc_b;
    // after rotation, axis would be -x,-y,-z
    this->_acc_ned ={-acc_ned(0),-acc_ned(1),(G-acc_ned(2))}; 

    // prediction
    if (this->_gps_initial_or_not * this->_gps_info_initial_or_not != 0 ){
        this->_X_matrix_updated();
        this->_Prediction();
        
        // measurement
        if ( this->_x_state_initial_or_not != 0){
            this->_GPS_matrix_updated();
            this->_Measurement(this->_x_state);
        }else{
            ROS_INFO_STREAM(BOLDWHITE <<" EKF : waiting for GPS updated ! ");
            ROS_INFO_STREAM(BOLDWHITE <<" EKF : waiting for GPS matrix updated ! ");
            ROS_INFO_STREAM( BOLDCYAN <<" EKF : waiting for measured mode ! ");
        }

        // check state 
        this->_state_check();
        
        // publish
        this->_Publisher();
        ROS_INFO_STREAM(GREEN <<" EKF : Vel: "<< this->_can_data.speed <<" km/hr ");
    }

    // time updated 
    this->_last_time = this->_imu_data.header.stamp;
    end_time = ros::Time::now();
    this->_time_cost = (end_time.toSec() - start_time.toSec())*1e3;
    ekf_mutex.unlock();
    ROS_INFO_STREAM(BOLDBLACK <<"------------------------------------------------------");
}
void Extended_Kalman_filter::_gps_callback(const sensor_msgs::NavSatFix::ConstPtr input){
    this->_gps_data = *input;
    // level arm compesated
    this->_llhA = mec_transformation::lever_arm(this->_gps_data.latitude,
        this->_gps_data.longitude,this->_gps_data.altitude,this->_rpy.roll,
        this->_rpy.pitch,this->_rpy.yaw,this->_level_arm.xx,
        this->_level_arm.yy,this->_level_arm.zz);
    this->_enuA = mec_transformation::lla2enu(this->llh0.Lat,this->llh0.Lon,
        this->llh0.High,this->_llhA.Lat,this->_llhA.Lon,this->_llhA.High);
    this->_nedA = {this->_enuA.n,this->_enuA.e,-this->_enuA.u};
    this->_gps_initial_or_not = true;
    this->_gps_covariance = (this->_gps_data.position_covariance[0] <= this->_gps_covariance_threshold_)? true:false;
    
    // ROS_INFO_STREAM(BOLDMAGENTA <<" EKF : GPS fix updated ! ");
}
void Extended_Kalman_filter::_gps_rpy_callback(const ublox_msgs::NavATT::ConstPtr input){
    this->_gps_rpy_data = *input;
    this->_gps_rpy.roll = this->_gps_rpy_data.roll * 1e-5;
    this->_gps_rpy.pitch = this->_gps_rpy_data.pitch * 1e-5; 
    this->_gps_rpy.yaw = this->_gps_rpy_data.heading * 1e-5; 
    // this->_gps_rpy_data.accRoll *= 1e-5;
    // this->_gps_rpy_data.accPitch *= 1e-5;
    // this->_gps_rpy_data.accHeading *= 1e-5;
    if (this->_gps_rpy_updated_or_not == false){
        this->_rpy.roll = this->_gps_rpy.roll;
        this->_rpy.pitch = this->_gps_rpy.pitch;
        this->_rpy.yaw = this->_gps_rpy.yaw;
    }
    this->_gps_rpy_updated_or_not = true;
    // ROS_INFO_STREAM(BOLDMAGENTA <<" EKF : GPS RPY updated ! ");
}
void Extended_Kalman_filter::_gps_info_callback(const ublox_msgs::NavPVT::ConstPtr input){
    this->_gps_info_data = *input;
    static int gps_sat_count = 0;
    // this->_gps_info_data.lon *= 1e-7;
    // this->_gps_info_data.lat *= 1e-7;
    // this->_gps_info_data.height *= 1e-3;
    // this->_gps_info_data.hMSL *= 1e-3;
    // this->_gps_info_data.hAcc *= 1e-3;
    // this->_gps_info_data.vAcc *= 1e-3;
    this->_gps_vel_ned.xx = this->_gps_info_data.velN * 1e-3;
    this->_gps_vel_ned.yy = this->_gps_info_data.velE * 1e-3;
    this->_gps_vel_ned.zz = this->_gps_info_data.velD * 1e-3;
    // this->_gps_info_data.gSpeed *= 1e-3;
    // this->_gps_info_data.heading *= 1e-5;
    // this->_gps_info_data.sAcc *= 1e-3;
    // this->_gps_info_data.headAcc *= 1e-5;
    // this->_gps_info_data.headVeh *= 1e-5;
    // this->_gps_info_data.magDec *= 1e-2;
    // this->_gps_info_data.magAcc *= 1e-2;
    if ( this->_gps_sat == false ){
        this->_gps_sat = (gps_sat_count == 5)? true : false;
        gps_sat_count = ( (gps_sat_count != 5) && (this->_gps_info_data.numSV > this->_ndt_gps_sat_threshold_) &&  (this->_gps_data.position_covariance[0] <= this->_gps_covariance_threshold_))? gps_sat_count + 1 : 0;
    }else{
        this->_gps_sat = ((this->_gps_info_data.numSV < this->_ndt_gps_sat_threshold_) ||  (this->_gps_data.position_covariance[0] > this->_gps_covariance_threshold_))? false : true;
        ROS_INFO_STREAM(WHITE <<" EKF : GPS false ");
    }
    this->_gps_info_initial_or_not = true;
    // ROS_INFO_STREAM(BOLDMAGENTA <<" EKF : GPS info updated ! ");
}
void Extended_Kalman_filter::_can_callback(const autoware_can_msgs::CANInfo::ConstPtr input){
    this->_can_data = *input;
    this->_can_vel_ms = (this->_can_data.speed < 35)? 
        this->_can_data.speed/3.6 : this->_can_vel_ms;
    //ROS_INFO_STREAM(GREEN <<" EKF : CAN updated ! ( Vel: "<< this->_can_data.speed <<" km/hr ) ");
}
void Extended_Kalman_filter::_ndt_callback(const geometry_msgs::PoseStamped::ConstPtr input){
    this->_ndt_data = *input;
    // ROS_INFO_STREAM(GREEN <<" EKF : NDT updated ! ");
}
void Extended_Kalman_filter::_ndt_rpy_callback(const autoware_msgs::RPY::ConstPtr input){
    this->_ndt_rpy_data = *input;
    // ROS_INFO_STREAM(GREEN <<" EKF : NDT RPY updated ! ");
}
void Extended_Kalman_filter::_ndt_rel_callback(const std_msgs::Float32::ConstPtr input){
    this->_ndt_rel_data = *input;
    static int total_count = 0, pointer = 0, unstabel_count = 0;
    static double rel_arr[100] = {};
    
    total_count = (total_count <100)? total_count + 1 : 100; 
    if (total_count == 100){
        if (this->_can_data.speed < 0.001){
            this->_ndt_weight = this->_ndt_weight;
        }
        else{
            // if (unstabel_count > 50){
            //     unstabel_count = 50;
            // }
            this->_ndt_weight = double(((total_count-unstabel_count)*0.01)*0.01 +0.99); 
        }
        unstabel_count = ( std::abs(rel_arr[pointer]) > this->_ndt_reliability_threshold_)? 
                (unstabel_count!=0)? (unstabel_count - 1) : unstabel_count : unstabel_count;
    }
    rel_arr[pointer] = this->_ndt_rel_data.data; 
    unstabel_count = ( std::abs(rel_arr[pointer]) > this->_ndt_reliability_threshold_)? unstabel_count + 1 : unstabel_count; 
    pointer = (pointer < 99)? pointer + 1 : 0; 
    // ROS_INFO_STREAM(GREEN <<" EKF : NDT reliability updated ! ");
}
void Extended_Kalman_filter::_ndt_stat_callback(const autoware_msgs::NDTStat::ConstPtr input){
    this->_ndt_stat_data = *input;
    this->_ndt_score = (this->_ndt_stat_data.score < this->_ndt_score_threshold_)? true : false;
    this->_ndt_iteration = (this->_ndt_stat_data.iteration < this->_ndt_iteration_threshold_)? true : false;
    this->_ndt_time_cost = (this->_ndt_stat_data.exe_time < this->_ndt_time_cost_threshold_)? true : false;
    // ROS_INFO_STREAM(GREEN <<" EKF : NDT status updated ! ");
}
// menber function
void Extended_Kalman_filter::_Prediction(){
    static Eigen::VectorXd x_state_predicted(15);
    static Eigen::MatrixXd p_state_cov_predicted(15,15);
    static bool can_move = (this->_can_data.speed < 0.001)? true : false;
    this->_dt = this->_imu_data.header.stamp.toSec() - this->_last_time.toSec();
    if (this->_dt != 0){
        if (this->_can_mode_ * can_move != 0){
            this->_gps_measurement(0) = this->_x_state(0);
            this->_gps_measurement(1) = this->_x_state(1);
            this->_gps_measurement(2) = this->_x_state(2);
            this->_gps_measurement(3) = 0;
            this->_gps_measurement(4) = 0;
            this->_gps_measurement(5) = 0;
            this->_gps_measurement(6) = this->_x_state(9);
            this->_gps_measurement(7) = this->_x_state(10);
            this->_gps_measurement(8) = this->_x_state(11);
        }else{
            this->_F_matrix_updated();
            x_state_predicted = this->_F_transfer * this->_x_state;
            p_state_cov_predicted = this->_F_transfer * this->_P_state_cov * this->_F_transfer.transpose() 
                + this->_Q_system_noise_cov;
            this->_x_state = x_state_predicted;
            this->_P_state_cov = p_state_cov_predicted;
            // ROS_INFO_STREAM(BOLDYELLOW <<" EKF : predicted mode ! ");
        }
    }else{
        // ROS_INFO_STREAM(BOLDBLUE <<" EKF : df = " << 0 << " (s)");
        // ROS_INFO_STREAM(BOLDYELLOW <<" EKF : waiting for predicted mode ! ");
    }
}
void Extended_Kalman_filter::_X_matrix_updated(){
    // x_state initialize
    if (this->_x_state_initial_or_not == false){
        this->_x_state << this->_nedA.n, this->_nedA.e, this->_nedA.d, 
            this->_gps_vel_ned.xx, this->_gps_vel_ned.yy, this->_gps_vel_ned.zz, 
            this->_acc_ned.xx, this->_acc_ned.yy, this->_acc_ned.zz,
            this->_rpy.roll, this->_rpy.pitch, this->_rpy.yaw, 
            this->_imu_data.angular_velocity.x, this->_imu_data.angular_velocity.y, this->_imu_data.angular_velocity.z;
        this->_x_state_initial_or_not = true; 
        this->_previous_yaw = _rpy.yaw;
        ROS_INFO_STREAM( BOLDCYAN<<" EKF : X vector initialized ! ");
    }else{
        this->_x_state(6) = this->_acc_ned.xx;
        this->_x_state(7) = this->_acc_ned.yy;
        this->_x_state(8) = this->_acc_ned.zz;
        this->_x_state(12) = this->_imu_data.angular_velocity.x * 180.0/PI;
        this->_x_state(13) = this->_imu_data.angular_velocity.y * 180.0/PI;
        this->_x_state(14) = this->_imu_data.angular_velocity.z * 180.0/PI;
    }   
    if ( this->_can_mode_ == true ){
        if ( this->_can_data.speed < 0.001 ){
            this->_x_state(6) = 0;
            this->_x_state(7) = 0;
            this->_x_state(8) = 0;
            this->_x_state(12) = 0;
            this->_x_state(13) = 0;
            this->_x_state(14) = 0;
            ROS_INFO_STREAM( BOLDBLUE <<" EKF : vehicle is static ! ");
        }else{
            // ROS_INFO_STREAM( BOLDBLUE <<" EKF : IMU data updated ! ");
        }
    }else{
        // ROS_INFO_STREAM( BOLDBLUE <<" EKF : IMU data updated ! ");
    } 
}
void Extended_Kalman_filter::_F_matrix_updated(){
    double dt = this->_dt;
    this->_F_transfer(0,3) = dt;
    this->_F_transfer(0,6) = 0.5*dt*dt;
    this->_F_transfer(1,4) = dt;
    this->_F_transfer(1,7) = 0.5*dt*dt;
    this->_F_transfer(2,5) = dt;
    this->_F_transfer(2,8) = 0.5*dt*dt;
    this->_F_transfer(3,6) = dt;
    this->_F_transfer(4,7) = dt;
    this->_F_transfer(5,8) = dt;
    this->_F_transfer(9,12) = dt;
    this->_F_transfer(10,13) = dt;
    this->_F_transfer(11,14) = dt;
    // ROS_INFO_STREAM(BOLDBLUE <<" EKF : F matrix updated ! , df = " << dt << " (s)");
}
void Extended_Kalman_filter::_Q_matrix_updated(){
    this->_Q_system_noise_cov(0,0) = 1e-9;
    this->_Q_system_noise_cov(1,1) = 1e-9;
    this->_Q_system_noise_cov(2,2) = 1e-7;
    this->_Q_system_noise_cov(3,3) = 1e-5;
    this->_Q_system_noise_cov(4,4) = 1e-9;
    this->_Q_system_noise_cov(5,5) = 1e-5;
    this->_Q_system_noise_cov(6,6) = 1e-5;
    this->_Q_system_noise_cov(7,7) = 1e-5;
    this->_Q_system_noise_cov(8,8) = 1e-4;
    this->_Q_system_noise_cov(9,9) = 1e-10;//-10
    this->_Q_system_noise_cov(10,10) = 1e-5;//-5
    this->_Q_system_noise_cov(11,11) = 1e-9;//-9
    this->_Q_system_noise_cov(12,12) = 1e-4;
    this->_Q_system_noise_cov(13,13) = 1e-4;
    this->_Q_system_noise_cov(14,14) = 1e-4;

    // ROS_INFO_STREAM(" EKF : Q matrix updated ! ");
}
void Extended_Kalman_filter::_R_matrix_updated(bool ndt_switch){
    static Eigen::Vector3d pos_gps(1e0,1e0,1e-1);
    static Eigen::Vector3d vel_gps(2.5e-2,7.5e-3,1e-2);

    static Eigen::Vector3d pos_ndt(1e-3,1e-3,1e-3);
    static Eigen::Vector3d vel_ndt(2.5e-1,7.5e-2,1e-1);
    
    if (ndt_switch == false){
        this->_R_measurement_cov(0,0) = pos_ndt(0);
        this->_R_measurement_cov(1,1) = pos_ndt(1);
        this->_R_measurement_cov(2,2) = pos_ndt(2);
        this->_R_measurement_cov(3,3) = vel_ndt(0);
        this->_R_measurement_cov(4,4) = vel_ndt(1);
        this->_R_measurement_cov(5,5) = vel_ndt(2);
    }else{
        this->_R_measurement_cov(0,0) = pos_gps(0);
        this->_R_measurement_cov(1,1) = pos_gps(1);
        this->_R_measurement_cov(2,2) = pos_gps(2);
        this->_R_measurement_cov(3,3) = vel_gps(0);
        this->_R_measurement_cov(4,4) = vel_gps(1);
        this->_R_measurement_cov(5,5) = vel_gps(2);
    }
    // this->_R_measurement_cov(0,0) = pos_gps(0);
    // this->_R_measurement_cov(1,1) = pos_gps(1);
    // this->_R_measurement_cov(2,2) = pos_gps(2);
    // this->_R_measurement_cov(3,3) = 2.5e-2;
    // this->_R_measurement_cov(4,4) = 7.5e-3;
    // this->_R_measurement_cov(5,5) = 1e-2;
    this->_R_measurement_cov(6,6) = 1e0;//1e0
    this->_R_measurement_cov(7,7) = 1e0;//1e0
    this->_R_measurement_cov(8,8) = 1e-2;//1e-2

    // ROS_INFO_STREAM(" EKF : R matrix updated ! ");
}
void Extended_Kalman_filter::_H_matrix_updated(){
    this->_H_measurement(0,0) = 1;
    this->_H_measurement(1,1) = 1;
    this->_H_measurement(2,2) = 1;
    this->_H_measurement(3,3) = 1;
    this->_H_measurement(4,4) = 1;
    this->_H_measurement(5,5) = 1;
    this->_H_measurement(6,9) = 1;
    this->_H_measurement(7,10) = 1;
    this->_H_measurement(8,11) = 1;
    // ROS_INFO_STREAM(" EKF : H matrix updated ! ");
}
void Extended_Kalman_filter::_GPS_matrix_updated(){
    static double current_diff_2d = 0.0;
    static double current_gps_diff_2d = 0.0;
    static double current_diff_3d = 0.0;
    static double weight = 0.0;
    static double ndt_last_time;
    static bool ndt_time_out = false; 

    if (this->_gps_mode_ ){
        if (this->_gps_covariance){
            this->_gps_measurement << this->_nedA.n, this->_nedA.e, this->_nedA.d,
                this->_gps_vel_ned.xx, this->_gps_vel_ned.yy, this->_gps_vel_ned.zz,
                this->_gps_rpy.roll, this->_gps_rpy.pitch, this->_gps_rpy.yaw;
        }else{
            this->_gps_measurement(0) = this->_nedA.n;
            this->_gps_measurement(1) = this->_nedA.e;
            // this->_gps_measurement(2) = this->_nedA.d;  //Anna add
            this->_gps_measurement(3) = this->_gps_vel_ned.xx;
            this->_gps_measurement(4) = this->_gps_vel_ned.yy;
            this->_gps_measurement(5) = this->_gps_vel_ned.zz;
            this->_gps_measurement(6) = this->_gps_rpy.roll;
            this->_gps_measurement(7) = this->_gps_rpy.pitch;
            this->_gps_measurement(8) = this->_gps_rpy.yaw;
        }
    }
    // if ( this->_can_mode_ * this->_gps_mode_ != 0 ){
    //     static double vel_gps = 0.0;
    //     static double vel_diff = 0.0;
    //     static double vel_can = 0.0;
    //     static double vel_compesated = 0.0;
    //     vel_can = this->_can_vel_ms;
    //     vel_gps = this->_gps_vel_ned.xx * this->_gps_vel_ned.xx +
    //               this->_gps_vel_ned.yy * this->_gps_vel_ned.yy + 
    //               this->_gps_vel_ned.zz * this->_gps_vel_ned.zz;

    //     vel_diff = (vel_can != 0.0)? (vel_gps - vel_can * vel_can) : 0.0;
    //     vel_compesated = std::sqrt(std::abs(vel_diff))/3;
    //     if ( vel_diff > 0 ){
    //         // this->_gps_measurement(3) -= vel_compesated;
    //         // this->_gps_measurement(4) -= vel_compesated;
    //         // this->_gps_measurement(5) -= vel_compesated;
    //     }else{
    //         // this->_gps_measurement(3) += vel_compesated;
    //         // this->_gps_measurement(4) += vel_compesated;
    //         // this->_gps_measurement(5) += vel_compesated;
    //     }
    // }
    if ( this->_ndt_mode_ * this->_gps_mode_ * this->_can_mode_ != 0 ){
        static bool can_move = (this->_can_data.speed < 0.001)? true : false;
        weight = this->_ndt_weight;
        ndt_last_time = ros::Time::now().toSec() - this->_ndt_data.header.stamp.toSec();

        ndt_time_out = (ndt_last_time >= 0.5)? true : false; 
        current_diff_2d = (this->_ndt_data.pose.position.y - this->_x_state(0))  * 
                        (this->_ndt_data.pose.position.y - this->_x_state(0))+
                        (this->_ndt_data.pose.position.x - this->_x_state(1))  * 
                        (this->_ndt_data.pose.position.x - this->_x_state(1));

        current_gps_diff_2d = (this->_ndt_data.pose.position.y - this->_gps_measurement(0))  * 
                        (this->_ndt_data.pose.position.y - this->_gps_measurement(0))+
                        (this->_ndt_data.pose.position.x - this->_gps_measurement(1))  * 
                        (this->_ndt_data.pose.position.x - this->_gps_measurement(1));
        
                            
        current_diff_3d = (this->_ndt_data.pose.position.y - this->_x_state(0))  * 
                        (this->_ndt_data.pose.position.y - this->_x_state(0))+
                        (this->_ndt_data.pose.position.x - this->_x_state(1))  * 
                        (this->_ndt_data.pose.position.x - this->_x_state(1))+
                        (-this->_ndt_data.pose.position.z - this->_x_state(2))  * 
                        (-this->_ndt_data.pose.position.z - this->_x_state(2));
        
        this->_ndt_msf_switch = ((current_diff_2d+current_diff_3d) > 
                    (this->_ndt_compesated_threshold_ * this->_ndt_compesated_threshold_))? 
                        true : false;
        
        static int ndt_sat_count = 0 ; 
        if ( this->_ndt_sat == false ){
                this->_ndt_sat = (ndt_sat_count == 10)? true : false;
                ndt_sat_count = ( (ndt_sat_count != 10) && ((!ndt_time_out) * this->_ndt_time_cost * this->_ndt_iteration * 
                this->_ndt_score != 0))? ndt_sat_count + 1 : 0;
        }else{
            this->_ndt_sat = ((!ndt_time_out) * this->_ndt_time_cost * this->_ndt_iteration * 
                this->_ndt_score == 0)? false : true;
        }





        if ((this->_ndt_sat == true) && (this->_gps_sat == true)){
            this->_gps_measurement(0) = (1-weight)*this->_gps_measurement(0) + weight * this->_ndt_data.pose.position.y;
            this->_gps_measurement(1) = (1-weight)*this->_gps_measurement(1) + weight * this->_ndt_data.pose.position.x;
            this->_gps_measurement(2) = (1-weight)*this->_gps_measurement(2) + weight * (-this->_ndt_data.pose.position.z);
            ROS_INFO_STREAM(WHITE <<" EKF : NDT & GPS Good  ");
        }
        else if((this->_ndt_sat == true) && (this->_gps_sat == false)){
            this->_gps_measurement(0) = this->_ndt_data.pose.position.y;
            this->_gps_measurement(1) = this->_ndt_data.pose.position.x;
            this->_gps_measurement(2) = (-this->_ndt_data.pose.position.z);
            ROS_INFO_STREAM(YELLOW <<" EKF : GPS bad  ");
        }
        // if ( (this->_gps_sat == false )) ){ //Anna change (use last good ndt position, because gps position can't be believed)
        //     if ( (!ndt_time_out) * this->_ndt_score * this->_ndt_iteration != 0){ //NDT good
        //         weight = this->_ndt_weight;
        //         this->_gps_measurement(0) = (1-weight)*this->_gps_measurement(0) + weight * this->_ndt_data.pose.position.y;
        //         this->_gps_measurement(1) = (1-weight)*this->_gps_measurement(1) + weight * this->_ndt_data.pose.position.x;
        //         this->_gps_measurement(2) = (1-weight)*this->_gps_measurement(2) + weight * (-this->_ndt_data.pose.position.z);
        //         ROS_INFO_STREAM(WHITE <<" EKF : GPS bad ONLY ");
            // }
        else if((this->_ndt_sat == false) && (this->_gps_sat == true) && (current_gps_diff_2d > 5)){    
            ROS_INFO_STREAM(RED <<" EKF : NDT bad ");
        }
        else{ //NDT bad
            weight = this->_ndt_weight;
            this->_gps_measurement(0) = this->_x_state(0);
            this->_gps_measurement(1) = this->_x_state(1);
            this->_gps_measurement(2) = this->_x_state(2);
            this->_gps_measurement(6) = this->_x_state(9);
            this->_gps_measurement(7) = this->_x_state(10);
            this->_gps_measurement(8) = this->_x_state(11);
            ROS_INFO_STREAM(BLUE <<" EKF : NDT & GPS bad ");
        }
            // weight =( (!ndt_time_out) * this->_ndt_score * this->_ndt_iteration == 0 )? 0.0 : this->_ndt_weight;
            // this->_gps_measurement(0) = (1-weight)*this->_gps_measurement(0) + weight * this->_ndt_data.pose.position.y;
            // this->_gps_measurement(1) = (1-weight)*this->_gps_measurement(1) + weight * this->_ndt_data.pose.position.x;
            // this->_gps_measurement(2) = (1-weight)*this->_gps_measurement(2) + weight * (-this->_ndt_data.pose.position.z);
            
        // }
        if ( can_move){
            this->_gps_measurement(0) = this->_x_state(0);
            this->_gps_measurement(1) = this->_x_state(1);
            this->_gps_measurement(2) = this->_x_state(2);
            this->_gps_measurement(3) = 0;
            this->_gps_measurement(4) = 0;
            this->_gps_measurement(5) = 0;
            this->_gps_measurement(6) = this->_x_state(9);
            this->_gps_measurement(7) = this->_x_state(10);
            this->_gps_measurement(8) = this->_x_state(11);
        }
        _R_matrix_updated(this->_gps_sat);
        ROS_INFO_STREAM(WHITE <<" EKF : Fusion weight: "<<this->_ndt_weight);
        ROS_INFO_STREAM(BOLDMAGENTA <<" EKF : GPS sat "<<this->_gps_sat<< "! ");
    }
    if (this->_ndt_mode_ != 0 && this->_gps_mode_ == 0){
        ndt_last_time = ros::Time::now().toSec() - this->_ndt_data.header.stamp.toSec();

        ndt_time_out = (ndt_last_time >= 0.5)? true : false;
        if ( (!ndt_time_out) * this->_ndt_time_cost * this->_ndt_iteration * 
                this->_ndt_score   != 0){
            this->_gps_measurement(0) = this->_ndt_data.pose.position.y;
            this->_gps_measurement(1) = this->_ndt_data.pose.position.x;
            this->_gps_measurement(2) = (-this->_ndt_data.pose.position.z);
            this->_gps_measurement(3) = this->_gps_vel_ned.xx;
            this->_gps_measurement(4) = this->_gps_vel_ned.yy;
            this->_gps_measurement(5) = this->_gps_vel_ned.zz;
            this->_gps_measurement(6) = this->_gps_rpy.roll;
            this->_gps_measurement(7) = this->_gps_rpy.pitch;
            this->_gps_measurement(8) = this->_gps_rpy.yaw;
        }else{ // if ndt fail, stop update state
            this->_gps_measurement(0) = this->_x_state(0);
            this->_gps_measurement(1) = this->_x_state(1);
            this->_gps_measurement(2) = this->_x_state(2);
            this->_gps_measurement(3) = 0;
            this->_gps_measurement(4) = 0;
            this->_gps_measurement(5) = 0;
            this->_gps_measurement(6) = this->_x_state(9);
            this->_gps_measurement(7) = this->_x_state(10);
            this->_gps_measurement(8) = this->_x_state(11);
        } 
        _R_matrix_updated(false);
    }

    if ( this->_can_mode_){
        if ( this->_can_data.speed < 0.001 ){
            this->_gps_measurement(0) = this->_x_state(0);
            this->_gps_measurement(1) = this->_x_state(1);
            this->_gps_measurement(2) = this->_x_state(2);
            this->_gps_measurement(3) = 0;
            this->_gps_measurement(4) = 0;
            this->_gps_measurement(5) = 0;
            this->_gps_measurement(6) = this->_x_state(9);
            this->_gps_measurement(7) = this->_x_state(10);
            this->_gps_measurement(8) = this->_x_state(11);
        }
    }

    ROS_INFO_STREAM(BOLDWHITE <<" EKF : GPS matrix updated ! ");
}
void Extended_Kalman_filter::_Measurement(Eigen::VectorXd & x_state_predicted){
    static Eigen::VectorXd x_state_compesated(15);
    static Eigen::MatrixXd p_state_cov_predicted(15,15);
    static Eigen::MatrixXd H_P_HT_plus_R(9,9);
    H_P_HT_plus_R = this->_H_measurement * this->_P_state_cov * this->_H_measurement.transpose() + this->_R_measurement_cov;
    this->_K_gain = this->_P_state_cov * this->_H_measurement.transpose()*(H_P_HT_plus_R.inverse());
    x_state_compesated = x_state_predicted + this->_K_gain*(this->_gps_measurement - this->_H_measurement*x_state_predicted);
    p_state_cov_predicted = this->_P_state_cov - this->_K_gain * this->_H_measurement * this->_P_state_cov;
    this->_x_state = x_state_compesated;
    this->_P_state_cov = p_state_cov_predicted;
    this->_gps_rpy_updated_or_not = false;
    ROS_INFO_STREAM( BOLDCYAN <<" EKF : measured mode ! ");
} 
void Extended_Kalman_filter::_Publisher(){
    static tf::TransformBroadcaster msf_br;
    static tf::TransformBroadcaster current_br_msf;
    static tf::Transform msf_tf_n2b;
    static tf::Quaternion msf_q;
    static tf::Vector3 msf_v;
    static ros::Time msf_time;
    static mec_transformation::llh_InitTypeDef msf_llh;
    static mec_transformation::enu_InitTypeDef msf_enu;

    msf_time = ros::Time::now();

    // send msf_tf
    msf_v.setValue(this->_x_state(1),this->_x_state(0),-this->_x_state(2));
    msf_q.setRPY(this->_x_state(9)*PI/180.0,-this->_x_state(10)*PI/180.0,(0.5*PI-this->_x_state(11)*PI/180.0));
    msf_tf_n2b.setOrigin(msf_v);
    msf_tf_n2b.setRotation(msf_q);

    if ( this->_tf_map_2_base_link_ == true){
        current_br_msf.sendTransform(tf::StampedTransform(msf_tf_n2b,msf_time,"/map","/base_link"));
    }
    if ( this->_tf_map_2_msf_link_ == true){
        msf_br.sendTransform(tf::StampedTransform(msf_tf_n2b,msf_time,"/map","/msf_link"));
    }

    // publish pose
    this->_msf_data.header.frame_id = "map";
    this->_msf_data.header.stamp = msf_time;
    this->_msf_data.pose.position.x = this->_x_state(1); // e
    this->_msf_data.pose.position.y = this->_x_state(0); // n
    this->_msf_data.pose.position.z = -this->_x_state(2); // u (-d)
    this->_msf_data.pose.orientation.x = msf_q.x(); 
    this->_msf_data.pose.orientation.y = msf_q.y(); 
    this->_msf_data.pose.orientation.z = msf_q.z(); 
    this->_msf_data.pose.orientation.w = msf_q.w(); 
    this->_msf_localizer_pub.publish(this->_msf_data);
    // publish state
    this->_msf_state_data.header.frame_id = "map";
    this->_msf_state_data.header.stamp = msf_time;
    this->_msf_state_data.pos_n = this->_x_state(0);
    this->_msf_state_data.pos_e = this->_x_state(1);
    this->_msf_state_data.pos_d = this->_x_state(2);
    this->_msf_state_data.vel_n = this->_x_state(3);
    this->_msf_state_data.vel_e = this->_x_state(4);
    this->_msf_state_data.vel_d = this->_x_state(5);
    this->_msf_state_data.acc_n = this->_x_state(6);
    this->_msf_state_data.acc_e = this->_x_state(7);
    this->_msf_state_data.acc_d = this->_x_state(8);
    this->_msf_state_data.roll  = this->_x_state(9);
    this->_msf_state_data.pitch = this->_x_state(10);
    this->_msf_state_data.yaw   = this->_x_state(11);
    this->_msf_state_data.w_x   = this->_x_state(12);
    this->_msf_state_data.w_y   = this->_x_state(13);
    this->_msf_state_data.w_z   = this->_x_state(14);
    this->_msf_state_data.time_cost = this->_time_cost ;
    this->_msf_state_pub.publish(this->_msf_state_data);
    // publish fix
    msf_enu.e = this->_x_state(1);
    msf_enu.n = this->_x_state(0);
    msf_enu.u = -this->_x_state(2);
    msf_llh = mec_transformation::enu2llh(msf_enu,this->llh0);
    this->_msf_fix_data.header.frame_id = "map";
    this->_msf_fix_data.header.stamp = msf_time;
    this->_msf_fix_data.latitude = msf_llh.Lat;
    this->_msf_fix_data.longitude = msf_llh.Lon;
    this->_msf_fix_data.altitude = msf_llh.High;
    this->_msf_fix_data.position_covariance[0] = this->_P_state_cov(0,0);
    this->_msf_fix_data.position_covariance[1] = this->_P_state_cov(0,1);
    this->_msf_fix_data.position_covariance[2] = this->_P_state_cov(0,2);
    this->_msf_fix_data.position_covariance[3] = this->_P_state_cov(1,0);
    this->_msf_fix_data.position_covariance[4] = this->_P_state_cov(1,1);
    this->_msf_fix_data.position_covariance[5] = this->_P_state_cov(1,2);
    this->_msf_fix_data.position_covariance[6] = this->_P_state_cov(2,0);
    this->_msf_fix_data.position_covariance[7] = this->_P_state_cov(2,1);
    this->_msf_fix_data.position_covariance[8] = this->_P_state_cov(2,2);
    this->_msf_fix_pub.publish(this->_msf_fix_data);
    
    // updated _rpy for imu linear acc rotation
    if ( this->_gps_rpy_updated_or_not == false){
        this->_rpy.roll = (this->_x_state(9));
        this->_rpy.pitch = (this->_x_state(10));
        this->_rpy.yaw = (this->_x_state(11));
        // ROS_INFO_STREAM(BOLDRED <<" in publisher Debug: rpy :"<<this->_rpy.roll<<", "<< this->_rpy.pitch<<", " << this->_rpy.yaw);
        // ROS_INFO_STREAM(BOLDRED <<" in publisher Debug: x_state :"<<(this->_x_state(9))<<", "<< (this->_x_state(10))<<", " << (this->_x_state(11)));
    }
    // ROS_INFO_STREAM(" EKF : publish mode ! ");
}
 void Extended_Kalman_filter::_state_check(){
    if (std::abs(this->_x_state(11)-this->_previous_yaw) >= this->_state_check_threshold_){
        if(this->_x_state(11) >= 180.0){
            this->_x_state(11) = 0.0;
        }else{
            this->_x_state(11) = 360.0;
        }
    }
    this->_previous_yaw = this->_x_state(11);
 }