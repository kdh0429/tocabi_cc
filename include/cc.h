#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"
#include <random>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <libdwbc/dwbc.h>

#include <iostream>
#include <string>

#ifndef URDF_DIR
#define URDF_DIR __FILE__
#endif

std::string getLastDirectory(const std::string& path) {
    size_t found = path.find_last_of("/\\"); // '/' 또는 '\'를 찾음
    if (found != std::string::npos) {
        return path.substr(0, found + 1); // '/' 이전까지의 부분 문자열 반환
    } else {
        return ""; // '/'가 없는 경우 빈 문자열 반환
    }
}

class RLWBC : public DWBC::RobotData
{
    int custum_data;
    double control_time;
};

class CustomController
{
public:
    CustomController(RobotData &rd);
    Eigen::VectorQd getControl();

    //void taskCommandToCC(TaskCommand tc_);
    
    void computeSlow();
    void computeFast();
    void computePlanner();
    void copyRobotData(RobotData &rd_l);

    RobotData &rd_;
    RobotData rd_cc_;

    RLWBC rd_wbc_;

    //////////////////////////////////////////// Donghyeon RL /////////////////////////////////////////
    void loadNetwork();
    void processNoise();
    void processObservation();
    void feedforwardPolicy();
    void initVariable();
    Eigen::Vector3d mat2euler(Eigen::Matrix3d mat);

    static const int num_action = 16;
    static const int num_actuator_action = 12;
    static const int num_cur_state = 50;
    static const int num_cur_internal_state = 37;
    static const int num_state_skip = 2;
    static const int num_state_hist = 5;
    static const int num_state = num_cur_internal_state*num_state_hist+num_action*(num_state_hist-1);
    static const int num_hidden = 256;

    Eigen::MatrixXd policy_net_w0_;
    Eigen::MatrixXd policy_net_b0_;
    Eigen::MatrixXd policy_net_w2_;
    Eigen::MatrixXd policy_net_b2_;
    Eigen::MatrixXd action_net_w_;
    Eigen::MatrixXd action_net_b_;
    Eigen::MatrixXd hidden_layer1_;
    Eigen::MatrixXd hidden_layer2_;
    Eigen::MatrixXd rl_action_;
    Eigen::MatrixXd rl_action_scale_;
    Eigen::VectorXd rl_action_scaled_;

    Eigen::MatrixXd value_net_w0_;
    Eigen::MatrixXd value_net_b0_;
    Eigen::MatrixXd value_net_w2_;
    Eigen::MatrixXd value_net_b2_;
    Eigen::MatrixXd value_net_w_;
    Eigen::MatrixXd value_net_b_;
    Eigen::MatrixXd value_hidden_layer1_;
    Eigen::MatrixXd value_hidden_layer2_;
    double value_;

    bool stop_by_value_thres_ = false;
    Eigen::Matrix<double, MODEL_DOF, 1> q_stop_;
    float stop_start_time_;
    
    Eigen::MatrixXd state_;
    Eigen::MatrixXd state_cur_;
    Eigen::MatrixXd state_buffer_;
    Eigen::MatrixXd state_mean_;
    Eigen::MatrixXd state_var_;

    std::ofstream writeFile;

    float phase_ = 0.0;

    bool is_on_robot_ = false;
    bool is_write_file_ = true;
    Eigen::Matrix<double, MODEL_DOF, 1> q_dot_lpf_;

    Eigen::Matrix<double, MODEL_DOF, 1> q_init_;
    Eigen::Matrix<double, MODEL_DOF, 1> q_noise_;
    Eigen::Matrix<double, MODEL_DOF, 1> q_noise_pre_;
    Eigen::Matrix<double, MODEL_DOF, 1> q_vel_noise_;

    Eigen::Matrix<double, MODEL_DOF, 1> torque_init_;
    Eigen::Matrix<double, MODEL_DOF, 1> torque_spline_;
    Eigen::Matrix<double, MODEL_DOF, 1> torque_rl_;
    Eigen::Matrix<double, MODEL_DOF, 1> torque_bound_;

    Eigen::Matrix<double, MODEL_DOF, MODEL_DOF> kp_;
    Eigen::Matrix<double, MODEL_DOF, MODEL_DOF> kv_;

    float start_time_;
    float time_inference_pre_ = 0.0;
    float time_write_pre_ = 0.0;

    double time_cur_;
    double time_pre_;
    double action_dt_accumulate_ = 0.0;

    Eigen::Vector3d euler_angle_;

    bool left_foot_contact_ref_ = true;
    bool right_foot_contact_ref_ = true;

    // float ft_left_init_ = 500.0;
    // float ft_right_init_ = 500.0;

    // Joystick
    ros::NodeHandle nh_;

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    ros::Subscriber joy_sub_;

    double target_vel_x_ = 0.0;
    double target_vel_y_ = 0.0;

private:
    Eigen::VectorQd ControlVal_;
};