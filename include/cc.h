#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"
#include <random>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

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

    //////////////////////////////////////////// Donghyeon RL /////////////////////////////////////////
    void processNoise();
    void initVariable();
    void movePose(float traj_start_time, float traj_duration, Eigen::Matrix<double, MODEL_DOF, 1> q_init, Eigen::Matrix<double, MODEL_DOF, 1> q_target);

    std::ofstream writeFile[12];

    bool is_on_robot_ = false;
    bool is_write_file_ = true;

    Eigen::Matrix<double, MODEL_DOF, 1> q_init_;
    Eigen::Matrix<double, MODEL_DOF, 1> q_init_1_;
    Eigen::Matrix<double, MODEL_DOF, 1> q_stretch_;
    Eigen::Matrix<double, MODEL_DOF, 1> q_noise_;
    Eigen::Matrix<double, MODEL_DOF, 1> q_noise_pre_;
    Eigen::Matrix<double, MODEL_DOF, 1> q_vel_noise_;
    Eigen::Matrix<double, MODEL_DOF, 1> q_cubic;

    Eigen::Matrix<double, MODEL_DOF, 1> torque_init_;
    Eigen::Matrix<double, MODEL_DOF, 1> torque_spline_;
    Eigen::Matrix<double, MODEL_DOF, 1> torque_stretch_;

    Eigen::Matrix<double, MODEL_DOF, MODEL_DOF> kp_;
    Eigen::Matrix<double, MODEL_DOF, MODEL_DOF> kv_;

    float start_time_;
    float time_write_pre_ = 0.0;

    double time_cur_;
    double time_pre_;

    int inspect_joint_idx_ = 0;


    Eigen::Matrix<double, MODEL_DOF, 1> q_inspect_upper_;
    Eigen::Matrix<double, MODEL_DOF, 1> q_inspect_lower_;

    float inspect_time_;
    float inspect_joint_finish_time_;
    float inspect_torque_change_time_;
    float torque_change_interval_ = 1.0;
    float torque_increment_ = 0.05;
    float q_dot_threshold_ = 0.05;

    bool inspect_on_ = false;

private:
    Eigen::VectorQd ControlVal_;
};