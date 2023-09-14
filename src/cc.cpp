#include "cc.h"

using namespace TOCABI;

CustomController::CustomController(RobotData &rd) : rd_(rd) //, wbc_(dc.wbc_)
{
    ControlVal_.setZero();

    if (is_write_file_)
    {
        if (is_on_robot_)
        {
            writeFile.open("/home/dyros/catkin_ws/src/tocabi_cc/result/data.csv", std::ofstream::out | std::ofstream::app);
        }
        else
        {
            writeFile.open("/home/kim/tocabi_ws/src/tocabi_cc/result/data.csv", std::ofstream::out | std::ofstream::app);
        }
        writeFile << std::fixed << std::setprecision(8);
    }
    initVariable();
}

Eigen::VectorQd CustomController::getControl()
{
    return ControlVal_;
}

void CustomController::initVariable()
{           
    q_init_ << 0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
                0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
                0.0, 0.0, 0.0,
                0.3, 0.3, 1.5, -1.27, -1.0, 0.0, -1.0, 0.0,
                0.0, 0.0,
                -0.3, -0.3, -1.5, 1.27, 1.0, 0.0, 1.0, 0.0;

    kp_.setZero();
    kv_.setZero();
    kp_.diagonal() <<   2000.0, 5000.0, 4000.0, 3700.0, 3200.0, 3200.0,
                        2000.0, 5000.0, 4000.0, 3700.0, 3200.0, 3200.0,
                        6000.0, 10000.0, 10000.0,
                        400.0, 1000.0, 400.0, 400.0, 400.0, 400.0, 100.0, 100.0,
                        100.0, 100.0,
                        400.0, 1000.0, 400.0, 400.0, 400.0, 400.0, 100.0, 100.0;
    kv_.diagonal() << 15.0, 50.0, 20.0, 25.0, 24.0, 24.0,
                        15.0, 50.0, 20.0, 25.0, 24.0, 24.0,
                        200.0, 100.0, 100.0,
                        10.0, 28.0, 10.0, 10.0, 10.0, 10.0, 3.0, 3.0,
                        2.0, 2.0,
                        10.0, 28.0, 10.0, 10.0, 10.0, 10.0, 3.0, 3.0;
}

void CustomController::processNoise()
{
    time_cur_ = rd_cc_.control_time_us_ / 1e6;
    q_vel_noise_ = rd_cc_.q_dot_virtual_.segment(6,MODEL_DOF);
    q_noise_= rd_cc_.q_virtual_.segment(6,MODEL_DOF);
}



void CustomController::computeSlow()
{
    copyRobotData(rd_);
    if (rd_cc_.tc_.mode == 7)
    {
        if (rd_cc_.tc_init)
        {
            //Initialize settings for Task Control! 
            start_time_ = rd_cc_.control_time_us_;
            q_noise_pre_ = q_noise_ = q_init_ = rd_cc_.q_virtual_.segment(6,MODEL_DOF);
            time_cur_ = start_time_ / 1e6;
            time_pre_ = time_cur_ - 0.005;
            // ft_left_init_ = abs(rd_cc_.LF_FT(2));
            // ft_right_init_ = abs(rd_cc_.RF_FT(2));

            rd_.tc_init = false;
            std::cout<<"cc mode 7"<<std::endl;
            torque_init_ = rd_cc_.torque_desired;

            processNoise();
        }

        processNoise();
        
        if (rd_cc_.control_time_us_ < start_time_ + 2.0e6)
        {
            moveStretchPose(start_time_/ 1e6, 2.0, q_init_);
        }
        else
        {
            if (inspect_joint_idx_ <12)
            {
                if (rd_cc_.control_time_us_ / 1e6 < inspect_joint_finish_time_ + 2.0)
                {
                    moveStretchPose(inspect_joint_finish_time_, 2.0, q_init_);
                }
                else
                {
                    inspect_time_ = rd_cc_.control_time_us_ / 1e6;

                    if (inspect_time_ - inspect_torque_change_time_ > torque_change_interval_)
                    {
                        inspect_torque_ += torque_increment_;
                        inspect_torque_change_time_ = inspect_time_;
                        std::cout << "Inspect Joint: Joint " << inspect_joint_idx_ << ", Torque: " << inspect_torque_ << std::endl;
                    }

                    Eigen::Matrix<double, MODEL_DOF, 1> command_torque;
                    Eigen::Matrix<double, MODEL_DOF, 1> q_target; 
                    q_target = q_init_;
                    for (int i=0; i<12; i++) {
                        q_target(i) = 0.0;
                    }
                    command_torque = kp_ * (q_target - q_noise_) - kv_*q_vel_noise_;
                    command_torque(inspect_joint_idx_) = inspect_torque_;
                    
                    if (abs(q_vel_noise_(inspect_joint_idx_)) > q_dot_threshold_)
                    {
                        inspect_joint_finish_time_ = inspect_time_;
                        q_init_ = rd_cc_.q_virtual_.segment(6,MODEL_DOF);
                        std::cout << "Joint " << inspect_joint_idx_ << " Finished!" << std::endl;

                        inspect_joint_idx_++;
                        inspect_torque_ = 0.0;
                    }

                    rd_.torque_desired = command_torque;
                }
            }
            else
            {
                Eigen::Matrix<double, MODEL_DOF, 1> q_target; 
                q_target = q_init_;
                for (int i=0; i<12; i++) {
                    q_target(i) = 0.0;
                }
                rd_.torque_desired = kp_ * (q_target - q_noise_) - kv_*q_vel_noise_;
            }
        }


        if (is_write_file_)
        {
            if ((rd_cc_.control_time_us_ - time_write_pre_)/1e6 > 1/240.0)
            {
                writeFile << (rd_cc_.control_time_us_ - start_time_)/1e6 << "\t";
                writeFile << inspect_joint_idx_  << "\t";
                writeFile << inspect_torque_  << "\t";
                writeFile << rd_.torque_desired.transpose()  << "\t";
                writeFile << q_vel_noise_.transpose()  << "\t";
               
                writeFile << std::endl;

                time_write_pre_ = rd_cc_.control_time_us_;
            }
        }

    }
}

void CustomController::moveStretchPose(float traj_start_time, float traj_duration, Eigen::Matrix<double, MODEL_DOF, 1> q_init)
{
    Eigen::Matrix<double, MODEL_DOF, 1> q_target;
    Eigen::Matrix<double, MODEL_DOF, 1> q_cubic;

    q_target = q_cubic = q_init;
    for (int i=0; i<12; i++) {
        q_target(i) = 0.0;
    }

    for (int i=0; i<12; i++) {
        q_cubic(i) = DyrosMath::cubic(rd_cc_.control_time_us_/1e6, traj_start_time, traj_start_time + traj_duration, q_init(i), q_target(i), 0.0, 0.0);;
    }

    torque_stretch_ = kp_ * (q_cubic - q_noise_) - kv_*q_vel_noise_;

    rd_.torque_desired = torque_stretch_;
}

void CustomController::computeFast()
{
    // if (tc.mode == 10)
    // {
    // }
    // else if (tc.mode == 11)
    // {
    // }
}

void CustomController::computePlanner()
{
}

void CustomController::copyRobotData(RobotData &rd_l)
{
    std::memcpy(&rd_cc_, &rd_l, sizeof(RobotData));
}