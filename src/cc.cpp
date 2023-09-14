#include "cc.h"

using namespace TOCABI;

CustomController::CustomController(RobotData &rd) : rd_(rd) //, wbc_(dc.wbc_)
{
    ControlVal_.setZero();

    if (is_write_file_)
    {
        if (is_on_robot_)
        {
            for (int i = 0; i <12; i++)
            {
                writeFile[i].open("/home/dyros/catkin_ws/src/tocabi_cc/result/data_joint_"+to_string(i)+".csv", std::ofstream::out | std::ofstream::app);
                writeFile[i] << std::fixed << std::setprecision(8);
            }
        }
        else
        {
            for (int i = 0; i <12; i++)
            {
                writeFile[i].open("/home/kim/tocabi_ws/src/tocabi_cc/result/data_joint_"+to_string(i)+".csv", std::ofstream::out | std::ofstream::app);
                writeFile[i] << std::fixed << std::setprecision(8);
            }
        }
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
    q_stretch_  = q_init_;
    q_stretch_.head(12).setZero();    
    q_cubic = q_init_;    

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

    q_inspect_upper_ << 0.2, 0.0, 0.4, 0.6, 0.4, 0.3,
                        0.2, 0.0, 0.4, 0.6, 0.4, -0.3,
                        0.0, 0.0, 0.0,
                        0.3, 0.3, 1.5, -1.27, -1.0, 0.0, -1.0, 0.0,
                        0.0, 0.0,
                        -0.3, -0.3, -1.5, 1.27, 1.0, 0.0, 1.0, 0.0;

    q_inspect_lower_ << -0.2, 0.4, -0.4, 0.0, -0.4, -0.1,
                        -0.2, -0.4, -0.4, 0.0, -0.4, 0.1,
                        0.0, 0.0, 0.0,
                        0.3, 0.3, 1.5, -1.27, -1.0, 0.0, -1.0, 0.0,
                        0.0, 0.0,
                        -0.3, -0.3, -1.5, 1.27, 1.0, 0.0, 1.0, 0.0;
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
        
        if (rd_cc_.control_time_us_ < start_time_ + 4.0e6)
        {
            movePose(start_time_/ 1e6, 4.0, q_init_, q_stretch_);
            torque_spline_ = kp_ * (q_cubic - q_noise_) - kv_*q_vel_noise_;
            rd_.torque_desired = torque_spline_;

            inspect_joint_finish_time_ = rd_cc_.control_time_us_/1e6;
            q_init_1_ = rd_cc_.q_virtual_.segment(6,MODEL_DOF);
        }
        else
        {
            if (inspect_joint_idx_ < 12)
            {
                inspect_on_ = true;
                // Move to Stretch Pose
                if (rd_cc_.control_time_us_ / 1e6 < inspect_joint_finish_time_ + 2.0)
                {
                    movePose(inspect_joint_finish_time_, 2.0, q_init_1_, q_stretch_);
                    q_init_ = rd_cc_.q_virtual_.segment(6,MODEL_DOF);
                }
                else
                {
                    inspect_time_ = rd_cc_.control_time_us_ / 1e6;

                    // Move to upper limit
                    if (inspect_time_ < inspect_joint_finish_time_ + 4.0)
                    {
                        Eigen::Matrix<double, MODEL_DOF, 1> q_target; 
                        q_target = q_stretch_;
                        q_target(inspect_joint_idx_) = q_inspect_upper_(inspect_joint_idx_);
                        movePose(inspect_joint_finish_time_+2.0, 2.0, q_init_, q_target);
                        q_init_1_ = rd_cc_.q_virtual_.segment(6,MODEL_DOF);
                    }
                    // Move from upper limit to lower limit using cubic spline for 4 seconds
                    else if (inspect_time_ < inspect_joint_finish_time_ + 8.0)
                    {
                        Eigen::Matrix<double, MODEL_DOF, 1> q_target; 
                        q_target = q_stretch_;
                        q_target(inspect_joint_idx_) = q_inspect_upper_(inspect_joint_idx_);
                        movePose(inspect_joint_finish_time_+4.0, 4.0, q_init_1_, q_target);
                        q_init_ = rd_cc_.q_virtual_.segment(6,MODEL_DOF);
                    }
                    // Move from lower limit to upper limit using cubic spline for 4 seconds
                    else if (inspect_time_ < inspect_joint_finish_time_ + 12.0)
                    {
                        Eigen::Matrix<double, MODEL_DOF, 1> q_target; 
                        q_target = q_stretch_;
                        q_target(inspect_joint_idx_) = q_inspect_lower_(inspect_joint_idx_);
                        movePose(inspect_joint_finish_time_ + 8.0, 4.0, q_init_, q_target);
                        q_init_1_ = rd_cc_.q_virtual_.segment(6,MODEL_DOF);
                    }
                    // Move from upper limit to lower limit using cubic spline for 2 seconds
                    else if (inspect_time_ < inspect_joint_finish_time_ + 14.0)
                    {
                        Eigen::Matrix<double, MODEL_DOF, 1> q_target; 
                        q_target = q_stretch_;
                        q_target(inspect_joint_idx_) = q_inspect_upper_(inspect_joint_idx_);
                        movePose(inspect_joint_finish_time_ + 12.0, 2.0, q_init_1_, q_target);
                        q_init_ = rd_cc_.q_virtual_.segment(6,MODEL_DOF);
                    }
                    // Move from lower limit to upper limit using cubic spline for 2 seconds
                    else if (inspect_time_ < inspect_joint_finish_time_ + 16.0)
                    {
                        Eigen::Matrix<double, MODEL_DOF, 1> q_target; 
                        q_target = q_stretch_;
                        q_target(inspect_joint_idx_) = q_inspect_lower_(inspect_joint_idx_);
                        movePose(inspect_joint_finish_time_ + 14.0, 2.0, q_init_, q_target);
                        q_init_1_ = rd_cc_.q_virtual_.segment(6,MODEL_DOF);
                    }
                    // Move from upper limit to lower limit using cubic spline for 1 seconds
                    else if (inspect_time_ < inspect_joint_finish_time_ + 17.0)
                    {
                        Eigen::Matrix<double, MODEL_DOF, 1> q_target; 
                        q_target = q_stretch_;
                        q_target(inspect_joint_idx_) = q_inspect_upper_(inspect_joint_idx_);
                        movePose(inspect_joint_finish_time_ + 16.0, 1.0, q_init_1_, q_target);
                        q_init_ = rd_cc_.q_virtual_.segment(6,MODEL_DOF);
                    }
                    // Move from lower limit to upper limit using cubic spline for 1 seconds
                    else if (inspect_time_ < inspect_joint_finish_time_ + 18.0)
                    {
                        Eigen::Matrix<double, MODEL_DOF, 1> q_target; 
                        q_target = q_stretch_;
                        q_target(inspect_joint_idx_) = q_inspect_lower_(inspect_joint_idx_);
                        movePose(inspect_joint_finish_time_ + 17.0, 1.0, q_init_, q_target);
                        q_init_1_ = rd_cc_.q_virtual_.segment(6,MODEL_DOF);
                    }
                    else
                    {
                        inspect_joint_finish_time_ = inspect_time_;
                        q_init_ = rd_cc_.q_virtual_.segment(6,MODEL_DOF);
                        q_init_1_ = rd_cc_.q_virtual_.segment(6,MODEL_DOF);
                        std::cout << "Joint " << inspect_joint_idx_ << " Finished!" << std::endl;

                        inspect_joint_idx_++;
                    }

                }

                torque_spline_ = kp_ * (q_cubic - q_noise_) - kv_*q_vel_noise_;
                rd_.torque_desired = torque_spline_;
            }
            else
            {
                inspect_on_ = false;
                Eigen::Matrix<double, MODEL_DOF, 1> q_target; 
                q_target = q_init_;
                for (int i=0; i<12; i++) {
                    q_target(i) = 0.0;
                }
                rd_.torque_desired = kp_ * (q_target - q_noise_) - kv_*q_vel_noise_;
            }
        }


        if (is_write_file_ && inspect_on_)
        {
            if ((rd_cc_.control_time_us_ - time_write_pre_)/1e6 > 1/240.0)
            {
                writeFile[inspect_joint_idx_] << (rd_cc_.control_time_us_ - start_time_)/1e6 << "\t";
                writeFile[inspect_joint_idx_]  << inspect_joint_idx_  << "\t";
                writeFile[inspect_joint_idx_]  << q_noise_.transpose()  << "\t";
                writeFile[inspect_joint_idx_]  << rd_.torque_desired.transpose()  << "\t";
                writeFile[inspect_joint_idx_]  << q_vel_noise_.transpose()  << "\t";
               
                writeFile[inspect_joint_idx_]  << std::endl;

                time_write_pre_ = rd_cc_.control_time_us_;
            }
        }

    }
}

void CustomController::movePose(float traj_start_time, float traj_duration, Eigen::Matrix<double, MODEL_DOF, 1> q_init, Eigen::Matrix<double, MODEL_DOF, 1> q_target)
{
    for (int i=0; i<12; i++) {
        q_cubic(i) = DyrosMath::cubic(rd_cc_.control_time_us_/1e6, traj_start_time, traj_start_time + traj_duration, q_init(i), q_target(i), 0.0, 0.0);;
    }
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