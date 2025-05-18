#include <ros/ros.h>
#include <ros/duration.h>
#include <sensor_msgs/JointState.h>
#include <ruckig/ruckig.hpp>
#include <ruckig/trajectory.hpp>
#include <ruckig/utils.hpp>
#include <iostream>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <shield_planner/shield_planner.h>
#include <mujoco/mujoco.h>
#include <std_msgs/Float64MultiArray.h>

using namespace ruckig;
using FollowJointTrajectoryActionClient =
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;

// Global variable to store current joint positions
std::vector<double> latest_joint_positions(6, 0.0);
std::vector<double> latest_joint_velocities(6, 0.0);
std::vector<double> latest_joint_accelerations(6, 0.0);
std::vector<double> last_joint_velocities(6, 0.0);
ros::Time last_time;

bool joint_state_received = false;

std::vector<double> goal_config;

mjModel* model = mj_loadXML("/home/shield/code/shield_min_ws/src/parallel_search/third_party/mujoco-2.3.2/model/abb/irb_1600/irb1600_6_12_realshield.xml", nullptr, nullptr, 0);
mjData* data = mj_makeData(model);

void sendTrajectory(Trajectory<6> &trajectory, std::unique_ptr<FollowJointTrajectoryActionClient> &client, double dt, int offset){

    // Create a trajectory message
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    double duration = trajectory.get_duration();
    int num_points = static_cast<int>(duration / dt);
    StandardVector<double, 6> position, velocity, acceleration;
    goal.trajectory.points.resize(num_points);

    // Fill in the trajectory points
    goal.trajectory.header.stamp = ros::Time::now()+ros::Duration(0.004);//+ ros::Duration(0.05);
    for (size_t i = 0; i < num_points; ++i) {
        trajectory.at_time((i) * dt, position, velocity, acceleration);
        goal.trajectory.points[i].positions.assign(position.begin(), position.end());
        goal.trajectory.points[i].velocities.assign(velocity.begin(), velocity.end());
        goal.trajectory.points[i].accelerations.assign(acceleration.begin(), acceleration.end());
        goal.trajectory.points[i].time_from_start = ros::Duration((i) * dt);
    }
    std::cout << "Sending trajectory with " << num_points << " points." << std::endl;
    // Send the trajectory to the action server
    client->sendGoal(goal);
    
    ROS_INFO_STREAM("Action finished with state: " << client->getState().toString());
}

void sendTrajectoryOffline(Trajectory<6> &trajectory, std::unique_ptr<FollowJointTrajectoryActionClient> &client, double dt){

    // Create a trajectory message
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    double duration = trajectory.get_duration();
    int num_points = static_cast<int>(duration / dt);
    StandardVector<double, 6> position, velocity, acceleration;
    goal.trajectory.points.resize(num_points);
    dt = 0.004;
    // Fill in the trajectory points
    goal.trajectory.header.stamp = ros::Time::now()+ ros::Duration(0.05);
    for (size_t i = 0; i < num_points; ++i) {
        trajectory.at_time(i * dt, position, velocity, acceleration);
        goal.trajectory.points[i].positions.assign(position.begin(), position.end());
        goal.trajectory.points[i].velocities.assign(velocity.begin(), velocity.end());
        goal.trajectory.points[i].accelerations.assign(acceleration.begin(), acceleration.end());
        goal.trajectory.points[i].time_from_start = ros::Duration((i) * dt);
    }
    std::cout << "Sending trajectory with " << num_points << " points." << std::endl;
    // Send the trajectory to the action server
    client->sendGoal(goal);
    ros::Duration(0.5).sleep();  // check every 100 ms
    
    ROS_INFO_STREAM("Action finished with state: " << client->getState().toString());
}

bool checkTrajectoryForCollisions(
    mjModel* model,
    mjData* data,
    const ruckig::Trajectory<6UL, ruckig::StandardVector>& trajectory,
    double dt)
{
    const int dof = 6;
    ruckig::StandardVector<double, dof> position, velocity, acceleration;

    double duration = trajectory.get_duration();
    int num_points = static_cast<int>(duration / dt);

    for (int i = 0; i <= num_points; ++i) {
        double t = i * dt;
        trajectory.at_time(t, position, velocity, acceleration);

        if (position.size() != model->nq) {
            std::cerr << "Ruckig DOF mismatch with MuJoCo model->nq!" << std::endl;
            return true;  // assume worst case
        }

        // Copy joint positions to qpos
        for (int j = 0; j < model->nq; ++j) {
            data->qpos[j] = position[j];
        }

        // Evaluate forward kinematics
        mj_forward(model, data);

        // Check for contact
        if (data->ncon > 0) {
            std::cerr << "Collision detected at time " << t << "s" << std::endl;
            return true;
        }
    }

    return false;  // No collision throughout the trajectory
}


// JointState callback
void jointStateCallback2(const control_msgs::JointTrajectoryControllerState& msg) {
    std::vector<std::string> joint_names = {
        "joint_1", "joint_2", "joint_3",
        "joint_4", "joint_5", "joint_6"
    };

    for (size_t i = 0; i < joint_names.size(); ++i) {
        auto it = std::find(msg.joint_names.begin(), msg.joint_names.end(), joint_names[i]);
        if (it != msg.joint_names.end()) {
            int idx = std::distance(msg.joint_names.begin(), it);

            if (idx < msg.actual.positions.size()) {
                latest_joint_positions[i] = msg.actual.positions[idx];
            }
            if (idx < msg.actual.velocities.size()) {
                latest_joint_velocities[i] = msg.actual.velocities[idx];
            }

            // If accelerations are available, update the cache
            if (idx < msg.actual.accelerations.size() && !std::isnan(msg.actual.accelerations[idx])) {
                latest_joint_accelerations[i] = msg.actual.accelerations[idx];
            }
            // Else, keep the last known acceleration (or 0 if still uninitialized)
        }
    }

    joint_state_received = true;
}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    std::vector<std::string> joint_names = {
        "joint_1", "joint_2", "joint_3",
        "joint_4", "joint_5", "joint_6"
    };
    ros::Time current_time = msg->header.stamp;
    last_joint_velocities = latest_joint_velocities;
    double dt;
    if(last_time.isZero()){
        dt = 0.004; // set to constant value to prevent divide by zero
    }
    else{
        dt = (current_time - last_time).toSec();
        if (dt < 1e-6) dt = 1e-6; // guards against numerical instability
    }
    for (size_t i = 0; i < joint_names.size(); ++i) {
        auto it = std::find(msg->name.begin(), msg->name.end(), joint_names[i]);
        if (it != msg->name.end()) {
            int idx = std::distance(msg->name.begin(), it);
            latest_joint_positions[i] = msg->position[idx];
            //std::cout<<msg->position[idx]<<std::endl;
            latest_joint_velocities[i] = msg->velocity[idx];
            latest_joint_accelerations[i] = (latest_joint_velocities[i] - last_joint_velocities[i]) / dt;
        }
    }
    //std::cout<<"test"<<std::endl;
    last_time = current_time;
    joint_state_received = true;
}

void newGoalCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    goal_config= msg->position;
}



void executeTrajectory(ros::Publisher& vel_pub, ros::Rate& rate, Ruckig<6>& otg, InputParameter<6>& input,OutputParameter<6>& output, ros::Duration& break_time, bool transition){
    int count = 0;
    ros::Time 
    t_start = ros::Time::now();

    if(transition){
        input.max_jerk = {50.0, 50.0, 50.0, 50.0, 50.0, 50.0};
        for (size_t i = 0; i < 6; ++i) {
            input.current_position[i] = latest_joint_positions[i];  
            //input.current_velocity[i] = latest_joint_velocities[i];
            //input.current_acceleration[i] = latest_joint_accelerations[i];
        }
    }

    while (ros::ok() && otg.update(input, output) == Result::Working) {
        if (checkTrajectoryForCollisions(model, data, output.trajectory, 0.004)){
            std::cout << "Collision detected!" << std::endl;
            std_msgs::Float64MultiArray vel_msg;
            break;
        }
        std_msgs::Float64MultiArray vel_msg;
        vel_msg.data.insert(vel_msg.data.end(), output.new_velocity.begin(), output.new_velocity.end());
        vel_pub.publish(vel_msg);

        // Pass output to next input

        output.pass_to_input(input);
        //output.pass_to_input(input);

        joint_state_received = false;
        while (ros::ok() && !joint_state_received) {
            ros::spinOnce();
            rate.sleep();
        }
        joint_state_received = false; 
        if ( ros::Time::now() - t_start >= break_time) {
            ROS_WARN("aborting trajectory");
            break;
        }
        if(count%15 == 0){
            for (size_t i = 0; i < 6; ++i) {
                input.current_position[i] = latest_joint_positions[i];  
                //input.current_velocity[i] = latest_joint_velocities[i];
                //input.current_acceleration[i] = latest_joint_accelerations[i];
            }
        }
        //rate.sleep();
        count++;
        if (transition && count > 60) {
            input.max_jerk = {100.0, 100.0, 100.0, 100.0, 100.0, 100.0};
        }
    }   
    output.pass_to_input(input);
    std::cout<<count<<std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ruckig_test_node");
    ros::NodeHandle nh;
    ros::Rate rate(250);
    
    ros::Subscriber joint_state_sub = nh.subscribe(

        "/egm/joint_states", 10, jointStateCallback
       //"/egm/joint_velocity_trajectory_controller/state", 10, jointStateCallback
       //"sim_joint_states", 10, jointStateCallback
    );
    
   /*
    ros::Subscriber joint_state_sub2 = nh.subscribe(

        //"/egm/joint_states", 10, jointStateCallback
       "/egm/joint_velocity_trajectory_controller/state", 10, jointStateCallback2
       //"sim_joint_states", 10, jointStateCallback
    );
    */
    ros::Subscriber get_new_goal_sub = nh.subscribe(
        "/mujoco_ik", 10, newGoalCallback
    );
    ros::Publisher vel_pub = nh.advertise<std_msgs::Float64MultiArray>(
        "/egm/joint_group_velocity_controller/command", 10);

    std::unique_ptr<FollowJointTrajectoryActionClient> traj_client;
    traj_client.reset(new FollowJointTrajectoryActionClient("egm/joint_velocity_trajectory_controller/follow_joint_trajectory/", true));
    //traj_client.reset(new FollowJointTrajectoryActionClient("simulation", true));
    if(!traj_client->waitForServer(ros::Duration(1.0))){
        ROS_ERROR("joint_trajectory_action server not available");
        return false;
    }
    ROS_INFO("Connected to follow_joint_trajectory server");
     // 100 Hz

    // Wait until the first joint state is received
    joint_state_received   = false; 
    while (ros::ok() && !joint_state_received) {
        ros::spinOnce();
        rate.sleep();
    }
    joint_state_received   = false; 

    // Initialize Ruckig OTG
    Ruckig<6> otg(0.004);  // 4ms control cycle
    Ruckig<6> otg_collision(0.004);  // 4ms control cycle
    InputParameter<6> input;
    OutputParameter<6> output;
    Trajectory<6> trajectory;
    double dt = 0.004;
    joint_state_received = false;
    while (ros::ok() && !joint_state_received) {
        ros::spinOnce();
        rate.sleep();
    }
    joint_state_received = false;
    for (size_t i = 0; i < 6; ++i) {
        input.current_position[i] = latest_joint_positions[i];
        input.current_velocity[i] = latest_joint_velocities[i];       
        input.current_acceleration[i] = latest_joint_accelerations[i];   
    }

    // Define target state
    input.target_position = {0.0, 0.0, -0.0, -0.0, 0.0, 0.0};
    input.target_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    input.target_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    input.max_velocity = {2.618, 2.7925, 2.967, 5.585, 6.9813, 7.854};
    input.max_acceleration = {50.0, 50.0, 50.0, 50.0, 50.0, 50.0};
    input.max_jerk = {500.0, 500.0, 500.0, 500.0, 500.0, 500.0};

    Result result = otg_collision.calculate(input, trajectory);
    if (result == Result::ErrorInvalidInput) {
        std::cout << "Invalid input!" << std::endl;
        return -1;
    }    
    if (checkTrajectoryForCollisions(model, data, trajectory, 0.004)){
        std::cout << "Collision detected!!" << std::endl;
        return -1;
    }

    std::cout << "Trajectory duration: " << trajectory.get_duration() << std::endl;
    ros::Duration break_time(5.0);
    executeTrajectory(vel_pub, rate, otg, input, output,break_time,false);
    std_msgs::Float64MultiArray final_stop;
    final_stop.data = std::vector<double>(6, 0.0);
    vel_pub.publish(final_stop);
    //sendTrajectory(trajectory, traj_client, 0.004, 0);
    ros::Duration(1.0).sleep();
    joint_state_received = false;
    while (ros::ok() && !joint_state_received) {
        ros::spinOnce();
        rate.sleep();
    }
    joint_state_received = false; 

    StandardVector<double, 6> position, velocity, acceleration;
    //trajectory.at_time(trajectory.get_duration(), position, velocity, acceleration);

    for (size_t i = 0; i < 6; ++i) {
        input.current_position[i] = latest_joint_positions[i];
        input.current_velocity[i] = 0.0;  
        input.current_acceleration[i] = 0.0;
    }

    // Define target state
    input.target_position = {0.89651783, 0.1268922, 0.6138888, 0.51162804,-1.0071, -0.63933818};
    //for (size_t i = 0; i < 6; ++i) {
    //    input.target_position[i] = goal_config[i];
    //}
    input.target_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    input.target_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    result = otg_collision.calculate(input, trajectory);

    if (result == Result::ErrorInvalidInput) {
        std::cout << "Invalid input!" << std::endl;
        return -1;
    }
    if (checkTrajectoryForCollisions(model, data, trajectory, 0.004)){
        std::cout << "Collision detected!!" << std::endl;
        return -1;
    }
    ros::Duration break_time2(0.2);
    executeTrajectory(vel_pub, rate, otg, input, output,break_time2,false);
    for (size_t i = 0; i < 6; ++i) {
        std::cout<<"cur vel:" << input.current_velocity[i] <<std::endl;  
    }

    joint_state_received = false;
    while (ros::ok() && !joint_state_received) {
        ros::spinOnce();
        rate.sleep();
    }
    for (size_t i = 0; i < 6; ++i) {
        input.current_position[i] = latest_joint_positions[i]; 
        //input.current_velocity[i] = latest_joint_velocities[i];
    }
    for (size_t i = 0; i < 6; ++i) {
        std::cout<<"cur vel:" << input.current_velocity[i] <<std::endl;  
    }

    input.target_position = {0.39651783, 0.1268922, 0.3138888, 0.51162804,-1.0071, -0.63933818};
    //for (size_t i = 0; i < 6; ++i) {
    //    input.target_position[i] = goal_config[i];
    //}
    input.target_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    input.target_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    //sendTrajectory(trajectory, traj_client, 0.004, 0);
    ros::Duration break_time3(5.0);
    executeTrajectory(vel_pub, rate, otg, input, output,break_time3,false);
    //std_msgs::Float64MultiArray final_stop;
    final_stop.data = std::vector<double>(6, 0.0);
    vel_pub.publish(final_stop);
    std::cout<<"test"<<std::endl;
    return 0;
}
