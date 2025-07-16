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
bool goal_received = false;

std::vector<double> goal_config(6,0.0);

mjModel* model = mj_loadXML("/home/shield/code/shield_min_ws/src/replanning/mujoco_ws/abb/irb_1600/irb1600_6_12_realshield.xml", nullptr, nullptr, 0);
mjData* data = mj_makeData(model);

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

    int link6_id = mj_name2id(model, mjtObj::mjOBJ_BODY, "link_6");
    double z_pos;
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

        z_pos = data->xpos[3 * link6_id + 2];
        // Check for contact
        if (data->ncon > 0) {
            std::cerr << "Collision detected at time " << t << "s" << std::endl;
            return true;
        }
        if(z_pos  < 0.5){
            std::cerr << "Link_6 Too Low: " << z_pos << "m" << std::endl;
            return true;
        }
    }

    return false;  // No collision throughout the trajectory
}

bool checkConfigurationForCollision(
    mjModel* model,
    mjData* data,
    std::vector<double> config)
{
    int link6_id = mj_name2id(model, mjtObj::mjOBJ_BODY, "link_6");
    double z_pos;

    if (config.size() != model->nq) {
        std::cerr << "Ruckig DOF mismatch with MuJoCo model->nq!" << std::endl;
        return true;  // assume worst case
    }

    // Copy joint positions to qpos
    for (int j = 0; j < model->nq; ++j) {
        data->qpos[j] = config[j];
    }

    // Evaluate forward kinematics
    mj_forward(model, data);

    z_pos = data->xpos[3 * link6_id + 2];
    // Check for contact
    if (data->ncon > 0) {
        std::cerr << "Collision detected at new IK solution!" << std::endl;
        return true;
    }

    if(z_pos  < 0.5){
        std::cerr << "Link_6 Too Low: " << z_pos << "m" << std::endl;
        return true;
    }

    return false;  // No collision
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
    goal_received = true;
}

void updateState(ros::Rate& rate, InputParameter<6>& input){
    joint_state_received = false;
    while (ros::ok() && !joint_state_received) {
        ros::spinOnce();
        rate.sleep();
    }

    for (size_t i = 0; i < 6; ++i) {
        input.current_position[i] = latest_joint_positions[i];  

    }
}

void updateGoal(ros::Rate& rate, InputParameter<6>& input){
    goal_received = false;
    while (ros::ok() && !goal_received) {
        ros::spinOnce();
        rate.sleep();
    }
    if(!checkConfigurationForCollision(model, data, goal_config)){
        for (size_t i = 0; i < 6; ++i) {
            input.target_position[i] = goal_config[i];    
        }
    }
    else{
        std::cout << "Collision detected at new IK solution!" << std::endl;
    }
}

void updateGoalNoStop(ros::Rate& rate, InputParameter<6>& input){

    if(!checkConfigurationForCollision(model, data, goal_config)){
        for (size_t i = 0; i < 6; ++i) {
            input.target_position[i] = goal_config[i];    
        }
    }
    else{
        std::cout << "Collision detected at new IK solution!" << std::endl;
    }
}

void updateStateAndGoal(ros::Rate& rate, InputParameter<6>& input){
    joint_state_received = false;

    while (ros::ok() && !joint_state_received) {
        ros::spinOnce();
        rate.sleep();
    }
    if(!checkConfigurationForCollision(model, data, goal_config)){
        for (size_t i = 0; i < 6; ++i) {
            input.target_position[i] = goal_config[i];    
        }
    }

    for (size_t i = 0; i < 6; ++i) {
        input.current_position[i] = latest_joint_positions[i];      
    }
}


void stopExecution(ros::Publisher& vel_pub){
    std_msgs::Float64MultiArray vel_msg;
    vel_msg.data = std::vector<double>(6, 0.0);
    vel_pub.publish(vel_msg);
    ros::Duration(1.0).sleep();
}

bool executeTrajectory(ros::Publisher& vel_pub, ros::Rate& rate, Ruckig<6>& otg, InputParameter<6>& input,OutputParameter<6>& output, ros::Duration& break_time, bool transition){
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
            stopExecution(vel_pub);
            return false;
        }
        std_msgs::Float64MultiArray vel_msg;
        vel_msg.data.insert(vel_msg.data.end(), output.new_velocity.begin(), output.new_velocity.end());
        vel_pub.publish(vel_msg);

        // Pass output to next input
        output.pass_to_input(input);

        if ( ros::Time::now() - t_start >= break_time) {
            ROS_WARN("aborting trajectory");
            break;
        }
        // periodically update the current position
        if(count%15 == 0){
            updateState(rate, input);
        }
        else{
            rate.sleep();
        }
        if(count%50 == 0){
            updateGoalNoStop(rate, input);
            rate.sleep();
        }

        count++;
        if (transition && count > 60) {
            input.max_jerk = {100.0, 100.0, 100.0, 100.0, 100.0, 100.0};
        }
    }   
    output.pass_to_input(input);
    std::cout<<count<<std::endl;
    return true;
}

bool goHome(ros::Publisher& vel_pub, ros::Rate& rate, Ruckig<6>& otg, InputParameter<6>& input,OutputParameter<6>& output){

    input.max_velocity = {1.618, 1.7925, 1.967, 2.585, 3.9813, 3.854};
    input.max_acceleration = {30.0, 30.0, 30.0, 30.0, 30.0, 30.0};
    input.max_jerk = {100.0, 100.0, 100.0, 100.0, 100.0, 100.0};
    input.target_position = {0.0, 0.0, -0.0, -0.0, 0.0, 0.0};
    input.target_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    input.target_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    int count = 0;
    while (ros::ok() && otg.update(input, output) == Result::Working) {
        if (checkTrajectoryForCollisions(model, data, output.trajectory, 0.004)){
            std::cout << "Collision detected!" << std::endl;
            stopExecution(vel_pub);
            return false;
        }
        std_msgs::Float64MultiArray vel_msg;
        vel_msg.data.insert(vel_msg.data.end(), output.new_velocity.begin(), output.new_velocity.end());
        vel_pub.publish(vel_msg);
        // Pass output to next input
        output.pass_to_input(input);

        if(count%15 == 0){
            updateState(rate, input);
        }
        count++;
    }   
    input.max_velocity = {2.618, 2.7925, 2.967, 5.585, 6.9813, 7.854};
    input.max_acceleration = {50.0, 50.0, 50.0, 50.0, 50.0, 50.0};
    input.max_jerk = {500.0, 500.0, 500.0, 500.0, 500.0, 500.0};
    ros::Duration(1.0).sleep();
    return true;
}

/*
void loadRobotModel(){

    smpl::collision::CollisionModelConfig cc_conf;
    if (!smpl::collision::CollisionModelConfig::Load(ph, cc_conf)) {
        ROS_ERROR("Failed to load Collision Model Config");
        return 1;
    }

    smpl::collision::CollisionSpaceMultithread cc;
    if (!cc.init(
            num_threads,
            &grid,
            grid_vec,
            robot_description,
            cc_conf,
            robot_config.group_name,
            robot_config.planning_joints))
    {
        ROS_ERROR("Failed to initialize Collision Space");
        return 1;
    }
}
*/
int main(int argc, char** argv) {
    ros::init(argc, argv, "ruckig_test_node");
    ros::NodeHandle nh;
    ros::Rate rate(250);
    
    ros::Subscriber joint_state_sub = nh.subscribe(

        "/egm/joint_states", 10, jointStateCallback
       //"sim_joint_states", 10, jointStateCallback
    );
    
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

    // Initialize Ruckig OTG
    Ruckig<6> otg(0.004);  // 4ms control cycle
    InputParameter<6> input;
    OutputParameter<6> output;
    Trajectory<6> trajectory;

    // Initialize control bounds
    input.max_velocity = {1.618, 1.7925, 1.967, 2.585, 3.9813, 3.854};
    input.max_acceleration = {30.0, 30.0, 30.0, 30.0, 30.0, 30.0};
    input.max_jerk = {100.0, 100.0, 100.0, 100.0, 100.0, 100.0};

    // update state variables
    updateState(rate, input);

    // Go to home config
    input.target_position = {0.0, 0.0, -0.0, -0.0, 0.0, 0.0};
    input.target_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    input.target_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // check for collision before executing trajectory
    Result result = otg.calculate(input, trajectory);
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

    stopExecution(vel_pub);

    input.max_velocity = {2.618, 2.7925, 2.967, 5.585, 6.9813, 7.854};
    input.max_acceleration = {50.0, 50.0, 50.0, 50.0, 50.0, 50.0};
    input.max_jerk = {500.0, 500.0, 500.0, 500.0, 500.0, 500.0};
    
    while(ros::ok()){
        updateGoal(rate, input);
        updateState(rate, input);
        for (size_t i = 0; i < 6; ++i) { 
            input.current_velocity[i] = 0.0;
            input.current_acceleration[i] = 0.0;
        }
        if(!executeTrajectory(vel_pub, rate, otg, input, output,break_time,false)){
            break;
        }
        stopExecution(vel_pub);
        updateState(rate, input);
        goHome(vel_pub, rate, otg, input, output);

    }
    stopExecution(vel_pub);
    return 0;
}
