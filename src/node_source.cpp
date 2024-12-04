#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/GetMaterialLocations.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ik_service/PoseIK.h>
#include <vector>
#include <rosgraph_msgs/Clock.h>


// Global vector to store received orders
std::vector<osrf_gear::Order> order_vector;
// Global Variable to store joint states
sensor_msgs::JointState current_joint_states;
std::string findPartLocation(ros::ServiceClient &location_client, const std::string &part_type);

// Callback for receiving orders
void orderCallback(const osrf_gear::Order::ConstPtr &msg) {
    order_vector.push_back(*msg);
    ROS_INFO_STREAM("Received Order: " << msg->order_id);
}

// Function to start the competition
bool startCompetition(ros::ServiceClient &client) {
    std_srvs::Trigger srv;
    if (client.call(srv)) {
        if (srv.response.success) {
            ROS_INFO("Competition started: %s", srv.response.message.c_str());
            return true;
        } else {
            ROS_WARN("Failed to start competition: %s", srv.response.message.c_str());
        }
    } else {
        ROS_ERROR("Service call to start competition failed!");
    }
    return false;
}

// Function to process orders and transform the part pose
void processOrders(ros::ServiceClient &location_client, tf2_ros::Buffer &tfBuffer) {
    if (order_vector.empty()) {
        ROS_WARN("No orders available to process.");
        return;
    }

    osrf_gear::Order current_order = order_vector.front();
    order_vector.erase(order_vector.begin());

    // Access first shipment and product
    if (current_order.shipments.empty() || current_order.shipments[0].products.empty()) {
        ROS_WARN("No products available in the first order.");
        return;
    }

    auto product = current_order.shipments[0].products[0];
    std::string product_type = product.type;

    // Find the part location
    std::string location = findPartLocation(location_client, product_type);
    if (location.empty()) return;

    // Transform part pose to arm coordinates
    geometry_msgs::PoseStamped part_pose, goal_pose;
    part_pose.header.frame_id = location + "_frame";  // e.g., logical_camera_bin4_frame
    part_pose.pose = product.pose;  // Assuming the product's pose is populated
    
    geometry_msgs::TransformStamped tfStamped;
    
    ROS_INFO("Attempting to transform pose...");
    ROS_INFO("Original pose: x: %.2f, y: %.2f, z: %.2f, qx: %.2f, qy: %.2f, qz: %.2f, qw: %.2f",
         part_pose.pose.position.x, part_pose.pose.position.y, part_pose.pose.position.z,
         part_pose.pose.orientation.x, part_pose.pose.orientation.y,
         part_pose.pose.orientation.z, part_pose.pose.orientation.w);

    try {

        tfStamped = tfBuffer.lookupTransform(
            "arm1_base_link", part_pose.header.frame_id,
            ros::Time(0.0), ros::Duration(1.0)
        );
        
        ROS_INFO("Transform successfully retrieved.");
        ROS_INFO("Transform: Translation - x: %.2f, y: %.2f, z: %.2f; Rotation - qx: %.2f, qy: %.2f, qz: %.2f, qw: %.2f",
             tfStamped.transform.translation.x, tfStamped.transform.translation.y, tfStamped.transform.translation.z,
             tfStamped.transform.rotation.x, tfStamped.transform.rotation.y,
             tfStamped.transform.rotation.z, tfStamped.transform.rotation.w);

        tf2::doTransform(part_pose, goal_pose, tfStamped);
        
        ROS_INFO("Transformed pose: x: %.2f, y: %.2f, z: %.2f, qx: %.2f, qy: %.2f, qz: %.2f, qw: %.2f",
             goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z,
             goal_pose.pose.orientation.x, goal_pose.pose.orientation.y,
             goal_pose.pose.orientation.z, goal_pose.pose.orientation.w);

        // Adjust Z position
        goal_pose.pose.position.z += 0.10;  // 10 cm above part

        ROS_WARN_STREAM("Product: " << product_type 
                        << ", Pose in Arm Frame - x: " << goal_pose.pose.position.x
                        << ", y: " << goal_pose.pose.position.y
                        << ", z: " << goal_pose.pose.position.z);
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("Transform failed: %s", ex.what());
    }
}

// Callback to store joint states
void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg) {
    current_joint_states = *msg;
}

// Function to send joint trajectory to the UR10
void sendTrajectory(ros::Publisher &trajectory_pub, const std::vector<double> &joint_angles) {
    trajectory_msgs::JointTrajectory traj_msg;
    traj_msg.header.stamp = ros::Time::now();
    traj_msg.header.frame_id = "arm1_base_link";
    traj_msg.joint_names = {"linear_arm_actuator_joint", "shoulder_pan_joint",
                            "shoulder_lift_joint", "elbow_joint",
                            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = joint_angles;
    point.time_from_start = ros::Duration(0.5);  // Adjust timing as needed
    traj_msg.points.push_back(point);

    trajectory_pub.publish(traj_msg);
    ros::Duration(1.0).sleep();  // Wait for the arm to move
}

// Function to call the IK service for a given pose
bool calculateIK(ros::ServiceClient &ik_client, const geometry_msgs::Pose &pose, std::vector<double> &joint_angles) {
    ik_service::PoseIK srv;
    srv.request.part_pose = pose;

    if (!ik_client.call(srv)) {
        ROS_WARN("Failed to call ik_service for pose: [%.2f, %.2f, %.2f]",
                 pose.position.x, pose.position.y, pose.position.z);
        return false;
    }

    if (srv.response.num_sols > 0) {
        // Extract the joint angles from the first solution
        joint_angles = std::vector<double>(srv.response.joint_solutions[0].joint_angles.begin(),
                                           srv.response.joint_solutions[0].joint_angles.end());
        return true;
    } else {
        ROS_WARN("No IK solutions available for the given pose.");
        return false;
    }
}

/*
bool calculateIK(ros::ServiceClient &ik_client, const geometry_msgs::Pose &pose, std::vector<double> &joint_angles) {
    ik_service::PoseIK srv;
    srv.request.part_pose = pose;

    if (ik_client.call(srv)) {
        if (!srv.response.joint_solutions.empty()) {
            // Convert boost::array to std::vector
            joint_angles = std::vector<double>(
                srv.response.joint_solutions[0].joint_angles.begin(),
                srv.response.joint_solutions[0].joint_angles.end()
            );

            // Check for nan values in joint angles
            for (double angle : joint_angles) {
                if (std::isnan(angle)) {
                    ROS_WARN("IK solution contains nan values. Skipping trajectory.");
                    return false;
                }
            }

            ROS_INFO("Using first IK solution: [%f, %f, %f, %f, %f, %f]",
                     joint_angles[0], joint_angles[1], joint_angles[2],
                     joint_angles[3], joint_angles[4], joint_angles[5]);
            return true;
        } else {
            ROS_WARN("No valid IK solutions for the given pose.");
        }
    } else {
        ROS_ERROR("Failed to call IK service.");
    }
    return false;
}
*/

bool isValidPose(const geometry_msgs::Pose &pose) {
    if (std::isnan(pose.position.x) || std::isnan(pose.position.y) || std::isnan(pose.position.z) ||
        std::isnan(pose.orientation.x) || std::isnan(pose.orientation.y) ||
        std::isnan(pose.orientation.z) || std::isnan(pose.orientation.w)) {
        ROS_WARN("Invalid pose detected with nan values.");
        return false;
    }
    return true;
}

// Action Client callbacks
void goalActiveCallback() {
    ROS_INFO("Goal is now active.");
}

void feedbackCallback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback) {
    ROS_INFO_STREAM("Feedback received from joint: " << feedback->joint_names[0]);
}

void resultCallback(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr& result) {
    ROS_INFO("Goal finished with state: %s", state.toString().c_str());
}

// Function to send a trajectory to the Action Server
void sendTrajectory(actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& trajectory_ac, 
                    const std::vector<trajectory_msgs::JointTrajectoryPoint>& points) {
    control_msgs::FollowJointTrajectoryGoal ac_goal;

    // Set trajectory header
    ac_goal.trajectory.header.stamp = ros::Time::now();
    ac_goal.trajectory.header.frame_id = "arm1_base_link";

    // Specify joint names (assuming UR10 joint order with linear actuator)
    ac_goal.trajectory.joint_names = {
        "linear_arm_actuator_joint",
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    };

    // Add points to the trajectory
    for (const auto& point : points) {
        ac_goal.trajectory.points.push_back(point);
    }

    // Send the goal to the Action Server
    trajectory_ac.sendGoal(ac_goal, &resultCallback, &goalActiveCallback, &feedbackCallback);

    // Wait for the goal to finish (optional blocking)
    trajectory_ac.waitForResult(ros::Duration(10.0));
    if (trajectory_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Trajectory successfully executed.");
    } else {
        ROS_ERROR("Trajectory execution failed with state: %s", trajectory_ac.getState().toString().c_str());
    }
}

// Function to find the part location using the material_location service
std::string findPartLocation(ros::ServiceClient &location_client, const std::string &part_type) {
    osrf_gear::GetMaterialLocations srv;
    srv.request.material_type = part_type;

    if (location_client.call(srv)) {
        if (!srv.response.storage_units.empty()) {
            std::string location = srv.response.storage_units[0].unit_id;
            ROS_INFO_STREAM("Part " << part_type << " found in " << location);
            return location;
        } else {
            ROS_WARN_STREAM("No locations found for part: " << part_type);
        }
    } else {
        ROS_ERROR("Service call to material_location failed!");
    }
    return "";
}

// Function to generate trajectory points for specific poses
std::vector<trajectory_msgs::JointTrajectoryPoint> generateTrajectoryPoints() {
    std::vector<trajectory_msgs::JointTrajectoryPoint> points;

    // Example point 1
    trajectory_msgs::JointTrajectoryPoint point1;
    point1.positions = {0.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};  // Replace with real positions
    point1.time_from_start = ros::Duration(2.0);
    points.push_back(point1);

    // Example point 2
    trajectory_msgs::JointTrajectoryPoint point2;
    point2.positions = {0.1, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6};  // Replace with real positions
    point2.time_from_start = ros::Duration(4.0);
    points.push_back(point2);

    return points;
}


int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "ariac_entry_node");
    ros::NodeHandle nh;
    
    // Ensure /clock is publishing before proceeding
    ROS_INFO("Waiting for /clock to start publishing...");
    while (!ros::topic::waitForMessage<rosgraph_msgs::Clock>("/clock", ros::Duration(10.0))) {
        ROS_WARN("Still waiting for /clock...");
    }
    ROS_INFO("/clock is now publishing!");

    
    // Async spinner to handle callbacks
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Service clients
    ros::ServiceClient start_client = nh.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    ros::ServiceClient location_client = nh.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
    ros::ServiceClient ik_client = nh.serviceClient<ik_service::PoseIK>("pose_ik");

    // Subscriber to the orders topic
    ros::Subscriber order_sub = nh.subscribe("/ariac/orders", 10, orderCallback);
    
    // Subscribe to joint states
    ros::Subscriber joint_states_sub = nh.subscribe("/ariac/arm1/joint_states", 10, jointStatesCallback);


    // Publisher for joint trajectory
    ros::Publisher trajectory_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm1/arm/command", 10);

    // tf2 listener for transformations
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Wait for the competition to start
    ros::Duration(1.0).sleep();
    if (!startCompetition(start_client)) {
        ROS_ERROR("Exiting due to failure in starting competition.");
        return -1;
    }
    
    // Wait for Action Server
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectory_ac("ariac/arm/follow_joint_trajectory", true);
    ROS_INFO("Waiting for Action Server to start...");
    trajectory_ac.waitForServer();
    ROS_INFO("Action Server started.");
    
    ROS_INFO("Waiting for Action Server to start...");
    trajectory_ac.waitForServer();
    ROS_INFO("Action Server started.");
    
    // Generate trajectory points for moving above piston_rod_part
    std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points = generateTrajectoryPoints();

    // Send trajectory to Action Server
    sendTrajectory(trajectory_ac, trajectory_points);

    // Keep the node alive
    ros::waitForShutdown();

    // Main processing loop
    ros::Rate rate(10);
    
    while (ros::ok()) {
        if (!order_vector.empty()) {
            // Process the first order in the queue
            osrf_gear::Order current_order = order_vector.front();
            order_vector.erase(order_vector.begin());

            // Check shipments and products
            if (!current_order.shipments.empty() && !current_order.shipments[0].products.empty()) {
                auto product = current_order.shipments[0].products[0];
                std::string product_type = product.type;

                // Find the part location
                std::string location = findPartLocation(location_client, product_type);
                if (location.empty()) continue;

                // Transform part pose to arm coordinates
                geometry_msgs::PoseStamped part_pose, goal_pose;
                part_pose.header.frame_id = location + "_frame";
                part_pose.pose = product.pose;

                try {
                    geometry_msgs::TransformStamped tfStamped = tfBuffer.lookupTransform(
                        "arm1_base_link", part_pose.header.frame_id,
                        ros::Time(0.0), ros::Duration(1.0)
                    );

                    tf2::doTransform(part_pose, goal_pose, tfStamped);
                    goal_pose.pose.position.z += 0.10;  // 10 cm above part


                    // Call IK service for the goal pose
                    ik_service::PoseIK srv;
                    srv.request.part_pose = goal_pose.pose;
                    if (ik_client.call(srv)) {
                        std::vector<double> joint_angles;
                    
                        if (calculateIK(ik_client, goal_pose.pose, joint_angles)) {
                            trajectory_msgs::JointTrajectory traj_msg;
                            traj_msg.header.stamp = ros::Time::now();
                            traj_msg.header.frame_id = "arm1_base_link";
                            traj_msg.joint_names = {"linear_arm_actuator_joint", "shoulder_pan_joint",
                            "shoulder_lift_joint", "elbow_joint",
                            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

                            trajectory_msgs::JointTrajectoryPoint point;
                            point.positions = joint_angles;  // Use the calculated joint angles
                            point.time_from_start = ros::Duration(0.5);  // Adjust timing as needed
                            traj_msg.points.push_back(point);

                            trajectory_pub.publish(traj_msg);
                            ROS_INFO("Published trajectory to move the arm.");
                            ROS_INFO("Trajectory joint names: %s", traj_msg.joint_names[0].c_str());
                            ROS_INFO("Trajectory first point positions: [%f, %f, %f, %f, %f, %f, %f]",
         traj_msg.points[0].positions[0], traj_msg.points[0].positions[1],
         traj_msg.points[0].positions[2], traj_msg.points[0].positions[3],
         traj_msg.points[0].positions[4], traj_msg.points[0].positions[5],
         traj_msg.points[0].positions[6]);

                            ros::Duration(1.0).sleep();  // Wait for motion
                        } else {
                            ROS_WARN("Skipping pose due to IK failure.");
                        }
                    } else {
                        ROS_ERROR("Failed to call IK service for the pose.");
                    }

                } catch (tf2::TransformException &ex) {
                    ROS_ERROR("Transform failed: %s", ex.what());
                }
            } else {
                ROS_WARN("No valid shipments or products in the order.");
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

