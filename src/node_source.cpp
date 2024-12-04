#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/GetMaterialLocations.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>

// Global vector to store received orders
std::vector<osrf_gear::Order> order_vector;

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

    try {
        geometry_msgs::TransformStamped tfStamped;
        tfStamped = tfBuffer.lookupTransform(
            "arm1_base_link", part_pose.header.frame_id,
            ros::Time(0.0), ros::Duration(1.0)
        );

        tf2::doTransform(part_pose, goal_pose, tfStamped);

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

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "ariac_entry_node");
    ros::NodeHandle nh;

    // Service client to start the competition
    ros::ServiceClient start_client = nh.serviceClient<std_srvs::Trigger>("/ariac/start_competition");

    // Service client for material_location
    ros::ServiceClient location_client = nh.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");

    // Subscriber to the orders topic
    ros::Subscriber order_sub = nh.subscribe("/ariac/orders", 10, orderCallback);

    // tf2 listener for transformations
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Wait for the competition to start
    ros::Duration(1.0).sleep();
    if (!startCompetition(start_client)) {
        ROS_ERROR("Exiting due to failure in starting competition.");
        return -1;
    }

    // Main processing loop
    ros::Rate rate(10);
    while (ros::ok()) {
        // Check and process orders
        processOrders(location_client, tfBuffer);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

