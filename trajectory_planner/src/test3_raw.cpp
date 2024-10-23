#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <ros/ros.h>
#include <global_planner/rrtStarOctomap.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <trajectory_planner/polyTrajOctomap.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <thread>

using namespace std;

// Octomap tree object
octomap::OcTree *octree;

std::vector<double> start(3);
std::vector<double> goal(3);
bool startReceived = false;
bool goalReceived = false;
octomap::point3d findNearestFreeVoxel(octomap::OcTree *octree, const octomap::point3d &point)
{
    double searchRadius = 0.5; // Radius to search for a free voxel (increase if needed)
    octomap::point3d nearestFreeVoxel = point;

    while (searchRadius <= 1.0) // Max search radius to avoid infinite loops
    {
        // Iterate through voxels in a spherical region
        for (octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(point - octomap::point3d(searchRadius, searchRadius, searchRadius),
                                                                             point + octomap::point3d(searchRadius, searchRadius, searchRadius)),
                                                end = octree->end_leafs_bbx();
             it != end; ++it)
        {
            if (octree->isNodeOccupied(*it))
                continue; // Skip occupied nodes

            // If the node is not occupied, return its center as the nearest free voxel
            nearestFreeVoxel = it.getCoordinate();
            return nearestFreeVoxel;
        }

        // Increase the search radius if no free voxel was found
        searchRadius += 0.5;
    }

    // Return the original point if no free voxel found (fallback, shouldn't happen if map is reasonable)
    return nearestFreeVoxel;
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    start[0] = msg->pose.pose.position.x;
    start[1] = msg->pose.pose.position.y;
    start[2] = msg->pose.pose.position.z;

    // Round start to the nearest free voxel
    octomap::point3d start_point(start[0], start[1], start[2]);
    octomap::OcTreeNode *start_node = octree->search(start_point);

    if (start_node == nullptr || octree->isNodeOccupied(start_node))
    {
        // If node is occupied, find nearest free voxel
        start_point = findNearestFreeVoxel(octree, start_point);
    }
    else
    {
        // If the node is free, round it to the center of the voxel
        start_point = octree->keyToCoord(octree->coordToKey(start_point));
    }

    start[0] = start_point.x();
    start[1] = start_point.y();
    start[2] = start_point.z();
    startReceived = true;
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    goal[0] = msg->pose.position.x;
    goal[1] = msg->pose.position.y;
    goal[2] = msg->pose.position.z;

    // Round goal to the nearest free voxel
    octomap::point3d goal_point(goal[0], goal[1], goal[2]);
    octomap::OcTreeNode *goal_node = octree->search(goal_point);

    if (goal_node == nullptr || octree->isNodeOccupied(goal_node))
    {
        // If node is occupied, find nearest free voxel
        goal_point = findNearestFreeVoxel(octree, goal_point);
    }
    else
    {
        // If the node is free, round it to the center of the voxel
        goal_point = octree->keyToCoord(octree->coordToKey(goal_point));
    }

    goal[0] = goal_point.x();
    goal[1] = goal_point.y();
    goal[2] = goal_point.z();
    goalReceived = true;
}

int main(int argc, char **argv)
{
    std::shared_ptr<octomap::OcTree> octree_ptr;
    ros::init(argc, argv, "Simple_Path_Planner");
    ros::NodeHandle nh;

    // Load octomap (replace with your actual octomap file path)
    std::string octomapFile = "/home/manh/Documents/DATN/Result/LV2/lv2_raw.binvox.bt";
    octree = new octomap::OcTree(octomapFile);
    // octree_ptr = std::make_shared<octomap::OcTree>(octomapFile);

    // Declare Publishers
    ros::Publisher posePub = nh.advertise<geometry_msgs::PoseStamped>("/planning/uav_path_poses", 10);
    ros::Publisher pathPub = nh.advertise<nav_msgs::Path>("/planning/uav_path", 10); // New publisher for nav_msgs/Path
    ros::Publisher startVisPub = nh.advertise<visualization_msgs::Marker>("/start_position", 1000);
    ros::Publisher goalVisPub = nh.advertise<visualization_msgs::Marker>("/goal_position", 1000);

    // Declare Subscribers
    ros::Subscriber odomSub = nh.subscribe("/hummingbird/odometry_sensor1/odometry", 10, odometryCallback);
    ros::Subscriber goalSub = nh.subscribe("/red/exploration/goal", 1, goalCallback);

    // RRT* planner initialization
    const int N = 3; // 3D space
    globalPlanner::rrtStarOctomap<N> rrtStarPlanner(nh);

    ros::Rate r(10);

    while (ros::ok())
    {
        ros::spinOnce(); // Handle callbacks

        // if (!startReceived || !goalReceived)
        if (!goalReceived)
        {
            r.sleep();
            continue; // Wait until both start and goal are received
        }
        std::cout << "------------" << std::endl;

        rrtStarPlanner.updateStart(start);
        rrtStarPlanner.updateGoal(goal);

        std::cout << "Start: " << start[0] << ", " << start[1] << ", " << start[2] << std::endl;
        std::cout << "Goal: " << goal[0] << ", " << goal[1] << ", " << goal[2] << std::endl;
        // Visualization for the start position
        visualization_msgs::Marker startMarker;
        startMarker.header.frame_id = "world";
        startMarker.header.stamp = ros::Time();
        startMarker.ns = "start_vis";
        startMarker.id = 0;
        startMarker.type = visualization_msgs::Marker::SPHERE;
        startMarker.action = visualization_msgs::Marker::ADD;
        startMarker.pose.position.x = start[0];
        startMarker.pose.position.y = start[1];
        startMarker.pose.position.z = start[2];
        startMarker.scale.x = 0.4;
        startMarker.scale.y = 0.4;
        startMarker.scale.z = 0.4;
        startMarker.color.a = 0.7;
        startMarker.color.r = 1.0;
        startMarker.color.g = 0.5;
        startMarker.color.b = 1.0;
        startVisPub.publish(startMarker);

        // Visualization for the goal position
        visualization_msgs::Marker goalMarker;
        goalMarker.header.frame_id = "world";
        goalMarker.header.stamp = ros::Time();
        goalMarker.ns = "goal_vis";
        goalMarker.id = 1;
        goalMarker.type = visualization_msgs::Marker::SPHERE;
        goalMarker.action = visualization_msgs::Marker::ADD;
        goalMarker.pose.position.x = goal[0];
        goalMarker.pose.position.y = goal[1];
        goalMarker.pose.position.z = goal[2];
        goalMarker.scale.x = 0.4;
        goalMarker.scale.y = 0.4;
        goalMarker.scale.z = 0.4;
        goalMarker.color.a = 0.7;
        goalMarker.color.r = 0.2;
        goalMarker.color.g = 1.0;
        goalMarker.color.b = 0.2;
        goalVisPub.publish(goalMarker);

        // Plan path from start to goal
        nav_msgs::Path path;
        rrtStarPlanner.makePlan(path);

        // Set frame ID and timestamp for path message
        path.header.frame_id = "world";
        path.header.stamp = ros::Time::now();

        // Publish the path as a nav_msgs/Path
        pathPub.publish(path);

        // Output the points in the path
        cout << "[Planner Node]: Path points:" << endl;
        for (size_t i = 0; i < path.poses.size(); ++i)
        {
            cout << "Point " << i + 1 << ": ("
                 << path.poses[i].pose.position.x << ", "
                 << path.poses[i].pose.position.y << ", "
                 << path.poses[i].pose.position.z << ")" << endl;
        }

        // Move UAV along the path (you can adjust the delay or logic here)
        for (const auto &pose : path.poses)
        {
            posePub.publish(pose);      // Publish each point
            ros::Duration(0.1).sleep(); // Sleep between each point to simulate UAV movement
        }

        goalReceived = false;
        r.sleep();
    }
    delete octree;
    return 0;
}
