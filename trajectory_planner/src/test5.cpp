#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <ros/ros.h>
#include <global_planner/rrtStarOctomap.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <trajectory_planner/polyTrajOctomap.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <thread>
#include <cmath>

using namespace std;

// Octomap tree object
octomap::OcTree *octree;

std::vector<double> start(3);
std::vector<double> goal(3);
bool startReceived = false;
bool goalReceived = false;
bool potentialReceived = false;
double shortestDistance = std::numeric_limits<double>::max();

std::vector<geometry_msgs::Pose> potentialPoints;
geometry_msgs::Pose shortestPoint;

octomap::point3d findNearestFreeVoxel(octomap::OcTree *octree, const octomap::point3d &point)
{
    double searchRadius = 0.5; // Radius to search for a free voxel (increase if needed)
    octomap::point3d nearestFreeVoxel = point;

    while (searchRadius <= 1.0) // Max search radius to avoid infinite loops
    {
        for (octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(point - octomap::point3d(searchRadius, searchRadius, searchRadius),
                                                                             point + octomap::point3d(searchRadius, searchRadius, searchRadius)),
                                                end = octree->end_leafs_bbx();
             it != end; ++it)
        {
            if (octree->isNodeOccupied(*it))
                continue; // Skip occupied nodes

            nearestFreeVoxel = it.getCoordinate();
            return nearestFreeVoxel;
        }
        searchRadius += 0.5;
    }
    return nearestFreeVoxel;
}

double calculateDistance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
{
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

double calculatePathDistance(const nav_msgs::Path &path, std::vector<double> goal)
{
    double totalDistance = 0.0;
    for (size_t i = 1; i < path.poses.size(); ++i)
    {
        totalDistance += calculateDistance(path.poses[i - 1].pose.position, path.poses[i].pose.position);
    }
    geometry_msgs::Point lastPoint = path.poses.back().pose.position;
    geometry_msgs::Point goalPoint;
    goalPoint.x = goal[0];
    goalPoint.y = goal[1];
    goalPoint.z = goal[2];
    totalDistance += calculateDistance(lastPoint, goalPoint);
    return totalDistance;
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    start[0] = msg->pose.pose.position.x;
    start[1] = msg->pose.pose.position.y;
    start[2] = msg->pose.pose.position.z;

    octomap::point3d start_point(start[0], start[1], start[2]);
    // octomap::OcTreeNode *start_node = octree->search(start_point);

    // if (start_node == nullptr || octree->isNodeOccupied(start_node))
    // {
    //     start_point = findNearestFreeVoxel(octree, start_point);
    // }
    // else
    // {
    //     start_point = octree->keyToCoord(octree->coordToKey(start_point));
    // }

    // start[0] = start_point.x();
    // start[1] = start_point.y();
    // start[2] = start_point.z();
    startReceived = true;
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    goal[0] = msg->pose.position.x;
    goal[1] = msg->pose.position.y;
    goal[2] = msg->pose.position.z;

    octomap::point3d goal_point(goal[0], goal[1], goal[2]);
    octomap::OcTreeNode *goal_node = octree->search(goal_point);

    if (goal_node == nullptr || octree->isNodeOccupied(goal_node))
    {
        goal_point = findNearestFreeVoxel(octree, goal_point);
    }
    else
    {
        goal_point = octree->keyToCoord(octree->coordToKey(goal_point));
    }

    goal[0] = goal_point.x();
    goal[1] = goal_point.y();
    goal[2] = goal_point.z();
    goalReceived = true;
}

// Callback to receive potential points from exploration/potential
void potentialCallback(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    potentialPoints = msg->poses; // Store potential points
    potentialReceived = true;
    std::cout << "--- Receive potentials ---" << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Simple_Path_Planner");
    ros::NodeHandle nh;

    // Get topic parameters from the launch file or use defaults
    std::string odomTopic, goalTopic, pathTopic, poseTopic, map_path;
    nh.param<std::string>("odom_topic", odomTopic, "/hummingbird/odometry_sensor1/odometry");
    nh.param<std::string>("goal_topic", goalTopic, "/red/exploration/goal");
    nh.param<std::string>("path_topic", pathTopic, "/planning/uav_path");
    nh.param<std::string>("pose_topic", poseTopic, "/planning/uav_path_poses");
    nh.param<std::string>("map_path", map_path, "/home/manh/pathplaning_ws/src/trajectory_planner/map/lv1_raw.binvox.bt");

    // Load octomap (replace with your actual octomap file path)
    // std::string octomapFile = "/home/manh/Documents/DATN/Result/LV1/lv1_raw.binvox.bt";
    octree = new octomap::OcTree(map_path);

    // Declare Publishers
    // ros::Publisher posePub = nh.advertise<geometry_msgs::PoseStamped>(poseTopic, 10);
    // ros::Publisher pathPub = nh.advertise<nav_msgs::Path>(pathTopic, 10);
    // ros::Publisher startVisPub = nh.advertise<visualization_msgs::Marker>("/start_position", 1000);
    // ros::Publisher goalVisPub = nh.advertise<visualization_msgs::Marker>("/goal_position", 1000);
    ros::Publisher bestPotentialPub = nh.advertise<geometry_msgs::PoseStamped>("/best_potential", 1);

    // Declare Subscribers
    ros::Subscriber odomSub = nh.subscribe(odomTopic, 10, odometryCallback);
    ros::Subscriber goalSub = nh.subscribe(goalTopic, 1, goalCallback);
    ros::Subscriber potentialSub = nh.subscribe("/red/exploration/potential", 1, potentialCallback);

    // RRT* planner initialization
    const int N = 3;
    globalPlanner::rrtStarOctomap<N> rrtStarPlanner(nh);

    ros::Rate r(10);
    int maxRetries = 5;
    double zIncrement = 1.0;

    // ros::spin();

    while (ros::ok())
    {
        ros::spinOnce();

        // if (!goalReceived)
        // {
        //     r.sleep();
        //     continue;
        // }
        // std::cout << "------------" << std::endl;

        // std::cout << "Goal: " << goal[0] << ", " << goal[1] << ", " << goal[2] << std::endl;

        // double eDis = sqrt(pow(goal[0] - start[0], 2) + pow(goal[1] - start[1], 2) + pow(goal[2] - start[2], 2));
        // double mDis = std::abs(start[0] - goal[0]) + std::abs(start[1] - goal[1]) + std::abs(start[2] - goal[2]);

        // std::cout << "Euclid distance: " << eDis << std::endl;
        // std::cout << "Manhattan distance: " << mDis << std::endl;

        bool pathFound = false;
        int retryCount = 0;

        if (potentialReceived)
        {
            rrtStarPlanner.updateStart(start);
            // rrtStarPlanner.updateGoal(goal);

            std::cout << "Start: " << start[0] << ", " << start[1] << ", " << start[2] << std::endl;
            shortestDistance = std::numeric_limits<double>::max();
            std::vector<double> InfGainVector(potentialPoints.size());

            for (int i = 0; i < potentialPoints.size(); i++)
            {
                goal[0] = potentialPoints[i].position.x;
                goal[1] = potentialPoints[i].position.y;
                goal[2] = potentialPoints[i].position.z;
                double unknownVolume = potentialPoints[i].orientation.x;

                nav_msgs::Path currentPath;
                rrtStarPlanner.updateGoal(goal);
                rrtStarPlanner.makePlan(currentPath);

                double currentDistance = calculatePathDistance(currentPath, goal);
                // std::cout << currentDistance << std::endl;
                // InfGainVector[i] = 100.0 * unknownVolume * exp(-0.3 * currentDistance);
                // std::cout << "( " << goal[0] << ", " << goal[1] << ", " << goal[2] << " ) ( " << unknownVolume << ", " << currentDistance << ", " << InfGainVector[i] << " )" << std::endl;

                if (currentDistance < shortestDistance && currentPath.poses.size() > 1)
                {
                    shortestDistance = currentDistance;
                    shortestPoint = potentialPoints[i];
                }
                std::cout << "( " << goal[0] << ", " << goal[1] << ", " << goal[2] << " ) ( " << unknownVolume << ", " << currentDistance << " )" << std::endl;
            }
            // int maxElementIndex =
            //     max_element(InfGainVector.begin(), InfGainVector.end()) - InfGainVector.begin();
            // shortestPoint = potentialPoints[maxElementIndex];

            geometry_msgs::PoseStamped bestPotential;
            bestPotential.header.frame_id = "world";
            bestPotential.header.stamp = ros::Time::now();
            // Update the goal to the shortestPoint and plan the final path
            bestPotential.pose.position.x = shortestPoint.position.x;
            bestPotential.pose.position.y = shortestPoint.position.y;
            bestPotential.pose.position.z = shortestPoint.position.z;
            bestPotential.pose.orientation.x = 0;
            bestPotential.pose.orientation.x = 0;
            bestPotential.pose.orientation.x = 0;
            bestPotential.pose.orientation.x = 1;

            std::cout << "--> Shortest point: " << shortestPoint.position.x << ", " << shortestPoint.position.y << ", " << shortestPoint.position.z << ", " << shortestDistance << std::endl;

            bestPotentialPub.publish(bestPotential);
            // if (shortestDistance < std::numeric_limits<double>::max())
            // {
            //     std::cout << "-->Shortest path found to point: ("
            //               << shortestPoint.position.x << ", "
            //               << shortestPoint.position.y << ", "
            //               << shortestPoint.position.z << ")" << std::endl;
            //     std::cout << "-->Shortest distance: " << shortestDistance << std::endl;

            //     geometry_msgs::PoseStamped bestPotential;
            //     bestPotential.header.frame_id = "world";
            //     bestPotential.header.stamp = ros::Time::now();
            //     // Update the goal to the shortestPoint and plan the final path
            //     bestPotential.pose.position.x = shortestPoint.position.x;
            //     bestPotential.pose.position.y = shortestPoint.position.y;
            //     bestPotential.pose.position.z = shortestPoint.position.z;
            //     bestPotential.pose.orientation.x = 0;
            //     bestPotential.pose.orientation.x = 0;
            //     bestPotential.pose.orientation.x = 0;
            //     bestPotential.pose.orientation.x = 1;

            //     bestPotentialPub.publish(bestPotential);
            //     // std::cout << "-->(" << goal[0] << ", " << goal[1] << ", " << goal[2] << " )" << std::endl;
            //     // nav_msgs::Path shortestPath;
            //     // rrtStarPlanner.updateGoal(goal);
            //     // rrtStarPlanner.makePlan(shortestPath);

            //     // Publish or visualize the shortest path
            //     // pathPub.publish(shortestPath);
            // }
        }
        potentialReceived = false;

        // Visualization and publishing code
        // visualization_msgs::Marker startMarker, goalMarker;
        // startMarker.header.frame_id = goalMarker.header.frame_id = "world";
        // startMarker.ns = "start_vis";
        // startMarker.id = 0;
        // startMarker.type = visualization_msgs::Marker::SPHERE;
        // startMarker.action = visualization_msgs::Marker::ADD;
        // startMarker.pose.position.x = start[0];
        // startMarker.pose.position.y = start[1];
        // startMarker.pose.position.z = start[2];
        // startMarker.scale.x = 0.4;
        // startMarker.scale.y = 0.4;
        // startMarker.scale.z = 0.4;
        // startMarker.color.a = 0.7;
        // startMarker.color.r = 1.0;
        // startMarker.color.g = 0.5;
        // startMarker.color.b = 1.0;
        // startVisPub.publish(startMarker);

        // goalMarker.ns = "goal_vis";
        // goalMarker.id = 1;
        // goalMarker.type = visualization_msgs::Marker::SPHERE;
        // goalMarker.action = visualization_msgs::Marker::ADD;
        // goalMarker.pose.position.x = goal[0];
        // goalMarker.pose.position.y = goal[1];
        // goalMarker.pose.position.z = goal[2];
        // goalMarker.scale.x = 0.4;
        // goalMarker.scale.y = 0.4;
        // goalMarker.scale.z = 0.4;
        // goalMarker.color.a = 0.7;
        // goalMarker.color.r = 0.2;
        // goalMarker.color.g = 1.0;
        // goalMarker.color.b = 0.2;
        // goalVisPub.publish(goalMarker);

        // path.header.frame_id = "world";
        // path.header.stamp = ros::Time::now();
        // pathPub.publish(path);

        // cout << "[Planner Node]: Path points:" << endl;
        // for (size_t i = 0; i < path.poses.size(); ++i)
        // {
        //     cout << "Point " << i + 1 << ": ("
        //          << path.poses[i].pose.position.x << ", "
        //          << path.poses[i].pose.position.y << ", "
        //          << path.poses[i].pose.position.z << ")" << endl;
        // }

        // goalReceived = false;
        // r.sleep();
    }

    delete octree;
    return 0;
}
