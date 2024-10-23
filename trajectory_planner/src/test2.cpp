#include <ros/ros.h>
#include <global_planner/rrtStarOctomap.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_planner/polyTrajOctomap.h>
#include <iostream>
#include <thread>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Simple_Path_Planner");
    ros::NodeHandle nh;

    // Khai báo Publisher
    ros::Publisher posePub = nh.advertise<geometry_msgs::PoseStamped>("/trajectory_pose", 1000);
    ros::Publisher startVisPub = nh.advertise<visualization_msgs::Marker>("/start_position", 1000);
    ros::Publisher goalVisPub = nh.advertise<visualization_msgs::Marker>("/goal_position", 1000);

    const int N = 3; // số chiều
    globalPlanner::rrtStarOctomap<N> rrtStarPlanner(nh);

    std::vector<double> start(3);
    std::vector<double> goal(3);
    ros::Rate r(10);

    while (ros::ok())
    {
        // Nhập điểm start
        cout << "Nhập tọa độ điểm start (x y z): ";
        cin >> start[0] >> start[1] >> start[2];
        rrtStarPlanner.updateStart(start);

        // Visualization cho điểm start
        visualization_msgs::Marker startMarker;
        startMarker.header.frame_id = "map";
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

        // Nhập điểm goal
        cout << "Nhập tọa độ điểm goal (x y z): ";
        cin >> goal[0] >> goal[1] >> goal[2];
        rrtStarPlanner.updateGoal(goal);

        // Visualization cho điểm goal
        visualization_msgs::Marker goalMarker;
        goalMarker.header.frame_id = "map";
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

        // Tạo đường đi từ start đến goal
        nav_msgs::Path path;
        rrtStarPlanner.makePlan(path);

        // In ra các điểm trên path
        cout << "[Planner Node]: Path points:" << endl;
        for (size_t i = 0; i < path.poses.size(); ++i)
        {
            cout << "Point " << i + 1 << ": ("
                 << path.poses[i].pose.position.x << ", "
                 << path.poses[i].pose.position.y << ", "
                 << path.poses[i].pose.position.z << ")" << endl;
        }

        // Di chuyển UAV qua các điểm trong path
        for (const auto &pose : path.poses)
        {
            posePub.publish(pose);      // Xuất bản từng điểm
            ros::Duration(0.1).sleep(); // Tạm dừng giữa các điểm để UAV di chuyển
        }
    }
    return 0;
}
