#include <ros/ros.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

class TurtleController {
public:
    TurtleController(ros::NodeHandle &nh, 
                   const std::string &leader_name,
                   const std::string &self_name,
                   const float target_x = 10.5, 
                   const float target_y = 1.5) 
        : nh_(nh), leader_name_(leader_name), self_name_(self_name),
          target_x_(target_x), target_y_(target_y) {
        
        // 初始化随机种子
        srand(time(NULL));
        
        // 设置发布者和订阅者
        if (self_name == "turtle1") {
            cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(self_name_ + "/cmd_vel", 10);
            pose_sub_ = nh_.subscribe(self_name_ + "/pose", 10, &TurtleController::poseCallback, this);
        } else {
            cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(self_name_ + "/cmd_vel", 10);
            leader_pose_sub_ = nh_.subscribe(leader_name_ + "/pose", 10, &TurtleController::leaderPoseCallback, this);
            self_pose_sub_ = nh_.subscribe(self_name_ + "/pose", 10, &TurtleController::selfPoseCallback, this);
        }
    }

    void spawn() {
        if (self_name_ == "turtle1") return;

        ros::service::waitForService("/spawn");
        ros::ServiceClient spawnClient = nh_.serviceClient<turtlesim::Spawn>("/spawn");
        turtlesim::Spawn srv;
        
        srv.request.x = rand() % 8 + 2;  // 2-10之间的随机坐标
        srv.request.y = rand() % 8 + 2;
        srv.request.theta = 0;
        srv.request.name = self_name_;
        
        if (spawnClient.call(srv)) {
            ROS_INFO("Spawned % (%f, %f)", 
                    self_name_.c_str(), srv.request.x, srv.request.y);
        }
    }

private:
    void poseCallback(const turtlesim::Pose::ConstPtr& msg) {
        // 只有turtle1需要主动移动到目标点
        if (self_name_ != "turtle1") return;

        float dx = target_x_ - msg->x;
        float dy = target_y_ - msg->y;
        float distance = sqrt(dx*dx + dy*dy);
        float target_angle = atan2(dy, dx);
        float angle_diff = target_angle - msg->theta;

        // 角度修正
        if (angle_diff > M_PI) angle_diff -= 2*M_PI;
        else if (angle_diff < -M_PI) angle_diff += 2*M_PI;

        geometry_msgs::Twist cmd;
        if (distance > 0.1) {
            cmd.linear.x = 1.5 * distance;
            cmd.angular.z = 4.0 * angle_diff;
        } else {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
        }
        cmd_vel_pub_.publish(cmd);
    }

    void leaderPoseCallback(const turtlesim::Pose::ConstPtr& msg) {
        leader_x_ = msg->x;
        leader_y_ = msg->y;
        has_leader_pose_ = true;
    }

    void selfPoseCallback(const turtlesim::Pose::ConstPtr& msg) {
        if (!has_leader_pose_) return;

        float dx = leader_x_ - msg->x;
        float dy = leader_y_ - msg->y;
        float distance = sqrt(dx*dx + dy*dy);
        float target_angle = atan2(dy, dx);
        float angle_diff = target_angle - msg->theta;

        // 角度修正
        if (angle_diff > M_PI) angle_diff -= 2*M_PI;
        else if (angle_diff < -M_PI) angle_diff += 2*M_PI;

        geometry_msgs::Twist cmd;
        if (distance > 0.1) {
            cmd.linear.x = 1.2 * distance;
            cmd.angular.z = 3.5 * angle_diff;
        } else {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
        }
        cmd_vel_pub_.publish(cmd);
    }

    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber leader_pose_sub_;
    ros::Subscriber self_pose_sub_;
    std::string leader_name_;
    std::string self_name_;
    float target_x_;
    float target_y_;
    float leader_x_ = 0;
    float leader_y_ = 0;
    bool has_leader_pose_ = false;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtle_follower");
    ros::NodeHandle nh;

    // 创建三个海龟控制器
    TurtleController turtle1(nh, "", "turtle1");
    TurtleController turtle2(nh, "turtle1", "turtle2");
    TurtleController turtle3(nh, "turtle2", "turtle3");

    // 生成新海龟
    turtle2.spawn();
    turtle3.spawn();

    ros::spin();
    return 0;
}
