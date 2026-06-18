#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <Eigen/Dense>

class SimplePlanner : public rclcpp::Node
{
public:
    SimplePlanner() : Node("simple_planner")
    {
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map/esdf", 1,
            std::bind(&SimplePlanner::mapCB, this, std::placeholders::_1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&SimplePlanner::odomCB, this, std::placeholders::_1));

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&SimplePlanner::goalCB, this, std::placeholders::_1));

        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SimplePlanner::controlLoop, this));



        goal_ = Eigen::Vector2f(0.0f, 0.0f);  // hardcoded
    }

private:

    // ===== DATA =====
    nav_msgs::msg::OccupancyGrid map_;
    nav_msgs::msg::Odometry odom_;

    Eigen::Vector2f goal_;

    int size_x_, size_y_;
    float resolution_;

    // ===== ROS =====
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // ===== Callbacks =====

    void goalCB(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        goal_.x() = msg->pose.position.x;
        goal_.y() = msg->pose.position.y;

        RCLCPP_INFO(this->get_logger(),
            "New goal: %.2f %.2f", goal_.x(), goal_.y());
    }

    void mapCB(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_ = *msg;
        size_x_ = map_.info.width;
        size_y_ = map_.info.height;
        resolution_ = map_.info.resolution;
    }

    void odomCB(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_ = *msg;
    }

    // ===== Helpers =====

    bool worldToGrid(float x, float y, int &ix, int &iy)
    {
        ix = (x / resolution_) + size_x_ / 2;
        iy = (y / resolution_) + size_y_ / 2;

        return (ix >= 0 && ix < size_x_ &&
                iy >= 0 && iy < size_y_);
    }

    float getESDF(int x, int y)
    {
        int idx = y * size_x_ + x;
        return static_cast<float>(map_.data[idx]) / 20.0f;
    }

    Eigen::Vector2f computeGradient(int x, int y)
    {
        float dx = (getESDF(x+1, y) - getESDF(x-1, y)) / (2.0f * resolution_);
        float dy = (getESDF(x, y+1) - getESDF(x, y-1)) / (2.0f * resolution_);

        return Eigen::Vector2f(dx, dy);
    }

    // ===== CONTROL LOOP =====

    void controlLoop()
    {
        if (map_.data.empty()) return;

        if (goal_.isZero(1e-3)) return;  // no goal set

        auto &p = odom_.pose.pose.position;

        Eigen::Vector2f pos(p.x, p.y);

        // ===== Goal direction =====
        Eigen::Vector2f goal_dir = goal_ - pos;

        // RCLCPP_INFO(get_logger(), "Pos: (%.2f, %.2f), Goal: (%.2f, %.2f), Dist: %.2f",
        //             pos.x(), pos.y(), goal_.x(), goal_.y(), goal_dir.norm());


        float dist = goal_dir.norm();
        if (dist < 0.2)
        {
            stopRobot();
            return;
        }

        goal_dir.normalize();

        // ===== ESDF gradient =====
        int ix, iy;
        if (!worldToGrid(pos.x(), pos.y(), ix, iy)) return;

        if (ix <= 1 || iy <= 1 || ix >= size_x_-2 || iy >= size_y_-2)
            return;

        Eigen::Vector2f grad = computeGradient(ix, iy);

        // ===== obstacle avoidance =====
        // Eigen::Vector2f avoid = -grad;
        float dist_to_obs = getESDF(ix, iy);

        Eigen::Vector2f avoid(0, 0);

        float safe_dist = 0.5;  // meters

        if (dist_to_obs < safe_dist)
        {
            RCLCPP_INFO(get_logger(), "Too close to obstacle! Dist(ESDF): %.2f", dist_to_obs);
            RCLCPP_INFO(get_logger(), "ESDF Gradient: (%.2f, %.2f)", grad.x(), grad.y());
            RCLCPP_INFO(get_logger(), "ESDF direction: %.2f", atan2(goal_dir.y(), goal_dir.x()));
            RCLCPP_INFO(get_logger(), "Pos: (%.2f, %.2f), Goal: (%.2f, %.2f), Dist: %.2f",
                    pos.x(), pos.y(), goal_.x(), goal_.y(), goal_dir.norm());

            avoid = grad;

            if (avoid.norm() > 1e-3)
                avoid.normalize();

            float strength = (safe_dist - dist_to_obs) / safe_dist;

            avoid *= 2.0 * strength;  // stronger near obstacles
            RCLCPP_INFO(get_logger(), "Avoidance strength: %.2f", strength);
            RCLCPP_INFO(get_logger(), "Avoidance Vector: (%.2f, %.2f)", avoid.x(), avoid.y());
        }

        // normalize
        if (avoid.norm() > 1e-3)
            avoid.normalize();

        // ===== combine =====
        // Eigen::Vector2f dir = goal_dir + 1.5f * avoid;
        Eigen::Vector2f dir = goal_dir + avoid;

        if (dir.norm() > 1e-3)
            dir.normalize();

        // ===== convert to velocity =====
        geometry_msgs::msg::Twist cmd;

        float target_yaw = atan2(dir.y(), dir.x());

        auto &q = odom_.pose.pose.orientation;
        double yaw = atan2(2.0*(q.w*q.z + q.x*q.y),
                           1.0 - 2.0*(q.y*q.y + q.z*q.z));

        // RCLCPP_INFO(get_logger(), "Target yaw: %.2f, Current yaw: %.2f", target_yaw, yaw);

        float yaw_error = target_yaw - yaw;

        // RCLCPP_INFO(get_logger(), "Yaw error: %.2f", yaw_error);

        // normalize angle
        while (yaw_error > M_PI) yaw_error -= 2*M_PI;
        while (yaw_error < -M_PI) yaw_error += 2*M_PI;

        cmd.linear.x = 0.2;
        cmd.angular.z = 1.5 * yaw_error;

        // RCLCPP_INFO(get_logger(), "Cmd: linear=%.2f, angular=%.2f", cmd.linear.x, cmd.angular.z);

        cmd_pub_->publish(cmd);
    }

    void stopRobot()
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePlanner>());
    rclcpp::shutdown();
    return 0;
}