#include <functional>
#include <memory>
#include <cassert>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Dense>

#include "falcon_mapping/esdf_integrator.hpp"
#include "falcon_mapping/tsdf_integrator.hpp"


namespace falcon_mapping
{
    class MappingNode : public rclcpp::Node {
    public:
        MappingNode() : Node("falcon_mapping_node"),
            tsdf_integrator_(0.1, 0.3, 200,200,80),
            esdf_integrator_(0.1, 200,200,80)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initializing Falcon Mapping Node...");

            esdf_.resize(size_x_ * size_y_, 1e6f);
            occupied_.resize(size_x_ * size_y_, false);
            grid_.resize(size_x_ * size_y_, -1);
            // esdf_integrator_.updateFromTSDF(tsdf_integrator_.getTSDF(), tsdf_integrator_.getWeight());

            // publishOccupancy();

            scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
             rclcpp::SensorDataQoS(), std::bind(&MappingNode::scanCB, this, std::placeholders::_1));

            if(scan_sub_ == nullptr){
                printf("Failed to create scan subscription\n");
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to create scan subscription");
                throw std::runtime_error("Failed to create scan subscription");
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Subscribed to /scan topic");
            
            //depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
            //    "/depth", 10,
            //   std::bind(&MappingNode::depthCB, this, std::placeholders::_1));

            occ_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map/occupancy",10);

            esdf_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map/esdf", 1);

            odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
                [this](const nav_msgs::msg::Odometry::SharedPtr msg){current_odom_ = *msg;});

            map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1);
            if (map_pub_ == nullptr) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to create map publisher");
                throw std::runtime_error("Failed to create map publisher");
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Created map publisher");


        }

    private:
        TSDFIntegrator tsdf_integrator_;
        ESDFIntegrator esdf_integrator_;
        
        // ROS   pub sub
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_pub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr esdf_pub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
        
        
        
        
        
        // grid storage
        std::vector<int8_t> grid_;   // occupancy (0 free, 100 occupied, -1 unknown)

        // odom
        nav_msgs::msg::Odometry current_odom_;


        // ESDF / occupancy grid
        int size_x_ = 200;
        int size_y_ = 200;
        float resolution_ = 0.05f;

        std::vector<float> esdf_;
        std::vector<bool> occupied_;

        inline bool worldToGrid(float x, float y, int &ix, int &iy)
        {
            ix = static_cast<int>(x / resolution_ + size_x_ / 2);
            iy = static_cast<int>(y / resolution_ + size_y_ / 2);

            return (ix >= 0 && ix < size_x_ && iy >= 0 && iy < size_y_);
        }

        void traceRay(const Eigen::Vector3f& start,
              const Eigen::Vector3f& end)
        {
            Eigen::Vector2f s(start.x(), start.y());
            Eigen::Vector2f e(end.x(), end.y());

            Eigen::Vector2f dir = e - s;
            float length = dir.norm();

            if (length < 1e-3) return;

            dir.normalize();

            float step = resolution_ * 0.5;

            for (float d = 0; d < length; d += step)
            {
                Eigen::Vector2f p = s + dir * d;

                int ix, iy;
                if (!worldToGrid(p.x(), p.y(), ix, iy)) continue;

                int idx = iy * size_x_ + ix;

                if (grid_[idx] != 100)  // do NOT overwrite obstacles
                {
                    grid_[idx] = 0;  // free space
                }
            }
        }
        

        void updateWorldMapClearByRayTrace(std::vector<Eigen::Vector3f> pts)
        {
            const auto& p = current_odom_.pose.pose.position;
            const auto& q = current_odom_.pose.pose.orientation;

            Eigen::Quaternionf quat(q.w, q.x, q.y, q.z);
            Eigen::Matrix3f R = quat.toRotationMatrix();
            Eigen::Vector3f t(p.x, p.y, p.z);
            Eigen::Vector3f robot_pos = t;


            for (auto& pt : pts)
            {
                
                Eigen::Vector3f pw = R * pt + t;
                 
                // mark free cells along ray from robot to point
                traceRay(robot_pos, pw);
                
                // mark occupied cell at point
                int ix, iy;
                if (worldToGrid(pw.x(), pw.y(), ix, iy))
                {
                    const int idx = iy * size_x_ + ix;
                    grid_[idx] = 100;  // occupied
                }
            }
        }


        void updateWorldMap(std::vector<Eigen::Vector3f> pts)
        {
            const auto& p = current_odom_.pose.pose.position;
            const auto& q = current_odom_.pose.pose.orientation;

            Eigen::Quaternionf quat(q.w, q.x, q.y, q.z);
            Eigen::Matrix3f R = quat.toRotationMatrix();
            Eigen::Vector3f t(p.x, p.y, p.z);
            

            for (auto& pt : pts)
            {
                
                Eigen::Vector3f pw = R * pt + t;

                int ix = static_cast<int>(pw.x() / resolution_ + size_x_ / 2);
                int iy = static_cast<int>(pw.y() / resolution_ + size_y_ / 2);

                if (ix >= 0 && ix < size_x_ && iy >= 0 && iy < size_y_)
                {
                    grid_[iy * size_x_ + ix] = 100;  // occupied
                }

            }
        }

        void scanCB(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

            RCLCPP_INFO(this->get_logger(), "Received LaserScan with %zu ranges", msg->ranges.size());

            auto pts = project(msg);

            RCLCPP_INFO(this->get_logger(), "Projected %zu points from LaserScan", pts.size());

            // updateOccupiedGrid(pts);

            // RCLCPP_INFO(this->get_logger(), "Updated occupied grid with %zu points", pts.size());

            // computeESDFNaive();

            // RCLCPP_INFO(this->get_logger(), "Computed ESDF from occupied grid");

            // publishESDF();

            // RCLCPP_INFO(this->get_logger(), "Published ESDF");

            // tsdf_integrator_.integratePointCloud(pts, Eigen::Vector3f(0,0,0));
            // esdf_integrator_.updateFromTSDF(tsdf_integrator_.getTSDF(), tsdf_integrator_.getWeight());

            // publishOccupancy();

            // updateWorldMap(pts);
            updateWorldMapClearByRayTrace(pts);

            publishMap();

            updateOccupiedESDFUsingMapGrid();

            computeESDF();

            publishESDF();

        
        }


        void updateOccupiedESDFUsingMapGrid() {
            for (size_t i = 0; i < grid_.size(); ++i)
            {
                occupied_[i] = (grid_[i] == 100);
                esdf_[i] = occupied_[i] ? 0.0f : 1e6f;
            }
        }

        void updateOccupiedGrid(const std::vector<Eigen::Vector3f>& pts) {
            for (const auto& p : pts)
            {
                float x = p.x();
                float y = p.y();

                int ix = static_cast<int>(x / resolution_ + size_x_/2);
                int iy = static_cast<int>(y / resolution_ + size_y_/2);

                if (ix >= 0 && ix < size_x_ && iy >= 0 && iy < size_y_)
                {
                    occupied_[iy * size_x_ + ix] = true;
                    esdf_[iy * size_x_ + ix] = 0.0f;
                }
            }
        }

        void computeESDF() 
        {
            auto idx = [&](int x, int y) {
                return y * size_x_ + x;
            };

            float d1 = resolution_;           // straight step
            float d2 = resolution_ * 1.414f;  // diagonal

            // ===== FORWARD PASS =====
            for (int y = 0; y < size_y_; ++y)
            {
                for (int x = 0; x < size_x_; ++x)
                {
                    int i = idx(x, y);

                    if (esdf_[i] == 0.0f) continue;

                    float min_d = esdf_[i];

                    if (x > 0)
                        min_d = std::min(min_d, esdf_[idx(x-1, y)] + d1);

                    if (y > 0)
                        min_d = std::min(min_d, esdf_[idx(x, y-1)] + d1);

                    if (x > 0 && y > 0)
                        min_d = std::min(min_d, esdf_[idx(x-1, y-1)] + d2);

                    if (x < size_x_-1 && y > 0)
                        min_d = std::min(min_d, esdf_[idx(x+1, y-1)] + d2);

                    esdf_[i] = min_d;
                }
            }

    // ===== BACKWARD PASS =====
    for (int y = size_y_ - 1; y >= 0; --y)
    {
        for (int x = size_x_ - 1; x >= 0; --x)
        {
            int i = idx(x, y);

            float min_d = esdf_[i];

            if (x < size_x_-1)
                min_d = std::min(min_d, esdf_[idx(x+1, y)] + d1);

            if (y < size_y_-1)
                min_d = std::min(min_d, esdf_[idx(x, y+1)] + d1);

            if (x < size_x_-1 && y < size_y_-1)
                min_d = std::min(min_d, esdf_[idx(x+1, y+1)] + d2);

            if (x > 0 && y < size_y_-1)
                min_d = std::min(min_d, esdf_[idx(x-1, y+1)] + d2);

            esdf_[i] = min_d;
        }
    }
}


        void computeESDFNaive()
        {
            for (int y = 0; y < size_y_; ++y)
            {
                for (int x = 0; x < size_x_; ++x)
                {
                    if (occupied_[y * size_x_ + x]) continue;

                    float min_dist = 1e6;

                    for (int yy = 0; yy < size_y_; ++yy)
                    {
                        for (int xx = 0; xx < size_x_; ++xx)
                        {
                            if (!occupied_[yy * size_x_ + xx]) continue;

                            float dx = (x - xx) * resolution_;
                            float dy = (y - yy) * resolution_;

                            float d = std::sqrt(dx*dx + dy*dy);
                            min_dist = std::min(min_dist, d);
                        }
                    }

                    esdf_[y * size_x_ + x] = min_dist;
                }
            }
        }

        void publishESDF()
        {
            nav_msgs::msg::OccupancyGrid msg;

            msg.header.frame_id = "map";
            msg.info.resolution = resolution_;
            msg.info.width = size_x_;
            msg.info.height = size_y_;

            msg.info.origin.position.x = - (size_x_ * resolution_) / 2;
            msg.info.origin.position.y = - (size_y_ * resolution_) / 2;

            msg.data.resize(size_x_ * size_y_);

            for (size_t i = 0; i < esdf_.size(); ++i)
            {
                float d = esdf_[i];

                // clamp for visualization
                int val = std::min(100, static_cast<int>(d * 20));
                msg.data[i] = val;
            }

            esdf_pub_->publish(msg);
        }

        // void publishESDF()
        // {
        //     nav_msgs::msg::OccupancyGrid msg;

        //     msg.info.resolution = resolution_;
        //     msg.info.width = size_x_;
        //     msg.info.height = size_y_;

        //     msg.data.resize(size_x_ * size_y_);

        //     for (size_t i = 0; i < esdf_.size(); ++i)
        //     {
        //         float d = esdf_[i];

        //         int val = std::min(100, static_cast<int>(d * 20));
        //         msg.data[i] = val;
        //     }

        //     esdf_pub_->publish(msg);
        // }

        std::vector<Eigen::Vector3f> project(
            const sensor_msgs::msg::LaserScan::SharedPtr& scan)
        {
            std::vector<Eigen::Vector3f> pts;

            float angle = scan->angle_min;

            for (size_t i = 0; i < scan->ranges.size(); ++i)
            {
                float r = scan->ranges[i];
            
                if (std::isfinite(r) && r > scan->range_min && r < scan->range_max)
                {   
                    float x = r * std::cos(angle);
                    float y = r * std::sin(angle);
                    float z = 0.0f;  // Laser scan is 2D plane
                    pts.emplace_back(x, y, z);
                }

                angle += scan->angle_increment;
            }

            return pts;
        }



        // void scanCB(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        // {
        //     std::vector<Eigen::Vector3f> pts;

        //     float angle = msg->angle_min;

        //     for (float r : msg->ranges)
        //     {
        //         if (std::isfinite(r))
        //         {
        //             float x = r * cos(angle);
        //             float y = r * sin(angle);
        //             float z = 0.0f;   // planar LiDAR

        //             pts.emplace_back(x, y, z);
        //         }
        //         angle += msg->angle_increment;
        //     }

        //     tsdf_integrator_.integratePointCloud(pts, Eigen::Vector3f(0,0,0));
        //     esdf_integrator_.updateFromTSDF(tsdf_integrator_.getTSDF(), tsdf_integrator_.getWeight());

        //     publishOccupancy();
        // }
        void publishMap()
        {
            nav_msgs::msg::OccupancyGrid msg;

            msg.header.frame_id = "map";
            msg.info.resolution = resolution_;
            msg.info.width = size_x_;
            msg.info.height = size_y_;

            msg.info.origin.position.x = - (size_x_ * resolution_) / 2;
            msg.info.origin.position.y = - (size_y_ * resolution_) / 2;
            
            msg.data = grid_;
            msg.data.resize(size_x_ * size_y_, -1);
            
            if (!map_pub_) {
                RCLCPP_ERROR(this->get_logger(), "map publisher not created");
                throw std::runtime_error("map publisher not created");
            }

            map_pub_->publish(msg);

            RCLCPP_INFO(this->get_logger(), "Published map with %zu cells", grid_.size());
        }

        void publishOccupancy() {
            nav_msgs::msg::OccupancyGrid grid;
            grid.info.resolution = 0.1;
            grid.info.width = 200;
            grid.info.height = 200;

            grid.data.resize(200*200,0);
            occ_pub_->publish(grid);
        }
    };


}

int main(int argc, char** argv) {
    printf("Starting Falcon Mapping Node...\n");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting Falcon Mapping Node...");
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initialized ROS2, spinning node...");
    rclcpp::spin(std::make_shared<falcon_mapping::MappingNode>());
    rclcpp::shutdown();
    return 0;
}