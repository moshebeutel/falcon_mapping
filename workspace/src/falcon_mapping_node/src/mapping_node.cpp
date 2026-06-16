#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "falcon_mapping/esdf_integrator.hpp"
#include "falcon_mapping/tsdf_integrator.hpp"


namespace falcon_mapping
{
    class MappingNode : public rclcpp::Node {
    public:
        MappingNode() : Node("falcon_mapping_node"),
            tsdf_(0.1, 0.3, 200,200,80),
            esdf_(0.1, 200,200,80)
        {
            depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
                "/depth", 10,
                std::bind(&MappingNode::depthCB, this, std::placeholders::_1));

            occ_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map/occupancy",10);
        }

    private:
        TSDFIntegrator tsdf_;
        ESDFIntegrator esdf_;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_pub_;

        void depthCB(const sensor_msgs::msg::Image::SharedPtr msg) {

            auto pts = project(msg);
            tsdf_.integratePointCloud(pts, Eigen::Vector3f(0,0,0));
            esdf_.updateFromTSDF(tsdf_.getTSDF(), tsdf_.getWeight());

            publishOccupancy();
        }

        std::vector<Eigen::Vector3f> project(
            const sensor_msgs::msg::Image::SharedPtr& img)
        {
            std::vector<Eigen::Vector3f> pts;

            const float* data = reinterpret_cast<const float*>(img->data.data());

            for (int i=0;i<img->width*img->height;i++) {
                float d = data[i];
                if (d>0 && d<5.0)
                    pts.emplace_back(d,d,d); // placeholder projection
            }
            return pts;
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
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<falcon_mapping::MappingNode>());
    rclcpp::shutdown();
    return 0;
}