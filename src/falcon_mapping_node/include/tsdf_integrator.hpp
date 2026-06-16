//
// Created by user1 on 6/15/26.
//
#pragma once
#include <vector>
#include <eigen3/Eigen/Dense>
namespace mapping
{
    class TSDFIntegrator
    {
    public:
        TSDFIntegrator(float resolution, float trunc_dist,
                       int size_x, int size_y, int size_z);

        void integratePointCloud(const std::vector<Eigen::Vector3f>& pts,
                                 const Eigen::Vector3f& cam);

        const std::vector<float>& getTSDF() const;
        const std::vector<float>& getWeight() const;

    private:
        float resolution_, trunc_dist_;
        int size_x_, size_y_, size_z_;

        std::vector<float> tsdf_;
        std::vector<float> weight_;

        inline int idx3D(int x,int y,int z) const;
        inline bool valid(int x,int y,int z) const;
    };

} // mapping

