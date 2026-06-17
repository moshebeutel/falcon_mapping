//
// Created by user1 on 6/15/26.
//

#pragma once
#include <vector>
#include <eigen3/Eigen/Dense>
namespace falcon_mapping
{
    class ESDFIntegrator
    {
    public:
        ESDFIntegrator(float resolution,
                       int size_x, int size_y, int size_z);

        void updateFromTSDF(const std::vector<float>& tsdf,
                    const std::vector<float>& weight);

        float getDistance(const Eigen::Vector3f& pos) const;


    private:
        float resolution_;
        int size_x_, size_y_, size_z_;
        std::vector<float> esdf_;

        inline int idx3D(int x,int y,int z) const;
        inline bool valid(int x,int y,int z) const;


    };
} // mapping

