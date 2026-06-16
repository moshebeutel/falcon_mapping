//
// Created by user1 on 6/15/26.
//
#include "tsdf_integrator.hpp"

namespace mapping
{

    TSDFIntegrator::TSDFIntegrator(float resolution, float trunc_dist,
                                   int sx,int sy,int sz)
    : resolution_(resolution), trunc_dist_(trunc_dist),
      size_x_(sx), size_y_(sy), size_z_(sz)
    {
        int N = sx*sy*sz;
        tsdf_.assign(N, 1.0f);
        weight_.assign(N, 0.0f);
    }

    void TSDFIntegrator::integratePointCloud(
        const std::vector<Eigen::Vector3f>& pts,
        const Eigen::Vector3f& cam)
    {
        float step = resolution_ * 0.5;

        for (const auto& p : pts) {

            Eigen::Vector3f dir = p - cam;
            float dist = dir.norm();
            dir.normalize();

            for (float t=0; t < dist + trunc_dist_; t+=step) {

                Eigen::Vector3f pos = cam + t*dir;

                int ix = int(pos.x()/resolution_) + size_x_/2;
                int iy = int(pos.y()/resolution_) + size_y_/2;
                int iz = int(pos.z()/resolution_);

                if (!valid(ix,iy,iz)) continue;

                int idx = idx3D(ix,iy,iz);

                float sdf = dist - t;
                sdf = std::max(-trunc_dist_, std::min(trunc_dist_, sdf));
                sdf /= trunc_dist_;

                float w_old = weight_[idx];
                float tsdf_old = tsdf_[idx];

                tsdf_[idx] = (w_old*tsdf_old + sdf) / (w_old + 1.0f);
                weight_[idx] = w_old + 1.0f;
            }
        }
    }

    const std::vector<float>& TSDFIntegrator::getTSDF() const { return tsdf_; }
    const std::vector<float>& TSDFIntegrator::getWeight() const { return weight_; }

    inline int TSDFIntegrator::idx3D(int x,int y,int z) const {
        return x + size_x_*(y + size_y_*z);
    }

    inline bool TSDFIntegrator::valid(int x,int y,int z) const {
        return x>=0 && y>=0 && z>=0 &&
               x<size_x_ && y<size_y_ && z<size_z_;
    }


} // mapping