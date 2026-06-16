#include "falcon_mapping/esdf_integrator.hpp"
#include <queue>
#include <cmath>



namespace falcon_mapping
{
    ESDFIntegrator::ESDFIntegrator(float res,int sx,int sy,int sz)
    : resolution_(res), size_x_(sx), size_y_(sy), size_z_(sz)
    {
        esdf_.resize(sx*sy*sz);
    }

    void ESDFIntegrator::updateFromTSDF(
        const std::vector<float>& tsdf,
        const std::vector<float>& weight)
    {
        const float INF = 1e6;
        std::queue<int> q;

        for (int i=0;i<tsdf.size();i++) {
            if (weight[i] < 1e-3) {
                esdf_[i] = INF;
                continue;
            }

            if (fabs(tsdf[i]) < 0.1f) {
                esdf_[i] = 0.0;
                q.push(i);
            } else {
                esdf_[i] = INF;
            }
        }

        int dx[6] = {1,-1,0,0,0,0};
        int dy[6] = {0,0,1,-1,0,0};
        int dz[6] = {0,0,0,0,1,-1};

        while(!q.empty()) {
            int idx = q.front(); q.pop();

            int x = idx % size_x_;
            int y = (idx / size_x_) % size_y_;
            int z = idx / (size_x_ * size_y_);

            for(int k=0;k<6;k++) {

                int nx=x+dx[k], ny=y+dy[k], nz=z+dz[k];
                if(!valid(nx,ny,nz)) continue;

                int nidx = idx3D(nx,ny,nz);

                float nd = esdf_[idx] + resolution_;

                if (nd < esdf_[nidx]) {
                    esdf_[nidx] = nd;
                    q.push(nidx);
                }
            }
        }

        // signed
        for (int i=0;i<tsdf.size();i++)
            if (tsdf[i] < 0)
                esdf_[i] = -esdf_[i];
    }

    float ESDFIntegrator::getDistance(const Eigen::Vector3f& p) const {

        int x = int(p.x()/resolution_) + size_x_/2;
        int y = int(p.y()/resolution_) + size_y_/2;
        int z = int(p.z()/resolution_);

        if (!valid(x,y,z)) return 1e6;

        return esdf_[idx3D(x,y,z)];
    }

    inline int ESDFIntegrator::idx3D(int x,int y,int z) const {
        return x + size_x_*(y + size_y_*z);
    }

    inline bool ESDFIntegrator::valid(int x,int y,int z) const {
        return x>=0 && y>=0 && z>=0 &&
               x<size_x_ && y<size_y_ && z<size_z_;
    }
}