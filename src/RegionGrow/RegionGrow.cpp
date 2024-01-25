#include "pch.h"

#include "RegionGrow/RegionGrow.h"

namespace VSR {

    RegionGrow::RegionGrow() {
        m_vLables = std::vector<int>();
    }

    RegionGrow::~RegionGrow() {
    }

    RegionGrow::RegionGrow(CGAL::Point_set_3<CGAL::Point, CGAL::Vector> pointCloud, 
            std::vector<std::pair<CGAL::Point, int>> seeds, 
            QEM qem, KNN knn) {
        SetPointCloud(pointCloud, seeds, qem, knn);
    }

    void RegionGrow::SetPointCloud(CGAL::Point_set_3<CGAL::Point, CGAL::Vector> pointCloud, 
            std::vector<std::pair<CGAL::Point, int>> seeds, 
            QEM qem, KNN knn) {
        // Init function
        m_qem = qem;
        m_knn = knn;
        m_vPointCloud = pointCloud;
        m_vSeeds = seeds;
        lambda = 10e-5;

        m_vLastLables.clear();
        m_vLastLables.resize(m_vPointCloud.size());
        for (int i = 0; i < m_vLastLables.size(); i++) {
            m_vLastLables[i] = -1;
        }
    }

    void RegionGrow::Grow() {
        // Init quadric
        m_vQuadric.clear();
        m_vQuadric.resize(m_vSeeds.size());
        for (int i = 0; i < m_vQuadric.size(); i++) {
            m_vQuadric[i] = Eigen::Matrix4f::Zero();
        }
        
        // Init labels
        m_vLables.clear();
        m_vLables.resize(m_vPointCloud.size());
        for (int i = 0; i < m_vLables.size(); i++) {
            m_vLables[i] = -1;
        }

        // Init queue with seeds
        for (int i = 0; i < m_vSeeds.size(); i++) {
            // i is the label index
            m_qGrowQueue.push({i, m_vSeeds[i].second, QEMDistance(m_vSeeds[i].second, i)});
        }

        // start grow
        bool stable = true;
        while (!m_qGrowQueue.empty()) {
            auto node = m_qGrowQueue.top();
            m_qGrowQueue.pop();

            // skip if point had a label
            if (m_vLables[node.pointIndex] != -1) {
                continue;
            }

            // set label
            m_vLables[node.pointIndex] = node.label;

            // confirm stable set false if found a different label
            if (m_vLastLables[node.pointIndex] != m_vLastLables[node.pointIndex]) {
                stable = false;
            } 

            // update Quadric
            Eigen::Matrix4f Q = m_qem.GetDiffusedQuadric()[node.pointIndex];
            m_vQuadric[node.label] += Q;

            auto neighbors = m_knn.Neighbors[node.pointIndex];
            for (auto neighbor : neighbors) {
                if (m_vLables[neighbor.first] == -1) {
                    m_qGrowQueue.push({node.label, neighbor.first, QEMDistance(neighbor.first, node.label)});
                }
            }
        }
        m_bStable = stable;

        // copy labels to last labels
        m_vLastLables = m_vLables;
    }

    void RegionGrow::SeedsUpdate() {
        for(int i = 0; i < m_vSeeds.size(); i++) {
            auto Q = m_vQuadric[i];

            Eigen::Matrix4f A;
            A << Q(0, 0), Q(0, 1), Q(0, 2), Q(0, 3),
                 Q(0, 1), Q(1, 1), Q(1, 2), Q(1, 3),
                 Q(0, 2), Q(1, 2), Q(2, 2), Q(2, 3),
                 0,       0,       0,       1;
            Eigen::Vector4f b = A.row(3);
            Eigen::Vector4f optim;

            Eigen::FullPivLU<Eigen::MatrixXf> lu_decomp(A);
            if(lu_decomp.isInvertible()) {
                optim = lu_decomp.inverse() * b;
            } else {
                Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
                svd.setThreshold(1e-5);
                optim = svd.solve(b);
            }

            m_vSeeds[i].first = CGAL::Point(optim.x(), optim.y(), optim.z());
            // update related point

            m_vSeeds[i].second = m_knn.SearchClosetPoint(m_vSeeds[i].first);

        }
    }

    float RegionGrow::QEMDistance(int pointIndex, int label) {
        float distance = 0;
        auto cj = m_vSeeds[label].first; 
        auto Q = m_qem[pointIndex];

        Eigen::Vector4f c = Eigen::Vector4f(CGAL::to_double(cj.x()), CGAL::to_double(cj.y()), CGAL::to_double(cj.z()), 1);
        distance = c.transpose() * Q * c;

        auto d = m_vPointCloud.point(pointIndex) - cj;
        distance += lambda * CGAL::to_double(d.squared_length());

        return distance;
    }

    std::vector<std::pair<CGAL::Point, int>> RegionGrow::GenerateSeeds(CGAL::Point_set_3<CGAL::Point, CGAL::Vector> pointSet, int seedNum) {
        std::vector<std::pair<CGAL::Point, int>> seeds;
        for (int i = 0; i < seedNum; i++) {
            int index = rand() % pointSet.size();
            auto point = pointSet.point(index);
            seeds.push_back({point, index});
        }
        return seeds;
    }

}