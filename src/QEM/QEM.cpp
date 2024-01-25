#include "QEM/QEM.h"

namespace VSR {

    QEM::QEM() {
        
    }

    QEM::~QEM() {

    }

    QEM::QEM(CGAL::Point_set_3<CGAL::Point, CGAL::Vector> pointCloud, KNN knn) 
        : m_oPointCloud(pointCloud), m_knn(knn)
    {
        ComputePlaneQuadric();
        ComputeDiffusedQuadric();
    }

    void QEM::SetPointCloud(CGAL::Point_set_3<CGAL::Point, CGAL::Vector> pointCloud, KNN knn) {
        m_oPointCloud = pointCloud;
        m_knn = knn;
        ComputePlaneQuadric();
        ComputeDiffusedQuadric();
    }

    void QEM::ComputePlaneQuadric() {
        m_vPlaneQuadric.clear();
        m_vPlaneQuadric.resize(m_oPointCloud.size());

        for (int i = 0; i < m_oPointCloud.size(); i++) {
            m_vPlaneQuadric[i] = Eigen::Matrix4f::Zero();
        }

        for (int i = 0; i < m_oPointCloud.size(); i++) {
            CGAL::Point point = m_oPointCloud.point(i);
            CGAL::Vector normal = m_oPointCloud.normal(i);

            Eigen::Vector4f plane(
                CGAL::to_double(normal.x()), 
                CGAL::to_double(normal.y()), 
                CGAL::to_double(normal.z()), 
                - CGAL::to_double(normal.x() * point.x() - normal.y() * point.y() - normal.z() * point.z())
            );
            m_vPlaneQuadric[i] = plane * plane.transpose();
        }
    }

    void QEM::ComputeDiffusedQuadric() {
        // Compute support area
        m_vSupportArea.resize(m_oPointCloud.size());
        for(int i = 0; i < m_oPointCloud.size(); i++) {
            m_vSupportArea[i] = 0;
            for(int j = 0; j < m_iK; j++) {
                m_vSupportArea[i] += m_knn.Neighbors[i][j].second;
            }
            m_vSupportArea[i] = m_vSupportArea[i] * m_vSupportArea[i];
            m_vSupportArea[i] /= 2 * m_iK * m_iK;
        }

        // Compute diffused quadric
        m_vDiffusedQuadric.clear();
        m_vDiffusedQuadric.resize(m_oPointCloud.size());
        for(int i = 0; i < m_oPointCloud.size(); i++) {
            m_vDiffusedQuadric[i] = Eigen::Matrix4f::Zero();
            for(int j = 0; j < m_iK; j++) {
                m_vDiffusedQuadric[i] += m_vSupportArea[m_knn.Neighbors[i][j].first] * m_vPlaneQuadric[m_knn.Neighbors[i][j].first];
            }
        }

    }

}