#pragma once

#include "pch.h"

#include "core/core.h"
#include "knn/knn.h"

#include "CGAL/Search_traits_3.h"
#include "CGAL/Search_traits_adapter.h"
#include "CGAL/k_neighbor_search.h"
#include "CGAL/Kd_tree.h"
#include "boost/tuple/tuple.hpp"
#include "boost/iterator/zip_iterator.hpp"

namespace VSR
{

    // QEM method to compute the quadric error metric
    class QEM
    {

    private:
        CGAL::Point_set_3<CGAL::Point, CGAL::Vector> m_oPointCloud;

        std::vector<Eigen::Matrix4f> m_vPlaneQuadric;
        std::vector<Eigen::Matrix4f> m_vDiffusedQuadric;
        std::vector<float> m_vSupportArea;

        KNN m_knn;
        int m_iK = 6;
    public:
        QEM();
        ~QEM();
        QEM(CGAL::Point_set_3<CGAL::Point, CGAL::Vector> pointCloud, KNN knn);
        void SetPointCloud(CGAL::Point_set_3<CGAL::Point, CGAL::Vector> pointCloud, KNN knn);

        inline std::vector<float>& GetSupportArea() { return m_vSupportArea; }
        inline std::vector<Eigen::Matrix4f>& GetPlaneQuadric() { return m_vPlaneQuadric; }
        inline std::vector<Eigen::Matrix4f>& GetDiffusedQuadric() { return m_vDiffusedQuadric; }

        friend std::ostream& operator <<(std::ostream& os, const QEM& qem) {
            for(auto& quadric : qem.m_vPlaneQuadric) {
                os << quadric << std::endl;
            }
            return os;
        }

        inline Eigen::Matrix4f operator[](int index) {
            return m_vDiffusedQuadric[index];
        }
    private:
        void ComputePlaneQuadric();
        void ComputeDiffusedQuadric();
    };

}