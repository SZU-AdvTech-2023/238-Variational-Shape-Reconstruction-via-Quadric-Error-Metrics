#pragma once

#include "core/core.h"

namespace VSR {
    class KNN
    {
    public:
        std::vector<std::vector<std::pair<int, float>>> Neighbors;
        CGAL::Point_set_3<CGAL::Point, CGAL::Vector> m_oPointCloud;

        int K;
    public:
        KNN();
        ~KNN();

        KNN(CGAL::Point_set_3<CGAL::Point, CGAL::Vector> pointCloud, int k = 6);
        void SetPointCloud(CGAL::Point_set_3<CGAL::Point, CGAL::Vector> pointCloud, int k = 6);

        int SearchClosetPoint(CGAL::Point point);
    };
    
}