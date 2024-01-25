#include "pch.h"

#include "Core/Core.h"
#include "knn/knn.h"
#include "QEM/QEM.h"

namespace VSR {

    class RegionGrow
    {
    public:
        CGAL::Point_set_3<CGAL::Point, CGAL::Vector> m_vPointCloud;
        QEM m_qem;
        KNN m_knn;
        float lambda;

        // max generator time
        int m_iMaxGeneratorTime = 6;

        // labels of each point
        std::vector<int> m_vLables;
        std::vector<int> m_vLastLables;

        bool m_bStable = false;

        // first point is generator, second point is related point index
        std::vector<std::pair<CGAL::Point, int>> m_vSeeds;
        // quadric of each generator
        std::vector<Eigen::Matrix4f> m_vQuadric;

        // grow queue
        struct QueueNode {
            int label;
            int pointIndex;
            float QEMdistance;
            bool operator<(const QueueNode& rhs) const {
                return QEMdistance > rhs.QEMdistance;
            }
        };
        std::priority_queue<QueueNode> m_qGrowQueue;
    public:
        RegionGrow();
        ~RegionGrow();

        RegionGrow(CGAL::Point_set_3<CGAL::Point, CGAL::Vector> pointCloud, 
            std::vector<std::pair<CGAL::Point, int>> seeds, 
            QEM qem, KNN knn);
        void SetPointCloud(CGAL::Point_set_3<CGAL::Point, CGAL::Vector> pointCloud, 
            std::vector<std::pair<CGAL::Point, int>> seeds,
            QEM qem, KNN knn);

        void Grow();
        void SeedsUpdate();

        static std::vector<std::pair<CGAL::Point, int>> GenerateSeeds(CGAL::Point_set_3<CGAL::Point, CGAL::Vector> pointSet, int seedNum);

        inline int GetLabelCount() { return m_vSeeds.size(); }
    private:
        float QEMDistance(int pointIndex, int label);
    };
}