#include "pch.h"

#include "knn/knn.h"

#include "CGAL/Search_traits_3.h"
#include "CGAL/Search_traits_adapter.h"
#include "CGAL/k_neighbor_search.h"
#include "CGAL/Kd_tree.h"
#include "boost/tuple/tuple.hpp"
#include "boost/iterator/zip_iterator.hpp"

namespace VSR {
    
    KNN::KNN() {

    }
    
    KNN::~KNN() {

    }

    KNN::KNN(CGAL::Point_set_3<CGAL::Point, CGAL::Vector> pointCloud, int k) {
        SetPointCloud(pointCloud, k);
    }

    void KNN::SetPointCloud(CGAL::Point_set_3<CGAL::Point, CGAL::Vector> pointCloud, int k) {

        typedef boost::tuple<CGAL::Point, int>                  PointAndIndex;
        typedef CGAL::Search_traits_3<CGAL::Kernel>             Traits_base;
        typedef CGAL::Search_traits_adapter<PointAndIndex, 
            CGAL::Nth_of_tuple_property_map<0, PointAndIndex>, 
            Traits_base>                                        Traits;
        typedef CGAL::K_neighbor_search<Traits>::Tree Tree;

        m_oPointCloud = pointCloud;
        K = k;

        Neighbors.clear();
        Neighbors.resize(m_oPointCloud.size());
        for(int i = 0; i < m_oPointCloud.size(); i++) {
            Neighbors[i].resize(k);
        }

        // Compute K neighbors
        // pair of index and distance
        std::vector<int> indices(m_oPointCloud.size());
        for (int i = 0; i < m_oPointCloud.size(); i++) {
            indices[i] = i;
        }

        Tree tree(
            boost::make_zip_iterator(boost::make_tuple(m_oPointCloud.points().begin(), indices.begin())),
            boost::make_zip_iterator(boost::make_tuple(m_oPointCloud.points().end(), indices.end()))
        );

        for(int i = 0; i < m_oPointCloud.size(); i++) {
            CGAL::Point point = m_oPointCloud.point(i);
            CGAL::Vector normal = m_oPointCloud.normal(i);

            CGAL::K_neighbor_search<Traits> search(tree, point, k + 1);
            int j = 0;
            for(auto it = search.begin() + 1; it != search.end(); it++) {
                Neighbors[i][j].first = boost::get<1>(it->first);
                Neighbors[i][j].second = sqrt(CGAL::to_double(it->second));
                j++;
            }
        }
    }
    
    int KNN::SearchClosetPoint(CGAL::Point point) {
        typedef boost::tuple<CGAL::Point, int>                  PointAndIndex;
        typedef CGAL::Search_traits_3<CGAL::Kernel>             Traits_base;
        typedef CGAL::Search_traits_adapter<PointAndIndex, 
            CGAL::Nth_of_tuple_property_map<0, PointAndIndex>, 
            Traits_base>                                        Traits;
        typedef CGAL::K_neighbor_search<Traits>::Tree Tree;


        std::vector<int> indices(m_oPointCloud.size());
        for (int i = 0; i < m_oPointCloud.size(); i++) {
            indices[i] = i;
        }
        Tree tree(
            boost::make_zip_iterator(boost::make_tuple(m_oPointCloud.points().begin(), indices.begin())),
            boost::make_zip_iterator(boost::make_tuple(m_oPointCloud.points().end(), indices.end()))
        );

        CGAL::K_neighbor_search<Traits> search(tree, point, 1);
        return boost::get<1>(search.begin()->first);
    }
}