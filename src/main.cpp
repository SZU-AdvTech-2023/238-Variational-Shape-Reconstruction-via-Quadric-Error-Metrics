#include "pch.h"

#include "CGAL/draw_point_set_3.h"
#include "CGAL/IO/read_off_points.h"
#include "CGAL/IO/write_ply_points.h"
#include "CGAL/property_map.h"
#include "CGAL/pca_estimate_normals.h"
#include "CGAL/Polygon_mesh_processing/distance.h"

#include "core/core.h"
#include "QEM/QEM.h"
#include "knn/knn.h"
#include "RegionGrow/RegionGrow.h"

#include "reconstruction/Variational_shape_reconstruction.h"


std::vector<std::tuple<int, int, int>> RandColor(int n)
{
	std::vector<std::tuple<int, int, int>> colors(n);
	for (int i = 0; i < n; i++) {
		colors[i] = (std::make_tuple(rand() % 255, rand() % 255, rand() % 255));
	}
	return colors;
}

void DrawPointSet(CGAL::Point_set &pointSet, VSR::RegionGrow& regionGrow)
{
	auto colors = RandColor(regionGrow.GetLabelCount());
	auto rmap = pointSet.add_property_map<int>("red").first;
	auto gmap = pointSet.add_property_map<int>("green").first;
	auto bmap = pointSet.add_property_map<int>("blue").first;

	for(int i = 0; i < pointSet.size(); i++) {
		if(i >= regionGrow.m_vLables.size()) {
			rmap[i] = 255;
			gmap[i] = 255;
			bmap[i] = 255;
			continue;
		}
		int label = regionGrow.m_vLables[i];
		if (label == -1) {
			rmap[i] = 255;
			gmap[i] = 255;
			bmap[i] = 255;
			continue;
		}
		rmap[i] = std::get<0>(colors[label]);
		gmap[i] = std::get<1>(colors[label]);
		bmap[i] = std::get<2>(colors[label]);
	}

	// cgal bug: uchar will be written as ascii char, so we set property to int and replace int with uchar
	std::stringstream ss;
	CGAL::set_ascii_mode(ss);
	CGAL::IO::write_PLY(ss, pointSet);

	std::fstream fs("asset/out.ply", std::ios::out);
	std::string line;
	while (std::getline(ss, line)) {
		if(line.find("int") != std::string::npos)
			line = line.replace(line.find("int"), 3, "uchar");
		fs << line << std::endl;
	}
}

void AddGenorator(CGAL::Point_set_3<CGAL::Point, CGAL::Vector>& pointSet, VSR::RegionGrow& regionGrow)
{
	for (int i = 0; i < regionGrow.m_vSeeds.size(); i++) {
		pointSet.insert(regionGrow.m_vSeeds[i].first);
	}
}

int main()
{
	CGAL::Point_set pointSet;
	if (!CGAL::IO::read_point_set( "asset/guitar.xyz", pointSet))
    {
        std::cerr << "Error: cannot read file " << std::endl;
        return EXIT_FAILURE;
    }

	// VSR::KNN knn(pointSet);
	// VSR::QEM qem(pointSet, knn);
	// VSR::RegionGrow regionGrow(pointSet, VSR::RegionGrow::GenerateSeeds(pointSet, 10), qem, knn);
	// regionGrow.Grow();
	// regionGrow.SeedsUpdate();
	// AddGenorator(pointSet, regionGrow);
	// DrawPointSet(pointSet, regionGrow);
	
	size_t generators = 30; 
    const size_t steps = 5;
    const double split_threshold = 5e-2;
    const double distance_weight = 10e-5;
    size_t iteration = 0 ;
	qem::Variational_shape_reconstruction manager(
        pointSet,
        generators,
        distance_weight);
	while(generators > 5 ) {
		manager.region_growing_and_update_poles(steps);
        generators = manager.guided_split_clusters(split_threshold, iteration++);
    }
	manager.region_growing_and_update_poles(steps);

    manager.reconstruction();

	auto mesh = manager.get_reconstructed_mesh();
    std::ofstream mesh_file;
    mesh_file.open("asset/mesh.off");
    CGAL::write_off(mesh_file, mesh);
    mesh_file.close();

	return 0;
}