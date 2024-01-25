#pragma once

#include "pch.h"

namespace CGAL {
	typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
	typedef Kernel::FT FT;
	typedef Kernel::Point_3 Point;
	typedef Kernel::Vector_3 Vector;
	typedef CGAL::Point_set_3<CGAL::Point, CGAL::Vector> Point_set;
	typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
	typedef CGAL::Surface_mesh<Kernel::Point_3> Mesh;
}
	