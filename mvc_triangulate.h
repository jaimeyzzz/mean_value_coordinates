#include <opencv2\opencv.hpp>
#include <vector>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>

void Triangulate(const std::vector<cv::Point>& bound, std::vector<cv::Point>& pts, std::vector<int>& tris, const cv::Mat& mask);