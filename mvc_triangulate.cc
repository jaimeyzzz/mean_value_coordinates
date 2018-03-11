#include "mvc_triangulate.h"

using namespace cv;
using namespace std;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_2<K> Vb;
typedef CGAL::Delaunay_mesh_face_base_2<K> Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, Tds> CDT;
typedef CGAL::Delaunay_mesh_size_criteria_2<CDT> Criteria;
typedef CGAL::Delaunay_mesher_2<CDT, Criteria> Mesher;

typedef CDT::Vertex_handle Vertex_handle;
typedef K::Point_2   CGPoint;

//#define DEBUG

void Triangulate(const std::vector<cv::Point>& bound, std::vector<cv::Point>& pts, std::vector<int>& tris, const Mat& mask) {
    vector<CGPoint> vertices;
    for (int i = 0; i < bound.size(); i++)
        vertices.push_back(CGPoint(bound[i].x, bound[i].y));
    CDT cdt;

    Vertex_handle v_start = cdt.insert(vertices[0]);
    Vertex_handle v_last = v_start;
    for (int i = 0; i < vertices.size(); i++) {
        Vertex_handle v_cur;
        if (i == vertices.size() - 1)
            v_cur = v_start;
        else
            v_cur = cdt.insert(vertices[i + 1]);

        cdt.insert_constraint(v_last, v_cur);
        v_last = v_cur;
    }

    assert(cdt.is_valid());

    Mesher mesher(cdt);
    mesher.refine_mesh();

    CDT::Finite_vertices_iterator vit;
    pts.clear(); tris.clear();
    for (vit = cdt.finite_vertices_begin(); vit != cdt.finite_vertices_end(); vit++) {
        pts.push_back(Point(vit->point().hx(), vit->point().hy()));
    }

    CDT::Finite_faces_iterator fit;
    for (fit = cdt.finite_faces_begin(); fit != cdt.finite_faces_end(); fit++) {
        double cx = 0.0, cy = 0.0;
        bool out = false;
        for (int k = 0; k < 3; k++) {
            Point pt(fit->vertex(k)->point().hx(), fit->vertex(k)->point().hy());
            cx += pt.x, cy += pt.y;
            if (mask.at<uchar>(pt.y, pt.x) == 0) {
                out = true;
                break;
            }
        }
        cx /= 3.0, cy /= 3.0;
        if (out || mask.at<uchar>(cvRound(cy), cvRound(cx)) == 0) continue;
        for (int k = 0; k < 3; k++) {
            Point pt(fit->vertex(k)->point().hx(), fit->vertex(k)->point().hy());
            int index = find(pts.begin(), pts.end(), pt) - pts.begin();
            tris.push_back(index);
        }
    }
}