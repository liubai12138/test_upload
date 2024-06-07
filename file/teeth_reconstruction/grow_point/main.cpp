#include <iostream>
#include <fstream>
#include <vector>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/IO/STL.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include<CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Shape_detection/Region_growing.h>
#include <CGAL/property_map.h>
#include <CGAL/Classification/Mesh_neighborhood.h>
#include <CGAL/Surface_mesh_shortest_path.h>
//fill_hole
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Polygon_mesh_processing/border.h>

#include <boost/lexical_cast.hpp>

#include <iterator>
#include <string>
#include <tuple>


//typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Vector_3 Vector_3;
typedef CGAL::Surface_mesh<Point_3> Mesh;
typedef boost::graph_traits<Mesh>::face_descriptor face_descriptor;
typedef boost::graph_traits<Mesh>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Mesh>::halfedge_descriptor halfedge_descriptor;


//typedef CGAL::Surface_mesh<Kernel::Point_3> Triangle_mesh;
typedef CGAL::Surface_mesh_shortest_path_traits<Kernel, Mesh> Traits;
typedef CGAL::Surface_mesh_shortest_path<Traits> Surface_mesh_shortest_path;

namespace PMP = CGAL::Polygon_mesh_processing;

double distance(const Point_3& p1, const Point_3& p2) {
    return std::sqrt(CGAL::squared_distance(p1, p2));
}

bool is_small_hole(halfedge_descriptor h, Mesh& mesh,
    double max_hole_diam, int max_num_hole_edges)
{
    int num_hole_edges = 0;
    CGAL::Bbox_3 hole_bbox;
    for (halfedge_descriptor hc : CGAL::halfedges_around_face(h, mesh))
    {
        const Point_3& p = mesh.point(target(hc, mesh));
        hole_bbox += p.bbox();
        ++num_hole_edges;
        // Exit early, to avoid unnecessary traversal of large holes
        if (num_hole_edges > max_num_hole_edges) return false;
        if (hole_bbox.xmax() - hole_bbox.xmin() > max_hole_diam) return false;
        if (hole_bbox.ymax() - hole_bbox.ymin() > max_hole_diam) return false;
        if (hole_bbox.zmax() - hole_bbox.zmin() > max_hole_diam) return false;
    }
    return true;
}

void output_region_mesh(const std::string& filename, const std::unordered_set<vertex_descriptor>& region, const Mesh& mesh) {
    Mesh region_mesh;
    std::unordered_map<vertex_descriptor, vertex_descriptor> old_to_new;

    // �������������ڵĶ��㵽�µ�������
    for (auto vd : region) {
        old_to_new[vd] = region_mesh.add_vertex(mesh.point(vd));
    }

    // �������������ڵ��浽�µ�������
    for (auto vd : region) {
        for (auto hf : CGAL::halfedges_around_target(vd, mesh)) {
            face_descriptor fd = face(hf, mesh);
            if (fd != Mesh::null_face()) {
                bool include_face = true;
                for (auto v : CGAL::vertices_around_face(mesh.halfedge(fd), mesh)) {
                    if (region.find(v) == region.end()) {
                        include_face = false;
                        break;
                    }
                }
                if (include_face) {
                    std::vector<vertex_descriptor> vertices;
                    for (auto v : CGAL::vertices_around_face(mesh.halfedge(fd), mesh)) {
                        vertices.push_back(old_to_new[v]);
                    }
                    region_mesh.add_face(vertices);
                }
            }
        }
    }

    //����
    std::vector<halfedge_descriptor> border_cycles;
    // collect one halfedge per boundary cycle
    PMP::extract_boundary_cycles(region_mesh, std::back_inserter(border_cycles));
    //
    double max_hole_diam = 50;
    double max_num_hole_edges = 200;
    for (halfedge_descriptor h : border_cycles)
    {
        if (max_hole_diam > 0 && max_num_hole_edges > 0 &&
            !is_small_hole(h, region_mesh, max_hole_diam, max_num_hole_edges))
            continue;
        std::vector<face_descriptor>  patch_facets;
        std::vector<vertex_descriptor> patch_vertices;
        bool success = std::get<0>(PMP::triangulate_refine_and_fair_hole(region_mesh,h,std::back_inserter(patch_facets),std::back_inserter(patch_vertices)));
        //std::cout << "* Number of facets in constructed patch: " << patch_facets.size() << std::endl;
        //std::cout << "  Number of vertices in constructed patch: " << patch_vertices.size() << std::endl;
        //std::cout << "  Is fairing successful: " << success << std::endl;
    }
    //// �޲������еĿ�
    //  // ʹ��CGAL���㷨�����
    //bool success = CGAL::Polygon_mesh_processing::triangulate_hole(mesh, border.begin(), border.end());
    //PMP::hole_filling(region_mesh, faces(region_mesh), PMP::parameters::all_default());

    // ����µ�����STL�ļ�
    std::ofstream output(filename, std::ios::binary);
    if (!output) {
        std::cerr << "Error: Cannot open file " << filename << std::endl;
        return;
    }
    CGAL::IO::write_STL(output, region_mesh);
    output.close();
}



void output_region(const std::string& filename, const std::unordered_set<vertex_descriptor>& region, const Mesh& mesh) {
    std::ofstream out(filename);
    if (!out) {
        std::cerr << "Error: Cannot open file " << filename << std::endl;
        return;
    }

    out << "Region vertices:" << std::endl;
    for (auto vd : region) {
        Point_3 p = mesh.point(vd);
        out << vd << ": " << p << std::endl;
    }

    out.close();
}


int main() {
    Mesh mesh;

    // ��ȡSTL�ļ�
    std::ifstream input("test.stl", std::ios::binary);
    if (!input) {
        std::cerr << "Error: Cannot open file mesh.stl" << std::endl;
        return 1;
    }

    std::vector<CGAL::cpp11::array<double, 3>> points;
    std::vector<CGAL::cpp11::array<std::size_t, 3>> triangles;

    if (!CGAL::IO::read_STL(input, points, triangles)) {
        std::cerr << "Error: Invalid STL file" << std::endl;
        return 1;
    }

    std::vector<vertex_descriptor> vertices;
    for (const auto& point : points) {
        vertices.push_back(mesh.add_vertex(Point_3(point[0], point[1], point[2])));
    }
    for (const auto& triangle : triangles) {
        mesh.add_face(vertices[triangle[0]], vertices[triangle[1]], vertices[triangle[2]]);
    }

    if (mesh.is_empty()) {
        std::cerr << "Error: Mesh is empty after loading STL file" << std::endl;
        return 1;
    }

    // �����淨����
    Mesh::Property_map<face_descriptor, Vector_3> fnormals =
        mesh.add_property_map<face_descriptor, Vector_3>("f:normals", CGAL::NULL_VECTOR).first;
    PMP::compute_face_normals(mesh, fnormals);

    // ���㶥�㷨����
    Mesh::Property_map<vertex_descriptor, Vector_3> vnormals =
        mesh.add_property_map<vertex_descriptor, Vector_3>("v:normals", CGAL::NULL_VECTOR).first;
    PMP::compute_vertex_normals(mesh, vnormals, PMP::parameters::vertex_point_map(mesh.points()).geom_traits(Kernel()));

    // ָ�����ӵ㣨���ӵ�����4122��
    vertex_descriptor seed_vertex = vertices[4122];
    //vertex_descriptor seed_vertex = vertices[10435];


    //Point_3 seed_point = mesh.point(seed_vertex);
    vertex_descriptor last_vertex = seed_vertex;

    //// 1. ������ָ�������ڵ���������
    //double distance_threshold = 4; // ������ֵ����λ������������ͬ
    //std::unordered_set<vertex_descriptor> distance_region;
    //std::queue<vertex_descriptor> queue;
    //queue.push(seed_vertex);
    //distance_region.insert(seed_vertex);

    ////// �������·������
    ////CGAL::Surface_mesh_shortest_path<Mesh, int> shortest_paths(mesh, CGAL::get(CGAL::vertex_index, mesh));
    //    // ʹ��Surface_mesh_shortest_path��������ھ������������
    ////CGAL::Surface_mesh_shortest_path<Kernel> shortest_paths(mesh);
    //Surface_mesh_shortest_path shortest_paths(mesh);
    //shortest_paths.add_source_point(seed_vertex);

    //while (!queue.empty()) {
    //    vertex_descriptor current = queue.front();
    //    queue.pop();
    //    Point_3 current_point = mesh.point(current);

    //    for (auto halfedge : CGAL::halfedges_around_target(current, mesh)) {
    //        vertex_descriptor adj_vd = target(opposite(halfedge, mesh), mesh);
    //        if (distance_region.find(adj_vd) == distance_region.end()) {
    //            auto dist_pair = shortest_paths.shortest_distance_to_source_points(adj_vd);
    //            double dist = dist_pair.first;
    //            if (dist < distance_threshold) {
    //                distance_region.insert(adj_vd);
    //                queue.push(adj_vd);
    //                last_vertex = adj_vd;
    //            }
    //        }
    //        //if (distance_region.find(adj_vd) == distance_region.end() &&
    //        //    distance(seed_point, mesh.point(adj_vd)) < distance_threshold) {
    //        //    distance_region.insert(adj_vd);
    //        //    queue.push(adj_vd);
    //        //}
    //    }
    //}
    //output_region("distance_region.txt", distance_region, mesh);
    //output_region_mesh("distance_region.stl", distance_region, mesh);
    //std::cout << "Distance-based region size: " << distance_region.size() << std::endl;

    // 2. �������ʵ���������
    double curvature_threshold = 0.1; // ������ֵ
    std::unordered_set<vertex_descriptor> curvature_region;
    //curvature_region = distance_region;
    curvature_region.insert(seed_vertex);
    std::queue<vertex_descriptor> queue;
    queue.push(seed_vertex);

    while (!queue.empty()) {
        vertex_descriptor current = queue.front();
        queue.pop();
        Vector_3 current_normal = vnormals[current];

        for (auto halfedge : CGAL::halfedges_around_target(current, mesh)) {
            vertex_descriptor adj_vd = target(opposite(halfedge, mesh), mesh);
            if (curvature_region.find(adj_vd) == curvature_region.end()) {
                Vector_3 adj_normal = vnormals[adj_vd];
                double angle = std::acos(std::min(1.0, std::max(-1.0, current_normal * adj_normal)));
                if (angle < curvature_threshold) {
                    curvature_region.insert(adj_vd);
                    queue.push(adj_vd);
                }
            }
        }
    }

    //output_region("curvature_region.txt", curvature_region, mesh);
    //output_region_mesh("curvature_region.stl", curvature_region, mesh);
    std::cout << "Curvature-based region size: " << curvature_region.size() << std::endl;

    return 0;
}
