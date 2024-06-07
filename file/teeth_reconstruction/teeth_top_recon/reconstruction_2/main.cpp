#include "reconstruction_teeth.h"
int main(int argc, char** argv) {


    const std::string file = "E:/other/data/output_n.ply";

    // create the viewer.
    Viewer viewer("EXAMPLE_TITLE");

    Model* model = viewer.add_model(file, true);
    //这个后面俊雄传给我3*n的矩阵，我转成点，现在先这样用着
    auto cloud_input = dynamic_cast<PointCloud*>(model);
    for (int i = -5; i < 5; ++i) {
        for (int j = -5; j < 5; ++j)
            cloud_input->add_vertex(vec3(static_cast<float>(i), static_cast<float>(j), 0));// z = 0: all points are on XY plane
    }
    std::cout << "point cloud has " << cloud_input->n_vertices() << " points" << std::endl;



    reconstruction_teeth rteeth = reconstruction_teeth(*cloud_input);
    rteeth.surf_reconstruction();
    rteeth.smooth();
    Model* model = rteeth.output_model();
}

