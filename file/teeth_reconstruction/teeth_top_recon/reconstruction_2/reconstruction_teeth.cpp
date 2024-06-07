#include "reconstruction_teeth.h"

void reconstruction_teeth::surf_reconstruction() {
 
    auto cloud = &cloud_;
    auto normals = cloud->get_vertex_property<vec3>("v:normal");
    if (!normals) {
        std::cerr << "Poisson surface reconstruction method requires normal information."
            << " Please provide normal information. Alternatively, you can use the "
            << " Tutorial_701_Cloud_NormalEstimation for normal estimation" << std::endl;

    }

    const int depth = 8;
    PoissonReconstruction algo;
    algo.set_depth(depth);
    algo.set_samples_per_node(8);
    //algo.set_full_depth(3);
    std::cout << "reconstruction depth: " << depth << std::endl;
    //Model* surface = algo.apply(cloud);

    SurfaceMesh* mesh = algo.apply(cloud);
    SurfaceMeshSmoothing smoother(mesh);
    smoother.explicit_smoothing(15, true);

    std::string density_attr_name_ = "v:density";
    auto density = mesh->vertex_property<float>(density_attr_name_);
    if (!density) {
        LOG(WARNING) << "no property \'density\' for trimming";

    }
    //for (auto v : mesh->vertices()) {
    //    float value = density[v];
    //    std::cout << "ÃÜ¶È£º" << value << std::endl;
    //}

    SurfaceMesh* trimmed_mesh = algo.trim(mesh, density_attr_name_, 4, 0.001, false);

    model_ = trimmed_mesh;

}
void reconstruction_teeth::smooth(){

        auto mesh = dynamic_cast<SurfaceMesh*>(model_);

        std::cout << "good" << std::endl;
        SurfaceMeshSmoothing smoother(mesh);
        smoother.explicit_smoothing();
        model_= mesh;
}
