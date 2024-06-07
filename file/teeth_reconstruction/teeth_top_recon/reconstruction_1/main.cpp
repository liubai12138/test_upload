#include <easy3d/viewer/viewer.h>
#include <easy3d/core/model.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/renderer/renderer.h>
#include <easy3d/algo/point_cloud_poisson_reconstruction.h>
#include <easy3d/algo/surface_mesh_smoothing.h>
#include <easy3d/util/resource.h>
#include <easy3d/util/initializer.h>


using namespace easy3d;

// This example shows how to
//		- reconstruct a smooth surface from a point cloud using the Poisson surface reconstruction method


bool reconstruction(Viewer* viewer, Model* model) {
    if (!viewer || !model)
        return false;

    auto cloud = dynamic_cast<PointCloud*>(model);
    auto normals = cloud->get_vertex_property<vec3>("v:normal");
    if (!normals) {
        std::cerr << "Poisson surface reconstruction method requires normal information."
            << " Please provide normal information. Alternatively, you can use the "
            << " Tutorial_701_Cloud_NormalEstimation for normal estimation" << std::endl;
        return false;
    }

    const int depth = 8;
    PoissonReconstruction algo;
    algo.set_depth(depth);
    algo.set_samples_per_node(8);
    //algo.set_full_depth(3);
    std::cout << "reconstruction depth: " << depth << std::endl;
    //Model* surface = algo.apply(cloud);

    SurfaceMesh *mesh= algo.apply(cloud);
    SurfaceMeshSmoothing smoother(mesh);
    smoother.explicit_smoothing(15,true);

    std::string density_attr_name_ = "v:density";
    auto density = mesh->vertex_property<float>(density_attr_name_);
    if (!density) {
        LOG(WARNING) << "no property \'density\' for trimming";
        return false;
    }
    //for (auto v : mesh->vertices()) {
    //    float value = density[v];
    //    std::cout << "ÃÜ¶È£º" << value << std::endl;
    //}

    SurfaceMesh* trimmed_mesh = algo.trim(mesh, density_attr_name_, 4, 0.001,false);

    Model* surface = trimmed_mesh;




    //smoother.explicit_smoothing(2, true);
    if (surface != nullptr) {
        viewer->add_model(surface, true);
        viewer->delete_model(cloud);
        viewer->update();
    }


    return true;
}
bool smooth(Viewer* viewer, Model* model) {
    if (!viewer || !model)
        return false;

    auto mesh = dynamic_cast<SurfaceMesh*>(model);

    std::cout << "good" << std::endl;
    SurfaceMeshSmoothing smoother(mesh);
    smoother.explicit_smoothing();
    Model* surface = mesh;

    //smoother.explicit_smoothing(2, true);
    if (surface != nullptr) {
        viewer->add_model(surface, true);
        viewer->delete_model(mesh);
        viewer->update();
    }


    return true;
}


int main(int argc, char** argv) {
    // initialize Easy3D.
    initialize();

    const std::string file = "E:/other/data/output_n.ply";

    // create the viewer.
    Viewer viewer("EXAMPLE_TITLE");

    Model* model = viewer.add_model(file, true);
    if (!model) {
        LOG(ERROR) << "failed to load model. Please make sure the file exists and format is correct.";
        return EXIT_FAILURE;
    }

    // setup rendering parameters
    auto drawable = model->renderer()->get_points_drawable("vertices");
    drawable->set_uniform_coloring(vec4(0.6f, 0.6f, 1.0f, 1.0f));
    drawable->set_point_size(3.0f);

    // usage
    viewer.set_usage("'Ctrl + r': run reconstruction");
    // set up the function to be executed and its corresponding shortcut
    viewer.bind(reconstruction, model, Viewer::KEY_R, Viewer::MODIF_CTRL);

    viewer.bind(smooth, model, Viewer::KEY_S, Viewer::MODIF_CTRL);

    // run the viewer
    return viewer.run();
}

