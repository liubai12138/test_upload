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

class reconstruction_teeth
{
public:
    reconstruction_teeth(const PointCloud& cloud) : cloud_(cloud), viewer_(nullptr), model_(nullptr) {
    initialize(); // Initialize Easy3D

}

~reconstruction_teeth() {
    delete viewer_;
}

// Create a point cloud

void surf_reconstruction();
// Add some points. Here we add 100 points on a 10*10 grid.

void  smooth();
//Êä³öÄ£ÐÍ
Model * output_model() {
    if (!model_) {
        return ;
    }

    return model_;
}
// Delete the point cloud (i.e., release memory)

private:
    std::string file_;
    Viewer* viewer_;
    Model* model_;
    PointCloud cloud_;

};