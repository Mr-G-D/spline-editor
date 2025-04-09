#include <igl/opengl/glfw/Viewer.h>
#include <Eigen/Core>
#include <iostream>
#include <limits>
#include <imgui.h>
#include <igl/opengl/glfw/imgui/ImguiPlugin.h>
#include <ImGuizmo.h>

#include <igl/unproject_onto_mesh.h>
#include <igl/opengl/glfw/imgui/ImGuizmoWidget.h>

#include "Mesh.h";
//#include "Callbacks.cpp";

using namespace std;

extern void gizmoCallback(const Eigen::Matrix4f& T,
    Eigen::Matrix4f& previousTransform,
    vector<Eigen::Vector3d>& controlPoints,
    int& selectedVertex,
    Eigen::MatrixXd& control_V,
    function<void()> updateViewer,
    function<void()> computeBSplineSurface
);
extern void mouseDownCallback(int button,
    bool& activeGizmo,
    igl::opengl::glfw::Viewer& viewer,
    Eigen::MatrixXd& control_V,
    int& selectedVertex,
    igl::opengl::glfw::imgui::ImGuizmoWidget& gizmo,
    function<void()> updateViewer
);


Mesh::Mesh() {

    viewer.plugins.push_back(&imgui_plugin);

    imgui_plugin.widgets.push_back(&gizmo);

    gizmo.operation = ImGuizmo::OPERATION::TRANSLATE;

    initializeControlMesh();
    computeBSplineSurface();
    previousTransform.setIdentity();
    setupViewerCallbacks();
    viewer.launch();
}


void Mesh::initializeControlMesh() {

    int COLS = 4;
    float minRange = -1;
    float maxRange = 1;
    float gridStep = (maxRange - minRange) / (COLS - 1);
    int rows = COLS; //we start with a square lattice
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < COLS; ++j) {
            float x = minRange + j * gridStep;
            float z = minRange + i * gridStep;
            controlPoints.push_back({ x, 0.0f, z });  // Keeping Y = 0
        }
    }


    control_V.resize(controlPoints.size(), 3);
    for (int i = 0; i < controlPoints.size(); i++) {
        control_V.row(i) = controlPoints[i];
    }

    std::vector<std::pair<int, int>> edges;
    rows = controlPoints.size() / COLS;
    for (int i = 0; i < controlPoints.size() / COLS; i++) {
        for (int j = 0; j < COLS; j++) {
            int idx = i * COLS + j;
            if (j < COLS - 1) edges.emplace_back(idx, idx + 1);
            if (i < rows - 1) edges.emplace_back(idx, idx + COLS);
        }
    }

    control_F.resize(edges.size(), 2);
    for (int i = 0; i < edges.size(); i++) {
        control_F.row(i) << edges[i].first, edges[i].second;
    }


    updateViewer();
}

void Mesh::updateViewer() {
    viewer.data().clear(); // Clear the existing mesh data before setting the new one

    // Render control lattice mesh
    if (showControlLattice) {
        viewer.data().set_points(control_V, Eigen::RowVector3d(0, 1, 0));
        viewer.data().set_edges(control_V, control_F, Eigen::RowVector3d(0, 1, 0));
        highlightControlPoints();  // Highlight control points in green
    }

    // Render B-Spline surface mesh
    if (!spline_V.isZero() && !spline_F.isZero()) {
        viewer.data().set_mesh(spline_V, spline_F); // Update viewer with the new B-Spline surface
    }
}


void Mesh::highlightControlPoints() {
    // Highlight control points in green (or any color of your choice)
    for (int i = 0; i < control_V.rows(); ++i) {
        viewer.data().add_points(control_V.row(i), Eigen::RowVector3d(0, 1, 0)); // Green color for all control points
    }
}

double Mesh::basisFunction0(double t)
{
	return std::pow(t, 3) / 6.0;
}

double Mesh::basisFunction1(double t) {
    return (-3.0 * std::pow(t, 3) + 3.0 * std::pow(t, 2) + 3.0 * t + 1.0) / 6.0;
}

double Mesh::basisFunction2(double t) {
    return (3.0 * std::pow(t, 3) - 6.0 * std::pow(t, 2) + 4.0) / 6.0;
}

double Mesh::basisFunction3(double t) {
    return std::pow(1.0 - t, 3) / 6.0;
}

Eigen::Vector3d Mesh::evaluate(const std::vector<Eigen::Vector3d>& controlPoints, double u, double v) {
    Eigen::Vector3d result = Eigen::Vector3d::Zero();

    // Precompute basis weights for u and v directions
    const double Bu[4] = {
        basisFunction3(u),
        basisFunction2(u),
        basisFunction1(u),
        basisFunction0(u)
    };

    const double Bv[4] = {
        basisFunction3(v),
        basisFunction2(v),
        basisFunction1(v),
        basisFunction0(v)
    };

    // Perform tensor product sum
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            int index = row * 4 + col;
            result += Bu[row] * Bv[col] * controlPoints[index];
        }
    }

    return result;
}


//Eigen::Vector3d Mesh::evaluate(std::vector<Eigen::Vector3d>& cpGrid, double u, double v) {
//    Eigen::Vector3d point(0, 0, 0);
//    double Bu[4] = { B0(u), B1(u), B2(u), B3(u) };
//    double Bv[4] = { B0(v), B1(v), B2(v), B3(v) };
//
//    for (int i = 0; i < 4; ++i) {
//        for (int j = 0; j < 4; ++j) {
//            point += Bu[i] * Bv[j] * cpGrid[i * 4 + j];
//        }
//    }
//    return point;
//}

void Mesh::computeBSplineSurface() {

    // Collect 4x4 control points from the flat control_V grid
    std::vector<Eigen::Vector3d> controlPatch;
    controlPatch.reserve(patchSize * patchSize);
    for (int row = 0; row < patchSize; ++row) {
        for (int col = 0; col < patchSize; ++col) {
            int linearIndex = row * patchSize + col;
            controlPatch.emplace_back(control_V.row(linearIndex));
        }
    }

    // Prepare B-Spline vertex and face buffers
    spline_V.resize(resolution * resolution, 3);
    spline_F.resize((resolution - 1) * (resolution - 1) * 2, 3);

    // Fill spline_V with evaluated points on the surface
    int vertexId = 0;
    for (int i = 0; i < resolution; ++i) {
        double u = static_cast<double>(i) / (resolution - 1); // u ∈ [0, 1]
        for (int j = 0; j < resolution; ++j) {
            double v = static_cast<double>(j) / (resolution - 1); // v ∈ [0, 1]
            spline_V.row(vertexId++) = evaluate(controlPatch, u, v);
        }
    }

    // Generate triangle faces for the surface mesh
    int faceId = 0;
    for (int i = 0; i < resolution - 1; ++i) {
        for (int j = 0; j < resolution - 1; ++j) {
            int topLeft = i * resolution + j;
            int topRight = topLeft + 1;
            int bottomLeft = topLeft + resolution;
            int bottomRight = bottomLeft + 1;

            // Triangle 1: topLeft → bottomLeft → topRight
            spline_F.row(faceId++) << topLeft, bottomLeft, topRight;
            // Triangle 2: topRight → bottomLeft → bottomRight
            spline_F.row(faceId++) << topRight, bottomLeft, bottomRight;
        }
    }

    updateViewer();
}


//void Mesh::computeBSplineSurface() {
//    int resolution = 20;  // Resolution of the B-Spline surface
//    int cols = 4;         // Assuming a 4x4 grid of control points
//
//    // Resize the B-Spline vertices and faces based on the resolution
//    spline_V.resize(resolution * resolution, 3);
//    spline_F.resize((resolution - 1) * (resolution - 1) * 2, 3);
//
//    int vertexOffset = 0;
//    int faceOffset = 0;
//
//    std::vector<Eigen::Vector3d> patchControlPoints;
//
//    // Iterate over the matrix to get each control point (4x4 grid in this example)
//    for (int i = 0; i < cols; ++i) {
//        for (int j = 0; j < cols; ++j) {
//            int index = i * cols + j; // Calculate the linear index of the matrix
//            // Extract x, y, z coordinates and create a vec3d object
//            patchControlPoints.push_back(Eigen::Vector3d(control_V(index, 0),
//                control_V(index, 1),
//                control_V(index, 2)));
//        }
//    }
//
//    double minU = 0.0, maxU = 1.0;
//    double minV = 0.0, maxV = 1.0;
//
//    // Calculate vertices for the B-Spline surface
//    for (int u = 0; u < resolution; u++) {
//        double uu = minU + (maxU - minU) * (double)u / (resolution - 1);
//        for (int v = 0; v < resolution; v++) {
//            double vv = minV + (maxV - minV) * (double)v / (resolution - 1);
//            spline_V.row(vertexOffset++) = evaluate(patchControlPoints, uu, vv);
//        }
//    }
//
//    // Calculate faces for the B-Spline surface
//    for (int i = 0; i < resolution - 1; i++) {
//        for (int j = 0; j < resolution - 1; j++) {
//            int idx = i * resolution + j;
//            spline_F.row(faceOffset++) << idx, idx + resolution, idx + 1;  // Triangle 1
//            spline_F.row(faceOffset++) << idx + 1, idx + resolution, idx + resolution + 1;  // Triangle 2
//        }
//    }
//
//    updateViewer();
//}


void Mesh::setupViewerCallbacks() {

    gizmo.callback = [&](const Eigen::Matrix4f& T)
    {

    	gizmoCallback(T,
            previousTransform, 
            controlPoints, 
            selectedVertex, 
            control_V, 
            [this]() {updateViewer(); },
            [this]() {computeBSplineSurface(); });

    };

viewer.callback_mouse_down = [&](igl::opengl::glfw::Viewer& viewer, int button, int modifier) {
    mouseDownCallback(button, 
        activeGizmo, 
        viewer,
        control_V, 
        selectedVertex, 
        gizmo, [this]() {updateViewer(); });
    return false;
};

viewer.callback_mouse_up = [&](igl::opengl::glfw::Viewer&, int, int) {
    activeGizmo = false;
    return false;
    };


    //gizmo.callback = [&](const Eigen::Matrix4f& T)
    //    {

    //        if (T.isApprox(previousTransform)) {
    //            return;  // No change, skip callback
    //        }

    //        // Store the new transformation for the next frame comparison
    //        previousTransform = T;

    //        // Perform transformation or update control points only when change is detected
    //        if (selectedVertex != -1) {
    //            cout << controlPoints[selectedVertex] << endl;
    //            control_V.row(selectedVertex) = T.block<3, 1>(0, 3).cast<double>();
    //            cout << controlPoints[selectedVertex] << endl;
    //            std::cout << "Updated control vertex position: " << control_V.row(selectedVertex) << std::endl;
    //            computeBSplineSurface();
    //            updateViewer();
    //        }

    //    };
    //viewer.callback_key_pressed = [&](decltype(viewer)&, unsigned int key, int mod)
    //    {
    //        switch (key)
    //        {
    //        case ' ': gizmo.visible = !gizmo.visible; return true;
    //        case 'W': case 'w': gizmo.operation = ImGuizmo::TRANSLATE; return true;
    //        }
    //        return false;
    //    };

    //viewer.callback_mouse_down = [&](igl::opengl::glfw::Viewer& viewer, int button, int modifier) {
    //    if (button == 0) { // Left click
    //        activeGizmo = true;
    //        // Convert mouse position to 2D screen space
    //        Eigen::Vector2f mousePos(viewer.current_mouse_x, viewer.core().viewport(3) - viewer.current_mouse_y);
    //        float minDist = std::numeric_limits<float>::max(); // Start with a very large number
    //        int closestVertex = -1;

    //        // Convert matrices to float for projection
    //        Eigen::Matrix4f view = viewer.core().view.cast<float>();
    //        Eigen::Matrix4f proj = viewer.core().proj.cast<float>();
    //        Eigen::Vector4f viewport = viewer.core().viewport.cast<float>();

    //        // Loop through all control points and find the closest one
    //        for (int i = 0; i < control_V.rows(); ++i) {
    //            Eigen::Vector3f point = control_V.row(i).cast<float>();
    //            Eigen::Vector3f screenPos = igl::project(point, view, proj, viewport); // Project the 3D point onto screen space

    //            // Calculate the 2D distance between the mouse click and the projected point
    //            float dist = (screenPos.head<2>() - mousePos).norm();
    //            if (dist < minDist) {
    //                minDist = dist;
    //                closestVertex = i; // Store the index of the closest control point
    //            }
    //        }

    //        // If a point is selected, set it as the selected vertex
    //        if (closestVertex != -1) {
    //            selectedVertex = closestVertex;
    //            auto pos = control_V.row(selectedVertex);
    //            gizmo.T(0, 3) = pos.x();
    //            gizmo.T(1, 3) = pos.y();
    //            gizmo.T(2, 3) = pos.z();
    //            gizmo.T(3, 3) = 1.0f;
    //            std::cout << "Selected vertex: " << selectedVertex << std::endl;
    //        }
    //        else {
    //            selectedVertex = -1;
    //            std::cerr << "No vertex found." << std::endl;
    //        }

    //        updateViewer();

    //    }
    //    return false;
    //    };

    //viewer.callback_mouse_up = [&](igl::opengl::glfw::Viewer&, int, int) {
    //    activeGizmo = false;
    //    return false;
    //    };
}
