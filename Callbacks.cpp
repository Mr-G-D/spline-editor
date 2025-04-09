
#include <iostream>
#include <vector>
#include <Eigen/core>;

#include "igl/opengl/glfw/imgui/ImGuiMenu.h"
#include "igl/opengl/glfw/imgui/ImGuizmoWidget.h"

using namespace std;

void gizmoCallback(const Eigen::Matrix4f& T,
    Eigen::Matrix4f& previousTransform,
    vector<Eigen::Vector3d>& controlPoints,
    int& selectedVertex,
    Eigen::MatrixXd& control_V,
    function<void()> updateViewer,
    function<void()> computeBSplineSurface
    )
{
    if (T.isApprox(previousTransform)) {
        return;  // No change, skip callback
    }

    // Store the new transformation for the next frame comparison
    previousTransform = T;

    // Perform transformation or update control points only when change is detected
    if (selectedVertex != -1) {
        cout << controlPoints[selectedVertex] << endl;
        control_V.row(selectedVertex) = T.block<3, 1>(0, 3).cast<double>();
        cout << controlPoints[selectedVertex] << endl;
        std::cout << "Updated control vertex position: " << control_V.row(selectedVertex) << std::endl;
        computeBSplineSurface();
        updateViewer();
    }
}

void mouseDownCallback(int button, 
    bool& activeGizmo,
    igl::opengl::glfw::Viewer& viewer,
    Eigen::MatrixXd& control_V,
    int& selectedVertex,
    igl::opengl::glfw::imgui::ImGuizmoWidget& gizmo,
    function<void()> updateViewer
    )
{
    if (button == 0) { // Left click
        activeGizmo = true;
        // Convert mouse position to 2D screen space
        Eigen::Vector2f mousePos(viewer.current_mouse_x, viewer.core().viewport(3) - viewer.current_mouse_y);
        float minDist = std::numeric_limits<float>::max(); // Start with a very large number
        int closestVertex = -1;

        // Convert matrices to float for projection
        Eigen::Matrix4f view = viewer.core().view.cast<float>();
        Eigen::Matrix4f proj = viewer.core().proj.cast<float>();
        Eigen::Vector4f viewport = viewer.core().viewport.cast<float>();

        // Loop through all control points and find the closest one
        for (int i = 0; i < control_V.rows(); ++i) {
            Eigen::Vector3f point = control_V.row(i).cast<float>();
            Eigen::Vector3f screenPos = igl::project(point, view, proj, viewport); // Project the 3D point onto screen space

            // Calculate the 2D distance between the mouse click and the projected point
            float dist = (screenPos.head<2>() - mousePos).norm();
            if (dist < minDist) {
                minDist = dist;
                closestVertex = i; // Store the index of the closest control point
            }
        }

        // If a point is selected, set it as the selected vertex
        if (closestVertex != -1) {
            selectedVertex = closestVertex;
            auto pos = control_V.row(selectedVertex);
            gizmo.T(0, 3) = pos.x();
            gizmo.T(1, 3) = pos.y();
            gizmo.T(2, 3) = pos.z();
            gizmo.T(3, 3) = 1.0f;
            std::cout << "Selected vertex: " << selectedVertex << std::endl;
        }
        else {
            selectedVertex = -1;
            std::cerr << "No vertex found." << std::endl;
        }

        updateViewer();

    }
}
