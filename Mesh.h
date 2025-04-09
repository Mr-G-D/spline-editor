#pragma once
#include <igl/opengl/glfw/Viewer.h>
#include <Eigen/Core>
#include <iostream>
#include <limits>
#include <imgui.h>
#include <igl/opengl/glfw/imgui/ImguiPlugin.h>
#include <ImGuizmo.h>

#include <igl/unproject_onto_mesh.h>
#include <igl/opengl/glfw/imgui/ImGuizmoWidget.h>


class Mesh {
public:
    Eigen::MatrixXd control_V; // Control lattice vertices
    Eigen::MatrixXi control_F; // Control lattice faces
    Eigen::MatrixXd spline_V;  // B-Spline mesh vertices
    Eigen::MatrixXi spline_F;  // B-Spline mesh faces
    Eigen::Matrix4f previousTransform;

    igl::opengl::glfw::imgui::ImGuiPlugin imgui_plugin;
    igl::opengl::glfw::imgui::ImGuizmoWidget gizmo;

    const Eigen::Matrix4f T0;
    std::vector<Eigen::Vector3d> controlPoints;
    bool showControlLattice = true;
    int selectedVertex = -1;
    bool activeGizmo = false;


    const int resolution = 20;  
    const int patchSize = 4;    

    igl::opengl::glfw::Viewer viewer;



    Mesh();

private:

    inline std::vector<Eigen::Vector3d>* getControlPoints() { return &controlPoints; }

    double basisFunction0(double t);

    double basisFunction1(double t);

    double basisFunction2(double t);

    double basisFunction3(double t);

    void initializeControlMesh();

    void updateViewer();

    void computeBSplineSurface();

    void highlightControlPoints();

    Eigen::Vector3d Mesh::evaluate(const std::vector<Eigen::Vector3d>& cpGrid, double u, double v);

    void setupViewerCallbacks();
};