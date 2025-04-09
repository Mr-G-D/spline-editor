#pragma once
#include <imgui.h>
#include <igl/readOFF.h>
#include <Eigen/Core>
#include <igl/opengl/glfw/Viewer.h>
#include <Eigen/Core>
#include "ImGuizmo.h"


#include "igl/opengl/glfw/imgui/ImGuiPlugin.h"
#include "igl/opengl/glfw/imgui/ImGuizmoWidget.h"
class Viewer
{
public: 
	Viewer();

	void renderMesh();

	bool loadMesh();

	void start();

	void updateViewer();
};

