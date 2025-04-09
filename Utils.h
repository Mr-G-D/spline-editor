#pragma once


#include <Eigen/Core>
#include <igl/opengl/glfw/Viewer.h>

class Utils
{
public:
	static Eigen::Vector3d unproject(const Eigen::Vector2d& screen_coords, const igl::opengl::glfw::Viewer& viewer);

	static Eigen::Vector2d project(const Eigen::Vector3d& point, const igl::opengl::glfw::Viewer& viewer);

};

