#ifndef COMMON_FUNCTIONS_HPP
#define COMMON_FUNCTIONS_HPP

#include "HUMLconfig.hpp"


/** In this namespace the functions commonly used by the classes of the library */
namespace CommonFunctions
{

using namespace std;
using namespace Eigen;
using namespace HUMotion;

/**
 * @brief Get the rotation matrix from the orientation in Roll-Pitch-Yaw
 * @param Rot
 * @param orr
 */
void getRPY_matrix(Matrix3f& Rot, orient orr);

/**
 * @brief Get the transformation matrix from the position and the orientation in Roll-Pitch-Yaw
 * @param Trans
 * @param orr
 * @param poss
 */
void getTrans_matrix(Matrix4f& Trans, orient orr, pos poss);

/**
 * @brief Get the norm of a vector from the position in Cartesian coordinates
 * @param poss
 * @return
 */
float getNorm(pos poss);

/**
 * @brief Get the axis of the orientation matrix from the orientation in Roll-Pitch-Yaw.
 *        id = 0 => retrieve the x axis, id = 1 => retrieve the y axis, id = 2 => retrieve the z axis
 * @param xt
 * @param orr
 * @param id
 */
void getRotAxis(std::vector<float>& xt, orient orr, int id);

/**
 * @brief Get the orientation in Roll-Pitch-Yaw from the transformation matrix
 * @param Trans
 * @param rpy
 */
void getRPY(Matrix4f Trans, std::vector<float>& rpy);




}// namespace CommonFunctions


#endif // COMMON_FUNCTIONS_HPP
