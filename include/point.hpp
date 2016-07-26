#ifndef POINT_HPP
#define POINT_HPP

#include "HUMLconfig.hpp"

namespace HUMotion
{

using namespace std;
using namespace Eigen;

//! The Point class
/**
 * @brief This class defines the concept of a point in the cartesian space.
 * It is defined by 6 degrees of freedom: 3 for position and 3 for orientation
 */
class Point
{
public:

    /**
     * @brief Point, default constructor.
     */
    Point();

    /**
     * @brief Point, a constructor.
     * @param name
     * @param ppos
     * @param oor
     */
    Point(string name, pos ppos, orient oor);

    /**
     * @brief Point, a copy constructor.
     * @param pt
     */
    Point(const Point& pt);


    /**
     * @brief ~Point, a destructor.
     */
    virtual ~Point();


    /**
     * @brief This method sets the name of the point.
     * @param name
     */
    void setName(const std::string& name);

    /**
     * @brief This method sets the position of the point.
     * @param ppos
     */
    void setPos(pos& ppos);

    /**
     * @brief This method sets the orientation of the point.
     * @param oor
     */
    void setOr(orient& oor);

    /**
     * @brief This method gets the name of the point.
     * @return
     */
    string getName() const;

    /**
     * @brief This method gets the position of the point.
     * @return
     */
    pos getPos() const;

    /**
     * @brief This method gets the orientation of the point.
     * @return
     */
    orient getOr() const;

    /**
     * @brief This method gets the x axis of the orientation frame.
     * @param xt
     */
    void getXt(std::vector<float>& xt);

    /**
     * @brief This method gets the y axis of the orientation frame.
     * @param yt
     */
    void getYt(std::vector<float>& yt);

    /**
     * @brief This method gets the y axis of the orientation frame.
     * @param zt
     */
    void getZt(std::vector<float>& zt);

    /**
     * @brief This method gets the norm of the vector pointing to the point.
     * @return
     */
    float getNorm();

    /**
     * @brief This method gets the orientation matrix of the point.
     * @param Rot
     */
    void RPY_matrix(Matrix3f& Rot);

    /**
     * @brief Get the transformation matrix from the position and the orientation in Roll-Pitch-Yaw
     * @param Trans
     */
    void Trans_matrix(Matrix4f& Trans);

protected:

    string m_name; /**< name of the point */
    pos m_pos; /**< position of the point */
    orient m_or; /**< orientation of the point */

    /**
     * @brief Get the axis of the orientation matrix from the orientation in Roll-Pitch-Yaw.
     *        id = 0 => retrieve the x axis, id = 1 => retrieve the y axis, id = 2 => retrieve the z axis
     * @param xt
     * @param id
     */
    void getRotAxis(std::vector<float>& xt, int id);


};

} // namespace HUMotion




#endif // POINT_HPP
