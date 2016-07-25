#ifndef ENGAGEPOINT_H
#define ENGAGEPOINT_H

#include "HUMLconfig.hpp"
#include "common_functions.hpp"

namespace HUMotion {

using namespace std;
using namespace CommonFunctions;

//! The EngagePoint class
/**
 * @brief This class defines the concept of point of engagement on a object.
 * A point of engagement is a point that represents a point of contact and a reference when engaging/disengaging movements are planned.
 * An engaging movement is a movement that causes the combination (engagement) of two or more objects, consequently forming one single structure.
 * On the contrary a disengaging movement is a movement that causes the disengagement of two or more objects that are forming one single structure.
 */
class EngagePoint
{
public:

    /**
     * @brief EngagePoint, default constructor.
     */
    EngagePoint();

    /**
     * @brief EngagePoint, a constructor.
     * @param name
     * @param ppos
     * @param oor
     */
    EngagePoint(string name, pos ppos, orient oor);

    /**
     * @brief EngagePoint, a copy constructor.
     * @param eng
     */
    EngagePoint(const EngagePoint& eng);


    /**
     * @brief ~EngagePoint, a destructor.
     */
    ~EngagePoint();


    /**
     * @brief This method sets the name of the engage point.
     * @param name
     */
    void setName(const std::string& name);

    /**
     * @brief This method sets the position of the engage point.
     * @param ppos
     */
    void setPos(pos& ppos);

    /**
     * @brief This method sets the orientation of the engage point.
     * @param oor
     */
    void setOr(orient& oor);

    /**
     * @brief This method gets the name of the engage point.
     * @return
     */
    string getName() const;

    /**
     * @brief This method gets the position of the engage point.
     * @return
     */
    pos getPos() const;

    /**
     * @brief This method gets the orientation of the engage point.
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
     * @brief This method gets the norm of the vector pointing to the engage point.
     * @return
     */
    float getNorm();

    /**
     * @brief This method gets the orientation matrix of the engage point.
     * @param Rot
     */
    void RPY_matrix(Matrix3f& Rot);

private:

    string m_name; /**< name of the engage point */
    pos m_pos; /**< position of the engage point */
    orient m_or; /**< orientation of the engage point */




};

} // namespace HUMotion

#endif // ENGAGEPOINT_H
