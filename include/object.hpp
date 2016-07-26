#ifndef OBJECT_H
#define OBJECT_H


#include "target.hpp"
#include "engagepoint.hpp"

namespace HUMotion{

typedef boost::shared_ptr<Target> targetPtr;
typedef boost::shared_ptr<EngagePoint> engagePtr;


class Object: public Point
{

public:
    // constructors
    Object();
    explicit Object(string name);
    Object(string name, pos ppos, orient oor, dim ssize,
                    Target* pTR, Target* pTL,
                    EngagePoint* pEng);

    //copy constructor
    Object(const Object& obj);

    // destructor
    ~Object();

    // *** setters *** //

    /**
     * @brief This method sets the position of the point.
     * @param ppos
     */
    void setPos(pos& ppos, bool update_features);

    /**
     * @brief This method sets the orientation of the point.
     * @param oor
     */
    void setOr(orient& oor, bool update_features);

    void setSize(dim ssize);
    void setHandle(int h);
    void setHandleBody(int h);
    bool setTargetRight(targetPtr tr);
    bool setTargetLeft(targetPtr tl);
    void setTargetRightEnabled(bool c);
    void setTargetLeftEnabled(bool c);
    bool setEngagePoint(engagePtr eng);



    // *** getters *** //

    dim getSize();
    int getHandle();
    int getHandleBody();
    targetPtr getTargetRight();
    targetPtr getTargetLeft();
    engagePtr getEngagePoint();
    bool isTargetRightEnabled();
    bool isTargetLeftEnabled();
    float getRadius();
    string getInfoLine();



private:


    dim m_size; // size of the object
    int handle; // handle of the object
    int handle_body; // handle of the visible object

    bool m_targetRightEnabled; // boolean to show if there is a target in this object for the right arm
    bool m_targetLeftEnabled; // boolean to show if there is a target in this object for the left arm
    targetPtr p_targetRight; // target for the right arm
    targetPtr p_targetLeft; // target for the left arm
    engagePtr p_engage; // engage point
    bool setup_features; // true when  it is necessary to set up the features, false otherwise


    //methods
    void getTar_right_matrix(Matrix4f& mat);
    void getTar_left_matrix(Matrix4f& mat);
    void getEngage_matrix(Matrix4f& mat);

    /**
     * @brief Get the orientation in Roll-Pitch-Yaw from the transformation matrix
     * @param Trans
     * @param rpy
     */
    void getRPY(Matrix4f Trans, std::vector<float>& rpy);



};

} // namespace HUMotion


#endif // OBJECT_H
