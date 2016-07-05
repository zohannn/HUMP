#ifndef OBJECT_H
#define OBJECT_H


#include "target.hpp"
#include "engagepoint.hpp"

namespace HUMotion{

typedef boost::shared_ptr<Target> targetPtr;
typedef boost::shared_ptr<EngagePoint> engagePtr;


class Object
{

public:
    // constructors
    explicit Object();
    explicit Object(string name);
    explicit Object(string name, pos ppos, orient oor, dim ssize,
                    Target* pTR, Target* pTL,
                    EngagePoint* pEng);

    //copy constructor
    Object(const Object& obj);

    // destructor
    ~Object();

    // *** setters *** //
    void setName(string name);
    bool setPos(pos ppos, bool update_features);
    bool setOr(orient oor, bool update_features);
    void setSize(dim ssize);
    void setHandle(int h);
    void setHandleBody(int h);
    bool setTargetRight(targetPtr tr);
    bool setTargetLeft(targetPtr tl);
    void setTargetRightEnabled(bool c);
    void setTargetLeftEnabled(bool c);
    bool setEngagePoint(engagePtr eng);



    // *** getters *** //
    string getName();
    pos getPos();
    orient getOr();
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
    float getNorm();
    void getXt(std::vector<float>& xt);
    void getYt(std::vector<float>& yt);
    void getZt(std::vector<float>& zt);

    void RPY_matrix(Matrix3f& Rot);




private:

    string m_name; // name of the object
    pos m_pos; // posiion of the center
    orient m_or; // orientation of the object
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

    void getRPY(Matrix4f Trans, std::vector<float> &rpy); // get RPY angles from the transformation matrix




};

} // namespace HUMotion


#endif // OBJECT_H
