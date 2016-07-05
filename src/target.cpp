#include "../include/target.hpp"

namespace HUMotion{

// constructors

Target::Target(){

    this->m_name="";
    this->m_pos.Xpos=0; this->m_pos.Ypos=0; this->m_pos.Ypos=0;
    this->m_or.pitch=0; this->m_or.roll=0; this->m_or.yaw=0;
}

/**
 * @brief Target::Target
 * @param name
 * @param ppos
 * @param oor
 */
Target::Target(string name, pos ppos, orient oor):
    m_name(name),m_pos(ppos),m_or(oor)
{

}


// copy constructor

/**
 * @brief Target::Target
 * @param tar
 */
Target::Target(const Target &tar){

    m_name = tar.m_name;
    m_pos = tar.m_pos;
    m_or = tar.m_or;
}


// destructor

/**
 * @brief Target::~Target
 */

Target::~Target(){

}



//setters
/**
 * @brief Target::setName
 * @param name
 */
void Target::setName(const std::string& name){

   this->m_name=name;
}


/**
 * @brief Target::setPos
 * @param ppos
 */
void Target::setPos(pos &ppos){

    this->m_pos=ppos;
}
/**
 * @brief Target::setOr
 * @param oor
 */
void Target::setOr(orient &oor){

    this->m_or=oor;
}

// getters
/**
 * @brief Target::getName
 * @return
 */
string Target::getName() const{

    return this->m_name;
}
/**
 * @brief Target::getPos
 * @return
 */
pos Target::getPos() const{

    return this->m_pos;
}
/**
 * @brief Target::getOr
 * @return
 */
orient Target::getOr() const{

    return this->m_or;
}
/**
 * @brief Target::getXt
 * @param xt
 */
void Target::getXt(std::vector<float> &xt){

    Matrix3f Rot;
    this->RPY_matrix(Rot);
    Vector3f v = Rot.col(0);

    xt.push_back(v(0));
    xt.push_back(v(1));
    xt.push_back(v(2));

}
/**
 * @brief Target::getYt
 * @param yt
 */
void Target::getYt(std::vector<float> &yt){

    Matrix3f Rot;
    this->RPY_matrix(Rot);
    Vector3f v = Rot.col(1);

    yt.push_back(v(0));
    yt.push_back(v(1));
    yt.push_back(v(2));
}
/**
 * @brief Target::getZt
 * @param zt
 */
void Target::getZt(std::vector<float> &zt){

    Matrix3f Rot;
    this->RPY_matrix(Rot);
    Vector3f v = Rot.col(2);

    zt.push_back(v(0));
    zt.push_back(v(1));
    zt.push_back(v(2));
}
/**
 * @brief Target::getNorm
 * @return
 */
float Target::getNorm(){

    return sqrt(pow((m_pos.Xpos),2)+pow((m_pos.Ypos),2)+pow((m_pos.Zpos),2));
}

/**
 * @brief Target::RPY_matrix
 * @param Rot
 */
void Target::RPY_matrix(Matrix3f &Rot){

    float roll = this->m_or.roll;
    float pitch = this->m_or.pitch;
    float yaw = this->m_or.yaw;


    Rot(0,0) = cos(roll)*cos(pitch);  Rot(0,1) = cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw); Rot(0,2) = sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw);
    Rot(1,0) = sin(roll)*cos(pitch);  Rot(1,1) = cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw); Rot(1,2) = sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw);
    Rot(2,0) = -sin(pitch);           Rot(2,1) = cos(pitch)*sin(yaw);                                Rot(2,2) = cos(pitch)*cos(yaw);


}




} // namespace HUMotion
