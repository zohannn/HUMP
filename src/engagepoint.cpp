#include "../include/engagepoint.hpp"


namespace HUMotion{

// constructors

EngagePoint::EngagePoint(){

    this->m_name="";
    this->m_pos.Xpos=0; this->m_pos.Ypos=0; this->m_pos.Ypos=0;
    this->m_or.pitch=0; this->m_or.roll=0; this->m_or.yaw=0;

}

/**
 * @brief EngagePoint::EngagePoint
 * @param name
 * @param ppos
 * @param oor
 */
EngagePoint::EngagePoint(string name, pos ppos, orient oor):
    m_name(name),m_pos(ppos),m_or(oor)
{


}

// copy constructor
/**
 * @brief EngagePoint::EngagePoint
 * @param eng
 */
EngagePoint::EngagePoint(const EngagePoint &eng){

   m_name = eng.m_name;
   m_pos = eng.m_pos;
   m_or = eng.m_or;
}

// destructor
/**
 * @brief EngagePoint::~EngagePoint
 */
EngagePoint::~EngagePoint(){

}

// setters
/**
 * @brief EngagePoint::setName
 * @param name
 */
void EngagePoint::setName(const std::string& name){

    this->m_name = name;
}

/**
 * @brief EngagePoint::setPos
 * @param ppos
 */
void EngagePoint::setPos(pos &ppos){

    this->m_pos = ppos;
}
/**
 * @brief EngagePoint::setOr
 * @param oor
 */
void EngagePoint::setOr(orient &oor){

    this->m_or = oor;
}

// getters
/**
 * @brief EngagePoint::getName
 * @return
 */
string EngagePoint::getName() const {

    return this->m_name;
}
/**
 * @brief EngagePoint::getPos
 * @return
 */
pos EngagePoint::getPos() const {

    return this->m_pos;
}
/**
 * @brief EngagePoint::getOr
 * @return
 */
orient EngagePoint::getOr() const{

   return this->m_or;
}

/**
 * @brief EngagePoint::getXt
 * @param xt
 */
void EngagePoint::getXt(std::vector<float> &xt){

    Matrix3f Rot;
    this->RPY_matrix(Rot);
    Vector3f v = Rot.col(0);

    xt.push_back(v(0));
    xt.push_back(v(1));
    xt.push_back(v(2));
}

/**
 * @brief EngagePoint::getYt
 * @param yt
 */
void EngagePoint::getYt(std::vector<float> &yt){

    Matrix3f Rot;
    this->RPY_matrix(Rot);
    Vector3f v = Rot.col(1);

    yt.push_back(v(0));
    yt.push_back(v(1));
    yt.push_back(v(2));

}
/**
 * @brief EngagePoint::getZt
 * @param zt
 */
void EngagePoint::getZt(std::vector<float> &zt){

    Matrix3f Rot;
    this->RPY_matrix(Rot);
    Vector3f v = Rot.col(2);

    zt.push_back(v(0));
    zt.push_back(v(1));
    zt.push_back(v(2));
}
/**
 * @brief EngagePoint::getNorm
 * @return
 */
float EngagePoint::getNorm(){

    return sqrt(pow((m_pos.Xpos),2)+pow((m_pos.Ypos),2)+pow((m_pos.Zpos),2));
}

/**
 * @brief EngagePoint::RPY_matrix
 * @param Rot
 */
void EngagePoint::RPY_matrix(Matrix3f &Rot){

    float roll = this->m_or.roll;
    float pitch = this->m_or.pitch;
    float yaw = this->m_or.yaw;

    Rot(0,0) = cos(roll)*cos(pitch);  Rot(0,1) = cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw); Rot(0,2) = sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw);
    Rot(1,0) = sin(roll)*cos(pitch);  Rot(1,1) = cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw); Rot(1,2) = sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw);
    Rot(2,0) = -sin(pitch);           Rot(2,1) = cos(pitch)*sin(yaw);

}



} // namespace HUMotion
