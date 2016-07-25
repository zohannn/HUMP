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

    CommonFunctions::getRotAxis(xt,this->getOr(),0);

}
/**
 * @brief Target::getYt
 * @param yt
 */
void Target::getYt(std::vector<float> &yt){

    CommonFunctions::getRotAxis(yt,this->getOr(),1);
}
/**
 * @brief Target::getZt
 * @param zt
 */
void Target::getZt(std::vector<float> &zt){

    CommonFunctions::getRotAxis(zt,this->getOr(),2);
}
/**
 * @brief Target::getNorm
 * @return
 */
float Target::getNorm(){

    return CommonFunctions::getNorm(this->m_pos);
}

/**
 * @brief Target::RPY_matrix
 * @param Rot
 */
void Target::RPY_matrix(Matrix3f &Rot){

    CommonFunctions::getRPY_matrix(Rot,this->m_or);
}



} // namespace HUMotion
