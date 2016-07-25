#include "../include/engagepoint.hpp"


namespace HUMotion{


EngagePoint::EngagePoint()
{

    this->m_name="";
    this->m_pos.Xpos=0; this->m_pos.Ypos=0; this->m_pos.Ypos=0;
    this->m_or.pitch=0; this->m_or.roll=0; this->m_or.yaw=0;

}


EngagePoint::EngagePoint(string name, pos ppos, orient oor):
    m_name(name),m_pos(ppos),m_or(oor)
{


}


EngagePoint::EngagePoint(const EngagePoint &eng)
{

   m_name = eng.m_name;
   m_pos = eng.m_pos;
   m_or = eng.m_or;
}


EngagePoint::~EngagePoint()
{

}


void EngagePoint::setName(const std::string& name)
{

    this->m_name = name;
}


void EngagePoint::setPos(pos &ppos)
{

    this->m_pos = ppos;
}

void EngagePoint::setOr(orient &oor)
{

    this->m_or = oor;
}


string EngagePoint::getName() const
{

    return this->m_name;
}

pos EngagePoint::getPos() const
{

    return this->m_pos;
}

orient EngagePoint::getOr() const
{

   return this->m_or;
}


void EngagePoint::getXt(std::vector<float> &xt)
{

    CommonFunctions::getRotAxis(xt,this->getOr(),0);
}


void EngagePoint::getYt(std::vector<float> &yt)
{

    CommonFunctions::getRotAxis(yt,this->getOr(),1);
}

void EngagePoint::getZt(std::vector<float> &zt)
{

    CommonFunctions::getRotAxis(zt,this->getOr(),2);
}

float EngagePoint::getNorm()
{

    return CommonFunctions::getNorm(this->m_pos);
}


void EngagePoint::RPY_matrix(Matrix3f &Rot)
{
    getRPY_matrix(Rot,this->m_or);
}



} // namespace HUMotion
