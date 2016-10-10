#include "../include/engagepoint.hpp"

namespace HUMotion{


EngagePoint::EngagePoint()
{

}

EngagePoint::EngagePoint(string name, pos ppos, orient oor)
{
    this->m_name = name;
    this->m_pos = ppos;
    this->m_or = oor;
}

EngagePoint::EngagePoint(const EngagePoint &eng)
{
    this->m_name = eng.m_name;
    this->m_pos = eng.m_pos;
    this->m_or = eng.m_or;
}

EngagePoint::~EngagePoint()
{


}

} // namespace HUMotion
