#include "../include/target.hpp"


namespace HUMotion{


Target::Target()
{

}

Target::Target(string name, pos ppos, orient oor)
{
    this->m_name = name;
    this->m_pos = ppos;
    this->m_or = oor;
}

Target::Target(const Target &tar)
{
    this->m_name = tar.m_name;
    this->m_pos = tar.m_pos;
    this->m_or = tar.m_or;
}

Target::~Target()
{


}

} // namespace HUMotion


