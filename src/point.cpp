#include "../include/point.hpp"


namespace HUMotion
{

Point::Point()
{

}

Point::Point(string name, pos ppos, orient oor)
{
    this->m_name = name;
    this->m_pos = ppos;
    this->m_or = oor;
}

Point::Point(const Point &pt)
{
    this->m_name = pt.m_name;
    this->m_pos = pt.m_pos;
    this->m_or = pt.m_or;
}

Point::~Point(){

}


void Point::setName(const std::string& name)
{

    this->m_name = name;
}


void Point::setPos(pos &ppos)
{

    this->m_pos = ppos;
}

void Point::setOr(orient &oor)
{

    this->m_or = oor;
}


string Point::getName() const
{

    return this->m_name;
}

pos Point::getPos() const
{

    return this->m_pos;
}

orient Point::getOr() const
{

   return this->m_or;
}


void Point::getXt(std::vector<float> &xt)
{

    this->getRotAxis(xt,0);
}


void Point::getYt(std::vector<float> &yt)
{

    this->getRotAxis(yt,1);
}

void Point::getZt(std::vector<float> &zt)
{

    this->getRotAxis(zt,2);
}

float Point::getNorm()
{

    return sqrt(pow((this->m_pos.Xpos),2)+pow((this->m_pos.Ypos),2)+pow((this->m_pos.Zpos),2));
}


void Point::RPY_matrix(Matrix3f &Rot)
{
    Rot = Matrix3f::Zero();

    float roll = this->m_or.roll; // around z
    float pitch = this->m_or.pitch; // around y
    float yaw = this->m_or.yaw; // around x

    // Rot = Rot_z * Rot_y * Rot_x

    Rot(0,0) = cos(roll)*cos(pitch);  Rot(0,1) = cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw); Rot(0,2) = sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw);
    Rot(1,0) = sin(roll)*cos(pitch);  Rot(1,1) = cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw); Rot(1,2) = sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw);
    Rot(2,0) = -sin(pitch);           Rot(2,1) = cos(pitch)*sin(yaw);                              Rot(2,2) = cos(pitch)*cos(yaw);

}

void Point::Trans_matrix(Matrix4f& Trans)
{

    Trans = Matrix4f::Zero();

    Matrix3f Rot;
    this->RPY_matrix(Rot);

    Trans(0,0) = Rot(0,0); Trans(0,1) = Rot(0,1); Trans(0,2) = Rot(0,2); Trans(0,3) = this->m_pos.Xpos;
    Trans(1,0) = Rot(1,0); Trans(1,1) = Rot(1,1); Trans(1,2) = Rot(1,2); Trans(1,3) = this->m_pos.Ypos;
    Trans(2,0) = Rot(2,0); Trans(2,1) = Rot(2,1); Trans(2,2) = Rot(2,2); Trans(2,3) = this->m_pos.Zpos;
    Trans(3,0) = 0;        Trans(3,1) = 0;        Trans(3,2) = 0;        Trans(3,3) = 1;

}


void Point::getRotAxis(std::vector<float>& xt, int id){

    Matrix3f Rot;
    this->RPY_matrix(Rot);
    Vector3f v = Rot.col(id);

    // get the components of the axis
    xt.push_back(v(0)); // x
    xt.push_back(v(1)); // y
    xt.push_back(v(2)); // z
}



} // namespace HUMotion
