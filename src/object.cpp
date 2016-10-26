#include "../include/object.hpp"

namespace HUMotion {

Object::Object()
{
    this->name="";
    this->xpos=0;
    this->ypos=0;
    this->zpos=0;
    this->roll=0;
    this->pitch=0;
    this->yaw=0;
    this->xsize=0;
    this->ysize=0;
    this->zsize=0;
}

Object::Object(string name)
{
    this->name=name;
    this->xpos=0;
    this->ypos=0;
    this->zpos=0;
    this->roll=0;
    this->pitch=0;
    this->yaw=0;
    this->xsize=0;
    this->ysize=0;
    this->zsize=0;
}

Object::Object(const Object &obj)
{
    this->name=obj.name;
    this->xpos=obj.xpos;
    this->ypos=obj.ypos;
    this->zpos=obj.zpos;
    this->roll=obj.roll;
    this->pitch=obj.pitch;
    this->yaw=obj.yaw;
    this->xsize=obj.xsize;
    this->ysize=obj.ysize;
    this->zsize=obj.zsize;
}

Object:: ~Object()
{

}

void Object::setName(string name)
{
    this->name=name;
}


string Object::getName()
{
    return this->name;
}


void Object::setPos(vector<double> &pos)
{
    if(!pos.empty()){
        this->xpos=pos.at(0);
        this->ypos=pos.at(1);
        this->zpos=pos.at(2);
    }
}

void Object::getPos(vector<double> &pos)
{
    pos.clear();
    pos.push_back(this->xpos);
    pos.push_back(this->ypos);
    pos.push_back(this->zpos);
}

void Object::setOr(vector<double> &orr)
{
    if(!orr.empty()){
        this->roll=orr.at(0);
        this->pitch=orr.at(1);
        this->yaw=orr.at(2);
    }
}

void Object::getOr(vector<double> &orr)
{
    orr.clear();
    orr.push_back(this->roll);
    orr.push_back(this->pitch);
    orr.push_back(this->yaw);
}

void Object::setSize(vector<double> &dim)
{
    if(!dim.empty()){
        this->xsize=dim.at(0);
        this->ysize=dim.at(1);
        this->zsize=dim.at(2);
    }
}

void Object::getSize(vector<double> &dim)
{
    dim.clear();
    dim.push_back(this->xsize);
    dim.push_back(this->ysize);
    dim.push_back(this->zsize);
}

void Object::setParams(vector<double> &pos, vector<double> &orr, vector<double> &dim)
{
    this->setPos(pos);
    this->setOr(orr);
    this->setSize(dim);
}

void Object::getParams(vector<double> &pos, vector<double> &orr, vector<double> &dim)
{
    this->getPos(pos);
    this->getOr(orr);
    this->getSize(dim);
}



}// namespace HUMotion
