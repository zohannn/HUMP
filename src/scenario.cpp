#include "../include/scenario.hpp"

namespace HUMotion{

//constructors
/**
 * @brief Scenario::Scenario
 * @param name
 * @param id
 */
Scenario::Scenario(string name, int id)
{
    this->m_name = name;
    this->m_scenarioID = id;
}

//copy constructor
/**
 * @brief Scenario::Scenario
 * @param scene
 */
Scenario::Scenario(const Scenario &scene){

    this->m_name=scene.m_name;
    this->m_scenarioID=scene.m_scenarioID;

    if(!scene.objs_list.empty()){
        this->objs_list = std::vector<objectPtr>(scene.objs_list.size());
        std::copy(scene.objs_list.begin(),scene.objs_list.end(),this->objs_list.begin());
    }

    this->hPtr=humanoidPtr(new Humanoid(*scene.hPtr.get()));
}

//destructor
/**
 * @brief Scenario::~Scenario
 */
Scenario::~Scenario(){


}


// setters
/**
 * @brief Scenario::setName
 * @param name
 */
void Scenario::setName(string &name){

    this->m_name = name;
}
/**
 * @brief Scenario::setID
 * @param id
 */
void Scenario::setID(int id){

    this->m_scenarioID = id;
}
/**
 * @brief Scenario::setObject
 * @param pos
 * @param obj
 */
void Scenario::setObject(int pos, objectPtr obj){

    this->objs_list.at(pos) = objectPtr(new Object(*obj.get()));

}

// getters
/**
 * @brief Scenario::getName
 * @return
 */
string Scenario::getName(){

    return this->m_name;
}
/**
 * @brief Scenario::getID
 * @return
 */
int Scenario::getID(){

    return this->m_scenarioID;
}
/**
 * @brief Scenario::getHumanoid
 * @return
 */
humanoidPtr Scenario::getHumanoid(){

    return hPtr;
}
/**
 * @brief Scenario::getObjects
 * @param objs
 */
void Scenario::getObjects(std::vector<objectPtr> &objs){

    objs=this->objs_list;
    /*
    if(!this->objs_list.empty()){
        objs = std::vector<objectPtr>(this->objs_list.size());
        std::copy(this->objs_list.begin(),this->objs_list.end(),objs.begin());
    }
    */

}

//objects
/**
 * @brief Scenario::addObject
 * @param ob
 */
void Scenario::addObject(Object* ob){

    this->objs_list.push_back(objectPtr(ob));
}
/**
 * @brief Scenario::getObject
 * @param pos
 * @return
 */
objectPtr Scenario::getObject(int pos){

    std::vector<objectPtr>::iterator ii = this->objs_list.begin();
    advance(ii,pos);

    return (*ii);

}
/**
 * @brief Scenario::getObject
 * @param obj_name
 * @return
 */
objectPtr Scenario::getObject(string obj_name){

    objectPtr obj = NULL;

    for(int i=0; i<this->objs_list.size();++i){
        string name = this->objs_list.at(i)->getName();
        if(boost::iequals(name,obj_name)){
            obj=this->objs_list.at(i);
            break;
        }
    }
    return obj;
}

//humanoid
/**
 * @brief Scenario::addHumanoid
 * @param hh
 */
void Scenario::addHumanoid(Humanoid* hh){

    this->hPtr = humanoidPtr(hh);
}



 } // namespace HUMotion
