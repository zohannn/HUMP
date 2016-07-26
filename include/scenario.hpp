#ifndef SCENARIO_H
#define SCENARIO_H


#include "humanoid.hpp"
#include "object.hpp"

namespace HUMotion{


typedef boost::shared_ptr<Humanoid> humanoidPtr;
typedef boost::shared_ptr<Object> objectPtr;

class Scenario
{

public:

    // contrsuctors
    Scenario(string name, int id);

    //copy constructor
    Scenario(const Scenario& scene);

    //destructor
    ~Scenario();


    //setters
    void setName(string& name);
    void setID(int id);
    void setObject(int pos, objectPtr obj);

    //getters
    string getName();
    int getID();
    humanoidPtr getHumanoid();
    void getObjects(std::vector<objectPtr>& objs);

    // objects
    void addObject(Object* obj);
    objectPtr getObject(int pos);
    objectPtr getObject(std::string obj_name);

    //humanoid
    void addHumanoid(Humanoid* hh);


private:

    string m_name;
    int m_scenarioID;
    std::vector<objectPtr> objs_list; // list of the objects in the scenario
    humanoidPtr hPtr;



};

} // namespace HUMotion

#endif // SCENARIO_H
