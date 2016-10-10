#ifndef SCENARIO_H
#define SCENARIO_H


#include "humanoid.hpp"
#include "object.hpp"

namespace HUMotion{

typedef boost::shared_ptr<Humanoid> humanoidPtr; /**< shared pointer to a humanoid */
typedef boost::shared_ptr<Object> objectPtr; /**< shared pointer to an object*/

//! The Scenario Class
/**
 * @brief This class defines the concept of scenario where the robot and/or a human partner is acting
 */
class Scenario
{

public:

    /**
     * @brief Scenario, a constructor
     * @param name
     * @param id
     */
    Scenario(string name, int id);

    /**
     * @brief Scenario, a copy constructor
     * @param scene
     */
    Scenario(const Scenario& scene);


    /**
     * @brief ~Scenario, a destructor.
     */
    ~Scenario();


    /**
     * @brief This method sets the name of the scenario
     * @param name
     */
    void setName(string& name);

    /**
     * @brief This method sets the ID of the scenario
     * @param id
     */
    void setID(int id);

    /**
     * @brief This method insert an object in the vector of objects at the position pos
     * @param pos
     * @param obj
     */
    void setObject(int pos, objectPtr obj);

    /**
     * @brief This method gets the name of the scenario
     * @return
     */
    string getName();

    /**
     * @brief This method gets the ID of the scenario
     * @return
     */
    int getID();

    /**
     * @brief This method gets a pointer to the humanoid robot in the scenario
     * @return
     */
    humanoidPtr getHumanoid();

    /**
     * @brief This method get the list of the objects in the scenario
     * @param objs
     */
    void getObjects(std::vector<objectPtr>& objs);

    /**
     * @brief This method adds a new object to the scenario
     * @param obj
     */
    void addObject(Object* obj);

    /**
     * @brief This method gets the object at the position pos in the vector of objects
     * @param pos
     * @return
     */
    objectPtr getObject(int pos);

    /**
     * @brief This method gets the object named "obj_name"
     * @param obj_name
     * @return
     */
    objectPtr getObject(std::string obj_name);

    /**
     * @brief This method adds the humanoid robot to the scenario
     * @param hh
     */
    void addHumanoid(Humanoid* hh);


private:

    string m_name; /**< the name of the scenario*/
    int m_scenarioID; /**< the ID of the scenario*/
    std::vector<objectPtr> objs_list; /**< the objects in the scenario */
    humanoidPtr hPtr; /**< the humanoid robot in the scenario */



};

} // namespace HUMotion

#endif // SCENARIO_H
