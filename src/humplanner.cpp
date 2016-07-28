#include "../include/humplanner.hpp"


namespace HUMotion {

// constructors
/**
 * @brief HUMPlanner::HUMPlanner
 * @param name
 */
HUMPlanner::HUMPlanner(string name = string ("Default Planner")):
    scene(nullptr),task(nullptr)
{

    this->name = name;
}

HUMPlanner::HUMPlanner(string name, Scenario* scene):
    task(nullptr)
{

    this->name=name;
    this->scene=scenarioPtr(scene);
}

HUMPlanner::HUMPlanner(string name, Scenario *scene, Task *tt){

    this->name=name;
    this->scene=scenarioPtr(scene);
    this->task=taskPtr(tt);

}

//copy constructor

HUMPlanner::HUMPlanner(const HUMPlanner &hp){

    this->name=hp.name;
    this->scene=scenarioPtr(new Scenario(*hp.scene.get()));
    this->task=taskPtr(new Task(*hp.task.get()));
}

// destructor

HUMPlanner::~HUMPlanner(){

}

//getters
/**
 * @brief HUMPlanner::getScenario
 * @return
 */
scenarioPtr HUMPlanner::getScenario(){

    return this->scene;
}

//setters
/**
 * @brief HUMPlanner::setScenario
 * @param scene
 */
void HUMPlanner::setScenario(scenarioPtr scene){

    this->scene=scene;
}

/**
 * @brief HUMPlanner::setTask
 * @param tt
 */
void HUMPlanner::setTask(taskPtr tt){

    this->task=tt;
}
/**
 * @brief HUMPlanner::getTask
 * @return
 */
taskPtr HUMPlanner::getTask(){

    return this->task;
}

/**
 * @brief HUMPlanner::addProblem
 * @param mm
 */
void HUMPlanner::addProblem(Movement *mm){

    this->task->addProblem(new Problem(mm,new Scenario(*scene.get())));
}
/**
 * @brief HUMPlanner::addProblem
 * @param prob
 */
void HUMPlanner::addProblem(problemPtr prob){

    this->task->addProblem(new Problem(*prob.get()));
}

/**
 * @brief HUMPlanner::getProblem
 * @param pos
 * @return
 */
problemPtr HUMPlanner::getProblem(int pos){

    return this->task->getProblem(pos);
}
/**
 * @brief HUMPlanner::getProblemInfo
 * @param pos
 * @return
 */
string HUMPlanner::getProblemInfo(int pos){

    return this->task->getProbInfo(pos);
}
/**
 * @brief HUMPlanner::getProblemNumber
 * @return
 */
int HUMPlanner::getProblemNumber(){

    return this->task->getProblemNumber();

}
/**
 * @brief HUMPlanner::clearProblems
 */
void HUMPlanner::clearProblems(){

    this->task->clearProblems();
}




} // namespace HUMotion
