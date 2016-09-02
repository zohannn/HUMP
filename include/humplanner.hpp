#ifndef HUMPLANNER_H
#define HUMPLANNER_H


#include "scenario.hpp"
#include "task.hpp"

namespace HUMotion{

typedef boost::shared_ptr<Scenario> scenarioPtr;
typedef boost::shared_ptr<Task> taskPtr; /**< shared pointer to a task object */

using namespace std;

//! The Object class
/**
 * @brief This class defines the concept of the Human-like Motion planner
 */
class HUMPlanner
{
public:

    /**
     * @brief HUMPlanner, a constructor
     * @param name
     */
    explicit HUMPlanner(string name);

    /**
     * @brief HUMPlanner, a constructor
     * @param name
     * @param scene
     */
    HUMPlanner(string name, Scenario* scene);

    /**
     * @brief HUMPlanner, a constructor
     * @param name
     * @param scene
     * @param tt
     */
    HUMPlanner(string name, Scenario *scene,Task* tt);

    /**
     * @brief HUMPlanner, a copy constructor
     * @param hp
     */
    HUMPlanner(const HUMPlanner& hp);

    /**
     * @brief ~HUMPlanner, a destructor
     */
    ~HUMPlanner();


    /**
     * @brief This method gets the scenario where the planner is currently working
     * @return
     */
    scenarioPtr getScenario();

    /**
     * @brief This method sets the scenario where the planner is going to work on
     * @param scene
     */
    void setScenario(scenarioPtr scene);

    /**
     * @brief This method sets the task that has to be planned
     * @param tt
     */
    void setTask(taskPtr tt);

    /**
     * @brief This method gets the task that has to be planned
     * @return
     */
    taskPtr getTask();

    /**
     * @brief This method adds a problem to the current task by the definition of a new movement
     * @param mm
     */
    void addProblem(movementPtr mm);

    /**
     * @brief This method adds a problem to the current task
     * @param prob
     */
    void addProblem(problemPtr prob);

    /**
     * @brief This method clears the problems of the current task
     */
    void clearProblems();

    /**
     * @brief This method gets the informtion related to the problem at the position pos in the task
     * @param pos
     * @return
     */
    string getProblemInfo(int pos);

    /**
     * @brief This method gets the problem at the position pos in the task
     * @param pos
     * @return
     */
    problemPtr getProblem(int pos);

    /**
     * @brief This method gets the total number of problems in the current task
     * @return
     */
    int getProblemNumber();



private:

    string name;/**< name of the planner */
    scenarioPtr scene;/**< scenario where the planner works */
    taskPtr task;/**< task that has to be planned */
};

} // namespace HUMotion

#endif // HUMPLANNER_H
