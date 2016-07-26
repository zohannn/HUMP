#ifndef HUMPLANNER_H
#define HUMPLANNER_H


#include "scenario.hpp"
#include "task.hpp"

namespace HUMotion{

typedef boost::shared_ptr<Scenario> scenarioPtr;
typedef boost::shared_ptr<Task> taskPtr;

using namespace std;

class HUMPlanner
{
public:

    // constructors
    HUMPlanner(string name);
    HUMPlanner(string name, Scenario* scene);
    HUMPlanner(string name, Scenario *scene,Task* tt);

    // copy constructor
    HUMPlanner(const HUMPlanner& hp);

    //destructor
    ~HUMPlanner();


    //getters
    scenarioPtr getScenario();

    //setters
    void setScenario(scenarioPtr scene);

    void setTask(taskPtr tt);
    taskPtr getTask();
    void addProblem(Movement* mm);
    void addProblem(problemPtr prob);
    void clearProblems();
    string getProblemInfo(int pos);
    problemPtr getProblem(int pos);
    int getProblemNumber();



private:

    string name;
    scenarioPtr scene;
    taskPtr task;
};

} // namespace HUMotion

#endif // HUMPLANNER_H
