#include "../include/humplanner.hpp"


namespace HUMotion {


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

HUMPlanner::HUMPlanner(string name, Scenario *scene, Task *tt)
{

    this->name=name;
    this->scene=scenarioPtr(scene);
    this->task=taskPtr(tt);

}


HUMPlanner::HUMPlanner(const HUMPlanner &hp)
{

    this->name=hp.name;
    this->scene=scenarioPtr(new Scenario(*hp.scene.get()));
    this->task=taskPtr(new Task(*hp.task.get()));
}

HUMPlanner::~HUMPlanner()
{

}

scenarioPtr HUMPlanner::getScenario()
{

    return this->scene;
}

void HUMPlanner::setScenario(scenarioPtr scene)
{

    this->scene=scene;
}

void HUMPlanner::setTask(taskPtr tt)
{

    this->task=tt;
}

taskPtr HUMPlanner::getTask()
{

    return this->task;
}


void HUMPlanner::addProblem(movementPtr mm)
{

    this->task->addProblem(new Problem(new Movement(*mm.get()),new Scenario(*scene.get())));
}

void HUMPlanner::addProblem(problemPtr prob)
{

    this->task->addProblem(new Problem(*prob.get()));
}


problemPtr HUMPlanner::getProblem(int pos)
{

    return this->task->getProblem(pos);
}

string HUMPlanner::getProblemInfo(int pos)
{

    return this->task->getProbInfo(pos);
}

int HUMPlanner::getProblemNumber()
{

    return this->task->getProblemNumber();

}
void HUMPlanner::clearProblems()
{

    this->task->clearProblems();
}




} // namespace HUMotion
