#include "../include/task.hpp"

namespace HUMotion {


Task::Task()
{

}


Task::Task(const Task &t)
{

    this->prolem_list=t.prolem_list;
}


Task::~Task()
{

}

void Task::addProblem(Problem* s)
{

    this->prolem_list.push_back(problemPtr(s));

}

string Task::getProbInfo(int pos)
{

    std::vector<problemPtr>::iterator ii = this->prolem_list.begin();
    advance(ii,pos);
    return (*ii)->getInfoLine();
}

int Task::getProblemNumber()
{

    return this->prolem_list.size();
}

problemPtr Task::getProblem(int pos)
{

    std::vector<problemPtr>::iterator ii = this->prolem_list.begin();
    advance(ii,pos);

    return (*ii);
}

void Task::clearProblems()
{

    if(!this->prolem_list.empty()){
        this->prolem_list.clear();
    }
}

} // namespace HUMotion
