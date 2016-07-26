#include "../include/task.hpp"

namespace HUMotion {


//constructors
/**
 * @brief Task::Task
 */
Task::Task()
{

}

// copy constructor

Task::Task(const Task &t){

    this->prolem_list=t.prolem_list;
    /*
    if(!t.prolem_list.empty()){
        std::copy(t.prolem_list.begin(),t.prolem_list.end(),this->prolem_list.begin());
    }
    */
}

//destructor
/**
 * @brief Task::~Task
 */
Task::~Task(){

}

/**
 * @brief Task::addProblem
 * @param s
 */
void Task::addProblem(Problem* s){

    this->prolem_list.push_back(problemPtr(s));

}

string Task::getProbInfo(int pos){

    std::vector<problemPtr>::iterator ii = this->prolem_list.begin();
    advance(ii,pos);

    return (*ii)->getInfoLine();
}
/**
 * @brief Task::getProblemNumber
 * @return
 */
int Task::getProblemNumber(){

    return this->prolem_list.size();
}

problemPtr Task::getProblem(int pos){

    std::vector<problemPtr>::iterator ii = this->prolem_list.begin();
    advance(ii,pos);

    return (*ii);
}
/**
 * @brief Task::clearProblems
 */
void Task::clearProblems(){

    if(!this->prolem_list.empty()){
        this->prolem_list.clear();
    }
}

} // namespace HUMotion
