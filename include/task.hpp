#ifndef TASK_H
#define TASK_H


#include "problem.hpp"


namespace HUMotion {


typedef boost::shared_ptr<Problem> problemPtr;


class Task
{

public:
    //constructors
    Task();

    // copy constructor
    Task(const Task& t);

    //destructor
    ~Task();

    //getters
    string getProbInfo(int pos);
    problemPtr getProblem(int pos);
    int getProblemNumber();

    void addProblem(Problem* s);
    void clearProblems();

private:

    std::vector<problemPtr> prolem_list;


};

} // namespace HUMotion

#endif // TASK_H
