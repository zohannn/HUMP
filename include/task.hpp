#ifndef TASK_H
#define TASK_H


#include "problem.hpp"


namespace HUMotion {


typedef boost::shared_ptr<Problem> problemPtr; /**< shared pointer to a problem */

//! The Task class
/**
 * @brief This class defines the concept of a task as a sequence of movements. \n
 * Each movement is obtained from the solution of the correspondent problem.
 */
class Task
{

public:
    /**
     * @brief Task, a costructor
     */
    Task();

    /**
     * @brief Task, a copy constructor
     * @param t
     */
    Task(const Task& t);

    /**
     * @brief ~Task
     */
    ~Task();

    /**
     * @brief This method gets information about the problem at the index pos.
     * @param pos
     * @return
     */
    string getProbInfo(int pos);

    /**
     * @brief This method gets the problem at the index pos.
     * @param pos
     * @return
     */
    problemPtr getProblem(int pos);

    /**
     * @brief This method gets the number of problems in the task.
     * @return
     */
    int getProblemNumber();

    /**
     * @brief This method adds a problem to the task
     * @param s
     */
    void addProblem(Problem* s);

    /**
     * @brief This method deletes all the problems of the task
     */
    void clearProblems();

private:

    std::vector<problemPtr> prolem_list; /**< problems of the task */


};

} // namespace HUMotion

#endif // TASK_H
