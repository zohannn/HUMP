#ifndef TARGET_H
#define TARGET_H

#include "point.hpp"


namespace HUMotion {

using namespace std;

//! The Target class
/**
 * @brief This class defines the target that is taken into account for the constraints on the hand.
 * The target is a point designed to be placed on a object that has to be manipulated.
 */
class Target:public Point
{
public:
    /**
     * @brief Target, default constructor.
     */
    Target();

    /**
     * @brief Target, a constructor.
     * @param name
     * @param ppos
     * @param oor
     */
    Target(string name, pos ppos, orient oor);

    /**
     * @brief Target, a copy constructor.
     * @param tar
     */
    Target(const Target& tar);


    /**
     * @brief ~Target, a destructor.
     */
    ~Target();



};

} // namespace HUMotion

#endif // TARGET_H
