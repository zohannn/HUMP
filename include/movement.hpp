#ifndef MOVEMENT_H
#define MOVEMENT_H


#include "object.hpp"

namespace HUMotion{

typedef boost::shared_ptr<Object> objectPtr;

class Movement
{

public:
    //constructors
    Movement(int type, int arm);
    Movement(int type, int arm, objectPtr obj);
    Movement(int type, int arm, objectPtr obj, int grip_id, bool prec);
    Movement(int type, int arm, objectPtr obj, objectPtr obj_eng, int grip_id, bool prec);

    // copy constructor
    Movement(const Movement& mov);

    //destructors
    ~Movement();


    //setters
    void setType(int t);
    void setStrType(string str);
    void setGrip(int g);
    void setObject(objectPtr obj);
    void setObjectInit(objectPtr obj);
    void setObjectEng(objectPtr obj_eng);
    void setArm(int a);
    void setExecuted(bool exec);


    //getters
    int getType();
    string getStrType();
    int getGrip();
    string getGripStr();
    objectPtr getObject();
    objectPtr getObjectInit();
    objectPtr getObjectEng();
    string getInfoLine();
    int getArm();
    string getArmInfo();
    bool getExecuted();

private:
    int arm;  // 0 (both arms), 1 (right-arm), 2 (left-arm)
    int type; // type of movement (code)
    string strType; // type of movement (string)
    int grip_code; // type of the grip (0 means no grip)
    string grip_str; // type of the grip (string)
    objectPtr obj; // object involved involved in the movement
    objectPtr obj_init; // object at the beginning of the movement
    objectPtr obj_eng; // object involved in the engagement
    bool executed; // true if the movement has been executed

    // Types of Movements and related code
    // ||||||||||||||||||||||||||
    // |||||||||||||||||| Code ||
    // ||Reach-to-grasp |   0  ||
    // ||Reaching       |   1  ||
    // ||Transport      |   2  ||
    // ||Engage         |   3  ||
    // ||Disengage      |   4  ||
    // ||Go home        |   5  ||
    // ||||||||||||||||||||||||||

    // Types of Grip and related code
    // |||||||||||||||||||||||||||||||||||||||||||||||||
    // |||||||||||||||||||| Precision |  Full | Index ||
    // ||Side thumb left  |    111    |  211  |   0   ||
    // ||Side thumb right |    112    |  212  |   1   ||
    // ||Side thumb up    |    113    |  213  |   2   ||
    // ||Side thumb down  |    114    |  214  |   3   ||
    // ||Above            |    121    |  221  |   4   ||
    // ||Below            |    122    |  222  |   5   ||
    // |||||||||||||||||||||||||||||||||||||||||||||||||


};

} // namespace HUMotion

#endif // MOVEMENT_H
