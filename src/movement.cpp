#include "../include/movement.hpp"



namespace HUMotion {


//constructors
/**
 * @brief Movement::Movement
 * @param type
 * @param arm
 */
Movement::Movement(int type, int arm){

    this->obj=objectPtr(new Object());
    this->obj_init=objectPtr(new Object());
    this->obj_eng=objectPtr(new Object());

    this->type=type;
    switch (type){

    case 0:

        this->strType = string("Reach-to-grasp");
        break;

    case 1:

        this->strType = string("Reaching");
        break;

    case 2:

        this->strType = string("Transport");
        break;

    case 3:

        this->strType = string("Engage");
        break;

    case 4:

        this->strType = string("Disengage");
        break;

    case 5:

        this->strType = string("Go home");
        break;


    }


    this->grip_code = 0;

    this->arm = arm;

}
/**
 * @brief Movement::Movement
 * @param type
 * @param arm
 * @param obj
 */

Movement::Movement(int type, int arm, objectPtr obj)
{

    this->type=type;
    switch (type){

    case 0:

        this->strType = string("Reach-to-grasp");
        break;

    case 1:

        this->strType = string("Reaching");
        break;

    case 2:

        this->strType = string("Transport");
        break;

    case 3:

        this->strType = string("Engage");
        break;

    case 4:

        this->strType = string("Disengage");
        break;

    case 5:

        this->strType = string("Go home");
        break;


    }

    this->obj = obj;
    this->obj_init = obj;
    this->obj_eng=objectPtr(new Object());
    this->grip_code = 0;
    this->arm=arm;


}
/**
 * @brief Movement::Movement
 * @param type
 * @param arm
 * @param obj
 * @param grip_id
 * @param prec
 */
Movement::Movement(int type, int arm, objectPtr obj, int grip_id, bool prec)
{

    this->type=type;
    switch (type){

    case 0:

        this->strType = string("Reach-to-grasp");
        break;

    case 1:

        this->strType = string("Reaching");
        break;

    case 2:

        this->strType = string("Transport");
        break;

    case 3:

        this->strType = string("Engage");
        break;

    case 4:

        this->strType = string("Disengage");
        break;

    case 5:

        this->strType = string("Go home");
        break;


    }


    this->obj = obj;
    this->obj_init = obj;
    this->obj_eng=objectPtr(new Object());

    if (prec){
        // precision grip
        switch (grip_id) {
        case 0:
            this->grip_code = 111; // Side thumb left
            this->grip_str=string("Precision Side thumb left");
            break;
        case 1:
            this->grip_code = 112; // Side thumb right
            this->grip_str=string("Precision Side thumb right");
            break;
        case 2:
            this->grip_code = 113; // Side thumb up
            this->grip_str=string("Precision Side thumb up");
            break;
        case 3:
            this->grip_code = 114; // Side thumb down
            this->grip_str=string("Precision Side thumb down");
            break;
        case 4:
            this->grip_code = 121; // Above
            this->grip_str=string("Precision Above");
            break;
        case 5:
            this->grip_code = 122; // Below
            this->grip_str=string("Precision Below");
            break;

        }
    }else{
        //full grip

        switch (grip_id) {
        case 0:
            this->grip_code = 211; // Side thumb left
            this->grip_str=string("Full Side thumb left");
            break;
        case 1:
            this->grip_code = 212; // Side thumb right
            this->grip_str=string("Full Side thumb right");
            break;
        case 2:
            this->grip_code = 213; // Side thumb up
            this->grip_str=string("Full Side thumb up");
            break;
        case 3:
            this->grip_code = 214; // Side thumb down
            this->grip_str=string("Full Side thumb down");
            break;
        case 4:
            this->grip_code = 221; // Above
            this->grip_str=string("Full Above");
            break;
        case 5:
            this->grip_code = 222; // Below
            this->grip_str=string("Full Below");
            break;

        }


    }

    this->arm=arm;
}
/**
 * @brief Movement::Movement
 * @param type
 * @param arm
 * @param obj
 * @param obj_eng
 * @param grip_id
 * @param prec
 */
Movement::Movement(int type, int arm,
                   objectPtr obj, objectPtr obj_eng, int grip_id, bool prec){

    this->type=type;
    switch (type){

    case 0:

        this->strType = string("Reach-to-grasp");
        break;

    case 1:

        this->strType = string("Reaching");
        break;

    case 2:

        this->strType = string("Transport");
        break;

    case 3:

        this->strType = string("Engage");
        break;

    case 4:

        this->strType = string("Disengage");
        break;

    case 5:

        this->strType = string("Go home");
        break;


    }


    this->obj = obj;
    this->obj_init = obj;
    this->obj_eng = obj_eng;

    if (prec){
        // precision grip
        switch (grip_id) {
        case 0:
            this->grip_code = 111; // Side thumb left
            this->grip_str=string("Precision Side thumb left");
            break;
        case 1:
            this->grip_code = 112; // Side thumb right
            this->grip_str=string("Precision Side thumb right");
            break;
        case 2:
            this->grip_code = 113; // Side thumb up
            this->grip_str=string("Precision Side thumb up");
            break;
        case 3:
            this->grip_code = 114; // Side thumb down
            this->grip_str=string("Precision Side thumb down");
            break;
        case 4:
            this->grip_code = 121; // Above
            this->grip_str=string("Precision Above");
            break;
        case 5:
            this->grip_code = 122; // Below
            this->grip_str=string("Precision Below");
            break;

        }
    }else{
        //full grip

        switch (grip_id) {
        case 0:
            this->grip_code = 211; // Side thumb left
            this->grip_str=string("Full Side thumb left");
            break;
        case 1:
            this->grip_code = 212; // Side thumb right
            this->grip_str=string("Full Side thumb right");
            break;
        case 2:
            this->grip_code = 213; // Side thumb up
            this->grip_str=string("Full Side thumb up");
            break;
        case 3:
            this->grip_code = 214; // Side thumb down
            this->grip_str=string("Full Side thumb down");
            break;
        case 4:
            this->grip_code = 221; // Above
            this->grip_str=string("Full Above");
            break;
        case 5:
            this->grip_code = 222; // Below
            this->grip_str=string("Full Below");
            break;

        }


    }

    this->arm=arm;

}

// copy constructor
/**
 * @brief Movement::Movement
 * @param mov
 */
Movement::Movement(const Movement &mov){

    this->arm = mov.arm;
    this->type = mov.type;
    this->strType = mov.strType;
    this->grip_code = mov.grip_code;
    this->grip_str = mov.grip_str;

    this->obj = objectPtr(new Object(*mov.obj.get()));
    this->obj_init = objectPtr(new Object(*mov.obj_init.get()));
    this->obj_eng = objectPtr(new Object(*mov.obj_eng.get()));
}

// destructors
/**
 * @brief Movement::~Movement
 */
Movement::~Movement(){


}

// setters
/**
 * @brief Movement::setType
 * @param t
 */
void Movement::setType(int t){

    this->type=t;
}



void Movement::setGrip(int g){

    this->grip_code=g;
}


/**
  * @brief Movement::setObject
  * @param obj
  */
void Movement::setObject(objectPtr obj){

     this->obj=obj;
}

/**
 * @brief Movement::setObjectInit
 * @param obj
 */
void Movement::setObjectInit(objectPtr obj){

     this->obj_init=obj;
}

/**
 * @brief Movement::setObjectEng
 * @param obj_eng
 */
void Movement::setObjectEng(objectPtr obj_eng){

   this->obj_eng = obj_eng;
}

/**
 * @brief Movement::setStrType
 * @param str
 */
void Movement::setStrType(string str){

    this->strType = str;
}


/**
 * @brief Movement::setArm
 * @param a
 */
void Movement::setArm(int a){

    this->arm = a;
}

//getters
/**
 * @brief Movement::getType
 * @return
 */
int Movement::getType(){

    return this->type;
}
/**
 * @brief Movement::getGrip
 * @return
 */
int Movement::getGrip(){

    return this->grip_code;
}
/**
 * @brief Movement::getGripStr
 * @return
 */
string Movement::getGripStr(){

    return this->grip_str;
}
/**
 * @brief Movement::getObject
 * @return
 */
objectPtr Movement::getObject(){

    return obj;
}

/**
 * @brief Movement::getObject
 * @return
 */
objectPtr Movement::getObjectInit(){

    return obj_init;
}
/**
 * @brief Movement::getObjectEng
 * @return
 */
objectPtr Movement::getObjectEng(){

    return obj_eng;
}

/**
 * @brief Movement::getStrType
 * @return
 */
string Movement::getStrType(){

    return this->strType;
}

/**
 * @brief Movement::getInfoLine
 * @return
 */
string Movement::getInfoLine(){

    if (obj && obj_eng && grip_code!=0){

        return strType +", Arm: "+this->getArmInfo()+", Object: "+obj->getName()+
                ", Object Engaged: "+obj_eng->getName()+", Grip Type: "+grip_str;

    }else if(obj && grip_code!=0){


        return strType +", Arm: "+this->getArmInfo()+", Object: "+obj->getName()+
                ", Grip Type: "+grip_str;

    }else if(obj){

        return strType +", Arm: "+this->getArmInfo()+", Object: "+obj->getName();

    }else{

        return strType+", Arm: "+this->getArmInfo();

    }
}
/**
 * @brief Movement::getArm
 * @return
 */
int Movement::getArm(){

    return this->arm;
}
/**
 * @brief Movement::getArmInfo
 * @return
 */
string Movement::getArmInfo(){

    switch (arm) {
    case 0:
        return string("both");

        break;
    case 1:

        return string("right");
        break;

    case 2:
        return string("left");
        break;

    default:
        return string("right");
        break;
    }

}


} // namespace HUMotion

