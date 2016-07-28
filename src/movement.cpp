#include "../include/movement.hpp"



namespace HUMotion {


Movement::Movement(int type, int arm)
{

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
    this->executed = false;

}


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
    this->grip_str=string("No Grip");
    this->arm=arm;
    this->executed = false;


}


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
        case 6:
            this->grip_code = 0;
            this->grip_str=string("No Grip");
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
        case 6:
            this->grip_code = 0;
            this->grip_str=string("No Grip");
            break;

        }


    }

    this->arm=arm;
    this->executed = false;
}


Movement::Movement(int type, int arm,
                   objectPtr obj, objectPtr obj_eng,
                   int grip_id, bool prec)
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
        case 6:
            this->grip_code = 0;
            this->grip_str=string("No Grip");
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
        case 6:
            this->grip_code = 0;
            this->grip_str=string("No Grip");
            break;

        }


    }

    this->arm=arm;
    this->executed = false;

}


Movement::Movement(const Movement &mov)
{

    this->arm = mov.arm;
    this->type = mov.type;
    this->strType = mov.strType;
    this->grip_code = mov.grip_code;
    this->grip_str = mov.grip_str;

    this->obj = objectPtr(new Object(*mov.obj.get()));
    this->obj_init = objectPtr(new Object(*mov.obj_init.get()));
    this->obj_eng = objectPtr(new Object(*mov.obj_eng.get()));
    this->executed = mov.executed;
}


Movement::~Movement()
{


}


void Movement::setType(int t)
{
    this->type=t;

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
}



void Movement::setGrip(int index, bool prec)
{

    this->grip_code=index;

    if (prec){
        // precision grip
        switch (index) {
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
        case 6:
            this->grip_code = 0;
            this->grip_str=string("No Grip");
            break;

        }
    }else{
        //full grip

        switch (index) {
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
        case 6:
            this->grip_code = 0;
            this->grip_str=string("No Grip");
            break;

        }


    }
}



void Movement::setObject(objectPtr obj)
{

     this->obj=obj;
}


void Movement::setObjectInit(objectPtr obj)
{

     this->obj_init=obj;
}


void Movement::setObjectEng(objectPtr obj_eng)
{

   this->obj_eng = obj_eng;
}




void Movement::setArm(int a)
{

    this->arm = a;
}


void Movement::setExecuted(bool exec)
{

    this->executed=exec;
}


int Movement::getType()
{

    return this->type;
}

int Movement::getGrip()
{

    return this->grip_code;
}

string Movement::getGripStr()
{

    return this->grip_str;
}

objectPtr Movement::getObject()
{

    return obj;
}


objectPtr Movement::getObjectInit()
{

    return obj_init;
}

objectPtr Movement::getObjectEng()
{

    return obj_eng;
}


string Movement::getStrType()
{

    return this->strType;
}


string Movement::getInfoLine()
{

    string arm_info;

    switch (arm) {
    case 0:
        arm_info = string("both");

        break;
    case 1:

        arm_info = string("right");
        break;

    case 2:
        arm_info = string("left");
        break;

    default:
        arm_info = string("right");
        break;
    }

    if (obj && obj_eng && grip_code!=0){

        return strType +", Arm: "+arm_info+", Object: "+obj->getName()+
                ", Object Engaged: "+obj_eng->getName()+", Grip Type: "+grip_str;

    }else if(obj && grip_code!=0){


        return strType +", Arm: "+arm_info+", Object: "+obj->getName()+
                ", Grip Type: "+grip_str;

    }else if(obj){

        return strType +", Arm: "+arm_info+", Object: "+obj->getName();

    }else{

        return strType+", Arm: "+arm_info;

    }
}

int Movement::getArm()
{

    return this->arm;
}


bool Movement::getExecuted()
{

    return this->executed;
}


} // namespace HUMotion

