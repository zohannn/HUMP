#include "../include/object.hpp"
#include <boost/format.hpp>

namespace HUMotion{


// constructors
Object::Object(){

    this->m_name = "";
    this->m_pos.Xpos=0;this->m_pos.Ypos=0;this->m_pos.Zpos=0;
    this->m_or.pitch = 0; this->m_or.roll = 0; this->m_or.yaw = 0;
    this->m_size.Xsize = 0; this->m_size.Ysize = 0; this->m_size.Zsize = 0;
    this->handle = -1;
    this->handle_body = -1;
    this->p_targetRight = targetPtr(new Target());
    this->p_targetLeft = targetPtr(new Target());
    this->p_engage = engagePtr(new EngagePoint());

    this->m_targetRightEnabled = false;
    this->m_targetLeftEnabled = false;

    this->setup_features = true;

}

/**
 * @brief Object::Object
 * @param name
 */
Object::Object(string name){

    this->m_name = name;
    this->m_pos.Xpos=0;this->m_pos.Ypos=0;this->m_pos.Zpos=0;
    this->m_or.pitch = 0; this->m_or.roll = 0; this->m_or.yaw = 0;
    this->m_size.Xsize = 0; this->m_size.Ysize = 0; this->m_size.Zsize = 0;
    this->handle = -1;
    this->handle_body = -1;
    this->p_targetRight = targetPtr(new Target());
    this->p_targetLeft = targetPtr(new Target());
    this->p_engage = engagePtr(new EngagePoint());

    this->m_targetRightEnabled = false;
    this->m_targetLeftEnabled = false;

    this->setup_features = true;

}

/**
 * @brief Object::Object
 * @param name
 * @param ppos
 * @param oor
 * @param ssize
 * @param pTR
 * @param pTL
 * @param pEng
 */
Object::Object(string name, pos ppos, orient oor, dim ssize,
               Target* pTR, Target* pTL,
               EngagePoint* pEng)
{

    this->m_name = name;
    this->m_pos = ppos;
    this->m_or = oor;
    this->m_size = ssize;
    this->handle = -1;
    this->handle_body = -1;
    this->p_targetRight = targetPtr(pTR);
    this->p_targetLeft = targetPtr(pTL);
    this->p_engage = engagePtr(pEng);

    this->m_targetRightEnabled = false;
    this->m_targetLeftEnabled = false;

    this->setup_features = false;

}

// copy constructor
/**
 * @brief Object::Object
 * @param obj
 */
Object::Object(const Object &obj){

    this->m_name = obj.m_name;
    this->m_pos = obj.m_pos;
    this->m_or = obj.m_or;
    this->m_size = obj.m_size;

    this->p_targetRight = targetPtr(new Target(*obj.p_targetRight.get()));
    this->p_targetLeft = targetPtr(new Target(*obj.p_targetLeft.get()));
    this->p_engage = engagePtr(new EngagePoint(*obj.p_engage.get()));

    this->handle = obj.handle;
    this->handle_body = obj.handle_body;

    this->m_targetRightEnabled = obj.m_targetRightEnabled;
    this->m_targetLeftEnabled = obj.m_targetLeftEnabled;

    this->setup_features = obj.setup_features;


}

// destructor
/**
 * @brief Object::~Object
 */
Object::~Object(){

}

// setters
/**
 * @brief Object::setName
 * @param name
 */
void Object::setName(string name){

    this->m_name = name;
}
/**
 * @brief Object::setPos
 * @param ppos
 * @param update_features
 * @return
 */
bool Object::setPos(pos ppos, bool update_features){


    if (update_features){

        float x;
        float y;
        float z;
        Matrix4f trans_tar_right;
        Matrix4f trans_tar_left;
        Matrix4f trans_engage;

        Matrix4f trans_obj;
        Matrix4f inv_trans_obj;

        Matrix4f trans_obj_tar_right;
        Matrix4f trans_obj_tar_left;
        Matrix4f trans_obj_engage;

        x = this->m_pos.Xpos;
        y = this->m_pos.Ypos;
        z = this->m_pos.Zpos;
        Matrix3f Rot;
        this->RPY_matrix(Rot);

        trans_obj(0,0) = Rot(0,0); trans_obj(0,1) = Rot(0,1); trans_obj(0,2) = Rot(0,2); trans_obj(0,3) = x;
        trans_obj(1,0) = Rot(1,0); trans_obj(1,1) = Rot(1,1); trans_obj(1,2) = Rot(1,2); trans_obj(1,3) = y;
        trans_obj(2,0) = Rot(2,0); trans_obj(2,1) = Rot(2,1); trans_obj(2,2) = Rot(2,2); trans_obj(2,3) = z;
        trans_obj(3,0) = 0;        trans_obj(3,1) = 0;        trans_obj(3,2) = 0;        trans_obj(3,3) = 1;

        this->getTar_right_matrix(trans_tar_right);
        this->getTar_left_matrix(trans_tar_left);
        this->getEngage_matrix(trans_engage);


        inv_trans_obj = trans_obj.inverse();
        trans_obj_tar_right=inv_trans_obj * trans_tar_right; // transformation matrix object - target right
        trans_obj_tar_left=inv_trans_obj * trans_tar_left; // transformation matrix object - target left
        trans_obj_engage=inv_trans_obj * trans_engage; // transformation matrix object - engage


        // set the new position of the object and update the features
        this->m_pos = ppos;
        x = this->m_pos.Xpos; trans_obj(0,3)=x;
        y = this->m_pos.Ypos; trans_obj(1,3)=y;
        z = this->m_pos.Zpos; trans_obj(2,3)=z;

        trans_tar_right = trans_obj * trans_obj_tar_right; // updated matrix of target right
        pos new_tar_right_pos;
        new_tar_right_pos.Xpos = trans_tar_right(0,3);
        new_tar_right_pos.Ypos = trans_tar_right(1,3);
        new_tar_right_pos.Zpos = trans_tar_right(2,3);
        this->p_targetRight->setPos(new_tar_right_pos);

        trans_tar_left = trans_obj * trans_obj_tar_left; // updated matrix of target left
        pos new_tar_left_pos;
        new_tar_left_pos.Xpos = trans_tar_left(0,3);
        new_tar_left_pos.Ypos = trans_tar_left(1,3);
        new_tar_left_pos.Zpos = trans_tar_left(2,3);
        this->p_targetLeft->setPos(new_tar_left_pos);

        trans_engage = trans_obj * trans_obj_engage; // updated matrix of the engage point
        pos new_engage_pos;
        new_engage_pos.Xpos = trans_engage(0,3);
        new_engage_pos.Ypos = trans_engage(1,3);
        new_engage_pos.Zpos = trans_engage(2,3);
        this->p_engage->setPos(new_engage_pos);



        return true;

    }else{
        this->m_pos = ppos;
        return true;
    }
}
/**
 * @brief Object::setOr
 * @param oor
 * @param update_features
 * @return
 */
bool Object::setOr(orient oor, bool update_features){

    if (update_features){

        float x;
        float y;
        float z;
        Matrix4f trans_tar_right;
        Matrix4f trans_tar_left;
        Matrix4f trans_engage;

        Matrix4f trans_obj;
        Matrix4f inv_trans_obj;

        Matrix4f trans_obj_tar_right;
        Matrix4f trans_obj_tar_left;
        Matrix4f trans_obj_engage;

        x = this->m_pos.Xpos;
        y = this->m_pos.Ypos;
        z = this->m_pos.Zpos;
        Matrix3f Rot;
        this->RPY_matrix(Rot);

        trans_obj(0,0) = Rot(0,0); trans_obj(0,1) = Rot(0,1); trans_obj(0,2) = Rot(0,2); trans_obj(0,3) = x;
        trans_obj(1,0) = Rot(1,0); trans_obj(1,1) = Rot(1,1); trans_obj(1,2) = Rot(1,2); trans_obj(1,3) = y;
        trans_obj(2,0) = Rot(2,0); trans_obj(2,1) = Rot(2,1); trans_obj(2,2) = Rot(2,2); trans_obj(2,3) = z;
        trans_obj(3,0) = 0;        trans_obj(3,1) = 0;        trans_obj(3,2) = 0;        trans_obj(3,3) = 1;

        this->getTar_right_matrix(trans_tar_right);
        this->getTar_left_matrix(trans_tar_left);
        this->getEngage_matrix(trans_engage);


        inv_trans_obj = trans_obj.inverse();
        trans_obj_tar_right=inv_trans_obj * trans_tar_right; // transformation matrix object - target right
        trans_obj_tar_left=inv_trans_obj * trans_tar_left; // transformation matrix object - target left
        trans_obj_engage=inv_trans_obj * trans_engage; // transformation matrix object - engage

        // update the new orientation of the object and update the features
        this->m_or = oor;
        this->RPY_matrix(Rot);
        trans_obj(0,0) = Rot(0,0); trans_obj(0,1) = Rot(0,1); trans_obj(0,2) = Rot(0,2);
        trans_obj(1,0) = Rot(1,0); trans_obj(1,1) = Rot(1,1); trans_obj(1,2) = Rot(1,2);
        trans_obj(2,0) = Rot(2,0); trans_obj(2,1) = Rot(2,1); trans_obj(2,2) = Rot(2,2);

        trans_tar_right = trans_obj * trans_obj_tar_right; // updated matrix of target right
        std::vector<float> rpy1;
        this->getRPY(trans_tar_right,rpy1);
        orient new_tar_right_or;
        new_tar_right_or.roll = rpy1.at(0); new_tar_right_or.pitch = rpy1.at(1); new_tar_right_or.yaw = rpy1.at(2);
        this->p_targetRight->setOr(new_tar_right_or);

        trans_tar_left = trans_obj * trans_obj_tar_left; // updated matrix of target left
        std::vector<float> rpy2;
        this->getRPY(trans_tar_left,rpy2);
        orient new_tar_left_or;
        new_tar_left_or.roll = rpy2.at(0); new_tar_left_or.pitch = rpy2.at(1); new_tar_left_or.yaw = rpy2.at(2);
        this->p_targetLeft->setOr(new_tar_left_or);

        trans_engage = trans_obj * trans_obj_engage; // updated matrix of engage point
        std::vector<float> rpy3;
        this->getRPY(trans_engage,rpy3);
        orient new_engage_or;
        new_engage_or.roll = rpy3.at(0); new_engage_or.pitch = rpy3.at(1); new_engage_or.yaw = rpy3.at(2);
        this->p_engage->setOr(new_engage_or);

        return true;


    }else{
        this->m_or = oor;
        return true;
    }
}

/**
 * @brief Object::getRPY
 * @param Trans
 * @param rpy
 */
void Object::getRPY(Matrix4f Trans, std::vector<float> &rpy){

    rpy = std::vector<float>(3);


    if(abs(Trans(0,0) < 1e-10) && (abs(Trans(1,0) < 1e-10))){

        rpy.at(0) = 0; // [rad]
        rpy.at(1) = atan2(-Trans(2,0),Trans(0,0)); // [rad]
        rpy.at(2) = atan2(-Trans(1,2),Trans(1,1)); // [rad]

    }else{

        rpy.at(0) = atan2(Trans(1,0),Trans(0,0)); // [rad]
        float sp = sin(rpy.at(0));
        float cp = cos(rpy.at(0));
        rpy.at(1) = atan2(-Trans(2,0), cp*Trans(0,0)+sp*Trans(1,0)); // [rad]
        rpy.at(2) = atan2(sp*Trans(0,2)-cp*Trans(1,2),cp*Trans(1,1)-sp*Trans(0,1)); // [rad]

    }

}

/**
 * @brief Object::setSize
 * @param ssize
 */
void Object::setSize(dim ssize){

    this->m_size = ssize;
}
/**
 * @brief Object::setTargetRight
 * @param tr
 * @return
 */
bool Object::setTargetRight(targetPtr tr){

    if (this->setup_features){
        this->p_targetRight = targetPtr(new Target(*tr.get()));
        return true;
    }else{
        return false;
    }
}
/**
 * @brief Object::setHandle
 * @param h
 */
void Object::setHandle(int h){

    this->handle = h;
}
/**
 * @brief Object::setHandleBody
 * @param h
 */
void Object::setHandleBody(int h){

    this->handle_body = h;
}

/**
 * @brief Object::setTargetLeft
 * @param tl
 * @return
 */
bool Object::setTargetLeft(targetPtr tl){

    if (this->setup_features){
        this->p_targetLeft = targetPtr(new Target(*tl.get()));
        return true;
    }else{
        return false;
    }
}
/**
 * @brief Object::setEngagePoint
 * @param eng
 * @return
 */
bool Object::setEngagePoint(engagePtr eng){

    if (this->setup_features){
        this->p_engage = engagePtr(new EngagePoint(*eng.get()));
        return true;
    }else{
        return false;
    }
}

/**
 * @brief Object::setTargetRightEnabled
 * @param c
 */
void Object::setTargetRightEnabled(bool c){

    this->m_targetRightEnabled = c;
}
/**
 * @brief Object::setTargetLeftEnabled
 * @param c
 */
void Object::setTargetLeftEnabled(bool c){

    this->m_targetLeftEnabled = c;
}


// getters
/**
 * @brief Object::getName
 * @return
 */
string Object::getName(){

    return this->m_name;
}
/**
 * @brief Object::getPos
 * @return
 */
pos Object::getPos(){

    return this->m_pos;
}
/**
 * @brief Object::getOr
 * @return
 */
orient Object::getOr(){

    return this->m_or;
}
/**
 * @brief Object::getSize
 * @return
 */
dim Object::getSize(){

    return this->m_size;
}
/**
 * @brief Object::getHandle
 * @return
 */
int Object::getHandle(){

    return this->handle;
}

/**
 * @brief Object::getHandleBody
 * @return
 */
int Object::getHandleBody(){

    return this->handle_body;
}

/**
 * @brief Object::getTargetRight
 * @return
 */
targetPtr Object::getTargetRight(){

    return p_targetRight;
}
/**
 * @brief Object::getTargetLeft
 * @return
 */
targetPtr Object::getTargetLeft(){

    return p_targetLeft;
}
/**
 * @brief Object::getEngagePoint
 * @return
 */
engagePtr Object::getEngagePoint(){

   return p_engage;
}

/**
 * @brief Object::getRadius
 * @return
 */
float Object::getRadius(){

    return (max(this->m_size.Xsize,this->m_size.Ysize)/2.0);
}
/**
 * @brief Object::isTargetRightEnabled
 * @return
 */
bool Object::isTargetRightEnabled(){

    return this->m_targetRightEnabled;
}
/**
 * @brief Object::isTargetLeftEnabled
 * @return
 */
bool Object::isTargetLeftEnabled(){

    return this->m_targetLeftEnabled;
}


/**
 * @brief Object::getInfoLine
 * @return
 */
string Object::getInfoLine(){

    return  this->m_name + COLUMN + SPACE +
            XposSTR + str(boost::format("%d") % this->m_pos.Xpos) + MILLIMETERS + SEP +
            YposSTR + str(boost::format("%d") % this->m_pos.Ypos) + MILLIMETERS + SEP+
            ZposSTR + str(boost::format("%d") % this->m_pos.Zpos) + MILLIMETERS + SEP+
            RollSTR + str(boost::format("%d") % this->m_or.roll) + RAD + SEP+
            PitchSTR + str(boost::format("%d") % this->m_or.pitch) + RAD + SEP+
            YawSTR + str(boost::format("%d") % this->m_or.yaw) + RAD + SEP+
            XsizeSTR + str(boost::format("%d") % this->m_size.Xsize) + MILLIMETERS + SEP+
            YsizeSTR + str(boost::format("%d") % this->m_size.Ysize) + MILLIMETERS + SEP+
            ZsizeSTR + str(boost::format("%d") % this->m_size.Zsize)+ MILLIMETERS;


}
/**
 * @brief Object::getNorm
 * @return
 */
float Object::getNorm(){


    return sqrt(pow((m_pos.Xpos),2)+pow((m_pos.Ypos),2)+pow((m_pos.Zpos),2));
}

/**
 * @brief Object::getXt
 * @param xt
 */
void Object::getXt(std::vector<float> &xt){

    Matrix3f Rot;
    this->RPY_matrix(Rot);
    Vector3f v = Rot.col(0);

    xt.push_back(v(0));
    xt.push_back(v(1));
    xt.push_back(v(2));

}

/**
 * @brief Object::getYt
 * @param yt
 */
void Object::getYt(std::vector<float> &yt){

    Matrix3f Rot;
    this->RPY_matrix(Rot);
    Vector3f v = Rot.col(1);

    yt.push_back(v(0));
    yt.push_back(v(1));
    yt.push_back(v(2));
}

/**
 * @brief Object::getZt
 * @param zt
 */
void Object::getZt(std::vector<float> &zt){

    Matrix3f Rot;
    this->RPY_matrix(Rot);
    Vector3f v = Rot.col(2);

    zt.push_back(v(0));
    zt.push_back(v(1));
    zt.push_back(v(2));
}

/**
 * @brief Object::RPY_matrix
 * @param Rot
 */
void Object::RPY_matrix(Matrix3f &Rot){

    float roll = this->m_or.roll;
    float pitch = this->m_or.pitch;
    float yaw = this->m_or.yaw;

    Rot(0,0) = cos(roll)*cos(pitch);  Rot(0,1) = cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw); Rot(0,2) = sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw);
    Rot(1,0) = sin(roll)*cos(pitch);  Rot(1,1) = cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw); Rot(1,2) = sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw);
    Rot(2,0) = -sin(pitch);           Rot(2,1) = cos(pitch)*sin(yaw);                              Rot(2,2) = cos(pitch)*cos(yaw);


}
/**
 * @brief Object::getTar_right_matrix
 * @param mat
 */
void Object::getTar_right_matrix(Matrix4f& mat){

    float x = this->p_targetRight->getPos().Xpos;
    float y = this->p_targetRight->getPos().Ypos;
    float z = this->p_targetRight->getPos().Zpos;

    float roll =  this->p_targetRight->getOr().roll;
    float pitch = this->p_targetRight->getOr().pitch;
    float yaw = this->p_targetRight->getOr().yaw;

    mat(0,0) = cos(roll)*cos(pitch); mat(0,1) = cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw);   mat(0,2) = sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw); mat(0,3) = x;
    mat(1,0) = sin(roll)*cos(pitch); mat(1,1) = cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw);   mat(1,2) = sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw); mat(1,3) = y;
    mat(2,0) = -sin(pitch);          mat(2,1) = cos(pitch)*sin(yaw);                                mat(2,2) = cos(pitch)*cos(yaw);                              mat(2,3) = z;
    mat(3,0) = 0;                    mat(3,1) = 0;                                                  mat(3,2) = 0;                                                mat(3,3) = 1;


}

/**
 * @brief Object::getTar_left_matrix
 * @param mat
 */
void Object::getTar_left_matrix(Matrix4f &mat){


    float x = this->p_targetLeft->getPos().Xpos;
    float y = this->p_targetLeft->getPos().Ypos;
    float z = this->p_targetLeft->getPos().Zpos;

    float roll =  this->p_targetLeft->getOr().roll;
    float pitch = this->p_targetLeft->getOr().pitch;
    float yaw = this->p_targetLeft->getOr().yaw;

    mat(0,0) = cos(roll)*cos(pitch); mat(0,1) = cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw);   mat(0,2) = sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw); mat(0,3) = x;
    mat(1,0) = sin(roll)*cos(pitch); mat(1,1) = cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw);   mat(1,2) = sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw); mat(1,3) = y;
    mat(2,0) = -sin(pitch);          mat(2,1) = cos(pitch)*sin(yaw);                                mat(2,2) = cos(pitch)*cos(yaw);                              mat(2,3) = z;
    mat(3,0) = 0;                    mat(3,1) = 0;                                                  mat(3,2) = 0;                                                mat(3,3) = 1;


}

/**
 * @brief Object::getEngage_matrix
 * @param mat
 */
void Object::getEngage_matrix(Matrix4f &mat){


    float x = this->p_engage->getPos().Xpos;
    float y = this->p_engage->getPos().Ypos;
    float z = this->p_engage->getPos().Zpos;

    float roll =  this->p_engage->getOr().roll;
    float pitch = this->p_engage->getOr().pitch;
    float yaw = this->p_engage->getOr().yaw;

    mat(0,0) = cos(roll)*cos(pitch); mat(0,1) = cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw);   mat(0,2) = sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw); mat(0,3) = x;
    mat(1,0) = sin(roll)*cos(pitch); mat(1,1) = cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw);   mat(1,2) = sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw); mat(1,3) = y;
    mat(2,0) = -sin(pitch);          mat(2,1) = cos(pitch)*sin(yaw);                                mat(2,2) = cos(pitch)*cos(yaw);                              mat(2,3) = z;
    mat(3,0) = 0;                    mat(3,1) = 0;                                                  mat(3,2) = 0;                                                mat(3,3) = 1;

}



} // namespace HUMotion


