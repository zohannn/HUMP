#include "../include/humanoid.hpp"
#include <boost/format.hpp>

namespace HUMotion{

#if HAND==0
// human hand

/**
 * @brief Humanoid::Humanoid
 * @param name
 * @param ppos
 * @param oor
 * @param ssize
 * @param aspecs
 * @param hspecs
 */
Humanoid::Humanoid(string name, pos ppos, orient oor, dim ssize, arm aspecs, human_hand hspecs){

    this->m_name = name;
    this->m_torso_pos = ppos;
    this->m_torso_or = oor;
    this->m_torso_size = ssize;
    this->m_arm_specs = aspecs;

    // human hand
    this->m_human_hand_specs = hspecs;

    this->rightPosture = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->leftPosture = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->rightHomePosture = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->leftHomePosture = std::vector<float>(JOINTS_ARM+JOINTS_HAND);

    this->min_rightLimits = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->min_leftLimits = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->max_rightLimits = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->max_leftLimits = std::vector<float>(JOINTS_ARM+JOINTS_HAND);

    this->rightVelocities= std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->leftVelocities = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->rightForces = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->leftForces = std::vector<float>(JOINTS_ARM+JOINTS_HAND);


    this->mat_right = Matrix4f::Constant(1);
    this->mat_left = Matrix4f::Constant(1);
    this->mat_r_hand = Matrix4f::Constant(1);
    this->mat_l_hand = Matrix4f::Constant(1);



}

/**
* @brief Humanoid::Humanoid
* @param name
* @param ppos
* @param oor
* @param ssize
* @param aspecs
* @param hspecs
* @param r
* @param l
*/
Humanoid::Humanoid(string name, pos ppos, orient oor, dim ssize, arm aspecs, human_hand hspecs,
                   std::vector<float> &r, std::vector<float> &l){

    this->m_name = name;
    this->m_torso_pos = ppos;
    this->m_torso_or = oor;
    this->m_torso_size = ssize;
    this->m_arm_specs = aspecs;

    // human hand
    this->m_human_hand_specs = hspecs;

    this->rightPosture = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->leftPosture = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->rightHomePosture = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->leftHomePosture = std::vector<float>(JOINTS_ARM+JOINTS_HAND);

    std::copy(r.begin(),r.end(),this->rightPosture.begin());
    std::copy(l.begin(),l.end(),this->leftPosture.begin());
    std::copy(r.begin(),r.end(),this->rightHomePosture.begin());
    std::copy(l.begin(),l.end(),this->leftHomePosture.begin());

    this->min_rightLimits = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->min_leftLimits = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->max_rightLimits = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->max_leftLimits = std::vector<float>(JOINTS_ARM+JOINTS_HAND);

    this->rightVelocities= std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->leftVelocities = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->rightForces = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->leftForces = std::vector<float>(JOINTS_ARM+JOINTS_HAND);

    this->mat_right = Matrix4f::Constant(1);
    this->mat_left = Matrix4f::Constant(1);
    this->mat_r_hand = Matrix4f::Constant(1);
    this->mat_l_hand = Matrix4f::Constant(1);


}

/**
 * @brief Humanoid::Humanoid
 * @param name
 * @param ppos
 * @param oor
 * @param ssize
 * @param aspecs
 * @param hspecs
 * @param r
 * @param l
 * @param min_rl
 * @param max_rl
 * @param min_ll
 * @param max_ll
 */
Humanoid::Humanoid(string name, pos ppos, orient oor, dim ssize, arm aspecs, human_hand hspecs,
                   std::vector<float> &r, std::vector<float> &l,
                   std::vector<float> &min_rl, std::vector<float> &max_rl,
                   std::vector<float> &min_ll, std::vector<float> &max_ll){

    this->m_name = name;
    this->m_torso_pos = ppos;
    this->m_torso_or = oor;
    this->m_torso_size = ssize;
    this->m_arm_specs = aspecs;

    // human hand
    this->m_human_hand_specs = hspecs;

    this->rightPosture = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->leftPosture = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->rightHomePosture = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->leftHomePosture = std::vector<float>(JOINTS_ARM+JOINTS_HAND);

    std::copy(r.begin(),r.end(),this->rightPosture.begin());
    std::copy(l.begin(),l.end(),this->leftPosture.begin());
    std::copy(r.begin(),r.end(),this->rightHomePosture.begin());
    std::copy(l.begin(),l.end(),this->leftHomePosture.begin());

    this->min_rightLimits = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->min_leftLimits = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->max_rightLimits = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->max_leftLimits = std::vector<float>(JOINTS_ARM+JOINTS_HAND);

    std::copy(min_rl.begin(),min_rl.end(),this->min_rightLimits.begin());
    std::copy(min_ll.begin(),min_ll.end(),this->min_leftLimits.begin());
    std::copy(max_rl.begin(),max_rl.end(),this->max_rightLimits.begin());
    std::copy(max_ll.begin(),max_ll.end(),this->max_leftLimits.begin());

    this->rightVelocities= std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->leftVelocities = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->rightForces = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->leftForces = std::vector<float>(JOINTS_ARM+JOINTS_HAND);

    this->mat_right = Matrix4f::Constant(1);
    this->mat_left = Matrix4f::Constant(1);
    this->mat_r_hand = Matrix4f::Constant(1);
    this->mat_l_hand = Matrix4f::Constant(1);
    /*
    this->mat_left(0,0) = 1.0; this->mat_left(0,1) = 0.0; this->mat_left(0,2) = 0.0; this->mat_left(0,3) = 0.0;
    this->mat_left(0,0) = 0.0; this->mat_left(0,1) = -1.0; this->mat_left(0,2) = 0.0; this->mat_left(0,3) = 100.0;
    this->mat_left(0,0) = 0.0; this->mat_left(0,1) = 0.0; this->mat_left(0,2) = -1.0; this->mat_left(0,3) = 0.0;
    this->mat_left(0,0) = 0.0; this->mat_left(0,1) = 0.0; this->mat_left(0,2) = 0.0; this->mat_left(0,3) = 1.0;
*/

}

#elif HAND==1
// barrett hand

/**
 * @brief Humanoid::Humanoid
 * @param name
 * @param ppos
 * @param oor
 * @param ssize
 * @param aspecs
 * @param hspecs
 */
Humanoid::Humanoid(string name, pos ppos, orient oor, dim ssize, arm aspecs, barrett_hand hspecs)
{

        this->m_name = name;
        this->m_torso_pos = ppos;
        this->m_torso_or = oor;
        this->m_torso_size = ssize;
        this->m_arm_specs = aspecs;

        // hand
        this->m_barrett_hand_specs = hspecs;

        this->rk.push_back(-1.0);
        this->rk.push_back(1.0);
        this->rk.push_back(0.0);

        this->jk.push_back(-1.0);
        this->jk.push_back(-1.0);
        this->jk.push_back(1.0);

        this->rightPosture = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
        this->leftPosture = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
        this->rightHomePosture = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
        this->leftHomePosture = std::vector<float>(JOINTS_ARM+JOINTS_HAND);

        this->min_rightLimits = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
        this->min_leftLimits = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
        this->max_rightLimits = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
        this->max_leftLimits = std::vector<float>(JOINTS_ARM+JOINTS_HAND);

        this->rightVelocities= std::vector<float>(JOINTS_ARM+JOINTS_HAND);
        this->leftVelocities = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
        this->rightForces = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
        this->leftForces = std::vector<float>(JOINTS_ARM+JOINTS_HAND);

    this->mat_right = Matrix4f::Identity(4,4);
    this->mat_left = Matrix4f::Identity(4,4);
    this->mat_r_hand = Matrix4f::Identity(4,4);
    this->mat_l_hand = Matrix4f::Identity(4,4);
    /*
        this->mat_left(0,0) = 1.0; this->mat_left(0,1) = 0.0; this->mat_left(0,2) = 0.0; this->mat_left(0,3) = 0.0;
        this->mat_left(0,0) = 0.0; this->mat_left(0,1) = -1.0; this->mat_left(0,2) = 0.0; this->mat_left(0,3) = 100.0;
        this->mat_left(0,0) = 0.0; this->mat_left(0,1) = 0.0; this->mat_left(0,2) = -1.0; this->mat_left(0,3) = 0.0;
        this->mat_left(0,0) = 0.0; this->mat_left(0,1) = 0.0; this->mat_left(0,2) = 0.0; this->mat_left(0,3) = 1.0;
*/
}
/**
 * @brief Humanoid::Humanoid
 * @param name
 * @param ppos
 * @param oor
 * @param ssize
 * @param aspecs
 * @param hspecs
 * @param r
 * @param l
 */
Humanoid::Humanoid(string name, pos ppos, orient oor, dim ssize, arm aspecs, barrett_hand hspecs,
                   std::vector<float>& r, std::vector<float>& l)
{

        this->m_name = name;
        this->m_torso_pos = ppos;
        this->m_torso_or = oor;
        this->m_torso_size = ssize;
        this->m_arm_specs = aspecs;

        // hand
        this->m_barrett_hand_specs = hspecs;

        this->rk.push_back(-1.0);
        this->rk.push_back(1.0);
        this->rk.push_back(0.0);

        this->jk.push_back(-1.0);
        this->jk.push_back(-1.0);
        this->jk.push_back(1.0);

        this->rightPosture = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
        this->leftPosture = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
        this->rightHomePosture = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
        this->leftHomePosture = std::vector<float>(JOINTS_ARM+JOINTS_HAND);

        std::copy(r.begin(),r.end(),this->rightPosture.begin());
        std::copy(l.begin(),l.end(),this->leftPosture.begin());
        std::copy(r.begin(),r.end(),this->rightHomePosture.begin());
        std::copy(l.begin(),l.end(),this->leftHomePosture.begin());

        this->min_rightLimits = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
        this->min_leftLimits = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
        this->max_rightLimits = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
        this->max_leftLimits = std::vector<float>(JOINTS_ARM+JOINTS_HAND);

        this->rightVelocities= std::vector<float>(JOINTS_ARM+JOINTS_HAND);
        this->leftVelocities = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
        this->rightForces = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
        this->leftForces = std::vector<float>(JOINTS_ARM+JOINTS_HAND);

        this->mat_right = Matrix4f::Identity(4,4);
        this->mat_left = Matrix4f::Identity(4,4);
        this->mat_r_hand = Matrix4f::Identity(4,4);
        this->mat_l_hand = Matrix4f::Identity(4,4);
        /*
        this->mat_left(0,0) = 1.0; this->mat_left(0,1) = 0.0; this->mat_left(0,2) = 0.0; this->mat_left(0,3) = 0.0;
        this->mat_left(0,0) = 0.0; this->mat_left(0,1) = -1.0; this->mat_left(0,2) = 0.0; this->mat_left(0,3) = 100.0;
        this->mat_left(0,0) = 0.0; this->mat_left(0,1) = 0.0; this->mat_left(0,2) = -1.0; this->mat_left(0,3) = 0.0;
        this->mat_left(0,0) = 0.0; this->mat_left(0,1) = 0.0; this->mat_left(0,2) = 0.0; this->mat_left(0,3) = 1.0;

        */
}
/**
 * @brief Humanoid::Humanoid
 * @param name
 * @param ppos
 * @param oor
 * @param ssize
 * @param aspecs
 * @param hspecs
 * @param r
 * @param l
 * @param min_rl
 * @param max_rl
 * @param min_ll
 * @param max_ll
 */
Humanoid::Humanoid(string name, pos ppos, orient oor, dim ssize, arm aspecs, barrett_hand hspecs,
                   std::vector<float> &r, std::vector<float> &l,
                   std::vector<float> &min_rl, std::vector<float> &max_rl,
                   std::vector<float> &min_ll, std::vector<float> &max_ll)
{


    this->m_name = name;
    this->m_torso_pos = ppos;
    this->m_torso_or = oor;
    this->m_torso_size = ssize;
    this->m_arm_specs = aspecs;

    // hand
    this->m_barrett_hand_specs = hspecs;

    this->rk.push_back(-1.0);
    this->rk.push_back(1.0);
    this->rk.push_back(0.0);

    this->jk.push_back(-1.0);
    this->jk.push_back(-1.0);
    this->jk.push_back(1.0);


    this->rightPosture = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->leftPosture = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->rightHomePosture = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->leftHomePosture = std::vector<float>(JOINTS_ARM+JOINTS_HAND);

    std::copy(r.begin(),r.end(),this->rightPosture.begin());
    std::copy(l.begin(),l.end(),this->leftPosture.begin());
    std::copy(r.begin(),r.end(),this->rightHomePosture.begin());
    std::copy(l.begin(),l.end(),this->leftHomePosture.begin());

    this->min_rightLimits = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->min_leftLimits = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->max_rightLimits = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->max_leftLimits = std::vector<float>(JOINTS_ARM+JOINTS_HAND);

    std::copy(min_rl.begin(),min_rl.end(),this->min_rightLimits.begin());
    std::copy(min_ll.begin(),min_ll.end(),this->min_leftLimits.begin());
    std::copy(max_rl.begin(),max_rl.end(),this->max_rightLimits.begin());
    std::copy(max_ll.begin(),max_ll.end(),this->max_leftLimits.begin());

    this->rightVelocities= std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->leftVelocities = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->rightForces = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->leftForces = std::vector<float>(JOINTS_ARM+JOINTS_HAND);

    this->mat_right = Matrix4f::Identity(4,4);
    this->mat_left = Matrix4f::Identity(4,4);
    this->mat_r_hand = Matrix4f::Identity(4,4);
    this->mat_l_hand = Matrix4f::Identity(4,4);
    /*
    this->mat_left(0,0) = 1.0; this->mat_left(0,1) = 0.0; this->mat_left(0,2) = 0.0; this->mat_left(0,3) = 0.0;
    this->mat_left(0,0) = 0.0; this->mat_left(0,1) = -1.0; this->mat_left(0,2) = 0.0; this->mat_left(0,3) = 100.0;
    this->mat_left(0,0) = 0.0; this->mat_left(0,1) = 0.0; this->mat_left(0,2) = -1.0; this->mat_left(0,3) = 0.0;
    this->mat_left(0,0) = 0.0; this->mat_left(0,1) = 0.0; this->mat_left(0,2) = 0.0; this->mat_left(0,3) = 1.0;
*/

}
#endif
// copy constructor
/**
 * @brief Humanoid::Humanoid
 * @param hh
 */
Humanoid::Humanoid(const Humanoid &hh){

    this->m_name = hh.m_name;
    this->m_torso_pos = hh.m_torso_pos;
    this->m_torso_or = hh.m_torso_or;
    this->m_torso_size = hh.m_torso_size;
    this->m_arm_specs = hh.m_arm_specs;
#if HAND==0
    this->m_human_hand_specs = hh.m_human_hand_specs; // human hand
#elif HAND==1
    this->m_barrett_hand_specs = hh.m_barrett_hand_specs; // barrett hand
    this->rk = hh.rk;
    this->jk = hh.jk;
#endif
    this->m_DH_rightArm=hh.m_DH_rightArm;
    this->m_DH_leftArm=hh.m_DH_leftArm;
    this->m_DH_rightHand = hh.m_DH_rightHand;
    this->m_DH_leftHand = hh.m_DH_leftHand;

//#if HEAD==1
 //this->head=hh.head;
//#endif
//#if NECK==1
  //  this->neck=hh.neck;
//#endif
//#if PELVIS==1
  //  this->pelvis=hh.pelvis;
//#endif
//#if RIGHT_UPPER_LEG==1
  //  this->right_upper_leg=hh.right_upper_leg;
//#endif
//#if RIGHT_LOWER_LEG==1
  //  this->right_lower_leg=hh.right_lower_leg;
//#endif
//#if RIGHT_FOOT==1
  //  this->right_foot=hh.right_foot;
//#endif
//#if LEFT_UPPER_LEG==1
  //  this->left_upper_leg=hh.left_upper_leg;
//#endif
//#if LEFT_LOWER_LEG==1
  //  this->left_lower_leg=hh.left_lower_leg;
//#endif
//#if LEFT_FOOT==1
  //  this->left_foot=hh.left_foot;
//#endif

    this->mat_right = hh.mat_right;
    this->mat_left = hh.mat_left;
    this->mat_r_hand = hh.mat_r_hand;
    this->mat_l_hand = hh.mat_l_hand;

    this->max_rightLimits = hh.max_rightLimits;
    this->min_rightLimits = hh.min_rightLimits;
    this->max_leftLimits = hh.max_leftLimits;
    this->min_leftLimits=hh.min_leftLimits;
    this->rightPosture = hh.rightPosture;
    this->leftPosture = hh.leftPosture;
    this->rightVelocities = hh.rightVelocities;
    this->leftVelocities = hh.leftVelocities;
    this->rightForces = hh.rightForces;
    this->leftForces = hh.leftForces;
    this->rightHomePosture = hh.rightHomePosture;
    this->leftHomePosture = hh.leftHomePosture;

    this->rightShoulderPos=hh.rightShoulderPos;
    this->rightShoulderOr=hh.rightShoulderOr;
    this->rightElbowPos=hh.rightElbowPos;
    this->rightElbowOr=hh.rightElbowOr;
    this->rightWristPos=hh.rightWristPos;
    this->rightWristOr=hh.rightWristOr;
    this->rightHandPos=hh.rightHandPos;
    this->rightHandOr=hh.rightHandOr;
    this->rightFingers=hh.rightFingers;

    this->leftShoulderPos=hh.leftShoulderPos;
    this->leftShoulderOr=hh.leftShoulderOr;
    this->leftElbowPos=hh.leftElbowPos;
    this->leftElbowOr=hh.leftElbowOr;
    this->leftWristPos=hh.leftWristPos;
    this->leftWristOr=hh.leftWristOr;
    this->leftHandPos=hh.leftHandPos;
    this->leftHandOr=hh.leftHandOr;
    this->leftFingers=hh.leftFingers;



}

// destructor
/**
 * @brief Humanoid::~Humanoid
 */
Humanoid::~Humanoid(){


}

// setters
/**
 * @brief Humanoid::setName
 * @param name
 */
void Humanoid::setName(string& name){

    this->m_name = name;

}
/**
 * @brief Humanoid::setPos
 * @param ppos
 */
void Humanoid::setPos(pos& ppos){

    this->m_torso_pos = ppos;
}

/**
 * @brief Humanoid::setOr
 * @param oor
 */
void Humanoid::setOr(orient& oor){

    this->m_torso_or = oor;
}
/**
 * @brief Humanoid::setSize
 * @param ssize
 */
void Humanoid::setSize(dim& ssize){

    this->m_torso_size = ssize;
}
/**
 * @brief Humanoid::setArm
 * @param specs
 */
void Humanoid::setArm(arm& specs){

    this->m_arm_specs = specs;
}

#if HAND==0
/**
 * @brief Humanoid::setHumanHand
 * @param specs
 */
void Humanoid::setHumanHand(human_hand &specs){

    this->m_human_hand_specs=specs;
}
#elif HAND==1

/**
 * @brief Humanoid::setBarrettHand
 * @param specs
 */
void Humanoid::setBarrettHand(barrett_hand& specs){

    this->m_barrett_hand_specs=specs;
}

#endif

/**
 * @brief Humanoid::setRightPosture
 * @param r
 */
void Humanoid::setRightPosture(std::vector<float> &r){

    //this->rightPosture=r;
    std::copy(r.begin(),r.end(),this->rightPosture.begin());
}
/**
 * @brief Humanoid::setLeftPosture
 * @param l
 */
void Humanoid::setLeftPosture(std::vector<float> &l){

    //this->leftPosture=l;
    std::copy(l.begin(),l.end(),this->leftPosture.begin());
}

/**
 * @brief Humanoid::setRightHomePosture
 * @param r
 */
void Humanoid::setRightHomePosture(std::vector<float> &r){

    //this->rightHomePosture=r;
    std::copy(r.begin(),r.end(),this->rightHomePosture.begin());
}
/**
 * @brief Humanoid::setLeftHomePosture
 * @param l
 */
void Humanoid::setLeftHomePosture(std::vector<float> &l){

    //this->leftHomePosture=l;

    std::copy(l.begin(),l.end(),this->leftHomePosture.begin());
}

void Humanoid::setRightMinLimits(std::vector<float> &min_rl){

    //this->min_rightLimits = min_rl;

    std::copy(min_rl.begin(),min_rl.end(),this->min_rightLimits.begin());
}

void Humanoid::setRightMaxLimits(std::vector<float> &max_rl){

    //this->max_rightLimits = max_rl;

    std::copy(max_rl.begin(),max_rl.end(),this->max_rightLimits.begin());
}

void Humanoid::setLeftMinLimits(std::vector<float> &min_ll){

    //this->min_leftLimits = min_ll;

    std::copy(min_ll.begin(),min_ll.end(),this->min_leftLimits.begin());
}

void Humanoid::setLeftMaxLimits(std::vector<float> &max_ll){

    //this->max_leftLimits = max_ll;

    std::copy(max_ll.begin(),max_ll.end(),this->max_leftLimits.begin());
}

void Humanoid::setRightVelocities(std::vector<float> &r){

    std::copy(r.begin(),r.end(),this->rightVelocities.begin());
}

void Humanoid::setLeftVelocities(std::vector<float> &l){

    std::copy(l.begin(),l.end(),this->leftVelocities.begin());
}


void Humanoid::setRightForces(std::vector<float> &r){

    std::copy(r.begin(),r.end(),this->rightForces.begin());
}


void Humanoid::setLeftForces(std::vector<float> &l){

    std::copy(l.begin(),l.end(),this->leftForces.begin());

}
/**
 * @brief Humanoid::setMatRight
 * @param m
 */
void Humanoid::setMatRight(Matrix4f &m){

    for (unsigned i = 0; i < m.rows(); ++ i){
        for (unsigned j = 0; j < m.cols(); ++ j){
            this->mat_right(i, j) = m(i,j);
        }
    }

}
/**
 * @brief Humanoid::setMatLeft
 * @param m
 */
void Humanoid::setMatLeft(Matrix4f &m){


    for (unsigned i = 0; i < m.rows(); ++ i){
        for (unsigned j = 0; j < m.cols(); ++ j){
            this->mat_left(i, j) = m(i,j);
        }
    }
}


/**
 * @brief Humanoid::setMatRightHand
 * @param m
 */
void Humanoid::setMatRightHand(Matrix4f &m){

    for (unsigned i = 0; i < m.rows(); ++ i){
        for (unsigned j = 0; j < m.cols(); ++ j){
            this->mat_r_hand(i, j) = m(i,j);
        }
    }
}

/**
 * @brief Humanoid::setMatLeftHand
 * @param m
 */
void Humanoid::setMatLeftHand(Matrix4f &m){

    for (unsigned i = 0; i < m.rows(); ++ i){
        for (unsigned j = 0; j < m.cols(); ++ j){
            this->mat_l_hand(i, j) = m(i,j);
        }
    }
}
// humanoid parts

//#if HEAD==1
/**
 * @brief Humanoid::setHead
 * @param head
 */
//void Humanoid::setHead(humanoid_part& head){

  //  this->head=head;
//}
//#endif

//#if NECK==1
/**
 * @brief Humanoid::setNeck
 * @param neck
 */
//void Humanoid::setNeck(humanoid_part& neck){

  //  this->neck=neck;
//}
//#endif

//#if PELVIS==1
/**
 * @brief Humanoid::setPelvis
 * @param pelvis
 */
//void Humanoid::setPelvis(humanoid_part& pelvis){

  //  this->pelvis=pelvis;

//}
//#endif

//#if RIGHT_UPPER_LEG==1
/**
 * @brief Humanoid::setRight_Upper_leg
 * @param right_upper_leg
 */
//void Humanoid::setRight_Upper_leg(humanoid_part& right_upper_leg){

  //  this->right_upper_leg=right_upper_leg;
//}
//#endif

//#if RIGHT_LOWER_LEG==1
/**
 * @brief Humanoid::setRight_Lower_leg
 * @param right_lower_leg
 */
//void Humanoid::setRight_Lower_leg(humanoid_part& right_lower_leg){

  //  this->right_lower_leg=right_lower_leg;
//}
//#endif

//#if RIGHT_FOOT==1
/**
 * @brief Humanoid::setRight_foot
 * @param right_foot
 */
//void Humanoid::setRight_foot(humanoid_part& right_foot){

  //  this->right_foot=right_foot;
//}
//#endif

//#if LEFT_UPPER_LEG==1
/**
 * @brief Humanoid::setLeft_Upper_leg
 * @param left_upper_leg
 */
//void Humanoid::setLeft_Upper_leg(humanoid_part& left_upper_leg){

  //  this->left_upper_leg=left_upper_leg;
//}
//#endif

//#if LEFT_LOWER_LEG==1
/**
 * @brief Humanoid::setLeft_Lower_leg
 * @param left_lower_leg
 */
//void Humanoid::setLeft_Lower_leg(humanoid_part& left_lower_leg){

  //  this->left_lower_leg=left_lower_leg;
//}
//#endif

//#if LEFT_FOOT==1
/**
 * @brief Humanoid::setLeft_foot
 * @param left_foot
 */
//void Humanoid::setLeft_foot(humanoid_part& left_foot){

  //  this->left_foot=left_foot;
//}
//#endif


//getters
/**
 * @brief Humanoid::getName
 * @return
 */
string Humanoid::getName(){

    return this->m_name;
}
/**
 * @brief Humanoid::getPos
 * @return
 */
pos Humanoid::getPos(){

    return this->m_torso_pos;
}

/**
 * @brief Humanoid::getOr
 * @return
 */
orient Humanoid::getOr(){

    return this->m_torso_or;
}
/**
 * @brief Humanoid::getSize
 * @return
 */
dim Humanoid::getSize(){

    return this->m_torso_size;
}

#if HAND ==1
/**
 * @brief Humanoid::getRK
 * @param rkk
 */
void Humanoid::getRK(std::vector<float> &rkk){

    for (int i=0; i < this->rk.size(); ++i){

        rkk.push_back(this->rk.at(i));


    }

}
/**
 * @brief Humanoid::getJK
 * @param jkk
 */
void Humanoid::getJK(std::vector<float> &jkk){

    for(int i=0; i <this->jk.size(); ++i){

        jkk.push_back(this->jk.at(i));

    }

}

#endif

/**
 * @brief Humanoid::getArm
 * @return
 */
arm Humanoid::getArm(){

    return this->m_arm_specs;
}

#if HAND==0
human_hand Humanoid::getHumanHand(){

    return this->m_human_hand_specs;
}
#elif HAND==1
/**
 * @brief Humanoid::getBarrettHand
 * @return
 */
barrett_hand Humanoid::getBarrettHand(){

    return this->m_barrett_hand_specs;
}
#endif


/**
 * @brief Humanoid::getRightPosture
 * @return
 */
void Humanoid::getRightPosture(std::vector<float>& p){

    p = std::vector<float>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->rightPosture.begin(),this->rightPosture.end(),p.begin());
/*
    for(std::vector<float>::iterator i = this->rightPosture.begin(); i !=this->rightPosture.end();++i){

        p.push_back(*i);

    }
    */
}
/**
 * @brief Humanoid::getRightArmPosture
 * @param p
 */
void Humanoid::getRightArmPosture(std::vector<float> &p){

     p = std::vector<float>(JOINTS_ARM);

    std::copy(this->rightPosture.begin(),this->rightPosture.end()-JOINTS_HAND,p.begin());

    /*

    for(std::vector<float>::iterator i = this->rightPosture.begin(); i !=this->rightPosture.end()-JOINTS_HAND;++i){

        p.push_back(*i);

    }

    */
}

/**
 * @brief Humanoid::getRightHandPosture
 * @param p
 */
void Humanoid::getRightHandPosture(std::vector<float> &p){

     p = std::vector<float>(JOINTS_HAND);

    std::copy(this->rightPosture.begin()+JOINTS_ARM,this->rightPosture.end(),p.begin());
/*
    for(std::vector<float>::iterator i = this->rightPosture.begin()+JOINTS_ARM; i !=this->rightPosture.end();++i){

        p.push_back(*i);

    }
    */

}

/**
 * @brief Humanoid::getLeftPosture
 * @return
 */
void Humanoid::getLeftPosture(std::vector<float>& p){

    p = std::vector<float>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->leftPosture.begin(),this->leftPosture.end(),p.begin());

    /*

    for(std::vector<float>::iterator i = this->leftPosture.begin(); i !=this->leftPosture.end();++i){

        p.push_back(*i);
    }

    */
}
/**
 * @brief Humanoid::getLeftArmPosture
 * @param p
 */
void Humanoid::getLeftArmPosture(std::vector<float> &p){

     p = std::vector<float>(JOINTS_ARM);

    std::copy(this->leftPosture.begin(),this->leftPosture.end()-JOINTS_HAND,p.begin());
/*

    for(std::vector<float>::iterator i = this->leftPosture.begin(); i !=this->leftPosture.end()-JOINTS_HAND;++i){

        p.push_back(*i);
    }

    */

}
/**
 * @brief Humanoid::getLeftHandPosture
 * @param p
 */
void Humanoid::getLeftHandPosture(std::vector<float> &p){

     p = std::vector<float>(JOINTS_HAND);

    std::copy(this->leftPosture.begin()+JOINTS_ARM,this->leftPosture.end(),p.begin());

/*
    for(std::vector<float>::iterator i = this->leftPosture.begin()+JOINTS_ARM; i !=this->leftPosture.end();++i){

        p.push_back(*i);
    }

    */
}

/**
 * @brief Humanoid::getRightHomePosture
 * @return
 */
void Humanoid::getRightHomePosture(std::vector<float>& p){

     p = std::vector<float>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->rightHomePosture.begin(),this->rightHomePosture.end(),p.begin());

    /*

    for(std::vector<float>::iterator i = this->rightHomePosture.begin(); i !=this->rightHomePosture.end();++i){

        p.push_back(*i);
    }

    */
}
/**
 * @brief Humanoid::getLeftHomePosture
 * @return
 */
void Humanoid::getLeftHomePosture(std::vector<float>& p){

     p = std::vector<float>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->leftHomePosture.begin(),this->leftHomePosture.end(),p.begin());

    /*

    for(std::vector<float>::iterator i = this->leftHomePosture.begin(); i !=this->leftHomePosture.end();++i){

        p.push_back(*i);
    }

    */
}


void Humanoid::getRightMinLimits(std::vector<float> &p){

     p = std::vector<float>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->min_rightLimits.begin(),this->min_rightLimits.end(),p.begin());

    /*

    for(std::vector<float>::iterator i = this->min_rightLimits.begin(); i !=this->min_rightLimits.end();++i){

        p.push_back(*i);
    }
    */
}

void Humanoid::getRightMaxLimits(std::vector<float> &p){

     p = std::vector<float>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->max_rightLimits.begin(),this->max_rightLimits.end(),p.begin());

    /*

    for(std::vector<float>::iterator i = this->max_rightLimits.begin(); i !=this->max_rightLimits.end();++i){

        p.push_back(*i);
    }

    */
}

void Humanoid::getLeftMinLimits(std::vector<float> &p){

     p = std::vector<float>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->min_leftLimits.begin(),this->min_leftLimits.end(),p.begin());

    /*

    for(std::vector<float>::iterator i = this->min_leftLimits.begin(); i !=this->min_leftLimits.end();++i){

        p.push_back(*i);
    }

    */

}

void Humanoid::getLeftMaxLimits(std::vector<float> &p){

     p = std::vector<float>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->max_leftLimits.begin(),this->max_leftLimits.end(),p.begin());

    /*

    for(std::vector<float>::iterator i = this->max_leftLimits.begin(); i !=this->max_leftLimits.end();++i){

        p.push_back(*i);
    }
    */
}

void Humanoid::getRightVelocities(std::vector<float> &p){

     p = std::vector<float>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->rightVelocities.begin(),this->rightVelocities.end(),p.begin());
}

void Humanoid::getRightArmVelocities(std::vector<float> &p){

     p = std::vector<float>(JOINTS_ARM);

     std::copy(this->rightVelocities.begin(),this->rightVelocities.end()-JOINTS_HAND,p.begin());

}

void Humanoid::getRightHandVelocities(std::vector<float> &p){

     p = std::vector<float>(JOINTS_HAND);

     std::copy(this->rightVelocities.begin()+JOINTS_ARM,this->rightVelocities.end(),p.begin());
}

void Humanoid::getLeftVelocities(std::vector<float> &p){

     p = std::vector<float>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->leftVelocities.begin(),this->leftVelocities.end(),p.begin());
}

void Humanoid::getLeftArmVelocities(std::vector<float> &p){

     p = std::vector<float>(JOINTS_ARM);

     std::copy(this->leftVelocities.begin(),this->leftVelocities.end()-JOINTS_HAND,p.begin());
}

void Humanoid::getLeftHandVelocities(std::vector<float> &p){

     p = std::vector<float>(JOINTS_HAND);

     std::copy(this->leftVelocities.begin()+JOINTS_ARM,this->leftVelocities.end(),p.begin());
}

void Humanoid::getRightForces(std::vector<float> &p){

     p = std::vector<float>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->rightForces.begin(),this->rightForces.end(),p.begin());
}

void Humanoid::getRightArmForces(std::vector<float> &p){

     p = std::vector<float>(JOINTS_ARM);

     std::copy(this->rightForces.begin(),this->rightForces.end()-JOINTS_HAND,p.begin());
}

void Humanoid::getRightHandForces(std::vector<float> &p){

     p = std::vector<float>(JOINTS_HAND);

     std::copy(this->rightForces.begin()+JOINTS_ARM,this->rightForces.end(),p.begin());
}

void Humanoid::getLeftForces(std::vector<float> &p){

     p = std::vector<float>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->leftForces.begin(),this->leftForces.end(),p.begin());
}

void Humanoid::getLeftArmForces(std::vector<float> &p){

     p = std::vector<float>(JOINTS_ARM);

     std::copy(this->leftForces.begin(),this->leftForces.end()-JOINTS_HAND,p.begin());
}

void Humanoid::getLeftHandForces(std::vector<float> &p){

     p = std::vector<float>(JOINTS_HAND);

     std::copy(this->leftForces.begin()+JOINTS_ARM,this->leftForces.end(),p.begin());
}
/**
 * @brief Humanoid::getMatRight
 * @param m
 */
void Humanoid::getMatRight(Matrix4f &m){


    for (unsigned i = 0; i < m.rows(); ++ i){
        for (unsigned j = 0; j < m.cols(); ++ j){
            m (i, j) = this->mat_right(i, j);
        }
    }
}
/**
 * @brief Humanoid::getMatLeft
 * @param m
 */
void Humanoid::getMatLeft(Matrix4f &m){

    for (unsigned i = 0; i < m.rows(); ++ i){
        for (unsigned j = 0; j < m.cols(); ++ j){
            m (i, j) = this->mat_left(i, j);
        }
    }
}
/**
 * @brief Humanoid::getMatRightHand
 * @param m
 */
void Humanoid::getMatRightHand(Matrix4f &m){


    for (unsigned i = 0; i < m.rows(); ++ i){
        for (unsigned j = 0; j < m.cols(); ++ j){
            m (i, j) = this->mat_r_hand(i, j);
        }
    }
}
/**
 * @brief Humanoid::getMatLeftHand
 * @param m
 */
void Humanoid::getMatLeftHand(Matrix4f &m){

    for (unsigned i = 0; i < m.rows(); ++ i){
        for (unsigned j = 0; j < m.cols(); ++ j){
            m (i, j) = this->mat_l_hand(i, j);
        }
    }
}

//info
/**
 * @brief Humanoid::getInfoLine
 * @return
 */
string Humanoid::getInfoLine(){

    return  this->m_name + COLUMN + SPACE +
            XposSTR + str(boost::format("%d") % this->m_torso_pos.Xpos) + MILLIMETERS + SEP +
            YposSTR + str(boost::format("%d") % this->m_torso_pos.Ypos) + MILLIMETERS + SEP+
            ZposSTR + str(boost::format("%d") % this->m_torso_pos.Zpos) + MILLIMETERS + SEP+
            RollSTR + str(boost::format("%d") % this->m_torso_or.roll) + RAD + SEP+
            PitchSTR + str(boost::format("%d") % this->m_torso_or.pitch) + RAD + SEP+
            YawSTR + str(boost::format("%d") % this->m_torso_or.yaw) + RAD + SEP+
            XsizeSTR + str(boost::format("%d") % this->m_torso_size.Xsize) + MILLIMETERS + SEP+
            YsizeSTR + str(boost::format("%d") % this->m_torso_size.Ysize) + MILLIMETERS + SEP+
            ZsizeSTR + str(boost::format("%d") % this->m_torso_size.Zsize)+ MILLIMETERS;
            //L1STR + str(boost::format("%d") % this->m_arm_specs.arm_specs.d.at(0))+ MILLIMETERS + SEP+
            //LuSTR + str(boost::format("%d") % this->m_arm_specs.arm_specs.d.at(2))+ MILLIMETERS + SEP+
            //LlSTR + str(boost::format("%d") % this->m_arm_specs.arm_specs.d.at(4)) + MILLIMETERS + SEP+
            //LhSTR + str(boost::format("%d") % this->m_arm_specs.arm_specs.d.at(6)) + MILLIMETERS;

}

/**
 * @brief Humanoid::getRightShoulderPos
 * @param pos
 */
void Humanoid::getRightShoulderPos(std::vector<float> &pos){

    // direct kinematics of the right arm
    directKinematicsSingleArm(1);

    pos = rightShoulderPos;
}
/**
 * @brief Humanoid::getRightShoulderNorm
 * @return
 */
float Humanoid::getRightShoulderNorm(){

    std::vector<float> pos;

    this->getRightShoulderPos(pos);

    return sqrt(pow(pos.at(0),2)+pow(pos.at(1),2)+pow(pos.at(2),2));

}

/**
 * @brief Humanoid::getRightShoulderOr
 * @param orr
 */
void Humanoid::getRightShoulderOr(Matrix3f &orr){

    // direct kinematics of the right arm
    directKinematicsSingleArm(1);

    orr=rightShoulderOr;

}
/**
 * @brief Humanoid::getRightElbowPos
 * @param pos
 */
void Humanoid::getRightElbowPos(std::vector<float> &pos){

    // direct kinematics of the right arm
    directKinematicsSingleArm(1);

    pos = rightElbowPos;
}
/**
 * @brief Humanoid::getRightElbowNorm
 * @return
 */
float Humanoid::getRightElbowNorm(){

    std::vector<float> pos;

    this->getRightElbowPos(pos);

    return sqrt(pow(pos.at(0),2)+pow(pos.at(1),2)+pow(pos.at(2),2));


}

/**
 * @brief Humanoid::getRightElbowOr
 * @param orr
 */
void Humanoid::getRightElbowOr(Matrix3f &orr){

    // direct kinematics of the right arm
    directKinematicsSingleArm(1);

    orr = rightElbowOr;
}

/**
 * @brief Humanoid::getRightWristPos
 * @param pos
 */
void Humanoid::getRightWristPos(std::vector<float> &pos){

    // direct kinematics of the right arm
    directKinematicsSingleArm(1);

    pos = rightWristPos;
}
/**
 * @brief Humanoid::getRightWristNorm
 * @return
 */
float Humanoid::getRightWristNorm(){

    std::vector<float> pos;

    this->getRightWristPos(pos);

    return sqrt(pow(pos.at(0),2)+pow(pos.at(1),2)+pow(pos.at(2),2));
}

/**
 * @brief Humanoid::getRightWristOr
 * @param orr
 */
void Humanoid::getRightWristOr(Matrix3f &orr){

    // direct kinematics of the right arm
    directKinematicsSingleArm(1);

    orr = rightWristOr;
}
/**
 * @brief Humanoid::getRightHandPos
 * @param pos
 */
void Humanoid::getRightHandPos(std::vector<float> &pos){

    // direct kinematics of the right arm
    directKinematicsSingleArm(1);

    pos = rightHandPos;
}
/**
 * @brief Humanoid::getRightHandNorm
 * @return
 */
float Humanoid::getRightHandNorm(){

    std::vector<float> pos;

    this->getRightHandPos(pos);

    return sqrt(pow(pos.at(0),2)+pow(pos.at(1),2)+pow(pos.at(2),2));
}

/**
 * @brief Humanoid::getRightHandOr
 * @param orr
 */
void Humanoid::getRightHandOr(Matrix3f &orr){

    // direct kinematics of the right arm
    directKinematicsSingleArm(1);

    orr = rightHandOr;
}
/**
 * @brief Humanoid::getLeftShoulderPos
 * @param pos
 */
void Humanoid::getLeftShoulderPos(std::vector<float> &pos){

    // direct kinematics of the left arm
    directKinematicsSingleArm(2);

    pos = leftShoulderPos;
}

/**
 * @brief Humanoid::getLeftShoulderNorm
 * @return
 */
float Humanoid::getLeftShoulderNorm(){

    std::vector<float> pos;

    this->getLeftShoulderPos(pos);

    return sqrt(pow(pos.at(0),2)+pow(pos.at(1),2)+pow(pos.at(2),2));
}

/**
 * @brief Humanoid::getLeftShoulderOr
 * @param orr
 */
void Humanoid::getLeftShoulderOr(Matrix3f &orr){

    // direct kinematics of the left arm
    directKinematicsSingleArm(2);

    orr = leftShoulderOr;
}

/**
 * @brief Humanoid::getLeftElbowPos
 * @param pos
 */
void Humanoid::getLeftElbowPos(std::vector<float> &pos){

    // direct kinematics of the left arm
    directKinematicsSingleArm(2);

    pos = leftElbowPos;

}
/**
 * @brief Humanoid::getLeftElbowNorm
 * @return
 */
float Humanoid::getLeftElbowNorm(){

    std::vector<float> pos;

    this->getLeftElbowPos(pos);

    return sqrt(pow(pos.at(0),2)+pow(pos.at(1),2)+pow(pos.at(2),2));
}

/**
 * @brief Humanoid::getLeftElbowOr
 * @param orr
 */
void Humanoid::getLeftElbowOr(Matrix3f &orr){

    // direct kinematics of the left arm
    directKinematicsSingleArm(2);

    orr = leftElbowOr;
}

/**
 * @brief Humanoid::getLeftWristPos
 * @param pos
 */
void Humanoid::getLeftWristPos(std::vector<float> &pos){

    // direct kinematics of the left arm
    directKinematicsSingleArm(2);

    pos = leftWristPos;
}
/**
 * @brief Humanoid::getLeftWristNorm
 * @return
 */
float Humanoid::getLeftWristNorm(){

    std::vector<float> pos;

    this->getLeftWristPos(pos);

    return sqrt(pow(pos.at(0),2)+pow(pos.at(1),2)+pow(pos.at(2),2));
}

/**
 * @brief Humanoid::getLeftWristOr
 * @param orr
 */
void Humanoid::getLeftWristOr(Matrix3f &orr){

    // direct kinematics of the left arm
    directKinematicsSingleArm(2);

    orr = leftWristOr;
}

/**
 * @brief Humanoid::getLeftHandPos
 * @param pos
 */
void Humanoid::getLeftHandPos(std::vector<float> &pos){

    // direct kinematics of the left arm
    directKinematicsSingleArm(2);

    pos = leftHandPos;
}
/**
 * @brief Humanoid::getLeftHandNorm
 * @return
 */
float Humanoid::getLeftHandNorm(){

    std::vector<float> pos;

    this->getLeftHandPos(pos);

    return sqrt(pow(pos.at(0),2)+pow(pos.at(1),2)+pow(pos.at(2),2));
}

/**
 * @brief Humanoid::getLeftHandOr
 * @param orr
 */
void Humanoid::getLeftHandOr(Matrix3f &orr){

    // direct kinematics of the left arm
    directKinematicsSingleArm(2);

    orr = leftHandOr;
}

//#if HEAD==1
/**
 * @brief Humanoid::getHead
 * @return
 */
//humanoid_part Humanoid::getHead(){
  //  return this->head;
//}
//#endif

//#if NECK==1

//#endif

//#if PELVIS==1
/**
 * @brief Humanoid::getPelvis
 * @return
 */
//humanoid_part Humanoid::getPelvis(){

  //  return this->pelvis;
//}
//#endif

//#if RIGHT_UPPER_LEG==1
/**
 * @brief Humanoid::getRight_Upper_leg
 * @return
 */
//humanoid_part Humanoid::getRight_Upper_leg(){

  //  return this->right_upper_leg;
//}
//#endif

//#if RIGHT_LOWER_LEG==1
/**
 * @brief Humanoid::getRight_Lower_leg
 * @return
 */
//humanoid_part Humanoid::getRight_Lower_leg(){

  //  return this->right_lower_leg;
//}
//#endif

//#if RIGHT_FOOT==1
/**
 * @brief Humanoid::getRight_foot
 * @return
 */
//humanoid_part Humanoid::getRight_foot(){

  //  return this->right_foot;
//}
//#endif

//#if LEFT_UPPER_LEG==1
/**
 * @brief Humanoid::getLeft_Upper_leg
 * @return
 */
//humanoid_part Humanoid::getLeft_Upper_leg(){

  //  return this->left_upper_leg;

//}
//#endif

//#if LEFT_LOWER_LEG==1
/**
 * @brief Humanoid::getLeft_Lower_leg
 * @return
 */
//humanoid_part Humanoid::getLeft_Lower_leg(){

  //  return this->left_lower_leg;

//}
//#endif

//#if LEFT_FOOT==1
/**
 * @brief Humanoid::getLeft_foot
 * @return
 */
//humanoid_part Humanoid::getLeft_foot(){

  //  return this->left_foot;

//}
//#endif

/**
 * @brief Humanoid::directKinematicsDualArm
 */
void Humanoid::directKinematicsDualArm(){


    directKinematicsSingleArm(1);// right arm
    directKinematicsSingleArm(2);// left arm

}
/**
 * @brief Humanoid::computeRightArmDHparams
 */
void Humanoid::computeRightArmDHparams(){

    this->m_DH_rightArm.a.clear();
    this->m_DH_rightArm.d.clear();
    this->m_DH_rightArm.alpha.clear();
    this->m_DH_rightArm.theta.clear();

    for (int i = 0; i < JOINTS_ARM; ++i){

    // d [mm]
    m_DH_rightArm.d.push_back(m_arm_specs.arm_specs.d.at(i));

    //a [mm]
    m_DH_rightArm.a.push_back(m_arm_specs.arm_specs.a.at(i));

    //alpha [rad]
    m_DH_rightArm.alpha.push_back(m_arm_specs.arm_specs.alpha.at(i));

    //theta [rad]
    m_DH_rightArm.theta.push_back(rightPosture.at(i));

    }


}
/**
 * @brief Humanoid::computeLeftArmDHparams
 */
void Humanoid::computeLeftArmDHparams(){

    this->m_DH_leftArm.a.clear();
    this->m_DH_leftArm.d.clear();
    this->m_DH_leftArm.alpha.clear();
    this->m_DH_leftArm.theta.clear();

    for (int i = 0; i < JOINTS_ARM; ++i){

    // d [mm]
    m_DH_leftArm.d.push_back(-m_arm_specs.arm_specs.d.at(i));

    //a [mm]
    m_DH_leftArm.a.push_back(m_arm_specs.arm_specs.a.at(i));

    //alpha [rad]
    if ((i == 0)){
        m_DH_leftArm.alpha.push_back(m_arm_specs.arm_specs.alpha.at(i));
    }else{
        m_DH_leftArm.alpha.push_back(-m_arm_specs.arm_specs.alpha.at(i));
    }

    //theta [rad]
    m_DH_leftArm.theta.push_back(leftPosture.at(i));
    }


}
/**
 * @brief Humanoid::computeRightHandDHparams
 */
void Humanoid::computeRightHandDHparams(){

    this->m_DH_rightHand.clear();

    for (int i = 0; i< HAND_FINGERS; ++i){

        std::vector<float> t;
        this->getRightHandPosture(t);

        DHparams f;
        std::vector<float> fing_pos;


#if HAND==0
        if (i==0){


            f.a = std::vector<float>(4);
            f.d = std::vector<float>(4);
            f.alpha = std::vector<float>(4);
            f.theta = std::vector<float>(4);
            human_finger fing = m_human_hand_specs.fingers.at(0); // index
            // finger positions [mm]
            fing_pos.push_back(fing.ux);
            fing_pos.push_back(fing.uy);
            fing_pos.push_back(fing.uz);
            //a [mm]
            f.a.at(0) = fing.finger_specs.a.at(0);
            f.a.at(1) = fing.finger_specs.a.at(1);
            f.a.at(2) = fing.finger_specs.a.at(2);
            f.a.at(3) = fing.finger_specs.a.at(3);
            //d [mm]
            f.d.at(0) = fing.finger_specs.d.at(0);
            f.d.at(1) = fing.finger_specs.d.at(1);
            f.d.at(2) = fing.finger_specs.d.at(2);
            f.d.at(3) = fing.finger_specs.d.at(3);
            //alpha [rad]
            f.alpha.at(0) = fing.finger_specs.alpha.at(0);
            f.alpha.at(1) = fing.finger_specs.alpha.at(1);
            f.alpha.at(2) = fing.finger_specs.alpha.at(2);
            f.alpha.at(3) = fing.finger_specs.alpha.at(3);
            //theta [rad]
            f.theta.at(0) = fing.finger_specs.theta.at(0);
            f.theta.at(1) = fing.finger_specs.theta.at(1);
            f.theta.at(2) = fing.finger_specs.theta.at(2);
            f.theta.at(3) = fing.finger_specs.theta.at(3);


        }else if(i==1){
            f.a = std::vector<float>(4);
            f.d = std::vector<float>(4);
            f.alpha = std::vector<float>(4);
            f.theta = std::vector<float>(4);

            human_finger fing = m_human_hand_specs.fingers.at(2); // ring
            // finger positions [mm]
            fing_pos.push_back(fing.ux);
            fing_pos.push_back(fing.uy);
            fing_pos.push_back(fing.uz);
            //a [mm]
            f.a.at(0) = fing.finger_specs.a.at(0);
            f.a.at(1) = fing.finger_specs.a.at(1);
            f.a.at(2) = fing.finger_specs.a.at(2);
            f.a.at(3) = fing.finger_specs.a.at(3);
            //d [mm]
            f.d.at(0) = fing.finger_specs.d.at(0);
            f.d.at(1) = fing.finger_specs.d.at(1);
            f.d.at(2) = fing.finger_specs.d.at(2);
            f.d.at(3) = fing.finger_specs.d.at(3);
            //alpha [rad]
            f.alpha.at(0) = fing.finger_specs.alpha.at(0);
            f.alpha.at(1) = fing.finger_specs.alpha.at(1);
            f.alpha.at(2) = fing.finger_specs.alpha.at(2);
            f.alpha.at(3) = fing.finger_specs.alpha.at(3);
            //theta [rad]
            f.theta.at(0) = fing.finger_specs.theta.at(0);
            f.theta.at(1) = fing.finger_specs.theta.at(1);
            f.theta.at(2) = fing.finger_specs.theta.at(2);
            f.theta.at(3) = fing.finger_specs.theta.at(3);

        }else if(i==2){

            f.a = std::vector<float>(5);
            f.d = std::vector<float>(5);
            f.alpha = std::vector<float>(5);
            f.theta = std::vector<float>(5);

            human_thumb thumb = m_human_hand_specs.thumb; // thumb
            // finger positions [mm]
            fing_pos.push_back(thumb.uTx);
            fing_pos.push_back(thumb.uTy);
            fing_pos.push_back(thumb.uTz);
            //a [mm]
            f.a.at(0) = thumb.thumb_specs.a.at(0);
            f.a.at(1) = thumb.thumb_specs.a.at(1);
            f.a.at(2) = thumb.thumb_specs.a.at(2);
            f.a.at(3) = thumb.thumb_specs.a.at(3);
            f.a.at(4) = thumb.thumb_specs.a.at(4);
            // d [mm]
            f.d.at(0) = thumb.thumb_specs.d.at(0);
            f.d.at(1) = thumb.thumb_specs.d.at(1);
            f.d.at(2) = thumb.thumb_specs.d.at(2);
            f.d.at(3) = thumb.thumb_specs.d.at(3);
            f.d.at(4) = thumb.thumb_specs.d.at(4);
            // alpha [rad]
            f.alpha.at(0) = thumb.thumb_specs.alpha.at(0);
            f.alpha.at(1) = thumb.thumb_specs.alpha.at(1);
            f.alpha.at(2) = thumb.thumb_specs.alpha.at(2);
            f.alpha.at(3) = thumb.thumb_specs.alpha.at(3);
            f.alpha.at(4) = thumb.thumb_specs.alpha.at(4);
            // theta [rad]
            f.theta.at(0) = thumb.thumb_specs.theta.at(0);
            f.theta.at(1) = thumb.thumb_specs.theta.at(1);
            f.theta.at(2) = thumb.thumb_specs.theta.at(2);
            f.theta.at(3) = thumb.thumb_specs.theta.at(3);
            f.theta.at(4) = thumb.thumb_specs.theta.at(4);



        }

#elif HAND==1

        f.a = std::vector<float>(4);
        f.d = std::vector<float>(4);
        f.alpha = std::vector<float>(4);
        f.theta = std::vector<float>(4);

        // finger positions [mm]
        fing_pos.push_back(0);
        fing_pos.push_back(0);
        fing_pos.push_back(0);

        //a [mm]
        f.a.at(0) = (rk.at(i)*(m_barrett_hand_specs.Aw));
        f.a.at(1) = m_barrett_hand_specs.A1;
        f.a.at(2) = m_barrett_hand_specs.A2;
        f.a.at(3) = m_barrett_hand_specs.A3;

        //d [mm]
        f.d.at(0) = 0.0;
        f.d.at(1) = 0.0;
        f.d.at(2) = 0.0;
        f.d.at(3) = m_barrett_hand_specs.D3;

        //alpha [rad]
        f.alpha.at(0) = 0.0;
        f.alpha.at(1) = 1.57;
        f.alpha.at(2) = 0.0;
        f.alpha.at(3) = -1.57;

        //theta [rad]
        f.theta.at(0) = (rk.at(i)*t.at(0))-1.57*jk.at(i);
        f.theta.at(1) = m_barrett_hand_specs.phi2+t.at(i+1);
        f.theta.at(2) = m_barrett_hand_specs.phi3+(1/3)*t.at(i+1);
        f.theta.at(3) = 0.0;

#endif

          m_DH_rightHand.push_back(f);
          right_fing_pos.push_back(fing_pos);

    }


}
/**
 * @brief Humanoid::computeLeftHandDHparams
 */
void Humanoid::computeLeftHandDHparams(){


    this->m_DH_leftHand.clear();


    for (int i = 0; i< HAND_FINGERS; ++i){

        DHparams f;
        std::vector<float> fing_pos;

        std::vector<float> t;
        this->getLeftHandPosture(t);


#if HAND==0
        if (i==0){

            f.a = std::vector<float>(4);
            f.d = std::vector<float>(4);
            f.alpha = std::vector<float>(4);
            f.theta = std::vector<float>(4);
            human_finger fing = m_human_hand_specs.fingers.at(0); // index
            // finger positions [mm]
            fing_pos.push_back(fing.ux);
            fing_pos.push_back(fing.uy);
            fing_pos.push_back(fing.uz);
            //a [mm]
            f.a.at(0) = fing.finger_specs.a.at(0);
            f.a.at(1) = fing.finger_specs.a.at(1);
            f.a.at(2) = fing.finger_specs.a.at(2);
            f.a.at(3) = fing.finger_specs.a.at(3);
            //d [mm]
            f.d.at(0) = fing.finger_specs.d.at(0);
            f.d.at(1) = fing.finger_specs.d.at(1);
            f.d.at(2) = fing.finger_specs.d.at(2);
            f.d.at(3) = fing.finger_specs.d.at(3);
            //alpha [rad]
            f.alpha.at(0) = fing.finger_specs.alpha.at(0);
            f.alpha.at(1) = fing.finger_specs.alpha.at(1);
            f.alpha.at(2) = fing.finger_specs.alpha.at(2);
            f.alpha.at(3) = fing.finger_specs.alpha.at(3);
            //theta [rad]
            f.theta.at(0) = fing.finger_specs.theta.at(0);
            f.theta.at(1) = fing.finger_specs.theta.at(1);
            f.theta.at(2) = fing.finger_specs.theta.at(2);
            f.theta.at(3) = fing.finger_specs.theta.at(3);


        }else if(i==1){
            f.a = std::vector<float>(4);
            f.d = std::vector<float>(4);
            f.alpha = std::vector<float>(4);
            f.theta = std::vector<float>(4);

            human_finger fing = m_human_hand_specs.fingers.at(2); // ring
            // finger positions [mm]
            fing_pos.push_back(fing.ux);
            fing_pos.push_back(fing.uy);
            fing_pos.push_back(fing.uz);
            //a [mm]
            f.a.at(0) = fing.finger_specs.a.at(0);
            f.a.at(1) = fing.finger_specs.a.at(1);
            f.a.at(2) = fing.finger_specs.a.at(2);
            f.a.at(3) = fing.finger_specs.a.at(3);
            //d [mm]
            f.d.at(0) = fing.finger_specs.d.at(0);
            f.d.at(1) = fing.finger_specs.d.at(1);
            f.d.at(2) = fing.finger_specs.d.at(2);
            f.d.at(3) = fing.finger_specs.d.at(3);
            //alpha [rad]
            f.alpha.at(0) = fing.finger_specs.alpha.at(0);
            f.alpha.at(1) = fing.finger_specs.alpha.at(1);
            f.alpha.at(2) = fing.finger_specs.alpha.at(2);
            f.alpha.at(3) = fing.finger_specs.alpha.at(3);
            //theta [rad]
            f.theta.at(0) = fing.finger_specs.theta.at(0);
            f.theta.at(1) = fing.finger_specs.theta.at(1);
            f.theta.at(2) = fing.finger_specs.theta.at(2);
            f.theta.at(3) = fing.finger_specs.theta.at(3);

        }else if(i==2){

            f.a = std::vector<float>(5);
            f.d = std::vector<float>(5);
            f.alpha = std::vector<float>(5);
            f.theta = std::vector<float>(5);

            human_thumb thumb = m_human_hand_specs.thumb; // thumb
            // finger positions [mm]
            fing_pos.push_back(thumb.uTx);
            fing_pos.push_back(thumb.uTy);
            fing_pos.push_back(thumb.uTz);
            //a [mm]
            f.a.at(0) = thumb.thumb_specs.a.at(0);
            f.a.at(1) = thumb.thumb_specs.a.at(1);
            f.a.at(2) = thumb.thumb_specs.a.at(2);
            f.a.at(3) = thumb.thumb_specs.a.at(3);
            f.a.at(4) = thumb.thumb_specs.a.at(4);
            // d [mm]
            f.d.at(0) = thumb.thumb_specs.d.at(0);
            f.d.at(1) = thumb.thumb_specs.d.at(1);
            f.d.at(2) = thumb.thumb_specs.d.at(2);
            f.d.at(3) = thumb.thumb_specs.d.at(3);
            f.d.at(4) = thumb.thumb_specs.d.at(4);
            // alpha [rad]
            f.alpha.at(0) = thumb.thumb_specs.alpha.at(0);
            f.alpha.at(1) = thumb.thumb_specs.alpha.at(1);
            f.alpha.at(2) = thumb.thumb_specs.alpha.at(2);
            f.alpha.at(3) = thumb.thumb_specs.alpha.at(3);
            f.alpha.at(4) = thumb.thumb_specs.alpha.at(4);
            // theta [rad]
            f.theta.at(0) = thumb.thumb_specs.theta.at(0);
            f.theta.at(1) = thumb.thumb_specs.theta.at(1);
            f.theta.at(2) = thumb.thumb_specs.theta.at(2);
            f.theta.at(3) = thumb.thumb_specs.theta.at(3);
            f.theta.at(4) = thumb.thumb_specs.theta.at(4);



        }

#elif HAND==1

        f.a = std::vector<float>(4);
        f.d = std::vector<float>(4);
        f.alpha = std::vector<float>(4);
        f.theta = std::vector<float>(4);

        // finger positions [mm]
        fing_pos.push_back(0);
        fing_pos.push_back(0);
        fing_pos.push_back(0);

        //a [mm]
        f.a.at(0) = (rk.at(i)*(m_barrett_hand_specs.Aw));
        f.a.at(1) = m_barrett_hand_specs.A1;
        f.a.at(2) = m_barrett_hand_specs.A2;
        f.a.at(3) = m_barrett_hand_specs.A3;

        //d [mm]
        f.d.at(0) = 0.0;
        f.d.at(1) = 0.0;
        f.d.at(2) = 0.0;
        f.d.at(3) = m_barrett_hand_specs.D3;

        //alpha [rad]
        f.alpha.at(0) = 0.0;
        f.alpha.at(1) = 1.57;
        f.alpha.at(2) = 0.0;
        f.alpha.at(3) = -1.57;

        //theta [rad]
        f.theta.at(0) = (rk.at(i)*t.at(0))-1.57*jk.at(i);
        f.theta.at(1) = m_barrett_hand_specs.phi2+t.at(i+1);
        f.theta.at(2) = m_barrett_hand_specs.phi3+(1/3)*t.at(i+1);
        f.theta.at(3) = 0.0;
#endif

        m_DH_leftHand.push_back(f);
        left_fing_pos.push_back(fing_pos);


    }


}

/**
 * @brief Humanoid::directKinematicsSingleArm
 * @param arm
 * @param joints
 */
void Humanoid::directKinematicsSingleArm(int arm){

    std::vector<float> posture;
    Matrix4f T;
    Matrix4f T_aux;
    Matrix4f mat_world;
    Matrix4f mat_hand;
    DHparams m_DH_arm;
    std::vector<DHparams> m_DH_hand;

    std::vector<float> shoulderPos = std::vector<float>(3);
    Matrix3f shoulderOr;
    std::vector<float> elbowPos = std::vector<float>(3);
    Matrix3f elbowOr;
    std::vector<float> wristPos = std::vector<float>(3);
    Matrix3f wristOr;
    std::vector<float> handPos = std::vector<float>(3);
    Matrix3f handOr;

    switch (arm) {

    case 1: // right arm

        this->getRightArmPosture(posture);
        mat_world = this->mat_right;
        mat_hand = this->mat_r_hand;
        this->computeRightArmDHparams();
        this->computeRightHandDHparams();
        m_DH_arm = this->m_DH_rightArm;
        m_DH_hand = this->m_DH_rightHand;

        break;

    case 2: //left arm

        this->getLeftArmPosture(posture);
        mat_world = this->mat_left;
        mat_hand = this->mat_l_hand;
        this->computeLeftArmDHparams();
        this->computeLeftHandDHparams();
        m_DH_arm = this->m_DH_leftArm;
        m_DH_hand = this->m_DH_leftHand;

        break;
    }

    T = mat_world;

    for (int i = 0; i < JOINTS_ARM; ++i){

        this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), m_DH_arm.theta.at(i),T_aux);

        T = T * T_aux;
        Vector3f v;

        if (i==0){
            // get the shoulder

            shoulderOr = T.block(0,0,3,3);
            v = T.block(0,3,3,1);
            shoulderPos[0] = v[0];
            shoulderPos[1] = v[1];
            shoulderPos[2] = v[2];

            switch (arm){

            case 1: // right arm

                this->rightShoulderPos = shoulderPos;
                this->rightShoulderOr = shoulderOr;

                break;

            case 2: // left arm

                this->leftShoulderPos = shoulderPos;
                this->leftShoulderOr = shoulderOr;

                break;

            }

        }else if (i==2){

            // get the elbow

            elbowOr = T.block(0,0,3,3);
            v = T.block(0,3,3,1);
            elbowPos[0] = v[0];
            elbowPos[1] = v[1];
            elbowPos[2] = v[2];

            switch(arm){

            case 1: // right arm

                this->rightElbowPos = elbowPos;
                this->rightElbowOr = elbowOr;

                break;

            case 2: // left arm

                this->leftElbowPos = elbowPos;
                this->leftElbowOr = elbowOr;

                break;

            }

        }else if (i==4){

            // get the wrist

            wristOr = T.block(0,0,3,3);
            v = T.block(0,3,3,1);
            wristPos[0] = v[0];
            wristPos[1] = v[1];
            wristPos[2] = v[2];

            switch(arm){

            case 1: // right arm

                this->rightWristPos = wristPos;
                this->rightWristOr = wristOr;

                break;

            case 2: // left arm

                this->leftWristPos = wristPos;
                this->leftWristOr = wristOr;

                break;

            }


        } else if (i==6){

            //get the hand
            T = T * mat_hand;

            handOr = T.block(0,0,3,3);
            v = T.block(0,3,3,1);
            handPos[0] = v[0];
            handPos[1] = v[1];
            handPos[2] = v[2];

            switch(arm){

            case 1: // right arm

                this->rightHandPos = handPos;
                this->rightHandOr = handOr;

                break;

            case 2: // left arm

                this->leftHandPos = handPos;
                this->leftHandOr = handOr;

                break;

            }


        }

    }

    // Direct kinematics of the fingers

    this->rightFingers.resize(HAND_FINGERS,12);
    this->leftFingers.resize(HAND_FINGERS,12);

    Matrix4f T_H_0_pos;
    std::vector<float> fing_pos;

    for (int i=0; i< HAND_FINGERS; ++i){
        DHparams p = m_DH_hand.at(i);
        switch (arm) {
        case 1: // right arm
            fing_pos=this->right_fing_pos.at(i);
            T_H_0_pos(0,3)=fing_pos.at(0);
            T_H_0_pos(1,3)=fing_pos.at(1);
            T_H_0_pos(2,3)=fing_pos.at(2);
            this->directKinematicsFinger(p,T,T_H_0_pos,i,rightFingers);
            break;
        case 2: // left arm
            fing_pos=this->left_fing_pos.at(i);
            T_H_0_pos(0,3)=fing_pos.at(0);
            T_H_0_pos(1,3)=fing_pos.at(1);
            T_H_0_pos(2,3)=fing_pos.at(2);
            this->directKinematicsFinger(p,T,T_H_0_pos,i,leftFingers);
            break;
        }
    }

}
/**
 * @brief Humanoid::transfMatrix
 * It performes the homogeneus transformation matrix given the
 * D-H parameters:
 * - translate by d_i along the z_i axis
 * - rotate counterclockwise  by theta around the z_i axis
 * - translate by a_(i-1) along the x_(i-1)
 * - rotate counterclockwise by alpha_(i-1) around the x_(i-1) axis
 * @param p
 * @param theta
 * @param T
 */
void Humanoid::transfMatrix(float alpha, float a, float d, float theta, Matrix4f &T){


    T(0,0) = cos(theta);            T(0,1) = -sin(theta);            T(0,2) = 0.0;         T(0,3) = a;
    T(1,0) = sin(theta)*cos(alpha); T(1,1) = -cos(theta)*cos(alpha); T(1,2) = -sin(alpha); T(1,3) = -sin(alpha)*d;
    T(2,0) = sin(theta)*sin(alpha); T(2,1) = cos(theta)*sin(alpha);  T(2,2) = cos(alpha);  T(2,3) = cos(alpha)*d;
    T(3,0) = 0.0;                   T(3,1) = 0.0;                    T(3,2) = 0.0;         T(3,3) = 1.0;



}

/**
 * @brief Humanoid::directKinematicsFinger
 * @param p
 * @param T_ext
 * @param T_H_0_pos
 * @param k
 * @param Fingers
 */
void Humanoid::directKinematicsFinger(DHparams& p, Matrix4f& T_ext, Matrix4f& T_H_0_pos, int k, MatrixXf& Fingers){



     Matrix4f T;
     std::vector<float> pos = std::vector<float>(3);
     Matrix4f T_aux;

     for(int i =0; i<T_aux.rows();++i){
         for(int j=0; j<T_aux.cols();++j){
             T_aux(i,j)=T_ext(i,j);
         }
     }

     // translate to the begenning of each finger
     T_aux = T_aux * T_H_0_pos;

#if HAND == 0
     int cnt;
     if (k == 3){
         // thumb
         cnt = N_PHALANGE+2;
     }else{
         cnt = N_PHALANGE+1;
     }
     for (int i=0; i< cnt; ++i){
#elif HAND == 1

     for (int i=0; i< N_PHALANGE+1; ++i){
#endif

         float a = p.a.at(i);
         float d = p.d.at(i);
         float alpha = p.alpha.at(i);
         float theta = p.theta.at(i);

#if HAND == 0

         T(0,0) = cos(theta); T(0,1) = -sin(theta)*cos(alpha);  T(0,2) = sin(theta)*cos(alpha);  T(0,3) = a*cos(theta);
         T(1,0) = sin(theta); T(1,1) = cos(theta)*cos(alpha);   T(1,2) = -cos(theta)*sin(alpha); T(1,3) = a*sin(theta);
         T(2,0) = 0.0;        T(2,1) = sin(alpha);              T(2,2) = cos(alpha);             T(2,3) = d;
         T(3,0) = 0.0;        T(3,1) = 0.0;                     T(3,2) = 0.0;                    T(3,3) = 1.0;

#elif HAND == 1

         T(0,0) = cos(theta);            T(0,1) = -sin(theta);            T(0,2) = 0.0;         T(0,3) = a;
         T(1,0) = sin(theta)*cos(alpha); T(1,1) = -cos(theta)*cos(alpha); T(1,2) = -sin(alpha); T(1,3) = -sin(alpha)*d;
         T(2,0) = sin(theta)*sin(alpha); T(2,1) = cos(theta)*sin(alpha);  T(2,2) = cos(alpha);  T(2,3) = cos(alpha)*d;
         T(3,0) = 0.0;                   T(3,1) = 0.0;                    T(3,2) = 0.0;         T(3,3) = 1.0;

#endif


         T_aux = T_aux * T;

         pos[0] = T_aux(0,3);
         pos[1] = T_aux(1,3);
         pos[2] = T_aux(2,3);

         Fingers(k,3*i) = pos[0]; Fingers(k,3*i+1) = pos[1]; Fingers(k,3*i+2) = pos[2];

     }




}



} // namespace HUMotion
