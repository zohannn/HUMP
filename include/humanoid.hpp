#ifndef HUMANOID_H
#define HUMANOID_H


#include "HUMLconfig.hpp"

namespace HUMotion{



class Humanoid
{

public:
    // constructors
#if HAND==0
    // human hand
    Humanoid(string name, pos ppos, orient oor, dim ssize, arm aspecs, human_hand hspecs);
    Humanoid(string name, pos ppos, orient oor, dim ssize, arm aspecs, human_hand hspecs,
             std::vector<float>& r, std::vector<float>& l);
    Humanoid(string name, pos ppos, orient oor, dim ssize, arm aspecs, human_hand hspecs,
             std::vector<float>& r, std::vector<float>& l,
             std::vector<float>& min_rl, std::vector<float>& max_rl,
             std::vector<float>& min_ll, std::vector<float>& max_ll);
#elif HAND==1
    // barrett hand
    Humanoid(string name, pos ppos, orient oor, dim ssize, arm aspecs, barrett_hand hspecs);
    Humanoid(string name, pos ppos, orient oor, dim ssize, arm aspecs, barrett_hand hspecs,
             std::vector<float>& r, std::vector<float>& l);
    Humanoid(string name, pos ppos, orient oor, dim ssize, arm aspecs, barrett_hand hspecs,
             std::vector<float>& r, std::vector<float>& l,
             std::vector<float>& min_rl, std::vector<float>& max_rl,
             std::vector<float>& min_ll, std::vector<float>& max_ll);
#endif

    // copty constructor
    Humanoid(const Humanoid& hh);

    //destructor
    ~Humanoid();

    // **** setters **** //
    void setName(string& name);
    void setPos(pos& ppos);
    void setOr(orient& oor);
    void setSize(dim& ssize);
    void setArm(arm& specs);
#if HAND==0
    void setHumanHand(human_hand& specs);
#elif HAND==1
    void setBarrettHand(barrett_hand& specs);
#endif

    // postures
    void setRightPosture(std::vector<float>& r);
    void setLeftPosture(std::vector<float>& l);
    void setRightHomePosture(std::vector<float>& r);
    void setLeftHomePosture(std::vector<float>& l);
    // velocities
    void setRightVelocities(std::vector<float>& r);
    void setLeftVelocities(std::vector<float>& l);
    //Forces
    void setRightForces(std::vector<float>& r);
    void setLeftForces(std::vector<float>& l);
    // limits
    void setRightMinLimits(std::vector<float>& min_rl);
    void setRightMaxLimits(std::vector<float>& max_rl);
    void setLeftMinLimits(std::vector<float>& min_ll);
    void setLeftMaxLimits(std::vector<float>& max_ll);
    // matrices
    void setMatRight(Matrix4f& m);
    void setMatLeft(Matrix4f& m);
    void setMatRightHand(Matrix4f& m);
    void setMatLeftHand(Matrix4f& m);
    // humanoid parts
//#if HEAD==1
  //  void setHead(humanoid_part& head);
//#endif
//#if NECK==1
  //  void setNeck(humanoid_part& neck);
//#endif
//#if PELVIS==1
  //  void setPelvis(humanoid_part& pelvis);
//#endif
//#if RIGHT_UPPER_LEG==1
  //  void setRight_Upper_leg(humanoid_part& right_upper_leg);
//#endif
//#if RIGHT_LOWER_LEG==1
  //  void setRight_Lower_leg(humanoid_part& right_lower_leg);
//#endif
//#if RIGHT_FOOT==1
  //  void setRight_foot(humanoid_part& right_foot);
//#endif
//#if LEFT_UPPER_LEG==1
  //  void setLeft_Upper_leg(humanoid_part& left_upper_leg);
//#endif
//#if LEFT_LOWER_LEG==1
  //  void setLeft_Lower_leg(humanoid_part& left_lower_leg);
//#endif
//#if LEFT_FOOT==1
  //  void setLeft_foot(humanoid_part& left_foot);
//#endif


    // *** getters **** //
    string getName();
    pos getPos();
    orient getOr();
    dim getSize();
    arm getArm();
#if HAND==0
    human_hand getHumanHand();
#elif HAND==1
    barrett_hand getBarrettHand();
    void getRK(std::vector<float>& rk);
    void getJK(std::vector<float>& jk);
#endif

    //DHparams getDH_rightArm();
    //DHparams getDH_leftArm();
    //void getDH_rightHand(std::vector<DHparams>& p);
    //void getDH_leftHand(std::vector<DHparams>& p);

    // postures
    void getRightPosture(std::vector<float>& p);
    void getRightArmPosture(std::vector<float>& p);
    void getRightHandPosture(std::vector<float>& p);
    void getLeftPosture(std::vector<float>& p);
    void getLeftArmPosture(std::vector<float>& p);
    void getLeftHandPosture(std::vector<float>& p);
    void getRightHomePosture(std::vector<float>& p);
    void getLeftHomePosture(std::vector<float>& p);
    // velocities
    void getRightVelocities(std::vector<float>& p);
    void getRightArmVelocities(std::vector<float>& p);
    void getRightHandVelocities(std::vector<float>& p);
    void getLeftVelocities(std::vector<float>& p);
    void getLeftArmVelocities(std::vector<float>& p);
    void getLeftHandVelocities(std::vector<float>& p);
    // forces
    void getRightForces(std::vector<float>& p);
    void getRightArmForces(std::vector<float>& p);
    void getRightHandForces(std::vector<float>& p);
    void getLeftForces(std::vector<float>& p);
    void getLeftArmForces(std::vector<float>& p);
    void getLeftHandForces(std::vector<float>& p);
    // limits
    void getRightMinLimits(std::vector<float>& p);
    void getRightMaxLimits(std::vector<float>& p);
    void getLeftMinLimits(std::vector<float>& p);
    void getLeftMaxLimits(std::vector<float>& p);
    // matrices
    void getMatRight(Matrix4f& m);
    void getMatLeft(Matrix4f& m);
    void getMatRightHand(Matrix4f& m);
    void getMatLeftHand(Matrix4f& m);


    // get points on right arm
    void getRightShoulderPos(std::vector<float>& pos);
    float getRightShoulderNorm();
    void getRightShoulderOr(Matrix3f& orr);

    void getRightElbowPos(std::vector<float>& pos);
    float getRightElbowNorm();
    void getRightElbowOr(Matrix3f& orr);

    void getRightWristPos(std::vector<float>& pos);
    float getRightWristNorm();
    void getRightWristOr(Matrix3f& orr);

    void getRightHandPos(std::vector<float>& pos);
    float getRightHandNorm();
    void getRightHandOr(Matrix3f& orr);

    // get points on the left arm
    void getLeftShoulderPos(std::vector<float>& pos);
    float getLeftShoulderNorm();
    void getLeftShoulderOr(Matrix3f& orr);

    void getLeftElbowPos(std::vector<float>& pos);
    float getLeftElbowNorm();
    void getLeftElbowOr(Matrix3f& orr);

    void getLeftWristPos(std::vector<float>& pos);
    float getLeftWristNorm();
    void getLeftWristOr(Matrix3f& orr);

    void getLeftHandPos(std::vector<float>& pos);
    float getLeftHandNorm();
    void getLeftHandOr(Matrix3f& orr);

    //info
    string getInfoLine();

//#if HEAD==1
  //  humanoid_part getHead();
//#endif
//#if NECK==1
  //  humanoid_part getNeck();
//#endif
//#if PELVIS==1
  //  humanoid_part getPelvis();
//#endif
//#if RIGHT_UPPER_LEG==1
  //  humanoid_part getRight_Upper_leg();
//#endif
//#if RIGHT_LOWER_LEG==1
  //  humanoid_part getRight_Lower_leg();
//#endif
//#if RIGHT_FOOT==1
  //  humanoid_part getRight_foot();
//#endif
//#if LEFT_UPPER_LEG==1
  //  humanoid_part getLeft_Upper_leg();
//#endif
//#if LEFT_LOWER_LEG==1
  //  humanoid_part getLeft_Lower_leg();
//#endif
//#if LEFT_FOOT==1
  //  humanoid_part getLeft_foot();
//#endif


private:

    string m_name;
    pos m_torso_pos;
    orient m_torso_or;
    dim m_torso_size;
//#if HEAD==1
  //  humanoid_part head;
//#endif
//#if NECK==1
  //  humanoid_part neck;
//#endif
//#if PELVIS==1
  //  humanoid_part pelvis;
//#endif
//#if RIGHT_UPPER_LEG==1
  //  humanoid_part right_upper_leg;
//#endif
//#if RIGHT_LOWER_LEG==1
  //  humanoid_part right_lower_leg;
//#endif
//#if RIGHT_FOOT==1
  //  humanoid_part right_foot;
//#endif
//#if LEFT_UPPER_LEG==1
  //  humanoid_part left_upper_leg;
//#endif
//#if LEFT_LOWER_LEG==1
  //  humanoid_part left_lower_leg;
//#endif
//#if LEFT_FOOT==1
  //  humanoid_part left_foot;
//#endif
    arm m_arm_specs;
#if HAND==0
    human_hand m_human_hand_specs;
#elif HAND==1
    barrett_hand m_barrett_hand_specs;
    std::vector<float> rk;
    std::vector<float> jk;
#endif

    DHparams m_DH_rightArm;
    DHparams m_DH_leftArm;
    std::vector<DHparams> m_DH_rightHand;
    std::vector< std::vector<float> > right_fing_pos;
    std::vector<DHparams> m_DH_leftHand;
    std::vector< std::vector<float> > left_fing_pos;

    // as default the reference frame of the right arm is the world frame, but
    // any raference frame can be set as reference.
    // the trasational part is in [mm]
    Matrix4f mat_right; // transformation matrix from the world (or any reference frame for the humanoid) and the reference frame of the right arm
    Matrix4f mat_left; // transformation matrix from the world (or any reference frame for the humanoid) and the reference frame of the left arm
    Matrix4f mat_r_hand; // trabsformation matrix from the last joint of the right arm and the palm of the right hand
    Matrix4f mat_l_hand; // trabsformation matrix from the last joint of the left arm and the palm of the left hand

    // functions to generate the D-H parameters
    void computeRightArmDHparams();
    void computeLeftArmDHparams();
    void computeRightHandDHparams();
    void computeLeftHandDHparams();

    // joints [rad]: 7 joints + 4 joints for each arm (total: 22 joints)
    std::vector<float> rightPosture; // right arm+hand current posture
    std::vector<float> leftPosture; // left arm+hand current posture
    std::vector<float> rightHomePosture; // right arm+hand home posture
    std::vector<float> leftHomePosture; // left arm+hand home posture
    std::vector<float> min_rightLimits; // minimum right limits
    std::vector<float> max_rightLimits; // maximum right limits
    std::vector<float> min_leftLimits; // minimum left limits
    std::vector<float> max_leftLimits; // maximum left limits

    // joints velocities
    std::vector<float> rightVelocities;
    std::vector<float> leftVelocities;

    // joints forces
    std::vector<float> rightForces;
    std::vector<float> leftForces;

    // positions on the right arm
    std::vector<float> rightShoulderPos;
    std::vector<float> rightElbowPos;
    std::vector<float> rightWristPos;
    std::vector<float> rightHandPos;

    // orientations on the right arm
    Matrix3f rightShoulderOr;
    Matrix3f rightElbowOr;
    Matrix3f rightWristOr;
    Matrix3f rightHandOr;

    // positions on the right hand
    MatrixXf rightFingers;

    // positions on the left arm
    std::vector<float> leftShoulderPos;
    std::vector<float> leftElbowPos;
    std::vector<float> leftWristPos;
    std::vector<float> leftHandPos;

    // orientations on the left arm
    Matrix3f leftShoulderOr;
    Matrix3f leftElbowOr;
    Matrix3f leftWristOr;
    Matrix3f leftHandOr;

    //positions on the left hand
    MatrixXf leftFingers;


    // direct kinematics of the arm
    void directKinematicsSingleArm(int arm);
    void directKinematicsDualArm();
    // direct kinematics of the hand
    void directKinematicsFinger(DHparams& p,Matrix4f& T_ext,Matrix4f& T_H_0_pos,int k,MatrixXf& Fingers);

    // transformation matrix
    void transfMatrix(float alpha, float a, float d, float theta, Matrix4f& T);

};

}// namespace HUMotion

#endif // HUMANOID_H
