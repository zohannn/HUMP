#ifndef HUMANOID_H
#define HUMANOID_H


#include "HUMLconfig.hpp"

namespace HUMotion{

//! The Humanoid class
/**
 * @brief This class defines the concept of a humanoid robot
 */
class Humanoid
{

public:

#if HAND==0
    /**
     * @brief Humanoid, a constructor
     * @param name
     * @param ppos
     * @param oor
     * @param ssize
     * @param aspecs
     * @param hspecs
     */
    Humanoid(string name, pos ppos, orient oor, dim ssize, arm aspecs, human_hand hspecs);

    /**
     * @brief Humanoid, a constructor
     * @param name
     * @param ppos
     * @param oor
     * @param ssize
     * @param aspecs
     * @param hspecs
     * @param r
     * @param l
     */
    Humanoid(string name, pos ppos, orient oor, dim ssize, arm aspecs, human_hand hspecs,
             std::vector<float>& r, std::vector<float>& l);

    /**
     * @brief Humanoid, a constructor
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
    Humanoid(string name, pos ppos, orient oor, dim ssize, arm aspecs, human_hand hspecs,
             std::vector<float>& r, std::vector<float>& l,
             std::vector<float>& min_rl, std::vector<float>& max_rl,
             std::vector<float>& min_ll, std::vector<float>& max_ll);
#elif HAND==1
    /**
     * @brief Humanoid, a constructor
     * @param name
     * @param ppos
     * @param oor
     * @param ssize
     * @param aspecs
     * @param hspecs
     */
    Humanoid(string name, pos ppos, orient oor, dim ssize, arm aspecs, barrett_hand hspecs);

    /**
     * @brief Humanoid, a constructor
     * @param name
     * @param ppos
     * @param oor
     * @param ssize
     * @param aspecs
     * @param hspecs
     * @param r
     * @param l
     */
    Humanoid(string name, pos ppos, orient oor, dim ssize, arm aspecs, barrett_hand hspecs,
             std::vector<float>& r, std::vector<float>& l);

    /**
     * @brief Humanoid, a constructor
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
    Humanoid(string name, pos ppos, orient oor, dim ssize, arm aspecs, barrett_hand hspecs,
             std::vector<float>& r, std::vector<float>& l,
             std::vector<float>& min_rl, std::vector<float>& max_rl,
             std::vector<float>& min_ll, std::vector<float>& max_ll);
#endif

    /**
     * @brief Humanoid, a copy constructor
     * @param hh
     */
    Humanoid(const Humanoid& hh);

    /**
     * @brief ~Humanoid, a destructor
     */
    ~Humanoid();

    /**
     * @brief This method sets the name of the humanoid
     * @param name
     */
    void setName(string& name);

    /**
     * @brief This method sets the position of the humanoid as the position of its torso
     * @param ppos
     */
    void setPos(pos& ppos);

    /**
     * @brief This method sets the orientation of the humanoid as the orientation of its torso
     * @param oor
     */
    void setOr(orient& oor);

    /**
     * @brief This method sets the size of the humanoid as the size of its torso
     * @param ssize
     */
    void setSize(dim& ssize);

    /**
     * @brief This method sets the specifications of the arms
     * @param specs
     */
    void setArm(arm& specs);
#if HAND==0

    /**
     * @brief This method sets the specifications of the hand
     * @param specs
     */
    void setHumanHand(human_hand& specs);
#elif HAND==1

    /**
     * @brief This method sets the specifications of the hand
     * @param specs
     */
    void setBarrettHand(barrett_hand& specs);
#endif

    /**
     * @brief This method sets the posture of the right arm
     * @param r
     */
    void setRightPosture(std::vector<float>& r);

    /**
     * @brief This method sets the posture of the left arm
     * @param l
     */
    void setLeftPosture(std::vector<float>& l);

    /**
     * @brief This method sets the home posture of the right arm
     * @param r
     */
    void setRightHomePosture(std::vector<float>& r);

    /**
     * @brief This method sets the home posture of the left arm
     * @param l
     */
    void setLeftHomePosture(std::vector<float>& l);

    /**
     * @brief This method sets the joint velocities of the right arm
     * @param r
     */
    void setRightVelocities(std::vector<float>& r);

    /**
     * @brief This method sets the joint velocities of the left arm
     * @param l
     */
    void setLeftVelocities(std::vector<float>& l);

    /**
     * @brief This method sets the joint forces of the right arm
     * @param r
     */
    void setRightForces(std::vector<float>& r);

    /**
     * @brief This method sets the joint forces of the left arm
     * @param l
     */
    void setLeftForces(std::vector<float>& l);

    /**
     * @brief This method sets the minimum joint limits of the right arm
     * @param min_rl
     */
    void setRightMinLimits(std::vector<float>& min_rl);

    /**
     * @brief This method sets the maximum joint limits of the right arm
     * @param max_rl
     */
    void setRightMaxLimits(std::vector<float>& max_rl);

    /**
     * @brief This method sets the minimum joint limits of the left arm
     * @param min_ll
     */
    void setLeftMinLimits(std::vector<float>& min_ll);

    /**
     * @brief This method sets the maximum joint limits of the left arm
     * @param max_ll
     */
    void setLeftMaxLimits(std::vector<float>& max_ll);

    /**
     * @brief This method sets the attribute mat_right
     * @param m
     */
    void setMatRight(Matrix4f& m);

    /**
     * @brief This method sets the attribute mat_left
     * @param m
     */
    void setMatLeft(Matrix4f& m);

    /**
     * @brief This method sets the attribute mat_r_hand
     * @param m
     */
    void setMatRightHand(Matrix4f& m);

    /**
     * @brief This method sets the attribute mat_l_hand
     * @param m
     */
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

    /**
     * @brief This method gets the name of the humanoid
     * @return
     */
    string getName();

    /**
     * @brief This method gets the position of the humanoid
     * @return
     */
    pos getPos();

    /**
     * @brief This method gets the orientation of the humanoid
     * @return
     */
    orient getOr();

    /**
     * @brief This method gets the size of the humanoid
     * @return
     */
    dim getSize();

    /**
     * @brief This method gets the arm specifications of the humanoid
     * @return
     */
    arm getArm();
#if HAND==0

    /**
     * @brief This method gets hand specifications of the humanoid
     * @return
     */
    human_hand getHumanHand();
#elif HAND==1
    /**
     * @brief This method gets the hand specifications of the humanoid
     * @return
     */
    barrett_hand getBarrettHand();

    /**
     * @brief This method gets the r parameters of the hand of the humanoid
     * @param rk
     */
    void getRK(std::vector<float>& rk);

    /**
     * @brief This method gets the j parameters of the hand of the humanoid
     * @param jk
     */
    void getJK(std::vector<float>& jk);
#endif

    /**
     * @brief This method gets the current posture of the right arm+hand
     * @param p
     */
    void getRightPosture(std::vector<float>& p);

    /**
     * @brief This method gets the current posture of the right arm
     * @param p
     */
    void getRightArmPosture(std::vector<float>& p);

    /**
     * @brief This method gets the current posture of the right hand
     * @param p
     */
    void getRightHandPosture(std::vector<float>& p);

    /**
     * @brief This method gets the current posture of the left arm+hand
     * @param p
     */
    void getLeftPosture(std::vector<float>& p);

    /**
     * @brief This method gets the current posture of the left arm
     * @param p
     */
    void getLeftArmPosture(std::vector<float>& p);

    /**
     * @brief This method gets the current posture of the left hand
     * @param p
     */
    void getLeftHandPosture(std::vector<float>& p);

    /**
     * @brief This method gets the home posture of the right arm+hand
     * @param p
     */
    void getRightHomePosture(std::vector<float>& p);

    /**
     * @brief This method gets the home posture of the left arm+hand
     * @param p
     */
    void getLeftHomePosture(std::vector<float>& p);

    /**
     * @brief This method gets the joint velocities of the right arm+hand
     * @param p
     */
    void getRightVelocities(std::vector<float>& p);

    /**
     * @brief This method gets the joint velocities of the right arm
     * @param p
     */
    void getRightArmVelocities(std::vector<float>& p);

    /**
     * @brief This method gets the joint velocities of the right hand
     * @param p
     */
    void getRightHandVelocities(std::vector<float>& p);

    /**
     * @brief This method gets the joint velocities of the left arm+hand
     * @param p
     */
    void getLeftVelocities(std::vector<float>& p);

    /**
     * @brief This method gets the joint velocities of the left arm
     * @param p
     */
    void getLeftArmVelocities(std::vector<float>& p);

    /**
     * @brief This method gets the joint velocities of the left hand
     * @param p
     */
    void getLeftHandVelocities(std::vector<float>& p);

    /**
     * @brief This method gets the joint forces of the right arm+hand
     * @param p
     */
    void getRightForces(std::vector<float>& p);

    /**
     * @brief This method gets the joint forces of the right arm+hand
     * @param p
     */
    void getRightArmForces(std::vector<float>& p);

    /**
     * @brief This method gets the joint forces of the right hand
     * @param p
     */
    void getRightHandForces(std::vector<float>& p);

    /**
     * @brief This method gets the joint forces of the left arm+hand
     * @param p
     */
    void getLeftForces(std::vector<float>& p);

    /**
     * @brief This method gets the joint forces of the left hand
     * @param p
     */
    void getLeftArmForces(std::vector<float>& p);

    /**
     * @brief This method gets the joint forces of the left hand
     * @param p
     */
    void getLeftHandForces(std::vector<float>& p);

    /**
     * @brief This method gets the minimum joint limits of the right arm+hand
     * @param p
     */
    void getRightMinLimits(std::vector<float>& p);

    /**
     * @brief This method gets the maximum joint limits of the right arm+hand
     * @param p
     */
    void getRightMaxLimits(std::vector<float>& p);

    /**
     * @brief This method gets the minimum joint limits of the left arm+hand
     * @param p
     */
    void getLeftMinLimits(std::vector<float>& p);

    /**
     * @brief This method gets the maximum joint limits of the left arm+hand
     * @param p
     */
    void getLeftMaxLimits(std::vector<float>& p);

    /**
     * @brief This method gets the attribute mat_right
     * @param m
     */
    void getMatRight(Matrix4f& m);

    /**
     * @brief This method gets the attribute mat_left
     * @param m
     */
    void getMatLeft(Matrix4f& m);

    /**
     * @brief This method gets the attribute mat_r_hand
     * @param m
     */
    void getMatRightHand(Matrix4f& m);

    /**
     * @brief This method gets the attribute mat_l_hand
     * @param m
     */
    void getMatLeftHand(Matrix4f& m);


    /**
     * @brief This method gets the position of the right shoulder
     * @param pos
     */
    void getRightShoulderPos(std::vector<float>& pos);

    /**
     * @brief This method gets the norm of the vector pointing to the right shoulder
     * @return
     */
    float getRightShoulderNorm();

    /**
     * @brief This method gets the orientation of the right shoulder
     * @param orr
     */
    void getRightShoulderOr(Matrix3f& orr);

    /**
     * @brief This method gets the position of the right elbow
     * @param pos
     */
    void getRightElbowPos(std::vector<float>& pos);

    /**
     * @brief This method gets the norm of the vector pointing to the right elbow
     * @return
     */
    float getRightElbowNorm();

    /**
     * @brief This method gets the orientation of the right elbow
     * @param orr
     */
    void getRightElbowOr(Matrix3f& orr);

    /**
     * @brief This method gets the position of the right wrist
     * @param pos
     */
    void getRightWristPos(std::vector<float>& pos);

    /**
     * @brief This method gets the norm of the vector pointing to the right wrist
     * @return
     */
    float getRightWristNorm();

    /**
     * @brief This method gets the orientation of the right wrist
     * @param orr
     */
    void getRightWristOr(Matrix3f& orr);

    /**
     * @brief This method gets the position of the right hand
     * @param pos
     */
    void getRightHandPos(std::vector<float>& pos);

    /**
     * @brief This method gets the norm of the vector pointing to the right hand
     * @return
     */
    float getRightHandNorm();

    /**
     * @brief This method gets the orientation of the right hand
     * @param orr
     */
    void getRightHandOr(Matrix3f& orr);

    /**
     * @brief This method gets the position of the left shoulder
     * @param pos
     */
    void getLeftShoulderPos(std::vector<float>& pos);

    /**
     * @brief This method gets the norm of the vector pointing to the left shoulder
     * @return
     */
    float getLeftShoulderNorm();

    /**
     * @brief This method gets the orientation of the left shoulder
     * @param orr
     */
    void getLeftShoulderOr(Matrix3f& orr);

    /**
     * @brief This method gets the position of the left elbow
     * @param pos
     */
    void getLeftElbowPos(std::vector<float>& pos);

    /**
     * @brief This method gets the norm of the vector pointing to the left elbow
     * @return
     */
    float getLeftElbowNorm();

    /**
     * @brief This method gets the orientation of the left elbow
     * @param orr
     */
    void getLeftElbowOr(Matrix3f& orr);

    /**
     * @brief This method gets the position of the left wrist
     * @param pos
     */
    void getLeftWristPos(std::vector<float>& pos);

    /**
     * @brief This method gets the norm pointing to the left wrist
     * @return
     */
    float getLeftWristNorm();
    /**
     * @brief This method gets the orientation of the left wrist
     * @param orr
     */
    void getLeftWristOr(Matrix3f& orr);

    /**
     * @brief This method gets the position of the left hand
     * @param pos
     */
    void getLeftHandPos(std::vector<float>& pos);

    /**
     * @brief This method gets the norm of the vector pointing to the left hand
     * @return
     */
    float getLeftHandNorm();

    /**
     * @brief This method gets the orientation of the left hand
     * @param orr
     */
    void getLeftHandOr(Matrix3f& orr);

    /**
     * @brief This method gets information about the humanoid
     * @return
     */
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

    string m_name; /**< name of the humanoid */
    pos m_torso_pos;/**< position of the torso */
    orient m_torso_or; /**< orientation of the torso */
    dim m_torso_size; /**< size of the torso */
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
    arm m_arm_specs; /**< specifications of the arm */
#if HAND==0
    human_hand m_human_hand_specs; /**< specifications of the hand */
#elif HAND==1
    barrett_hand m_barrett_hand_specs; /**< specifications of the hand */
    std::vector<float> rk; /**< r parameters of the barrett hand */
    std::vector<float> jk; /**< j parameters of the barrett hand */
#endif

    DHparams m_DH_rightArm; /**< current D-H parameters of the right arm */
    DHparams m_DH_leftArm; /**< current D-H parameters of the left arm */
    std::vector<DHparams> m_DH_rightHand; /**< current D-H parameters of the fingers on the right hand */
    std::vector< std::vector<float> > right_fing_pos; /**< current positions of the phalanges of the fingers on the right hand */
    std::vector<DHparams> m_DH_leftHand; /**< current D-H parameters of the fingers on the left hand */
    std::vector< std::vector<float> > left_fing_pos; /**< current positions of the phalanges of the fingers on the left hand */

    Matrix4f mat_right; /**< transformation matrix from the fixed world frame and the reference frame of the right arm (positions are in [mm]) */
    Matrix4f mat_left; /**< transformation matrix from the fixed world frame and the reference frame of the left arm (positions are in [mm]) */
    Matrix4f mat_r_hand; /**< trabsformation matrix from the last joint of the right arm and the palm of the right hand (positions are in [mm]) */
    Matrix4f mat_l_hand; /**< trabsformation matrix from the last joint of the left arm and the palm of the left hand (positions are in [mm]) */

    // joints [rad]: 7 joints + 4 joints for each arm (total: 22 joints)
    std::vector<float> rightPosture; /**< right arm+hand current posture */
    std::vector<float> leftPosture; /**< left arm+hand current posture */
    std::vector<float> rightHomePosture; /**< right arm+hand home posture */
    std::vector<float> leftHomePosture; /**< left arm+hand home posture */
    std::vector<float> min_rightLimits; /**< minimum right limits */
    std::vector<float> max_rightLimits; /**< maximum right limits */
    std::vector<float> min_leftLimits; /**< minimum left limits */
    std::vector<float> max_leftLimits; /**< maximum left limits */

    // joints velocities
    std::vector<float> rightVelocities; /**< right arm+hand current velocities */
    std::vector<float> leftVelocities; /**< left arm+hand current velocities */

    // joints forces
    std::vector<float> rightForces; /**< right arm+hand current forces */
    std::vector<float> leftForces; /**< left arm+hand current forces */

    // positions on the right arm
    std::vector<float> rightShoulderPos; /**< position of the right shoulder */
    std::vector<float> rightElbowPos; /**< position of the right elbow */
    std::vector<float> rightWristPos; /**< position of the right wrist */
    std::vector<float> rightHandPos; /**< position of the right hand */

    // orientations on the right arm
    Matrix3f rightShoulderOr; /**< orientation of the right shoulder */
    Matrix3f rightElbowOr; /**< orientation of the right elbow */
    Matrix3f rightWristOr; /**< orientation of the right wrist */
    Matrix3f rightHandOr; /**< orientation of the right hand */

    // positions on the right hand
    MatrixXf rightFingers; /**< positions of the phalanges of the fingers on the right hand */

    // positions on the left arm
    std::vector<float> leftShoulderPos; /**< position of the left shoulder */
    std::vector<float> leftElbowPos; /**< position of the left elbow */
    std::vector<float> leftWristPos; /**< position of the left wrist */
    std::vector<float> leftHandPos; /**< position of the left hand */

    // orientations on the left arm
    Matrix3f leftShoulderOr; /**< orientation of the left shoulder */
    Matrix3f leftElbowOr; /**< orientation of the left elbow */
    Matrix3f leftWristOr; /**< orientation of the left wrist */
    Matrix3f leftHandOr; /**< orientation of the left hand */

    //positions on the left hand
    MatrixXf leftFingers; /**< positions of the phalanges of the fingers on the left hand */


    /**
     * @brief This method computes the current D-H parameters of the right arm
     */
    void computeRightArmDHparams();

    /**
     * @brief This method computes the current D-H parameters of the left arm
     */
    void computeLeftArmDHparams();

    /**
     * @brief This method computes the current D-H parameters of the right hand
     */
    void computeRightHandDHparams();

    /**
     * @brief This method computes the current D-H parameters of the left hand
     */
    void computeLeftHandDHparams();

    /**
     * @brief This method computes the direct kinematic of the arm
     * @param arm
     */
    void directKinematicsSingleArm(int arm);

    /**
     * @brief This method computes the direct kinematic of both arms
     */
    void directKinematicsDualArm();

    /**
     * @brief This method computes the direct kinematics of the fingers
     * @param p
     * @param T_ext
     * @param T_H_0_pos
     * @param id_fing
     * @param Fingers
     */
    void directKinematicsFinger(DHparams& p,Matrix4f& T_ext,Matrix4f& T_H_0_pos,int id_fing, MatrixXf& Fingers);

    /**
     * @brief This method computes the transformation matrix from the D-H parameters
     * It performes the homogeneus transformation matrix given the D-H parameters: \n
     * - translate by d_i along the z_i axis \n
     * - rotate counterclockwise  by theta around the z_i axis \n
     * - translate by a_(i-1) along the x_(i-1) \n
     * - rotate counterclockwise by alpha_(i-1) around the x_(i-1) axis \n
     * @param alpha
     * @param a
     * @param d
     * @param theta
     * @param T
     */
    void transfMatrix(float alpha, float a, float d, float theta, Matrix4f& T);

};

}// namespace HUMotion

#endif // HUMANOID_H
