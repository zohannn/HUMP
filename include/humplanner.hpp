#ifndef HUMPLANNER_H
#define HUMPLANNER_H

#include "HUMLconfig.hpp"
#include "amplinterface.hpp"
#include "object.hpp"

using namespace std;

namespace HUMotion{

typedef boost::shared_ptr<Object> objectPtr;/**< shared pointer to an object in the scenario */

//! The Object class
/**
 * @brief This class defines the concept of the Human-like Motion planner
 */
class HUMPlanner
{
public:
    static int hand_fingers; /**< number of fingers per hand */
    static int joints_arm; /**< number of joints per arm */
    static int joints_hand; /**< number of joints per hand */
    static int n_phalange; /**< number of phalanges per finger */


    /**
     * @brief HUMPlanner, a constructor
     * @param name
     */
    explicit HUMPlanner(string name);


    /**
     * @brief HUMPlanner, a copy constructor
     * @param hp
     */
    HUMPlanner(const HUMPlanner& hp);

    /**
     * @brief ~HUMPlanner, a destructor
     */
    ~HUMPlanner();

    /**
     * @brief setName
     * @param name
     */
    void setName(string name);

    /**
     * @brief getName
     * @return
     */
    string getName();

    /**
     * @brief addObstacle
     * @param obs
     */
    void addObstacle(objectPtr obs);

    /**
     * @brief setObstacle
     * @param obs
     * @param pos
     * @return
     */
    bool setObstacle(objectPtr obs, int pos);

    /**
     * @brief getObstacles
     * @param obs
     * @return
     */
    bool getObstacles(std::vector<objectPtr>& obs);

    /**
     * @brief getObstacle
     * @param pos
     * @return
     */
    objectPtr getObstacle(int pos);

    /**
     * @brief getObstacle
     * @param name
     * @return
     */
    objectPtr getObstacle(std::string name);

    /**
     * @brief setObjTarget
     * @param obj
     */
    void setObjTarget(objectPtr obj);

    /**
     * @brief getObjTarget
     * @return
     */
    objectPtr getObjTarget();

    /**
     * @brief clearScenario
     */
    void clearScenario();

    /**
     * @brief setMatRightArm
     * @param m
     */
    void setMatRightArm(Matrix4f &m);

    /**
     * @brief getMatRightArm
     * @param m
     */
    void getMatRightArm(Matrix4f &m);

    /**
     * @brief setMatRightHand
     * @param m
     */
    void setMatRightHand(Matrix4f &m);

    /**
     * @brief getMatRightHand
     * @param m
     */
    void getMatRightHand(Matrix4f &m);

    /**
     * @brief setMatLeftArm
     * @param m
     */
    void setMatLeftArm(Matrix4f &m);

    /**
     * @brief getMatLeftArm
     * @param m
     */
    void getMatLeftArm(Matrix4f &m);

    /**
     * @brief setMatLeftHand
     * @param m
     */
    void setMatLeftHand(Matrix4f &m);

    /**
     * @brief getMatLeftHand
     * @param m
     */
    void getMatLeftHand(Matrix4f &m);

    /**
     * @brief setRightMinLimits
     * @param min_rl
     */
    void setRightMinLimits(vector<double> &min_rl);

    /**
     * @brief getRightMinLimits
     * @param min_rl
     */
    void getRightMinLimits(vector<double> &min_rl);

    /**
     * @brief setRightMaxLimits
     * @param max_rl
     */
    void setRightMaxLimits(vector<double> &max_rl);

    /**
     * @brief getRightMaxLimits
     * @param max_rl
     */
    void getRightMaxLimits(vector<double> &max_rl);

    /**
     * @brief setLeftMinLimits
     * @param min_rl
     */
    void setLeftMinLimits(vector<double> &min_ll);

    /**
     * @brief getLeftMinLimits
     * @param min_ll
     */
    void getLeftMinLimits(vector<double> &min_ll);

    /**
     * @brief setLeftMaxLimits
     * @param max_ll
     */
    void setLeftMaxLimits(vector<double> &max_ll);

    /**
     * @brief getLeftMaxLimits
     * @param max_ll
     */
    void getLeftMaxLimits(vector<double> &max_ll);

    /**
     * @brief setTorsoSize
     * @param tsize
     */
    void setTorsoSize(vector<double>& tsize);

    /**
     * @brief getTorsoSize
     * @param tsize
     */
    void getTorsoSize(vector<double>& tsize);

    /**
     * @brief setDH_rightArm
     * @param p
     */
    void setDH_rightArm(DHparameters& p);

    /**
     * @brief getDH_rightArm
     */
    DHparameters getDH_rightArm();

    /**
     * @brief setDH_leftArm
     * @param p
     */
    void setDH_leftArm(DHparameters& p);

    /**
     * @brief getDH_leftArm
     */
    DHparameters getDH_leftArm();

    /**
     * @brief setBarrettHand
     * @param bhand
     */
    void setBarrettHand(BarrettHand& bhand);

    /**
     * @brief getBarrettHand
     * @return
     */
    BarrettHand getBarrettHand();

    /**
     * @brief setHumanHand
     * @param hhand
     */
    void setHumanHand(HumanHand& hhand);

    /**
     * @brief getHumanHand
     * @return
     */
    HumanHand getHumanHand();



    /**
     * @brief This method solves the final posture selection problem of single arm reach-to-grasp movements
     * @return
     */
    bool singleArmFinalPostureReachToGrasp();



private:

    string name;/**< name of the planner */
    // scenario info    this->planner_id=planner_id;
    vector<objectPtr> obstacles; /**< obstacles in the scenario */
    objectPtr obj_tar; /**< object that has the target of the movement (reach-to-grasp, transport, engage, disengage) */
    // humanoid info
    Matrix4f matWorldToRightArm; /**< transformation matrix from the fixed world frame and the reference frame of the right arm (positions are in [mm]) */
    Matrix4f matRightHand;/**< trabsformation matrix from the last joint of the right arm and the palm of the right hand (positions are in [mm]) */
    std::vector<double> minRightLimits; /**< minimum right limits */
    std::vector<double> maxRightLimits; /**< maximum right limits */
    Matrix4f matWorldToLeftArm; /**< transformation matrix from the fixed world frame and the reference frame of the left arm (positions are in [mm]) */
    Matrix4f matLeftHand; /**< trabsformation matrix from the last joint of the left arm and the palm of the left hand (positions are in [mm]) */
    std::vector<double> minLeftLimits; /**< minimum left limits */
    std::vector<double> maxLeftLimits; /**< maximum left limits */
    std::vector<double> torso_size; /**< size of the torso: xsize, ysize, zsize */
    DHparameters DH_rightArm; /**< current D-H parameters of the right arm */
    DHparameters DH_leftArm; /**< current D-H parameters of the left arm */
    BarrettHand bhand; /**< parameters of the barrett hand */
    HumanHand hhand; /**< parameters of the human hand */

    /**
     * @brief This method writes down the dimensions of the body ofthe humanoid
     * @param h_xsize
     * @param h_ysize
     * @param stream
     */
    void writeBodyDim(double h_xsize,double h_ysize,std::ofstream& stream);

    /**
     * @brief This method writes down the D-H parameters of the arms of the humanoid robot
     * @param dh
     * @param stream
     * @param k
     */
    void writeArmDHParams(DHparameters dh, std::ofstream& stream, int k);

    /**
     * @brief This method writes down the distance between the object and the hand
     * @param stream
     * @param dHO
     */
    void write_dHO(std::ofstream& stream, float dHO);

    /**
     * @brief This method writes down the joint limits of the arm
     * @param stream
     * @param minArmLimits
     * @param maxArmLimits
     */
    void writeArmLimits(std::ofstream& stream, std::vector<double>& minArmLimits,std::vector<double>& maxArmLimits);

    /**
     * @brief This method writes down the initial posture of the arm
     * @param stream
     * @param initArmPosture
     */
    void writeArmInitPose(std::ofstream& stream,std::vector<double>& initArmPosture);

    /**
     * @brief This method writes down the final posture of the fingers
     * @param stream
     * @param finalHand
     */
    void writeFingerFinalPose(std::ofstream& stream,std::vector<double>& finalHand);

    /**
     * @brief This method writes down the lambda of the objective function
     * @param stream
     * @param lambda
     */
    void writeLambda(std::ofstream& stream,std::vector<double>& lambda);


    /**
     * @brief writeHumanHandParams
     * This method writes down the parameter of the human hand
     * @param hhand
     * @param stream
     * @param k
     */
    void writeHumanHandParams(HumanHand& hhand, std::ofstream& stream, int k);

    /**
     * @brief This method writes down the declaration of the parameters of the human hand
     * @param stream
     */
    void writeHumanHandParamsMod(std::ofstream& stream);

    /**
     * @brief This method writes down the direct kinematics of the human hand
     * @param stream
     * @param tolsHand
     * @param final
     * @param transport
     */
    void writeHumanHandDirKin(std::ofstream& stream,MatrixXf& tolsHand, bool final, bool transport);

    /**
     * @brief writeBarrettHandParams
     * Thi method writes down the parameters of the Barrett Hand
     * @param bhand
     * @param stream
     */
    void writeBarrettHandParams(BarrettHand& bhand, std::ofstream& stream);

    /**
     * @brief This method writes down the declaration of the parameters of the Barrett hand
     * @param stream
     */
    void writeBarrettHandParamsMod(std::ofstream& stream);

    /**
     * @brief This method writes down the direct kinematics of the Barrett hand
     * @param stream
     * @param tolsHand
     * @param final
     * @param transport
     */
    void writeBarrettHandDirKin(std::ofstream& stream, MatrixXf& tolsHand, bool final, bool transport);

    /**
     * @brief writeInfoTarget
     * This method writes down the info of the target
     * @param stream
     * @param tar: tar(0)=x, tar(1)=y, tar(2)=z, tar(3)=roll, tar(4)=pitch, tar(5)=yaw
     */
    void writeInfoTarget(ofstream &stream, std::vector<double> tar);

    /**
     * @brief writeInfoObstacles
     * @param stream
     * @param objs
     */
    void writeInfoObstacles(ofstream &stream, std::vector<objectPtr> &obstacles);

    /**
     * @brief writeInfoObjectTarget
     * @param stream
     * @param obj
     */
    void writeInfoObjectTarget(ofstream &stream, objectPtr obj);

    /**
     * @brief writePI
     * @param stream
     */
    void writePI(std::ofstream& stream);

    /**
     * @brief writeBodyDimMod
     * @param stream
     */
    void writeBodyDimMod(ofstream &stream);

    /**
     * @brief writeArmDHParamsMod
     * @param stream
     */
    void writeArmDHParamsMod(ofstream &stream);

    /**
     * @brief write_dHOMod
     * @param stream
     */
    void write_dHOMod(ofstream &stream);
    /**
     * @brief writeInfoObjectsMod
     * @param stream
     */
    void writeInfoObjectsMod(ofstream &stream);

    /**
     * @brief writeRotMatObsts
     * @param stream
     */
    void writeRotMatObsts(ofstream &stream);

    /**
     * @brief writeArmDirKin
     * @param stream
     * @param matWorldToArm
     * @param matHand
     * @param tolsArm
     * @param final
     */
    void writeArmDirKin(ofstream &stream, Matrix4f &matWorldToArm, Matrix4f &matHand, std::vector<double>& tolsArm, bool final);

    /**
     * @brief writeObjective
     * @param stream
     * @param final
     */
    void writeObjective(ofstream &stream, bool final);

    /**
     * @brief writeBodyConstraints
     * @param stream
     * @param final
     */
    void writeBodyConstraints(ofstream &stream, bool final);

    /**
     * @brief RPY_matrix
     * @param rpy
     * @param Rot
     */
    void RPY_matrix(std::vector<double>rpy, Matrix3f &Rot);

    /**
     * @brief Trans_matrix
     * @param xyz
     * @param rpy
     * @param Trans
     */
    void Trans_matrix(std::vector<double>xyz,std::vector<double>rpy,Matrix4f& Trans);

    /**
     * @brief getRotAxis
     * @param xt
     * @param id
     * @param rpy
     */
    void getRotAxis(vector<double>& xt, int id,std::vector<double>rpy);

    /**
     * @brief writeFilesFinalPosture
     * @param mov_type
     * @param dHO
     * @param griptype
     * @param initArmPosture
     * @param finalHand
     * @param initialGuess
     * @param objs
     * @param tar: tar(0)=x, tar(1)=y, tar(2)=z, tar(3)=roll, tar(4)=pitch, tar(5)=yaw
     * @param obj
     * @param tolsArm
     * @param tolsHand
     * @param tolsObstacles
     * @param tolTarPos
     * @param tolTarOr
     * @param lambda
     * @param obstacle_avoidance
     * @param arm_code
     * @param hand_code
     * @return
     */
    bool writeFilesFinalPosture(int mov_type, double dHO, int griptype,
                                std::vector<double> initArmPosture, std::vector<double> finalHand,
                                std::vector<double> initialGuess, std::vector<objectPtr> objs,
                                std::vector<double> tar, objectPtr obj,
                                std::vector<double> tolsArm, MatrixXf tolsHand, MatrixXf tolsObstacles,
                                double tolTarPos, double tolTarOr, std::vector<double> lambda, bool obstacle_avoidance,
                                int arm_code,int hand_code);




};

} // namespace HUMotion

#endif // HUMPLANNER_H
