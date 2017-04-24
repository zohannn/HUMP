#ifndef HUMPLANNER_H
#define HUMPLANNER_H

#include "HUMPconfig.hpp"
#include "amplinterface.hpp"
#include "IpIpoptApplication.hpp"

using namespace Ipopt;
using namespace std;

namespace HUMotion{

typedef boost::shared_ptr<planning_result> planning_result_ptr; /**< pointer to the results of the planning process*/

//! The Object class
/**
 * @brief This class defines the concept of the Human-like Motion planner
 */
class HUMPlanner
{
public:
    static unsigned hand_fingers; /**< number of fingers per hand */
    static unsigned joints_arm; /**< number of joints per arm */
    static unsigned joints_hand; /**< number of joints per hand */
    static unsigned n_phalange; /**< number of phalanges per finger */


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
     * @brief setObstacleStagiare sviluppatore, informatico presso la sede principale
     * @param obs
     * @param pos
     * @return
     */
    bool setObstacle(objectPtr obs, unsigned pos);

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
    objectPtr getObstacle(unsigned pos);

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
    void setMatRightArm(Matrix4d &m);

    /**
     * @brief getMatRightArm
     * @param m
     */
    void getMatRightArm(Matrix4d &m);

    /**
     * @brief setMatRightHand
     * @param m
     */
    void setMatRightHand(Matrix4d &m);

    /**
     * @brief getMatRightHand
     * @param m
     */
    void getMatRightHand(Matrix4d &m);

    /**
     * @brief setMatLeftArm
     * @param m
     */
    void setMatLeftArm(Matrix4d &m);

    /**
     * @brief getMatLeftArm
     * @param m
     */
    void getMatLeftArm(Matrix4d &m);

    /**
     * @brief setMatLeftHand
     * @param m
     */
    void setMatLeftHand(Matrix4d &m);

    /**
     * @brief getMatLeftHand
     * @param m
     */
    void getMatLeftHand(Matrix4d &m);

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
     * @param min_ll
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
     * @brief setShpos
     * @param shPos
     */
    //void setShpos(std::vector<double>& shPos);

    /**
     * @brief setElpos
     * @param elPos
     */
    //void setElpos(std::vector<double>& elPos);

    /**
     * @brief setWrpos
     * @param wrPos
     */
    //void setWrpos(std::vector<double>& wrPos);

    /**
     * @brief setHapos
     * @param haPos
     */
   // void setHapos(std::vector<double>& haPos);

    /**
     * @brief setHaor
     * @param haOr
     */
    //void setHaor(std::vector<double>& haOr);

    /**
     * @brief getShpos
     * @param shPos
     */
    //void getShpos(std::vector<double>& shPos);

    /**
     * @brief getElpos
     * @param elPos
     */
    //void getElpos(std::vector<double>& elPos);

    /**
     * @brief getWrpos
     * @param wrPos
     */
   // void getWrpos(std::vector<double>& wrPos);

    /**
     * @brief getHapos
     * @param haPos
     */
    //void getHapos(std::vector<double>& haPos);

    /**
     * @brief getHaor
     * @param haOr
     */
    //void getHaor(std::vector<double>& haOr);

    /**
     * @brief plan_pick
     * @param params
     * @param initPosture
     * @return
     */
    planning_result_ptr plan_pick(hump_params& params, std::vector<double> initPosture);

    /**
     * @brief plan_place
     * @param params
     * @param initPosture
     * @return
     */
    planning_result_ptr plan_place(hump_params& params, std::vector<double> initPosture);

    /**
     * @brief plan_move
     * @param params
     * @param initPosture
     * @return
     */
    planning_result_ptr plan_move(hump_params& params, std::vector<double> initPosture);

    /**
     * @brief plan_move
     * @param params
     * @param initPosture
     * @param finalPosture
     * @return
     */
    planning_result_ptr plan_move(hump_params& params, std::vector<double> initPosture, std::vector<double> finalPosture);




private:

    string name;/**< name of the planner */
    // scenario info
    vector<objectPtr> obstacles; /**< obstacles in the scenario */
    // humanoid info
    std::vector<double> shPose; /**< pose of the shoulder of the humanoid: shPose(0)=x, shPose(1)=y, shPose(2)=z, shPose(3)=roll, shPose(4)=pitch, shPose(5)=yaw */
    std::vector<double> elPose; /**< pose of the elbow of the humanoid: shPose(0)=x, shPose(1)=y, shPose(2)=z, shPose(3)=roll, shPose(4)=pitch, shPose(5)=yaw  */
    std::vector<double> wrPose; /**< pose of the wrist of the humanoid: shPose(0)=x, shPose(1)=y, shPose(2)=z, shPose(3)=roll, shPose(4)=pitch, shPose(5)=yaw  */
    std::vector<double> haPose; /**< pose of the hand of the humanoid: shPose(0)=x, shPose(1)=y, shPose(2)=z, shPose(3)=roll, shPose(4)=pitch, shPose(5)=yaw  */
    Matrix4d matWorldToRightArm; /**< transformation matrix from the fixed world frame and the reference frame of the right arm (positions are in [mm]) */
    Matrix4d matRightHand;/**< trabsformation matrix from the last joint of the right arm and the palm of the right hand (positions are in [mm]) */
    std::vector<double> minRightLimits; /**< minimum right limits */
    std::vector<double> maxRightLimits; /**< maximum right limits */
    Matrix4d matWorldToLeftArm; /**< transformation matrix from the fixed world frame and the reference frame of the left arm (positions are in [mm]) */
    Matrix4d matLeftHand; /**< trabsformation matrix from the last joint of the left arm and the palm of the left hand (positions are in [mm]) */
    std::vector<double> minLeftLimits; /**< minimum left limits */
    std::vector<double> maxLeftLimits; /**< maximum left limits */
    std::vector<double> torso_size; /**< size of the torso: xsize, ysize, zsize */
    DHparameters DH_rightArm; /**< current D-H parameters of the right arm */
    DHparameters DH_leftArm; /**< current D-H parameters of the left arm */
    BarrettHand bhand; /**< parameters of the barrett hand */
    HumanHand hhand; /**< parameters of the human hand */

    /**
     * @brief getTimeStep
     * @param tols
     * @param jointTraj
     * @return
     */
    double getTimeStep(hump_params& tols, MatrixXd& jointTraj);

    /**
     * @brief setBoundaryConditions
     * @param params
     * @param steps
     * @param initPosture
     * @param finalPosture
     * @param mod
     */
    void setBoundaryConditions(hump_params& params, int steps, std::vector<double>& initPosture, std::vector<double>& finalPosture, int mod=0);

    /**
     * @brief directTrajectory
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param timestep
     * @param Traj
     * @param mod
     */
    void directTrajectory(int steps, hump_params& tols, std::vector<double>& initPosture, std::vector<double>& finalPosture, double timestep, MatrixXd& Traj, int mod);

    /**
     * @brief directTrajectoryNoBound
     * @param steps
     * @param initPosture
     * @param finalPosture
     * @param Traj
     */
    void directTrajectoryNoBound(int steps,std::vector<double>& initPosture, std::vector<double>& finalPosture, MatrixXd &Traj);

    /**
     * @brief directVelocity
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param timestep
     * @param Vel
     * @param mod
     */
    void directVelocity(int steps,hump_params& tols, std::vector<double>& initPosture, std::vector<double>& finalPosture, double timestep,MatrixXd& Vel, int mod);

    /**
     * @brief directAcceleration
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param timestep
     * @param Acc
     * @param mod
     */
    void directAcceleration(int steps,hump_params& tols, std::vector<double>& initPosture, std::vector<double>& finalPosture, double timestep, MatrixXd& Acc,int mod);

    /**
     * @brief backForthTrajectory
     * @param steps
     * @param initPosture
     * @param bouncePosture
     * @param Traj
     */
    void backForthTrajectory(int steps, std::vector<double>& initPosture, std::vector<double>& bouncePosture, MatrixXd& Traj);

    /**
     * @brief backForthVelocity
     * @param steps
     * @param tols
     * @param initPosture
     * @param bouncePosture
     * @param timestep
     * @param Vel
     */
    void backForthVelocity(int steps,hump_params& tols, std::vector<double>& initPosture, std::vector<double>& bouncePosture, double timestep, MatrixXd& Vel);

    /**
     * @brief backForthAcceleration
     * @param steps
     * @param tols
     * @param initPosture
     * @param bouncePosture
     * @param timestep
     * @param Acc
     */
    void backForthAcceleration(int steps,hump_params& tols, std::vector<double>& initPosture, std::vector<double>& bouncePosture, double timestep, MatrixXd& Acc);

    /**
     * @brief computeMovement
     * @param direct
     * @param back
     * @param tot
     */
    void computeMovement(const MatrixXd& direct, const MatrixXd& back, MatrixXd& tot);

    /**
     * @brief getTrajectory
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param bouncePosture
     * @param traj
     * @param mod
     * @return
     */
    double getTrajectory(int steps,hump_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, std::vector<double> bouncePosture, MatrixXd &traj,int mod);

    /**
     * @brief getTrajectory
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param traj
     * @param mod
     * @return
     */
    double getTrajectory(int steps,hump_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, MatrixXd &traj,int mod);

    /**
     * @brief getVelocity
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param bouncePosture
     * @param traj
     * @param vel
     * @param mod
     * @return
     */
    double getVelocity(int steps,hump_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, std::vector<double> bouncePosture, MatrixXd &traj, MatrixXd &vel,int mod);

    /**
     * @brief getVelocity
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param traj
     * @param vel
     * @param mod
     * @return
     */
    double getVelocity(int steps,hump_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, MatrixXd &traj, MatrixXd &vel, int mod);

    /**
     * @brief getAcceleration
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param bouncePosture
     * @param traj
     * @param vel
     * @param acc
     * @param mod
     * @return
     */
    double getAcceleration(int steps,hump_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, std::vector<double> bouncePosture, MatrixXd &traj, MatrixXd &vel, MatrixXd &acc, int mod);

    /**
     * @brief getAcceleration
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param traj
     * @param vel
     * @param acc
     * @param mod
     * @return
     */
    double getAcceleration(int steps,hump_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, MatrixXd &traj, MatrixXd &vel, MatrixXd &acc,int mod);

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
    void write_dHO(std::ofstream& stream, double dHO);

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
    void writeHumanHandDirKin(std::ofstream& stream,MatrixXd& tolsHand, bool final, bool transport);

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
    void writeBarrettHandDirKin(std::ofstream& stream, MatrixXd& tolsHand, bool final, bool transport);

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
     * @param obstacles
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
     * @param vec
     */
    void writeInfoObjectsMod(ofstream &stream,bool vec);

    /**
     * @brief writeInfoObjectsMod_place
     * @param stream
     * @param vec
     */
    void writeInfoObjectsMod_place(ofstream &stream,bool vec);

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
    void writeArmDirKin(ofstream &stream, Matrix4d &matWorldToArm, Matrix4d &matHand, std::vector<double>& tolsArm, bool final);

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
    void RPY_matrix(std::vector<double>rpy, Matrix3d &Rot);

    /**
     * @brief Trans_matrix
     * @param xyz
     * @param rpy
     * @param Trans
     */
    void Trans_matrix(std::vector<double>xyz,std::vector<double>rpy,Matrix4d& Trans);

    /**
     * @brief getRotAxis
     * @param xt
     * @param id
     * @param rpy
     */
    void getRotAxis(vector<double>& xt, int id,std::vector<double>rpy);

    /**
     * @brief getRand
     * @param min
     * @param max
     * @return
     */
    double getRand(double min, double max);


    //std::string exec(const char* cmd);

    /**
     * @brief amplRead
     * @param datFile
     * @param modFile
     * @param nlFile
     * @return
     */
    bool amplRead(string &datFile, string &modFile, string &nlFile);

    /**
     * @brief optimize
     * @param nlfile
     * @param x
     * @param tol
     * @param acc_tol
     * @return
     */
    bool optimize(string &nlfile, std::vector<Number>& x,double tol, double acc_tol);

    /**
     * @brief getObstaclesSingleArm
     * @param center
     * @param radius
     * @param obsts
     * @param hand_code
     */
    void getObstaclesSingleArm(std::vector<double> center, double radius, std::vector<objectPtr>& obsts, int hand_code);

    /**
     * @brief writeInfoApproachRetreat
     * @param stream
     * @param tar
     * @param approach_retreat
     */
    void writeInfoApproachRetreat(ofstream &stream, std::vector<double> tar, std::vector<double> approach_retreat);

    /**
     * @brief writeInfoApproachRetreat_place
     * @param stream
     * @param tar
     * @param approach
     * @param retreat
     */
    void writeInfoApproachRetreat_place(ofstream &stream, std::vector<double> tar, std::vector<double> approach, std::vector<double> retreat);

    /**
     * @brief singleArmFinalPosture
     * @param mov_type
     * @param pre_post
     * @param params
     * @param initPosture
     * @param finalPosture
     * @return
     */
    bool singleArmFinalPosture(int mov_type,int pre_post,hump_params& params, std::vector<double> initPosture, std::vector<double>& finalPosture);

    /**
     * @brief writeFilesFinalPosture
     * @param params
     * @param mov_type
     * @param initArmPosture
     * @param initialGuess
     * @param obsts
     * @param pre_post
     * @return
     */
    bool writeFilesFinalPosture(hump_params& params,int mov_type, int pre_post,std::vector<double> initArmPosture, std::vector<double> initialGuess,std::vector<objectPtr> obsts);


    /**
     * @brief singleArmBouncePosture
     * @param steps
     * @param mov_type
     * @param pre_post
     * @param params
     * @param initPosture
     * @param finalPosture
     * @param bouncePosture
     * @return
     */
    bool singleArmBouncePosture(int steps,int mov_type,int pre_post,hump_params& params,std::vector<double> initPosture,std::vector<double> finalPosture,std::vector<double>& bouncePosture);


    /**
     * @brief writeFilesBouncePosture
     * @param steps
     * @param params
     * @param mov_type
     * @param pre_post
     * @param minAuxLimits
     * @param maxAuxLimits
     * @param initAuxPosture
     * @param finalAuxPosture
     * @param initialGuess
     * @param objs
     * @param bAux
     * @return
     */
    bool writeFilesBouncePosture(int steps,hump_params& params,int mov_type, int pre_post,std::vector<double> minAuxLimits, std::vector<double> maxAuxLimits,std::vector<double> initAuxPosture, std::vector<double> finalAuxPosture,
                                             std::vector<double> initialGuess, std::vector<objectPtr> objs,boundaryConditions bAux);


    /**
     * @brief getSteps
     * @param maxLimits
     * @param minLimits
     * @param initPosture
     * @param finalPosture
     * @return
     */
    int getSteps(std::vector<double>& maxLimits,std::vector<double>& minLimits,std::vector<double>& initPosture,std::vector<double>& finalPosture);

    /**
     * @brief getAlpha
     * @param arm
     * @param posture
     * @return
     */
    double getAlpha(int arm, std::vector<double> &posture);

    /**
     * @brief invKinematics
     * @param arm
     * @param pose
     * @param alpha
     * @param init_posture
     * @param posture
     * @return
     */
    int invKinematics(int arm, std::vector<double>& pose, double alpha, std::vector<double> &init_posture, std::vector<double>& posture);

    /**
     * @brief RotMatrix
     * @param theta
     * @param alpha
     * @param Rot
     */
    void RotMatrix(double theta, double alpha, Matrix3d& Rot);

    /**
     * @brief transfMatrix
     * @param alpha
     * @param a
     * @param d
     * @param theta
     * @param T
     */
    void transfMatrix(double alpha, double a, double d, double theta, Matrix4d &T);

    /**
     * @brief getRPY
     * @param rpy
     * @param Rot
     * @return
     */
    bool getRPY(std::vector<double>& rpy, Matrix3d& Rot);

    /**
     * @brief directKinematicsSingleArm
     * @param arm
     * @param posture
     */
    void directKinematicsSingleArm(int arm, std::vector<double>& posture);


    /**
     * @brief getShoulderPos
     * @param arm
     * @param posture
     * @param pos
     */
    void getShoulderPos(int arm,vector<double> &posture,vector<double> &pos);

    /**
     * @brief getShoulderOr
     * @param arm
     * @param posture
     * @param orient
     */
    void getShoulderOr(int arm, vector<double> &posture,vector<double> &orient);


    /**
     * @brief getElbowPos
     * @param arm
     * @param posture
     * @param pos
     */
    void getElbowPos(int arm,vector<double> &posture,vector<double> &pos);


    /**
     * @brief getElbowOr
     * @param arm
     * @param posture
     * @param orient
     */
    void getElbowOr(int arm, vector<double> &posture,vector<double> &orient);

    /**
     * @brief getWristPos
     * @param arm
     * @param posture
     * @param pos
     */
    void getWristPos(int arm,vector<double> &posture,vector<double> &pos);

    /**
     * @brief getWristOr
     * @param arm
     * @param posture
     * @param orient
     */
    void getWristOr(int arm, vector<double> &posture,vector<double> &orient);

    /**
     * @brief getHandPos
     * @param arm
     * @param posture
     * @param pos
     */
    void getHandPos(int arm,vector<double> &posture,vector<double> &pos);

    /**
     * @brief getHandOr
     * @param arm
     * @param posture
     * @param orient
     */
    void getHandOr(int arm, vector<double> &posture,vector<double> &orient);

    /**
     * @brief singleArmInvKinematics
     * @param params
     * @param hand_pose
     * @param init_posture
     * @param goal_posture
     * @return
     */
    bool singleArmInvKinematics(hump_params& params,std::vector<double> &init_posture,std::vector<double>& hand_pose,std::vector<double>& goal_posture);



};

} // namespace HUMotion

#endif // HUMPLANNER_H
