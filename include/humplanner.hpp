#ifndef HUMPLANNER_H
#define HUMPLANNER_H

#include "HUMPconfig.hpp"
#include "amplinterface.hpp"
#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"

using namespace Ipopt;
using namespace std;

namespace HUMotion{

typedef boost::shared_ptr<planning_result> planning_result_ptr; /**< pointer to the results of the planning process (single-arm) */
typedef boost::shared_ptr<planning_dual_result> planning_dual_result_ptr; /**< pointer to the results of the planning process (dual-arm) */

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
     * @brief addObstacleRight
     * @param obs
     */
    void addObstacleRight(objectPtr obs);

    /**
     * @brief addObstacleLeft
     * @param obs
     */
    void addObstacleLeft(objectPtr obs);

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


    // -------------- single-arm planning --------------------- //
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


    // -------------- dual-arm planning --------------------- //
    /**
     * @brief plan_dual_pick_pick
     * @param params
     * @param initPosture_right
     * @param initPosture_left
     * @return
     */
    planning_dual_result_ptr plan_dual_pick_pick(hump_dual_params& params, std::vector<double> initPosture_right, std::vector<double> initPosture_left);

    /**
     * @brief plan_dual_place_place
     * @param params
     * @param initPosture_right
     * @param initPosture_left
     * @return
     */
    planning_dual_result_ptr plan_dual_place_place(hump_dual_params& params, std::vector<double> initPosture_right, std::vector<double> initPosture_left);

    /**
     * @brief plan_dual_move_move
     * @param params
     * @param initPosture_right
     * @param initPosture_left
     * @return
     */
    planning_dual_result_ptr plan_dual_move_move(hump_dual_params& params, std::vector<double> initPosture_right, std::vector<double> initPosture_left);

    /**
     * @brief plan_dual_move_move
     * @param params
     * @param initPosture_right
     * @param finalPosture_right
     * @param initPosture_left
     * @param finalPosture_left
     * @return
     */
    planning_dual_result_ptr plan_dual_move_move(hump_dual_params& params, std::vector<double> initPosture_right, std::vector<double> finalPosture_right, std::vector<double> initPosture_left, std::vector<double> finalPosture_left);

    /**
     * @brief plan_dual_pick_place
     * @param params
     * @param initPosture_right
     * @param initPosture_left
     * @return
     */
    planning_dual_result_ptr plan_dual_pick_place(hump_dual_params& params, std::vector<double> initPosture_right, std::vector<double> initPosture_left);

    /**
     * @brief plan_dual_pick_move
     * @param params
     * @param initPosture_right
     * @param initPosture_left
     * @return
     */
    planning_dual_result_ptr plan_dual_pick_move(hump_dual_params& params, std::vector<double> initPosture_right, std::vector<double> initPosture_left);

    /**
     * @brief plan_dual_place_pick
     * @param params
     * @param initPosture_right
     * @param initPosture_left
     * @return
     */
    planning_dual_result_ptr plan_dual_place_pick(hump_dual_params& params, std::vector<double> initPosture_right, std::vector<double> initPosture_left);

    /**
     * @brief plan_dual_place_move
     * @param params
     * @param initPosture_right
     * @param initPosture_left
     * @return
     */
    planning_dual_result_ptr plan_dual_place_move(hump_dual_params& params, std::vector<double> initPosture_right, std::vector<double> initPosture_left);

    /**
     * @brief plan_dual_move_pick
     * @param params
     * @param initPosture_right
     * @param initPosture_left
     * @return
     */
    planning_dual_result_ptr plan_dual_move_pick(hump_dual_params& params, std::vector<double> initPosture_right, std::vector<double> initPosture_left);

    /**
     * @brief plan_dual_move_place
     * @param params
     * @param initPosture_right
     * @param initPosture_left
     * @return
     */
    planning_dual_result_ptr plan_dual_move_place(hump_dual_params& params, std::vector<double> initPosture_right, std::vector<double> initPosture_left);



private:

    string name;/**< name of the planner */
    // scenario info
    vector<objectPtr> obstacles; /**< obstacles in the scenario (single-arm) */
    vector<objectPtr> obstacles_right; /**< obstacles in the scenario (dual-arm right) */
    vector<objectPtr> obstacles_left; /**< obstacles in the scenario (dual-arm left) */
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
     * @brief getDerivative
     * @param function
     * @param step_values
     * @param derFunction
     */
    void getDerivative(std::vector<double> &function, std::vector<double> &step_values, std::vector<double> &derFunction);

    /**
     * @brief getTimeStep
     * @param tols
     * @param jointTraj
     * @param mod
     * @return
     */
    double getTimeStep(hump_params& tols, MatrixXd& jointTraj, int mod);

    /**
     * @brief getDualTimeStep
     * @param tols
     * @param jointTraj
     * @param mod
     * @return
     */
    double getDualTimeStep(hump_dual_params& tols, MatrixXd& jointTraj, int mod);

    /**
     * @brief setBoundaryConditions
     * @param mov_type
     * @param params
     * @param steps
     * @param initPosture
     * @param finalPosture
     * @param mod
     * @return
     */
    bool setBoundaryConditions(int mov_type, hump_params& params, int steps, std::vector<double>& initPosture, std::vector<double>& finalPosture, int mod=0);

    /**
     * @brief setDualBoundaryConditions
     * @param dual_mov_type
     * @param params
     * @param steps
     * @param initPosture
     * @param finalPosture
     * @param mod
     * @return
     */
    bool setDualBoundaryConditions(int dual_mov_type, hump_dual_params& params, int steps, std::vector<double>& initPosture, std::vector<double>& finalPosture, int mod=0);

    /**
     * @brief directTrajectory
     * @param mov_type
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param timestep
     * @param Traj
     * @param vel_app_ret
     * @param mod
     * @return
     */
    bool directTrajectory(int mov_type, int steps, hump_params& tols, std::vector<double>& initPosture, std::vector<double>& finalPosture, double timestep, MatrixXd& Traj, MatrixXd &vel_app_ret, int mod);

    /**
     * @brief directDualTrajectory
     * @param dual_mov_type
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param timestep
     * @param Traj
     * @param vel_app_ret
     * @param mod
     * @return
     */
    bool directDualTrajectory(int dual_mov_type, int steps, hump_dual_params& tols, std::vector<double>& initPosture, std::vector<double>& finalPosture, double timestep, MatrixXd& Traj, MatrixXd &vel_app_ret, int mod);

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
     * @param vel_app_ret
     * @param mod
     * @return
     */
    bool directVelocity(int steps, hump_params& tols, std::vector<double>& initPosture, std::vector<double>& finalPosture, double timestep, MatrixXd& Vel, MatrixXd &vel_app_ret, int mod);

    /**
     * @brief directDualVelocity
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param timestep
     * @param Vel
     * @param vel_app_ret
     * @param mod
     * @return
     */
    bool directDualVelocity(int steps, hump_dual_params& tols, std::vector<double>& initPosture, std::vector<double>& finalPosture, double timestep, MatrixXd& Vel, MatrixXd &vel_app_ret, int mod);

    /**
     * @brief directAcceleration
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param timestep
     * @param Acc
     * @param vel_app_ret
     * @param mod
     * @return
     */
    bool directAcceleration(int steps,hump_params& tols, std::vector<double>& initPosture, std::vector<double>& finalPosture, double timestep, MatrixXd& Acc,MatrixXd &vel_app_ret,int mod);

    /**
     * @brief directDualAcceleration
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param timestep
     * @param Acc
     * @param vel_app_ret
     * @param mod
     * @return
     */
    bool directDualAcceleration(int steps,hump_dual_params& tols, std::vector<double>& initPosture, std::vector<double>& finalPosture, double timestep, MatrixXd& Acc,MatrixXd &vel_app_ret,int mod);

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
     * @param initPosture
     * @param bouncePosture
     * @param timestep
     * @param Vel
     */
    void backForthVelocity(int steps, std::vector<double>& initPosture, std::vector<double>& bouncePosture, double timestep, MatrixXd& Vel);

    /**
     * @brief backForthAcceleration
     * @param steps
     * @param initPosture
     * @param bouncePosture
     * @param timestep
     * @param Acc
     */
    void backForthAcceleration(int steps, std::vector<double>& initPosture, std::vector<double>& bouncePosture, double timestep, MatrixXd& Acc);

    /**
     * @brief computeMovement
     * @param direct
     * @param back
     * @param tot
     */
    void computeMovement(const MatrixXd& direct, const MatrixXd& back, MatrixXd& tot);

    /**
     * @brief getTrajectory
     * @param mov_type
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param bouncePosture
     * @param traj
     * @param vel_app_ret
     * @param success
     * @param mod
     * @return
     */
    double getTrajectory(int mov_type,int steps,hump_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, std::vector<double> bouncePosture,
                         MatrixXd &traj,MatrixXd &vel_app_ret,bool &success,int mod);

    /**
     * @brief getDualTrajectory
     * @param dual_mov_type
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param bouncePosture
     * @param traj
     * @param vel_app_ret
     * @param success
     * @param mod
     * @return
     */
    double getDualTrajectory(int dual_mov_type,int steps,hump_dual_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, std::vector<double> bouncePosture,
                         MatrixXd &traj,MatrixXd &vel_app_ret,bool &success,int mod);

    /**
     * @brief getTrajectory
     * @param mov_type
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param traj
     * @param vel_app_ret
     * @param success
     * @param mod
     * @return
     */
    double getTrajectory(int mov_type,int steps,hump_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture,
                         MatrixXd &traj, MatrixXd &vel_app_ret,bool &success, int mod);

    /**
     * @brief getDualTrajectory
     * @param dual_mov_type
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param traj
     * @param vel_app_ret
     * @param success
     * @param mod
     * @return
     */
    double getDualTrajectory(int dual_mov_type,int steps,hump_dual_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture,
                         MatrixXd &traj, MatrixXd &vel_app_ret,bool &success, int mod);

    /**
     * @brief getVelocity
     * @param mov_type
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param bouncePosture
     * @param traj
     * @param vel
     * @param vel_app_ret
     * @param success
     * @param mod
     * @return
     */
    double getVelocity(int mov_type, int steps, hump_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, std::vector<double> bouncePosture,
                       MatrixXd &traj, MatrixXd &vel, MatrixXd &vel_app_ret, bool &success, int mod);

    /**
     * @brief getDualVelocity
     * @param dual_mov_type
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param bouncePosture
     * @param traj
     * @param vel
     * @param vel_app_ret
     * @param success
     * @param mod
     * @return
     */
    double getDualVelocity(int dual_mov_type, int steps, hump_dual_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, std::vector<double> bouncePosture,
                       MatrixXd &traj, MatrixXd &vel, MatrixXd &vel_app_ret, bool &success, int mod);
    /**
     * @brief getVelocity
     * @param mov_type
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param traj
     * @param vel
     * @param vel_app_ret
     * @param success
     * @param mod
     * @return
     */
    double getVelocity(int mov_type, int steps, hump_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, MatrixXd &traj, MatrixXd &vel, MatrixXd &vel_app_ret,bool &success, int mod);

    /**
     * @brief getDualVelocity
     * @param dual_mov_type
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param traj
     * @param vel
     * @param vel_app_ret
     * @param success
     * @param mod
     * @return
     */
    double getDualVelocity(int dual_mov_type, int steps, hump_dual_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, MatrixXd &traj, MatrixXd &vel, MatrixXd &vel_app_ret,bool &success, int mod);

    /**
     * @brief getAcceleration
     * @param mov_type
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param bouncePosture
     * @param traj
     * @param vel
     * @param acc
     * @param success
     * @param mod
     * @return
     */
    double getAcceleration(int mov_type, int steps, hump_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, std::vector<double> bouncePosture, MatrixXd &traj, MatrixXd &vel, MatrixXd &acc, bool &success, int mod);

    /**
     * @brief getDualAcceleration
     * @param dual_mov_type
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param bouncePosture
     * @param traj
     * @param vel
     * @param acc
     * @param success
     * @param mod
     * @return
     */
    double getDualAcceleration(int dual_mov_type, int steps, hump_dual_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, std::vector<double> bouncePosture, MatrixXd &traj, MatrixXd &vel, MatrixXd &acc, bool &success, int mod);

    /**
     * @brief getAcceleration
     * @param mov_type
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param traj
     * @param vel
     * @param acc
     * @param success
     * @param mod
     * @return
     */
    double getAcceleration(int mov_type, int steps, hump_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, MatrixXd &traj, MatrixXd &vel, MatrixXd &acc, bool &success, int mod);

    /**
     * @brief getDualAcceleration
     * @param dual_mov_type
     * @param steps
     * @param tols
     * @param initPosture
     * @param finalPosture
     * @param traj
     * @param vel
     * @param acc
     * @param success
     * @param mod
     * @return
     */
    double getDualAcceleration(int dual_mov_type, int steps, hump_dual_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, MatrixXd &traj, MatrixXd &vel, MatrixXd &acc, bool &success, int mod);

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
     * @brief writeDualArmDHParams
     * @param dh_right
     * @param dh_left
     * @param stream
     */
    void writeDualArmDHParams(DHparameters dh_right, DHparameters dh_left, std::ofstream& stream);

    /**
     * @brief This method writes down the distance between the object and the hand
     * @param stream
     * @param dHO
     */
    void write_dHO(std::ofstream& stream, double dHO);

    /**
     * @brief write_dual_dHO
     * @param stream
     * @param dHO_right
     * @param dHO_left
     */
    void write_dual_dHO(std::ofstream& stream, double dHO_right, double dHO_left);

    /**
     * @brief This method writes down the joint limits of the arm
     * @param stream
     * @param minArmLimits
     * @param maxArmLimits
     * @param final
     */
    void writeArmLimits(std::ofstream& stream, std::vector<double>& minArmLimits, std::vector<double>& maxArmLimits, bool final);

    /**
     * @brief writeArmLimitsMultipliers
     * @param stream
     * @param minArmLimitsMultipliers
     * @param maxArmLimitsMultipliers
     */
    void writeArmLimitsMultipliers(std::ofstream& stream, std::vector<double>& minArmLimitsMultipliers, std::vector<double>& maxArmLimitsMultipliers);

    /**
     * @brief writeFinalConstraintsMultipliers
     * @param stream
     * @param coll
     * @param coll_body
     * @param coll_obsts
     * @param n_s
     * @param n_obsts
     * @param mov_type
     * @param pre_post
     * @param n_tar
     * @param duals
     */
    void writeFinalConstraintsMultipliers(std::ofstream& stream, bool coll, bool coll_body, bool coll_obsts, int n_s, int n_obsts, int mov_type, int pre_post, int n_tar, std::vector<double> &duals);

    /**
     * @brief writeBounceConstraintsMultipliers
     * @param stream
     * @param coll
     * @param coll_body
     * @param coll_obsts
     * @param n_s
     * @param n_obsts
     * @param mov_type
     * @param pre_post
     * @param n_tar
     * @param duals
     */
    void writeBounceConstraintsMultipliers(std::ofstream& stream, int n_s, int n_obsts, int mov_type, int pre_post, int n_tar, std::vector<double> &duals);

    /**
     * @brief writeConstraintsMultipliersMod
     * @param stream
     */
    void writeConstraintsMultipliersMod(std::ofstream& stream);

    /**
     * @brief writeDualArmLimits
     * @param stream
     * @param minRightArmLimits
     * @param maxRightArmLimits
     * @param minLeftArmLimits
     * @param maxLeftArmLimits
     * @param final
     */
    void writeDualArmLimits(std::ofstream& stream, std::vector<double>& minRightArmLimits, std::vector<double>& maxRightArmLimits,
                            std::vector<double>& minLeftArmLimits, std::vector<double>& maxLeftArmLimits, bool final);

    /**
     * @brief This method writes down the initial posture of the arm
     * @param stream
     * @param initArmPosture
     */
    void writeArmInitPose(std::ofstream& stream,std::vector<double>& initArmPosture);

    /**
     * @brief writeDualArmInitPose
     * @param stream
     * @param initRightArmPosture
     * @param initLeftArmPosture
     */
    void writeDualArmInitPose(std::ofstream& stream,std::vector<double>& initRightArmPosture,std::vector<double>& initLeftArmPosture);

    /**
     * @brief This method writes down the final posture of the fingers
     * @param stream
     * @param finalHand
     */
    void writeFingerFinalPose(std::ofstream& stream,std::vector<double>& finalHand);

    /**
     * @brief writeFingerDualFinalPose
     * @param stream
     * @param finalHand_right
     * @param finalHand_left
     */
    void writeFingerDualFinalPose(std::ofstream& stream,std::vector<double>& finalHand_right,std::vector<double>& finalHand_left);

    /**
     * @brief This method writes down the lambda of the objective function
     * @param stream
     * @param lambda
     */
    void writeLambda(std::ofstream& stream,std::vector<double>& lambda);

    /**
     * @brief writeDualLambda
     * @param stream
     * @param lambda_right
     * @param lambda_left
     */
    void writeDualLambda(std::ofstream& stream,std::vector<double>& lambda_right,std::vector<double>& lambda_left);

    /**
     * @brief writeHumanHandParams
     * This method writes down the parameter of the human hand
     * @param hhand
     * @param stream
     * @param k
     */
    void writeHumanHandParams(HumanHand& hhand, std::ofstream& stream, int k);

    /**
     * @brief writeDualHumanHandParams
     * @param hhand
     * @param stream
     * @param right
     */
    void writeDualHumanHandParams(HumanHand& hhand, std::ofstream& stream, bool right);

    /**
     * @brief This method writes down the declaration of the parameters of the human hand
     * @param stream
     */
    void writeHumanHandParamsMod(std::ofstream& stream);

    /**
     * @brief writeDualHumanHandParamsMod
     * @param stream
     * @param right
     */
    void writeDualHumanHandParamsMod(std::ofstream& stream,bool right);

    /**
     * @brief This method writes down the direct kinematics of the human hand
     * @param stream
     * @param tolsHand
     * @param final
     * @param transport
     */
    void writeHumanHandDirKin(std::ofstream& stream,MatrixXd& tolsHand, bool final, bool transport);

    /**
     * @brief writeDualHumanHandDirKin
     * @param stream
     * @param tolsHand
     * @param final
     * @param transport_right
     * @param transport_left
     * @param right
     */
    void writeDualHumanHandDirKin(std::ofstream& stream,MatrixXd& tolsHand, bool final, bool transport_right, bool transport_left,bool right);

    /**
     * @brief writeBarrettHandParams
     * Thi method writes down the parameters of the Barrett Hand
     * @param bhand
     * @param stream
     */
    void writeBarrettHandParams(BarrettHand& bhand, std::ofstream& stream);

    /**
     * @brief writeDualBarrettHandParams
     * @param bhand
     * @param stream
     * @param right
     */
    void writeDualBarrettHandParams(BarrettHand& bhand, std::ofstream& stream,bool right);

    /**
     * @brief This method writes down the declaration of the parameters of the Barrett hand
     * @param stream
     */
    void writeBarrettHandParamsMod(std::ofstream& stream);

    /**
     * @brief writeDualBarrettHandParamsMod
     * @param stream
     * @param right
     */
    void writeDualBarrettHandParamsMod(std::ofstream& stream, bool right);

    /**
     * @brief This method writes down the direct kinematics of the Barrett hand
     * @param stream
     * @param tolsHand
     * @param final
     * @param transport
     */
    void writeBarrettHandDirKin(std::ofstream& stream, MatrixXd& tolsHand, bool final, bool transport);

    /**
     * @brief writeDualBarrettHandDirKin
     * @param stream
     * @param tolsHand
     * @param final
     * @param transport_right
     * @param transport_left
     * @param right
     */
    void writeDualBarrettHandDirKin(std::ofstream& stream, MatrixXd& tolsHand, bool final, bool transport_right, bool transport_left, bool right);


    /**
     * @brief writeInfoTarget
     * This method writes down the info of the target
     * @param stream
     * @param tar: tar(0)=x, tar(1)=y, tar(2)=z, tar(3)=roll, tar(4)=pitch, tar(5)=yaw
     */
    void writeInfoTarget(ofstream &stream, std::vector<double> tar);

    /**
     * @brief writeDualInfoTarget
     * @param stream
     * @param tar_right : tar(0)=x, tar(1)=y, tar(2)=z, tar(3)=roll, tar(4)=pitch, tar(5)=yaw
     * @param tar_left : tar(0)=x, tar(1)=y, tar(2)=z, tar(3)=roll, tar(4)=pitch, tar(5)=yaw
     */
    void writeDualInfoTarget(ofstream &stream,std::vector<double> tar_right,std::vector<double> tar_left);

    /**
     * @brief writeInfoObstacles
     * @param stream
     * @param obstacles
     */
    void writeInfoObstacles(ofstream &stream, std::vector<objectPtr> &obstacles);

    /**
     * @brief writeDualInfoObstacles
     * @param stream
     * @param obstacles
     * @param right
     */
    void writeDualInfoObstacles(ofstream &stream, std::vector<objectPtr> &obstacles, bool right);

    /**
     * @brief writeInfoObjectTarget
     * @param stream
     * @param obj
     */
    void writeInfoObjectTarget(ofstream &stream, objectPtr obj);

    /**
     * @brief writeInfoObjectTarget
     * @param stream
     * @param tar
     * @param T_tar_to_obj
     * @param dim
     * @param name
     */
    void writeInfoObjectTarget(ofstream &stream,std::vector<double> tar, Matrix4d T_tar_to_obj, std::vector<double> dim, std::string name);

    /**
     * @brief writeInfoObjectTargetPlaceRetreat
     * @param stream
     * @param tar
     * @param T_tar_to_obj
     * @param dim
     * @param name
     */
    void writeInfoObjectTargetPlaceRetreat(ofstream &stream,std::vector<double> tar, Matrix4d T_tar_to_obj,std::vector<double> dim, std::string name);

    /**
     * @brief writeDualInfoObjectTarget
     * @param stream
     * @param obj_right
     * @param obj_left
     */
    void writeDualInfoObjectTarget(ofstream &stream, objectPtr obj_right, objectPtr obj_left);

    /**
     * @brief writeDualInfoObjectTarget
     * @param stream
     * @param tar_right
     * @param T_tar_to_obj_right
     * @param dim_right
     * @param name_right
     * @param tar_left
     * @param T_tar_to_obj_left
     * @param dim_left
     * @param name_left
     */
    void writeDualInfoObjectTarget(ofstream &stream, std::vector<double> tar_right, Matrix4d T_tar_to_obj_right, std::vector<double> dim_right, std::string name_right, std::vector<double> tar_left, Matrix4d T_tar_to_obj_left,std::vector<double> dim_left,std::string name_left);

    /**
     * @brief writeDualInfoObjectTargetPlaceRetreat
     * @param stream
     * @param tar_right
     * @param T_tar_to_obj_right
     * @param dim_right
     * @param name_right
     * @param tar_left
     * @param T_tar_to_obj_left
     * @param dim_left
     * @param name_left
     */
    void writeDualInfoObjectTargetPlaceRetreat(ofstream &stream, std::vector<double> tar_right, Matrix4d T_tar_to_obj_right, std::vector<double> dim_right, std::string name_right, std::vector<double> tar_left, Matrix4d T_tar_to_obj_left,std::vector<double> dim_left,std::string name_left);

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
     * @brief writeDualArmDHParamsMod
     * @param stream
     */
    void writeDualArmDHParamsMod(ofstream &stream);

    /**
     * @brief write_dHOMod
     * @param stream
     */
    void write_dHOMod(ofstream &stream);

    /**
     * @brief write_dual_dHOMod
     * @param stream
     */
    void write_dual_dHOMod(ofstream &stream);
    /**
     * @brief writeInfoObjectsMod
     * @param stream
     * @param vec
     */
    void writeInfoObjectsMod(ofstream &stream,bool vec);

    /**
     * @brief writeDualInfoObjectsMod
     * @param stream
     * @param vec_right
     * @param vec_left
     */
    void writeDualInfoObjectsMod(ofstream &stream,bool vec_right,bool vec_left);

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
     * @brief writeDualRotMatObsts
     * @param stream
     */
    void writeDualRotMatObsts(ofstream &stream);

    /**
     * @brief writeRotMatObjTar
     * @param stream
     */
    void writeRotMatObjTar(ofstream &stream);

    /**
     * @brief writeRotMatObjTarDual
     * @param stream
     */
    void writeRotMatObjTarDual(ofstream &stream);

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
     * @brief writeDualArmDirKin
     * @param stream
     * @param dual_mov_type
     * @param matWorldToRightArm
     * @param matRightHand
     * @param tolsRightArm
     * @param matWorldToLeftArm
     * @param matLeftHand
     * @param tolsLeftArm
     * @param final
     */
    void writeDualArmDirKin(ofstream &stream, int dual_mov_type, Matrix4d &matWorldToRightArm, Matrix4d &matRightHand, std::vector<double>& tolsRightArm, Matrix4d &matWorldToLeftArm, Matrix4d &matLeftHand, std::vector<double>& tolsLeftArm,bool final);

    /**
     * @brief writeInitDualArmDirKin
     * @param stream
     * @param tolsRightArm
     * @param tolsLeftArm
     */
    void writeInitDualArmDirKin(ofstream &stream, std::vector<double>& tolsRightArm,std::vector<double>& tolsLeftArm);
    /**
     * @brief writeObjective
     * @param stream
     * @param final
     */
    void writeObjective(ofstream &stream, bool final);

    /**
     * @brief writeBodyConstraints
     * @param stream
     * @param warm_start
     * @param final
     */
    void writeBodyConstraints(ofstream &stream, bool warm_start, bool final);

    /**
     * @brief writeDualBodyConstraints
     * @param stream
     * @param final
     * @param right
     */
    void writeDualBodyConstraints(ofstream &stream, bool final,bool right);

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
     * @param zL
     * @param zU
     * @param lambda
     * @param iter_count
     * @param cpu_time
     * @param obj
     * @param tol
     * @param acc_tol
     * @param constr_viol_tol
     * @return
     */
    bool optimize(string &nlfile, std::vector<Number>& x, std::vector<Number> &zL, std::vector<Number> &zU, std::vector<Number> &lambda, Index &iter_count, Number &cpu_time, Number &obj, double tol, double acc_tol, double constr_viol_tol);

    /**
     * @brief optimize_warm_start
     * @param nlfile
     * @param x
     * @param zL
     * @param zU
     * @param lambda
     * @param iter_count
     * @param cpu_time
     * @param obj
     * @param tol
     * @param acc_tol
     * @param constr_viol_tol
     * @return
     */
    bool optimize_warm_start(string &nlfile, std::vector<Number>& x, std::vector<Number>& zL, std::vector<Number>& zU, std::vector<Number>& lambda, Index &iter_count, Number &cpu_time, Number &obj, double tol, double acc_tol, double constr_viol_tol);


    /**
     * @brief getObstaclesSingleArm
     * @param center
     * @param radius
     * @param obsts
     * @param hand_code
     */
    void getObstaclesSingleArm(std::vector<double> center, double radius, std::vector<objectPtr>& obsts, int hand_code);

    /**
     * @brief getObstaclesDualArm
     * @param center_right
     * @param center_left
     * @param radius_right
     * @param radius_left
     * @param obsts_right
     * @param obsts_left
     * @param hand_code_right
     * @param hand_code_left
     */
    void getObstaclesDualArm(std::vector<double> center_right, std::vector<double> center_left, double radius_right, double radius_left, std::vector<objectPtr>& obsts_right, std::vector<objectPtr>& obsts_left,int hand_code_right,int hand_code_left);

    /**
     * @brief writeInfoApproachRetreat
     * @param stream
     * @param tar
     * @param approach_retreat
     */
    void writeInfoApproachRetreat(ofstream &stream, std::vector<double> tar, std::vector<double> approach_retreat);

    /**
     * @brief writeDualInfoApproachRetreat
     * @param stream
     * @param tar
     * @param approach_retreat
     * @param right
     */
    void writeDualInfoApproachRetreat(ofstream &stream, std::vector<double> tar, std::vector<double> approach_retreat,bool right);

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
     * @param zL
     * @param zU
     * @param lambda
     * @param iterations
     * @param time
     * @param obj
     * @return
     */
    bool singleArmFinalPosture(int mov_type, int pre_post, hump_params& params, std::vector<double> initPosture, std::vector<double>& finalPosture, std::vector<double> &zL, std::vector<double> &zU, std::vector<double> &lambda, int &iterations, double &time, double &obj);

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
     * @param x
     * @param zL
     * @param zU
     * @param lambda_dual
     * @param iterations
     * @param time
     * @param obj
     * @return
     */
    bool singleArmBouncePosture(int steps, int mov_type, int pre_post, hump_params& params, std::vector<double> initPosture, std::vector<double> finalPosture, std::vector<double>& bouncePosture, std::vector<double> &x, std::vector<double> &zL, std::vector<double> &zU, std::vector<double> &lambda_dual, int &iterations, double &time, double &obj);

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
     * @param lambda
     * @param objs
     * @param bAux
     * @return
     */
    bool writeFilesBouncePosture(int steps,hump_params& params,int mov_type, int pre_post,std::vector<double> minAuxLimits, std::vector<double> maxAuxLimits,std::vector<double> initAuxPosture, std::vector<double> finalAuxPosture,
                                             std::vector<double> initialGuess, std::vector<double> lambda, std::vector<objectPtr> objs,boundaryConditions bAux);


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
     * @brief model_spheres
     * @param stream_dat
     * @param stream_model
     * @param obj_tar_size
     * @param final
     * @return
     */
    int model_spheres(ofstream &stream_dat,ofstream &stream_model,std::vector<double>& obj_tar_size,bool final);

    /**
     * @brief dual_obj_model_spheres
     * @param stream_dat
     * @param stream_model
     * @param obj_tar_right_size
     * @param obj_tar_left_size
     * @param final
     * @param n_s_right
     * @param n_s_left
     */
    void dual_obj_model_spheres(ofstream &stream_dat,ofstream &stream_model,std::vector<double>& obj_tar_right_size,std::vector<double>& obj_tar_left_size,bool final, int& n_s_right, int& n_s_left);

    /**
     * @brief dual_obj_model_spheres
     * @param stream_dat
     * @param stream_model
     * @param obj_tar_size
     * @param final
     * @return
     */
    int dual_obj_model_spheres(ofstream &stream_dat,ofstream &stream_model,std::vector<double>& obj_tar_size,bool final);

    /**
     * @brief compare_sizes
     * @param pair_1
     * @param pair_2
     * @return
     */
    bool static compare_sizes (std::pair<std::string,double> pair_1, std::pair<std::string,double> pair_2);

    //double getAlpha(int arm, std::vector<double> &posture);


    //int invKinematics(int arm, std::vector<double>& pose, double alpha, std::vector<double> &init_posture, std::vector<double>& posture);

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


    //bool singleArmInvKinematics(hump_params& params,std::vector<double> &init_posture,std::vector<double>& hand_pose,std::vector<double>& goal_posture);

    /**
     * @brief singleDualArmFinalPosture
     * @param dual_mov_type
     * @param pre_post
     * @param params
     * @param initPosture
     * @param finalPosture
     * @return
     */
    bool singleDualArmFinalPosture(int dual_mov_type,int pre_post,hump_dual_params& params,std::vector<double> initPosture,std::vector<double>& finalPosture);

    /**
     * @brief writeFilesDualFinalPosture
     * @param params
     * @param dual_mov_type
     * @param pre_post
     * @param initRightArmPosture
     * @param initLeftArmPosture
     * @param initialGuess
     * @param obsts_right
     * @param obsts_left
     * @return
     */
    bool writeFilesDualFinalPosture(hump_dual_params& params,int dual_mov_type, int pre_post,std::vector<double> initRightArmPosture, std::vector<double> initLeftArmPosture, std::vector<double> initialGuess,std::vector<objectPtr> obsts_right,std::vector<objectPtr> obsts_left);

    /**
     * @brief singleDualArmBouncePosture
     * @param steps
     * @param dual_mov_type
     * @param pre_post
     * @param params
     * @param initPosture
     * @param finalPosture
     * @param bouncePosture
     * @return
     */
    bool singleDualArmBouncePosture(int steps,int dual_mov_type,int pre_post,hump_dual_params& params,std::vector<double> initPosture,std::vector<double> finalPosture,std::vector<double>& bouncePosture);

    /**
     * @brief writeFilesDualBouncePosture
     * @param steps
     * @param params
     * @param dual_mov_type
     * @param pre_post
     * @param minAuxLimits
     * @param maxAuxLimits
     * @param initAuxPosture
     * @param finalAuxPosture
     * @param initialGuess
     * @param lambda
     * @param objs_right
     * @param objs_left
     * @param bAux
     * @return
     */
    bool writeFilesDualBouncePosture(int steps,hump_dual_params& params,int dual_mov_type, int pre_post,std::vector<double> minAuxLimits, std::vector<double> maxAuxLimits,std::vector<double> initAuxPosture, std::vector<double> finalAuxPosture,
                                             std::vector<double> initialGuess, std::vector<double> lambda, std::vector<objectPtr> objs_right,std::vector<objectPtr> objs_left,boundaryConditions bAux);


};

} // namespace HUMotion

#endif // HUMPLANNER_H
