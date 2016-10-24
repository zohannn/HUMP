#ifndef PROBLEM_H
#define PROBLEM_H


#include "movement.hpp"
#include "scenario.hpp"
#include "amplinterface.hpp"


namespace HUMotion{

typedef boost::shared_ptr<Movement> movementPtr; /**< shared pointer to a movement */
typedef boost::shared_ptr<Scenario> scenarioPtr; /**< shared pointer to a scenario */

//! The Problem class
/**
 * @brief The Problem class
 */
class Problem
{
public:

    /**
     * @brief Problem, a constructor
     */
    Problem();

    /**
     * @brief Problem, a constructor
     * @param mov
     * @param scene
     */
    Problem(Movement* mov, Scenario* scene);

    /**
     * @brief Problem, a copy constructor
     * @param s
     */
    Problem(const Problem& s);

    /**
     * @brief ~Problem, a desctructor
     */
    ~Problem();


    /**
     * @brief This method sets the axis of the target that during the movement has to be approached
     * @param a
     */
    void setApproachingTargetAxis(int a);

    /**
     * @brief This method sets the problem as solved or unsolved
     * @param s
     */
    void setSolved(bool s);

    /**
     * @brief If p is true, then this method sets the problem as part of a task, otherwise the problem is not part of the task
     * @param p
     */
    void setPartOfTask(bool p);

    /**
     * @brief This method solves the problem give the tolerances and the parameters
     * @param probTols
     * @return
     */
    bool solve(Tols probTols);

    /**
     * @brief This method gets the information of the problem
     * @return
     */
    string getInfoLine();

    /**
     * @brief This method gets the trajectory planned
     * @param traj
     * @return
     */
    float getTrajectory(MatrixXf& traj);

    /**
     * @brief This method gets the velocity of the trajectory planned
     * @param vel
     * @return
     */
    float getVelocity(MatrixXf& vel);

    /**
     * @brief This method gets the movement that is related to the problem
     * @return
     */
    movementPtr getMovement();

    /**
     * @brief This method gets the final posture of the arm
     * @param arm_code: 1 to indicate the right arm, 2 to indicate the left arm
     * @param final_posture: final posture of the arm in [rad]
     */
    void getFinalPosture(int arm_code, std::vector<float>& final_posture);

    /**
     * @brief This method gets the bounce posture of the arm
     * @param arm_code: 1 to indicate the right arm, 2 to indicate the left arm
     * @param bounce_posture: bounce posture of the arm in [rad]
     */
    void getBouncePosture(int arm_code, std::vector<float>& bounce_posture);

    /**
     * @brief This method gets the engaged object given in engaging/disengaging movements
     * @return
     */
    objectPtr getObjectEngaged();

    /**
     * @brief This method returns true if the problem has been solved, false otherwise
     * @return
     */
    bool getSolved();

    /**
     * @brief This method returns true if the problem is part of a task, false otherwise
     * @return
     */
    bool getPartOfTask();

    /**
     * @brief This method gets the error log of the problem
     * @return
     */
    int getErrLog();


private:

    bool solved; /**< true if the problem has been solved */

   /**
     * @brief error log of the problem
     * <table>
     * <caption id="multi_row">Types of the errors</caption>
     * <tr><th>Type      <th>Code <th>Description
     * <tr><td>Any <td>0 <td>the problems have been successfully solved
     * <tr><td rowspan="2">Reach-to-grasp <td>10 <td>final posture reach-to-grasp not solved
     * <tr><td>20 <td>bounce posture reach-to-grasp not solved
     * <tr><td rowspan="4">Engage <td>130 <td>final posture engage sub-disengage not solved
     * <tr><td>13 <td>final posture engage not solved
     * <tr><td>131 <td>final posture engage sub-engage not solved
     * <tr><td>23 <td>bounce posture engage not solved
     * <tr><td>Go Park <td>25 <td> bounce posture Go Park not solved
     * </table>
     */
    int err_log;
    bool part_of_task; /**< true if the problem is part of a task */
    float dHOr; /**< distance between the right hand and the center of the object that is being manipulated */
    float dHOl; /**< distance between the left hand and the center of the object that is being manipulated */
    float dFF; /**< distance between the fingertip F3 and the fingertips F1 and F2 */
    float dFH; /**< distance between the fingers and the palm of the hand */
    std::vector<float> rightFinalPosture; /**< final posture of the right arm+hand */
    std::vector<float> rightFinalHand; /**< final posture of the right hand */
    std::vector<float> leftFinalPosture; /**< final posture of the left arm+hand */
    std::vector<float> leftFinalHand; /**< final posture of the left hand */
    std::vector<float> rightBouncePosture; /**< bounce posture of the right arm+hand */
    std::vector<float> leftBouncePosture; /**< bounce posture of the left arm+hand */
    std::vector<float> rightFinalPosture_diseng; /**< final posture of the right arm+hand for disengaging movements*/
    std::vector<float> rightFinalPosture_eng; /**< final posture of the right arm+hand for engaging movements*/
    std::vector<float> leftFinalPosture_diseng; /**< final posture of the left arm+hand for disengaging movements*/
    std::vector<float> leftFinalPosture_eng; /**< final posture of the left arm+hand for engaging movements*/
    MatrixXf optimalTraj; /**< human-like optimized trajectory */
    Tols tolerances; /**< tolerances and parameters of the optimization problem */
    movementPtr mov; /**< movement to be planned */
    scenarioPtr scene;/**< current scenario */
    int targetAxis; /**< approaching direction towards the target: 0 = none , 1 = x axis , 2 = y axis, 3 = z axis*/
    objectPtr obj_curr; /**< current object being manipulated */
    targetPtr tar_eng; /**< target of the engaged object */
    objectPtr obj_eng; /**< engaged object */

    /**
     * @brief This method computes the final posture of the fingers.\n
     * It takes into account the type of grip and the size of the object
     * @param hand_id: it is 1 for right hand and 2 for the left hand
     * @return
     */
    bool finalPostureFingers(int hand_id);

    /**
     * @brief This method computes the inverse kinematics of the hand.
     * @param d_obj: diameter of the object
     * @param hand_id: it is 1 for right hand and 2 for the left hand
     * @param sols: solution
     * @return
     */
    bool invKinHand(float d_obj,int hand_id,std::vector<float>& sols);

    // --- single arm --- //

    /**
     * @brief This method solves the final posture selection problem of single arm reach-to-grasp movements
     * @return
     */
    bool singleArmFinalPostureReachToGrasp();

    /**
     * @brief This method solves the bounce posture selection problem of reach-to-grasp movements
     * @return
     */
    bool singleArmBouncePostureReachToGrasp();

    /**
     * @brief This method solves the final posture selection problem of single arm sub-disengaging movements
     * @return
     */
    bool singleArmFinalPostureSubDisengage();

    /**
     * @brief This method solves the final posture selection problem of single arm engaging movements
     * @return
     */
    bool singleArmFinalPostureEngage();

    /**
     * @brief This method solves the final posture selection problem for sub-engaging movements
     * @return
     */
    bool singleArmFinalPostureSubEngage();

    /**
     * @brief This method solves the bounce posture selection problem for engaging movements
     * @return
     */
    bool singleArmBouncePostureEngage();

    /**
     * @brief This method solves the bounce posture selection problem of single arm go-park movements
     * @return
     */
    bool singleArmBouncePostureGoPark();

    /**
     * @brief This method gets the obstacles in the workspace of a single arm
     * @param objs
     * @param center
     * @param radius
     * @param obsts
     */
    void getObstaclesSingleArm(std::vector<objectPtr> objs, std::vector<float> center, float radius, std::vector<objectPtr>& obsts);

    /**
     * @brief This method writes all the necessary files to solve the final posture selection problem
     * @param mov_type
     * @param hh
     * @param dHO
     * @param griptype
     * @param initArmPosture
     * @param finalHand
     * @param initialGuess
     * @param objs
     * @param tar
     * @param obj
     * @param tolsArm
     * @param tolsHand
     * @param tolsObstacles
     * @param tolTarPos
     * @param tolTarOr
     * @param lambda
     * @param obstacle_avoidance
     * @param arm_code
     * @return
     */
    bool writeFilesFinalPosture(int mov_type,humanoidPtr hh,float dHO,int griptype,
                                std::vector<float> initArmPosture, std::vector<float> finalHand,
                                std::vector<float> initialGuess,
                                std::vector<objectPtr> objs,
                                targetPtr tar, objectPtr obj,
                                std::vector<float> tolsArm, MatrixXf tolsHand, MatrixXf tolsObstacles,
                                float tolTarPos, float tolTarOr,std::vector<float> lambda,
                                bool obstacle_avoidance,int arm_code);

    /**
     * @brief This method writes all the necessary files to solve the bounce posture selection problem
     * @param move_type
     * @param hh
     * @param dHO
     * @param griptype
     * @param steps
     * @param totalTime
     * @param minAuxLimits
     * @param maxAuxLimits
     * @param initAuxPosture
     * @param finalAuxPosture
     * @param finalHand
     * @param initialGuess
     * @param objs
     * @param tar
     * @param obj
     * @param b
     * @param tolsArm
     * @param tolsHand
     * @param tolsTarget
     * @param tolsObstacles
     * @param lambda
     * @param target_avoidance
     * @param obstacle_avoidance
     * @param arm_code
     * @return
     */
    bool writeFilesBouncePosture(int move_type,humanoidPtr hh, float dHO, int griptype, int steps, float totalTime,
                                 std::vector<float> minAuxLimits, std::vector<float> maxAuxLimits,
                                 std::vector<float> initAuxPosture,std::vector<float> finalAuxPosture, std::vector<float> finalHand,
                                 std::vector<float> initialGuess,std::vector<objectPtr> objs,
                                 targetPtr tar, objectPtr obj,
                                 boundaryConditions b, std::vector<float> tolsArm, MatrixXf tolsHand,
                                 std::vector< MatrixXf > tolsTarget, std::vector< MatrixXf > tolsObstacles, std::vector<float> lambda,
                                 bool target_avoidance, bool obstacle_avoidance, int arm_code);


    /**
     * @brief This method reads the .mod and .dat files and produces a .nl file of the problem
     * @param datFile
     * @param modFile
     * @param nlFile
     * @return
     */
    bool amplRead(string& datFile, string& modFile, string& nlFile);

    /**
     * @brief This method solves the problem described in the .nl file
     * @param nlfile
     * @param x_sol
     * @param tol
     * @param acc_tol
     * @return
     */
    bool optimize(string& nlfile,std::vector<Number>& x_sol, float tol, float acc_tol);


    /**
     * @brief Given the direct and the back-and-forth trajectories, this method computes the composite trajectory
     * @param dTraj
     * @param bTraj
     */
    void computeTraj(const MatrixXf& dTraj, const MatrixXf& bTraj);

    /**
     * @brief This method computes the direct movement
     * @param initPosture
     * @param finalPosture
     * @param steps
     * @param Traj
     */
    void directMovement(std::vector<float> initPosture, std::vector<float> finalPosture, int steps, MatrixXf& Traj);

    /**
     * @brief This method computes the back-and-forth movement
     * @param initPosture
     * @param bouncePosture
     * @param steps
     * @param Traj
     */
    void backForthMovement(std::vector<float> initPosture, std::vector<float> bouncePosture, int steps, MatrixXf& Traj);

    /**
     * @brief This method computes the numerical differentiation of the trajectory
     * @param jointTraj
     * @param delta
     */
    void getDelta(VectorXf  jointTraj, std::vector<float>& delta);

    /**
     * @brief This method computes the time step given between two consecutive postures
     * @param jointTraj
     * @param timeStep
     */
    void getTimeStep(MatrixXf jointTraj, float& timeStep);

    // -- file of the data -- //

    /**
     * @brief This method writes down the distance between the object and the hand
     * @param stream
     * @param dHO
     */
    void write_dHO(std::ofstream& stream, float dHO);

    /**
     * @brief This method writes down the dimensions of the body
     * @param hh
     * @param stream
     */
    void writeBodyDim(humanoidPtr hh,std::ofstream& stream);

    /**
     * @brief This method writes down the D-H parameters of the arms of the humanoid robot
     * @param hh
     * @param stream
     * @param k
     */
    void writeArmDHParams(humanoidPtr hh, std::ofstream& stream, int k);

    /**
     * @brief This method writes down the joint limits of the arm
     * @param stream
     * @param minArmLimits
     * @param maxArmLimits
     */
    void writeArmLimits(std::ofstream& stream, std::vector<float>& minArmLimits,std::vector<float>& maxArmLimits);

    /**
     * @brief This method writes down the initial posture of the arm
     * @param stream
     * @param initArmPosture
     */
    void writeArmInitPose(std::ofstream& stream,std::vector<float>& initArmPosture);

    /**
     * @brief This method writes down the final posture of the fingers
     * @param stream
     * @param finalHand
     */
    void writeFingerFinalPose(std::ofstream& stream,std::vector<float>& finalHand);

    /**
     * @brief This method writes down the lambda of the objective function
     * @param stream
     * @param lambda
     */
    void writeLambda(std::ofstream& stream,std::vector<float>& lambda);

    /**
     * @brief This method writes down the information about the objects that are obstacles of the movement
     * @param stream
     * @param objs
     */
    void writeInfoObjects(std::ofstream& stream,std::vector<objectPtr>& objs);

    /**
     * @brief This method writes down the information about the target of the movement
     * @param stream
     * @param tar
     */
    void writeInfoTarget(std::ofstream& stream, targetPtr tar);

    /**
     * @brief This method writes down the information about the object that has got the target
     * @param stream
     * @param obj
     */
    void writeInfoObjectTarget(std::ofstream& stream, objectPtr obj);

    // -- file of the model -- //

    /**
     * @brief This method writes down the value of PI
     * @param stream
     */
    void writePI(std::ofstream& stream);

    /**
     * @brief This method writes down the declaration of the body of the robot
     * @param stream
     */
    void writeBodyDimMod(std::ofstream& stream);

    /**
     * @brief This method writes down the declaration of the D-H parameters of the arm
     * @param stream
     */
    void writeArmDHParamsMod(std::ofstream& stream);

    /**
     * @brief This method writes down the declaration of the distance Object-Hand
     * @param stream
     */
    void write_dHOMod(std::ofstream& stream);

    /**
     * @brief This method writes down the declaration of the objects
     * @param stream
     */
    void writeInfoObjectsMod(std::ofstream& stream);

    /**
     * @brief This method writes down the rotation matrix of each obstacle
     * @param stream
     */
    void writeRotMatObsts(std::ofstream& stream);

    /**
     * @brief This method writes down the direct kinematics of the arm
     * @param stream
     * @param matWorldToArm
     * @param matHand
     * @param tolsArm
     * @param final
     */
    void writeArmDirKin(std::ofstream& stream, Matrix4f& matWorldToArm, Matrix4f& matHand, std::vector<float>& tolsArm,bool final);

    /**
     * @brief This method writes down the objective function of the problem
     * @param stream
     * @param final
     */
    void writeObjective(std::ofstream& stream, bool final);

    /**
     * @brief This method writes down the constraints with the body of the humanoid
     * @param stream
     * @param final
     */
    void writeBodyConstraints(std::ofstream& stream, bool final);

    /**
     * @brief This method writes down the constraints withe the table of the scenaio
     * @param stream
     * @param final
     * @param griptype
     * @param tols_table
     * @param steps
     */
    void writeTableConstraints(std::ofstream& stream, bool final,int griptype, std::vector<float>& tols_table,int steps);

#if HAND==0
    /**
     * @brief This method writes down the parameters of the human hand
     * @param hh
     * @param stream
     * @param k
     */
    void writeHumanHandParams(humanoidPtr hh, std::ofstream& stream, int k);    

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
#elif HAND==1
    /**
     * @brief This method writes down the parameters of the Barrett hand
     * @param hh
     * @param stream
     */
    void writeBarrettHandParams(humanoidPtr hh, std::ofstream& stream);

    /**
     * @brief This method writes down the declaration of the parameters of the Barrett hand
     * @param stream
     */
    void writeBarrettHandParamsMod(std::ofstream& stream);

    /**
     * @brief This method writes down the direct kinematics of the Barrett hand
     * @param stream
     * @param rk
     * @param jk
     * @param tolsHand
     * @param final
     * @param transport
     */
    void writeBarrettHandDirKin(std::ofstream& stream, std::vector<int>& rk,std::vector<int>& jk, MatrixXf& tolsHand, bool final, bool transport);
#endif



};


} // namespace HUMotion


#endif // PROBLEM_H
