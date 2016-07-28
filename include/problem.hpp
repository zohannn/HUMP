#ifndef SOLUTION_H
#define SOLUTION_H


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
    // constructors
    Problem();
    Problem(Movement* mov, Scenario* scene);

    // copy constructor
    Problem(const Problem& s);

    //destructor
    ~Problem();


    // setters
    void setApproachingTargetAxis(int a);
    void setSolved(bool s); // set solved
    void setPartOfTask(bool p); // set part of the task
    bool solve(Tols probTols); // tolerances and parameters for the Posture selection problems


    // getters
    string getInfoLine();
    float getTrajectory(MatrixXf& traj);
    float getVelocity(MatrixXf& vel);
    movementPtr getMovement();
    void getFinalPosture(int arm_code, std::vector<float>& final_posture);
    void getBouncePosture(int arm_code, std::vector<float>& bounce_posture);
    objectPtr getObjectEngaged();
    bool getSolved();
    bool getPartOfTask();
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
     * <tr><td>Go home <td>25 <td> bounce posture go home not solved
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
    //targetPtr tar_eng; /**< target of the engaged object */
    objectPtr obj_eng; /**< engaged object */

    // hand
    bool finalPostureFingers(int hand_id);
    bool invKinHand(float d_obj,int hand_id,std::vector<float>& sols);

    // --- single arm --- //
    // Reach-to-grasp
    bool singleArmFinalPostureReachToGrasp();
    bool singleArmBouncePostureReachToGrasp();
    // DisEngage
    bool singleArmFinalPostureSubDisengage();
    // Engage
    bool singleArmFinalPostureEngage();
    bool singleArmFinalPostureSubEngage();
    bool singleArmBouncePostureEngage();
    // Go home
    bool singleArmBouncePostureGoHome();

    void getObstaclesSingleArm(std::vector<objectPtr> objs, std::vector<float> center, float radius, std::vector<objectPtr>& obsts);

    bool writeFilesFinalPosture(int mov_type,humanoidPtr hh,float dHO,int griptype,
                                std::vector<float> initArmPosture, std::vector<float> finalHand,
                                std::vector<float> initialGuess,
                                std::vector<objectPtr> objs,
                                targetPtr tar, objectPtr obj,
                                std::vector<float> tolsArm, MatrixXf tolsHand, MatrixXf tolsObstacles,
                                float tolTarPos, float tolTarOr,std::vector<float> lambda,
                                bool obstacle_avoidance,int arm_code);

    bool writeFilesBouncePosture(int move_type,humanoidPtr hh, float dHO, int griptype, int steps, float totalTime,
                                 std::vector<float> minAuxLimits, std::vector<float> maxAuxLimits,
                                 std::vector<float> initAuxPosture,std::vector<float> finalAuxPosture, std::vector<float> finalHand,
                                 std::vector<float> initialGuess,std::vector<objectPtr> objs,
                                 targetPtr tar, objectPtr obj,
                                 boundaryConditions b, std::vector<float> tolsArm, MatrixXf tolsHand,
                                 std::vector< MatrixXf > tolsTarget, std::vector< MatrixXf > tolsObstacles, std::vector<float> lambda,
                                 bool target_avoidance, bool obstacle_avoidance, int arm_code);


    bool amplRead(string& datFile, string& modFile, string& nlFile); // write the .nl file
    bool optimize(string& nlfile,std::vector<Number>& x_sol,
                      float tol, float acc_tol); // read the problem and solve it


    void computeTraj(const MatrixXf& dTraj, const MatrixXf& bTraj);
    void directMovement(std::vector<float> initPosture, std::vector<float> finalPosture, int steps, MatrixXf& Traj);
    void backForthMovement(std::vector<float> initPosture, std::vector<float> bouncePosture, int steps, MatrixXf& Traj);

    // numerical differentiation
    void getDelta(VectorXf  jointTraj, std::vector<float>& delta);
    // time step
    void getTimeStep(MatrixXf jointTraj, float& timeStep);

    // writing methods
    // data file
    void write_dHO(std::ofstream& stream, float dHO);
    void writeBodyDim(humanoidPtr hh,std::ofstream& stream);
    void writeArmDHParams(humanoidPtr hh, std::ofstream& stream, int k);
    void writeArmLimits(std::ofstream& stream, std::vector<float>& minArmLimits,std::vector<float>& maxArmLimits);
    void writeArmInitPose(std::ofstream& stream,std::vector<float>& initArmPosture);
    void writeFingerFinalPose(std::ofstream& stream,std::vector<float>& finalHand);
    void writeLambda(std::ofstream& stream,std::vector<float>& lambda);
    void writeInfoObjects(std::ofstream& stream,std::vector<objectPtr>& objs);
    void writeInfoTarget(std::ofstream& stream, targetPtr tar);
    void writeInfoObjectTarget(std::ofstream& stream, objectPtr obj);
    // model file
    void writePI(std::ofstream& stream);
    void writeBodyDimMod(std::ofstream& stream);
    void writeArmDHParamsMod(std::ofstream& stream);
    void write_dHOMod(std::ofstream& stream);
    void writeInfoObjectsMod(std::ofstream& stream);
    void writeRotMatObsts(std::ofstream& stream);
    void writeArmDirKin(std::ofstream& stream, Matrix4f& matWorldToArm, Matrix4f& matHand, std::vector<float>& tolsArm,bool final);
    void writeObjective(std::ofstream& stream, bool final);
    void writeBodyConstraints(std::ofstream& stream, bool final);
    void writeTableConstraints(std::ofstream& stream, bool final,int griptype, std::vector<float>& tols_table,int steps);
#if HAND==0
    void writeHumanHandParams(humanoidPtr hh, std::ofstream& stream, int k);
    void writeHumanHandParamsMod(std::ofstream& stream);
    void writeHumanHandDirKin(std::ofstream& stream,MatrixXf& tolsHand, bool final, bool transport);
#elif HAND==1
    void writeBarrettHandParams(humanoidPtr hh, std::ofstream& stream);
    void writeBarrettHandParamsMod(std::ofstream& stream);
    void writeBarrettHandDirKin(std::ofstream& stream, std::vector<int>& rk,std::vector<int>& jk, MatrixXf& tolsHand, bool final, bool transport);
#endif



};


} // namespace HUMotion


#endif // SOLUTION_H
