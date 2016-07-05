#ifndef SOLUTION_H
#define SOLUTION_H


#include "movement.hpp"
#include "scenario.hpp"
#include "amplinterface.hpp"


namespace HUMotion{

typedef boost::shared_ptr<Movement> movementPtr;
typedef boost::shared_ptr<Scenario> scenarioPtr;


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


private:

    bool solved; //true if the problem has been solved
    bool part_of_task; // true if the problem is part of the task
    float dHOr; // distance between the right hand and the center of the object
    float dHOl; // distance between the left hand and the center of the object
    float dFF; // distance between the fingertip F3 and the fingers F1 and F2
    float dFH; // distance between the fingers and the palm of the hand
    std::vector<float> rightFinalPosture; // right final total posture [rad]
    std::vector<float> rightFinalHand; // right final hand posture [rad]
    std::vector<float> leftFinalPosture; // left final total posture [rad]
    std::vector<float> leftFinalHand; // left final hand posture [rad]
    std::vector<float> rightBouncePosture; // right bounce posture [rad]
    std::vector<float> leftBouncePosture; // left bounce posture [rad]
    std::vector<float> rightFinalPosture_diseng;
    std::vector<float> rightFinalPosture_eng;
    std::vector<float> leftFinalPosture_diseng;
    std::vector<float> leftFinalPosture_eng;
    MatrixXf optimalTraj; // human-like optimized trajectory
    Tols tolerances; // tolerances and parameters for the optimization problems
    movementPtr mov;
    scenarioPtr scene;
    int targetAxis; // approaching target mode: 0 = none , 1 = x, 2 = y , 3 = z
    objectPtr obj_curr; // current object
    targetPtr tar_eng; // target of the engaged object
    objectPtr obj_eng; // engaged object

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
                                std::vector<float> tols_table,int arm_code);

    bool writeFilesBouncePosture(int move_type,humanoidPtr hh, float dHO, int griptype, int steps, float totalTime,
                                 std::vector<float> minAuxLimits, std::vector<float> maxAuxLimits,
                                 std::vector<float> initAuxPosture,std::vector<float> finalAuxPosture, std::vector<float> finalHand,
                                 std::vector<float> initialGuess,std::vector<objectPtr> objs,
                                 targetPtr tar, objectPtr obj,
                                 boundaryConditions b, std::vector<float> tolsArm, MatrixXf tolsHand,
                                 std::vector< MatrixXf > tolsTarget, std::vector< MatrixXf > tolsObstacles,std::vector<float> lambda,
                                 std::vector<float> tols_table, int arm_code);


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
