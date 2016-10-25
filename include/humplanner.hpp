#ifndef HUMPLANNER_H
#define HUMPLANNER_H

#include "HUMLconfig.hpp"
#include "amplinterface.hpp"

using namespace std;

namespace HUMotion{


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
     * @brief This method solves the final posture selection problem of single arm reach-to-grasp movements
     * @return
     */
    bool singleArmFinalPostureReachToGrasp();



private:

    string name;/**< name of the planner */
    std::vector<object> obstacles; /**< obstacles in the scenario */
    object obj_tar; /**< object that has the target of the movement */

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
     * @brief This method writes down the dimensions of the body ofthe humanoid
     * @param h_xsize
     * @param h_ysize
     * @param stream
     */
    void writeBodyDim(double h_xsize,double h_ysize,std::ofstream& stream);

    /**
     * @brief This method writes down the D-H parameters of the arms of the humanoid robot
     * @param alpha
     * @param a
     * @param d
     * @param stream
     * @param k
     */
    void writeArmDHParams(std::vector<double> alpha,std::vector<double> a, std::vector<double> d,
                          std::ofstream& stream, int k);

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
     * @param index
     * @param ring
     * @param thumb
     * @param stream
     * @param k
     */
    void writeHumanHandParams(human_finger index, human_finger ring, human_finger thumb, std::ofstream& stream, int k);

    /**
     * @brief This method writes down the declaration of the parameters of the human hand
     * @param stream
     */
    //void writeHumanHandParamsMod(std::ofstream& stream);

    /**
     * @brief This method writes down the direct kinematics of the human hand
     * @param stream
     * @param tolsHand
     * @param final
     * @param transport
     */
    //void writeHumanHandDirKin(std::ofstream& stream,MatrixXf& tolsHand, bool final, bool transport);

    /**
     * @brief writeBarrettHandParams
     * Thi method writes down the parameters of the Barrett Hand
     * @param Aw
     * @param A1
     * @param A2
     * @param A3
     * @param D3
     * @param phi2
     * @param phi3
     * @param rk
     * @param jk
     * @param stream
     */
    void writeBarrettHandParams(double Aw, double A1, double A2, double A3,double D3, double phi2, double phi3,
                                std::vector<double> rk, std::vector<double> jk, std::ofstream& stream);

    /**
     * @brief This method writes down the declaration of the parameters of the Barrett hand
     * @param stream
     */
    //void writeBarrettHandParamsMod(std::ofstream& stream);

    /**
     * @brief This method writes down the direct kinematics of the Barrett hand
     * @param stream
     * @param rk
     * @param jk
     * @param tolsHand
     * @param final
     * @param transport
     */
    //void writeBarrettHandDirKin(std::ofstream& stream, std::vector<int>& rk,std::vector<int>& jk, MatrixXf& tolsHand, bool final, bool transport);

    /**
     * @brief writeInfoTarget
     * This method writes down the info of the target
     * @param stream
     * @param pos
     * @param xt
     * @param yt
     * @param zt
     */
    void writeInfoTarget(ofstream &stream, std::vector<double> pos, std::vector<double> xt,
                                  std::vector<double> yt, std::vector<double> zt);
};

} // namespace HUMotion

#endif // HUMPLANNER_H
