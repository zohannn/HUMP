#include "../include/problem.hpp"
#include "IpIpoptApplication.hpp"

using namespace Ipopt;

namespace HUMotion {



Problem::Problem():
    mov(nullptr),scene(nullptr)
{

    this->rightFinalPosture = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->rightFinalPosture_diseng = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->rightFinalPosture_eng = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->leftFinalPosture = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->leftFinalPosture_diseng = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->leftFinalPosture_eng = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->rightFinalHand = std::vector<float>(JOINTS_HAND);
    this->leftFinalHand = std::vector<float>(JOINTS_HAND);
    this->rightBouncePosture = std::vector<float>(JOINTS_ARM);
    this->leftBouncePosture = std::vector<float>(JOINTS_ARM);
    this->targetAxis = 0;
    this->solved=false;
    this->part_of_task=false;
    this->err_log=0;



}

Problem::Problem(Movement* mov, Scenario* scene)
{

    this->rightFinalPosture = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->rightFinalPosture_diseng = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->rightFinalPosture_eng = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->leftFinalPosture = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->leftFinalPosture_diseng = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->leftFinalPosture_eng = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->rightFinalHand = std::vector<float>(JOINTS_HAND);
    this->leftFinalHand = std::vector<float>(JOINTS_HAND);
    this->rightBouncePosture = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->leftBouncePosture = std::vector<float>(JOINTS_ARM+JOINTS_HAND);
    this->targetAxis = 0;
    this->solved=false;
    this->part_of_task=false;
    this->err_log=0;

    this->mov = movementPtr(mov);
    this->scene = scenarioPtr(scene);
}


Problem::Problem(const Problem& s)
{


    this->dFF = s.dFF;
    this->dFH = s.dFH;
    this->dHOl = s.dHOl;
    this->dHOr = s.dHOr;
    this->solved=s.solved;
    this->part_of_task=s.part_of_task;

    this->rightFinalPosture = s.rightFinalPosture;
    this->rightFinalPosture_diseng = s.rightFinalPosture_diseng;
    this->rightFinalPosture_eng = s.rightFinalPosture_eng;
    this->rightFinalHand = s.rightFinalHand;
    this->leftFinalPosture = s.leftFinalPosture;
    this->leftFinalPosture_eng = s.leftFinalPosture_eng;
    this->leftFinalPosture_diseng = s.leftFinalPosture_diseng;
    this->leftFinalHand = s.leftFinalHand;
    this->rightBouncePosture = s.rightBouncePosture;
    this->leftBouncePosture = s.leftBouncePosture;
    this->err_log=s.err_log;

    this->targetAxis = s.targetAxis;
    this->mov = movementPtr(new Movement(*s.mov.get()));
    this->scene = scenarioPtr(new Scenario(*s.scene.get()));
}


Problem::~Problem()
{

}

int Problem::getErrLog()
{
    return this->err_log;
}


bool Problem::getSolved()
{

    return this->solved;
}

bool Problem::getPartOfTask()
{

    return this->part_of_task;
}

string Problem::getInfoLine()
{

    return string("Planner: HUML, ")+string("Humanoid: ")+this->scene->getHumanoid()->getName()+string(",Movement: ")+this->mov->getInfoLine();
}


void Problem::getFinalPosture(int arm_code, std::vector<float> &final_posture)
{

#ifdef DEBUG
    ASSERT(arm_code < 3 && arm_code >=0);
#endif

    switch (arm_code) {

    case 0: // dual arm

        // TO DO

        break;

    case 1: // right arm

        final_posture = this->rightFinalPosture;

        break;

    case 2: // left arm

        final_posture = this->leftFinalPosture;

        break;
    }
}


void Problem::getBouncePosture(int arm_code, std::vector<float> &bounce_posture)
{

    switch (arm_code) {

    case 0: // dual arm

        // TO DO

        break;

    case 1: // right arm

        bounce_posture = this->rightBouncePosture;

        break;

    case 2: // left arm

        bounce_posture = this->leftBouncePosture;

        break;
    }
}

objectPtr Problem::getObjectEngaged()
{

    return this->obj_eng;
}

void Problem::setSolved(bool s)
{

    this->solved=s;
}

void Problem::setPartOfTask(bool p)
{

    this->part_of_task=p;
}


bool Problem::solve(Tols probTols)
{

    this->tolerances = probTols;
    int arm_code =  this->mov->getArm();

    switch (this->mov->getType()){

        case 0: // Reach-to-grasp

            // calculate the final posture of the fingers and
            // the distance dHO
            try{
                    this->finalPostureFingers(arm_code);
                        // get the trajectory for reach to grasp movements
                        switch (arm_code){

                        case 0: // both arms
                            //TO DO
                            break;

                        case 1: case 2: // right arm (1) , left arm (2)

                            try{
                            // --- Final Posture selection ---- //
                               bool FPosture = this->singleArmFinalPostureReachToGrasp();
                               if (FPosture){
                                   // --- Bounce Posture selection ---- //
                                   bool BPosture = this->singleArmBouncePostureReachToGrasp();
                                   if (BPosture){
                                       this->solved=true;
                                       this->err_log=0;
                                   }else{
                                       this->solved=false;
                                       this->err_log=20;
                                   }
                               }else{
                                   this->solved=false;
                                   this->err_log=10;
                               }

                            }catch (const string message){
                                    throw message;
                            }catch( ... ){
                                throw string ("HUML: error in optimizing the trajecory");
                            }
                            break;
                        }

            }catch(string str){
                throw str;
            }

            break;
        case 1: // Reaching

            break;

        case 2: // Transport

            break;

        case 3: // Engage

        try{


            switch(arm_code){

            case 0: // both arms

                // TO DO
                break;
            case 1: case 2: // right arm (1) , left arm (2)

                // --- Final Posture disengage selection ---- //
                this->finalPostureFingers(arm_code);

                bool FPostureSubDis = this->singleArmFinalPostureSubDisengage();
                if (FPostureSubDis){
                   // --- Final Posture selection ---- //
                   bool FPosture = this->singleArmFinalPostureEngage();
                   if (FPosture){
                       // --- Final Posture engage selection ---- //
                       bool FPostureSubEng = this->singleArmFinalPostureSubEngage();
                       if(FPostureSubEng){
                           // --- Bounce Posture selection ---- //
                           bool BPosture = this->singleArmBouncePostureEngage();
                           if (BPosture){
                                this->solved=true;
                                this->err_log=0;
                           }else{
                               this->solved=false;
                               this->err_log=23;
                           }
                       }else{
                           this->solved=false;
                           this->err_log=131;
                       }
                   }else{
                       this->solved=false;
                       this->err_log=13;
                   }
                }else{
                    this->solved=false;
                    this->err_log=130;
                }

                break;
            }

        }catch (const string message){
            throw message;
        }catch( const std::exception ex){
            throw string ("HUML: ")+ex.what();
        }
            break;

        case 4: // Disengage

            break;

        case 5:// Go park


        try{

        switch(arm_code){

        case 0: // both arms

            // TO DO
            break;
        case 1: case 2: // right arm (1) , left arm (2)

            this->solved = this->singleArmBouncePostureGoPark();
            if(this->solved){
                this->err_log=0;
            }else{
                this->err_log=25;
            }



            break;

        }



        }catch(const std::exception ex){

                throw string ("HUML: ")+ex.what();

        }

        break;

    }



    return this->solved;


}

bool Problem::singleArmBouncePostureGoPark()
{

    bool release_object;

    int arm_code = this->mov->getArm();
    int griptype = this->mov->getGrip();
    int mov_type = this->mov->getType();
    objectPtr obj = this->mov->getObject();
    release_object=std::strcmp(obj->getName().c_str(),"")!=0;
    humanoidPtr hh = this->scene->getHumanoid();
    std::vector<objectPtr> objs;
    this->scene->getObjects(objs);

    std::vector<float> initPosture;
    std::vector<float> finalPosture;
    std::vector<float> initialGuess;
    std::vector<float> minLimits;
    std::vector<float> maxLimits;
    std::vector<float> finalHand = std::vector<float>(JOINTS_HAND);
    targetPtr tar;
    float dHO;
    //float shPos;
    std::vector<float> shPos;


    //tolerances
    boundaryConditions b = this->tolerances.bounds;
    boundaryConditions bAux;
    int steps = this->tolerances.steps;
    float totalTime = 1.0;
    std::vector<float> tolsArm = this->tolerances.tolsArm;
    MatrixXf tolsHand = this->tolerances.tolsHand;
    std::vector< MatrixXf > tolsTarget = this->tolerances.singleArm_tolsTarget;
    std::vector< MatrixXf > tolsObstacles = this->tolerances.singleArm_tolsObstacles;
    std::vector<float> lambda = this->tolerances.lambda_bounce;
    //std::vector<float> tols_table = this->tolerances.tols_table;
    bool target_avoidance = this->tolerances.target_avoidance;
    bool obstacle_avoidance = this->tolerances.obstacle_avoidance;

    switch(arm_code){
    case 1: //right arm
        hh->getRightPosture(initPosture);
        hh->getRightHomePosture(finalPosture);
        hh->getRightMinLimits(minLimits);
        hh->getRightMaxLimits(maxLimits);
        //shPos = hh->getRightShoulderNorm();
        hh->getRightShoulderPos(shPos);
        dHO = this->dHOr;
        tar=obj->getTargetRight();
        break;
    case 2: // left arm
        hh->getLeftPosture(initPosture);
        hh->getLeftHomePosture(finalPosture);
        hh->getLeftMinLimits(minLimits);
        hh->getLeftMaxLimits(maxLimits);
        //shPos = hh->getLeftShoulderNorm();
        hh->getLeftShoulderPos(shPos);
        dHO = this->dHOl;
        tar=obj->getTargetLeft();
        break;
    }

    std::vector<float> initAuxPosture(initPosture.begin(),initPosture.begin()+JOINTS_ARM);
    if(release_object){
#if HAND==0
        initAuxPosture.push_back(0.0);
#endif
        initAuxPosture.push_back(0.0);
        initAuxPosture.push_back(0.0);
    }else{
#if HAND==0
        initAuxPosture.push_back(initPosture.at(7));
#endif
        initAuxPosture.push_back(initPosture.at(8));
        initAuxPosture.push_back(initPosture.at(10));
    }
    std::vector<float> finalAuxPosture(finalPosture.begin(),finalPosture.begin()+JOINTS_ARM);
#if HAND==0
    finalAuxPosture.push_back(finalPosture.at(7));
#endif
    finalAuxPosture.push_back(finalPosture.at(8));
    finalAuxPosture.push_back(finalPosture.at(10));
    std::vector<float> minAuxLimits(minLimits.begin(),minLimits.begin()+JOINTS_ARM);
#if HAND==0
    minAuxLimits.push_back(minLimits.at(7));
#endif
    minAuxLimits.push_back(minLimits.at(8));
    minAuxLimits.push_back(minLimits.at(10));
    std::vector<float> maxAuxLimits(maxLimits.begin(),maxLimits.begin()+JOINTS_ARM);
#if HAND==0
    maxAuxLimits.push_back(maxLimits.at(7));
#endif
    maxAuxLimits.push_back(maxLimits.at(8));
    maxAuxLimits.push_back(maxLimits.at(10));
    std::vector<float> lambdaAux(lambda.begin(),lambda.begin()+JOINTS_ARM);
#if HAND==0
    lambdaAux.push_back(lambda.at(7));
#endif
    lambdaAux.push_back(lambda.at(8));
    lambdaAux.push_back(lambda.at(10));
    std::vector<float> vel0Aux(b.vel_0.begin(),b.vel_0.begin()+JOINTS_ARM);
#if HAND==0
    vel0Aux.push_back(b.vel_0.at(7));
#endif
    vel0Aux.push_back(b.vel_0.at(8));
    vel0Aux.push_back(b.vel_0.at(10));
    std::vector<float> velfAux(b.vel_f.begin(),b.vel_f.begin()+JOINTS_ARM);
#if HAND==0
    velfAux.push_back(b.vel_f.at(7));
#endif
    velfAux.push_back(b.vel_f.at(8));
    velfAux.push_back(b.vel_f.at(10));
    std::vector<float> acc0Aux(b.acc_0.begin(),b.acc_0.begin()+JOINTS_ARM);
#if HAND==0
    acc0Aux.push_back(b.acc_0.at(7));
#endif
    acc0Aux.push_back(b.acc_0.at(8));
    acc0Aux.push_back(b.acc_0.at(10));
    std::vector<float> accfAux(b.acc_f.begin(),b.acc_f.begin()+JOINTS_ARM);
#if HAND==0
    accfAux.push_back(b.acc_f.at(7));
#endif
    accfAux.push_back(b.acc_f.at(8));
    accfAux.push_back(b.acc_f.at(10));
    bAux.vel_0=vel0Aux;
    bAux.vel_f=velfAux;
    bAux.acc_0=acc0Aux;
    bAux.acc_f=accfAux;

    // initial guess choice
    initialGuess = initAuxPosture;
/*
    for (int i = 0; i < initAuxPosture.size(); ++i ){
        initialGuess.push_back((initAuxPosture.at(i)+finalAuxPosture.at(i))/2);
    }
    */

    float Lu = hh->getArm().arm_specs.d.at(2);
    float Ll = hh->getArm().arm_specs.d.at(4);
    float Lh = hh->getArm().arm_specs.d.at(6);
    float max_ext = Lh+Ll+Lu;

    // get the obstacles of the workspace
    std::vector<objectPtr> obsts;
    this->getObstaclesSingleArm(objs, shPos, max_ext, obsts);


    bool written = this->writeFilesBouncePosture(mov_type,hh,dHO,griptype, steps, totalTime,
                                                  minAuxLimits,maxAuxLimits,
                                                  initAuxPosture, finalAuxPosture, finalHand,
                                                  initialGuess,obsts,
                                                  tar,obj,bAux,tolsArm,tolsHand,
                                                  tolsTarget, tolsObstacles,lambdaAux,target_avoidance,obstacle_avoidance,arm_code);


    //bool written = true;



    if(written){

        // call AMPL the produce the .nl file
        string fn = string("BouncePosture");
        bool nlwritten = this->amplRead(fn,fn,fn);

        if(nlwritten){
            // call ipopt for optimization
            string nlfile = string("Models/")+fn+string(".nl");
            std::vector<Number> x_sol = std::vector<Number>(JOINTS_ARM);

            try
            {
                float tol_stop = 1e-6;
                if (this->optimize(nlfile,x_sol,tol_stop,tol_stop)){


                    for (std::size_t i=0; i < x_sol.size()-2; ++i){

                       // printf("x[%i] = %f\n", i, x_sol[i]);
                        switch(arm_code){

                        case 1: //right arm

                            this->rightBouncePosture.at(i) = x_sol[i];
                            break;

                        case 2: // left arm

                            this->leftBouncePosture.at(i) = x_sol[i];
                            break;
                        }

                    }

                    switch(arm_code){

                    case 1: //right arm
#if HAND == 0
                        this->rightBouncePosture.at(7) = x_sol[7];
                        this->rightBouncePosture.at(8) = x_sol[8];
                        this->rightBouncePosture.at(9) = x_sol[8];
                        this->rightBouncePosture.at(10) = x_sol[9];
#elif HAND ==1
                        if (release_object){
                            this->rightBouncePosture.at(7) = 0.0;
                            this->rightBouncePosture.at(8) = 0.0;
                            this->rightBouncePosture.at(9) = 0.0;
                            this->rightBouncePosture.at(10) = 0.0;
                        }else{
                            this->rightBouncePosture.at(7) = 0.0;
                            this->rightBouncePosture.at(8) = x_sol[8];
                            this->rightBouncePosture.at(9) = x_sol[8];
                            this->rightBouncePosture.at(10) = x_sol[7];
                        }

#endif

                        break;

                    case 2: // left arm

#if HAND == 0
                        this->leftBouncePosture.at(7) = x_sol[7];
                        this->leftBouncePosture.at(8) = x_sol[8];
                        this->leftBouncePosture.at(9) = x_sol[8];
                        this->leftBouncePosture.at(10) = x_sol[9];
#elif HAND ==1

                        if (release_object){
                            this->leftBouncePosture.at(7) = 0.0;
                            this->leftBouncePosture.at(8) = 0.0;
                            this->leftBouncePosture.at(9) = 0.0;
                            this->leftBouncePosture.at(10) = 0.0;
                        }else{
                            this->leftBouncePosture.at(7) = 0.0;
                            this->leftBouncePosture.at(8) = x_sol[8];
                            this->leftBouncePosture.at(9) = x_sol[8];
                            this->leftBouncePosture.at(10) = x_sol[7];

                        }
#endif

                        break;
                    }


                    return true;
                }else{

                    return false;
                }

            }
            catch (const std::exception &exc)
            {

                throw string(exc.what());
            }

        }else{

            throw string("Error in writing the files for optimization");

        }



    }else{

        throw string("Error in writing the files for optimization");
    }






    return true;

}


bool Problem::singleArmFinalPostureSubDisengage()
{

    int mov_type = this->mov->getType();
    int arm_code = this->mov->getArm();
    int griptype = this->mov->getGrip();
    objectPtr obj = this->mov->getObject();
    //objectPtr obj_eng = this->mov->getObjectEng();
    humanoidPtr hh = this->scene->getHumanoid();
    std::vector<objectPtr> objs;
    this->scene->getObjects(objs);

    // tolerances
    std::vector<float> tolsArm = this->tolerances.tolsArm;
    MatrixXf tolsHand = this->tolerances.tolsHand;
    MatrixXf tolsObstacles = this->tolerances.final_tolsObstacles;
    float tolTarPos = this->tolerances.tolTarPos;
    float tolTarOr = this->tolerances.tolTarOr;
    std::vector<float> lambda = this->tolerances.lambda_final;
    //std::vector<float> tols_table = this->tolerances.tols_table;
    bool obstacle_avoidance = this->tolerances.obstacle_avoidance;
    float diseng_dist = this->tolerances.diseng_dist;
    int diseng_dir = this->tolerances.diseng_dir;


    std::vector<float> initPosture;
    std::vector<float> initialGuess;
    std::vector<float> finalHand;
    float dHO;
    //float shPos;
    std::vector<float> shPos;
    targetPtr tar;
    std::vector<float> pos_hand;
    Matrix3f Rot_hand;

    switch (arm_code){

    case 1: // right arm

        hh->getRightPosture(initPosture);
        hh->getRightHandOr(Rot_hand);
        hh->getRightHandPos(pos_hand);
        dHO = this->dHOr;

        //shPos = hh->getRightShoulderNorm();
        hh->getRightShoulderPos(shPos);
        hh->getRightHandPosture(finalHand);
        this->rightFinalHand=finalHand;

        if (obj->isTargetRightEnabled()){
            tar = obj->getTargetRight();
        }else{
            throw string("There is not target for the right arm");
        }

        break;

    case 2: // left arm

        hh->getLeftPosture(initPosture);
        hh->getLeftHandOr(Rot_hand);
        hh->getLeftHandPos(pos_hand);
        dHO = this->dHOl;

        //shPos = hh->getLeftShoulderNorm();
        hh->getLeftShoulderPos(shPos);
        hh->getLeftHandPosture(finalHand);
        this->leftFinalHand=finalHand;

        if (obj->isTargetLeftEnabled()){
            tar = obj->getTargetLeft();
        }else{
            throw string("There is no target for the left arm");
        }

        break;
    }

    std::vector<float> initArmPosture(initPosture.begin(),initPosture.begin()+JOINTS_ARM);
    //std::vector<float> initHandPosture(initPosture.begin()+JOINTS_ARM+1,initPosture.end()-1);
    //std::vector<float> minArmLimits(minLimits.begin(),minLimits.begin()+JOINTS_ARM);
    //std::vector<float> maxArmLimits(maxLimits.begin(),maxLimits.begin()+JOINTS_ARM);

    // initial guess choice
    initialGuess = initArmPosture; // arbitrary choice
    pos tar_pos = tar->getPos(); // [mm]
    pos obj_pos = obj->getPos();
    engagePtr eeng = obj->getEngagePoint();
    pos eng_pos = eeng->getPos();
    pos new_tar;
    pos new_obj;
    pos new_eng;


    switch(diseng_dir){

    case 0: // none

        new_tar.Xpos=tar_pos.Xpos;
        new_tar.Ypos=tar_pos.Ypos;
        new_tar.Zpos=tar_pos.Zpos;

        new_obj.Xpos=obj_pos.Xpos;
        new_obj.Ypos=obj_pos.Ypos;
        new_obj.Zpos=obj_pos.Zpos;

        new_eng.Xpos=eng_pos.Xpos;
        new_eng.Ypos=eng_pos.Ypos;
        new_eng.Zpos=eng_pos.Zpos;

        break;

    case 1: // x

        new_tar.Xpos=tar_pos.Xpos + diseng_dist;
        new_tar.Ypos=tar_pos.Ypos;
        new_tar.Zpos=tar_pos.Zpos;

        new_obj.Xpos=obj_pos.Xpos+ diseng_dist;
        new_obj.Ypos=obj_pos.Ypos;
        new_obj.Zpos=obj_pos.Zpos;

        new_eng.Xpos=eng_pos.Xpos+ diseng_dist;
        new_eng.Ypos=eng_pos.Ypos;
        new_eng.Zpos=eng_pos.Zpos;

        break;

    case 2: // y

        new_tar.Xpos=tar_pos.Xpos;
        new_tar.Ypos=tar_pos.Ypos + diseng_dist;
        new_tar.Zpos=tar_pos.Zpos;

        new_obj.Xpos=obj_pos.Xpos;
        new_obj.Ypos=obj_pos.Ypos+ diseng_dist;
        new_obj.Zpos=obj_pos.Zpos;

        new_eng.Xpos=eng_pos.Xpos;
        new_eng.Ypos=eng_pos.Ypos+ diseng_dist;
        new_eng.Zpos=eng_pos.Zpos;

        break;

    case 3: // z

        new_tar.Xpos=tar_pos.Xpos;
        new_tar.Ypos=tar_pos.Ypos;
        new_tar.Zpos=tar_pos.Zpos + diseng_dist;

        new_obj.Xpos=obj_pos.Xpos;
        new_obj.Ypos=obj_pos.Ypos;
        new_obj.Zpos=obj_pos.Zpos+ diseng_dist;

        new_eng.Xpos=eng_pos.Xpos;
        new_eng.Ypos=eng_pos.Ypos;
        new_eng.Zpos=eng_pos.Zpos+ diseng_dist;

        break;

    }

    targetPtr ttar=targetPtr(new Target(tar->getName(),new_tar,tar->getOr()));

    objectPtr oobj = objectPtr(new Object(obj->getName(),new_obj,obj->getOr(),obj->getSize(),
                                             new Target(tar->getName(),new_tar,tar->getOr()),
                                             new Target(tar->getName(),new_tar,tar->getOr()),
                                             new EngagePoint(eeng->getName(),new_eng,eeng->getOr())));

    switch(arm_code){

    case 0:
        break;

    case 1:
        oobj->setTargetRightEnabled(true);
        oobj->setTargetLeftEnabled(false);
        break;

    case 2:
        oobj->setTargetRightEnabled(false);
        oobj->setTargetLeftEnabled(true);

        break;

    }

    this->obj_curr = oobj;

    float Lu = hh->getArm().arm_specs.d.at(2);
    float Ll = hh->getArm().arm_specs.d.at(4);
    float Lh = hh->getArm().arm_specs.d.at(6);
    float max_ext = Lh+Ll+Lu;

    // get the obstacles of the workspace
    std::vector<objectPtr> obsts;
    this->getObstaclesSingleArm(objs, shPos, max_ext, obsts);

    bool written = this->writeFilesFinalPosture(mov_type, hh, dHO, griptype,
                                 initArmPosture, finalHand,initialGuess,
                                 objs,ttar, oobj,
                                 tolsArm, tolsHand, tolsObstacles,
                                 tolTarPos, tolTarOr,lambda, obstacle_avoidance,arm_code);

    if (written){

        // call AMPL the produce the .nl file
        string fn = string("FinalPosture");
        bool nlwritten = this->amplRead(fn,fn,fn);

        if(nlwritten){
            // call ipopt for optimization
            string nlfile = string("Models/")+fn+string(".nl");
            std::vector<Number> x_sol = std::vector<Number>(JOINTS_ARM);

            try
            {


                float tol_stop = 1e-2;
                std::vector<float> x_sol_deg = std::vector<float>(7);
                if (this->optimize(nlfile,x_sol,tol_stop,tol_stop)){


                    for (std::size_t i=0; i < x_sol.size(); ++i){

                       // printf("x[%i] = %f\n", i, x_sol[i]);
                        switch(arm_code){

                        case 1: //right arm

                            this->rightFinalPosture_diseng.at(i) = x_sol[i];
                            x_sol_deg.at(i) = x_sol[i]*180/M_PI;
                            break;

                        case 2: // left arm

                            this->leftFinalPosture_diseng.at(i) = x_sol[i];
                            break;
                        }


                    }
                    switch (arm_code) {
                    case 0: // both arms

                        break;

                    case 1: // right arm

                        for (int i =0; i <JOINTS_HAND; ++i){
                            this->rightFinalPosture_diseng.at(JOINTS_ARM+i)=finalHand.at(i);
                        }


                        break;

                    case 2: // left arm

                        for (int i =0; i <JOINTS_HAND; ++i){
                            this->leftFinalPosture_diseng.at(JOINTS_ARM+i)=finalHand.at(i);
                        }

                        break;
                    }

                    return true;
                }else{

                    return false;
                }

            }
            catch (const std::exception &exc)
            {

                throw string(exc.what());
            }

        }else{

            throw string("Error in writing the files for optimization");

        }



    }else{

        throw string("Error in writing the files for optimization");
    }

}


bool Problem::singleArmFinalPostureSubEngage()
{

    int mov_type = this->mov->getType();
    int arm_code = this->mov->getArm();
    int griptype = this->mov->getGrip();
    objectPtr obj = this->obj_eng; // the engaged object
    //objectPtr obj_eng = this->mov->getObjectEng();
    humanoidPtr hh = this->scene->getHumanoid();
    std::vector<objectPtr> objs;
    this->scene->getObjects(objs);

    // tolerances
    std::vector<float> tolsArm = this->tolerances.tolsArm;
    MatrixXf tolsHand = this->tolerances.tolsHand;
    MatrixXf tolsObstacles = this->tolerances.final_tolsObstacles;
    float tolTarPos = this->tolerances.tolTarPos;
    float tolTarOr = this->tolerances.tolTarOr;
    std::vector<float> lambda = this->tolerances.lambda_final;
    //std::vector<float> tols_table = this->tolerances.tols_table;
    bool obstacle_avoidance = this->tolerances.obstacle_avoidance;
    float eng_dist = this->tolerances.eng_dist;
    int eng_dir = this->tolerances.eng_dir;


    std::vector<float> initPosture;
    std::vector<float> initialGuess;
    std::vector<float> finalHand;
    float dHO;
    //float shPos;
    std::vector<float> shPos;
    targetPtr tar;
    std::vector<float> pos_hand;
    Matrix3f Rot_hand;

    switch (arm_code){

    case 1: // right arm

        dHO = this->dHOr;
        initPosture=this->rightFinalPosture;
        hh->getRightHandOr(Rot_hand);
        hh->getRightHandPos(pos_hand);
        //shPos = hh->getRightShoulderNorm();
        hh->getRightShoulderPos(shPos);
        hh->getRightHandPosture(finalHand);
        this->rightFinalHand=finalHand;

        if (obj->isTargetRightEnabled()){
            tar = obj->getTargetRight();
        }else{
            throw string("There is not target for the right arm");
        }

        break;

    case 2: // left arm


        dHO = this->dHOl;
        initPosture=this->leftFinalPosture;
        hh->getLeftHandOr(Rot_hand);
        hh->getLeftHandPos(pos_hand);
        //shPos = hh->getRightShoulderNorm();
        hh->getLeftShoulderPos(shPos);
        hh->getLeftHandPosture(finalHand);
        this->leftFinalHand=finalHand;


        if (obj->isTargetLeftEnabled()){
            tar = obj->getTargetLeft();
        }else{
            throw string("There is no target for the left arm");
        }

        break;
    }

    std::vector<float> initArmPosture(initPosture.begin(),initPosture.begin()+JOINTS_ARM);
    //std::vector<float> initHandPosture(initPosture.begin()+JOINTS_ARM+1,initPosture.end()-1);
    //std::vector<float> minArmLimits(minLimits.begin(),minLimits.begin()+JOINTS_ARM);
    //std::vector<float> maxArmLimits(maxLimits.begin(),maxLimits.begin()+JOINTS_ARM);

    // initial guess choice
    initialGuess = initArmPosture; // arbitrary choice

    pos tar_pos = tar->getPos(); // [mm]
    pos obj_pos = obj->getPos();
    engagePtr eeng = obj->getEngagePoint();
    pos eng_pos = eeng->getPos();
    pos new_tar;
    pos new_obj;
    pos new_eng;


    switch(eng_dir){

    case 0: // none

        new_tar.Xpos=tar_pos.Xpos;
        new_tar.Ypos=tar_pos.Ypos;
        new_tar.Zpos=tar_pos.Zpos;

        new_obj.Xpos=obj_pos.Xpos;
        new_obj.Ypos=obj_pos.Ypos;
        new_obj.Zpos=obj_pos.Zpos;

        new_eng.Xpos=eng_pos.Xpos;
        new_eng.Ypos=eng_pos.Ypos;
        new_eng.Zpos=eng_pos.Zpos;

        break;

    case 1: // x

        new_tar.Xpos=tar_pos.Xpos + eng_dist;
        new_tar.Ypos=tar_pos.Ypos;
        new_tar.Zpos=tar_pos.Zpos;

        new_obj.Xpos=obj_pos.Xpos+ eng_dist;
        new_obj.Ypos=obj_pos.Ypos;
        new_obj.Zpos=obj_pos.Zpos;

        new_eng.Xpos=eng_pos.Xpos+ eng_dist;
        new_eng.Ypos=eng_pos.Ypos;
        new_eng.Zpos=eng_pos.Zpos;

        break;

    case 2: // y

        new_tar.Xpos=tar_pos.Xpos;
        new_tar.Ypos=tar_pos.Ypos + eng_dist;
        new_tar.Zpos=tar_pos.Zpos;

        new_obj.Xpos=obj_pos.Xpos;
        new_obj.Ypos=obj_pos.Ypos+ eng_dist;
        new_obj.Zpos=obj_pos.Zpos;

        new_eng.Xpos=eng_pos.Xpos;
        new_eng.Ypos=eng_pos.Ypos+ eng_dist;
        new_eng.Zpos=eng_pos.Zpos;

        break;

    case 3: // z

        new_tar.Xpos=tar_pos.Xpos;
        new_tar.Ypos=tar_pos.Ypos;
        new_tar.Zpos=tar_pos.Zpos + eng_dist;

        new_obj.Xpos=obj_pos.Xpos;
        new_obj.Ypos=obj_pos.Ypos;
        new_obj.Zpos=obj_pos.Zpos+ eng_dist;

        new_eng.Xpos=eng_pos.Xpos;
        new_eng.Ypos=eng_pos.Ypos;
        new_eng.Zpos=eng_pos.Zpos+ eng_dist;

        break;

    }

    targetPtr ttar=targetPtr(new Target(tar->getName(),new_tar,tar->getOr()));

    objectPtr oobj = objectPtr(new Object(obj->getName(),new_obj,obj->getOr(),obj->getSize(),
                                             new Target(tar->getName(),new_tar,tar->getOr()),
                                             new Target(tar->getName(),new_tar,tar->getOr()),
                                             new EngagePoint(eeng->getName(),new_eng,eeng->getOr())));

    switch(arm_code){

    case 0:
        break;

    case 1:
        oobj->setTargetRightEnabled(true);
        oobj->setTargetLeftEnabled(false);
        break;

    case 2:
        oobj->setTargetRightEnabled(false);
        oobj->setTargetLeftEnabled(true);

        break;

    }

    this->obj_curr = oobj;
    //this->getMovement()->setObject(oobj);

    float Lu = hh->getArm().arm_specs.d.at(2);
    float Ll = hh->getArm().arm_specs.d.at(4);
    float Lh = hh->getArm().arm_specs.d.at(6);
    float max_ext = Lh+Ll+Lu;
    // get the obstacles of the workspace
    std::vector<objectPtr> obsts;
    this->getObstaclesSingleArm(objs, shPos, max_ext, obsts);



    bool written = this->writeFilesFinalPosture(mov_type, hh, dHO, griptype,
                                 initArmPosture, finalHand,initialGuess,
                                 objs,ttar, oobj,
                                 tolsArm, tolsHand, tolsObstacles,
                                 tolTarPos, tolTarOr,lambda, obstacle_avoidance,arm_code);

    if (written){

        // call AMPL the produce the .nl file
        string fn = string("FinalPosture");
        bool nlwritten = this->amplRead(fn,fn,fn);

        if(nlwritten){
            // call ipopt for optimization
            string nlfile = string("Models/")+fn+string(".nl");
            std::vector<Number> x_sol = std::vector<Number>(JOINTS_ARM);

            try
            {


                float tol_stop = 1e-2;
                std::vector<float> x_sol_deg = std::vector<float>(7);
                if (this->optimize(nlfile,x_sol,tol_stop,tol_stop)){


                    for (std::size_t i=0; i < x_sol.size(); ++i){

                       // printf("x[%i] = %f\n", i, x_sol[i]);
                        switch(arm_code){

                        case 1: //right arm

                            this->rightFinalPosture_eng.at(i) = x_sol[i];
                            x_sol_deg.at(i) = x_sol[i]*180/M_PI;
                            break;

                        case 2: // left arm

                            this->leftFinalPosture_eng.at(i) = x_sol[i];
                            break;
                        }


                    }
                    switch (arm_code) {
                    case 0: // both arms

                        break;

                    case 1: // right arm

                        for (int i =0; i <JOINTS_HAND; ++i){
                            this->rightFinalPosture_eng.at(JOINTS_ARM+i)=finalHand.at(i);
                        }


                        break;

                    case 2: // left arm

                        for (int i =0; i <JOINTS_HAND; ++i){
                            this->leftFinalPosture_eng.at(JOINTS_ARM+i)=finalHand.at(i);
                        }

                        break;
                    }
                    return true;
                }else{

                    return false;
                }

            }
            catch (const std::exception &exc)
            {

                throw string(exc.what());
            }

        }else{

            throw string("Error in writing the files for optimization");

        }



    }else{

        throw string("Error in writing the files for optimization");
    }

}

bool Problem::singleArmFinalPostureReachToGrasp()
{


    int mov_type = this->mov->getType();
    int arm_code = this->mov->getArm();
    int griptype = this->mov->getGrip();
    objectPtr obj = this->mov->getObject();
    humanoidPtr hh = this->scene->getHumanoid();
    std::vector<objectPtr> objs;
    this->scene->getObjects(objs);

    // tolerances
    std::vector<float> tolsArm = this->tolerances.tolsArm;
    MatrixXf tolsHand = this->tolerances.tolsHand;
    MatrixXf tolsObstacles = this->tolerances.final_tolsObstacles;
    float tolTarPos = this->tolerances.tolTarPos;
    float tolTarOr = this->tolerances.tolTarOr;
    std::vector<float> lambda = this->tolerances.lambda_final;
    //std::vector<float> tols_table = this->tolerances.tols_table;
    bool obstacle_avoidance = this->tolerances.obstacle_avoidance;


    std::vector<float> initPosture;
    std::vector<float> initialGuess;
    std::vector<float> finalHand;
    float dHO;
    //float shPos;
    std::vector<float> shPos;
    targetPtr tar;


    switch (arm_code){

    case 1: // right arm

        hh->getRightPosture(initPosture);
        dHO = this->dHOr;
        //shPos = hh->getRightShoulderNorm();
        hh->getRightShoulderPos(shPos);
        finalHand = this->rightFinalHand;

        if (obj->isTargetRightEnabled()){
            tar = obj->getTargetRight();
        }else{
            throw string("There is not target for the right arm");
        }

        break;

    case 2: // left arm

        hh->getLeftPosture(initPosture);
        dHO = this->dHOl;
        //shPos = hh->getLeftShoulderNorm();
        hh->getLeftShoulderPos(shPos);
        finalHand = this->leftFinalHand;

        if (obj->isTargetLeftEnabled()){
            tar = obj->getTargetLeft();
        }else{
            throw string("There is no target for the left arm");
        }

        break;
    }

    std::vector<float> initArmPosture(initPosture.begin(),initPosture.begin()+JOINTS_ARM);
    //std::vector<float> initHandPosture(initPosture.begin()+JOINTS_ARM+1,initPosture.end()-1);
    //std::vector<float> minArmLimits(minLimits.begin(),minLimits.begin()+JOINTS_ARM);
    //std::vector<float> maxArmLimits(maxLimits.begin(),maxLimits.begin()+JOINTS_ARM);

    // initial guess choice
    initialGuess = initArmPosture; // arbitrary choice

    /*
    // joint expense factors choise
    for (int i=0; i < this->joint_expFactors.size(); ++i ){

        this->joint_expFactors.at(i) = 1.0;
    }
    std::vector<float> joint_armExpFactors(joint_expFactors.begin(),joint_expFactors.begin()+JOINTS_ARM);
    */

    float Lu = hh->getArm().arm_specs.d.at(2);
    float Ll = hh->getArm().arm_specs.d.at(4);
    float Lh = hh->getArm().arm_specs.d.at(6);
    float max_ext = Lh+Ll+Lu;
    //if (abs(obj->getNorm()-shPos) >= (max_ext)){
    if(sqrt(pow(obj->getPos().Xpos -shPos.at(0),2)+
            pow(obj->getPos().Ypos -shPos.at(1),2)+
            pow(obj->getPos().Zpos -shPos.at(2),2))>= max_ext){
        throw string("The object to grasp is too far away");
    }

    // get the obstacles of the workspace
    std::vector<objectPtr> obsts;
    this->getObstaclesSingleArm(objs, shPos, max_ext, obsts);

    bool written = this->writeFilesFinalPosture(mov_type, hh, dHO, griptype,
                                 initArmPosture, finalHand,initialGuess,
                                 obsts,tar, obj,
                                 tolsArm, tolsHand, tolsObstacles,
                                 tolTarPos, tolTarOr,lambda, obstacle_avoidance,arm_code);

    //bool written = true;

    if (written){

        // call AMPL the produce the .nl file
        string fn = string("FinalPosture");
        bool nlwritten = this->amplRead(fn,fn,fn);

        if(nlwritten){
            // call ipopt for optimization
            string nlfile = string("Models/")+fn+string(".nl");
            std::vector<Number> x_sol = std::vector<Number>(JOINTS_ARM);

            try
            {


                float tol_stop = 1e-2;
                std::vector<float> x_sol_deg = std::vector<float>(7);
                if (this->optimize(nlfile,x_sol,tol_stop,tol_stop)){


                    for (std::size_t i=0; i < x_sol.size(); ++i){

                       // printf("x[%i] = %f\n", i, x_sol[i]);
                        switch(arm_code){

                        case 1: //right arm
                            this->rightFinalPosture.at(i) = x_sol[i];
                            x_sol_deg.at(i) = x_sol[i]*180/M_PI;
                            break;
                        case 2: // left arm
                            this->leftFinalPosture.at(i) = x_sol[i];
                            break;
                        }


                    }



                    return true;
                }else{

                    return false;
                }

            }
            catch (const std::exception &exc)
            {

                throw string(exc.what());
            }

        }else{

            throw string("Error in writing the files for optimization");

        }



    }else{

        throw string("Error in writing the files for optimization");
    }




}

bool Problem::singleArmBouncePostureReachToGrasp()
{


    int arm_code = this->mov->getArm();
    int griptype = this->mov->getGrip();
    int mov_type = this->mov->getType();
    objectPtr obj = this->mov->getObject();
    humanoidPtr hh = this->scene->getHumanoid();
    std::vector<objectPtr> objs;
    this->scene->getObjects(objs);

    std::vector<float> initPosture;
    std::vector<float> finalPosture;
    std::vector<float> initialGuess;
    std::vector<float> minLimits;
    std::vector<float> maxLimits;
    std::vector<float> finalHand;
    targetPtr tar;
    float dHO;
    std::vector<float> shPos;

    //tolerances
    boundaryConditions b = this->tolerances.bounds;
    boundaryConditions bAux;
    int steps = this->tolerances.steps;
    float totalTime = 1.0;
    std::vector<float> tolsArm = this->tolerances.tolsArm;
    MatrixXf tolsHand = this->tolerances.tolsHand;
    std::vector< MatrixXf > tolsTarget = this->tolerances.singleArm_tolsTarget;
    std::vector< MatrixXf > tolsObstacles = this->tolerances.singleArm_tolsObstacles;
    std::vector<float> lambda = this->tolerances.lambda_bounce;
    //std::vector<float> tols_table = this->tolerances.tols_table;
    bool target_avoidance = this->tolerances.target_avoidance;
    bool obstacle_avoidance = this->tolerances.obstacle_avoidance;

    switch(arm_code){

    case 1: // right arm

        hh->getRightPosture(initPosture);
        finalPosture = this->rightFinalPosture;
        hh->getRightMinLimits(minLimits);
        hh->getRightMaxLimits(maxLimits);
        hh->getRightShoulderPos(shPos);
        finalHand = this->rightFinalHand;
        dHO = this->dHOr;

        if (obj->isTargetRightEnabled()){
            tar = obj->getTargetRight();
        }else{throw string("There is not target for the right arm");}
        break;

    case 2: // left arm

        hh->getLeftPosture(initPosture);
        finalPosture = this->leftFinalPosture;
        hh->getLeftMinLimits(minLimits);
        hh->getLeftMaxLimits(maxLimits);
        finalHand = this->leftFinalHand;
        hh->getLeftShoulderPos(shPos);
        dHO = this->dHOl;

        if (obj->isTargetLeftEnabled()){
            tar = obj->getTargetLeft();
        }else{throw string("There is no target for the left arm");}
        break;
    }

    std::vector<float> initAuxPosture(initPosture.begin(),initPosture.begin()+JOINTS_ARM);
#if HAND==0
    initAuxPosture.push_back(initPosture.at(7));
#endif
    initAuxPosture.push_back(initPosture.at(8));
    initAuxPosture.push_back(initPosture.at(10));
    std::vector<float> finalAuxPosture(finalPosture.begin(),finalPosture.begin()+JOINTS_ARM);
#if HAND==0
    finalAuxPosture.push_back(finalPosture.at(7));
#endif
    finalAuxPosture.push_back(finalPosture.at(8));
    finalAuxPosture.push_back(finalPosture.at(10));
    std::vector<float> minAuxLimits(minLimits.begin(),minLimits.begin()+JOINTS_ARM);
#if HAND==0
    minAuxLimits.push_back(minLimits.at(7));
#endif
    minAuxLimits.push_back(minLimits.at(8));
    minAuxLimits.push_back(minLimits.at(10));
    std::vector<float> maxAuxLimits(maxLimits.begin(),maxLimits.begin()+JOINTS_ARM);
#if HAND==0
    maxAuxLimits.push_back(maxLimits.at(7));
#endif
    maxAuxLimits.push_back(maxLimits.at(8));
    maxAuxLimits.push_back(maxLimits.at(10));
    std::vector<float> lambdaAux(lambda.begin(),lambda.begin()+JOINTS_ARM);
#if HAND==0
    lambdaAux.push_back(lambda.at(7));
#endif
    lambdaAux.push_back(lambda.at(8));
    lambdaAux.push_back(lambda.at(10));
    std::vector<float> vel0Aux(b.vel_0.begin(),b.vel_0.begin()+JOINTS_ARM);
#if HAND==0
    vel0Aux.push_back(b.vel_0.at(7));
#endif
    vel0Aux.push_back(b.vel_0.at(8));
    vel0Aux.push_back(b.vel_0.at(10));
    std::vector<float> velfAux(b.vel_f.begin(),b.vel_f.begin()+JOINTS_ARM);
#if HAND==0
    velfAux.push_back(b.vel_f.at(7));
#endif
    velfAux.push_back(b.vel_f.at(8));
    velfAux.push_back(b.vel_f.at(10));
    std::vector<float> acc0Aux(b.acc_0.begin(),b.acc_0.begin()+JOINTS_ARM);
#if HAND==0
    acc0Aux.push_back(b.acc_0.at(7));
#endif
    acc0Aux.push_back(b.acc_0.at(8));
    acc0Aux.push_back(b.acc_0.at(10));
    std::vector<float> accfAux(b.acc_f.begin(),b.acc_f.begin()+JOINTS_ARM);
#if HAND==0
    accfAux.push_back(b.acc_f.at(7));
#endif
    accfAux.push_back(b.acc_f.at(8));
    accfAux.push_back(b.acc_f.at(10));
    bAux.vel_0=vel0Aux;
    bAux.vel_f=velfAux;
    bAux.acc_0=acc0Aux;
    bAux.acc_f=accfAux;

    // initial guess choice
    initialGuess = initAuxPosture;
    /*
    for (int i = 0; i < initAuxPosture.size(); ++i ){
        initialGuess.push_back((initAuxPosture.at(i)+finalAuxPosture.at(i))/2);
    }
    */

    float Lu = hh->getArm().arm_specs.d.at(2);
    float Ll = hh->getArm().arm_specs.d.at(4);
    float Lh = hh->getArm().arm_specs.d.at(6);
    float max_ext = Lh+Ll+Lu;

    // get the obstacles of the workspace
    std::vector<objectPtr> obsts;
    this->getObstaclesSingleArm(objs, shPos, max_ext, obsts);



    bool written = this->writeFilesBouncePosture(mov_type,hh,dHO,griptype, steps, totalTime,
                                                  minAuxLimits,maxAuxLimits,
                                                  initAuxPosture, finalAuxPosture, finalHand,
                                                  initialGuess, obsts,
                                                  tar,obj,bAux,tolsArm,tolsHand,
                                                  tolsTarget, tolsObstacles,lambdaAux,target_avoidance,obstacle_avoidance,arm_code);


    if (written){

        // call AMPL the produce the .nl file
        string fn = string("BouncePosture");
        bool nlwritten = this->amplRead(fn,fn,fn);

        if(nlwritten){
            // call ipopt for optimization
            string nlfile = string("Models/")+fn+string(".nl");
            std::vector<Number> x_sol = std::vector<Number>(JOINTS_ARM);

            try
            {


                float tol_stop = 1e-6;
                if (this->optimize(nlfile,x_sol,tol_stop,tol_stop)){


                    for (std::size_t i=0; i < x_sol.size()-2; ++i){

                       // printf("x[%i] = %f\n", i, x_sol[i]);
                        switch(arm_code){

                        case 1: //right arm

                            this->rightBouncePosture.at(i) = x_sol[i];
                            break;

                        case 2: // left arm

                            this->leftBouncePosture.at(i) = x_sol[i];
                            break;
                        }

                    }

                    switch(arm_code){

                    case 1: //right arm
#if HAND == 0
                        this->rightBouncePosture.at(7) = x_sol[7];
                        this->rightBouncePosture.at(8) = x_sol[8];
                        this->rightBouncePosture.at(9) = x_sol[8];
                        this->rightBouncePosture.at(10) = x_sol[9];
#elif HAND ==1
                        this->rightBouncePosture.at(7) = 0.0;
                        this->rightBouncePosture.at(8) = x_sol[8];
                        this->rightBouncePosture.at(9) = x_sol[8];
                        this->rightBouncePosture.at(10) = x_sol[7];

#endif

                        break;

                    case 2: // left arm
#if HAND == 0
                        this->leftBouncePosture.at(7) = x_sol[7];
                        this->leftBouncePosture.at(8) = x_sol[8];
                        this->leftBouncePosture.at(9) = x_sol[8];
                        this->leftBouncePosture.at(10) = x_sol[9];
#elif HAND ==1
                        this->leftBouncePosture.at(7) = 0.0;
                        this->leftBouncePosture.at(8) = x_sol[8];
                        this->leftBouncePosture.at(9) = x_sol[8];
                        this->leftBouncePosture.at(10) = x_sol[7];
#endif

                        break;
                    }


                    return true;
                }else{

                    return false;
                }

            }
            catch (const std::exception &exc)
            {

                throw string(exc.what());
            }

        }else{

            throw string("Error in writing the files for optimization");

        }



    }else{

        throw string("Error in writing the files for optimization");
    }

}

bool Problem::singleArmFinalPostureEngage()
{

    int mov_type = this->mov->getType();
    int arm_code = this->mov->getArm();
    int griptype = this->mov->getGrip();
    //objectPtr obj = this->mov->getObject();
    objectPtr obj = this->obj_curr;
    objectPtr obj_eng = this->mov->getObjectEng();
    humanoidPtr hh = this->scene->getHumanoid();
    std::vector<objectPtr> objs;
    this->scene->getObjects(objs);
    float tol_tar_x=this->tolerances.eng_tols.at(0);
    float tol_tar_y=this->tolerances.eng_tols.at(1);
    float tol_tar_z=this->tolerances.eng_tols.at(2);

    // tolerances
    std::vector<float> tolsArm = this->tolerances.tolsArm;
    MatrixXf tolsHand = this->tolerances.tolsHand;
    MatrixXf tolsObstacles = this->tolerances.final_tolsObstacles;
    float tolTarPos = this->tolerances.tolTarPos;
    float tolTarOr = this->tolerances.tolTarOr;
    std::vector<float> lambda = this->tolerances.lambda_final;
    //std::vector<float> tols_table = this->tolerances.tols_table;
    bool obstacle_avoidance = this->tolerances.obstacle_avoidance;

    std::vector<float> initPosture;
    std::vector<float> initialGuess;
    std::vector<float> finalHand;
    float dHO;
    //float shPos;
    std::vector<float> shPos;
    targetPtr tar;
    engagePtr eng;
    engagePtr eng1;
    std::vector<float> pos_hand;
    Matrix3f Rot_hand;

    switch (arm_code){

    case 1: // right arm

        dHO = this->dHOr;
        initPosture = this->rightFinalPosture_diseng;
        hh->getRightHandOr(Rot_hand);
        hh->getRightHandPos(pos_hand);

        //shPos = hh->getRightShoulderNorm();
        hh->getRightShoulderPos(shPos);
        hh->getRightHandPosture(finalHand);
        this->rightFinalHand=finalHand;

        if (obj->isTargetRightEnabled()){
            tar = obj->getTargetRight();
        }else{
            throw string("There is not target for the right arm");
        }

        break;

    case 2: // left arm

        dHO = this->dHOl;
        initPosture=this->leftFinalPosture_diseng;
        hh->getLeftHandOr(Rot_hand);
        hh->getLeftHandPos(pos_hand);
        //shPos = hh->getLeftShoulderNorm();
        hh->getLeftShoulderPos(shPos);
        hh->getLeftHandPosture(finalHand);
        this->leftFinalHand=finalHand;

        if (obj->isTargetLeftEnabled()){

            tar = obj->getTargetLeft();

        }else{

            throw string("There is no target for the left arm");
        }

        break;
    }

    std::vector<float> initArmPosture(initPosture.begin(),initPosture.begin()+JOINTS_ARM);
    //std::vector<float> initHandPosture(initPosture.begin()+JOINTS_ARM+1,initPosture.end()-1);
    //std::vector<float> minArmLimits(minLimits.begin(),minLimits.begin()+JOINTS_ARM);
    //std::vector<float> maxArmLimits(maxLimits.begin(),maxLimits.begin()+JOINTS_ARM);

    // initial guess choice
    initialGuess = initArmPosture; // arbitrary choice

    // check the feasibility of the problem
    // P^W_e = o^W_e + R^W_tar*P^tar_e
    eng = obj->getEngagePoint();
    eng1 = obj_eng->getEngagePoint();
    // compute the position of the engage point relative to the target frame
    pos tar_pos = tar->getPos(); // [mm]
    pos eng_pos = eng->getPos(); // [mm]
    Matrix3f Rot_tar;
    tar->RPY_matrix(Rot_tar);
    Matrix3f Rot_tar_inv = Rot_tar.inverse();
    //InvertMatrix(Rot,Rotinv);
    Vector3f diff;
    diff(0) = eng_pos.Xpos - tar_pos.Xpos;
    diff(1) = eng_pos.Ypos - tar_pos.Ypos;
    diff(2) = eng_pos.Zpos - tar_pos.Zpos;
    Vector3f eng_to_tar;
    eng_to_tar = Rot_tar_inv * diff; //[mm]

    // compute the position of the target when the object will be engaged
    pos eng1_pos = eng1->getPos(); // [mm] position of the engage point of the other object
    eng1_pos.Xpos+=tol_tar_x;
    eng1_pos.Ypos+=tol_tar_y;
    eng1_pos.Zpos+=tol_tar_z;

    pos new_tar;
    new_tar.Xpos=eng1_pos.Xpos - eng_to_tar(0);
    new_tar.Ypos=eng1_pos.Ypos - eng_to_tar(1);
    new_tar.Zpos=eng1_pos.Zpos - eng_to_tar(2);

    //float new_tar_norm = sqrt(pow((new_tar.Xpos),2)+pow((new_tar.Ypos),2)+pow((new_tar.Zpos),2));

    float Lu = hh->getArm().arm_specs.d.at(2);
    float Ll = hh->getArm().arm_specs.d.at(4);
    float Lh = hh->getArm().arm_specs.d.at(6);
    float max_ext = Lh+Ll+Lu;
    //if (abs(obj->getNorm()-shPos) >= (max_ext)){
    if(sqrt(pow(obj->getPos().Xpos -shPos.at(0),2)+
            pow(obj->getPos().Ypos -shPos.at(1),2)+
            pow(obj->getPos().Zpos -shPos.at(2),2))>= max_ext){
        throw string("The object to grasp is too far away");
    }

    switch (arm_code) {

    case 0: // dual arm
        // TO DO
        break;

    case 1: // right arm
        this->dHOr= dHO;
        break;

    case 2: // left arm
        this->dHOl= dHO;
        break;
    }

    // the new target has the position computed in relation
    // with the engage point pf the second object.
    // Its orientation is going to be equal to the orientation of the second
    // engage point
    this->tar_eng = targetPtr(new  Target(tar->getName(),new_tar,eng1->getOr()));
    // compute the new object info according to the relation with
    // the object involved in the movement and its target/ engage point
    pos obj_pos = obj->getPos();
    Matrix3f Rot_obj;
    obj->RPY_matrix(Rot_obj);
    Matrix3f Rot_obj_inv = Rot_obj.inverse();
    //InvertMatrix(Rot,Rotinv);
    diff(0) = eng_pos.Xpos - obj_pos.Xpos;
    diff(1) = eng_pos.Ypos - obj_pos.Ypos;
    diff(2) = eng_pos.Zpos - obj_pos.Zpos;
    Vector3f eng_to_obj;
    eng_to_obj = Rot_obj_inv * diff; //[mm]
    // compute the position of the object when it will be engaged

    pos new_obj;
    new_obj.Xpos=eng1_pos.Xpos - eng_to_obj(0);
    new_obj.Ypos=eng1_pos.Ypos - eng_to_obj(1);
    new_obj.Zpos=eng1_pos.Zpos - eng_to_obj(2);

    this->obj_eng = objectPtr(new Object(obj->getName(),new_obj,eng1->getOr(),obj->getSize(),
                                          new Target(tar->getName(),new_tar,eng1->getOr()),
                                          new Target(tar->getName(),new_tar,eng1->getOr()),
                                          new EngagePoint(eng->getName(),eng1_pos,eng1->getOr())));


    switch(arm_code){

    case 0:
        break;

    case 1:
        this->obj_eng->setTargetRightEnabled(true);
        this->obj_eng->setTargetLeftEnabled(false);
        break;

    case 2:
        this->obj_eng->setTargetRightEnabled(false);
        this->obj_eng->setTargetLeftEnabled(true);

        break;

    }

    // get the obstacles of the workspace
    std::vector<objectPtr> obsts;
    this->getObstaclesSingleArm(objs, shPos, max_ext, obsts);



    // evaluate the possibility of providing diff1 at the following method
    bool written = this->writeFilesFinalPosture(mov_type,hh, dHO, griptype,
                                 initArmPosture, finalHand,initialGuess,
                                 obsts,tar_eng, this->obj_eng,
                                 tolsArm, tolsHand, tolsObstacles,
                                 tolTarPos, tolTarOr,lambda, obstacle_avoidance,arm_code);

    if (written){

        // call AMPL the produce the .nl file
        string fn = string("FinalPosture");
        bool nlwritten = this->amplRead(fn,fn,fn);

        if(nlwritten){
            // call ipopt for optimization
            string nlfile = string("Models/")+fn+string(".nl");
            std::vector<Number> x_sol = std::vector<Number>(JOINTS_ARM);

            try
            {


                float tol_stop = 1e-2;
                std::vector<float> x_sol_deg = std::vector<float>(JOINTS_ARM);
                if (this->optimize(nlfile,x_sol,tol_stop,tol_stop)){


                    for (std::size_t i=0; i < x_sol.size(); ++i){

                       // printf("x[%i] = %f\n", i, x_sol[i]);
                        switch(arm_code){

                        case 1: //right arm

                            this->rightFinalPosture.at(i) = x_sol[i];
                            x_sol_deg.at(i) = x_sol[i]*180/M_PI;
                            break;

                        case 2: // left arm

                            this->leftFinalPosture.at(i) = x_sol[i];
                            break;
                        }


                    }
                    switch (arm_code) {
                    case 0: // both arms

                        break;

                    case 1: // right arm

                        for (int i =0; i <JOINTS_HAND; ++i){
                            this->rightFinalPosture.at(JOINTS_ARM+i)=finalHand.at(i);
                        }


                        break;

                    case 2: // left arm

                        for (int i =0; i <JOINTS_HAND; ++i){
                            this->leftFinalPosture.at(JOINTS_ARM+i)=finalHand.at(i);
                        }

                        break;
                    }


                    return true;
                }else{

                    return false;
                }

            }
            catch (const std::exception &exc)
            {

                throw string(exc.what());
            }



        }else{

            throw string("Error in writing the files for optimization");

        }




    }else{

        throw string("Error in writing the files for optimization");
    }




}

bool Problem::singleArmBouncePostureEngage()
{

    int arm_code = this->mov->getArm();
    int griptype = this->mov->getGrip();
    int mov_type = this->mov->getType();
    objectPtr obj = this->obj_curr;
    humanoidPtr hh = this->scene->getHumanoid();
    std::vector<objectPtr> objs;
    this->scene->getObjects(objs);

    std::vector<float> initPosture;
    std::vector<float> finalPosture;
    std::vector<float> initialGuess;
    std::vector<float> minLimits;
    std::vector<float> maxLimits;
    std::vector<float> finalHand;
    targetPtr tar;
    float dHO;
    //float shPos;
    std::vector<float> shPos;

    //tolerances
    boundaryConditions b = this->tolerances.bounds;
    boundaryConditions bAux;
    int steps = this->tolerances.steps;
    float totalTime = 1.0;
    std::vector<float> tolsArm = this->tolerances.tolsArm;
    MatrixXf tolsHand = this->tolerances.tolsHand;
    std::vector< MatrixXf > tolsTarget = this->tolerances.singleArm_tolsTarget;
    std::vector< MatrixXf > tolsObstacles = this->tolerances.singleArm_tolsObstacles;
    std::vector<float> lambda = this->tolerances.lambda_bounce;
    //std::vector<float> tols_table = this->tolerances.tols_table;
    bool target_avoidance = this->tolerances.target_avoidance;
    bool obstacle_avoidance = this->tolerances.obstacle_avoidance;

    switch(arm_code){

    case 1: // right arm

        initPosture=this->rightFinalPosture_diseng;
        finalPosture = this->rightFinalPosture_eng;
        hh->getRightMinLimits(minLimits);
        hh->getRightMaxLimits(maxLimits);
        hh->getRightHandPosture(finalHand);
        //shPos = hh->getRightShoulderNorm();
        hh->getRightShoulderPos(shPos);
        dHO = this->dHOr;

        if (obj->isTargetRightEnabled()){
            tar = obj->getTargetRight();
        }else{
            throw string("There is not target for the right arm");
        }


        break;

    case 2: // left arm

        initPosture=this->leftFinalPosture_diseng;
        finalPosture = this->leftFinalPosture_eng;
        hh->getLeftMinLimits(minLimits);
        hh->getLeftMaxLimits(maxLimits);
        hh->getLeftHandPosture(finalHand);
        //shPos = hh->getLeftShoulderNorm();
        hh->getLeftShoulderPos(shPos);
        dHO = this->dHOl;

        if (obj->isTargetLeftEnabled()){
            tar = obj->getTargetLeft();
        }else{
            throw string("There is no target for the left arm");
        }


        break;
    }

    std::vector<float> initAuxPosture(initPosture.begin(),initPosture.begin()+JOINTS_ARM);
    std::vector<float> finalAuxPosture(finalPosture.begin(),finalPosture.begin()+JOINTS_ARM);
    std::vector<float> minAuxLimits(minLimits.begin(),minLimits.begin()+JOINTS_ARM);
    std::vector<float> maxAuxLimits(maxLimits.begin(),maxLimits.begin()+JOINTS_ARM);
    std::vector<float> lambdaAux(lambda.begin(),lambda.begin()+JOINTS_ARM);
    std::vector<float> vel0Aux(b.vel_0.begin(),b.vel_0.begin()+JOINTS_ARM);
    std::vector<float> velfAux(b.vel_f.begin(),b.vel_f.begin()+JOINTS_ARM);
    std::vector<float> acc0Aux(b.acc_0.begin(),b.acc_0.begin()+JOINTS_ARM);
    std::vector<float> accfAux(b.acc_f.begin(),b.acc_f.begin()+JOINTS_ARM);
    bAux.vel_0=vel0Aux;
    bAux.vel_f=velfAux;
    bAux.acc_0=acc0Aux;
    bAux.acc_f=accfAux;

    // initial guess choice
    initialGuess = initAuxPosture;
    /*
    for (int i = 0; i < initAuxPosture.size(); ++i ){
        initialGuess.push_back((initAuxPosture.at(i)+finalAuxPosture.at(i))/2);
    }
    */

    float Lu = hh->getArm().arm_specs.d.at(2);
    float Ll = hh->getArm().arm_specs.d.at(4);
    float Lh = hh->getArm().arm_specs.d.at(6);
    float max_ext = Lh+Ll+Lu;

    // get the obstacles of the workspace
    std::vector<objectPtr> obsts;
    this->getObstaclesSingleArm(objs, shPos, max_ext, obsts);

    bool written = this->writeFilesBouncePosture(mov_type,hh,dHO,griptype, steps, totalTime,
                                                  minAuxLimits,maxAuxLimits,
                                                  initAuxPosture, finalAuxPosture, finalHand,
                                                  initialGuess,obsts,
                                                  this->tar_eng,this->obj_eng,bAux,tolsArm,tolsHand,
                                                  tolsTarget, tolsObstacles,lambdaAux,
                                                  target_avoidance,obstacle_avoidance,arm_code);

    if (written){

        // call AMPL the produce the .nl file
        string fn = string("BouncePosture");
        bool nlwritten = this->amplRead(fn,fn,fn);

        if(nlwritten){
            // call ipopt for optimization
            string nlfile = string("Models/")+fn+string(".nl");
            std::vector<Number> x_sol = std::vector<Number>(JOINTS_ARM);
            std::vector<float> x_sol_deg = std::vector<float>(JOINTS_ARM);
            float tol_stop = 1e-6;

            try
            {

                if (this->optimize(nlfile,x_sol,tol_stop,tol_stop)){

                    for (std::size_t i=0; i < x_sol.size(); ++i){

                       //printf("x[%i] = %f\n", i, x_sol[i]);
                        switch(arm_code){

                        case 1: //right arm

                            this->rightBouncePosture.at(i) = x_sol[i];
                            x_sol_deg.at(i) = x_sol[i]*180/M_PI;
                            break;

                        case 2: // left arm

                            this->leftBouncePosture.at(i) = x_sol[i];
                            break;
                        }

                    }

                    switch(arm_code){

                    case 1: //right arm

                        for (int i =0; i <JOINTS_HAND; ++i){
                            this->rightBouncePosture.at(JOINTS_ARM+i)=finalHand.at(i);
                        }

                        break;

                    case 2: // left arm

                        for (int i =0; i <JOINTS_HAND; ++i){
                            this->leftBouncePosture.at(JOINTS_ARM+i)=finalHand.at(i);
                        }

                        break;
                    }





                    return true;
                }else{

                    return false;
                }

            }
            catch (const std::exception &exc)
            {

                throw string(exc.what());
            }

        }else{

            throw string("Error in writing the files for optimization");

        }



    }else{

        throw string("Error in writing the files for optimization");
    }


}


bool Problem::writeFilesBouncePosture(int mov_type, humanoidPtr hh,float dHO, int griptype, int steps, float totalTime,
                                      std::vector<float> minAuxLimits, std::vector<float> maxAuxLimits,
                                      std::vector<float> initAuxPosture, std::vector<float> finalAuxPosture, std::vector<float> finalHand,
                                      std::vector<float> initialGuess, std::vector<objectPtr> objs,
                                      targetPtr tar, objectPtr obj,
                                      boundaryConditions b, std::vector<float> tolsArm, MatrixXf tolsHand,
                                      std::vector< MatrixXf > tolsTarget, std::vector< MatrixXf > tolsObstacles,
                                      std::vector<float> lambda,bool target_avoidance, bool obstacle_avoidance,int arm_code)
{



    //  --- create the "Models" directory if it does not exist ---
    struct stat st = {0};
    if (stat("Models", &st) == -1) {
        mkdir("Models", 0700);
    }
    string path("Models/");

    Matrix4f matWorldToArm;
    Matrix4f matHand;

    int k;
    switch(arm_code){
    case 0: // dual arm
        //TO DO
        break;

    case 1: // right arm
        k=1;
        hh->getMatRight(matWorldToArm);
        hh->getMatRightHand(matHand);
        break;

    case 2: // left arm
        k=-1;
        hh->getMatLeft(matWorldToArm);
        hh->getMatLeftHand(matHand);
        break;
    }

   //------------------------- Write the dat file --------------------------------------------------
    string filename("BouncePosture.dat");
    ofstream PostureDat;
    // open the file
    PostureDat.open(path+filename);

    PostureDat << string("# BOUNCE POSTURE DATA FILE \n");
    PostureDat << string("# Units of measure: [rad], [mm] \n\n");

    PostureDat << string("data; \n");

    // number of steps
    PostureDat << string("param Nsteps :=")+to_string(steps)+string(";\n");

    // total time
    string time =  boost::str(boost::format("%.2f") % (totalTime));
    boost::replace_all(time,",",".");
    PostureDat << string("param TotalTime :=")+time+string(";\n");


    // Body dimension
    this->writeBodyDim(hh,PostureDat);
    // D-H Parameters of the Arm
    this->writeArmDHParams(hh,PostureDat,k);
    // distance between the hand and the object
    this->write_dHO(PostureDat,dHO);

    // joint limits
    this->writeArmLimits(PostureDat,minAuxLimits,maxAuxLimits);

    // initial pose of the arm
    this->writeArmInitPose(PostureDat,initAuxPosture);


    // final pose of the arm
    PostureDat << string("# FINAL POSE \n");
    PostureDat << string("param thet_final := \n");

    for (std::size_t i=0; i < finalAuxPosture.size(); ++i){
        string finalAuxstr =  boost::str(boost::format("%.2f") % (finalAuxPosture.at(i)));
        boost::replace_all(finalAuxstr,",",".");
        if (i == finalAuxPosture.size()-1){
            PostureDat << to_string(i+1)+string(" ")+finalAuxstr+string(";\n");
        }else{
            PostureDat << to_string(i+1)+string(" ")+finalAuxstr+string("\n");
        }
    }

    // final posture of the fingers
    this->writeFingerFinalPose(PostureDat,finalHand);

    // joint expense factors of the arm
    this->writeLambda(PostureDat,lambda);

    // initial guess
    PostureDat << string("# INITIAL GUESS \n");
    PostureDat << string("var theta_b := \n");

    for (std::size_t i=0; i < initialGuess.size(); ++i){
        string guess =  boost::str(boost::format("%.2f") % (initialGuess.at(i)));
        boost::replace_all(guess,",",".");
        if (i == initialGuess.size()-1){
            PostureDat << to_string(i+1)+string(" ")+guess+string(";\n");
        }else{
            PostureDat << to_string(i+1)+string(" ")+guess+string("\n");
        }
    }

    // boundary conditions initial velocity
    PostureDat << string("# INITIAL VELOCITY \n");
    PostureDat << string("param vel_0 := \n");

    for (std::size_t i=0; i < b.vel_0.size(); ++i){
        string vel_0 =  boost::str(boost::format("%.2f") % (b.vel_0.at(i)));
        boost::replace_all(vel_0,",",".");
        if (i == b.vel_0.size()-1){
            PostureDat << to_string(i+1)+string(" ")+vel_0+string(";\n");
        }else{
            PostureDat << to_string(i+1)+string(" ")+vel_0+string("\n");
        }


    }

    // boundary conditions final velocity
    PostureDat << string("# FINAL VELOCITY \n");
    PostureDat << string("param vel_f := \n");

    for (std::size_t i=0; i < b.vel_f.size(); ++i){
        string vel_f =  boost::str(boost::format("%.2f") % (b.vel_f.at(i)));
        boost::replace_all(vel_f,",",".");
        if (i == b.vel_f.size()-1){
            PostureDat << to_string(i+1)+string(" ")+vel_f+string(";\n");
        }else{
            PostureDat << to_string(i+1)+string(" ")+vel_f+string("\n");
        }


    }

    // boundary conditions initial acceleration
    PostureDat << string("# INITIAL ACCELERATION \n");
    PostureDat << string("param acc_0 := \n");

    for (std::size_t i=0; i < b.acc_0.size(); ++i){
        string acc_0 =  boost::str(boost::format("%.2f") % (b.acc_0.at(i)));
        boost::replace_all(acc_0,",",".");
        if (i == b.acc_0.size()-1){
            PostureDat << to_string(i+1)+string(" ")+acc_0+string(";\n");
        }else{
            PostureDat << to_string(i+1)+string(" ")+acc_0+string("\n");
        }
    }

    // boundary conditions final acceleration
    PostureDat << string("# FINAL ACCELERATION \n");
    PostureDat << string("param acc_f := \n");

    for (std::size_t i=0; i < b.acc_f.size(); ++i){
        string acc_f =  boost::str(boost::format("%.2f") % (b.acc_f.at(i)));
        boost::replace_all(acc_f,",",".");

        if (i == b.acc_f.size()-1){
            PostureDat << to_string(i+1)+string(" ")+acc_f+string(";\n");
        }else{
            PostureDat << to_string(i+1)+string(" ")+acc_f+string("\n");
        }
    }


    // Parameters of the Fingers


#if HAND == 0
    this->writeHumanHandParams(hh,PostureDat,k);
#elif HAND == 1
    this->writeBarrettHandParams(hh,PostureDat);
#endif

    // info of the target to reach
    this->writeInfoTarget(PostureDat,tar);

    // info objects
    this->writeInfoObjects(PostureDat,objs);
    // object that has the target
    this->writeInfoObjectTarget(PostureDat,obj);

    //close the file
    PostureDat.close();

    // ------------- Write the mod file ------------------------- //

    string filenamemod("BouncePosture.mod");
    ofstream PostureMod;
    // open the file
    PostureMod.open(path+filenamemod);

    PostureMod << string("# BOUNCE POSTURE MODEL FILE \n");
    PostureMod << string("# Movement to plan: \n");
    PostureMod << string("# ")+this->mov->getInfoLine()+string("\n\n");

    PostureMod << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
    PostureMod << string("# PARAMETERS \n\n");

    this->writePI(PostureMod);
    this->writeBodyDimMod(PostureMod);
    this->writeArmDHParamsMod(PostureMod);
    this->write_dHOMod(PostureMod);

    PostureMod << string("# Joint Limits \n");
    PostureMod << string("param llim {i in 1..")+to_string(minAuxLimits.size())+string("} ; \n");
    PostureMod << string("param ulim {i in 1..")+to_string(maxAuxLimits.size())+string("} ; \n");

    PostureMod << string("# Initial posture \n");
    PostureMod << string("param thet_init {i in 1..")+to_string(initAuxPosture.size())+string("} ; \n");


    PostureMod << string("# Final posture \n");
    PostureMod << string("param thet_final {i in 1..")+to_string(finalAuxPosture.size())+string("} ; \n");

    PostureMod << string("# Final finger posture \n");
    PostureMod << string("param joint_fingers {i in 1..")+to_string(JOINTS_HAND)+string("} ; \n");

    PostureMod << string("# Joint Expense Factors \n");
    PostureMod << string("param lambda {i in 1..")+to_string(lambda.size())+string("} ; \n");


#if HAND==0
    this->writeHumanHandParamsMod(PostureMod);
#elif HAND==1
    this->writeBarrettHandParamsMod(PostureMod);
#endif

    // info objects
    this->writeInfoObjectsMod(PostureMod);

    PostureMod << string("# Boundary Conditions \n");
    PostureMod << string("param vel_0 {i in 1..")+to_string(b.vel_0.size())+string("} ; \n");
    PostureMod << string("param vel_f {i in 1..")+to_string(b.vel_f.size())+string("} ; \n");
    PostureMod << string("param acc_0 {i in 1..")+to_string(b.acc_0.size())+string("} ; \n");
    PostureMod << string("param acc_f {i in 1..")+to_string(b.acc_f.size())+string("} ; \n");

    PostureMod << string("# Time and iterations\n");
    PostureMod << string("param Nsteps;\n");
    PostureMod << string("param TotalTime;\n");
    PostureMod << string("set Iterations := 1..(Nsteps+1);\n");
    PostureMod << string("set nJoints := 1..")+to_string(initialGuess.size())+string(";\n");
    PostureMod << string("set Iterations_nJoints := Iterations cross nJoints;\n");
    PostureMod << string("param time {i in Iterations} = (i-1)/Nsteps;\n");

    PostureMod << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
    PostureMod << string("# DECISION VARIABLES \n");
    PostureMod << string("# Bounce Posture \n");
    PostureMod << string("var theta_b {i in 1..")+to_string(initialGuess.size())+string("} >= llim[i], <= ulim[i]; \n");

    PostureMod << string("# Direct Movement \n");
    PostureMod << string("param the_direct {(i,j) in Iterations_nJoints} := \n");
    PostureMod << string("( thet_final[j] - thet_init[j] ) * (10*(time[i])^3 -15*(time[i])^4 +6*(time[i])^5) \n");
    PostureMod << string("+ \n");
    PostureMod << string("vel_0[j] *TotalTime* (time[i] - 6 *(time[i])^3 +8*(time[i])^4 -3*(time[i])^5) \n");
    PostureMod << string("+ \n");
    PostureMod << string("vel_f[j] *TotalTime* (- 4 *(time[i])^3 +7*(time[i])^4 -3*(time[i])^5) \n");
    PostureMod << string("+ \n");
    PostureMod << string("acc_0[j]/2 *TotalTime^2* (time[i]^2 - 3 *(time[i])^3 +3*(time[i])^4 -(time[i])^5) \n");
    PostureMod << string("+ \n");
    PostureMod << string("acc_f[j]/2 *TotalTime^2* ((time[i])^3 -2*(time[i])^4 +(time[i])^5); \n");

    PostureMod << string("# Back and forth Movement \n");
    PostureMod << string("var the_bf 	{(i,j) in Iterations_nJoints} =  \n");
    PostureMod << string("(theta_b[j] - thet_init[j])*(sin(pi*time[i]))^2; \n");

    PostureMod << string("# Composite Movement \n");
    PostureMod << string("var theta 	{(i,j) in Iterations_nJoints} = \n");
    PostureMod << string("thet_init[j] + 	the_direct[i,j] + the_bf[i,j];\n");

    // Rotation matrix of the obstacles
    this->writeRotMatObsts(PostureMod);
    // Direct Kinematics of the arm
    this->writeArmDirKin(PostureMod,matWorldToArm,matHand,tolsArm,false);

   string obj_radius =  boost::str(boost::format("%.2f") % obj->getRadius()); boost::replace_all(obj_radius,",",".");
   string obj_size_z =  boost::str(boost::format("%.2f") %  obj->getSize().Zsize); boost::replace_all(obj_size_z,",",".");

   bool engage = false; // true when there is an object to engage
   bool transport = false; // true when there is an object to transport
   bool gopark = false; // true when the movement is Go park
   bool release_object = false; // true when Go park and release an object

   switch (mov_type) {
   case 0: // reach to grasp

       break;
   case 1: // reaching

       break;
   case 2: // transport
       transport = true;
       break;
   case 3: // engage

       engage = true;

       PostureMod << string("var Obj2Transp {j in 1..4, i in Iterations} = if j<3 then Hand[j,i] + dFH * z_H[j,i] \n");
       PostureMod << string("else 	if (j=4) then ")+obj_radius+string("; \n");

       switch (griptype) {

       case 111: case 211: case 112: case 212: // Side thumb right, Side thumb left


           PostureMod << string("var Obj2Transp_1 {j in 1..4, i in Iterations} = if j<3 then Obj2Transp[j,i] + x_H[j,i] *(")+obj_size_z+string("/2)\n");
           PostureMod << string("else 	if (j=4) then ")+obj_radius+string("; \n");

           PostureMod << string("var Obj2Transp_2 {j in 1..4, i in Iterations} = if j<3 then Obj2Transp[j,i] - x_H[j,i] *(")+obj_size_z+string("/2)\n");
           PostureMod << string("else 	if (j=4) then ")+obj_radius+string("; \n");

           break;

       case 113: case 213: case 114: case 214: // Side thumb up, Side thumb down

           // TO DO
           break;

       case 121: case 221: case 122: case 222: // Above, Below

           // TO DO
           break;
       }




       break;

   case 4: // disengage

       break;

   case 5: // Go park
       gopark=true;
       release_object=std::strcmp(mov->getObject()->getName().c_str(),"")!=0;

       break;


   }

#if HAND == 0
   this->writeHumanHandDirKin(PostureMod,tolsHand,false,engage || transport);
#elif HAND == 1
   std::vector<int> rk; std::vector<int> jk;
   rk.push_back(-1); rk.push_back(1); rk.push_back(0);
   jk.push_back(-1); jk.push_back(-1); jk.push_back(1);
   this->writeBarrettHandDirKin(PostureMod,rk,jk,tolsHand,false,engage || transport);
#endif

   // Points of the arm
   if (engage){
    PostureMod << string("var Points_Arm {j in 1..18, i in 1..4,k in Iterations} = \n");
   }else{
    PostureMod << string("var Points_Arm {j in 1..15, i in 1..4,k in Iterations} = \n");
   }
   PostureMod << string("if ( j=1 ) then 	(Shoulder[i,k]+Elbow[i,k])/2  \n");
   PostureMod << string("else	if ( j=2 ) then 	Elbow[i,k] \n");
   PostureMod << string("else    if ( j=3 ) then 	(Wrist[i,k]+Elbow[i,k])/2  \n");
   PostureMod << string("else	if ( j=4 ) then 	Wrist[i,k] \n");
   PostureMod << string("else	if ( j=5 ) then 	Wrist[i,k]+0.45*(Hand[i,k]-Wrist[i,k]) \n");
   PostureMod << string("else	if ( j=6 ) then 	Wrist[i,k]+0.75*(Hand[i,k]-Wrist[i,k]) \n");
   PostureMod << string("else	if ( j=7 ) then 	Finger1_1[i,k] \n");
   PostureMod << string("else	if ( j=8 ) then 	Finger2_1[i,k] \n");
   PostureMod << string("else	if ( j=9 ) then 	Finger3_1[i,k]\n");
   PostureMod << string("#else	if ( j=10 ) then 	(Finger1_1[i,k]+Finger1_2[i,k])/2 \n");
   PostureMod << string("#else	if ( j=11 ) then 	(Finger2_1[i,k]+Finger2_2[i,k])/2 \n");
   PostureMod << string("#else	if ( j=12 ) then 	(Finger3_1[i,k]+Finger3_2[i,k])/2 \n");
   PostureMod << string("else	if ( j=10 ) then 	 Finger1_2[i,k] \n");
   PostureMod << string("else	if ( j=11 ) then 	 Finger2_2[i,k] \n");
   PostureMod << string("else	if ( j=12 ) then 	 Finger3_2[i,k] \n");
   PostureMod << string("#else	if ( j=16 ) then 	(Finger1_2[i,k]+Finger1_tip[i,k])/2	 \n");
   PostureMod << string("#else	if ( j=17 ) then 	(Finger2_2[i,k]+Finger2_tip[i,k])/2 \n");
   PostureMod << string("#else	if ( j=18 ) then 	(Finger3_2[i,k]+Finger3_tip[i,k])/2 \n");
   PostureMod << string("else	if ( j=13 ) then 	Finger1_tip[i,k]\n");
   PostureMod << string("else	if ( j=14 ) then 	Finger2_tip[i,k] \n");
   PostureMod << string("else	if ( j=15 ) then 	Finger3_tip[i,k] \n");

   if (engage){
       PostureMod << string("else    if ( j=16 ) then 	Obj2Transp[i,k] \n");
       PostureMod << string("else    if ( j=17 ) then 	Obj2Transp_1[i,k] \n");
       PostureMod << string("else    if ( j=18 ) then 	Obj2Transp_2[i,k] \n");
   }
   PostureMod << string("; \n\n");


   // objective function
   this->writeObjective(PostureMod,false);

   // constraints
   PostureMod << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
   PostureMod << string("#  \n");
   PostureMod << string("#		     Constraints                  # \n");
   PostureMod << string("#  \n");
   PostureMod << string("# joint limits for all the trajectory \n");
   PostureMod << string("subject to co_JointLimits {i in Iterations, j in nJoints}: llim[j] <= theta[i,j]  <= ulim[j]; \n\n");


   // hand constraints for approaching direction setting
   switch (griptype) {

   case 111: case 211: // side thumb left

       switch (mov_type) {

       case 0: // reach-to-grasp

           switch (this->targetAxis) {

           case 0: // none

               break;

           case 1: // x target

               PostureMod << string("# Hand approach orientation\n");
               PostureMod << string("subject to constr_hand_or {k in (Nsteps-5)..(Nsteps+1)}: ( sum{i in 1..3} (z_H[i,k] + x_t[i])^2 )<= 0.010; #  z_H = -x_t  \n\n");

               break;

           case 2: // y target


               PostureMod << string("# Hand approach orientation\n");
               PostureMod << string("subject to constr_hand_or {k in (Nsteps-5)..(Nsteps+1)}: ( sum{i in 1..3} (z_H[i,k] + y_t[i])^2 )<= 0.010; #  z_H = -y_t  \n\n");

               break;

           case 3: // z target

               //PostureMod << string("# Hand approach orientation\n");
               //PostureMod << string("subject to const_hand_or {k in (Nsteps-5)..(Nsteps+1)}: ( sum{i in 1..3} (z_H[i,k] + z_t[i])^2 )<= 0.010; #  z_H = -z_t  \n\n");

               break;
           }

           break;

       case 1: // reaching

           break;

       case 2: // transport

           break;

       case 3: // engage

           switch (this->targetAxis) {

           case 0: // none

               break;

           case 1: // x target

               //PostureMod << string("# Hand approach orientation\n");
               //PostureMod << string("subject to constr_hand_or {k in (Nsteps-5)..(Nsteps+1)}: ( sum{i in 1..3} (z_H[i,k] + x_t[i])^2 )<= 0.010; #  z_H = -x_t  \n\n");

               break;

           case 2: // y target

               //PostureMod << string("# Hand approach orientation\n");
               //PostureMod << string("subject to constr_hand_or {k in (Nsteps-5)..(Nsteps+1)}: ( sum{i in 1..3} (z_H[i,k] + y_t[i])^2 )<= 0.010; #  z_H = -y_t  \n\n");

               break;

           case 3: // z target

               //PostureMod << string("# Hand approach orientation\n");
               //PostureMod << string("subject to const_hand_or {k in (Nsteps-5)..(Nsteps+1)}: ( sum{i in 1..3} (x_H[i,k] - z_t[i])^2 )<= 0.010; #  z_H = -z_t  \n\n");

               //PostureMod << string("# Hand approach orientation at the beginning\n");
               //PostureMod << string("subject to constr_hand_or {k in 1..5}: ( sum{i in 1..3} (z_H[i,k] + z_w[i])^2 )<= 0.010; #  z_H = -y_t  \n\n");

               break;
           }

           break;

       case 4: // disengage

           break;

       case 5: // Go park

           switch (this->targetAxis) {

           case 0: // none

               break;

           case 1: // x target

               //PostureMod << string("# Hand approach orientation\n");
               //PostureMod << string("subject to constr_hand_or {k in 1..5}: ( sum{i in 1..3} (z_H[i,k] + x_t[i])^2 )<= 0.010; #  z_H = -x_t  \n\n");

               break;

           case 2: // y target


               //PostureMod << string("# Hand approach orientation\n");
               //PostureMod << string("subject to constr_hand_or {k in 1..5}: ( sum{i in 1..3} (z_H[i,k] + y_t[i])^2 )<= 0.010; #  z_H = -y_t  \n\n");

               break;

           case 3: // z target

               //PostureMod << string("# Hand approach orientation\n");
               //PostureMod << string("subject to const_hand_or {k in 1..5}: ( sum{i in 1..3} (z_H[i,k] + z_t[i])^2 )<= 0.010; #  z_H = -z_t  \n\n");

               break;
           }

           break;


       }



       break;

   case 112: case 212: // side thumb right

       break;

   case 113: case 213: // side thumb up

       break;

   case 114: case 214: // side thumb down

       break;

   case 121: case 221: // above

       break;

   case 122: case 222: // below

       break;
   }

if((gopark && release_object) || (!gopark && !release_object) ){

      if(target_avoidance){

           // constraints with the targets
           MatrixXf tols_0 = tolsTarget.at(0);
           MatrixXf tols_1 = tolsTarget.at(1);
           MatrixXf tols_2 = tolsTarget.at(2);

           //xx1
           string txx1_0 = boost::str(boost::format("%.2f") % tols_0(0,0)); boost::replace_all(txx1_0,",",".");
           string txx1_1 = boost::str(boost::format("%.2f") % tols_1(0,0)); boost::replace_all(txx1_1,",",".");
           string txx1_2 = boost::str(boost::format("%.2f") % tols_2(0,0)); boost::replace_all(txx1_2,",",".");

           PostureMod << string("param tol_target_xx1 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txx1_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txx1_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txx1_2+string("; \n");

           //xx2
           string txx2_0 = boost::str(boost::format("%.2f") % tols_0(1,0)); boost::replace_all(txx2_0,",",".");
           string txx2_1 = boost::str(boost::format("%.2f") % tols_1(1,0)); boost::replace_all(txx2_1,",",".");
           string txx2_2 = boost::str(boost::format("%.2f") % tols_2(1,0)); boost::replace_all(txx2_2,",",".");

           PostureMod << string("param tol_target_xx2 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txx2_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txx2_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txx2_2+string("; \n");


           // xx3
           string txx3_0 = boost::str(boost::format("%.2f") % tols_0(2,0)); boost::replace_all(txx3_0,",",".");
           string txx3_1 = boost::str(boost::format("%.2f") % tols_1(2,0)); boost::replace_all(txx3_1,",",".");
           string txx3_2 = boost::str(boost::format("%.2f") % tols_2(2,0)); boost::replace_all(txx3_2,",",".");

           PostureMod << string("param tol_target_xx3 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txx3_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txx3_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txx3_2+string("; \n");

           // yy1
           string tyy1_0 = boost::str(boost::format("%.2f") % tols_0(0,1)); boost::replace_all(tyy1_0,",",".");
           string tyy1_1 = boost::str(boost::format("%.2f") % tols_1(0,1)); boost::replace_all(tyy1_1,",",".");
           string tyy1_2 = boost::str(boost::format("%.2f") % tols_2(0,1)); boost::replace_all(tyy1_2,",",".");

           PostureMod << string("param tol_target_yy1 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tyy1_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tyy1_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tyy1_2+string("; \n");

           // yy2
           string tyy2_0 = boost::str(boost::format("%.2f") % tols_0(1,1)); boost::replace_all(tyy2_0,",",".");
           string tyy2_1 = boost::str(boost::format("%.2f") % tols_1(1,1)); boost::replace_all(tyy2_1,",",".");
           string tyy2_2 = boost::str(boost::format("%.2f") % tols_2(1,1)); boost::replace_all(tyy2_2,",",".");

           PostureMod << string("param tol_target_yy2 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tyy2_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tyy2_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tyy2_2+string("; \n");


           // yy3
           string tyy3_0 = boost::str(boost::format("%.2f") % tols_0(2,1)); boost::replace_all(tyy3_0,",",".");
           string tyy3_1 = boost::str(boost::format("%.2f") % tols_1(2,1)); boost::replace_all(tyy3_1,",",".");
           string tyy3_2 = boost::str(boost::format("%.2f") % tols_2(2,1)); boost::replace_all(tyy3_2,",",".");

           PostureMod << string("param tol_target_yy3 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tyy3_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tyy3_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tyy3_2+string("; \n");

           // zz1
           string tzz1_0 = boost::str(boost::format("%.2f") % tols_0(0,2)); boost::replace_all(tzz1_0,",",".");
           string tzz1_1 = boost::str(boost::format("%.2f") % tols_1(0,2)); boost::replace_all(tzz1_1,",",".");
           string tzz1_2 = boost::str(boost::format("%.2f") % tols_2(0,2)); boost::replace_all(tzz1_2,",",".");

           PostureMod << string("param tol_target_zz1 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tzz1_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tzz1_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tzz1_2+string("; \n");

           // zz2
           string tzz2_0 = boost::str(boost::format("%.2f") % tols_0(1,2)); boost::replace_all(tzz2_0,",",".");
           string tzz2_1 = boost::str(boost::format("%.2f") % tols_1(1,2)); boost::replace_all(tzz2_1,",",".");
           string tzz2_2 = boost::str(boost::format("%.2f") % tols_2(1,2)); boost::replace_all(tzz2_2,",",".");

           PostureMod << string("param tol_target_zz2 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tzz2_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tzz2_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tzz2_2+string("; \n");

           // zz3
           string tzz3_0 = boost::str(boost::format("%.2f") % tols_0(2,2)); boost::replace_all(tzz3_0,",",".");
           string tzz3_1 = boost::str(boost::format("%.2f") % tols_1(2,2)); boost::replace_all(tzz3_1,",",".");
           string tzz3_2 = boost::str(boost::format("%.2f") % tols_2(2,2)); boost::replace_all(tzz3_2,",",".");

           PostureMod << string("param tol_target_zz3 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tzz3_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tzz3_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tzz3_2+string("; \n");


           // xy1
           string txy1_0 = boost::str(boost::format("%.2f") % tols_0(0,3)); boost::replace_all(txy1_0,",",".");
           string txy1_1 = boost::str(boost::format("%.2f") % tols_1(0,3)); boost::replace_all(txy1_1,",",".");
           string txy1_2 = boost::str(boost::format("%.2f") % tols_2(0,3)); boost::replace_all(txy1_2,",",".");

           PostureMod << string("param tol_target_xy1 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txy1_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txy1_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txy1_2+string("; \n");

           // xy2
           string txy2_0 = boost::str(boost::format("%.2f") % tols_0(1,3)); boost::replace_all(txy2_0,",",".");
           string txy2_1 = boost::str(boost::format("%.2f") % tols_1(1,3)); boost::replace_all(txy2_1,",",".");
           string txy2_2 = boost::str(boost::format("%.2f") % tols_2(1,3)); boost::replace_all(txy2_2,",",".");

           PostureMod << string("param tol_target_xy2 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txy2_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txy2_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txy2_2+string("; \n");

           // xy3
           string txy3_0 = boost::str(boost::format("%.2f") % tols_0(2,3)); boost::replace_all(txy3_0,",",".");
           string txy3_1 = boost::str(boost::format("%.2f") % tols_1(2,3)); boost::replace_all(txy3_1,",",".");
           string txy3_2 = boost::str(boost::format("%.2f") % tols_2(2,3)); boost::replace_all(txy3_2,",",".");

           PostureMod << string("param tol_target_xy3 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txy3_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txy3_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txy3_2+string("; \n");


           // xz1
           string txz1_0 = boost::str(boost::format("%.2f") % tols_0(0,4)); boost::replace_all(txz1_0,",",".");
           string txz1_1 = boost::str(boost::format("%.2f") % tols_1(0,4)); boost::replace_all(txz1_1,",",".");
           string txz1_2 = boost::str(boost::format("%.2f") % tols_2(0,4)); boost::replace_all(txz1_2,",",".");

           PostureMod << string("param tol_target_xz1 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txz1_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txz1_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txz1_2+string("; \n");

           // xz2
           string txz2_0 = boost::str(boost::format("%.2f") % tols_0(1,4)); boost::replace_all(txz2_0,",",".");
           string txz2_1 = boost::str(boost::format("%.2f") % tols_1(1,4)); boost::replace_all(txz2_1,",",".");
           string txz2_2 = boost::str(boost::format("%.2f") % tols_2(1,4)); boost::replace_all(txz2_2,",",".");

           PostureMod << string("param tol_target_xz2 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txz2_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txz2_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txz2_2+string("; \n");

           // xz3
           string txz3_0 = boost::str(boost::format("%.2f") % tols_0(2,4)); boost::replace_all(txz3_0,",",".");
           string txz3_1 = boost::str(boost::format("%.2f") % tols_1(2,4)); boost::replace_all(txz3_1,",",".");
           string txz3_2 = boost::str(boost::format("%.2f") % tols_2(2,4)); boost::replace_all(txz3_2,",",".");

           PostureMod << string("param tol_target_xz3 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txz3_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txz3_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txz3_2+string("; \n");

           // yz1
           string tyz1_0 = boost::str(boost::format("%.2f") % tols_0(0,5)); boost::replace_all(tyz1_0,",",".");
           string tyz1_1 = boost::str(boost::format("%.2f") % tols_1(0,5)); boost::replace_all(tyz1_1,",",".");
           string tyz1_2 = boost::str(boost::format("%.2f") % tols_2(0,5)); boost::replace_all(tyz1_2,",",".");

           PostureMod << string("param tol_target_yz1 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tyz1_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tyz1_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tyz1_2+string("; \n");

           // yz2
           string tyz2_0 = boost::str(boost::format("%.2f") % tols_0(1,5)); boost::replace_all(tyz2_0,",",".");
           string tyz2_1 = boost::str(boost::format("%.2f") % tols_1(1,5)); boost::replace_all(tyz2_1,",",".");
           string tyz2_2 = boost::str(boost::format("%.2f") % tols_2(1,5)); boost::replace_all(tyz2_2,",",".");

           PostureMod << string("param tol_target_yz2 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tyz2_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tyz2_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tyz2_2+string("; \n");


           // yz3
           string tyz3_0 = boost::str(boost::format("%.2f") % tols_0(2,5)); boost::replace_all(tyz3_0,",",".");
           string tyz3_1 = boost::str(boost::format("%.2f") % tols_1(2,5)); boost::replace_all(tyz3_1,",",".");
           string tyz3_2 = boost::str(boost::format("%.2f") % tols_2(2,5)); boost::replace_all(tyz3_2,",",".");

           PostureMod << string("param tol_target_yz3 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tyz3_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tyz3_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tyz3_2+string("; \n");


           PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
           PostureMod << string("# \n");
           if (engage){
               PostureMod << string("subject to target_Arm{j in 1..18, l in 1..Nsteps-2}:   \n");

           }else if (gopark && release_object){
               PostureMod << string("subject to target_Arm{j in 1..15, l in 5..Nsteps+1}:   \n");
           }else if(!gopark && !release_object){
               PostureMod << string("subject to target_Arm{j in 1..15, l in 1..Nsteps-2}:   \n");

           }
           if((gopark && release_object) || (!gopark && !release_object) ){
               PostureMod << string("((Points_Arm[j,1,l]-ObjTar[1,1])^2)*( \n");
               PostureMod << string("(x_t[1])^2 / ((ObjTar[1,4]+Points_Arm[j,4,l]+tol_target_xx1[l])^2) + \n");
               PostureMod << string("(x_t[2])^2 / ((ObjTar[1,5]+Points_Arm[j,4,l]+tol_target_xx2[l])^2) + \n");
               PostureMod << string("(x_t[3])^2 / ((ObjTar[1,6]+Points_Arm[j,4,l]+tol_target_xx3[l])^2)) \n");
               PostureMod << string("+ \n");
               PostureMod << string("((Points_Arm[j,2,l]-ObjTar[1,2])^2)*(  \n");
               PostureMod << string("(y_t[1])^2 / ((ObjTar[1,4]+Points_Arm[j,4,l]+tol_target_yy1[l])^2) + \n");
               PostureMod << string("(y_t[2])^2 / ((ObjTar[1,5]+Points_Arm[j,4,l]+tol_target_yy2[l])^2) + \n");
               PostureMod << string("(y_t[3])^2 / ((ObjTar[1,6]+Points_Arm[j,4,l]+tol_target_yy3[l])^2)) \n");
               PostureMod << string("+ \n");
               PostureMod << string("((Points_Arm[j,3,l]-ObjTar[1,3])^2)*( \n");
               PostureMod << string("(z_t[1])^2 / ((ObjTar[1,4]+Points_Arm[j,4,l]+tol_target_zz1[l])^2) + \n");
               PostureMod << string("(z_t[2])^2 / ((ObjTar[1,5]+Points_Arm[j,4,l]+tol_target_zz2[l])^2) +  \n");
               PostureMod << string("(z_t[3])^2 / ((ObjTar[1,6]+Points_Arm[j,4,l]+tol_target_zz3[l])^2)) \n");
               PostureMod << string("+ \n");
               PostureMod << string("2*(Points_Arm[j,1,l]-ObjTar[1,1])*(Points_Arm[j,2,l]-ObjTar[1,2])* ( \n");
               PostureMod << string("(x_t[1]*y_t[1])/((ObjTar[1,4]+Points_Arm[j,4,l]+tol_target_xy1[l])^2) + \n");
               PostureMod << string("(x_t[2]*y_t[2])/((ObjTar[1,5]+Points_Arm[j,4,l]+tol_target_xy2[l])^2) + \n");
               PostureMod << string("(x_t[3]*y_t[3])/((ObjTar[1,6]+Points_Arm[j,4,l]+tol_target_xy3[l])^2)) \n");
               PostureMod << string("+ \n");
               PostureMod << string("2*(Points_Arm[j,1,l]-ObjTar[1,1])*(Points_Arm[j,3,l]-ObjTar[1,3])* ( \n");
               PostureMod << string("(x_t[1]*z_t[1])/((ObjTar[1,4]+Points_Arm[j,4,l]+tol_target_xz1[l])^2) + \n");
               PostureMod << string("(x_t[2]*z_t[2])/((ObjTar[1,5]+Points_Arm[j,4,l]+tol_target_xz2[l])^2) + \n");
               PostureMod << string("(x_t[3]*z_t[3])/((ObjTar[1,6]+Points_Arm[j,4,l]+tol_target_xz3[l])^2)) \n");
               PostureMod << string("+ \n");
               PostureMod << string("2*(Points_Arm[j,2,l]-ObjTar[1,2])*(Points_Arm[j,3,l]-ObjTar[1,3])* (\n");
               PostureMod << string("(y_t[1]*z_t[1])/((ObjTar[1,4]+Points_Arm[j,4,l]+tol_target_yz1[l])^2) + \n");
               PostureMod << string("(y_t[2]*z_t[2])/((ObjTar[1,5]+Points_Arm[j,4,l]+tol_target_yz2[l])^2) + \n");
               PostureMod << string("(y_t[3]*z_t[3])/((ObjTar[1,6]+Points_Arm[j,4,l]+tol_target_yz3[l])^2)) \n");
               PostureMod << string("-1 >=0; \n");
               PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
               PostureMod << string("# \n");
           }

      }

   }

if(obstacle_avoidance){

   // coinstraints with the obstacles
   MatrixXf tolsObs_0 = tolsObstacles.at(0);
   MatrixXf tolsObs_1 = tolsObstacles.at(1);

   //xx1
   string tbxx1_0 = boost::str(boost::format("%.2f") % tolsObs_0(0,0)); boost::replace_all(tbxx1_0,",",".");
   string tbxx1_1 = boost::str(boost::format("%.2f") % tolsObs_1(0,0)); boost::replace_all(tbxx1_1,",",".");

   PostureMod << string("param tol_obs_xx1 {i in 1..Nsteps+1} :=  \n");
   PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxx1_0+string("\n");
   PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxx1_1+string("; \n");

   //xx2
   string tbxx2_0 = boost::str(boost::format("%.2f") % tolsObs_0(1,0)); boost::replace_all(tbxx2_0,",",".");
   string tbxx2_1 = boost::str(boost::format("%.2f") % tolsObs_1(1,0)); boost::replace_all(tbxx2_1,",",".");

   PostureMod << string("param tol_obs_xx2 {i in 1..Nsteps+1} :=  \n");
   PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxx2_0+string("\n");
   PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxx2_1+string("; \n");


   // xx3
   string tbxx3_0 = boost::str(boost::format("%.2f") % tolsObs_0(2,0)); boost::replace_all(tbxx3_0,",",".");
   string tbxx3_1 = boost::str(boost::format("%.2f") % tolsObs_1(2,0)); boost::replace_all(tbxx3_1,",",".");

   PostureMod << string("param tol_obs_xx3 {i in 1..Nsteps+1} :=  \n");
   PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxx3_0+string("\n");
   PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxx3_1+string("; \n");

   // yy1
   string tbyy1_0 = boost::str(boost::format("%.2f") % tolsObs_0(0,1)); boost::replace_all(tbyy1_0,",",".");
   string tbyy1_1 = boost::str(boost::format("%.2f") % tolsObs_1(0,1)); boost::replace_all(tbyy1_1,",",".");

   PostureMod << string("param tol_obs_yy1 {i in 1..Nsteps+1} :=  \n");
   PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbyy1_0+string("\n");
   PostureMod << string("else if (i > (Nsteps/2)) then ")+tbyy1_1+string("; \n");

   // yy2
   string tbyy2_0 = boost::str(boost::format("%.2f") % tolsObs_0(1,1)); boost::replace_all(tbyy2_0,",",".");
   string tbyy2_1 = boost::str(boost::format("%.2f") % tolsObs_1(1,1)); boost::replace_all(tbyy2_1,",",".");

   PostureMod << string("param tol_obs_yy2 {i in 1..Nsteps+1} :=  \n");
   PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbyy2_0+string("\n");
   PostureMod << string("else if (i > (Nsteps/2)) then ")+tbyy2_1+string("; \n");


   // yy3
   string tbyy3_0 = boost::str(boost::format("%.2f") % tolsObs_0(2,1)); boost::replace_all(tbyy3_0,",",".");
   string tbyy3_1 = boost::str(boost::format("%.2f") % tolsObs_1(2,1)); boost::replace_all(tbyy3_1,",",".");

   PostureMod << string("param tol_obs_yy3 {i in 1..Nsteps+1} :=  \n");
   PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbyy3_0+string("\n");
   PostureMod << string("else if (i > (Nsteps/2)) then ")+tbyy3_1+string("; \n");


   // zz1
   string tbzz1_0 = boost::str(boost::format("%.2f") % tolsObs_0(0,2)); boost::replace_all(tbzz1_0,",",".");
   string tbzz1_1 = boost::str(boost::format("%.2f") % tolsObs_1(0,2)); boost::replace_all(tbzz1_1,",",".");

   PostureMod << string("param tol_obs_zz1 {i in 1..Nsteps+1} :=  \n");
   PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbzz1_0+string("\n");
   PostureMod << string("else if (i > (Nsteps/2)) then ")+tbzz1_1+string("; \n");

   // zz2
   string tbzz2_0 = boost::str(boost::format("%.2f") % tolsObs_0(1,2)); boost::replace_all(tbzz2_0,",",".");
   string tbzz2_1 = boost::str(boost::format("%.2f") % tolsObs_1(1,2)); boost::replace_all(tbzz2_1,",",".");

   PostureMod << string("param tol_obs_zz2 {i in 1..Nsteps+1} :=  \n");
   PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbzz2_0+string("\n");
   PostureMod << string("else if (i > (Nsteps/2)) then ")+tbzz2_1+string("; \n");

   // zz3
   string tbzz3_0 = boost::str(boost::format("%.2f") % tolsObs_0(2,2)); boost::replace_all(tbzz3_0,",",".");
   string tbzz3_1 = boost::str(boost::format("%.2f") % tolsObs_1(2,2)); boost::replace_all(tbzz3_1,",",".");

   PostureMod << string("param tol_obs_zz3 {i in 1..Nsteps+1} :=  \n");
   PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbzz3_0+string("\n");
   PostureMod << string("else if (i > (Nsteps/2)) then ")+tbzz3_1+string("; \n");


   // xy1
   string tbxy1_0 = boost::str(boost::format("%.2f") % tolsObs_0(0,3)); boost::replace_all(tbxy1_0,",",".");
   string tbxy1_1 = boost::str(boost::format("%.2f") % tolsObs_1(0,3)); boost::replace_all(tbxy1_1,",",".");

   PostureMod << string("param tol_obs_xy1 {i in 1..Nsteps+1} :=  \n");
   PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxy1_0+string("\n");
   PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxy1_1+string("; \n");

   // xy2
   string tbxy2_0 = boost::str(boost::format("%.2f") % tolsObs_0(1,3)); boost::replace_all(tbxy2_0,",",".");
   string tbxy2_1 = boost::str(boost::format("%.2f") % tolsObs_1(1,3)); boost::replace_all(tbxy2_1,",",".");

   PostureMod << string("param tol_obs_xy2 {i in 1..Nsteps+1} :=  \n");
   PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxy2_0+string("\n");
   PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxy2_1+string("; \n");

   // xy3
   string tbxy3_0 = boost::str(boost::format("%.2f") % tolsObs_0(2,3)); boost::replace_all(tbxy3_0,",",".");
   string tbxy3_1 = boost::str(boost::format("%.2f") % tolsObs_1(2,3)); boost::replace_all(tbxy3_1,",",".");

   PostureMod << string("param tol_obs_xy3 {i in 1..Nsteps+1} :=  \n");
   PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxy3_0+string("\n");
   PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxy3_1+string("; \n");


   // xz1
   string tbxz1_0 = boost::str(boost::format("%.2f") % tolsObs_0(0,4)); boost::replace_all(tbxz1_0,",",".");
   string tbxz1_1 = boost::str(boost::format("%.2f") % tolsObs_1(0,4)); boost::replace_all(tbxz1_1,",",".");

   PostureMod << string("param tol_obs_xz1 {i in 1..Nsteps+1} :=  \n");
   PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxz1_0+string("\n");
   PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxz1_1+string("; \n");

   // xz2
   string tbxz2_0 = boost::str(boost::format("%.2f") % tolsObs_0(1,4)); boost::replace_all(tbxz2_0,",",".");
   string tbxz2_1 = boost::str(boost::format("%.2f") % tolsObs_1(1,4)); boost::replace_all(tbxz2_1,",",".");

   PostureMod << string("param tol_obs_xz2 {i in 1..Nsteps+1} :=  \n");
   PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxz2_0+string("\n");
   PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxz2_1+string("; \n");

   // xz3
   string tbxz3_0 = boost::str(boost::format("%.2f") % tolsObs_0(2,4)); boost::replace_all(tbxz3_0,",",".");
   string tbxz3_1 = boost::str(boost::format("%.2f") % tolsObs_1(2,4)); boost::replace_all(tbxz3_1,",",".");

   PostureMod << string("param tol_obs_xz3 {i in 1..Nsteps+1} :=  \n");
   PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxz3_0+string("\n");
   PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxz3_1+string("; \n");

   // yz1
   string tbyz1_0 = boost::str(boost::format("%.2f") % tolsObs_0(0,5)); boost::replace_all(tbyz1_0,",",".");
   string tbyz1_1 = boost::str(boost::format("%.2f") % tolsObs_1(0,5)); boost::replace_all(tbyz1_1,",",".");

   PostureMod << string("param tol_obs_yz1{i in 1..Nsteps+1} :=  \n");
   PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbyz1_0+string("\n");
   PostureMod << string("else if (i > (Nsteps/2)) then ")+tbyz1_1+string("; \n");

   // yz2
   string tbyz2_0 = boost::str(boost::format("%.2f") % tolsObs_0(1,5)); boost::replace_all(tbyz2_0,",",".");
   string tbyz2_1 = boost::str(boost::format("%.2f") % tolsObs_1(1,5)); boost::replace_all(tbyz2_1,",",".");

   PostureMod << string("param tol_obs_yz2{i in 1..Nsteps+1} :=  \n");
   PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbyz2_0+string("\n");
   PostureMod << string("else if (i > (Nsteps/2)) then ")+tbyz2_1+string("; \n");


   // yz3
   string tbyz3_0 = boost::str(boost::format("%.2f") % tolsObs_0(2,5)); boost::replace_all(tbyz3_0,",",".");
   string tbyz3_1 = boost::str(boost::format("%.2f") % tolsObs_1(2,5)); boost::replace_all(tbyz3_1,",",".");

   PostureMod << string("param tol_obs_yz3{i in 1..Nsteps+1} :=  \n");
   PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbyz3_0+string("\n");
   PostureMod << string("else if (i > (Nsteps/2)) then ")+tbyz3_1+string("; \n");

   PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
   PostureMod << string("# \n");
   if(engage || transport){
        PostureMod << string("subject to obst_Arm{j in 1..18, i in 1..n_Obstacles, l in 1..Nsteps+1}:\n");
   }else if(gopark){
       PostureMod << string("subject to obst_Arm{j in 1..15, i in 1..(n_Obstacles), l in 1..Nsteps+1}:\n");
   }else{
        PostureMod << string("subject to obst_Arm{j in 1..15, i in 1..(n_Obstacles), l in 1..Nsteps+1}:\n");
   }
   PostureMod << string("((Points_Arm[j,1,l]-Obstacles[i,1])^2)*(\n");
   PostureMod << string("(Rot[1,1,i])^2 / ((Obstacles[i,4]+Points_Arm[j,4,l]+tol_obs_xx1[l])^2) +\n");
   PostureMod << string("(Rot[2,1,i])^2 / ((Obstacles[i,5]+Points_Arm[j,4,l]+tol_obs_xx2[l])^2) + \n");
   PostureMod << string("(Rot[3,1,i])^2 / ((Obstacles[i,6]+Points_Arm[j,4,l]+tol_obs_xx3[l])^2)) \n");
   PostureMod << string("+ \n");
   PostureMod << string("((Points_Arm[j,2,l]-Obstacles[i,2])^2)*( \n");
   PostureMod << string("(Rot[1,2,i])^2 / ((Obstacles[i,4]+Points_Arm[j,4,l]+tol_obs_yy1[l])^2) + \n");
   PostureMod << string("(Rot[2,2,i])^2 / ((Obstacles[i,5]+Points_Arm[j,4,l]+tol_obs_yy2[l])^2) + \n");
   PostureMod << string("(Rot[3,2,i])^2 / ((Obstacles[i,6]+Points_Arm[j,4,l]+tol_obs_yy3[l])^2)) \n");
   PostureMod << string("+ \n");
   PostureMod << string("((Points_Arm[j,3,l]-Obstacles[i,3])^2)*( \n");
   PostureMod << string("(Rot[1,3,i])^2 / ((Obstacles[i,4]+Points_Arm[j,4,l]+tol_obs_zz1[l])^2) + \n");
   PostureMod << string("(Rot[2,3,i])^2 / ((Obstacles[i,5]+Points_Arm[j,4,l]+tol_obs_zz2[l])^2) + \n");
   PostureMod << string("(Rot[3,3,i])^2 / ((Obstacles[i,6]+Points_Arm[j,4,l]+tol_obs_zz3[l])^2)) \n");
   PostureMod << string("+ \n");
   PostureMod << string("2*(Points_Arm[j,1,l]-Obstacles[i,1])*(Points_Arm[j,2,l]-Obstacles[i,2])* ( \n");
   PostureMod << string("(Rot[1,1,i]*Rot[1,2,i])/((Obstacles[i,4]+Points_Arm[j,4,l]+tol_obs_xy1[l])^2) + \n");
   PostureMod << string("(Rot[2,1,i]*Rot[2,2,i])/((Obstacles[i,5]+Points_Arm[j,4,l]+tol_obs_xy2[l])^2) + \n");
   PostureMod << string("(Rot[3,1,i]*Rot[3,2,i])/((Obstacles[i,6]+Points_Arm[j,4,l]+tol_obs_xy3[l])^2)) \n");
   PostureMod << string("+ \n");
   PostureMod << string("2*(Points_Arm[j,1,l]-Obstacles[i,1])*(Points_Arm[j,3,l]-Obstacles[i,3])* ( \n");
   PostureMod << string("(Rot[1,1,i]*Rot[1,3,i])/((Obstacles[i,4]+Points_Arm[j,4,l]+tol_obs_xz1[l])^2) + \n");
   PostureMod << string("(Rot[2,1,i]*Rot[2,3,i])/((Obstacles[i,5]+Points_Arm[j,4,l]+tol_obs_xz2[l])^2) + \n");
   PostureMod << string("(Rot[3,1,i]*Rot[3,3,i])/((Obstacles[i,6]+Points_Arm[j,4,l]+tol_obs_xz3[l])^2)) \n");
   PostureMod << string("+ \n");
   PostureMod << string("2*(Points_Arm[j,2,l]-Obstacles[i,2])*(Points_Arm[j,3,l]-Obstacles[i,3])* ( \n");
   PostureMod << string("(Rot[1,2,i]*Rot[1,3,i])/((Obstacles[i,4]+Points_Arm[j,4,l]+tol_obs_yz1[l])^2) + \n");
   PostureMod << string("(Rot[2,2,i]*Rot[2,3,i])/((Obstacles[i,5]+Points_Arm[j,4,l]+tol_obs_yz2[l])^2) + \n");
   PostureMod << string("(Rot[3,2,i]*Rot[3,3,i])/((Obstacles[i,6]+Points_Arm[j,4,l]+tol_obs_yz3[l])^2)) \n");
   PostureMod << string("-1 >=0; \n");
   PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n");
   PostureMod << string("# \n");

}

   // constraints with the body
   this->writeBodyConstraints(PostureMod,false);

   // constraints with the table
   //this->writeTableConstraints(PostureMod,false,griptype,tols_table);

   PostureMod << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n\n\n");

   // close the file
   PostureMod.close();

   // ----- write run file for options --------
   string filenamerun("options.run");
   ofstream optionsrun;
   // open the file
   optionsrun.open(path+filenamerun);

   optionsrun << string("option presolve 0; \n");


   //close the file
   optionsrun.close();

   return true;


}

bool Problem::writeFilesFinalPosture(int mov_type, humanoidPtr hh, float dHO, int griptype,
                                     std::vector<float> initArmPosture, std::vector<float> finalHand,
                                     std::vector<float> initialGuess, std::vector<objectPtr> objs,
                                     targetPtr tar, objectPtr obj,
                                     std::vector<float> tolsArm, MatrixXf tolsHand, MatrixXf tolsObstacles,
                                     float tolTarPos, float tolTarOr, std::vector<float> lambda, bool obstacle_avoidance,int arm_code)
{


    //  --- create the "Models" directory if it does not exist ---
    struct stat st = {0};
    if (stat("Models", &st) == -1) {
        mkdir("Models", 0700);
    }
    string path("Models/");

    Matrix4f matWorldToArm;
    Matrix4f matHand;
    std::vector<float> minLimits;
    std::vector<float> maxLimits;

    int k;
    switch(arm_code){
    case 0: // dual arm
        //TO DO
        break;

    case 1: // right arm
        k=1;
        hh->getMatRight(matWorldToArm);
        hh->getMatRightHand(matHand);
        hh->getRightMinLimits(minLimits);
        hh->getRightMaxLimits(maxLimits);
        break;

    case 2: // left arm
        k=-1;
        hh->getMatLeft(matWorldToArm);
        hh->getMatLeftHand(matHand);
        hh->getLeftMinLimits(minLimits);
        hh->getLeftMaxLimits(maxLimits);
        break;
    }

    std::vector<float> minArmLimits(minLimits.begin(),minLimits.begin()+JOINTS_ARM);
    std::vector<float> maxArmLimits(maxLimits.begin(),maxLimits.begin()+JOINTS_ARM);

   //------------------------- Write the dat file --------------------------------------------------
    string filename("FinalPosture.dat");
    ofstream PostureDat;
    // open the file
    PostureDat.open(path+filename);

    PostureDat << string("# FINAL POSTURE DATA FILE \n");
    PostureDat << string("# Units of measure: [rad], [mm] \n\n");

    PostureDat << string("data; \n");

    //PostureDat << string("param pi := 4*atan(1); \n");

    // Body dimension
    this->writeBodyDim(hh,PostureDat);

    // D-H Parameters of the Arm
    this->writeArmDHParams(hh,PostureDat,k);
    // distance between the hand and the object
    this->write_dHO(PostureDat,dHO);

    // joint limits    
    this->writeArmLimits(PostureDat,minArmLimits,maxArmLimits);


    // initial pose of the arm
    this->writeArmInitPose(PostureDat,initArmPosture);

    // final posture of the fingers
    this->writeFingerFinalPose(PostureDat,finalHand);


    // joint expense factors of the arm    
    this->writeLambda(PostureDat,lambda);


    // initial guess
    PostureDat << string("# INITIAL GUESS \n");
    PostureDat << string("var theta := \n");
    for (std::size_t i=0; i < initialGuess.size(); ++i){
        string guess =  boost::str(boost::format("%.2f") % (initialGuess.at(i)));
        boost::replace_all(guess,",",".");
        if (i == initialGuess.size()-1){
            PostureDat << to_string(i+1)+string(" ")+guess+string(";\n");
        }else{
            PostureDat << to_string(i+1)+string(" ")+guess+string("\n");
        }
    }

    // Parameters of the Fingers
//#if HAND == 0
    this->writeHumanHandParams(hh,PostureDat,k);
//#elif HAND == 1
    this->writeBarrettHandParams(hh,PostureDat);
//#endif


    // info of the target to reach
    this->writeInfoTarget(PostureDat,tar);

    // info objects
    this->writeInfoObjects(PostureDat,objs);


    // object that has the target
    this->writeInfoObjectTarget(PostureDat,obj);

    //close the file
    PostureDat.close();

    // ------------- Write the mod file ------------------------- //

    string filenamemod("FinalPosture.mod");
    ofstream PostureMod;
    // open the file
    PostureMod.open(path+filenamemod);

    PostureMod << string("# FINAL POSTURE MODEL FILE \n");
    PostureMod << string("# Movement to plan: \n");
    PostureMod << string("# ")+this->mov->getInfoLine()+string("\n\n");

    PostureMod << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
    PostureMod << string("# PARAMETERS \n\n");
    PostureMod << string("set nJoints := 1..")+to_string(initialGuess.size())+string(";\n");

    this->writePI(PostureMod);
    this->writeBodyDimMod(PostureMod);
    this->writeArmDHParamsMod(PostureMod);
    this->write_dHOMod(PostureMod);

    PostureMod << string("# Joint Limits \n");
    PostureMod << string("param llim {i in 1..")+to_string(JOINTS_ARM)+string("} ; \n");
    PostureMod << string("param ulim {i in 1..")+to_string(JOINTS_ARM)+string("} ; \n");

    PostureMod << string("# Initial posture \n");
    PostureMod << string("param thet_init {i in 1..")+to_string(JOINTS_ARM)+string("} ; \n");

    PostureMod << string("# Final finger posture \n");
    PostureMod << string("param joint_fingers {i in 1..")+to_string(JOINTS_HAND)+string("} ; \n");

    PostureMod << string("# Joint Expense Factors \n");
    PostureMod << string("param lambda {i in 1..")+to_string(JOINTS_ARM)+string("} ; \n");




    this->writeHumanHandParamsMod(PostureMod);

    this->writeBarrettHandParamsMod(PostureMod);


    // info objects
    this->writeInfoObjectsMod(PostureMod);

    PostureMod << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
    PostureMod << string("# DECISION VARIABLES \n");
    PostureMod << string("var theta {i in 1..")+to_string(JOINTS_ARM)+string("} >= llim[i], <= ulim[i]; \n");

    // Rotation matrix of the obstacles
    this->writeRotMatObsts(PostureMod);
    // Direct Kinematics of the arm
    this->writeArmDirKin(PostureMod,matWorldToArm,matHand,tolsArm,true);

    // direct kinematics of the fingers

    this->writeHumanHandDirKin(PostureMod,tolsHand,true,false);

    std::vector<int> rk; std::vector<int> jk;
    rk.push_back(-1); rk.push_back(1); rk.push_back(0);
    jk.push_back(-1); jk.push_back(-1); jk.push_back(1);
    this->writeBarrettHandDirKin(PostureMod,rk,jk,tolsHand,true,false);


    // Points of the arm
    PostureMod << string("var Points_Arm {j in 1..21, i in 1..4} = \n");
    PostureMod << string("if ( j=1 ) then 	(Shoulder[i]+Elbow[i])/2  \n");
    PostureMod << string("else	if ( j=2 ) then 	Elbow[i] \n");
    PostureMod << string("else    if ( j=3 ) then 	(Wrist[i]+Elbow[i])/2  \n");
    PostureMod << string("else	if ( j=4 ) then 	Wrist[i] \n");
    PostureMod << string("else	if ( j=5 ) then 	Wrist[i]+0.45*(Hand[i]-Wrist[i]) \n");
    PostureMod << string("else	if ( j=6 ) then 	Wrist[i]+0.75*(Hand[i]-Wrist[i]) \n");
    PostureMod << string("else	if ( j=7 ) then 	Finger1_1[i] \n");
    PostureMod << string("else	if ( j=8 ) then 	Finger2_1[i] \n");
    PostureMod << string("else	if ( j=9 ) then 	Finger3_1[i]\n");
    PostureMod << string("else	if ( j=10 ) then 	(Finger1_1[i]+Finger1_2[i])/2 \n");
    PostureMod << string("else	if ( j=11 ) then 	(Finger2_1[i]+Finger2_2[i])/2 \n");
    PostureMod << string("else	if ( j=12 ) then 	(Finger3_1[i]+Finger3_2[i])/2 \n");
    PostureMod << string("else	if ( j=13 ) then 	 Finger1_2[i] \n");
    PostureMod << string("else	if ( j=14 ) then 	 Finger2_2[i] \n");
    PostureMod << string("else	if ( j=15 ) then 	 Finger3_2[i] \n");
    PostureMod << string("else	if ( j=16 ) then 	(Finger1_2[i]+Finger1_tip[i])/2	 \n");
    PostureMod << string("else	if ( j=17 ) then 	(Finger2_2[i]+Finger2_tip[i])/2 \n");
    PostureMod << string("else	if ( j=18 ) then 	(Finger3_2[i]+Finger3_tip[i])/2 \n");
    PostureMod << string("else	if ( j=19 ) then 	Finger1_tip[i]\n");
    PostureMod << string("else	if ( j=20 ) then 	Finger2_tip[i] \n");
    PostureMod << string("else	if ( j=21 ) then 	Finger3_tip[i] \n");
    PostureMod << string("; \n\n");

    // objective function
    this->writeObjective(PostureMod,true);

    // constraints
    PostureMod << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
    PostureMod << string("#  \n");
    PostureMod << string("#		      Constraints                  # \n");
    PostureMod << string("#  \n");
    PostureMod << string("# Hand position \n");
    PostureMod << string("# subject to contr_hand_pos  {i in 1..3}: Hand[i] + dFH*z_H[i] - Tar_pos[i] = 0; \n");

    string tarpos = boost::str(boost::format("%.2f") % tolTarPos); boost::replace_all(tarpos,",",".");
    string taror = boost::str(boost::format("%.4f") % tolTarOr); boost::replace_all(taror,",",".");

    switch(mov_type){

    case 0: // reach-to-grasp
        PostureMod << string("subject to contr_hand_pos: (sum{i in 1..3} (Hand[i] + dFH*z_H[i] - Tar_pos[i])^2) <= ")+tarpos+string("; \n\n");
        break;
    case 1: // reaching

        break;
    case 2:// transport

        break;

    case 3: // engage
        // the target here has been computed according the new position of the target
        // (the constraint is equal to that one for reaching and grasping)
        PostureMod << string("subject to contr_hand_pos: (sum{i in 1..3} (Hand[i] + dFH*z_H[i] - Tar_pos[i])^2) <= ")+tarpos+string("; \n\n");
        break;

    case 4: // disengage

        break;

    case 5: // Go park

        break;

    }

    PostureMod << string("# Hand orientation\n");
    switch(griptype){

    case 111: case 211: // side thumb left

        switch (mov_type) {

        case 0: // reach-to-grasp

            // hand constraints for approaching direction setting
            switch (this->targetAxis) {

            case 0: // none

                PostureMod << string("subject to constr_hand_orient: (sum{i in 1..3} (x_H[i] - z_t[i])^2 )<= ")+taror+string("; #  x_H = z_t \n");

                break;
            case 1: // x target

                PostureMod << string("subject to constr_hand_orient: (sum{i in 1..3} (x_H[i] - z_t[i])^2 + sum{i in 1..3} (z_H[i] + x_t[i])^2 )<= ")+taror+string("; #  x_H = z_t and z_H = -x_t \n");
                break;

            case 2: // y target

                PostureMod << string("subject to constr_hand_orient: (sum{i in 1..3} (x_H[i] - z_t[i])^2 + sum{i in 1..3} (z_H[i] + y_t[i])^2 )<= ")+taror+string("; #  x_H = z_t and z_H = -y_t \n");

                break;

            case 3: // z target

                // error
                // TO DO

                break;

            }

            break;

        case 1:// reaching
            break;

        case 2: // transport

            break;

        case 3: // engage

            // hand constraints for approaching direction setting
            // (TO REVIEW)
            switch (this->targetAxis) {

            case 0: // none

                PostureMod << string("subject to constr_hand_orient: (sum{i in 1..3} (x_H[i] - z_t[i])^2 )<= ")+taror+string("; #  x_H = z_t \n");

                break;
            case 1: // x target

                PostureMod << string("subject to constr_hand_orient: (sum{i in 1..3} (x_H[i] - z_t[i])^2 + sum{i in 1..3} (z_H[i] + x_t[i])^2 )<= ")+taror+string("; #  x_H = z_t and z_H = -x_t \n");
                break;

            case 2: // y target

                PostureMod << string("subject to constr_hand_orient: (sum{i in 1..3} (x_H[i] - z_t[i])^2 + sum{i in 1..3} (z_H[i] + y_t[i])^2 )<= ")+taror+string("; #  x_H = z_t and z_H = -y_t \n");

                break;

            case 3: // z target

                PostureMod << string("subject to constr_hand_orient: (sum{i in 1..3} (x_H[i] - z_t[i])^2 )<= ")+taror+string("; #  x_H = z_t \n");

                break;

            }

            break;

        case 4: // disengage

            break;

        case 5: // Go park

            break;

        }



        break;

    case 112: case 212: // side thumb right

        PostureMod << string("subject to constr_hand_orient: (sum{i in 1..3} (x_H[i] + z_t[i])^2) <= ")+taror+string("; #  x_H = -z_t \n");

        break;

    case 113: case 213: // side thumb up

        PostureMod << string("subject to constr_hand_orient: (sum{i in 1..3} (y_H[i] + z_t[i])^2) <= ")+taror+string("; #  y_H = -z_t \n");

        break;

    case 114: case 214: // side thumb down

        PostureMod << string("subject to constr_hand_orient: (sum{i in 1..3} (y_H[i] - z_t[i])^2) <= ")+taror+string("; #  y_H = z_t \n");

        break;

    case 121: case 221: // above

        PostureMod << string("subject to constr_hand_orient: (sum{i in 1..3} (z_H[i] + z_t[i])^2) <= ")+to_string(tolTarOr)+string("; #  z_H = -z_t \n");

        break;

    case 122: case 222: // below

        PostureMod << string("subject to constr_hand_orient: (sum{i in 1..3} (z_H[i] - z_t[i])^2) <= ")+to_string(tolTarOr)+string("; #  z_H = z_t \n");

        break;
    }


    if(obstacle_avoidance){
        // obstacles
        //xx
        string txx1 = boost::str(boost::format("%.2f") % tolsObstacles(0,0)); boost::replace_all(txx1,",",".");
        string txx2 = boost::str(boost::format("%.2f") % tolsObstacles(1,0)); boost::replace_all(txx2,",",".");
        string txx3 = boost::str(boost::format("%.2f") % tolsObstacles(2,0)); boost::replace_all(txx3,",",".");
        //yy
        string tyy1 = boost::str(boost::format("%.2f") % tolsObstacles(0,1)); boost::replace_all(tyy1,",",".");
        string tyy2 = boost::str(boost::format("%.2f") % tolsObstacles(1,1)); boost::replace_all(tyy2,",",".");
        string tyy3 = boost::str(boost::format("%.2f") % tolsObstacles(2,1)); boost::replace_all(tyy3,",",".");
        //zz
        string tzz1 = boost::str(boost::format("%.2f") % tolsObstacles(0,2)); boost::replace_all(tzz1,",",".");
        string tzz2 = boost::str(boost::format("%.2f") % tolsObstacles(1,2)); boost::replace_all(tzz2,",",".");
        string tzz3 = boost::str(boost::format("%.2f") % tolsObstacles(2,2)); boost::replace_all(tzz3,",",".");
        //xy
        string txy1 = boost::str(boost::format("%.2f") % tolsObstacles(0,3)); boost::replace_all(txy1,",",".");
        string txy2 = boost::str(boost::format("%.2f") % tolsObstacles(1,3)); boost::replace_all(txy2,",",".");
        string txy3 = boost::str(boost::format("%.2f") % tolsObstacles(2,3)); boost::replace_all(txy3,",",".");
        //xz
        string txz1 = boost::str(boost::format("%.2f") % tolsObstacles(0,4)); boost::replace_all(txz1,",",".");
        string txz2 = boost::str(boost::format("%.2f") % tolsObstacles(1,4)); boost::replace_all(txz2,",",".");
        string txz3 = boost::str(boost::format("%.2f") % tolsObstacles(2,4)); boost::replace_all(txz3,",",".");
        //yz
        string tyz1 = boost::str(boost::format("%.2f") % tolsObstacles(0,5)); boost::replace_all(tyz1,",",".");
        string tyz2 = boost::str(boost::format("%.2f") % tolsObstacles(1,5)); boost::replace_all(tyz2,",",".");
        string tyz3 = boost::str(boost::format("%.2f") % tolsObstacles(2,5)); boost::replace_all(tyz3,",",".");


        PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
        PostureMod << string("# \n");
        PostureMod << string("subject to obst_Arm{j in 1..21, i in 1..n_Obstacles}:  \n");
        PostureMod << string("((Points_Arm[j,1]-Obstacles[i,1])^2)*(  \n");
        PostureMod << string("(Rot[1,1,i])^2 / ((Obstacles[i,4]+Points_Arm[j,4]+")+txx1+string(")^2) + \n");
        PostureMod << string("(Rot[2,1,i])^2 / ((Obstacles[i,5]+Points_Arm[j,4]+")+txx2+string(")^2) + \n");
        PostureMod << string("(Rot[3,1,i])^2 / ((Obstacles[i,6]+Points_Arm[j,4]+")+txx3+string(")^2)) \n");
        PostureMod << string("+ \n");
        PostureMod << string("((Points_Arm[j,2]-Obstacles[i,2])^2)*(  \n");
        PostureMod << string("(Rot[1,2,i])^2 / ((Obstacles[i,4]+Points_Arm[j,4]+")+tyy1+string(")^2) + \n");
        PostureMod << string("(Rot[2,2,i])^2 / ((Obstacles[i,5]+Points_Arm[j,4]+")+tyy2+string(")^2) + \n");
        PostureMod << string("(Rot[3,2,i])^2 / ((Obstacles[i,6]+Points_Arm[j,4]+")+tyy3+string(")^2)) \n");
        PostureMod << string("+ \n");
        PostureMod << string("((Points_Arm[j,3]-Obstacles[i,3])^2)*( \n");
        PostureMod << string("(Rot[1,3,i])^2 / ((Obstacles[i,4]+Points_Arm[j,4]+")+tzz1+string(")^2) + \n");
        PostureMod << string("(Rot[2,3,i])^2 / ((Obstacles[i,5]+Points_Arm[j,4]+")+tzz2+string(")^2) +  \n");
        PostureMod << string("(Rot[3,3,i])^2 / ((Obstacles[i,6]+Points_Arm[j,4]+")+tzz3+string(")^2)) \n");
        PostureMod << string("+ \n");
        PostureMod << string("2*(Points_Arm[j,1]-Obstacles[i,1])*(Points_Arm[j,2]-Obstacles[i,2])* ( \n");
        PostureMod << string("(Rot[1,1,i]*Rot[1,2,i])/((Obstacles[i,4]+Points_Arm[j,4]+")+txy1+string(")^2) + \n");
        PostureMod << string("(Rot[2,1,i]*Rot[2,2,i])/((Obstacles[i,5]+Points_Arm[j,4]+")+txy2+string(")^2) + \n");
        PostureMod << string("(Rot[3,1,i]*Rot[3,2,i])/((Obstacles[i,6]+Points_Arm[j,4]+")+txy3+string(")^2)) \n");
        PostureMod << string("+ \n");
        PostureMod << string("2*(Points_Arm[j,1]-Obstacles[i,1])*(Points_Arm[j,3]-Obstacles[i,3])* ( \n");
        PostureMod << string("(Rot[1,1,i]*Rot[1,3,i])/((Obstacles[i,4]+Points_Arm[j,4]+")+txz1+string(")^2) + \n");
        PostureMod << string("(Rot[2,1,i]*Rot[2,3,i])/((Obstacles[i,5]+Points_Arm[j,4]+")+txz2+string(")^2) + \n");
        PostureMod << string("(Rot[3,1,i]*Rot[3,3,i])/((Obstacles[i,6]+Points_Arm[j,4]+")+txz3+string(")^2)) \n");
        PostureMod << string("+ \n");
        PostureMod << string("2*(Points_Arm[j,2]-Obstacles[i,2])*(Points_Arm[j,3]-Obstacles[i,3])* ( \n");
        PostureMod << string("(Rot[1,2,i]*Rot[1,3,i])/((Obstacles[i,4]+Points_Arm[j,4]+")+tyz1+string(")^2) + \n");
        PostureMod << string("(Rot[2,2,i]*Rot[2,3,i])/((Obstacles[i,5]+Points_Arm[j,4]+")+tyz2+string(")^2) + \n");
        PostureMod << string("(Rot[3,2,i]*Rot[3,3,i])/((Obstacles[i,6]+Points_Arm[j,4]+")+tyz3+string(")^2)) \n");
        PostureMod << string("-1 >=0; \n");
        PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
        PostureMod << string("#  \n");

    }



    // constraints with the body
    this->writeBodyConstraints(PostureMod,true);

    // constraints with the table
    // this->writeTableConstraints(PostureMod,true,griptype,tols_table);

    PostureMod << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n\n\n");

    // close the file
    PostureMod.close();

    // ----- write run file for options --------

    string filenamerun("options.run");
    ofstream optionsrun;
    // open the file
    optionsrun.open(path+filenamerun);

    optionsrun << string("option presolve 0; \n");

    //close the file
    optionsrun.close();


    return true;



}


bool Problem::finalPostureFingers(int hand_id)
{

    bool success=false;
    humanoidPtr hh = this->scene->getHumanoid();


    // get the object(s) involved in this movement
    objectPtr obj = this->mov->getObject();
    // get the type of grip for this movement
    int grip_code = this->mov->getGrip();

    // compute the diameter (plus tolerance) of the object to be grasped
    float d_obj;

    switch (grip_code) {

    case 111: case 112:
        // Precision Side thumb left and Precision Side thumb right

        d_obj = obj->getRadius()*2.0+TOL_GRIP;
#if HAND==0
        if(d_obj > hh->getHumanHand().maxAperture){
            success=false;
            throw string("impossible to grasp the object ")+obj->getName()+
                    string(" with the grip ")+this->mov->getGripStr()+
                    string(". The object is too large");
        }
#elif HAND==1

        if (d_obj > hh->getBarrettHand().maxAperture){
            success=false;
            throw string("impossible to grasp the object ")+obj->getName()+
                    string(" with the grip ")+this->mov->getGripStr()+
                    string(". The object is too large");
        }
#endif


        break;

    case 113: case 114:
        // Precision Side thumb up and Precision Side thumb down
        d_obj=obj->getSize().Zsize+TOL_GRIP;
#if HAND==0

        if(d_obj > hh->getHumanHand().maxAperture){
            success=false;
            throw string("impossible to grasp the object ")+obj->getName()+
                    string(" with the grip ")+this->mov->getGripStr()+
                    string(". The object is too large");
        }
#elif HAND==1
        if (d_obj > hh->getBarrettHand().maxAperture){
            success=false;
            throw string("impossible to grasp the object ")+obj->getName()+
                    string(" with the grip ")+this->mov->getGripStr()+
                    string(". The object is too large");
        }

#endif



        break;

    case 121: case 122:
        // Precision Above and Precision Below
        d_obj = obj->getRadius()*2+TOL_GRIP;
#if HAND==0

        if(d_obj > hh->getHumanHand().maxAperture){
            success=false;
            throw string("impossible to grasp the object ")+obj->getName()+
                    string(" with the grip ")+this->mov->getGripStr()+
                    string(". The object is too large");

        }
#elif HAND==1

        if (d_obj > hh->getBarrettHand().maxAperture){
            success=false;
            throw string("impossible to grasp the object ")+obj->getName()+
                    string(" with the grip ")+this->mov->getGripStr()+
                    string(". The object is too large");
        }

#endif

        break;

    case 211: case 212:
        // Full Side thumb left and Full Side thumb right

#if HAND==0

        d_obj = min(hh->getHumanHand().maxAperture,float(1.2)*obj->getRadius()*2+TOL_GRIP);

        if(obj->getRadius()*2+TOL_GRIP > hh->getHumanHand().maxAperture){
            success=false;
            throw string("impossible to grasp the object ")+obj->getName()+
                    string(" with the grip ")+this->mov->getGripStr()+
                    string(". The object is too large");

        }

#elif HAND==1
        d_obj = min(hh->getBarrettHand().maxAperture,float(1.2)*obj->getRadius()*2+TOL_GRIP);

        if (obj->getRadius()*2+TOL_GRIP > hh->getBarrettHand().maxAperture){
            success=false;
            throw string("impossible to grasp the object ")+obj->getName()+
                    string(" with the grip ")+this->mov->getGripStr()+
                    string(". The object is too large");
        }
#endif
        break;

    case 213: case 214:
        // Full Side thumb up and Full Side thumb down

#if HAND==0
        d_obj = min(hh->getHumanHand().maxAperture,float(1.2)*(obj->getSize().Zsize+TOL_GRIP));

        if(obj->getSize().Zsize+TOL_GRIP > hh->getHumanHand().maxAperture){
            success=false;
            throw string("impossible to grasp the object ")+obj->getName()+
                    string(" with the grip ")+this->mov->getGripStr()+
                    string(". The object is too large");
        }
#elif HAND==1
        d_obj = min(hh->getBarrettHand().maxAperture,float(1.2)*(obj->getSize().Zsize+TOL_GRIP));

        if (obj->getSize().Zsize+TOL_GRIP > hh->getBarrettHand().maxAperture){
            success=false;
            throw string("impossible to grasp the object ")+obj->getName()+
                    string(" with the grip ")+this->mov->getGripStr()+
                    string(". The object is too large");
        }

#endif




        break;

    case 221: case 222:
        // Full Above and Full Below

#if HAND==0

        d_obj = min(hh->getHumanHand().maxAperture,float(1.2)*(obj->getRadius()*2+TOL_GRIP));

        if(obj->getRadius()*2+TOL_GRIP > hh->getHumanHand().maxAperture){
            success=false;
            throw string("impossible to grasp the object ")+obj->getName()+
                    string(" with the grip ")+this->mov->getGripStr()+
                    string(". The object is too large");
        }
#elif HAND==1

        d_obj = min(hh->getBarrettHand().maxAperture,float(1.2)*(obj->getRadius()*2+TOL_GRIP));

        if (obj->getRadius()*2+TOL_GRIP > hh->getBarrettHand().maxAperture){
            success=false;
            throw string("impossible to grasp the object ")+obj->getName()+
                    string(" with the grip ")+this->mov->getGripStr()+
                    string(". The object is too large");
        }

#endif




        break;

    }// switch grip code

    // compute the inverse kinematics of the hand
    std::vector<float> sols;
    bool inv_succ;
    float theta;
    float thetaT;

    try{

        inv_succ = this->invKinHand(d_obj,hand_id,sols);
        if (inv_succ){
            theta = sols.at(0);
            thetaT = sols.at(1);
        }else{
            throw string("Error: hand inverse kinematic not solved");
        }
    }
    catch(const string str){

        success=false;
        throw str;

    }
    success=true;

    switch(this->mov->getArm()){

    case 0: // both arms

        //TO DO;
        break;

    case 1: // right arm

        this->rightFinalHand.at(0) = THETA8_FINAL;
        this->rightFinalPosture.at(JOINTS_ARM) = THETA8_FINAL;
        this->rightFinalPosture_diseng.at(JOINTS_ARM) = THETA8_FINAL;
        this->rightFinalPosture_eng.at(JOINTS_ARM) = THETA8_FINAL;
#if HAND == 0

        this->rightFinalHand.at(1) = theta;
        this->rightFinalHand.at(2) = theta;
        this->rightFinalHand.at(3) = thetaT;

        this->rightFinalPosture.at(JOINTS_ARM+1)=theta;
        this->rightFinalPosture.at(JOINTS_ARM+2)=theta;
        this->rightFinalPosture.at(JOINTS_ARM+3)=thetaT;

        this->rightFinalPosture_diseng.at(JOINTS_ARM+1)=theta;
        this->rightFinalPosture_diseng.at(JOINTS_ARM+2)=theta;
        this->rightFinalPosture_diseng.at(JOINTS_ARM+3)=thetaT;

        this->rightFinalPosture_eng.at(JOINTS_ARM+1)=theta;
        this->rightFinalPosture_eng.at(JOINTS_ARM+2)=theta;
        this->rightFinalPosture_eng.at(JOINTS_ARM+3)=thetaT;


#elif HAND == 1
        // we consider:
        // 1. the spread of the fingers F1 and F2 always at home ( theta8 = THETA8_home)
        // 2. the displacement of the other joints is equal (theta9=theta10=theta11)

        for (int i=0; i< HAND_FINGERS; ++i){
             this->rightFinalHand.at(i+1) = theta;
             this->rightFinalPosture.at(JOINTS_ARM+i+1)=theta;
        }

#endif


        switch (grip_code) {
        case 111: case 112: case 113: case 114: case 121: case 122:
            //Precision grip

             this->dHOr = this->dFH;

            break;
        case 211: case 212: case 213: case 214:
            // Full Side grip

            this->dHOr = obj->getRadius()+TOL_GRIP;

            break;
        case 221: case 222:
            // Full Above and Full Below

            this->dHOr = obj->getSize().Zsize/2+TOL_GRIP;

            break;

        }

        break;


    case 2: // left arm



        this->leftFinalHand.at(0) = THETA8_FINAL; // spread of F1 and F2
        this->leftFinalPosture.at(JOINTS_ARM)=THETA8_FINAL;

#if HAND==0

        this->leftFinalHand.at(1) = theta;
        this->leftFinalHand.at(2) = theta;
        this->leftFinalHand.at(3) = thetaT;

        this->leftFinalPosture.at(JOINTS_ARM+1)=theta;
        this->leftFinalPosture.at(JOINTS_ARM+2)=theta;
        this->leftFinalPosture.at(JOINTS_ARM+3)=thetaT;

        this->leftFinalPosture_diseng.at(JOINTS_ARM+1)=theta;
        this->leftFinalPosture_diseng.at(JOINTS_ARM+2)=theta;
        this->leftFinalPosture_diseng.at(JOINTS_ARM+3)=thetaT;

        this->leftFinalPosture_eng.at(JOINTS_ARM+1)=theta;
        this->leftFinalPosture_eng.at(JOINTS_ARM+2)=theta;
        this->leftFinalPosture_eng.at(JOINTS_ARM+3)=thetaT;


#elif HAND==1

        // we consider:
        // 1. the spread of the fingers F1 and F2 always at home ( theta8 = THETA8_final)
        // 2. the displacement of the other joints is equal (theta9=theta10=theta11)

        for (int i = 0; i<HAND_FINGERS; ++i){
             this->leftFinalHand.at(i+1) = theta;
             this->leftFinalPosture.at(JOINTS_ARM+i+1)=theta;
        }

#endif

        switch (grip_code) {
        case 111: case 112: case 113: case 114: case 121: case 122:
            //Precision grip

             this->dHOl = this->dFH;

            break;
        case 211: case 212: case 213: case 214:
            // Full Side grip

            this->dHOl = obj->getRadius()+TOL_GRIP;

            break;
        case 221: case 222:
            // Full Above and Full Below

            this->dHOl = obj->getSize().Zsize/2+TOL_GRIP;

            break;

        }

        break;

    }



    return success;


}


bool Problem::invKinHand(float d_obj,int hand_id,std::vector<float>& sols)
{

    humanoidPtr hh = this->scene->getHumanoid();

    bool success = false;
    sols = std::vector<float>(2);
    float theta;
    float thetaT;


#if HAND==0

    human_hand hand = hh->getHumanHand();
    human_finger middle = hand.fingers.at(1);
    human_thumb thumb = hand.thumb;
    float maxAp = hand.maxAperture;
    int k;

    // middle finger
    //float ux = middle.ux;
    float uy;
    float uz = middle.uz;
    float Lp = middle.finger_specs.a.at(1);
    float Lmi = middle.finger_specs.a.at(2);
    float Ld = middle.finger_specs.a.at(3);
    float alpha0;
    float theta0;

    // thumb
    //float uTx = thumb.uTx;
    float uTy;
    //float uTz = thumb.uTz;
    float LTm = thumb.thumb_specs.a.at(2);
    float LTp = thumb.thumb_specs.a.at(3);
    float LTd = thumb.thumb_specs.a.at(4);
    float alpha0T;
    float alpha1T = thumb.thumb_specs.alpha.at(1);
    float theta0T = thumb.thumb_specs.theta.at(0);
    float theta1T = THETA8_FINAL;

    switch(hand_id){

    case 1: // right hand
        k=1;
        uy = middle.uy;
        alpha0 = middle.finger_specs.alpha.at(0);
        theta0 = middle.finger_specs.theta.at(0);
        uTy = thumb.uTy;
        alpha0T = thumb.thumb_specs.alpha.at(0);
        break;
    case 2: // left hand
        k=-1;
        uy = -middle.uy;
        alpha0 = -middle.finger_specs.alpha.at(0);
        theta0 = -middle.finger_specs.theta.at(0);
        uTy = -thumb.uTy;
        alpha0T = thumb.thumb_specs.alpha.at(0)-90*M_PI/180;
        break;
    }


#elif HAND==1

    float A1 = hh->getBarrettHand().A1;
    float A2 = hh->getBarrettHand().A2;
    float A3 = hh->getBarrettHand().A3;
    float D3 = hh->getBarrettHand().D3;
    float phi2 = hh->getBarrettHand().phi2;
    float phi3 = hh->getBarrettHand().phi3;
    float maxAp = hh->getBarrettHand().maxAperture;
    float fnew; // dFF
    float dfnew;// ddFF

    float x0 = 60.0* M_PI/180.0; // initial approximation
    float xold = x0;
    theta = xold;
    float xnew = 140.0* M_PI/180.0;

#endif

    if (d_obj > maxAp){ throw string(" the object is too big to be grasped");}
    int cnt=0;


#if HAND==0

    // initial approximation
    float xold = 30.0* M_PI/180.0;
    float xoldT = k*30.0* M_PI/180.0;
    theta = xold;
    thetaT = xoldT;
    float xnew = 140.0* M_PI/180.0;
    float xnewT = k*140.0* M_PI/180.0;
    float fnew;
    float dfnew;
    float fnewT;
    float dfnewT;
    float dMH;
    float dTH;

    while((abs(xnew-xold)>1e-4 || abs(xnewT-xoldT)>1e-4) && cnt <100){
        cnt++;

        xold=theta;
        xoldT=thetaT;

        // fnew = k*P_middle(2) - d_obj/2;
        fnew = k*(uy
                 -Ld*cos(theta/3)*(sin((2*theta)/3)*(sin(theta0)*sin(theta) - cos(alpha0)*cos(theta0)*cos(theta)) - cos((2*theta)/3)*(cos(theta)*sin(theta0) + cos(alpha0)*cos(theta0)*sin(theta)))
                 +Lmi*cos((2*theta)/3)*(cos(theta)*sin(theta0) + cos(alpha0)*cos(theta0)*sin(theta))
                 -Ld*sin(theta/3)*(sin((2*theta)/3)*(cos(theta)*sin(theta0) + cos(alpha0)*cos(theta0)*sin(theta)) + cos((2*theta)/3)*(sin(theta0)*sin(theta) - cos(alpha0)*cos(theta0)*cos(theta)))
                 -Lmi*sin((2*theta)/3)*(sin(theta0)*sin(theta) - cos(alpha0)*cos(theta0)*cos(theta))
                 +Lp*cos(theta)*sin(theta0) +Lp*cos(alpha0)*cos(theta0)*sin(theta))
                  -d_obj/2;

        dfnew = -k*((Ld*cos(theta/3)*(sin((2*theta)/3)*(cos(theta)*sin(theta0) + cos(alpha0)*cos(theta0)*sin(theta)) + cos((2*theta)/3)*(sin(theta0)*sin(theta) - cos(alpha0)*cos(theta0)*cos(theta))))/3
                    +Ld*cos(theta/3)*((5*sin((2*theta)/3)*(cos(theta)*sin(theta0) + cos(alpha0)*cos(theta0)*sin(theta)))/3 + (5*cos((2*theta)/3)*(sin(theta0)*sin(theta)-cos(alpha0)*cos(theta0)*cos(theta)))/3)
                    +(5*Lmi*cos((2*theta)/3)*(sin(theta0)*sin(theta) - cos(alpha0)*cos(theta0)*cos(theta)))/3
                    -(Ld*sin(theta/3)*(sin((2*theta)/3)*(sin(theta0)*sin(theta) - cos(alpha0)*cos(theta0)*cos(theta)) - cos((2*theta)/3)*(cos(theta)*sin(theta0) + cos(alpha0)*cos(theta0)*sin(theta))))/3
                    -Ld*sin(theta/3)*((5*sin((2*theta)/3)*(sin(theta0)*sin(theta) - cos(alpha0)*cos(theta0)*cos(theta)))/3 - (5*cos((2*theta)/3)*(cos(theta)*sin(theta0) + cos(alpha0)*cos(theta0)*sin(theta)))/3)
                    +(5*Lmi*sin((2*theta)/3)*(cos(theta)*sin(theta0) + cos(alpha0)*cos(theta0)*sin(theta)))/3
                    +Lp*sin(theta0)*sin(theta) -Lp*cos(alpha0)*cos(theta0)*cos(theta));

        // fnewT = k*P_thumb(2) + d_obj/2;
        fnewT = -k*(LTp*sin((11*thetaT)/10)*(cos(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T)+cos(alpha1T)*sin(theta0T)*sin(theta1T)-cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T))+sin(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T)))
                 -uTy
                 +LTm*sin(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T))
                 +LTd*cos((11*thetaT)/12)*(cos((11*thetaT)/10)*(sin(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) - cos(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))) + sin((11*thetaT)/10)*(cos(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) + sin(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))))
                 +LTd*sin((11*thetaT)/12)*(cos((11*thetaT)/10)*(cos(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) + sin(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))) - sin((11*thetaT)/10)*(sin(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) - cos(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))))
                 -LTm*cos(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))
                 +LTp*cos((11*thetaT)/10)*(sin(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) - cos(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))))
                 +d_obj/2;

        dfnewT = -k*(LTm*cos(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T))
                     -(21*LTp*sin((11*thetaT)/10)*(sin(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) - cos(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))))/10
                     +(11*LTd*cos((11*thetaT)/12)*(cos((11*thetaT)/10)*(cos(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) + sin(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T)))
                                                   -sin((11*thetaT)/10)*(sin(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) - cos(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T)))))/12
                     +LTd*cos((11*thetaT)/12)*((21*cos((11*thetaT)/10)*(cos(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) + sin(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))))/10
                                               -(21*sin((11*thetaT)/10)*(sin(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) - cos(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))))/10)
                     -(11*LTd*sin((11*thetaT)/12)*(cos((11*thetaT)/10)*(sin(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) - cos(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T)))
                                                   +sin((11*thetaT)/10)*(cos(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) + sin(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T)))))/12
                     -LTd*sin((11*thetaT)/12)*((21*cos((11*thetaT)/10)*(sin(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) - cos(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))))/10
                                               +(21*sin((11*thetaT)/10)*(cos(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) + sin(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))))/10)
                     +(21*LTp*cos((11*thetaT)/10)*(cos(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) + sin(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))))/10
                     +LTm*sin(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T)));



        xnew=xold-fnew/dfnew;
        theta = xnew;

        xnewT=xoldT-fnewT/dfnewT;
        thetaT = xnewT;

    }
    if (cnt < 100){success = true;}

    theta=xnew;
    thetaT=abs(xnewT);


    dMH = k*(uy
             -Ld*cos(theta/3)*(sin((2*theta)/3)*(sin(theta0)*sin(theta) - cos(alpha0)*cos(theta0)*cos(theta)) - cos((2*theta)/3)*(cos(theta)*sin(theta0) + cos(alpha0)*cos(theta0)*sin(theta)))
             +Lmi*cos((2*theta)/3)*(cos(theta)*sin(theta0) + cos(alpha0)*cos(theta0)*sin(theta))
             -Ld*sin(theta/3)*(sin((2*theta)/3)*(cos(theta)*sin(theta0) + cos(alpha0)*cos(theta0)*sin(theta)) + cos((2*theta)/3)*(sin(theta0)*sin(theta) - cos(alpha0)*cos(theta0)*cos(theta)))
             -Lmi*sin((2*theta)/3)*(sin(theta0)*sin(theta) - cos(alpha0)*cos(theta0)*cos(theta))
             +Lp*cos(theta)*sin(theta0) +Lp*cos(alpha0)*cos(theta0)*sin(theta));

    dTH =k*(LTp*sin((11*thetaT)/10)*(cos(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T)+cos(alpha1T)*sin(theta0T)*sin(theta1T)-cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T))+sin(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T)))
             -uTy
             +LTm*sin(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T))
             +LTd*cos((11*thetaT)/12)*(cos((11*thetaT)/10)*(sin(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) - cos(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))) + sin((11*thetaT)/10)*(cos(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) + sin(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))))
             +LTd*sin((11*thetaT)/12)*(cos((11*thetaT)/10)*(cos(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) + sin(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))) - sin((11*thetaT)/10)*(sin(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) - cos(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))))
             -LTm*cos(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))
             +LTp*cos((11*thetaT)/10)*(sin(thetaT)*(sin(alpha0T)*sin(alpha1T)*cos(theta0T) + cos(alpha1T)*sin(theta0T)*sin(theta1T) - cos(alpha0T)*cos(alpha1T)*cos(theta0T)*cos(theta1T)) - cos(thetaT)*(cos(theta1T)*sin(theta0T) + cos(alpha0T)*cos(theta0T)*sin(theta1T))));


    this->dFF = dMH + dTH;

    this->dFH =uz+Lp*sin(alpha0)*sin(theta) + Ld*cos(theta/3)*(cos((2*theta)/3)*sin(alpha0)*sin(theta) + sin((2*theta)/3)*sin(alpha0)*cos(theta))
              +Ld*sin(theta/3)*(cos((2*theta)/3)*sin(alpha0)*cos(theta) - sin((2*theta)/3)*sin(alpha0)*sin(theta))
              +Lmi*cos((2*theta)/3)*sin(alpha0)*sin(theta)
              +Lmi*sin((2*theta)/3)*sin(alpha0)*cos(theta);

    sols.at(0)=theta;
    sols.at(1)=thetaT;


#elif HAND==1
    while(abs(xnew-xold)>1e-4 && cnt <100){

        cnt++;
        xold=theta;
        fnew=2*cos((4/3)*theta+phi2+phi3)*A3-2*sin((4/3)*theta+phi2+phi3)*D3+
                2*cos(theta+phi2)*A2+2*A1-d_obj;
        dfnew=-(8/3)*sin((4/3)*theta+phi2+phi3)*A3-(8/3)*cos((4/3)*theta+phi2+phi3)*D3-
                2*sin(theta+phi2)*A2;
        xnew=xold-fnew/dfnew;
        theta = xnew;


    }
    if (cnt < 100){success = true;}

    theta=xnew;
    this->dFF = 2*cos((4/3)*theta+phi2+phi3)*A3-2*sin((4/3)*theta+phi2+phi3)*D3+
            2*cos(theta+phi2)*A2+2*A1;
    this->dFH = sin((4/3)*theta+phi2+phi3)*A3+cos((4/3)*theta+phi2+phi3)*D3+sin(theta+phi2)*A2;

    sols.at(0)=theta;
    sols.at(1)=theta;

#endif


    return success;

}

bool Problem::amplRead(string &datFile, string &modFile, string &nlFile)
{

    string cmdLine;

#if AMPL==0
    cmdLine = string("wine ")+AMPL_PATH+string("/ampl.exe -ogModels/")+nlFile+string(" Models/")+modFile+string(".mod")+
            string(" Models/")+datFile+string(".dat")+string(" Models/options.run");
#elif AMPL==1
    cmdLine = AMPL_PATH+string("/./ampl -ogModels/")+nlFile+string(" Models/")+modFile+string(".mod")+
            string(" Models/")+datFile+string(".dat")+string(" Models/options.run");
#endif
    int status = system(cmdLine.c_str());

    return (status >= 0);

}

bool Problem::optimize(string &nlfile, std::vector<Number>& x,
                       float tol, float acc_tol)
{

    // Create a new instance of IpoptApplication
    //  (use a SmartPtr, not raw)
    // We are using the factory, since this allows us to compile this
    // example with an Ipopt Windows DLL
    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();
    app->RethrowNonIpoptException(true);

    app->Options()->SetNumericValue("tol", tol);
    app->Options()->SetNumericValue("acceptable_tol", acc_tol);
    app->Options()->SetStringValue("output_file", "ipopt.out");
    app->Options()->SetStringValue("hessian_approximation", "limited-memory");
    app->Options()->SetIntegerValue("print_level",3);

    // Initialize the IpoptApplication and process the options
    ApplicationReturnStatus status;
    status = app->Initialize();
    if (status != Solve_Succeeded) {
      std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
      return (int) status;
    }

    // Add the suffix handler for scaling
    SmartPtr<AmplSuffixHandler> suffix_handler = new AmplSuffixHandler();
    suffix_handler->AddAvailableSuffix("scaling_factor", AmplSuffixHandler::Variable_Source, AmplSuffixHandler::Number_Type);
    suffix_handler->AddAvailableSuffix("scaling_factor", AmplSuffixHandler::Constraint_Source, AmplSuffixHandler::Number_Type);
    suffix_handler->AddAvailableSuffix("scaling_factor", AmplSuffixHandler::Objective_Source, AmplSuffixHandler::Number_Type);
    // Modified for warm-start from AMPL
    suffix_handler->AddAvailableSuffix("ipopt_zL_out", AmplSuffixHandler::Variable_Source, AmplSuffixHandler::Number_Type);
    suffix_handler->AddAvailableSuffix("ipopt_zU_out", AmplSuffixHandler::Variable_Source, AmplSuffixHandler::Number_Type);
    suffix_handler->AddAvailableSuffix("ipopt_zL_in", AmplSuffixHandler::Variable_Source, AmplSuffixHandler::Number_Type);
    suffix_handler->AddAvailableSuffix("ipopt_zU_in", AmplSuffixHandler::Variable_Source, AmplSuffixHandler::Number_Type);

    char *cstr = new char[nlfile.length() + 1];
    strcpy(cstr, nlfile.c_str());

    SmartPtr<AmplInterface> ampl_tnlp = new AmplInterface(ConstPtr(app->Jnlst()),
                                            app->Options(),
                                            cstr, suffix_handler);


    delete [] cstr;

    // Ask Ipopt to solve the problem
    status = app->OptimizeTNLP(ampl_tnlp);
    std::vector<Number> x_sol;
    std::vector<Number> z_L_sol;
    std::vector<Number> z_U_sol;
    std::vector<Number> lambda_sol;
    Number obj_sol;

    if (ampl_tnlp->get_status() == SolverReturn::SUCCESS){



        ampl_tnlp->get_solutions(x_sol,
                                 z_L_sol,
                                 z_U_sol,
                                 lambda_sol,
                                 obj_sol);



        x=x_sol;

        return true;

    }else{
        x=x_sol;

        return false;

    }
}


void Problem::directMovement(std::vector<float> initPosture,
                             std::vector<float> finalPosture,
                             int steps,
                             MatrixXf& Traj)
{

float delta = 1.0/steps;
std::vector<float> time;

std::vector<float> vel_0 = this->tolerances.bounds.vel_0;
std::vector<float> vel_f = this->tolerances.bounds.vel_f;
std::vector<float> acc_0 = this->tolerances.bounds.acc_0;
std::vector<float> acc_f = this->tolerances.bounds.acc_f;
float T = 1.0;

time.push_back(0.0);
for (int i = 0; i<steps; ++i){
    time.push_back(time.at(i)+delta);
}
Traj = MatrixXf::Constant(steps+1,initPosture.size(),0);

for (int i = 0; i <= steps;++i){
    for (std::size_t j = 0; j<initPosture.size(); ++j){
        Traj(i,j) = initPosture.at(j) +
                (initPosture.at(j) - finalPosture.at(j))*
                (15*pow(time.at(i),4)-6*pow(time.at(i),5)-10*pow(time.at(i),3))+
                vel_0.at(j)*T*(time.at(i)-6*pow(time.at(i),3)+8*pow(time.at(i),4)-3*pow(time.at(i),5))+
                vel_f.at(j)*T*(-4*pow(time.at(i),3)+7*pow(time.at(i),4)-3*pow(time.at(i),5))+
                0.5*acc_0.at(j)*pow(T,2)*(pow(time.at(i),2)-3*pow(time.at(i),3)+3*pow(time.at(i),4)-pow(time.at(i),5))+
                0.5*acc_f.at(j)*pow(T,2)*(pow(time.at(i),3)-2*pow(time.at(i),4)+pow(time.at(i),5));
    }
}



}

void Problem::backForthMovement(std::vector<float> initPosture,
                                std::vector<float> bouncePosture,
                                int steps,
                                MatrixXf &Traj)
{

    float delta = 1.0/steps;
    std::vector<float> time;

    float tb = 0.5; // tb = [0,1], it constrols when the bounce posture is reached. 0.5 means at the middle of the movement

    time.push_back(0.0);
    for (int i = 0; i<steps; ++i){
        time.push_back(time.at(i)+delta);
    }
    Traj = MatrixXf::Constant(steps+1,initPosture.size(),0);

    for (int i = 0; i<steps;++i){ // the last row is zero for all the joints
        for (std::size_t j = 0; j<initPosture.size(); ++j){
              Traj(i,j) = (bouncePosture.at(j) - initPosture.at(j))* pow(sin(M_PI*time.at(i)),(-log(2.0)/log(tb)));
        }
    }

}


void Problem::computeTraj(const MatrixXf& dTraj, const MatrixXf& bTraj)
{


    this->optimalTraj = MatrixXf::Constant(dTraj.rows(),dTraj.cols(),0);

    for (int i = 0; i<dTraj.rows();++i){
        for (int j = 0; j<dTraj.cols(); ++j){

            this->optimalTraj(i,j) = dTraj(i,j)+bTraj(i,j);

        }
    }



}

float Problem::getTrajectory(MatrixXf& traj)
{

    int mov_type = this->getMovement()->getType();

    MatrixXf dTraj;
    MatrixXf dTraj_diseng;
    MatrixXf dTraj_eng;
    MatrixXf bTraj;
    MatrixXf totTraj;
    std::vector<float> initPosture;
    std::vector<float> finalPosture;
    std::vector<float> bouncePosture;
    std::vector<float> fPosture_diseng;
    std::vector<float> fPosture_eng;
    int arm_code =  this->mov->getArm();
    int steps = this->tolerances.steps;

    float timeStep;

    switch (mov_type) {

    case 0: // reach-to-grasp

        switch(arm_code){

        case 0: // both arms

            // TO DO
            break;

        case 1: // right arm
            this->scene->getHumanoid()->getRightPosture(initPosture);
            finalPosture=this->rightFinalPosture;
            bouncePosture=this->rightBouncePosture;
            break;
        case 2: // left arm
            this->scene->getHumanoid()->getLeftPosture(initPosture);
            finalPosture=this->leftFinalPosture;
            bouncePosture=this->leftBouncePosture;
            break;
        }

        this->directMovement(initPosture,finalPosture,steps,dTraj);
        this->backForthMovement(initPosture,bouncePosture,steps,bTraj);
        this->computeTraj(dTraj,bTraj);

        traj = this->optimalTraj;

        this->getTimeStep(traj,timeStep);

        break;

    case 1: // reaching

        break;

    case 2: // transport

        break;

    case 3: // engage

        switch(arm_code){

        case 0: // both arms

            // TO DO
            break;

        case 1: // right arm
            this->scene->getHumanoid()->getRightPosture(initPosture);
            finalPosture=this->rightFinalPosture;
            fPosture_diseng=this->rightFinalPosture_diseng;
            fPosture_eng=this->rightFinalPosture_eng;
            bouncePosture=this->rightBouncePosture;
            break;
        case 2: // left arm
            this->scene->getHumanoid()->getLeftPosture(initPosture);
            finalPosture=this->leftFinalPosture;
            fPosture_diseng=this->leftFinalPosture_diseng;
            fPosture_eng=this->leftFinalPosture_eng;
            bouncePosture=this->leftBouncePosture;
            break;
        }

        this->directMovement(initPosture,fPosture_diseng,4,dTraj_diseng);
        this->directMovement(fPosture_diseng,fPosture_eng,steps,dTraj);
        this->backForthMovement(fPosture_diseng,bouncePosture,steps,bTraj);
        this->computeTraj(dTraj,bTraj);
        this->directMovement(fPosture_eng,finalPosture,4,dTraj_eng);

        // compose the trajectory
        totTraj.resize(this->optimalTraj.rows()+10,this->optimalTraj.cols());

        totTraj.row(0) = dTraj_diseng.row(0);
        totTraj.row(1) = dTraj_diseng.row(1);
        totTraj.row(2) = dTraj_diseng.row(2);
        totTraj.row(3) = dTraj_diseng.row(3);
        totTraj.row(4) = dTraj_diseng.row(4);

        for(int i =0; i < this->optimalTraj.rows(); ++i){
            totTraj.row(i+5) = this->optimalTraj.row(i);
        }

        totTraj.row(totTraj.rows()-5) = dTraj_eng.row(0);
        totTraj.row(totTraj.rows()-4) = dTraj_eng.row(1);
        totTraj.row(totTraj.rows()-3) = dTraj_eng.row(2);
        totTraj.row(totTraj.rows()-2) = dTraj_eng.row(3);
        totTraj.row(totTraj.rows()-1) = dTraj_eng.row(4);

        this->optimalTraj=totTraj;
        traj = this->optimalTraj;

        this->getTimeStep(traj,timeStep);

        break;

    case 4: // disengage

        break;

    case 5: // Go park

        switch (arm_code) {

        case 0: // both arms

            break;

        case 1: // right arm
            this->scene->getHumanoid()->getRightPosture(initPosture);
            this->scene->getHumanoid()->getRightHomePosture(finalPosture);
            bouncePosture=this->rightBouncePosture;
            break;

        case 2: // left arm
            this->scene->getHumanoid()->getLeftPosture(initPosture);
            this->scene->getHumanoid()->getLeftHomePosture(finalPosture);
            bouncePosture=this->rightBouncePosture;
            break;
        }
        if(std::strcmp(this->getMovement()->getObject()->getName().c_str(),"")!=0){
            // an object has to be released
            initPosture.at(7) = 0.0;
            initPosture.at(8) = 0.0;
            initPosture.at(9) = 0.0;
            initPosture.at(10) = 0.0;
        }
        this->directMovement(initPosture,finalPosture,steps,dTraj);
        this->backForthMovement(initPosture,bouncePosture,steps,bTraj);
        this->computeTraj(dTraj,bTraj);

        traj = this->optimalTraj;

        this->getTimeStep(traj,timeStep);

        break;
    }

    return timeStep;

}


float Problem::getVelocity(MatrixXf &vel)
{

    MatrixXf traj;
    this->getTrajectory(traj);
    vel.resize(traj.rows(),traj.cols());

    std::vector<float> delta;
    float timeStep;
    VectorXf posture;

    for (int k = 0; k < traj.cols(); ++k){

        posture = traj.col(k);
        this->getDelta(posture,delta);
        this->getTimeStep(traj,timeStep);

        for (int i =0; i < traj.rows(); ++i){
            vel(i,k) = delta.at(i)/timeStep;
        }
    }

    return timeStep;

}

movementPtr Problem::getMovement()
{

    return this->mov;
}

void Problem::getDelta(VectorXf  jointTraj, std::vector<float> &delta)
{


    // Formula of the numarical differentiation with 5 points
    // f'0 = (-25*f0 + 48*f1 - 36*f2 + 16*f3 -  3*f4)/(12*h) + h^4/5*f^(5)(c_0)
    // f'1 = ( -3*f0 - 10*f1 + 18*f2 -  6*f3 +  1*f4)/(12*h) - h^4/20*f^(5)(c_1)
    // f'2 = (  1*f0 -  8*f1         +  8*f3 -  1*f4)/(12*h) + h^4/30*f^(5)(c_2)
    // f'3 = ( -1*f0 +  6*f1 - 18*f2 + 10*f3 +  3*f4)/(12*h) - h^4/20*f^(5)(c_3)
    // f'4 = (  3*f0 - 16*f1 + 36*f2 - 48*f3 + 25*f4)/(12*h) + h^4/5*f^(5)(c_4)

    delta = std::vector<float>(jointTraj.size());

    int h = 1;
    int tnsample;
    float f0;
    float f1;
    float f2;
    float f3;
    float f4;

    // 1st point
    // f'0 = (-25*f0 + 48*f1 - 36*f2 + 16*f3 -  3*f4)/(12*h) + h^4/5*f^(5)(c_0)
    tnsample = 0;
    f0 = jointTraj(tnsample);
    f1 = jointTraj(tnsample+1);
    f2 = jointTraj(tnsample+2);
    f3 = jointTraj(tnsample+3);
    f4 = jointTraj(tnsample+4);
    delta.at(tnsample) = (-25*f0 + 48*f1 - 36*f2 + 16*f3 -  3*f4)/(12*h);

    // 2nd point
    // f'1 = ( -3*f0 - 10*f1 + 18*f2 -  6*f3 +  1*f4)/(12*h) - h^4/20*f^(5)(c_1)
    tnsample = 1;
    f0 = jointTraj(tnsample-1);
    f1 = jointTraj(tnsample);
    f2 = jointTraj(tnsample+1);
    f3 = jointTraj(tnsample+2);
    f4 = jointTraj(tnsample+3);
    delta.at(tnsample) = ( -3*f0 - 10*f1 + 18*f2 -  6*f3 +  1*f4)/(12*h);

    // 3rd point
    // f'2 = (  1*f0 -  8*f1         +  8*f3 -  1*f4)/(12*h) + h^4/30*f^(5)(c_2)
    for (int i=2; i< jointTraj.size() -2;++i){     // centered
        f0 = jointTraj(i-2);
        f1 = jointTraj(i-1);
        f2 = jointTraj(i);
        f3 = jointTraj(i+1);
        f4 = jointTraj(i+2);
        delta.at(i) = (  1*f0 -  8*f1         +  8*f3 -  1*f4)/(12*h);
    }

    // 4th point
    // f'3 = ( -1*f0 +  6*f1 - 18*f2 + 10*f3 +  3*f4)/(12*h) - h^4/20*f^(5)(c_3)
    tnsample = jointTraj.size()-2;
    f0 = jointTraj(tnsample-3);
    f1 = jointTraj(tnsample-2);
    f2 = jointTraj(tnsample-1);
    f3 = jointTraj(tnsample);
    f4 = jointTraj(tnsample+1);
    delta.at(tnsample) = ( -f0+6*f1-18*f2+10*f3+3*f4)/(12*h);

    // 5th point
    // f'4 = (  3*f0 - 16*f1 + 36*f2 - 48*f3 + 25*f4)/(12*h) + h^4/5*f^(5)(c_4)
    tnsample = jointTraj.size()-1;
    f0 = jointTraj(tnsample-4);
    f1 = jointTraj(tnsample-3);
    f2 = jointTraj(tnsample-2);
    f3 = jointTraj(tnsample-1);
    f4 = jointTraj(tnsample);
    delta.at(tnsample) = (  3*f0 - 16*f1 + 36*f2 - 48*f3 + 25*f4)/(12*h);

}



void Problem::getTimeStep(MatrixXf jointTraj, float& timeStep)
{

int steps1 = jointTraj.rows();
int n_joints = jointTraj.cols();

std::vector<float> w_max = this->tolerances.w_max;
std::vector<float> lambda = this->tolerances.lambda_bounce;



float num = 0;
float den = 0;

for (int k =0; k < n_joints; ++k){

    float time_k;
    float deltaTheta = 0;
    std::vector<float> diffs;
    for (int i = 1; i < steps1; ++i){

        float diff = abs(jointTraj(i,k) - jointTraj(i-1,k))*180/M_PI;
        diffs.push_back(diff);
        deltaTheta = deltaTheta + diff;


    }
    std::vector<float>::iterator res = std::max_element(diffs.begin(),diffs.end());
    int poss = std::distance(diffs.begin(),res);
    float deltaThetaMax = diffs.at(poss);
    float tol = -3.0; // tolerance in [deg/sec]
    time_k = ((steps1-1)*deltaThetaMax/(w_max.at(k)+tol)) + (lambda.at(k)*log(1+deltaTheta));

    num = num + lambda.at(k)*deltaTheta*time_k;
    den = den + lambda.at(k)*deltaTheta;

}

float totalTime = num/den;

timeStep = (totalTime/(steps1-1));

}

void Problem::setApproachingTargetAxis(int a)
{

    this->targetAxis = a;

}



void Problem::write_dHO(std::ofstream& stream, float dHO)
{

    string dHOstr=  boost::str(boost::format("%.2f") % (dHO));
    boost::replace_all(dHOstr,",",".");
    stream << string("# DISTANCE HAND - TARGET\n");
    stream << string("param dFH := ")+dHOstr+string(";\n");
}


void Problem::writeBodyDim(humanoidPtr hh, ofstream &stream)
{

    string bodyxsize=  boost::str(boost::format("%.2f") % (hh->getSize().Xsize/2));
    boost::replace_all(bodyxsize,",",".");
    string bodyysize=  boost::str(boost::format("%.2f") % (hh->getSize().Ysize/2));
    boost::replace_all(bodyysize,",",".");

    stream << string("# BODY INFO \n");
    stream << string("param body := \n");
    stream << to_string(1)+string(" ")+bodyxsize+string("\n");
    stream << to_string(2)+string(" ")+bodyysize+string(";\n");
}


void Problem::writeArmDHParams(humanoidPtr hh, ofstream &stream, int k)
{

    // D-H Parameters of the Arm
    stream << string("# D-H PARAMETERS OF THE ARM \n");
    stream << string("param alpha := \n");
    string alpha_str;
    alpha_str =  boost::str(boost::format("%.2f") % (hh->getArm().arm_specs.alpha.at(0)));
    stream << to_string(1)+string(" ")+alpha_str+string("\n");
    for(std::size_t i=1; i<HUMotion::JOINTS_ARM; ++i){
        alpha_str =  boost::str(boost::format("%.2f") % (k*hh->getArm().arm_specs.alpha.at(i)));

        if (i == hh->getArm().arm_specs.alpha.size()-1){
            stream << to_string(i+1)+string(" ")+alpha_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+alpha_str+string("\n");
        }
    }
    stream << string("param a := \n");
    string a_str;
    for(std::size_t i=0; i<HUMotion::JOINTS_ARM; ++i){
        a_str =  boost::str(boost::format("%.2f") % (hh->getArm().arm_specs.a.at(i)));
        if (i == hh->getArm().arm_specs.a.size()-1){
            stream << to_string(i+1)+string(" ")+a_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+a_str+string("\n");
        }
    }
    stream << string("param d := \n");
    string d_str;
    for(std::size_t i=0; i<HUMotion::JOINTS_ARM; ++i){
        d_str =  boost::str(boost::format("%.2f") % (k*hh->getArm().arm_specs.d.at(i)));
        if (i == hh->getArm().arm_specs.d.size()-1){
            stream << to_string(i+1)+string(" ")+d_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+d_str+string("\n");
        }
    }

}

//#if HAND==0


void Problem::writeHumanHandParams(humanoidPtr hh, ofstream &stream, int k)
{

    stream << string("# PARAMETERS OF THE HAND \n");

    // Index finger
    stream << string("# Index Finger \n");
    human_finger finger_1 = hh->getHumanHand().fingers.at(0);

    stream << string("param u1x := ");
    string ux_fing1_str =  boost::str(boost::format("%.2f") % (finger_1.ux));
    stream << ux_fing1_str+string(";\n");

    stream << string("param u1y := ");
    string uy_fing1_str =  boost::str(boost::format("%.2f") % (k*finger_1.uy));
    stream << uy_fing1_str+string(";\n");

    stream << string("param u1z := ");
    string uz_fing1_str =  boost::str(boost::format("%.2f") % (finger_1.uz));
    stream << uz_fing1_str+string(";\n");

    stream << string("param alpha_fing1 := \n");
    string alpha_fing1_str;
    alpha_fing1_str =  boost::str(boost::format("%.2f") % (k*finger_1.finger_specs.alpha.at(0)));
    stream << to_string(1)+string(" ")+alpha_fing1_str+string("\n");
    for(std::size_t i=1; i<HUMotion::N_PHALANGE+1; ++i){
        alpha_fing1_str =  boost::str(boost::format("%.2f") % (finger_1.finger_specs.alpha.at(i)));
        if (i == finger_1.finger_specs.alpha.size()-1){
            stream << to_string(i+1)+string(" ")+alpha_fing1_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+alpha_fing1_str+string("\n");
        }
    }
    stream << string("param a_fing1 := \n");
    string a_fing1_str;
    for(std::size_t i=0; i<HUMotion::N_PHALANGE+1; ++i){
        a_fing1_str =  boost::str(boost::format("%.2f") % (finger_1.finger_specs.a.at(i)));
        if (i == finger_1.finger_specs.a.size()-1){
            stream << to_string(i+1)+string(" ")+a_fing1_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+a_fing1_str+string("\n");
        }
    }
    stream << string("param d_fing1 := \n");
    string d_fing1_str;
    for(std::size_t i=0; i<HUMotion::N_PHALANGE+1; ++i){
        d_fing1_str =  boost::str(boost::format("%.2f") % (finger_1.finger_specs.d.at(i)));
        if (i == finger_1.finger_specs.d.size()-1){
            stream << to_string(i+1)+string(" ")+d_fing1_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+d_fing1_str+string("\n");
        }
    }
    stream << string("param theta0_fing1 := ");
    string theta0_fing1_str;
    theta0_fing1_str =  boost::str(boost::format("%.2f") % (k*finger_1.finger_specs.theta.at(0)));
    stream << theta0_fing1_str+string(";\n");

    // Ring finger
    stream << string("# Ring Finger \n");
    human_finger finger_3 = hh->getHumanHand().fingers.at(2);

    stream << string("param u3x := ");
    string ux_fing3_str =  boost::str(boost::format("%.2f") % (finger_3.ux));
    stream << ux_fing3_str+string(";\n");

    stream << string("param u3y := ");
    string uy_fing3_str =  boost::str(boost::format("%.2f") % (k*finger_3.uy));
    stream << uy_fing3_str+string(";\n");

    stream << string("param u3z := ");
    string uz_fing3_str =  boost::str(boost::format("%.2f") % (finger_3.uz));
    stream << uz_fing3_str+string(";\n");

    stream << string("param alpha_fing3 := \n");
    string alpha_fing3_str;
    alpha_fing3_str =  boost::str(boost::format("%.2f") % (k*finger_3.finger_specs.alpha.at(0)));
    stream << to_string(1)+string(" ")+alpha_fing3_str+string("\n");
    for(std::size_t i=1; i<HUMotion::N_PHALANGE+1; ++i){
        alpha_fing3_str =  boost::str(boost::format("%.2f") % (finger_3.finger_specs.alpha.at(i)));
        if (i == finger_3.finger_specs.alpha.size()-1){
            stream << to_string(i+1)+string(" ")+alpha_fing3_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+alpha_fing3_str+string("\n");
        }
    }
    stream << string("param a_fing3 := \n");
    string a_fing3_str;
    for(std::size_t i=0; i<HUMotion::N_PHALANGE+1; ++i){
        a_fing3_str =  boost::str(boost::format("%.2f") % (finger_3.finger_specs.a.at(i)));
        if (i == finger_3.finger_specs.a.size()-1){
            stream << to_string(i+1)+string(" ")+a_fing3_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+a_fing3_str+string("\n");
        }
    }
    stream << string("param d_fing3 := \n");
    string d_fing3_str;
    for(std::size_t i=0; i<HUMotion::N_PHALANGE+1; ++i){
        d_fing3_str =  boost::str(boost::format("%.2f") % (finger_3.finger_specs.d.at(i)));
        if (i == finger_3.finger_specs.d.size()-1){
            stream << to_string(i+1)+string(" ")+d_fing3_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+d_fing3_str+string("\n");
        }
    }
    stream << string("param theta0_fing3 := ");
    string theta0_fing3_str;
    theta0_fing3_str =  boost::str(boost::format("%.2f") % (k*finger_3.finger_specs.theta.at(0)));
    stream << theta0_fing3_str+string(";\n");

    // Thumb finger
    stream << string("# Thumb Finger \n");
    human_thumb thumb_finger = hh->getHumanHand().thumb;

    stream << string("param uTx := ");
    string uTx_fing2_str =  boost::str(boost::format("%.2f") % (thumb_finger.uTx));
    stream << uTx_fing2_str+string(";\n");

    stream << string("param uTy := ");
    string uTy_fing2_str =  boost::str(boost::format("%.2f") % (k*thumb_finger.uTy));
    stream << uTy_fing2_str+string(";\n");

    stream << string("param uTz := ");
    string uTz_fing2_str =  boost::str(boost::format("%.2f") % (thumb_finger.uTz));
    stream << uTz_fing2_str+string(";\n");

    stream << string("param alpha_thumb := \n");
    string alpha_thumb_str;
    if (k == 1){ // right hand
        alpha_thumb_str =  boost::str(boost::format("%.2f") % (thumb_finger.thumb_specs.alpha.at(0)));
    }else if(k==-1){// left hand
        alpha_thumb_str =  boost::str(boost::format("%.2f") % (thumb_finger.thumb_specs.alpha.at(0)-M_PI/2));
    }
    stream << to_string(1)+string(" ")+alpha_thumb_str+string("\n");
    for(std::size_t i=1; i<HUMotion::N_PHALANGE+2; ++i){
        alpha_thumb_str =  boost::str(boost::format("%.2f") % (thumb_finger.thumb_specs.alpha.at(i)));
        if (i == thumb_finger.thumb_specs.alpha.size()-1){
            stream << to_string(i+1)+string(" ")+alpha_thumb_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+alpha_thumb_str+string("\n");
        }
    }

    stream << string("param a_thumb := \n");
    string a_thumb_str;
    for(std::size_t i=0; i<HUMotion::N_PHALANGE+2; ++i){
        a_thumb_str =  boost::str(boost::format("%.2f") % (thumb_finger.thumb_specs.a.at(i)));
        if (i == thumb_finger.thumb_specs.a.size()-1){
            stream << to_string(i+1)+string(" ")+a_thumb_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+a_thumb_str+string("\n");
        }
    }

    stream << string("param d_thumb := \n");
    string d_thumb_str;
    for(std::size_t i=0; i<HUMotion::N_PHALANGE+2; ++i){
        d_thumb_str =  boost::str(boost::format("%.2f") % (thumb_finger.thumb_specs.d.at(i)));
        if (i == thumb_finger.thumb_specs.d.size()-1){
            stream << to_string(i+1)+string(" ")+d_thumb_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+d_thumb_str+string("\n");
        }
    }

    stream << string("param theta0_thumb:= ");
    string theta0_thumb_str;
    theta0_thumb_str =  boost::str(boost::format("%.2f") % (thumb_finger.thumb_specs.theta.at(0)));
    stream << theta0_thumb_str+string(";\n");
}


void Problem::writeHumanHandParamsMod(ofstream &stream)
{
    stream << string("# Parameters of the Hand \n");

    stream << string("# Index finger \n");
    stream << string("param u1x; \n");
    stream << string("param u1y; \n");
    stream << string("param u1z; \n");
    stream << string("param alpha_fing1 {i in 1..")+to_string(N_PHALANGE+1)+string("} ; \n");
    stream << string("param a_fing1 {i in 1..")+to_string(N_PHALANGE+1)+string("} ; \n");
    stream << string("param d_fing1 {i in 1..")+to_string(N_PHALANGE+1)+string("} ; \n");
    stream << string("param theta0_fing1; \n");

    stream << string("# Ring finger \n");
    stream << string("param u3x; \n");
    stream << string("param u3y; \n");
    stream << string("param u3z; \n");
    stream << string("param alpha_fing3 {i in 1..")+to_string(N_PHALANGE+1)+string("} ; \n");
    stream << string("param a_fing3 {i in 1..")+to_string(N_PHALANGE+1)+string("} ; \n");
    stream << string("param d_fing3 {i in 1..")+to_string(N_PHALANGE+1)+string("} ; \n");
    stream << string("param theta0_fing3; \n");

    stream << string("# Thumb finger \n");
    stream << string("param uTx; \n");
    stream << string("param uTy; \n");
    stream << string("param uTz; \n");
    stream << string("param alpha_thumb {i in 1..")+to_string(N_PHALANGE+2)+string("} ; \n");
    stream << string("param a_thumb {i in 1..")+to_string(N_PHALANGE+2)+string("} ; \n");
    stream << string("param d_thumb {i in 1..")+to_string(N_PHALANGE+2)+string("} ; \n");
    stream << string("param theta0_thumb; \n");

}


void Problem::writeHumanHandDirKin(ofstream &stream, MatrixXf &tolsHand, bool final, bool transport)
{

    stream << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
    stream << string("#  Direct Kinematics model of the fingers \n\n");

    string tolHand1;
    string tolHand2;
    string tolHand3;
    string tolHand4;

    stream << string("# Index finger \n\n");

    stream << string("param TF1_H_0 {i1 in 1..4, i2 in 1..4} =  \n");
    stream << string("# 1st row \n");
    stream << string("if ( i1=1 && i2=1 ) then cos(theta0_fing1) \n");
    stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta0_fing1)*cos(alpha_fing1[1])  \n");
    stream << string("else	if ( i1=1 && i2=3 ) then sin(theta0_fing1)*sin(alpha_fing1[1])  \n");
    stream << string("else	if ( i1=1 && i2=4 ) then u1x \n");
    stream << string("# 2nd row \n");
    stream << string("else	if ( i1=2 && i2=1 ) then sin(theta0_fing1)  \n");
    stream << string("else	if ( i1=2 && i2=2 ) then cos(theta0_fing1)*cos(alpha_fing1[1])  \n");
    stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta0_fing1)*sin(alpha_fing1[1])  \n");
    stream << string("else	if ( i1=2 && i2=4 ) then u1y  \n");
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing1[1])  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing1[1])  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then u1z  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
    stream << string(";  \n");


    if (final || transport){
        // final posture selection or bounce posture selection for transporting movements
        stream << string("param TF1_0_1 {i1 in 1..4, i2 in 1..4} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(joint_fingers[2]) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(joint_fingers[2])*cos(alpha_fing1[2])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(joint_fingers[2])*sin(alpha_fing1[2])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_fing1[2]*cos(joint_fingers[2]) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(joint_fingers[2])  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(joint_fingers[2])*cos(alpha_fing1[2])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(joint_fingers[2])*sin(alpha_fing1[2])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_fing1[2]*sin(joint_fingers[2])  \n");
    }else{
        // bounce posture selection for reaching movements
        stream << string("var TF1_0_1 {i1 in 1..4, i2 in 1..4,i in Iterations} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(theta[i,9]) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[i,9])*cos(alpha_fing1[2])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(theta[i,9])*sin(alpha_fing1[2])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_fing1[2]*cos(theta[i,9]) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[i,9])  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[i,9])*cos(alpha_fing1[2])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta[i,9])*sin(alpha_fing1[2])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_fing1[2]*sin(theta[i,9])  \n");

    }
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing1[2])  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing1[2])  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then d_fing1[2]  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
    stream << string(";  \n");


    if (final || transport){
        // final posture selection or bounce posture selection for transporting movements
        stream << string("param TF1_1_2 {i1 in 1..4, i2 in 1..4} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(2*joint_fingers[2]/3) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(2*joint_fingers[2]/3)*cos(alpha_fing1[3])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(2*joint_fingers[2]/3)*sin(alpha_fing1[3])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_fing1[3]*cos(2*joint_fingers[2]/3) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(2*joint_fingers[2]/3)  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(2*joint_fingers[2]/3)*cos(alpha_fing1[3])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(2*joint_fingers[2]/3)*sin(alpha_fing1[3])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_fing1[3]*sin(2*joint_fingers[2]/3)  \n");
    }else{
        // bounce posture selection for reaching movements
        stream << string("var TF1_1_2 {i1 in 1..4, i2 in 1..4,i in Iterations} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(2*theta[i,9]/3) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(2*theta[i,9]/3)*cos(alpha_fing1[3])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(2*theta[i,9]/3)*sin(alpha_fing1[3])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_fing1[3]*cos(2*theta[i,9]/3) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(2*theta[i,9]/3)  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(2*theta[i,9]/3)*cos(alpha_fing1[3])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(2*theta[i,9]/3)*sin(alpha_fing1[3])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_fing1[3]*sin(2*theta[i,9]/3)  \n");

    }
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing1[3])  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing1[3])  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then d_fing1[3]  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
    stream << string(";  \n");

    if (final || transport){
        // final posture selection or bounce posture selection for transporting movements
        stream << string("param TF1_2_3 {i1 in 1..4, i2 in 1..4} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(joint_fingers[2]/3) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(joint_fingers[2]/3)*cos(alpha_fing1[4])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(joint_fingers[2]/3)*sin(alpha_fing1[4])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_fing1[4]*cos(joint_fingers[2]/3) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(joint_fingers[2]/3)  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(joint_fingers[2]/3)*cos(alpha_fing1[4])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(joint_fingers[2]/3)*sin(alpha_fing1[4])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_fing1[4]*sin(joint_fingers[2]/3)  \n");
    }else{
        // bounce posture selection for reaching movements
        stream << string("var TF1_2_3 {i1 in 1..4, i2 in 1..4,i in Iterations} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(theta[i,9]/3) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[i,9]/3)*cos(alpha_fing1[4])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(theta[i,9]/3)*sin(alpha_fing1[4])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_fing1[4]*cos(theta[i,9]/3) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[i,9]/3)  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[i,9]/3)*cos(alpha_fing1[4])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta[i,9]/3)*sin(alpha_fing1[4])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_fing1[4]*sin(theta[i,9]/3)  \n");
    }
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing1[4])  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing1[4])  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then d_fing1[4]  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
    stream << string(";  \n");


    // position of the fingers
    tolHand1 =  boost::str(boost::format("%.2f") % tolsHand(0,0)); boost::replace_all(tolHand1,",",".");
    tolHand2 =  boost::str(boost::format("%.2f") % tolsHand(1,0)); boost::replace_all(tolHand2,",",".");
    tolHand3 =  boost::str(boost::format("%.2f") % tolsHand(2,0)); boost::replace_all(tolHand3,",",".");
    tolHand4 =  boost::str(boost::format("%.2f") % tolsHand(3,0)); boost::replace_all(tolHand4,",",".");

    if(final){
        // final posture selection
        stream << string("var F1_0   {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4} T_W_H[i1,j]*TF1_H_0[j,i2]; \n");
        stream << string("var F1_1   {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F1_0[i1,j]*TF1_0_1[j,i2]; \n");
        stream << string("var F1_2   {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F1_1[i1,j]*TF1_1_2[j,i2]; \n");
        stream << string("var F1_tip {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F1_2[i1,j]*TF1_2_3[j,i2]; \n\n");

        stream << string("var Finger1_0   {i1 in 1..4} =  if i1<4 then F1_0[i1,4] 	else ")+tolHand1+string("; \n");
        stream << string("var Finger1_1   {i1 in 1..4} =  if i1<4 then F1_1[i1,4] 	else ")+tolHand2+string("; \n");
        stream << string("var Finger1_2   {i1 in 1..4} =  if i1<4 then F1_2[i1,4] 	else ")+tolHand3+string("; \n");
        stream << string("var Finger1_tip {i1 in 1..4} =  if i1<4 then F1_tip[i1,4] else ")+tolHand4+string("; \n\n");
    }else{
        //bounce posture selection
        stream << string("var F1_0   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4} T_W_H[i1,j,i]*TF1_H_0[j,i2]; \n");
        if(transport){
            stream << string("var F1_1   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F1_0[i1,j,i]*TF1_0_1[j,i2]; \n");
            stream << string("var F1_2   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F1_1[i1,j,i]*TF1_1_2[j,i2]; \n");
            stream << string("var F1_tip {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F1_2[i1,j,i]*TF1_2_3[j,i2]; \n\n");
        }else{
            stream << string("var F1_1   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F1_0[i1,j,i]*TF1_0_1[j,i2,i]; \n");
            stream << string("var F1_2   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F1_1[i1,j,i]*TF1_1_2[j,i2,i]; \n");
            stream << string("var F1_tip {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F1_2[i1,j,i]*TF1_2_3[j,i2,i]; \n\n");
        }
        stream << string("var Finger1_0   {i1 in 1..4,i in Iterations} =  if i1<4 then F1_0[i1,4,i] 	else ")+tolHand1+string("; \n");
        stream << string("var Finger1_1   {i1 in 1..4,i in Iterations} =  if i1<4 then F1_1[i1,4,i] 	else ")+tolHand2+string("; \n");
        stream << string("var Finger1_2   {i1 in 1..4,i in Iterations} =  if i1<4 then F1_2[i1,4,i] 	else ")+tolHand3+string("; \n");
        stream << string("var Finger1_tip {i1 in 1..4,i in Iterations} =  if i1<4 then F1_tip[i1,4,i] else ")+tolHand4+string("; \n\n");

    }

    stream << string("# Ring finger \n\n");

    stream << string("param TF2_H_0 {i1 in 1..4, i2 in 1..4} =  \n");
    stream << string("# 1st row \n");
    stream << string("if ( i1=1 && i2=1 ) then cos(theta0_fing3) \n");
    stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta0_fing3)*cos(alpha_fing3[1])  \n");
    stream << string("else	if ( i1=1 && i2=3 ) then sin(theta0_fing3)*sin(alpha_fing3[1])  \n");
    stream << string("else	if ( i1=1 && i2=4 ) then u3x \n");
    stream << string("# 2nd row \n");
    stream << string("else	if ( i1=2 && i2=1 ) then sin(theta0_fing3)  \n");
    stream << string("else	if ( i1=2 && i2=2 ) then cos(theta0_fing3)*cos(alpha_fing3[1])  \n");
    stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta0_fing3)*sin(alpha_fing3[1])  \n");
    stream << string("else	if ( i1=2 && i2=4 ) then u3y  \n");
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing3[1])  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing3[1])  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then u3z  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
    stream << string(";  \n");


    if (final || transport){
        // final posture selection or bounce posture selection for transporting movements
        stream << string("param TF2_0_1 {i1 in 1..4, i2 in 1..4} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(joint_fingers[3]) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(joint_fingers[3])*cos(alpha_fing3[2])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(joint_fingers[3])*sin(alpha_fing3[2])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_fing3[2]*cos(joint_fingers[3]) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(joint_fingers[3])  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(joint_fingers[3])*cos(alpha_fing3[2])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(joint_fingers[3])*sin(alpha_fing3[2])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_fing3[2]*sin(joint_fingers[3])  \n");
    }else{
        // bounce posture selection for reaching movements
        stream << string("var TF2_0_1 {i1 in 1..4, i2 in 1..4,i in Iterations} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(theta[i,9]) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[i,9])*cos(alpha_fing3[2])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(theta[i,9])*sin(alpha_fing3[2])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_fing3[2]*cos(theta[i,9]) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[i,9])  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[i,9])*cos(alpha_fing3[2])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta[i,9])*sin(alpha_fing3[2])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_fing3[2]*sin(theta[i,9])  \n");
    }
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing3[2])  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing3[2])  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then d_fing3[2]  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
    stream << string(";  \n");

    if (final || transport){
        // final posture selection or bounce posture selection for transporting movements
        stream << string("param TF2_1_2 {i1 in 1..4, i2 in 1..4} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(2*joint_fingers[3]/3) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(2*joint_fingers[3]/3)*cos(alpha_fing3[3])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(2*joint_fingers[3]/3)*sin(alpha_fing3[3])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_fing3[3]*cos(2*joint_fingers[3]/3) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(2*joint_fingers[3]/3)  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(2*joint_fingers[3]/3)*cos(alpha_fing3[3])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(2*joint_fingers[3]/3)*sin(alpha_fing3[3])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_fing1[3]*sin(2*joint_fingers[3]/3)  \n");
    }else{
        // bounce posture selection for reaching movements
        stream << string("var TF2_1_2 {i1 in 1..4, i2 in 1..4,i in Iterations} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(2*theta[i,9]/3) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(2*theta[i,9]/3)*cos(alpha_fing3[3])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(2*theta[i,9]/3)*sin(alpha_fing3[3])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_fing3[3]*cos(2*theta[i,9]/3) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(2*theta[i,9]/3)  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(2*theta[i,9]/3)*cos(alpha_fing3[3])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(2*theta[i,9]/3)*sin(alpha_fing3[3])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_fing1[3]*sin(2*theta[i,9]/3)  \n");

    }
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing3[3])  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing3[3])  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then d_fing3[3]  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
    stream << string(";  \n");

    if (final || transport){
        // final posture selection or bounce posture selection for transporting movements
        stream << string("param TF2_2_3 {i1 in 1..4, i2 in 1..4} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(joint_fingers[3]/3) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(joint_fingers[3]/3)*cos(alpha_fing3[4])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(joint_fingers[3]/3)*sin(alpha_fing3[4])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_fing3[4]*cos(joint_fingers[3]/3) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(joint_fingers[3]/3)  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(joint_fingers[3]/3)*cos(alpha_fing3[4])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(joint_fingers[3]/3)*sin(alpha_fing3[4])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_fing3[4]*sin(joint_fingers[3]/3)  \n");
    }else{
        // bounce posture selection for reaching movements
        stream << string("var TF2_2_3 {i1 in 1..4, i2 in 1..4, i in Iterations} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(theta[i,9]/3) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[i,9]/3)*cos(alpha_fing3[4])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(theta[i,9]/3)*sin(alpha_fing3[4])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_fing3[4]*cos(theta[i,9]/3) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[i,9]/3)  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[i,9]/3)*cos(alpha_fing3[4])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta[i,9]/3)*sin(alpha_fing3[4])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_fing3[4]*sin(theta[i,9]/3)  \n");
    }
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing3[4])  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing3[4])  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then d_fing3[4]  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
    stream << string(";  \n");

    // position of the fingers
    tolHand1 =  boost::str(boost::format("%.2f") % tolsHand(0,1)); boost::replace_all(tolHand1,",",".");
    tolHand2 =  boost::str(boost::format("%.2f") % tolsHand(1,1)); boost::replace_all(tolHand2,",",".");
    tolHand3 =  boost::str(boost::format("%.2f") % tolsHand(2,1)); boost::replace_all(tolHand3,",",".");
    tolHand4 =  boost::str(boost::format("%.2f") % tolsHand(3,1)); boost::replace_all(tolHand4,",",".");

    if(final){
        // final posture selection
        stream << string("var F2_0   {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4} T_W_H[i1,j]*TF2_H_0[j,i2]; \n");
        stream << string("var F2_1   {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F2_0[i1,j]*TF2_0_1[j,i2]; \n");
        stream << string("var F2_2   {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F2_1[i1,j]*TF2_1_2[j,i2]; \n");
        stream << string("var F2_tip {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F2_2[i1,j]*TF2_2_3[j,i2]; \n\n");

        stream << string("var Finger2_0   {i1 in 1..4} =  if i1<4 then F2_0[i1,4] 	else ")+tolHand1+string("; \n");
        stream << string("var Finger2_1   {i1 in 1..4} =  if i1<4 then F2_1[i1,4] 	else ")+tolHand2+string("; \n");
        stream << string("var Finger2_2   {i1 in 1..4} =  if i1<4 then F2_2[i1,4] 	else ")+tolHand3+string("; \n");
        stream << string("var Finger2_tip {i1 in 1..4} =  if i1<4 then F2_tip[i1,4] else ")+tolHand4+string("; \n\n");
    }else{
        //bounce posture selection
        stream << string("var F2_0   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4} T_W_H[i1,j,i]*TF2_H_0[j,i2]; \n");
        if (transport){
            stream << string("var F2_1   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F2_0[i1,j,i]*TF2_0_1[j,i2]; \n");
            stream << string("var F2_2   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F2_1[i1,j,i]*TF2_1_2[j,i2]; \n");
            stream << string("var F2_tip {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F2_2[i1,j,i]*TF2_2_3[j,i2]; \n\n");
        }else{
            stream << string("var F2_1   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F2_0[i1,j,i]*TF2_0_1[j,i2,i]; \n");
            stream << string("var F2_2   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F2_1[i1,j,i]*TF2_1_2[j,i2,i]; \n");
            stream << string("var F2_tip {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F2_2[i1,j,i]*TF2_2_3[j,i2,i]; \n\n");
        }
        stream << string("var Finger2_0   {i1 in 1..4,i in Iterations} =  if i1<4 then F2_0[i1,4,i] 	else ")+tolHand1+string("; \n");
        stream << string("var Finger2_1   {i1 in 1..4,i in Iterations} =  if i1<4 then F2_1[i1,4,i] 	else ")+tolHand2+string("; \n");
        stream << string("var Finger2_2   {i1 in 1..4,i in Iterations} =  if i1<4 then F2_2[i1,4,i] 	else ")+tolHand3+string("; \n");
        stream << string("var Finger2_tip {i1 in 1..4,i in Iterations} =  if i1<4 then F2_tip[i1,4,i] else ")+tolHand4+string("; \n\n");

    }

    stream << string("# Thumb finger \n\n");

    stream << string("param TF3_H_0 {i1 in 1..4, i2 in 1..4} =  \n");
    stream << string("# 1st row \n");
    stream << string("if ( i1=1 && i2=1 ) then cos(theta0_thumb) \n");
    stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta0_thumb)*cos(alpha_thumb[1])  \n");
    stream << string("else	if ( i1=1 && i2=3 ) then sin(theta0_thumb)*sin(alpha_thumb[1])  \n");
    stream << string("else	if ( i1=1 && i2=4 ) then uTx \n");
    stream << string("# 2nd row \n");
    stream << string("else	if ( i1=2 && i2=1 ) then sin(theta0_thumb)  \n");
    stream << string("else	if ( i1=2 && i2=2 ) then cos(theta0_thumb)*cos(alpha_thumb[1])  \n");
    stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta0_thumb)*sin(alpha_thumb[1])  \n");
    stream << string("else	if ( i1=2 && i2=4 ) then uTy  \n");
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_thumb[1])  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_thumb[1])  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then uTz  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
    stream << string(";  \n");

    if (final || transport){
        // final posture selection or bounce posture selection for transporting movements
        stream << string("param TF3_0_1 {i1 in 1..4, i2 in 1..4} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(joint_fingers[1]) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(joint_fingers[1])*cos(alpha_thumb[2])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(joint_fingers[1])*sin(alpha_thumb[2])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_thumb[2]*cos(joint_fingers[1]) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(joint_fingers[1])  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(joint_fingers[1])*cos(alpha_thumb[2])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(joint_fingers[1])*sin(alpha_thumb[2])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_thumb[2]*sin(joint_fingers[1])  \n");
    }else{
        // bounce posture selection for reaching movements
        stream << string("var TF3_0_1 {i1 in 1..4, i2 in 1..4,i in Iterations} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(theta[i,8]) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[i,8])*cos(alpha_thumb[2])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(theta[i,8])*sin(alpha_thumb[2])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_thumb[2]*cos(theta[i,8]) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[i,8])  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[i,8])*cos(alpha_thumb[2])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta[i,8])*sin(alpha_thumb[2])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_thumb[2]*sin(theta[i,8])  \n");
    }
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_thumb[2])  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_thumb[2])  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then d_thumb[2]  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
    stream << string(";  \n");

    if (final || transport){
        // final posture selection or bounce posture selection for transporting movements
        stream << string("param TF3_1_2 {i1 in 1..4, i2 in 1..4} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(joint_fingers[4]) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(joint_fingers[4])*cos(alpha_thumb[3])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(joint_fingers[4])*sin(alpha_thumb[3])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_thumb[3]*cos(joint_fingers[4]) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(joint_fingers[4])  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(joint_fingers[4])*cos(alpha_thumb[3])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(joint_fingers[4])*sin(alpha_thumb[3])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_thumb[3]*sin(joint_fingers[4])  \n");
    }else{
        // bounce posture selection for reaching movements
        stream << string("var TF3_1_2 {i1 in 1..4, i2 in 1..4, i in Iterations} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(theta[i,10]) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[i,10])*cos(alpha_thumb[3])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(theta[i,10])*sin(alpha_thumb[3])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_thumb[3]*cos(theta[i,10]) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[i,10])  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[i,10])*cos(alpha_thumb[3])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta[i,10])*sin(alpha_thumb[3])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_thumb[3]*sin(theta[i,10])  \n");
    }
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_thumb[3])  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_thumb[3])  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then d_thumb[3]  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
    stream << string(";  \n");

    if (final || transport){
        // final posture selection or bounce posture selection for transporting movements
        stream << string("param TF3_2_3 {i1 in 1..4, i2 in 1..4} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(11*joint_fingers[4]/10) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(11*joint_fingers[4]/10)*cos(alpha_thumb[4])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(11*joint_fingers[4]/10)*sin(alpha_thumb[4])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_thumb[4]*cos(11*joint_fingers[4]/10) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(11*joint_fingers[4]/10)  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(11*joint_fingers[4]/10)*cos(alpha_thumb[4])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(11*joint_fingers[4]/10)*sin(alpha_thumb[4])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_thumb[4]*sin(11*joint_fingers[4]/10)  \n");
    }else{
        // bounce posture selection for reaching movements
        stream << string("var TF3_2_3 {i1 in 1..4, i2 in 1..4,i in Iterations} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(11*theta[i,10]/10) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(11*theta[i,10]/10)*cos(alpha_thumb[4])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(11*theta[i,10]/10)*sin(alpha_thumb[4])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_thumb[4]*cos(11*theta[i,10]/10) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(11*theta[i,10]/10)  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(11*theta[i,10]/10)*cos(alpha_thumb[4])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(11*theta[i,10]/10)*sin(alpha_thumb[4])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_thumb[4]*sin(11*theta[i,10]/10)  \n");

    }
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_thumb[4])  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_thumb[4])  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then d_thumb[4]  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
    stream << string(";  \n");

    if (final || transport){
        // final posture selection or bounce posture selection for transporting movements
        stream << string("param TF3_3_4 {i1 in 1..4, i2 in 1..4} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(11*joint_fingers[4]/12) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(11*joint_fingers[4]/12)*cos(alpha_thumb[5])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(11*joint_fingers[4]/12)*sin(alpha_thumb[5])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_thumb[5]*cos(11*joint_fingers[4]/12) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(11*joint_fingers[4]/12)  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(11*joint_fingers[4]/12)*cos(alpha_thumb[5])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(11*joint_fingers[4]/12)*sin(alpha_thumb[5])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_thumb[5]*sin(11*joint_fingers[4]/12)  \n");
    }else{
        // bounce posture selection for reaching movements
        stream << string("var TF3_3_4 {i1 in 1..4, i2 in 1..4, i in Iterations} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(11*theta[i,10]/12) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(11*theta[i,10]/12)*cos(alpha_thumb[5])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(11*theta[i,10]/12)*sin(alpha_thumb[5])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_thumb[5]*cos(11*theta[i,10]/12) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(11*theta[i,10]/12)  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(11*theta[i,10]/12)*cos(alpha_thumb[5])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(11*theta[i,10]/12)*sin(alpha_thumb[5])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_thumb[5]*sin(11*theta[i,10]/12)  \n");
    }
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_thumb[5])  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_thumb[5])  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then d_thumb[5]  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
    stream << string(";  \n");

    // position of the fingers
    tolHand1 =  boost::str(boost::format("%.2f") % tolsHand(0,2)); boost::replace_all(tolHand1,",",".");
    tolHand2 =  boost::str(boost::format("%.2f") % tolsHand(1,2)); boost::replace_all(tolHand2,",",".");
    tolHand3 =  boost::str(boost::format("%.2f") % tolsHand(2,2)); boost::replace_all(tolHand3,",",".");
    tolHand4 =  boost::str(boost::format("%.2f") % tolsHand(3,2)); boost::replace_all(tolHand4,",",".");

    if(final){
        // final posture selection
        stream << string("var F3_0   {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4} T_W_H[i1,j]*TF3_H_0[j,i2]; \n");
        stream << string("var F3_1   {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F3_0[i1,j]*TF3_0_1[j,i2]; \n");
        stream << string("var F3_2   {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F3_1[i1,j]*TF3_1_2[j,i2]; \n");
        stream << string("var F3_3   {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F3_2[i1,j]*TF3_2_3[j,i2]; \n");
        stream << string("var F3_tip {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F3_3[i1,j]*TF3_3_4[j,i2]; \n\n");

        //PostureMod << string("var Finger3_0   {i1 in 1..4} =  if i1<4 then F3_0[i1,4] 	else ")+tolHand1+string("; \n");
        stream << string("var Finger3_1   {i1 in 1..4} =  if i1<4 then F3_1[i1,4] 	else ")+tolHand1+string("; \n");
        stream << string("var Finger3_2   {i1 in 1..4} =  if i1<4 then F3_2[i1,4] 	else ")+tolHand2+string("; \n");
        stream << string("var Finger3_3   {i1 in 1..4} =  if i1<4 then F3_3[i1,4] 	else ")+tolHand3+string("; \n");
        stream << string("var Finger3_tip {i1 in 1..4} =  if i1<4 then F3_tip[i1,4] else ")+tolHand4+string("; \n\n");

    }else{
        //bounce posture selection
        stream << string("var F3_0   {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4} T_W_H[i1,j,i]*TF3_H_0[j,i2]; \n");
        if(transport){
            stream << string("var F3_1   {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_0[i1,j,i]*TF3_0_1[j,i2]; \n");
            stream << string("var F3_2   {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_1[i1,j,i]*TF3_1_2[j,i2]; \n");
            stream << string("var F3_3   {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_2[i1,j,i]*TF3_2_3[j,i2]; \n");
            stream << string("var F3_tip {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_3[i1,j,i]*TF3_3_4[j,i2]; \n\n");
        }else{
            stream << string("var F3_1   {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_0[i1,j,i]*TF3_0_1[j,i2,i]; \n");
            stream << string("var F3_2   {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_1[i1,j,i]*TF3_1_2[j,i2,i]; \n");
            stream << string("var F3_3   {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_2[i1,j,i]*TF3_2_3[j,i2,i]; \n");
            stream << string("var F3_tip {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_3[i1,j,i]*TF3_3_4[j,i2,i]; \n\n");
        }
        stream << string("var Finger3_1   {i1 in 1..4, i in Iterations} =  if i1<4 then F3_1[i1,4,i] 	else ")+tolHand1+string("; \n");
        stream << string("var Finger3_2   {i1 in 1..4, i in Iterations} =  if i1<4 then F3_2[i1,4,i] 	else ")+tolHand2+string("; \n");
        stream << string("var Finger3_3   {i1 in 1..4, i in Iterations} =  if i1<4 then F3_3[i1,4,i] 	else ")+tolHand3+string("; \n");
        stream << string("var Finger3_tip {i1 in 1..4, i in Iterations} =  if i1<4 then F3_tip[i1,4,i] else ")+tolHand4+string("; \n\n");

    }





}

//#elif HAND==1

void Problem::writeBarrettHandParams(humanoidPtr hh, ofstream &stream)
{

    std::vector<float> rk;
    std::vector<float> jk;
    hh->getRK(rk);
    hh->getJK(jk);

    // rk and jk parameters
    stream << string("# R and J parameters \n");
    stream << string("param rk := \n");

    for (size_t i=0; i < rk.size(); ++i){
        string rkstr =  boost::str(boost::format("%.2f") % (rk.at(i)));
        boost::replace_all(rkstr,",",".");
        if (i == rk.size()-1){
            stream << to_string(i+1)+string(" ")+rkstr+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+rkstr+string("\n");
        }
    }

    stream << string("param jk := \n");

    for (size_t i=0; i < jk.size(); ++i){
        string jkstr =  boost::str(boost::format("%.2f") % (jk.at(i)));
        boost::replace_all(jkstr,",",".");
        if (i == jk.size()-1){
            stream << to_string(i+1)+string(" ")+jkstr+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+jkstr+string("\n");
        }

    }


    string Awstr =  boost::str(boost::format("%.2f") % (hh->getBarrettHand().Aw));
    boost::replace_all(Awstr,",",".");
    stream << string("param Aw :=")+Awstr+string(";\n");

    string A1str =  boost::str(boost::format("%.2f") % (hh->getBarrettHand().A1));
    boost::replace_all(A1str,",",".");
    stream << string("param A1 :=")+A1str+string(";\n");

    string A2str =  boost::str(boost::format("%.2f") % (hh->getBarrettHand().A2));
    boost::replace_all(A2str,",",".");
    stream << string("param A2 :=")+A2str+string(";\n");

    string A3str =  boost::str(boost::format("%.2f") % (hh->getBarrettHand().A3));
    boost::replace_all(A3str,",",".");
    stream << string("param A3 :=")+A3str+string(";\n");

    string D3str =  boost::str(boost::format("%.2f") % (hh->getBarrettHand().D3));
    boost::replace_all(D3str,",",".");
    stream << string("param D3 :=")+D3str+string(";\n");

    string phi2str =  boost::str(boost::format("%.2f") % (hh->getBarrettHand().phi2));
    boost::replace_all(phi2str,",",".");
    stream << string("param phi_2 :=")+phi2str+string(";\n");

    string phi3str =  boost::str(boost::format("%.2f") % (hh->getBarrettHand().phi3));
    boost::replace_all(phi3str,",",".");
    stream << string("param phi_3 :=")+phi3str+string(";\n");

}


void Problem::writeBarrettHandParamsMod(std::ofstream& stream)
{

    stream << string("param rk {i in 1..3} ; \n");
    stream << string("param jk {i in 1..3} ; \n");
    stream << string("param Aw; \n");
    stream << string("param A1; \n");
    stream << string("param A2; \n");
    stream << string("param A3; \n");
    stream << string("param D3; \n");
    stream << string("param phi_2; \n");
    stream << string("param phi_3; \n");
}



void Problem::writeBarrettHandDirKin(ofstream &stream, std::vector<int> &rk, std::vector<int> &jk, MatrixXf &tolsHand, bool final, bool transport)
{

    stream << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
    stream << string("#  Direct Kinematics model of the fingers \n\n");

    if(final){
        // final posture selection
        for (int i = 0 ; i < HAND_FINGERS; ++i){
            //for (int j = 0; j <N_PHALANGE; ++j){

            string rkk = boost::str(boost::format("%.2f") % rk.at(i)); boost::replace_all(rkk,",",".");
            string jkk = boost::str(boost::format("%.2f") % jk.at(i)); boost::replace_all(jkk,",",".");

                 stream << string("# Finger ")+to_string(i+1)+string(" \n\n");

                 stream << string("param TF")+to_string(i+1)+string("_")+to_string(1)+string("{i1 in 1..4, i2 in 1..4} :=   \n");
                 stream << string(" if ((i1=1&&i2=1)||(i1=2&&i2=2)) then 	cos(")+rkk+string("*joint_fingers[")+to_string(1)+string("]-(pi/2)*(")+jkk+string(")) \n");
                 stream << string("else 	if (i1=1&&i2=2)					then   -sin(")+rkk+string("*joint_fingers[")+to_string(1)+string("]-(pi/2)*(")+jkk+string(")) \n");
                 stream << string("else 	if (i1=2&&i2=1)					then 	sin(")+rkk+string("*joint_fingers[")+to_string(1)+string("]-(pi/2)*(")+jkk+string("))  \n");
                 stream << string("else 	if (( i1=1 && i2=3 )||( i1=2 && i2>2 )||( i1=3 && i2<3 )||( i1=4 && i2<4 )) then 	0 \n");
                 stream << string("else	if (( i1=3 && i2=3 )||( i1=4 && i2=4 ) ) then 								1 \n");
                 stream << string("else	if ( i1=1 && i2=4 ) then 				")+rkk+string("*Aw 	 				 \n");
                 stream << string("else	if ( i1=3 && i2=4 ) then 										0 \n");
                 stream << string("; \n");

                 stream << string("param TF")+to_string(i+1)+string("_")+to_string(2)+string("{i1 in 1..4, i2 in 1..4} :=   \n");
                 stream << string("if ((i1=1&&i2=1)||(i1=3&&i2=2)) then 	cos(joint_fingers[")+to_string(i+2)+string("]+phi_2)  \n");
                 stream << string("else 	if (i1=1&&i2=2)					then   -sin(joint_fingers[")+to_string(i+2)+string("]+phi_2)  \n");
                 stream << string("else 	if (i1=3&&i2=1)					then    sin(joint_fingers[")+to_string(i+2)+string("]+phi_2)  \n");
                 stream << string("else 	if (( i1=1 && i2=3 )||( i1=2 && i2<3 )||( i1=3 && i2>2 )||( i1=4 && i2<4 )||( i1=2 && i2=4 )) then 	0 \n");
                 stream << string("else	if ( i1=4 && i2=4 )  then 								1 \n");
                 stream << string("else	if ( i1=2 && i2=3 )  then 								-1 \n");
                 stream << string("else	if ( i1=1 && i2=4 ) then 								A1 \n");
                 stream << string("; \n");

                 stream << string("param TF")+to_string(i+1)+string("_")+to_string(3)+string("{i1 in 1..4, i2 in 1..4} :=   \n");
                 stream << string("if ((i1=1&&i2=1)||(i1=2&&i2=2)) then 	cos((joint_fingers[")+to_string(i+2)+string("])/3+phi_3) \n");
                 stream << string("else 	if (i1=1&&i2=2)					then   -sin((joint_fingers[")+to_string(i+2)+string("])/3+phi_3) \n");
                 stream << string("else 	if (i1=2&&i2=1)					then    sin((joint_fingers[")+to_string(i+2)+string("])/3+phi_3) \n");
                 stream << string("else 	if (( i1=1 && i2=3 )||( i1=3 && i2<3 )||( i1=2 && i2>2 )||( i1=4 && i2<4 )||( i1=3 && i2=4 )) then 	0 \n");
                 stream << string("else	if (( i1=4 && i2=4 )||( i1=3 && i2=3 ))  then 														1 \n");
                 stream << string("else	if ( i1=1 && i2=4 ) then 								A2 \n");
                 stream << string("; \n");

                 stream << string("param TF")+to_string(i+1)+string("_")+to_string(4)+string("{i1 in 1..4, i2 in 1..4} :=   \n");
                 stream << string("if ((i1=1&&i2=1)||(i1=2&&i2=3)||(i1=4&&i2=4)) then 	  1\n");
                 stream << string("else 	if (i1=3&&i2=2)								  then   -1 \n");
                 stream << string("else 	if (i1=1&&i2=4)					              then    A3 \n");
                 stream << string("else 	if (i1=2&&i2=4)					              then    D3 \n");
                 stream << string("else 	if (( i1=1 && i2=2 )||( i1=1 && i2=3 )||( i1=2 && i2<3 )||( i1=3 && i2>2 )||( i1=3 && i2=1 )||( i1=4 && i2<4 )) then 	0 \n");
                 stream << string(";\n\n");


                 // position of the fingers
                 stream << string("var F")+to_string(i+1)+string("_0   {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4} T_W_6[i1,j]*TF")+to_string(i+1)+string("_1[j,i2]; \n");
                 stream << string("var F")+to_string(i+1)+string("_1   {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F")+to_string(i+1)+string("_0[i1,j]*TF")+to_string(i+1)+string("_2[j,i2]; \n");
                 stream << string("var F")+to_string(i+1)+string("_2   {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F")+to_string(i+1)+string("_1[i1,j]*TF")+to_string(i+1)+string("_3[j,i2]; \n");
                 stream << string("var F")+to_string(i+1)+string("_tip {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F")+to_string(i+1)+string("_2[i1,j]*TF")+to_string(i+1)+string("_4[j,i2]; \n\n");

                 string tolHand1 =  boost::str(boost::format("%.2f") % tolsHand(0,i)); boost::replace_all(tolHand1,",",".");
                 string tolHand2 =  boost::str(boost::format("%.2f") % tolsHand(1,i)); boost::replace_all(tolHand2,",",".");
                 string tolHand3 =  boost::str(boost::format("%.2f") % tolsHand(2,i)); boost::replace_all(tolHand3,",",".");
                 string tolHand4 =  boost::str(boost::format("%.2f") % tolsHand(3,i)); boost::replace_all(tolHand4,",",".");

                 stream << string("var Finger")+to_string(i+1)+string("_0   {i1 in 1..4} =  if i1<4 then F")+to_string(i+1)+string("_0[i1,4] 	else ")+tolHand1+string("; \n");
                 stream << string("var Finger")+to_string(i+1)+string("_1   {i1 in 1..4} =  if i1<4 then F")+to_string(i+1)+string("_1[i1,4] 	else ")+tolHand2+string("; \n");
                 stream << string("var Finger")+to_string(i+1)+string("_2   {i1 in 1..4} =  if i1<4 then F")+to_string(i+1)+string("_2[i1,4] 	else ")+tolHand3+string("; \n");
                 stream << string("var Finger")+to_string(i+1)+string("_tip {i1 in 1..4} =  if i1<4 then F")+to_string(i+1)+string("_tip[i1,4] else ")+tolHand4+string("; \n\n");

           // }

        }
    }else{
        // bounce poncure selection


        if (transport){
            // transport or engage movement
            for (int i = 0 ; i < HAND_FINGERS; ++i){
                //for (int j = 0; j <N_PHALANGE; ++j){
                int k;
                if (i == 2){
                    k = 8;
                }else{
                    k = 9;
                }

                string rkk = boost::str(boost::format("%.2f") % rk.at(i)); boost::replace_all(rkk,",",".");
                string jkk = boost::str(boost::format("%.2f") % jk.at(i)); boost::replace_all(jkk,",",".");

                 stream << string("# Finger ")+to_string(i+1)+string(" \n\n");

                 stream << string("param TF")+to_string(i+1)+string("_")+to_string(1)+string("{i1 in 1..4, i2 in 1..4,i in Iterations} =   \n");
                 stream << string(" if ((i1=1&&i2=1)||(i1=2&&i2=2)) then 	cos(")+rkk+string("*")+to_string(0)+string("-(pi/2)*(")+jkk+string(")) \n");
                 stream << string("else 	if (i1=1&&i2=2)					then   -sin(")+rkk+string("*")+to_string(0)+string("-(pi/2)*(")+jkk+string(")) \n");
                 stream << string("else 	if (i1=2&&i2=1)					then 	sin(")+rkk+string("*")+to_string(0)+string("-(pi/2)*(")+jkk+string("))  \n");
                 stream << string("else 	if (( i1=1 && i2=3 )||( i1=2 && i2>2 )||( i1=3 && i2<3 )||( i1=4 && i2<4 )) then 	0 \n");
                 stream << string("else	if (( i1=3 && i2=3 )||( i1=4 && i2=4 ) ) then 								1 \n");
                 stream << string("else	if ( i1=1 && i2=4 ) then 				")+rkk+string("*Aw 	 				 \n");
                 stream << string("else	if ( i1=3 && i2=4 ) then 										0 \n");
                 stream << string("; \n");

                 stream << string("param TF")+to_string(i+1)+string("_")+to_string(2)+string("{i1 in 1..4, i2 in 1..4,i in Iterations} =   \n");
                 stream << string("if ((i1=1&&i2=1)||(i1=3&&i2=2)) then 	cos(joint_fingers[")+to_string(i+2)+string("]+phi_2)  \n");
                 stream << string("else 	if (i1=1&&i2=2)					then   -sin(joint_fingers[")+to_string(i+2)+string("]+phi_2)  \n");
                 stream << string("else 	if (i1=3&&i2=1)					then    sin(joint_fingers[")+to_string(i+2)+string("]+phi_2)  \n");
                 stream << string("else 	if (( i1=1 && i2=3 )||( i1=2 && i2<3 )||( i1=3 && i2>2 )||( i1=4 && i2<4 )||( i1=2 && i2=4 )) then 	0 \n");
                 stream << string("else	if ( i1=4 && i2=4 )  then 								1 \n");
                 stream << string("else	if ( i1=2 && i2=3 )  then 								-1 \n");
                 stream << string("else	if ( i1=1 && i2=4 ) then 								A1 \n");
                 stream << string("; \n");

                 stream << string("param TF")+to_string(i+1)+string("_")+to_string(3)+string("{i1 in 1..4, i2 in 1..4,i in Iterations} =   \n");
                 stream << string("if ((i1=1&&i2=1)||(i1=2&&i2=2)) then 	cos((joint_fingers[")+to_string(i+2)+string("])/3+phi_3) \n");
                 stream << string("else 	if (i1=1&&i2=2)					then   -sin((joint_fingers[")+to_string(i+2)+string("])/3+phi_3) \n");
                 stream << string("else 	if (i1=2&&i2=1)					then    sin((joint_fingers[")+to_string(i+2)+string("])/3+phi_3) \n");
                 stream << string("else 	if (( i1=1 && i2=3 )||( i1=3 && i2<3 )||( i1=2 && i2>2 )||( i1=4 && i2<4 )||( i1=3 && i2=4 )) then 	0 \n");
                 stream << string("else	if (( i1=4 && i2=4 )||( i1=3 && i2=3 ))  then 														1 \n");
                 stream << string("else	if ( i1=1 && i2=4 ) then 								A2 \n");
                 stream << string("; \n");

                 stream << string("param TF")+to_string(i+1)+string("_")+to_string(4)+string("{i1 in 1..4, i2 in 1..4,i in Iterations} :=   \n");
                 stream << string("if ((i1=1&&i2=1)||(i1=2&&i2=3)||(i1=4&&i2=4)) then 	  1\n");
                 stream << string("else 	if (i1=3&&i2=2)								  then   -1 \n");
                 stream << string("else 	if (i1=1&&i2=4)					              then    A3 \n");
                 stream << string("else 	if (i1=2&&i2=4)					              then    D3 \n");
                 stream << string("else 	if (( i1=1 && i2=2 )||( i1=1 && i2=3 )||( i1=2 && i2<3 )||( i1=3 && i2>2 )||( i1=3 && i2=1 )||( i1=4 && i2<4 )) then 	0 \n");
                 stream << string(";\n\n");


                 // position of the fingers
                 stream << string("var F")+to_string(i+1)+string("_0   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4} T_W_6[i1,j,i]*TF")+to_string(i+1)+string("_1[j,i2,i]; \n");
                 stream << string("var F")+to_string(i+1)+string("_1   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F")+to_string(i+1)+string("_0[i1,j,i]*TF")+to_string(i+1)+string("_2[j,i2,i]; \n");
                 stream << string("var F")+to_string(i+1)+string("_2   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F")+to_string(i+1)+string("_1[i1,j,i]*TF")+to_string(i+1)+string("_3[j,i2,i]; \n");
                 stream << string("var F")+to_string(i+1)+string("_tip {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F")+to_string(i+1)+string("_2[i1,j,i]*TF")+to_string(i+1)+string("_4[j,i2,i]; \n\n");


                 string tolHand1 =  boost::str(boost::format("%.2f") % tolsHand(0,i)); boost::replace_all(tolHand1,",",".");
                 string tolHand2 =  boost::str(boost::format("%.2f") % tolsHand(1,i)); boost::replace_all(tolHand2,",",".");
                 string tolHand3 =  boost::str(boost::format("%.2f") % tolsHand(2,i)); boost::replace_all(tolHand3,",",".");
                 string tolHand4 =  boost::str(boost::format("%.2f") % tolsHand(3,i)); boost::replace_all(tolHand4,",",".");

                 stream << string("var Finger")+to_string(i+1)+string("_0   {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_0[i1,4,i] 	else ")+tolHand1+string("; \n");
                 stream << string("var Finger")+to_string(i+1)+string("_1   {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_1[i1,4,i] 	else ")+tolHand2+string("; \n");
                 stream << string("var Finger")+to_string(i+1)+string("_2   {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_2[i1,4,i] 	else ")+tolHand3+string("; \n");
                 stream << string("var Finger")+to_string(i+1)+string("_tip {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_tip[i1,4,i] else ")+tolHand4+string("; \n\n");


               // }


            }

        }else{
            // reaching movements
            for (int i = 0 ; i < HAND_FINGERS; ++i){
                //for (int j = 0; j <N_PHALANGE; ++j){
                int k;
                if (i == 2){
                    k = 8;
                }else{
                    k = 9;
                }
                string rkk = boost::str(boost::format("%.2f") % rk.at(i)); boost::replace_all(rkk,",",".");
                string jkk = boost::str(boost::format("%.2f") % jk.at(i)); boost::replace_all(jkk,",",".");

                 stream << string("# Finger ")+to_string(i+1)+string(" \n\n");

                 stream << string("param TF")+to_string(i+1)+string("_")+to_string(1)+string("{i1 in 1..4, i2 in 1..4,i in Iterations} =   \n");
                 stream << string(" if ((i1=1&&i2=1)||(i1=2&&i2=2)) then 	cos(")+rkk+string("*")+to_string(0)+string("-(pi/2)*(")+jkk+string(")) \n");
                 stream << string("else 	if (i1=1&&i2=2)					then   -sin(")+rkk+string("*")+to_string(0)+string("-(pi/2)*(")+jkk+string(")) \n");
                 stream << string("else 	if (i1=2&&i2=1)					then 	sin(")+rkk+string("*")+to_string(0)+string("-(pi/2)*(")+jkk+string("))  \n");
                 stream << string("else 	if (( i1=1 && i2=3 )||( i1=2 && i2>2 )||( i1=3 && i2<3 )||( i1=4 && i2<4 )) then 	0 \n");
                 stream << string("else	if (( i1=3 && i2=3 )||( i1=4 && i2=4 ) ) then 								1 \n");
                 stream << string("else	if ( i1=1 && i2=4 ) then 				")+rkk+string("*Aw 	 				 \n");
                 stream << string("else	if ( i1=3 && i2=4 ) then 										0 \n");
                 stream << string("; \n");

                 stream << string("var TF")+to_string(i+1)+string("_")+to_string(2)+string("{i1 in 1..4, i2 in 1..4,i in Iterations} =   \n");
                 stream << string("if ((i1=1&&i2=1)||(i1=3&&i2=2)) then 	cos(theta[i,")+to_string(k)+string("]+phi_2)  \n");
                 stream << string("else 	if (i1=1&&i2=2)					then   -sin(theta[i,")+to_string(k)+string("]+phi_2)  \n");
                 stream << string("else 	if (i1=3&&i2=1)					then    sin(theta[i,")+to_string(k)+string("]+phi_2)  \n");
                 stream << string("else 	if (( i1=1 && i2=3 )||( i1=2 && i2<3 )||( i1=3 && i2>2 )||( i1=4 && i2<4 )||( i1=2 && i2=4 )) then 	0 \n");
                 stream << string("else	if ( i1=4 && i2=4 )  then 								1 \n");
                 stream << string("else	if ( i1=2 && i2=3 )  then 								-1 \n");
                 stream << string("else	if ( i1=1 && i2=4 ) then 								A1 \n");
                 stream << string("; \n");

                 stream << string("var TF")+to_string(i+1)+string("_")+to_string(3)+string("{i1 in 1..4, i2 in 1..4,i in Iterations} =   \n");
                 stream << string("if ((i1=1&&i2=1)||(i1=2&&i2=2)) then 	cos((theta[i,")+to_string(k)+string("])/3+phi_3) \n");
                 stream << string("else 	if (i1=1&&i2=2)					then   -sin((theta[i,")+to_string(k)+string("])/3+phi_3) \n");
                 stream << string("else 	if (i1=2&&i2=1)					then    sin((theta[i,")+to_string(k)+string("])/3+phi_3) \n");
                 stream << string("else 	if (( i1=1 && i2=3 )||( i1=3 && i2<3 )||( i1=2 && i2>2 )||( i1=4 && i2<4 )||( i1=3 && i2=4 )) then 	0 \n");
                 stream << string("else	if (( i1=4 && i2=4 )||( i1=3 && i2=3 ))  then 														1 \n");
                 stream << string("else	if ( i1=1 && i2=4 ) then 								A2 \n");
                 stream << string("; \n");

                 stream << string("param TF")+to_string(i+1)+string("_")+to_string(4)+string("{i1 in 1..4, i2 in 1..4,i in Iterations} :=   \n");
                 stream << string("if ((i1=1&&i2=1)||(i1=2&&i2=3)||(i1=4&&i2=4)) then 	  1\n");
                 stream << string("else 	if (i1=3&&i2=2)								  then   -1 \n");
                 stream << string("else 	if (i1=1&&i2=4)					              then    A3 \n");
                 stream << string("else 	if (i1=2&&i2=4)					              then    D3 \n");
                 stream << string("else 	if (( i1=1 && i2=2 )||( i1=1 && i2=3 )||( i1=2 && i2<3 )||( i1=3 && i2>2 )||( i1=3 && i2=1 )||( i1=4 && i2<4 )) then 	0 \n");
                 stream << string(";\n\n");


                 // position of the fingers
                 stream << string("var F")+to_string(i+1)+string("_0   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4} T_W_6[i1,j,i]*TF")+to_string(i+1)+string("_1[j,i2,i]; \n");
                 stream << string("var F")+to_string(i+1)+string("_1   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F")+to_string(i+1)+string("_0[i1,j,i]*TF")+to_string(i+1)+string("_2[j,i2,i]; \n");
                 stream << string("var F")+to_string(i+1)+string("_2   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F")+to_string(i+1)+string("_1[i1,j,i]*TF")+to_string(i+1)+string("_3[j,i2,i]; \n");
                 stream << string("var F")+to_string(i+1)+string("_tip {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F")+to_string(i+1)+string("_2[i1,j,i]*TF")+to_string(i+1)+string("_4[j,i2,i]; \n\n");

                 string tolHand1 =  boost::str(boost::format("%.2f") % tolsHand(0,i)); boost::replace_all(tolHand1,",",".");
                 string tolHand2 =  boost::str(boost::format("%.2f") % tolsHand(1,i)); boost::replace_all(tolHand2,",",".");
                 string tolHand3 =  boost::str(boost::format("%.2f") % tolsHand(2,i)); boost::replace_all(tolHand3,",",".");
                 string tolHand4 =  boost::str(boost::format("%.2f") % tolsHand(3,i)); boost::replace_all(tolHand4,",",".");

                 stream << string("var Finger")+to_string(i+1)+string("_0   {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_0[i1,4,i] 	else ")+tolHand1+string("; \n");
                 stream << string("var Finger")+to_string(i+1)+string("_1   {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_1[i1,4,i] 	else ")+tolHand2+string("; \n");
                 stream << string("var Finger")+to_string(i+1)+string("_2   {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_2[i1,4,i] 	else ")+tolHand3+string("; \n");
                 stream << string("var Finger")+to_string(i+1)+string("_tip {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_tip[i1,4,i] else ")+tolHand4+string("; \n\n");


               // }


            }

        }


    }

}

//#endif


void Problem::writeArmLimits(ofstream &stream, std::vector<float> &minArmLimits, std::vector<float> &maxArmLimits)
{

    stream << string("# JOINT LIMITS \n");
    stream << string("# Lower Bound \n");
    stream << string("param llim := \n");

    for (std::size_t i=0; i < minArmLimits.size(); ++i){
        string minLim=  boost::str(boost::format("%.2f") % (minArmLimits.at(i)));
        boost::replace_all(minLim,",",".");
        if (i == minArmLimits.size()-1){
            stream << to_string(i+1)+string(" ")+minLim+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+minLim+string("\n");
        }
    }

    stream << string("# Upper Bound \n");
    stream << string("param ulim := \n");

    for (std::size_t i=0; i < maxArmLimits.size(); ++i){
        string maxLim=  boost::str(boost::format("%.2f") % (maxArmLimits.at(i)));
        boost::replace_all(maxLim,",",".");

        if (i == maxArmLimits.size()-1){
            stream << to_string(i+1)+string(" ")+maxLim+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+maxLim+string("\n");
        }
    }

}


void Problem::writeArmInitPose(ofstream &stream, std::vector<float> &initArmPosture)
{

    stream << string("# INITIAL POSE \n");
    stream << string("param thet_init := \n");

    for (std::size_t i=0; i < initArmPosture.size(); ++i){
        string initArmstr =  boost::str(boost::format("%.2f") % (initArmPosture.at(i)));
        boost::replace_all(initArmstr,",",".");
        if (i == initArmPosture.size()-1){
            stream << to_string(i+1)+string(" ")+initArmstr+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+initArmstr+string("\n");
        }
    }

}


void Problem::writeFingerFinalPose(ofstream &stream, std::vector<float> &finalHand)
{

    stream << string("# FINAL FINGER JOINTS \n");
    stream << string("param joint_fingers := \n");

    for (std::size_t i=0; i < finalHand.size(); ++i){
        string finalHandstr =  boost::str(boost::format("%.2f") % (finalHand.at(i)));
        boost::replace_all(finalHandstr,",",".");
        if (i == finalHand.size()-1){
            stream << to_string(i+1)+string(" ")+finalHandstr+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+finalHandstr+string("\n");
        }
    }

}


void Problem::writeLambda(ofstream &stream, std::vector<float> &lambda)
{

    stream << string("# JOINT EXPENSE FACTORS \n");
    stream << string("param lambda := \n");

    for (std::size_t i=0; i < lambda.size(); ++i){
        string lambdastr =  boost::str(boost::format("%.2f") % (lambda.at(i)));
        boost::replace_all(lambdastr,",",".");
        if (i == lambda.size()-1){
            stream << to_string(i+1)+string(" ")+lambdastr+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+lambdastr+string("\n");
        }
    }
}

void Problem::writeInfoObjects(ofstream &stream, std::vector<objectPtr> &objs)
{

    /*
    // table
    stream << string("# TABLE \n");
    stream << string("param Table := \n");

    objectPtr table;
    string strTable("Table");
    for (int i = 0; objs.size(); ++i){
        table = objs.at(i);
        if (boost::iequals(table->getName(),strTable)){
            break;
        }
    }

    string tablexpos =  boost::str(boost::format("%.2f") % (table->getPos().Xpos)); boost::replace_all(tablexpos,",",".");
    string tableypos =  boost::str(boost::format("%.2f") % (table->getPos().Ypos)); boost::replace_all(tableypos,",",".");
    string tablezpos =  boost::str(boost::format("%.2f") % (table->getPos().Zpos); boost::replace_all(tablezpos,",",".");
    string tablexsize = boost::str(boost::format("%.2f") % (table->getSize().Xsize/2); boost::replace_all(tablexsize,",",".");
    string tableysize = boost::str(boost::format("%.2f") % (table->getSize().Ysize/2); boost::replace_all(tableysize,",",".");
    string tablezsize = boost::str(boost::format("%.2f") % (table->getSize().Zsize/2); boost::replace_all(tablezsize,",",".");
    string tableroll = boost::str(boost::format("%.2f") % (table->getOr().roll); boost::replace_all(tableroll,",",".");
    string tablepitch = boost::str(boost::format("%.2f") % (table->getOr().pitch); boost::replace_all(tablepitch,",",".");
    string tableyaw = boost::str(boost::format("%.2f") % (table->getOr().yaw); boost::replace_all(tableyaw,",",".");

    stream << to_string(1)+string(" ")+tablexpos+string("\n");
    stream << to_string(2)+string(" ")+tableypos+string(";\n");
    stream << to_string(3)+string(" ")+tablezpos+string(";\n");
    stream << to_string(4)+string(" ")+tablexsize+string(";\n");
    stream << to_string(5)+string(" ")+tableysize+string(";\n");
    stream << to_string(6)+string(" ")+tablezsize+string(";\n");
    stream << to_string(7)+string(" ")+tableroll+string(";\n");
    stream << to_string(8)+string(" ")+tablepitch+string(";\n");
    stream << to_string(9)+string(" ")+tableyaw+string(";\n");
    */

    // obstacles
    stream << string("# OBSTACLES POSITION+RADIUS+ORIENTATION \n");
    stream << string("param Obstacles : 1 2 3 4 5 6 7 8 9 := \n");

    std::vector<objectPtr> obstacles;
    for (std::size_t i = 0; i < objs.size(); ++i){

        objectPtr obs = objs.at(i);
        //if ((!boost::iequals(obs->getName(),strTable)) && (!obs->isTargetRightEnabled()) && (!obs->isTargetLeftEnabled()))
        if ((!obs->isTargetRightEnabled()) && (!obs->isTargetLeftEnabled()))
        {

            obstacles.push_back(obs);
            string obsx =  boost::str(boost::format("%.2f") % (obs->getPos().Xpos)); boost::replace_all(obsx,",",".");
            string obsy =  boost::str(boost::format("%.2f") % (obs->getPos().Ypos)); boost::replace_all(obsy,",",".");
            string obsz =  boost::str(boost::format("%.2f") % (obs->getPos().Zpos)); boost::replace_all(obsz,",",".");
            string obsxsize =  boost::str(boost::format("%.2f") % (obs->getSize().Xsize/2)); boost::replace_all(obsxsize,",",".");
            string obsysize =  boost::str(boost::format("%.2f") % (obs->getSize().Ysize/2)); boost::replace_all(obsysize,",",".");
            string obszsize =  boost::str(boost::format("%.2f") % (obs->getSize().Zsize/2)); boost::replace_all(obszsize,",",".");
            string obsroll =  boost::str(boost::format("%.2f") % (obs->getOr().roll)); boost::replace_all(obsroll,",",".");
            string obspitch =  boost::str(boost::format("%.2f") % (obs->getOr().pitch)); boost::replace_all(obspitch,",",".");
            string obsyaw =  boost::str(boost::format("%.2f") % (obs->getOr().yaw)); boost::replace_all(obsyaw,",",".");

            stream << to_string(obstacles.size())+string(" ")+
                               obsx+string(" ")+
                               obsy+string(" ")+
                               obsz+string(" ")+
                               obsxsize+string(" ")+
                               obsysize+string(" ")+
                               obszsize+string(" ")+
                               obsroll+string(" ")+
                               obspitch+string(" ")+
                               obsyaw+string(" ")+
                               string(" #")+obs->getName()+
                               string("\n");

        }

        if (i == objs.size()-1){

            stream << string("  ;\n");
        }

    }

    if (obstacles.empty()){

         stream << string(" param n_Obstacles := ")+to_string(0)+string(";\n");

    }else{


        stream << string(" param n_Obstacles := ")+to_string(obstacles.size())+string(";\n");
    }

}


void Problem::writeInfoTarget(ofstream &stream, targetPtr tar)
{

    stream << string("# TARGET POSITION \n");
    stream << string("param Tar_pos := \n");

    string tarx =  boost::str(boost::format("%.2f") % (tar->getPos().Xpos));
    boost::replace_all(tarx,",",".");
    stream << to_string(1)+string(" ")+tarx+string("\n");

    string tary =  boost::str(boost::format("%.2f") % (tar->getPos().Ypos));
    boost::replace_all(tary,",",".");
    stream << to_string(2)+string(" ")+tary+string("\n");

    string tarz =  boost::str(boost::format("%.2f") % (tar->getPos().Zpos));
    boost::replace_all(tarz,",",".");
    stream << to_string(3)+string(" ")+tarz+string(";\n");

    stream << string("# TARGET ORIENTATION \n");
    std::vector<float> xt;
    std::vector<float> yt;
    std::vector<float> zt;
    tar->getXt(xt);
    tar->getYt(yt);
    tar->getZt(zt);

    stream << string("param x_t := \n");
    string tarxt0 =  boost::str(boost::format("%.2f") % (xt[0]));
    boost::replace_all(tarxt0,",",".");
    stream << to_string(1)+string(" ")+tarxt0+string("\n");

    string tarxt1 =  boost::str(boost::format("%.2f") % (xt[1]));
    boost::replace_all(tarxt1,",",".");
    stream << to_string(2)+string(" ")+tarxt1+string("\n");

    string tarxt2 =  boost::str(boost::format("%.2f") % (xt[2]));
    boost::replace_all(tarxt2,",",".");
    stream << to_string(3)+string(" ")+tarxt2+string(";\n");

    stream << string("param y_t := \n");
    string taryt0 =  boost::str(boost::format("%.2f") % (yt[0]));
    boost::replace_all(taryt0,",",".");
    stream << to_string(1)+string(" ")+taryt0+string("\n");

    string taryt1 =  boost::str(boost::format("%.2f") % (yt[1]));
    boost::replace_all(taryt1,",",".");
    stream << to_string(2)+string(" ")+taryt1+string("\n");

    string taryt2 =  boost::str(boost::format("%.2f") % (yt[2]));
    boost::replace_all(taryt2,",",".");
    stream << to_string(3)+string(" ")+taryt2+string(";\n");

    stream << string("param z_t := \n");
    string tarzt0 =  boost::str(boost::format("%.2f") % (zt[0]));
    boost::replace_all(tarzt0,",",".");
    stream << to_string(1)+string(" ")+tarzt0+string("\n");

    string tarzt1 =  boost::str(boost::format("%.2f") % (zt[1]));
    boost::replace_all(tarzt1,",",".");
    stream << to_string(2)+string(" ")+tarzt1+string("\n");

    string tarzt2 =  boost::str(boost::format("%.2f") % (zt[2]));
    boost::replace_all(tarzt2,",",".");
    stream << to_string(3)+string(" ")+tarzt2+string(";\n");

}


void Problem::writeInfoObjectTarget(ofstream &stream, objectPtr obj)
{

    stream << string("# OBJECT OF THE TARGET POSITION+RADIUS+ORIENTATION \n");
    stream << string("param ObjTar : 1 2 3 4 5 6 7 8 9 := \n");

    string objx =  boost::str(boost::format("%.2f") % (obj->getPos().Xpos));
    boost::replace_all(objx,",",".");
    string objy =  boost::str(boost::format("%.2f") % (obj->getPos().Ypos));
    boost::replace_all(objy,",",".");
    string objz =  boost::str(boost::format("%.2f") % (obj->getPos().Zpos));
    boost::replace_all(objz,",",".");
    string objxsize =  boost::str(boost::format("%.2f") % (obj->getSize().Xsize/2));
    boost::replace_all(objxsize,",",".");
    string objysize =  boost::str(boost::format("%.2f") % (obj->getSize().Ysize/2));
    boost::replace_all(objysize,",",".");
    string objzsize =  boost::str(boost::format("%.2f") % (obj->getSize().Zsize/2));
    boost::replace_all(objzsize,",",".");
    string objroll =  boost::str(boost::format("%.2f") % (obj->getOr().roll));
    boost::replace_all(objroll,",",".");
    string objpitch =  boost::str(boost::format("%.2f") % (obj->getOr().pitch));
    boost::replace_all(objpitch,",",".");
    string objyaw =  boost::str(boost::format("%.2f") % (obj->getOr().yaw));
    boost::replace_all(objyaw,",",".");

    stream << to_string(1)+string(" ")+
                       objx+string(" ")+
                       objy+string(" ")+
                       objz+string(" ")+
                       objxsize+string(" ")+
                       objysize+string(" ")+
                       objzsize+string(" ")+
                       objroll+string(" ")+
                       objpitch+string(" ")+
                       objyaw+string(" ")+
                       string(" #")+obj->getName()+
                       string("\n");
    stream << string("  ;\n");

    stream << string(" param n_ObjTar := ")+to_string(1)+string(";\n");

}



void Problem::writePI(std::ofstream& stream)
{
    stream << string("param pi := 4*atan(1); \n");
}


void Problem::writeBodyDimMod(ofstream &stream)
{
    stream << string("# Body info \n");
    stream << string("param body {i in 1..2}; \n");
}


void Problem::writeArmDHParamsMod(ofstream &stream)
{
    stream << string("# D-H parameters of the arm \n");
    stream << string("param alpha {i in 1..")+to_string(JOINTS_ARM)+string("} ; \n");
    stream << string("param a {i in 1..")+to_string(JOINTS_ARM)+string("} ; \n");
    stream << string("param d {i in 1..")+to_string(JOINTS_ARM)+string("} ; \n");

}


void Problem::write_dHOMod(ofstream &stream)
{
    stream << string("# Distance hand - target  \n");
    stream << string("param dFH; \n");
}



void Problem::writeInfoObjectsMod(ofstream &stream)
{

    //stream << string("# Table \n");
    //stream << string("param Table {i in 1..2} ; # x-->Table[1], z-->Table[2]\n");

    stream << string("# Target Position \n");
    stream << string("param Tar_pos {i in 1..3}; \n");
    stream << string("# Target orientation \n");
    stream << string("param x_t {i in 1..3}; \n");
    stream << string("param y_t {i in 1..3}; \n");
    stream << string("param z_t {i in 1..3}; \n");

    stream << string("# Obstacles \n");
    stream << string("param n_Obstacles; \n");
    stream << string("param Obstacles {i in 1..n_Obstacles, j in 1..9}; \n");

    stream << string("# Object of the target \n");
    stream << string("param n_ObjTar; \n");
    stream << string("param ObjTar {i in 1..n_ObjTar, j in 1..9}; \n");
}


void Problem::writeRotMatObsts(ofstream &stream)
{

    stream << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
    stream << string("# Rotation matrix of the obstacles \n");
    stream << string("param c_roll {i in 1..n_Obstacles} := cos(Obstacles[i,7]); \n");
    stream << string("param s_roll {i in 1..n_Obstacles} := sin(Obstacles[i,7]); \n");
    stream << string("param c_pitch {i in 1..n_Obstacles} := cos(Obstacles[i,8]); \n");
    stream << string("param s_pitch {i in 1..n_Obstacles} := sin(Obstacles[i,8]); \n");
    stream << string("param c_yaw {i in 1..n_Obstacles} := cos(Obstacles[i,9]); \n");
    stream << string("param s_yaw {i in 1..n_Obstacles} := sin(Obstacles[i,9]); \n");

    stream << string("param Rot {i1 in 1..4, i2 in 1..4,i in 1..n_Obstacles} :=  \n");
    stream << string("# 1st row \n");
    stream << string("if 		   ( i1=1 && i2=1 ) then 	c_roll[i]*c_pitch[i] \n");
    stream << string("else	if ( i1=1 && i2=2 ) then   -s_roll[i]*c_yaw[i]+c_roll[i]*s_pitch[i]*s_yaw[i] \n");
    stream << string("else	if ( i1=1 && i2=3 ) then 	s_roll[i]*s_yaw[i]+c_roll[i]*s_pitch[i]*c_yaw[i] \n");
    stream << string("# 2nd row \n");
    stream << string("else	if ( i1=2 && i2=1 ) then 	s_roll[i]*c_pitch[i] \n");
    stream << string("else	if ( i1=2 && i2=2 ) then 	c_roll[i]*c_yaw[i]+s_roll[i]*s_pitch[i]*s_yaw[i] \n");
    stream << string("else	if ( i1=2 && i2=3 ) then   -c_roll[i]*s_yaw[i]+s_roll[i]*s_pitch[i]*c_yaw[i] \n");
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then   -s_pitch[i] \n");
    stream << string("else	if ( i1=3 && i2=2 ) then	c_pitch[i]*s_yaw[i] \n");
    stream << string("else	if ( i1=3 && i2=3 ) then	c_pitch[i]*c_yaw[i] \n");
    stream << string("   ; \n");

}



void Problem::writeArmDirKin(ofstream &stream, Matrix4f &matWorldToArm, Matrix4f &matHand, std::vector<float>& tolsArm, bool final)
{

    stream << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
    stream << string("#  Direct Kinematics model of the arm \n\n");

    stream << string("param c_alpha {i in 1..7} := cos(alpha[i]); \n");
    stream << string("param s_alpha {i in 1..7} := sin(alpha[i]); \n");
    stream << string("\n");

    string mat00 =  boost::str(boost::format("%.2f") % (matWorldToArm(0,0))); boost::replace_all(mat00,",",".");
    string mat01 =  boost::str(boost::format("%.2f") % (matWorldToArm(0,1))); boost::replace_all(mat01,",",".");
    string mat02 =  boost::str(boost::format("%.2f") % (matWorldToArm(0,2))); boost::replace_all(mat02,",",".");
    string mat03 =  boost::str(boost::format("%.2f") % (matWorldToArm(0,3))); boost::replace_all(mat03,",",".");
    string mat10 =  boost::str(boost::format("%.2f") % (matWorldToArm(1,0))); boost::replace_all(mat10,",",".");
    string mat11 =  boost::str(boost::format("%.2f") % (matWorldToArm(1,1))); boost::replace_all(mat11,",",".");
    string mat12 =  boost::str(boost::format("%.2f") % (matWorldToArm(1,2))); boost::replace_all(mat12,",",".");
    string mat13 =  boost::str(boost::format("%.2f") % (matWorldToArm(1,3))); boost::replace_all(mat13,",",".");
    string mat20 =  boost::str(boost::format("%.2f") % (matWorldToArm(2,0))); boost::replace_all(mat20,",",".");
    string mat21 =  boost::str(boost::format("%.2f") % (matWorldToArm(2,1))); boost::replace_all(mat21,",",".");
    string mat22 =  boost::str(boost::format("%.2f") % (matWorldToArm(2,2))); boost::replace_all(mat22,",",".");
    string mat23 =  boost::str(boost::format("%.2f") % (matWorldToArm(2,3))); boost::replace_all(mat23,",",".");
    string mat30 =  boost::str(boost::format("%.2f") % (matWorldToArm(3,0))); boost::replace_all(mat30,",",".");
    string mat31 =  boost::str(boost::format("%.2f") % (matWorldToArm(3,1))); boost::replace_all(mat31,",",".");
    string mat32 =  boost::str(boost::format("%.2f") % (matWorldToArm(3,2))); boost::replace_all(mat32,",",".");
    string mat33 =  boost::str(boost::format("%.2f") % (matWorldToArm(3,3))); boost::replace_all(mat33,",",".");

    stream << string("param T_WorldToArm {i1 in 1..4, i2 in 1..4} =  \n");
    stream << string("# 1st row \n");
    stream << string("if ( i1=1 && i2=1 ) then ")+mat00+string("  \n");
    stream << string("else	if ( i1=1 && i2=2 ) then ")+mat01+string("  \n");
    stream << string("else	if ( i1=1 && i2=3 ) then ")+mat02+string("  \n");
    stream << string("else	if ( i1=1 && i2=4 ) then ")+mat03+string("  \n");
    stream << string("# 2nd row \n");
    stream << string("else	if ( i1=2 && i2=1 ) then ")+mat10+string("  \n");
    stream << string("else	if ( i1=2 && i2=2 ) then ")+mat11+string("  \n");
    stream << string("else	if ( i1=2 && i2=3 ) then ")+mat12+string("  \n");
    stream << string("else	if ( i1=2 && i2=4 ) then ")+mat13+string("  \n");
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then ")+mat20+string("  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then ")+mat21+string("  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then ")+mat22+string("  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then ")+mat23+string("  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then ")+mat30+string("  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then ")+mat31+string("  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then ")+mat32+string("  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then ")+mat33+string("  \n");
    stream << string(";  \n");

    string idx;
    string idx1;
    for (int i = 0 ; i < JOINTS_ARM; ++i){

        idx = to_string(i);
        idx1 = to_string(i+1);

        if (final){
            stream << string("var T_")+idx+string("_")+idx1+string(" {i1 in 1..4, i2 in 1..4} =  \n");
        }else{
            stream << string("var T_")+idx+string("_")+idx1+string(" {i1 in 1..4, i2 in 1..4, i in Iterations} =  \n");
        }
        stream << string("# 1st row \n");
        if(final){
            stream << string("if ( i1=1 && i2=1 ) then cos(theta[")+idx1+string("]) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[")+idx1+string("])  \n");
        }else{
            stream << string("if ( i1=1 && i2=1 ) then cos(theta[i,")+idx1+string("]) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[i,")+idx1+string("])  \n");
        }
        stream << string("else	if ( i1=1 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a[")+idx1+string("]  \n");
        stream << string("# 2st row \n");
        if(final){
            stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[")+idx1+string("])*c_alpha[")+idx1+string("] \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[")+idx1+string("])*c_alpha[")+idx1+string("] \n");
        }else{
            stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[i,")+idx1+string("])*c_alpha[")+idx1+string("] \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[i,")+idx1+string("])*c_alpha[")+idx1+string("] \n");
        }
        stream << string("else	if ( i1=2 && i2=3 ) then -s_alpha[")+idx1+string("] \n");
        stream << string("else	if ( i1=2 && i2=4 ) then -s_alpha[")+idx1+string("]*d[")+idx1+string("] \n");
        stream << string("# 3rd row \n");
        if(final){
            stream << string("else	if ( i1=3 && i2=1 ) then sin(theta[")+idx1+string("])*s_alpha[")+idx1+string("] \n");
            stream << string("else	if ( i1=3 && i2=2 ) then cos(theta[")+idx1+string("])*s_alpha[")+idx1+string("] \n");
        }else{
            stream << string("else	if ( i1=3 && i2=1 ) then sin(theta[i,")+idx1+string("])*s_alpha[")+idx1+string("] \n");
            stream << string("else	if ( i1=3 && i2=2 ) then cos(theta[i,")+idx1+string("])*s_alpha[")+idx1+string("] \n");
        }
        stream << string("else	if ( i1=3 && i2=3 ) then c_alpha[")+idx1+string("] \n");
        stream << string("else	if ( i1=3 && i2=4 ) then c_alpha[")+idx1+string("]*d[")+idx1+string("] \n");
        stream << string("# 4th row \n");
        stream << string("else	if ( i1=4 && i2=1 ) then 0 \n");
        stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
        stream << string(";  \n");

    }

    string matHand00 =  boost::str(boost::format("%.2f") % (matHand(0,0))); boost::replace_all(matHand00,",",".");
    string matHand01 =  boost::str(boost::format("%.2f") % (matHand(0,1))); boost::replace_all(matHand01,",",".");
    string matHand02 =  boost::str(boost::format("%.2f") % (matHand(0,2))); boost::replace_all(matHand02,",",".");
    string matHand03 =  boost::str(boost::format("%.2f") % (matHand(0,3))); boost::replace_all(matHand03,",",".");
    string matHand10 =  boost::str(boost::format("%.2f") % (matHand(1,0))); boost::replace_all(matHand10,",",".");
    string matHand11 =  boost::str(boost::format("%.2f") % (matHand(1,1))); boost::replace_all(matHand11,",",".");
    string matHand12 =  boost::str(boost::format("%.2f") % (matHand(1,2))); boost::replace_all(matHand12,",",".");
    string matHand13 =  boost::str(boost::format("%.2f") % (matHand(1,3))); boost::replace_all(matHand13,",",".");
    string matHand20 =  boost::str(boost::format("%.2f") % (matHand(2,0))); boost::replace_all(matHand20,",",".");
    string matHand21 =  boost::str(boost::format("%.2f") % (matHand(2,1))); boost::replace_all(matHand21,",",".");
    string matHand22 =  boost::str(boost::format("%.2f") % (matHand(2,2))); boost::replace_all(matHand22,",",".");
    string matHand23 =  boost::str(boost::format("%.2f") % (matHand(2,3))); boost::replace_all(matHand23,",",".");
    string matHand30 =  boost::str(boost::format("%.2f") % (matHand(3,0))); boost::replace_all(matHand30,",",".");
    string matHand31 =  boost::str(boost::format("%.2f") % (matHand(3,1))); boost::replace_all(matHand31,",",".");
    string matHand32 =  boost::str(boost::format("%.2f") % (matHand(3,2))); boost::replace_all(matHand32,",",".");
    string matHand33 =  boost::str(boost::format("%.2f") % (matHand(3,3))); boost::replace_all(matHand33,",",".");

    stream << string("param T_")+idx1+string("_H {i1 in 1..4, i2 in 1..4} =  \n");
    stream << string("# 1st row \n");
    stream << string("if ( i1=1 && i2=1 ) then ")+matHand00+string("  \n");
    stream << string("else	if ( i1=1 && i2=2 ) then ")+matHand01+string("  \n");
    stream << string("else	if ( i1=1 && i2=3 ) then ")+matHand02+string("  \n");
    stream << string("else	if ( i1=1 && i2=4 ) then ")+matHand03+string("  \n");
    stream << string("# 2nd row \n");
    stream << string("else	if ( i1=2 && i2=1 ) then ")+matHand10+string("  \n");
    stream << string("else	if ( i1=2 && i2=2 ) then ")+matHand11+string("  \n");
    stream << string("else	if ( i1=2 && i2=3 ) then ")+matHand12+string("  \n");
    stream << string("else	if ( i1=2 && i2=4 ) then ")+matHand13+string("  \n");
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then ")+matHand20+string("  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then ")+matHand21+string("  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then ")+matHand22+string("  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then ")+matHand23+string("  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then ")+matHand30+string("  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then ")+matHand31+string("  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then ")+matHand32+string("  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then ")+matHand33+string("  \n");
    stream << string(";  \n");

    // positions on the arm
    if(final){
        stream << string("var T_W_1 {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}   T_WorldToArm[i1,j]*T_0_1[j,i2];\n");
    }else{
        stream << string("var T_W_1 {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}   T_WorldToArm[i1,j]*T_0_1[j,i2,i];\n");
    }
    for (int i = 1 ; i < JOINTS_ARM; ++i){
        idx = to_string(i);
        idx1 = to_string(i+1);
        if(final){
            stream << string("var T_W_")+idx1+string(" {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}   T_W_")+idx+string("[i1,j]*T_")+idx+string("_")+idx1+string("[j,i2];\n");
        }else{
            stream << string("var T_W_")+idx1+string(" {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}   T_W_")+idx+string("[i1,j,i]*T_")+idx+string("_")+idx1+string("[j,i2,i];\n");
        }
    }
    if(final){
        stream << string("var T_W_H {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}   T_W_")+idx1+string("[i1,j]*T_")+idx1+string("_H[j,i2];\n\n");
    }else{
        stream << string("var T_W_H {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}   T_W_")+idx1+string("[i1,j,i]*T_")+idx1+string("_H[j,i2];\n\n");
    }

    string tolArm1 =  boost::str(boost::format("%.2f") % tolsArm.at(0)); boost::replace_all(tolArm1,",",".");
    string tolArm2 =  boost::str(boost::format("%.2f") % tolsArm.at(1)); boost::replace_all(tolArm2,",",".");
    string tolArm3 =  boost::str(boost::format("%.2f") % tolsArm.at(2)); boost::replace_all(tolArm3,",",".");
    string tolArm4 =  boost::str(boost::format("%.2f") % tolsArm.at(3)); boost::replace_all(tolArm4,",",".");

    if(final){
        stream << string("var Shoulder {i in 1..4} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_1[i,4] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm1+string("\n");
        stream << string(";  \n");

        stream << string("var Elbow {i in 1..4} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_3[i,4] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm2+string("\n");
        stream << string(";  \n");

        stream << string("var Wrist {i in 1..4} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_5[i,4] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm3+string("\n");
        stream << string(";  \n");

        stream << string("var Hand {i in 1..4} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_H[i,4] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm4+string("\n");
        stream << string(";  \n");

        stream << string("# Hand orientation \n");
        stream << string("var x_H {j in 1..3} = T_W_H [j,1]; \n");
        stream << string("var y_H {j in 1..3} = T_W_H [j,2]; \n");
        stream << string("var z_H {j in 1..3} = T_W_H [j,3]; \n");

    }else{
        stream << string("var Shoulder {i in 1..4,j in Iterations} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_1[i,4,j] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm1+string("\n");
        stream << string(";  \n");

        stream << string("var Elbow {i in 1..4,j in Iterations} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_3[i,4,j] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm2+string("\n");
        stream << string(";  \n");

        stream << string("var Wrist {i in 1..4,j in Iterations} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_5[i,4,j] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm3+string("\n");
        stream << string(";  \n");

        stream << string("var Hand {i in 1..4,j in Iterations} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_H[i,4,j] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm4+string("\n");
        stream << string(";  \n");

        stream << string("# Hand orientation \n");
        stream << string("var x_H {j in 1..3,i in Iterations} = T_W_H [j,1,i]; \n");
        stream << string("var y_H {j in 1..3,i in Iterations} = T_W_H [j,2,i]; \n");
        stream << string("var z_H {j in 1..3,i in Iterations} = T_W_H [j,3,i]; \n");
    }


}



void Problem::writeObjective(ofstream &stream, bool final)
{

    stream << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
    stream << string("#  \n");
    stream << string("#		       Objective function          # \n");
    stream << string("#  \n");
    if(final){
        stream << string("minimize z: sum {j in nJoints} (lambda[j]*(thet_init[j]-theta[j])^2); \n");
    }else{
        stream << string("minimize z: sum {j in nJoints} (lambda[j]*(thet_init[j]-theta_b[j])^2); \n");
    }
    stream << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");

}


void Problem::writeBodyConstraints(ofstream &stream, bool final)
{

    stream << string("# Constraints with the body: the body is modeled as a cylinder \n");
    if (final){

        stream << string("subject to BodyArm_Elbow: (Elbow[1]/body[1])^2 + (Elbow[2]/body[2])^2 >= 1; \n");
        stream << string("subject to BodyArm_Wrist: (Wrist[1]/body[1])^2 + (Wrist[2]/body[2])^2 >= 1; \n");
        stream << string("subject to BodyArm_Hand:  (Hand[1]/body[1])^2  + (Hand[2]/body[2])^2  >= 1; \n\n");
    }else{
        stream << string("subject to BodyArm_Elbow{l in Iterations}: (Elbow[1,l]/body[1])^2 + (Elbow[2,l]/body[2])^2 >= 1; \n");
        stream << string("subject to BodyArm_Wrist{l in Iterations}: (Wrist[1,l]/body[1])^2 + (Wrist[2,l]/body[2])^2 >= 1; \n");
        stream << string("subject to BodyArm_Hand{l in Iterations}:  (Hand[1,l]/body[1])^2  + (Hand[2,l]/body[2])^2  >= 1; \n\n");

    }

}



void Problem::writeTableConstraints(ofstream &stream, bool final, int griptype, std::vector<float> &tols_table,int steps)
{

    stream << string("# Constraints with the Table \n");

    string t_table_w = boost::str(boost::format("%.2f") % tols_table.at(0)); boost::replace_all(t_table_w,",",".");
    string t_table_h = boost::str(boost::format("%.2f") % tols_table.at(1)); boost::replace_all(t_table_h,",",".");
    string t_table_f = boost::str(boost::format("%.2f") % tols_table.at(2)); boost::replace_all(t_table_f,",",".");

    if(final){
        stream << string("subject to Table_Wrist: Wrist[3] 		- Wrist[4] 		-Table[2] >= ")+t_table_w+string("; \n");
        stream << string("subject to Table_Hand:  Hand[3] 		- Hand[4]  		-Table[2] >= ")+t_table_h+string("; \n");
        //stream << string("subject to Table_fingers{k in 7..18}: Points_Arm[k,3] - Points_Arm[k,4] - Table[2] >= ")+t_table_f+string("; \n");
        switch (griptype) {
        case 111: case 112: case 113: case 114: case 121: case 122:
            // precision
            stream << string("subject to Table_fingers{k in 7..21}: Points_Arm[k,3] - Points_Arm[k,4] - Table[2] >= ")+t_table_f+string("; \n");
            break;
        case 211: case 212: case 213: case 214: case 221: case 222:
            //full
            stream << string("subject to Table_fingers{k in 7..18}: Points_Arm[k,3] - Points_Arm[k,4] - Table[2] >= ")+t_table_f+string("; \n");
            break;
        }
    }else{
        stream << string("subject to Table_Wrist{l in Iterations}: Wrist[3,l] 	- Wrist[4,l] 	- Table[2] >= ")+t_table_w+string("; \n");
        stream << string("subject to Table_Hand{l in Iterations}:  Hand[3,l] 		- Hand[4,l]  	- Table[2] >= ")+t_table_h+string("; \n");
        stream << string("subject to Table_fingers{(k,l) in 7..15 cross Iterations}: Points_Arm[k,3,l]- Points_Arm[k,4,l] - Table[2] >= ")+t_table_f+string("; \n");


    }


}


void Problem::getObstaclesSingleArm(std::vector<objectPtr> objs, std::vector<float> center, float radius, std::vector<objectPtr> & obsts)
{


        // position of the shoulder = center of the workspace
        pos shPos;
        shPos.Xpos = center.at(0);
        shPos.Ypos = center.at(1);
        shPos.Zpos = center.at(2);

        // get the tolerance [mm]
#if HAND == 0
        float tol = 50.0;
#elif HAND == 1
        float tol = 5.0;
#endif


        for (std::size_t i = 0; i < objs.size(); ++i){
            objectPtr obj = objs.at(i);
            if ((!obj->isTargetRightEnabled()) && (!obj->isTargetLeftEnabled())){

                // get the RPY matrix
                Matrix3f Rot;
                obj->RPY_matrix(Rot);
                // get the position of the object
                pos objPos = obj->getPos();
                // get the size of the object
                dim objSize = obj->getSize();

                // distance vector
                Vector3f diff;
                diff(0) = shPos.Xpos - objPos.Xpos;
                diff(1) = shPos.Ypos - objPos.Ypos;
                diff(2) = shPos.Zpos - objPos.Zpos;

                // A matrix
                Matrix3f A;
                A(0,0) = 1/pow(radius+objSize.Xsize+tol,2); A(0,1) = 0; A(0,2) = 0;
                A(1,0) = 0; A(1,1) = 1/pow(radius+objSize.Ysize+tol,2); A(1,2) = 0;
                A(2,0) = 0; A(2,1) = 0; A(2,2) = 1/pow(radius+objSize.Zsize+tol,2);

                MatrixXf diff1 = MatrixXf::Zero(3,1);
                diff1(0,0) = diff(0); diff1(1,0) = diff(1); diff1(2,0) = diff(2);

                MatrixXf diffT = diff1.transpose();
                MatrixXf RotT = Rot.transpose();

                MatrixXf to_check = diffT*RotT;
                to_check = to_check * A;
                to_check = to_check * Rot;
                to_check = to_check * diff;

                if (to_check(0,0) < 1){
                    // the object is an obstacle
                    obsts.push_back(obj);

                }

            }


    }
}



} // namespace HUMotion
