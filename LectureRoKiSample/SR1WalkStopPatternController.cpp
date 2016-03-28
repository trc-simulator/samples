#include <cnoid/SimpleController>
#include <cnoid/Link>
#include <cnoid/BodyMotion>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <cnoid/BasicSensors>
#include "Interpolator.h"

using namespace std;
using namespace cnoid;

static double pgain[] = {
    8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
    3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0,
    8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
    3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0,
    8000.0, 8000.0, 8000.0 };

static double dgain[] = {
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0 };

class SR1WalkStopPatternController : public cnoid::SimpleController
{
    BodyPtr body;
    Interpolator<VectorXd> interpolator;
    MultiValueSeqPtr qseq;
    VectorXd qref, qold, qref_old;
    int currentFrame;
    double timeStep_;
    int numJoints;
    int phase;
    double time;

public:

    virtual bool initialize() {

        if(!qseq){
            string filename = getNativePathString(
                boost::filesystem::path(shareDirectory())
                / "motion" / "SR1" / "SR1WalkPattern3.yaml");

            BodyMotion motion;
            if(!motion.loadStandardYAMLformat(filename)){
                os() << motion.seqMessage() << endl;
                return false;
            }
            qseq = motion.jointPosSeq();
            if(qseq->numFrames() == 0){
                os() << "Empty motion data." << endl;
                return false;
            }
            timeStep_ = qseq->getTimeStep();
        }

        if(fabs(timeStep() - timeStep_) > 1.0e-6){
            os() << "Time step must be " << timeStep_ << "." << endl;;
            return false;
        }

        body = ioBody();

        numJoints = body->numJoints();
        if(numJoints != qseq->numParts()){
            os() << "The number of joints must be " << qseq->numParts() << endl;
            return false;
        }

        qold.resize(numJoints);
        VectorXd q0(numJoints);
        VectorXd q1(numJoints);
        for(int i=0; i < numJoints; ++i){
            qold[i] = q0[i] = body->joint(i)->q();
        }
        MultiValueSeq::Frame frame = qseq->frame(0);
        for(int i=0; i < numJoints; ++i){
            q1[i] = frame[i];
        }
        interpolator.clear();
        interpolator.appendSample(0.0, q0);
        interpolator.appendSample(2.0, q1);
        interpolator.update();

        qref_old = interpolator.interpolate(0.0);

        currentFrame = 0;

        phase = 0;
        time = 0.0;

        return true;
    }

    virtual bool control() {

        switch(phase){
        case 0 :
            qref = interpolator.interpolate(time);
            if(time > interpolator.domainUpper()){
                phase = 1;
            }
            break;
        case 1:
            if(currentFrame < qseq->numFrames()){
                MultiValueSeq::Frame frame = qseq->frame(currentFrame++);
                for(int i=0; i < numJoints; ++i){
                    qref[i] = frame[i];
                }
            }else{
                phase = 2;
            }
            break;
        case 2 :
            qref = interpolator.interpolate(time);
        }

        for(int i=0; i < body->numJoints(); ++i){
            Link* joint = body->joint(i);
            double q = joint->q();
            double dq_ref = (qref[i] - qref_old[i]) / timeStep_;
            double dq = (q - qold[i]) / timeStep_;
            joint->u() = (qref[i] - q) * pgain[i] + (dq_ref - dq) * dgain[i];
            qold[i] = q;
        }
        qref_old = qref;

        time += timeStep_;

        return true;
    }

};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SR1WalkStopPatternController)
