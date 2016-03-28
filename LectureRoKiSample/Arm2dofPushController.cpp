#include <cnoid/SimpleController>
#include <cnoid/BodyState>
#include <cnoid/EigenUtil>

using namespace std;
using namespace cnoid;
using namespace boost;

#define DOF (2)
#define K 12000
#define C 120

class Arm2dofControllerGeard : public cnoid::SimpleController
{

public:
    VectorXd qref;
    double time;

    virtual bool initialize() {
        const BodyPtr& io = ioBody();
        for(int i=0; i<DOF; i++)
            io->joint(i)->u() = 0.0;
        qref.resize(DOF);
        return true;
    }

    virtual bool control() {

        qref[0] = qref[1] = radian(90);

        const BodyPtr& io = ioBody();
        for(int i=0; i<DOF; i++){
            double q = io->joint(i)->q();
            double dq = io->joint(i)->dq();
            io->joint(i)->u() = -K*(q-qref[i]) - C*dq;
        }

        double dt = timeStep();
        time += dt;
        return true;
    }

};


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(Arm2dofControllerGeard);
