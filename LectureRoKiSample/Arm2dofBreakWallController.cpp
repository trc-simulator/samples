#include <cnoid/SimpleController>
#include <cnoid/BodyState>
#include <cnoid/EigenUtil>

using namespace std;
using namespace cnoid;
using namespace boost;

#define DOF (2)
#define K 4800
#define C 48
#define T1 5.0
#define T2 7.0

class Arm2dofBreakWallController : public cnoid::SimpleController
{

public:
	VectorXd qref, qinit, dqref;
    double time;

    virtual bool initialize() {
        const BodyPtr& io = ioBody();
				qinit.resize(DOF);
        for(int i=0; i<DOF; i++){
            io->joint(i)->u() = 0.0;
						qinit[i] = io->joint(i)->q();
				}
        qref.resize(DOF);
				dqref.resize(DOF);
        return true;
    }

    virtual bool control() {

			if( time < T1 ){
				qref[1] = qinit[1] + time / T1 * (radian(-45) - qinit[1]);
				dqref[1] = (radian(90) - qinit[1]) / T1;
			}else if( time < T2 ){
        qref[1] = radian(-90);
				dqref[1] = 0.0;
			}else{
				qref[1] = radian(90);
				dqref[1] = 0.0;
			}
			qref[0] = radian(90);
			dqref[0] = 0.0;

        const BodyPtr& io = ioBody();
        for(int i=0; i<DOF; i++){
            double q = io->joint(i)->q();
            double dq = io->joint(i)->dq();
            io->joint(i)->u() = -K*(q-qref[i]) - C*(dq-dqref[i]);
        }

        double dt = timeStep();
        time += dt;
        return true;
    }

};


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(Arm2dofBreakWallController);
