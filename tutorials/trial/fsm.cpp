#include <Eigen/Dense>
#include <iostream>
using namespace std;
using namespace Eigen;

class FSM{
    public:
        //// Constructor
        FSM(float _duration_time, int _dof, VectorXd _goalPos)
            : duration_time(_duration_time),
            dof(_dof),
            goalPos(_goalPos);
        //// Get
        float get_duration_time(){return duration_time;}
        int get_dof(){return dof;}
        VectorXd get_goalPos(){return goalPos;}
        //// Set (change) -'TODO
    private:
        float duration_time;
        int dof;
        VectorXd goalPos(dof);
}
