#include <Eigen/Dense>
#include <iostream>
using namespace std;
using namespace Eigen;

class FSM_state{
    public:
        //// Constructor
        FSM_state(float _duration_time, int _dof, VectorXd _goalPos)
            : duration_time(_duration_time),
            dof(_dof){
                goalPos(dof);
                goalPos=_goalPos;
            }
        //// Get
        float get_duration_time(){return duration_time;}
        int get_dof(){return dof;}
        VectorXd get_goalPos(){return goalPos;}
        //// Set (change) -'TODO
    private:
        float duration_time;
        int dof;
        VectorXd goalPos;
};

class transition{
    public:
        transition(int _from, int _input, int _to): from(_from), input(_input), to(_to) {}
        int from;
        int input;
        int to;
};
class FSM{
    public:
        FSM(){ state_num=0; cur_state=-1; }
        FSM_state get_state(int i){ return states[i]; }
        void add_state(FSM_state newstate) {states.push_back(newstate); state_num++; }
        void add_transition(transition newtr) {transitions.push_back(newtr); }
        void goto_next_state(int input=0);
    private:
        vector<FSM_state> states;
        vector<transition> transitions;
        int state_num;
        int cur_state;
};
void FSM::goto_next_state(int input){
    for(int i=0; i<transitions.size(); i++){
        if(transitions[i].from==cur_state && transitions[i].input==input){
            cur_state= transitions[i].to;
        }
    }
}


