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
        void print_state(){
            cout<<"duration: "<<duration_time<<"ms"<<endl;
            cout<<"goal pos: "<<goalPos.transpose()<<endl;
        }
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
        void print(){cout<<from<<" -- "<<input<<" --> "<<to<<endl;}
};
class FSM{
    public:
        FSM(){ state_num=0; cur_state=-1; }
        FSM_state get_state(int i){ return states[i]; }
        VectorXd get_goalPos(){return states[cur_state].get_goalPos(); }
        float get_cur_duration(){return states[cur_state].get_duration_time(); }
        void add_state(FSM_state newstate) {states.push_back(newstate); state_num++; }
        void add_transition(transition newtr) {transitions.push_back(newtr); }
        void goto_next_state(int input=0);
        void set_start(){
            if(state_num!=0) cur_state=0;
        }
        void print_fsm(){
            for(int i=0; i<state_num; i++){
                states[i].print_state();
            }
        }
    private:
        vector<FSM_state> states;
        vector<transition> transitions;
        int state_num;
        int cur_state;
};
void FSM::goto_next_state(int input){
    cout<<"cur state: "<<cur_state<<endl;
    cout<<"input: "<<input<<endl;
    for(int i=0; i<transitions.size(); i++){
        transitions[i].print();
        if((transitions[i].from==cur_state) && (transitions[i].input==input)){
            cout<<"to: "<<transitions[i].to<<endl;
            cur_state= transitions[i].to;
            cout<<"cur_state changed:"<<cur_state<<endl;
            break;
        }
    }
    cout<<get_goalPos()<<endl;
}


