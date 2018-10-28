#ifndef __FSM_H__
#define __FSM_H__

#include <Eigen/Dense>
#include <iostream>
#include <chrono>
#include <vector>
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
        FSM(){ state_num=0; cur_state=-1; automode=true; }
        FSM_state get_state(int i){ return states[i]; }
        int get_cur_state_n() {return cur_state; }
        VectorXd get_goalPos(){return states[cur_state].get_goalPos(); }
        float get_cur_duration(){return states[cur_state].get_duration_time(); }
        void add_state(FSM_state newstate) {states.push_back(newstate); state_num++; }
        void add_transition(transition newtr) {transitions.push_back(newtr); }
        void goto_next_state(int input=0);
        void set_start(){
            if(state_num>=2) cur_state=1;
            start=std::chrono::system_clock::now();
        }
        bool isAutomode(){return automode;}
        void change_automode(){
            automode=!automode;
        }
        void print_fsm(){
            for(int i=0; i<state_num; i++){
                states[i].print_state();
            }
        }
        VectorXd timeStepping(); //Controller controller);
    private:
        vector<FSM_state> states;
        vector<transition> transitions;
        int state_num;
        int cur_state;
        bool automode;
        chrono::system_clock::time_point start;
};

#endif