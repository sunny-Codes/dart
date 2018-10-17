#include "fsm.h"

void FSM::goto_next_state(int input){
    for(int i=0; i<transitions.size(); i++){
        //transitions[i].print();
        if((transitions[i].from==cur_state) && (transitions[i].input==input)){
            cur_state= transitions[i].to;
            break;
        }
    }

}


VectorXd FSM::timeStepping(){ //Controller controller){
    if(automode){
        chrono::system_clock::time_point now= std::chrono::system_clock::now();
        float diff=(float) std::chrono::duration_cast<std::chrono::milliseconds> (now-start).count();
        if(diff > get_cur_duration()) {
            goto_next_state();
           // controller->mGoalPos= get_goalPos();
            start= now;
        }
    }
    return get_goalPos();
}
