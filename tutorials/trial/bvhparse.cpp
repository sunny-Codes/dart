#include <vector>
#include <string.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <stdio.h>
#include <cmath>
#include <algorithm>
#include <vector>
#include <istream>
#include <fstream>
#include <list>
#include <map>
#include "opengl_basic.h"
using namespace std;
using namespace Eigen;
//JOINT, HIERARCHY is for structure info (& offset(OFFSET) info- no rotation included)
typedef struct JOINT JOINT;

#define POS_X 0
#define POS_Y 1
#define POS_Z 2
#define ROT_X 3
#define ROT_Y 4
#define ROT_Z 5

int convert_to_channel_code(string word){
    if (word=="Xposition"){
        return POS_X;
    }else if (word=="Yposition"){
        return POS_Y;
    }else if (word=="Zposition"){
        return POS_Z;
    }else if (word=="Xrotation"){
        return ROT_X;
    }else if (word=="Yrotation"){
        return ROT_Y;
    }else if (word=="Zrotation"){
        return ROT_Z;
    }else{
        cout<<"ERR: wrong channel string ("<<word<<")"<<endl;
        return -1;
    }
}
Matrix4f calculate_pos_matrix(float x, float y, float z){
    Matrix4f p= Matrix4f::Identity();
    p(0,3)=x;
    p(1,3)=y;
    p(2,3)=z;
    return p;
}

Matrix4f calculate_transform_matrix(int channel_code, float dof_value){
    Matrix4f M= Matrix4f::Identity();
    // dof_value: deg -> convert to rad
    float cos_t= cos(deg_to_rad(dof_value));
    float sin_t= sin(deg_to_rad(dof_value));

    switch(channel_code){
        case POS_X:
            M(0,3)=dof_value;
            break;
        case POS_Y:
            M(1,3)=dof_value;
            break;
        case POS_Z:
            M(2,3)=dof_value;
            break;
        case ROT_X:
            M.block(1,1,2,2) << cos_t, -sin_t, sin_t, cos_t;
            break;
        case ROT_Y:
            M(0,0)=cos_t;
            M(0,2)=sin_t;
            M(2,0)=-sin_t;
            M(2,2)=cos_t;
            break;
        case ROT_Z:
            M.block(0,0,2,2)<< cos_t, -sin_t, sin_t, cos_t;
            break;
        default:
            cout<<"ERR: wrong channel_code ("<<channel_code<<")"<<endl;
            break;
    }
    return M;
}

Matrix4f convert_offset_to_matrix(Vector3f v){
    Matrix4f M=Matrix4f::Identity();
    M(0,3)=v[0];
    M(1,3)=v[1];
    M(2,3)=v[2];
    return M;
}
struct JOINT
{
    string name = "";
    JOINT * parent= NULL; //new JOINT;
    Vector3f offset_v;  // offset from parent
    Matrix4f offset_m;	
    int channel_n=0; // number of channels;
    vector<int> channel_order; //e.g. Z X Y
    vector<JOINT *> children;
    int index; //position
};

typedef struct{
    JOINT* rootJoint=NULL;
    int channel_n_tot;
    vector<JOINT*> joints; //in dfs order
}HIERARCHY;

typedef struct{
    vector<float> channel_data;
    Matrix4f relative_tf;
    Matrix4f parent_fk;
    Matrix4f forward_kinematics;

    Vector3f origin;
    Vector3f x_axis;
    Vector3f y_axis;
    Vector3f z_axis;

    Vector3f p_origin;
}JOINT_FRAME_DATA;

class BVH{
    private:
        HIERARCHY* hierarchy;
        int frame_n;
        int data_per_frame=0;
        int tmp_index=0;
        vector< vector<JOINT_FRAME_DATA>> frames;
        vector< string > joint_order;
        //	list<JOINT *> schema_address;
        void load_hierarchy(istream & stream);
        JOINT* load_joint(bool isroot, istream &stream, JOINT * parent=NULL);
        void load_frame(istream & stream);
        void load_motion(istream & stream);
    public:
        BVH(){hierarchy= new HIERARCHY;} 
        // frames= new vector<vector<JOINT*>>(); }//frames= new vector<map<string, JOINT_FRAME_DATA>>(); };
        ~BVH(){};
void load_file(const string &filename);
HIERARCHY* get_hierarchy(){return hierarchy;}
vector<vector<JOINT_FRAME_DATA>> get_frames(){return frames;}
Vector3f get_initial_root_pos(); //return root_pos;}
//JOINT* get_joint_data(string name);
};

Vector3f BVH::get_initial_root_pos(){
    return hierarchy->joints[0]->offset_v;
}
void BVH::load_file(const string &filename){
    fstream file;
    file.open(filename.c_str(), ios_base::in);
    if(file.is_open()){
        string word;
        while(file.good()){
            file>>word;
            if(word=="HIERARCHY"){
                load_hierarchy(file);
                break;
            }
        }
        file.close();
    }
}

void BVH::load_hierarchy(istream &stream){
    hierarchy->joints=vector<JOINT*>();
    string word;
    while (stream.good()){
        stream >> word;
        if (word=="ROOT")
            hierarchy->rootJoint = load_joint(true, stream);
        else if (word=="MOTION")
        {	load_motion(stream); break;}
    }
}

JOINT* BVH::load_joint(bool isroot, istream &stream, JOINT * parent){
    JOINT *joint = new JOINT;
    joint->parent= parent;

    string word_name;
    string word;
    stream>> word_name;
    joint->name= word_name; //word.c_str();

    joint->index= tmp_index;
    tmp_index++;
    joint_order.push_back (word_name);
    hierarchy->joints.push_back(joint);
    //joint_map.push_back(word_name, joint);

    stream>>word;
    if(word!="{"){
        cout<<"wrong format"<<endl;
        return NULL;
    }
    stream>>word;
    if(word!="OFFSET"){
        cout<<"wrong format"<<endl;
        return NULL;
    }
    stream>>word;
    string tmp2;
    string tmp3;
    stream>> tmp2;
    stream>> tmp3;
    joint-> offset_v= Vector3f(stof(word), stof(tmp2), stof(tmp3));
    joint->offset_m= convert_offset_to_matrix(joint->offset_v);

    stream>>word;
    if(word!="CHANNELS"){
        cout<<"wrong format"<<endl;
        return NULL;
    }
    stream>>word;
    joint->channel_n= stoi(word);
    data_per_frame+= stoi(word);
    joint->channel_order= vector<int>();

    for(int i=0; i<joint->channel_n; i++){
        stream>>word;
        int channel_code= convert_to_channel_code(word);
        joint->channel_order.push_back(channel_code);
    }

    while(stream.good()){
        stream>>word;
        if(word=="JOINT"){
            JOINT * tmp_joint= load_joint(false,stream, joint);
            tmp_joint->parent=joint;
            joint->children.push_back(tmp_joint);
        }else if(word=="End"){
            stream >>word >> word; // Site{

            JOINT* tmp_joint = new JOINT;
            tmp_joint->parent = joint;
            tmp_joint->channel_n = 0;
            tmp_joint->name = "EndSite";
            joint->children.push_back(tmp_joint);
            stream >> word;
            if (word=="OFFSET"){
                string tmp1;
                string tmp2;
                string	tmp3;
                stream >> tmp1 >>tmp2>> tmp3;
                tmp_joint->offset_v=Vector3f(stof(tmp1), stof(tmp2), stof(tmp3));
                tmp_joint->offset_m=convert_offset_to_matrix(tmp_joint->offset_v);
                //tmp_joint->offset.x >> tmp_joint->offset.y >> tmp_joint->offset.z;
                stream>>word;//RP (End ENDSITE): 
            }
        }else if(word=="}") {return joint;}
        }
    }

    void BVH::load_motion(istream & stream){
        //cout<<"load motion entered"<<endl;
        //cout<<"data_per_frame:"<< data_per_frame<<endl;
        string word;
        stream>>word>>word; //Frames:
        frame_n = stoi(word);
        stream>>word>>word>>word; // Frame Time: ...

        string line;
        for(int i=0; i<frame_n; i++){
            vector<JOINT_FRAME_DATA> a_frame;
            //for(vector<string>::iterator it= joint_order.begin(); it!=joint_order.end(); it++){
            for(int j=0; j< joint_order.size();j++){
                //string joint_name= *it;
                JOINT* joint_data= hierarchy->joints[j];
                JOINT_FRAME_DATA new_frame_data;
                new_frame_data.relative_tf= Matrix4f::Identity();

                for(vector<int>::iterator ch_it= joint_data->channel_order.begin(); ch_it!=joint_data->channel_order.end(); ch_it++){
                    int channel_code= *ch_it;
                    string t;
                    stream>> t;
                    Matrix4f m= calculate_transform_matrix(channel_code, stof(t));
                    new_frame_data.relative_tf= new_frame_data.relative_tf *m;
                }
                /*            Matrix4f p= Matrix4f::Identity();
                              if(joint_data->channel_order.size()!=3){
                              string px, py, pz;
                              stream>>px>>py>>pz;
                              p= calculate_pos_matrix(stof(px),stof(py),stof(pz));
                              }
                              string rz, rx, ry;
                              stream>>rz>>rx>>ry;
                              Matrix4f r= calculate_transform_matrix(ROT_Z, stof(rz))*calculate_transform_matrix(ROT_X, stof(rx))*calculate_transform_matrix(ROT_Y, stof(ry));

                              new_frame_data.relative_tf= r*p; //p*r;
                              */
                if(!(joint_data->parent)) {
                    new_frame_data.parent_fk= Matrix4f::Identity();
                    new_frame_data.p_origin= Vector3f(0,0,0);
                }else{
                    int parent_idx= joint_data->parent->index;
                    JOINT_FRAME_DATA *parent_frame_data= &a_frame[parent_idx];//&frames[i][parent_idx];
                    new_frame_data.parent_fk= parent_frame_data->forward_kinematics;
                    new_frame_data.p_origin= parent_frame_data->origin; 
                }
                new_frame_data.forward_kinematics=
                    new_frame_data.parent_fk*joint_data->offset_m* new_frame_data.relative_tf ;
                //new_frame_data.relative_tf * joint_data->offset_m * new_frame_data.parent_fk; 
                Matrix4f new_fk= new_frame_data.forward_kinematics;
                new_frame_data.origin= Vector3f(new_fk(0,3),new_fk(1,3),new_fk(2,3));
                new_frame_data.x_axis= Vector3f(new_fk(0,0), new_fk(1,0), new_fk(2,0));
                new_frame_data.y_axis= Vector3f(new_fk(0,1), new_fk(1,1), new_fk(2,1));
                new_frame_data.z_axis= Vector3f(new_fk(0,2), new_fk(1,2), new_fk(2,2));

                a_frame.push_back(new_frame_data);
                //cout<<"rot:"<< (newdata.raw_rot).transpose()<<" "<<a_frame[j].raw_rot.transpose()<<endl;
            }
            frames.push_back(a_frame);
            //cout<<frames[i][joint_order.size()-1].raw_rot.transpose()<<endl;

        }
        //cout<<"load_motion done"<<endl;
        }

