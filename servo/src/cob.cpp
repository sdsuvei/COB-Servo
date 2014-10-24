#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <rw/rw.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include "std_msgs/Bool.h"
#include <rw/models/Device.hpp>
#include <ros/package.h>
#include <vector>
#include <time.h>
#include <rw/math/Quaternion.hpp>
#include "tf/transform_listener.h"
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <rw/kinematics/MovableFrame.hpp>

#define pi 3.14159265
#define epsilon 0.000001
#define deltatess 0.1

using namespace std;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::math;
using namespace rwlibs::proximitystrategies;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

//Time variables
double T1,T2,Ttotal,sec=0;
double inc=0;
//Safety flag
int flag=0;

//Function definitions
Q computenewQ(Q qin, Transform3D<> urTmrk, Transform3D<> urTobj, Device::Ptr UR, State state,Frame* marker,Frame* bottleframe);
Vector3D<> getW(Rotation3D<> r);
VelocityScrew6D<> computeG(Transform3D<> urTmrk, Transform3D<> urTobj);
Transform3D<> computeinterpolation(Transform3D<> urTmrk, Transform3D<> urTobj,double delta);
Transform3D<> rw_camTobj(Device::Ptr UR, State state,Frame* bottle,Frame* camera);
Transform3D<> find_urTcam(Device::Ptr UR, State state,Frame* camera);
Transform3D<> find_camTobj(Device::Ptr UR, State state,Frame* bottle,Frame* camera,Transform3D<> previous);
Transform3D<> find_camTmrk(Device::Ptr UR, State state,Frame* marker,Frame* camera,Transform3D<> previous);

//Callback function
float joint_pos[6];
bool itHappened=false;
string name[6] ;
ros::Subscriber sub;

void jointCallback(const sensor_msgs::JointState & msg){
	ROS_INFO("RECEIVED JOINT VALUES");
	if(itHappened){
		sub.shutdown();
		cout<<"not again"<<endl;
		return;
	}
	if (msg.name.size()== 6 ) {
	itHappened=true;
	for (int i=0;i<6;i++){
	joint_pos[i] = msg.position[i];
	name[i] = msg.name[i];
//	cout<<" , "<<name[i]<<":"<<joint_pos[i];
	}
	sub.shutdown();
	}
}

int main(int argc, char** argv) {
	//get wcFile and device name
	string wcFile = "/home/sdsuvei/robwork/RobWorkStudio/scenes/COBLab/KitchenScene.wc.xml";
	string robotName = "UR";
	string gripperName = "SDH";
	std::string connectorName = "URConnector";
	cout << "Trying to use workcell: " << wcFile << " and device " << robotName << endl;

	//load work cell
	WorkCell::Ptr wc = WorkCellFactory::load(wcFile);
	Device::Ptr UR = wc->findDevice(robotName);
	Device::Ptr gripper = wc->findDevice(gripperName);
	Device::Ptr connector = wc->findDevice(connectorName);
	if (UR == NULL) {
		cerr << "Device: " << robotName << " not found!" << endl;
		return 0;
	}

	//default state of the RobWork Workcell
	State state = wc->getDefaultState();

	//load the collision detector
	CollisionDetector cd(wc,ProximityStrategyFactory::makeDefaultCollisionStrategy());

	//ROS initialization
	ros::init(argc, argv, "listener");
	ros::NodeHandle nh;
	ros::Rate loop_rate(5);
	sub = nh.subscribe ("/joint_states", 1, jointCallback);

	while(ros::ok())
	{
		if(itHappened)
		{

	//RobWork robot model initialization
	//starting configuration of the UR5 arm
//	Q pos(6,0.0785, -1.7667, 2.037, -2.1101, -2.1724, -1.8505);
//	Q pos(6,-0.3946, -1.8944, 1.9378, -1.3807, -1.1023, -2.4439);
	Q pos(6, joint_pos[0], joint_pos[1], joint_pos[2], joint_pos[3], joint_pos[4], joint_pos[5]);
	cout<<"Initial arm configuration:\n"<<pos<<"\n";
	//set the initial position of the UR5 arm
	UR->setQ(pos,state);

	//starting configuration of the SDH gripper
	Q grip(7,-1.047,0.000,0.000,-0.765,-0.282,-0.846,0);
	//set the initial position of the gripper
	gripper->setQ(grip,state);

	//half-away configuration of the URconnector
	Q conn(1,-1.597);
	//set the half-away configuration of the URconnector
	connector->setQ(conn,state);

	//RobWork Workcell frame definitions
//	Frame* bottle = wc->findFrame("BottleTray"); //bootle frame
	MovableFrame* bottle = (MovableFrame*)wc->findFrame("BottleTray");
	Frame* marker=wc->findFrame("SDH.Marker"); //marker frame
	Frame* camera=wc->findFrame("Camera3D"); //camera frame
	Frame* pregrasp=wc->findFrame("Pregrasp"); // pregrasping frame

	if(cd.inCollision(state))
	{
		cout<<"Before change: In collision!\n";
	}

	//update the bottle frame with the value from the tracker
	Transform3D<> camTobj_rw=rw_camTobj(UR,state,bottle,camera);
	//change the object frame in the RobWork scene
	bottle->setTransform(camTobj_rw,state);

	if(cd.inCollision(state))
		{
			cout<<"After change: In collision!\n";
		}
	//store the kinematic values, from the RobWork workcell
	Transform3D<> kinematic_camTobj=Kinematics::frameTframe(camera,bottle,state);
	Transform3D<> kinematic_camTmrk=Kinematics::frameTframe(camera,marker,state);

    //compute the intermediate transforms
	// 1)URbase -> MARKER transform
	Transform3D<> urTmrk  = UR->baseTframe(marker,state);

    // 2)CAMERA -> OBJECT transform
	Transform3D<> camTobj=find_camTobj(UR,state,bottle,camera,kinematic_camTobj);

    // 3)CAMERA -> MARKER transform
	Transform3D<> camTmrk=find_camTmrk(UR,state,marker,camera,kinematic_camTmrk);

    // 4)MARKER -> OBJECT transform
	Transform3D<> mrkTobj=inverse(camTmrk)*camTobj;

	// 5)OBJECT -> PREGRASP transform
//	Transform3D<> objTpg=Kinematics::frameTframe(bottle,pregrasp,state);

	// 6)URbase -> PreGrasping frame
	Transform3D<> objTpg=Kinematics::frameTframe(bottle,pregrasp,state);
	Transform3D<> urTcam=find_urTcam(UR,state,camera);
	Transform3D<> urTpg=urTcam*camTobj_rw*objTpg; //take initial rotation of object as pregrasping rotation
//	Transform3D<> urTpg=urTcam*camTobj_rw;

	// 7)URbase -> OBJECT transform
	Transform3D<> urTobj((urTmrk*mrkTobj*objTpg).P(),urTpg.R());
//	Transform3D<> urTobj((urTmrk*mrkTobj).P(),urTpg.R());

	//determine initial distance between the gripper-marker and the object
	Vector3D<> dist=(urTobj.P()-urTmrk.P());
	cout<<"Initial distance is: "<<dist.norm2()<<endl;
	cout<<endl;
	//set the admissible threshold - distance between gripper-marker and object
	double tresh=0.01;

	//store the current value
	kinematic_camTmrk=camTmrk;
	kinematic_camTobj=camTobj;

	//start servoing method
	while (!cd.inCollision(state)){
		//compute current distance between gripper-marker and object
		Vector3D<> cur_dist=(urTobj.P()-urTmrk.P());
		cout<<"Current distance: "<<cur_dist.norm2()<<"\n";
		//stop program if threshold limit is exceeded
		if(cur_dist.norm2()<tresh)
		{
			cout<<"Threshold limit exceeded!"<<endl;
			ros::shutdown();
			break;
		}

		//compute the new urTmrk transform by performing interpolation between urTmrk & urTobj
		Transform3D<> urTmrk_new=computeinterpolation(urTmrk,urTobj,deltatess);

		//compute the new UR5 configuration which moves the gripper-marker closer to the object
		T1=clock();
		Q new_pos=computenewQ(pos,urTmrk,urTmrk_new,UR,state,marker,bottle); //new configuration
		//Bring robot in new configuration
		UR->setQ(new_pos,state); //set device
		cout<<"new_pos:"<<new_pos;
		cout<<endl;

		//store the new urTmrk transform
		urTmrk=urTmrk_new;
		//store the new UR5 arm configuration
		pos=new_pos;

		//recompute the intermediate transforms
	    // 1)CAMERA -> OBJECT transform
		Transform3D<> camTobj=find_camTobj(UR,state,bottle,camera,kinematic_camTobj);

	    // 2)CAMERA -> MARKER transform
		Transform3D<> camTmrk=find_camTmrk(UR,state,marker,camera,kinematic_camTmrk);

	    // 3)MARKER -> OBJECT transform
		Transform3D<> mrkTobj=inverse(camTmrk)*camTobj;

		// 4)URbase -> OBJECT transform
//		Transform3D<> urTobj((urTmrk*mrkTobj*objTpg).P(),urTpg.R()); //use the pregrasping rotation
		Transform3D<> urTobj((urTmrk*mrkTobj).P(),urTpg.R());
		//store kinematic values for camTobj & camTmrk
		kinematic_camTobj=camTobj;
		kinematic_camTmrk=camTmrk;

		//execute robot movement
		//open file to write the script
		std::ofstream myfile;
		cout << "Generating script ... ";
		myfile.open ("/home/sdsuvei/workspace/PythonScripts/armQ.py");
		if (myfile.is_open()){
			myfile << 	"#!/usr/bin/python \n \n" <<
						"import roslib \n" <<
						"roslib.load_manifest('cob_script_server') \n" <<
						"import rospy \n \n" <<
						"from simple_script_server import * \n" <<
						"sss = simple_script_server() \n \n" <<
						"if __name__ == \"__main__\": \n" <<
						"	rospy.init_node(\"asd\") \n" <<
						"	sss.move(\"arm\", [["<<pos[0]<<","<<pos[1]<<","<<pos[2]<<","<<pos[3]<<","<<pos[4]<<","<<pos[5]<<"]]) \n";
			myfile.close();
			cout << "Done!" << endl;
		}
		else{
			cout << "myfile not open!" << endl;
		}

		int val1;
		cout<<"\n Press key to allow movement: ";
		cin>>val1;
		cout<<"\n";

		cout << "Executing movement..." << endl;
		system("python /home/sdsuvei/workspace/PythonScripts/armQ.py");


		cout << "Movement done!" << endl;

		int val;
		cout<<"\n Press any key to continue ";
		cin>>val;
		cout<<"\n";
	}
	itHappened=false;
	}
	ros::spinOnce();
	loop_rate.sleep();
	}
	cout<<"After While";
			cout<<endl;

	if(flag)
		{
			cout<<"\nOrientation impossible! Try a different grasping approach!\n"<< endl;
		}
	else
		{
			cout<<"\nDone!\n\n";
		}
	return 0;
}

//compute camTmrk transform
Transform3D<> find_camTmrk(Device::Ptr device, State state,Frame* marker,Frame* camera,Transform3D<> previous){
	tf::TransformListener listen_cammk;
	tf::StampedTransform cammk_transform[5];
	Transform3D<> camTmrk;
	bool cammrk=true;
	int i=0;
	try{
		  listen_cammk.waitForTransform("/head_cam3d_link", "/gripper",ros::Time(0),ros::Duration(5.0));
		  for(i=0;i<5;i++)
		  {
			  listen_cammk.lookupTransform("/head_cam3d_link", "/gripper",ros::Time(0), cammk_transform[i]);
		  }

	    }
	    catch (tf::TransformException ex){
		      ROS_ERROR("%s",ex.what());
		      cout<<"Using previous value for camTmrk.\n";
		      cammrk=false;
	    }
	    if(cammrk)
	    {
	    	float vec_X=0;
	    	float vec_Y=0;
	    	float vec_Z=0;
	    	float rot_X=0;
	    	float rot_Y=0;
	    	float rot_Z=0;
	    	float rot_W=0;
	    	for(i=0;i<5;i++)
		    {
	    		vec_X+=cammk_transform[i].getOrigin().getX();
	    		vec_Y+=cammk_transform[i].getOrigin().getY();
	    		vec_Z+=cammk_transform[i].getOrigin().getZ();
	    		rot_X+=cammk_transform[i].getRotation().getX();
	    		rot_Y+=cammk_transform[i].getRotation().getY();
	    		rot_Z+=cammk_transform[i].getRotation().getZ();
	    		rot_W+=cammk_transform[i].getRotation().getW();
		    }
	    	//compute the positional component of the gripper-marker
	    	Vector3D<> cammk_vec(vec_X/5,vec_Y/5,vec_Z/5);
	    	//read the quaternion values of the gripper-marker
	    	Quaternion<>cammk_q (rot_X/5,rot_Y/5,rot_Z/5,rot_W/5);
	    	//compute the rotational component of the gripper-marker
	    	Rotation3D<> cammk_rot(cammk_q.toRotation3D());
	    	//compute the camTmrk transform for the current gripper-marker placement
	    	Transform3D<> cammk(cammk_vec,cammk_rot);
	    	camTmrk=cammk;
	    }
		    else
		    {
		    	camTmrk=previous;
		    }
	return camTmrk;
}

//compute urTcam transform
Transform3D<> find_urTcam(Device::Ptr device, State state,Frame* camera){
	tf::TransformListener listen_ucam;
	tf::StampedTransform ucam_transform[5];
	Transform3D<> urTcam;
	bool ucam=true;
	int i=0;
	try{
		  listen_ucam.waitForTransform("/arm_base_link", "/head_cam3d_link",ros::Time(0),ros::Duration(5.0));
		  for(i=0;i<5;i++)
		  {
			  listen_ucam.lookupTransform("/arm_base_link", "/head_cam3d_link",ros::Time(0), ucam_transform[i]);
		  }

	    }
	    catch (tf::TransformException ex){
		      ROS_ERROR("%s",ex.what());
		      cout<<"Using RW values for urTcam.\n";
		      ucam=false;
	    }
	    if(ucam)
	    {
	    	float vec_X=0;
	    	float vec_Y=0;
	    	float vec_Z=0;
	    	float rot_X=0;
	    	float rot_Y=0;
	    	float rot_Z=0;
	    	float rot_W=0;
	    	for(i=0;i<5;i++)
	    	{
	    		vec_X+=ucam_transform[i].getOrigin().getX();
	    		vec_Y+=ucam_transform[i].getOrigin().getY();
	    		vec_Z+=ucam_transform[i].getOrigin().getZ();
	    		rot_X+=ucam_transform[i].getRotation().getX();
	    		rot_Y+=ucam_transform[i].getRotation().getY();
	    		rot_Z+=ucam_transform[i].getRotation().getZ();
	    		rot_W+=ucam_transform[i].getRotation().getW();
	    	}

	    	Vector3D<> ucam_vec(vec_X/5,vec_Y/5,vec_Z/5);
	    	Quaternion<>ucam_q (rot_X/5,rot_Y/5,rot_Z/5,rot_W/5);
	    	Rotation3D<> ucam_rot(ucam_q.toRotation3D());
	    	Transform3D<> urcam(ucam_vec,ucam_rot);
	    	urTcam=urcam;
	    }
	    else
	    {
	    	urTcam=device->baseTframe(camera,state);
	    }
	 return urTcam;
}

//compute camTobj_rw transform
Transform3D<> rw_camTobj(Device::Ptr UR, State state,Frame* bottle,Frame* camera){
 	tf::TransformListener listen_obj;
 	tf::StampedTransform obj_transform[5];
 	Transform3D<> camTobj_rw;
 	bool camo=true;
 	int i=0;
 	try{
 		listen_obj.waitForTransform("/head_cam3d_link", "/object",ros::Time(0),ros::Duration(5.0));
 		for(i=0;i<5;i++)
 		{
 			 listen_obj.lookupTransform("/head_cam3d_link", "/object",ros::Time(0), obj_transform[i]);
 		}
 		    }
 		    catch (tf::TransformException ex){
 		      ROS_ERROR("%s",ex.what());
 		     cout<<"Object not visible!\n";
 		      camo=false;
 		    }
 		    if(camo){
 		    	float vec_X=0;
 		    	float vec_Y=0;
 		    	float vec_Z=0;
 		    	float rot_X=0;
 		    	float rot_Y=0;
 		    	float rot_Z=0;
 		    	float rot_W=0;
 		    	for(i=0;i<5;i++)
 		    	{
 		    		vec_X+=obj_transform[i].getOrigin().getX();
 		    		vec_Y+=obj_transform[i].getOrigin().getY();
 		    		vec_Z+=obj_transform[i].getOrigin().getZ();
 		    		rot_X+=obj_transform[i].getRotation().getX();
 		    		rot_Y+=obj_transform[i].getRotation().getY();
 		    		rot_Z+=obj_transform[i].getRotation().getZ();
 		    		rot_W+=obj_transform[i].getRotation().getW();
 		    	}
 		    	Vector3D<> obj_vec(vec_X/5,vec_Y/5,vec_Z/5);
 		    	Quaternion<>obj_q (rot_X/5,rot_Y/5,rot_Z/5,rot_W/5);
 		    	Rotation3D<> obj_rot(obj_q.toRotation3D());
 		    	Transform3D<> cambot(obj_vec,obj_rot);
 		    	camTobj_rw=cambot;
 		    }
 		    else{
 		    	exit(0);
 		    }
 		 	return camTobj_rw;
}

//compute camTobj transform
Transform3D<> find_camTobj(Device::Ptr device, State state,Frame* bottle,Frame* camera,Transform3D<> previous){
 	tf::TransformListener listen_obj;
 	tf::StampedTransform obj_transform[5];
 	Transform3D<> camTbot;
 	bool camo=true;
 	int i=0;
 	try{
 		listen_obj.waitForTransform("/head_cam3d_link", "/object",ros::Time(0),ros::Duration(5.0));
 		for(i=0;i<5;i++)
 		{
 			 listen_obj.lookupTransform("/head_cam3d_link", "/object",ros::Time(0), obj_transform[i]);
 		}
 		    }
 		    catch (tf::TransformException ex){
 		      ROS_ERROR("%s",ex.what());
 		     cout<<"Using RW values for camTobj.\n";
 		      camo=false;
 		    }
 		    if(camo){
 		    	float vec_X=0;
 		    	float vec_Y=0;
 		    	float vec_Z=0;
 		    	float rot_X=0;
 		    	float rot_Y=0;
 		    	float rot_Z=0;
 		    	float rot_W=0;
 		    	for(i=0;i<5;i++)
 		    	{
 		    		vec_X+=obj_transform[i].getOrigin().getX();
 		    		vec_Y+=obj_transform[i].getOrigin().getY();
 		    		vec_Z+=obj_transform[i].getOrigin().getZ();
 		    		rot_X+=obj_transform[i].getRotation().getX();
 		    		rot_Y+=obj_transform[i].getRotation().getY();
 		    		rot_Z+=obj_transform[i].getRotation().getZ();
 		    		rot_W+=obj_transform[i].getRotation().getW();
 		    	}
 		    	Vector3D<> obj_vec(vec_X/5,vec_Y/5,vec_Z/5);
 		    	Quaternion<>obj_q (rot_X/5,rot_Y/5,rot_Z/5,rot_W/5);
 		    	Rotation3D<> obj_rot(obj_q.toRotation3D());
 		    	Transform3D<> cambot(obj_vec,obj_rot);
 		    	camTbot=cambot;
 		    }
 		    else{
// 		    	camTbot=Kinematics::frameTframe(camera,bottle,state);
 		    	camTbot=previous;
 		    }
 	return camTbot;
}

//computes new configuration for urTobj
Q computenewQ(Q qin, Transform3D<> Tin, Transform3D<> Tdes, Device::Ptr device, State state,Frame* marker,Frame* bottleframe){
	//set device state
	//Q home(6, 1.6765, -2.0368, 1.5597, -1.6505, -1.6474, -1.6449);
	device->setQ(qin,state);
	//compute the small iteration, show error
	VelocityScrew6D<> deltau=computeG(Tin,Tdes);
	//do while error larger than threshold
	while (deltau.norm2()>epsilon && flag==0){
		//get Jacobian
		Jacobian J=device->baseJframe(marker,state);
		//calcultate dq
//		Q dq (LinearAlgebra::inverse(J.e())*deltau.e());
		Q dq (J.e().inverse()*deltau.e());
		//add dq to q
//		Q dq(6,1,2,3,4,5,6);
		qin=qin+dq;
		//qin=0;
		//set new q
		device->setQ(qin,state);
		//get new transform
		Transform3D<> bThan = device->baseTframe(marker,state);
		Tin=bThan;
		//compute and show new deltau, error
		deltau=computeG(Tin,Tdes);
		T2=clock();
		Ttotal=T2-T1;
		sec += (Ttotal/CLOCKS_PER_SEC);
		if(sec>30)
		{
			flag=1;
		}
	}
	//return final Q
	return(qin);
}

//compute the small iteration
VelocityScrew6D<> computeG(Transform3D<> Tin, Transform3D<> Tdes){
	//compute positional difference
	Vector3D<> dif=Tdes.P()-Tin.P();
	//compute rotation difference
	Rotation3D<> rot=Tdes.R()*inverse(Tin.R());
	//assign values to deltau
	VelocityScrew6D<> deltau(dif(0),dif(1),dif(2),(rot(2,1)-rot(1,2))/2,(rot(0,2)-rot(2,0))/2,(rot(1,0)-rot(0,1))/2);
	return(deltau);
}

//compute interpolation
Transform3D<> computeinterpolation(Transform3D<> Tin, Transform3D<> Tdes,double delta)
{
	//number of steps, considering ti=1, ti-1=0, each step is 1/n
	int n=0;
	double WS=1.0;
	Transform3D<> Tout;

	//compute positional difference
	Vector3D<> dif=Tdes.P()-Tin.P();
	//Compute Wtotal
	Rotation3D<> forW =inverse(Tin.R())*Tdes.R();
	//Compute W vector
	Vector3D<> W=getW(forW);
	//Get the number of steps necessary
	Vector3D<> trans, rot;
//	trans=(1.0/n)*dif;
//	rot=(1.0/n)*W;
	double step;
	 do
	    {n=n+1;
	     trans=(1.0/n)*dif;
	     rot=(1.0/n)*W;
	     step=trans.norm2()/WS+rot.norm2()/pi;
	    }
	    while (step>delta);
	n=1; //number of steps
	EAA<> eaa(rot*n);
	Transform3D<> current(Tin.P()+trans*n,Tin.R()*eaa.toRotation3D());
	Tout=current;
	return Tout;
}

Vector3D<> getW(Rotation3D<> r)
{   //see which formula should be used
    Vector3D<> to_norm(r(2,1)-r(1,2), r(0,2)-r(2,0), r(1,0)-r(0,1));
    double norm =to_norm.norm2();

	//The result
	Vector3D<> theresult;

	//if theta close to 0
	if (norm/2<1/1000000)
	{
		theresult(0) = 0.5*(r(2,1) - r(1,2));
        theresult(1) = 0.5*(r(0,2) - r(2,0));
        theresult(2) = 0.5*(r(1,0) - r(0,1));
	}
	else if ((pi-norm/2)<1/1000000)
		{
			theresult(0) = (double)sgn(r(2,1) - r(1,2))* sqrt(1 + r(0, 0));
		    theresult(1) = (double)sgn(r(0,2) - r(2,0))* sqrt(1 + r(1, 1));
			theresult(2) = (double)sgn(r(1,0) - r(0,1))* sqrt(1 + r(2, 2));
			theresult=((2.0*pi-norm)/(2.0*sqrt(3.0+r(0,0)+r(1,1)+r(2,2))))*theresult;
		}
		else
		{
			theresult(0) = (r(2,1) - r(1,2));
			theresult(1) = (r(0,2) - r(2,0));
			theresult(2) = (r(1,0) - r(0,1));
			theresult=((acos((r(0,0)+r(1,1)+r(2,2)-1)/2))/norm)*theresult;
		}
	return(theresult);
}


