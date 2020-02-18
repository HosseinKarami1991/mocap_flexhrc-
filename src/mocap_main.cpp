#include "mocap_optitrack/mocap_config.h"
#include "mocap_optitrack/mocap_datapackets.h"
#include "mocap_optitrack/skeletons.h"
#include "mocap_optitrack/socket.h"
#include "std_msgs/String.h"
// ROS includes
#include <geometry_msgs/Pose2D.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

// System includes
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <vector>
#include <fstream>
#include "mocap/GetPoseFromMocapMsg.h"
#include"mocap_msgs/mocapvector.h"
mocap::GetPoseFromMocapMsg objectCtrl;
const std::string MULTICAST_IP_KEY = "optitrack_config/multicast_address";
const std::string MULTICAST_IP_DEFAULT = "224.0.0.1";
const int MOCAP_DATA_FRAME = 7;
const int NumberOfObjects = 6;
const std::string MOCAP_MODEL_KEY = "mocap_model";
const std::string RIGID_BODIES_KEY = "rigid_bodies";
const char** DEFAULT_MOCAP_MODEL = SKELETON_WITHOUT_TOES;
mocap_msgs::mocapvector objvec;
const int LOCAL_PORT = 1511;
string s;
int global;
ros::Publisher objectCtrl1Pub;
int main(int argc, char  *argv[])
{ 
	ros::init(argc, argv, "mocap_flex");
    ros::NodeHandle n("~");
    double freq = 80;
    std::string objectCtrl1Topic = "/object_ctrl_1";
    std::string objectCtrl2Topic = "/object_ctrl_2";
    std::string objectCtrl3Topic = "/pos_obj";
 //   for(int i=0;i<NumberOfObjects;i++){
  //      std::stringstream ss;
   //      ss << i+1;
    //    std::string pubtopic = objectCtrl3Topic + "_" + ss.str();
//
 //       mypubvec[i] = n.advertise<mocap::GetPoseFromMocapMsg>(pubtopic, freq);
  //  }
    objectCtrl1Pub = n.advertise<mocap_msgs::mocapvector>(objectCtrl1Topic, freq);
    ros::Publisher objectCtrl2Pub = n.advertise<mocap::GetPoseFromMocapMsg>(objectCtrl2Topic, freq);
     const char** mocap_model(DEFAULT_MOCAP_MODEL);
    ros::Rate loopRate(freq);

    if (n.hasParam(MOCAP_MODEL_KEY)) {
        std::string tmp;
        if (n.getParam(MOCAP_MODEL_KEY, tmp)) {
            if (tmp == "SKELETON_WITH_TOES")
                mocap_model = SKELETON_WITH_TOES;
            else if (tmp == "SKELETON_WITHOUT_TOES")
                mocap_model = SKELETON_WITHOUT_TOES;
            else if (tmp == "OBJECT")
                mocap_model = OBJECT;
        }
    }
  s= "salam";
    // Get configuration from ROS parameter server
    std::string multicast_ip(MULTICAST_IP_DEFAULT);
    if (n.hasParam(MULTICAST_IP_KEY)) {
        n.getParam(MULTICAST_IP_KEY, multicast_ip);
    } else {
        ROS_WARN_STREAM(
            "Could not get Multicast address, using default: " << multicast_ip);
    }
        //objectCtrl.resize(NumberOfObjects);

    RigidBodyMap published_rigid_bodies;
    global=0;
    if (n.hasParam(RIGID_BODIES_KEY)) {
        XmlRpc::XmlRpcValue body_list;
        n.getParam("rigid_bodies", body_list);
        if (body_list.getType() == XmlRpc::XmlRpcValue::TypeStruct
            && body_list.size() > 0) {
            XmlRpc::XmlRpcValue::iterator i;
            for (i = body_list.begin(); i != body_list.end(); ++i) {
                if (i->second.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                    PublishedRigidBody body(i->second);
                    string id = (string&)(i->first);
                    RigidBodyItem item(atoi(id.c_str()), body);

                    std::pair<RigidBodyMap::iterator, bool> result = published_rigid_bodies.insert(item);
                    if (!result.second) {
                        ROS_ERROR(
                            "Could not insert configuration for rigid body ID %s",
                            id.c_str());
                    }
                }
            }
        }
    }

    UdpMulticastSocket multicast_client_socket(LOCAL_PORT, multicast_ip);
     cout<<"ip is "<<multicast_ip<<endl;

   // std::vector<boost::array<int>> objs;
    //double zeros[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    //objs.resize(NumberOfObjects);
   // double vehiclepose1[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  //  double vehiclepose2[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    ushort payload;

std_msgs::String mm;
while(ros::ok()){

     
    bool packetread = false;
    int numBytes = 0;
    bool gotOnePacket = false;
    const char* buffer;
 //   do {

            // Receive data from mocap device
            numBytes = multicast_client_socket.recv();
            //cout<<"number of bytes outside is "<<numBytes<<endl;
            // Parse mocap data
            if (numBytes > 0) {
                //cout<<"number of bytes"<<numBytes<<endl;
                buffer = multicast_client_socket.getBuffer();
                //const char* buffer = multicast_client_socket.getBuffer();
                unsigned short header = *((unsigned short*)(&buffer[0]));

                // Look for the beginning of a NatNet package
                if (header == MOCAP_DATA_FRAME) {
                    payload = *((ushort*)&buffer[2]);
                    MoCapDataFormat format(buffer, payload);
                    format.parse();
                    packetread = true;
                    //numberOfPackets++;
                    //cout<<"format.model.numRigidBodies"<<format.model.numRigidBodies<<endl;
                    if (format.model.numRigidBodies > 0) {
                        std::stringstream logMsg;
                   
                 
                          //objvec.objects.resize(format.model.numRigidBodies);
                        for (int i = 0; i < format.model.numRigidBodies; i++) {
 
                            int ID = format.model.rigidBodies[i].ID;
                            RigidBodyMap::iterator item = published_rigid_bodies.find(ID);

                
                                   
                                    tf::Quaternion q(
                                    format.model.rigidBodies[i].pose.orientation.x,
                                    -format.model.rigidBodies[i].pose.orientation.z,
                                    format.model.rigidBodies[i].pose.orientation.y,
                                    format.model.rigidBodies[i].pose.orientation.w);

                                     geometry_msgs::PoseStamped ros_pose;
                                     ros_pose.header.stamp = ros::Time::now();
                                     tf::Matrix3x3 m(q);
                                      double roll, pitch, yaw;
                                     m.getRPY(roll, pitch, yaw);
                      
                                    objectCtrl.index=i;
                                    objectCtrl.mocap.yaw = yaw;
                                    objectCtrl.mocap.pitch = pitch;
                                    objectCtrl.mocap.roll = roll;
                                    objectCtrl.mocap.x = format.model.rigidBodies[i].pose.position.x;
                                    objectCtrl.mocap.y = -format.model.rigidBodies[i].pose.position.z;
                                    objectCtrl.mocap.z = format.model.rigidBodies[i].pose.position.y;
                                   // objvec.objects[i] = objectCtrl;
                                    objvec.objects.push_back(objectCtrl);

                                   // cout<<"Size of vector"<<objvec.objects.size()<<endl;
          
                        }


                    }

                }

             //objectCtrl1Pub.publish(objvec);
            }
objectCtrl1Pub.publish(objvec);
objvec.objects.clear();
//        }
	                           

//while (numBytes > 0);

            ros::spinOnce();
            loopRate.sleep();}
           
                
                //
                
return 0;
}

