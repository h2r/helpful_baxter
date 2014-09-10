// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>

// Baxter Utilities
#include <baxter_control/baxter_utilities.h>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_listener.h>
#include "move_msgs/BaxterAction.h"
#include "object_recognition_msgs/RecognizedObjectArray.h"
#include "baxter_grasps_server/GraspService.h"
#include "ros/time.h"
 
#include "ros/console.h"

#include <iostream>

#include <map>
#include <vector>
#include <string>
#include <utility> // header for pair
#include <math.h>



class ActionServer
{
public:
	baxter_control::BaxterUtilities baxterUtil;

	tf::TransformListener transformer;
	ros::Publisher markersPub;
	ros::Subscriber objectsSub;
	ros::Subscriber actionSub;
	ros::ServiceClient graspClient;

	typedef std::map<std::string, geometry_msgs::PoseStamped> PoseMap;
	PoseMap objectPoses;

	bool isPicking = false;
	bool isPlacing = false;

	move_group_interface::MoveGroup *moveGroup;

	ActionServer()
	{
		ros::NodeHandle nh;
		moveGroup = new move_group_interface::MoveGroup("left_arm");
		markersPub = nh.advertise<visualization_msgs::Marker>("/grasp_markers", 1000);
		graspClient = nh.serviceClient<baxter_grasps_server::GraspService>("/grasp_service");

	    moveGroup->setPlanningTime(5.0);
	    moveGroup->setSupportSurfaceName("table");
	    moveGroup->setWorkspace(0.0, -0.2, -0.30, 0.9, 1.0, 2.0);

	    actionSub = nh.subscribe("/baxter_action", 1000, &ActionServer::actionCB, this);
	    objectsSub = nh.subscribe("/ar_objects", 1000, &ActionServer::markersCB, this);		
	}

	bool isPickingOrPlacing() 
	{
		return isPicking || isPlacing;
	}

	void moveToNeutral()
	{
		moveGroup->setNamedTarget("left_neutral");
		moveGroup->move();
	}

	geometry_msgs::Point getRegion(std::string name)
	{
		geometry_msgs::Point point;
		if (name.compare("sink") == 0)
		{
			point.x = 0.7;
			point.y = 0.4;
			point.z = 0.5;
		}
		else if (name.compare("counter") == 0)
		{
			point.x = 0.6;
			point.y = -0.25;
			point.z = 0.3;
		}
		else if (name.compare("robot_counter") == 0)
		{
			point.x = 0.6;
			point.y = 0.0;
			point.z = 0.0;
		}
		return point;
	}

	geometry_msgs::PoseStamped getPoseStampedFromPoseWithCovariance(geometry_msgs::PoseWithCovarianceStamped pose){
			geometry_msgs::PoseStamped poseStamped;
			poseStamped.header = pose.header;
			poseStamped.pose.position = pose.pose.pose.position;
			poseStamped.pose.orientation = pose.pose.pose.orientation;
			return poseStamped;
	}

	geometry_msgs::Vector3Stamped getDirectionFromPose(geometry_msgs::PoseStamped graspPose, geometry_msgs::Vector3Stamped direction)
	{
		tf::Transform graspPoseTransform = getTransformFromPose(graspPose.pose);
		tf::Vector3 tfVector;
		tf::vector3MsgToTF(direction.vector, tfVector);
		tfVector = graspPoseTransform * tfVector;

		geometry_msgs::Vector3Stamped newDirection;
		newDirection.header = direction.header;
		tf::vector3TFToMsg(tfVector, newDirection.vector);
		return newDirection;
	}
	tf::Transform getTransformFromPose(geometry_msgs::Pose pose)
	{
		tf::Pose tfPose;
		tf::poseMsgToTF(pose, tfPose);
		tf::Transform transform(tfPose.getRotation(), tfPose.getOrigin());
		return transform;
	}

	geometry_msgs::PoseStamped getPoseFromTransform(tf::Transform transform, std_msgs::Header header)
	{

		geometry_msgs::PoseStamped pose;
		pose.header = header;

		tf::Pose tfPose(transform.getRotation(), transform.getOrigin());
		tf::poseTFToMsg(tfPose, pose.pose);

		return pose;
	}

	geometry_msgs::PoseStamped getGraspPoseRelativeToStampedPose(geometry_msgs::PoseStamped graspPose, geometry_msgs::PoseStamped pose)
	{
			std::string *error;
			transformer.getLatestCommonTime("world", "camera_link", pose.header.stamp, error);
			
			if (pose.header.frame_id.compare("world") != 0)
			{
				geometry_msgs::PoseStamped newPose;
				transformer.transformPose("world", pose, newPose);
				pose = newPose;
			}

			tf::Transform graspPoseTransform = getTransformFromPose(graspPose.pose);
			tf::Transform poseTransform = getTransformFromPose(pose.pose);
			tf::Transform totalTransform = poseTransform * graspPoseTransform;

			return getPoseFromTransform(totalTransform, pose.header);
	}

	std::vector<moveit_msgs::Grasp> setGraspsAtPose(geometry_msgs::PoseStamped pose,std::vector<moveit_msgs::Grasp> grasps)
	{
		ros::Time when;
		std::string *error;
		transformer.getLatestCommonTime("world", "camera_link", when, error);
		std::vector<moveit_msgs::Grasp> newGrasps;
		for (int i = 0; i < grasps.size(); i++)
		{
			moveit_msgs::Grasp grasp = grasps[i];
			grasp.id = i;
			grasp.pre_grasp_posture.header.stamp = when;
			grasp.grasp_posture.header.stamp = when;

			grasp.grasp_pose = getGraspPoseRelativeToStampedPose(grasp.grasp_pose, pose);
			grasp.pre_grasp_approach.direction = getDirectionFromPose(grasp.grasp_pose, grasp.pre_grasp_approach.direction);
			grasp.grasp_quality = 1.0;
			newGrasps.push_back(grasp);
		}
		return newGrasps;
	}

	bool pick(std::string objectName, geometry_msgs::PoseStamped objectPose)
	{
			moveGroup->detachObject();			

			baxter_grasps_server::GraspService graspRequest;
			graspRequest.request.name = objectName;
			
			if (!graspClient.call(graspRequest) || !graspRequest.response.success)
			{
				ROS_ERROR_STREAM("No grasps were found for the object " << objectName);
				return false;
			}


			moveGroup->setPlanningTime(20.0);
			moveGroup->setStartStateToCurrentState();

			std::vector<moveit_msgs::Grasp> grasps = setGraspsAtPose(objectPose, graspRequest.response.grasps);
			publishMarkers(grasps, objectName);
			
			bool result = moveGroup->pick(objectName, grasps);
			return result;
	}

	bool place(std::string objectName, geometry_msgs::PoseStamped originalPose, geometry_msgs::PoseStamped placePose)
	{
			geometry_msgs::PoseStamped goalPose = originalPose;
			goalPose.pose.position.x = placePose.pose.position.x;
			goalPose.pose.position.y = placePose.pose.position.y;
			goalPose.pose.position.z = placePose.pose.position.z;
			
			bool result = moveGroup->place(objectName, goalPose);
			return result;
	}

	std::vector<geometry_msgs::PoseStamped> getValidPlacePoses(geometry_msgs::Point placePoint)
	{
		std::vector<geometry_msgs::PoseStamped> placePoses;
		placePoses.resize(36);
		for (int i = 0; i < 36; i++)
		{	
			placePoses[i].header.frame_id = "world";
			placePoses[i].pose.position = placePoint;
			placePoses[i].pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI/2.0, i * 2.0 * M_PI / 36.0);
		}	

		return placePoses;
	}


	void pickAndPlace(std::string objectName, geometry_msgs::PoseStamped objectPose, geometry_msgs::Point placePoint)
	{
		ROS_INFO_STREAM("Attempting to pick up object " << objectName);
		
		isPicking = true;
		
		ROS_INFO_STREAM("Finding a valid place pose");
		std::vector<geometry_msgs::PoseStamped> placePoses = getValidPlacePoses(placePoint);

		ROS_INFO_STREAM("Attempting to pick up object " + objectName);
		moveToNeutral();

		bool pickSuccess = false;
		try
		{	
			pickSuccess = pick(objectName, objectPose);
			ROS_INFO_STREAM("Pick success: " << pickSuccess);
			isPlacing = pickSuccess;
			isPicking = false;
		}
		catch (...)
		{
			isPicking = false;
			isPlacing = false;
			return;

		}

		if (!pickSuccess)
		{
			ROS_ERROR_STREAM("Object pick up failed");
			isPicking = false;
			isPlacing = false;
			return;
		}

		
		bool place_result = false;
		try
		{
			for (int i = 0; i < placePoses.size(); i++)
			{
				ROS_INFO("Attempting to place object");
				if (place(objectName, objectPose, placePoses[i]))
				{
					isPlacing = false;
					break;
				}
			}
		}
		catch (...)
		{

		}
		isPlacing = false;
		isPicking = false;
	}

	void actionCB(move_msgs::BaxterActionConstPtr &msg)
	{
		if (msg->type == move_msgs::BaxterAction::MOVE)
		{
			if (!isPickingOrPlacing())
			{
				std::string objectName = msg->move_action.object.name;
				PoseMap::iterator findIt = objectPoses.find(objectName);
				if (findIt != objectPoses.end())
				{
					geometry_msgs::Point point = getRegion(msg->move_action.region.name);
					geometry_msgs::PoseStamped pose = findIt->second;
					pickAndPlace(objectName, pose, point);
				}
			}
		}
	}

	void markersCB(object_recognition_msgs::RecognizedObjectArrayConstPtr &msg)
	{
		objectPoses.clear();
		for (int i = 0; i < msg->objects.size(); i++)
		{

			geometry_msgs::PoseStamped pose = getPoseStampedFromPoseWithCovariance(msg->objects[i].pose);
			objectPoses[msg->objects[i].type.key] = pose;
		}
	}

	visualization_msgs::Marker createGraspMarker(moveit_msgs::Grasp grasp, std::string markerName, double lifetime = 15.0)
	{
		visualization_msgs::Marker marker;
		marker.type = 0;
		marker.header = grasp.grasp_pose.header;
		marker.header.stamp = ros::Time::now();
		// This may not be the correct pose
		marker.pose = grasp.grasp_pose.pose;
		marker.ns = markerName;
		marker.lifetime = ros::Duration(lifetime);
		marker.action = 0;
		marker.color.r = 1;
		marker.color.g = 1;
		marker.color.b = 1;
		marker.color.a = 1;
		marker.scale.x = 0.1;
		marker.scale.y = 0.03;
		marker.scale.z = 0.03;	

		return marker;
	}

	void publishMarkers(std::vector<moveit_msgs::Grasp> grasps, std::string object_name)
	{
		for (int i = 0; i < grasps.size(); i++)
		{
			visualization_msgs::Marker marker = createGraspMarker(grasps[i], object_name);
			markersPub.publish(marker);
		}
	}
};


int main(int argc, char**argv) 
{
	ros::init(argc, argv, "baxter_action_server");
	ActionServer server();
	ros::spin();
	return 0;	
}