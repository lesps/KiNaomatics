#include <stdio.h>
#include <math.h>
#include <XnCppWrapper.h>

#define PI 3.14159265

/**
*Given a reference point and two additional points, find angle at
*the reference point
*@param XnSkeletonJointTransformation refJoint The joint at the vertex of the angle
*@param XnSkeletonJointTransformation joint1 One of the connected joints to refJoint
*@param XnSkeletonJointTransformation joint2 Another joint connected to refJoint
*@param int plane Which coordinate plane should be used (0: x-y, otherwise: y-z) 
*@return The angle at offset point due to points one and two.
**/

float findAngle(XnSkeletonJointTransformation refJoint,
                XnSkeletonJointTransformation joint1,
                XnSkeletonJointTransformation joint2,
                int plane) {
  float pX, pY, pZ, qX, qY, qZ, offsetX, offsetY, offsetZ, result; 
  
  //Get coordinates of the point of reference
  offsetX = refJoint.position.position.X;
  offsetY = refJoint.position.position.Y;
  offsetZ = refJoint.position.position.Z;
  
  //Transform the origin of the coordinate axis
  pX = joint1.position.position.X - offsetX;
  pY = joint1.position.position.Y - offsetY;
  pZ = joint1.position.position.Z - offsetZ;
  qX = joint2.position.position.X - offsetX;
  qY = joint2.position.position.Y - offsetY;
  qZ = joint2.position.position.Z - offsetZ;
  
  //Calculate the angle in the given plane
  if(plane==0)
    result = (atan2(qY,qX) - atan2(pY,pX)) * 180/PI;
  else
    result = (atan2(qZ,qY) - atan2(pZ,pY)) * 180/PI;

  //Adjust for answers >pi or <-pi
  if(result < -180)
    result += 360;
  
  return result;
}
