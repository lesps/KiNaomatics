#include <XnCppWrapper.h>
#include <string>
#include "string.h"
#include <math.h>
#include <iostream>
#include <deque>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <assert.h>
#include <vector>
#include <stdint.h>
#include <sstream>
#include <unistd.h>

#define MDELAY 2
#define TTL 16
#define MAX_LENGTH 160000 //Size for sending 640*480 yuyv data without resampling


//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define SAMPLE_XML_PATH "../../../../Data/SamplesConfig.xml"
#define SAMPLE_XML_PATH_LOCAL "SamplesConfig.xml"
#define JOINT_ARR_SIZE 15
#define MAX_NUM_USERS 1
#define PI 3.14159265
#define MDELAY 2
#define TTL 16
#define MAX_LENGTH 160000 //Size for sending 640*480 yuyv data without resampling

//Parameters for arm positions
#define DX 600
#define DY 600
#define DZ 550
#define TX 200
#define TY 200
#define TZ 200

//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------

using namespace std;

xn::Context g_Context;
xn::ScriptNode g_scriptNode;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator g_UserGenerator;

XnBool g_bNeedPose = FALSE;
XnChar g_strPose[20] = "";

XnSkeletonJointTransformation jointArr[JOINT_ARR_SIZE];

const int maxQueueSize = 12;
static string IP;
static int PORT = 0;

static deque<string> recvQueue;
static int send_fd, recv_fd;

static int commInit(string ip, int port) {
	IP = ip;
	PORT = port;
 	return 1;
}

static int commUpdate() {
  static sockaddr_in source_addr;
  static char data[MAX_LENGTH];

	// Check whether initiated
  assert(IP.empty()!=1);	

	// Check port
	assert(PORT!=0);

  static bool init = false;
  if (!init) {
    struct hostent *hostptr = gethostbyname(IP.c_str());
    if (hostptr == NULL) {
      printf("Could not get hostname\n");
      return -1;
    }

    send_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (send_fd < 0) {
      printf("Could not open datagram send socket\n");
      return -1;
    }

    int i = 1;
    if (setsockopt(send_fd, SOL_SOCKET, SO_BROADCAST, (const char *) &i, sizeof(i)) < 0) {
      printf("Could not set broadcast option\n");
      return -1;
    }
      
    struct sockaddr_in dest_addr;
    bzero((char *) &dest_addr, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    bcopy(hostptr->h_addr, (char *) &dest_addr.sin_addr, hostptr->h_length);
    dest_addr.sin_port = htons(PORT);

    if (connect(send_fd, (struct sockaddr *) &dest_addr, sizeof(dest_addr)) < 0) {
      printf("Could not connect to destination address\n");
      return -1;
    }

    recv_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (recv_fd < 0) {
      printf("Could not open datagram recv socket\n");
      return -1;
    }

    struct sockaddr_in local_addr;
    bzero((char *) &local_addr, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    local_addr.sin_port = htons(PORT);
    if (bind(recv_fd, (struct sockaddr *) &local_addr, sizeof(local_addr)) < 0) {
      printf("Could not bind to port\n");
      return -1;
    }

    // Nonblocking receive:
    int flags  = fcntl(recv_fd, F_GETFL, 0);
    if (flags == -1) 
      flags = 0;
    if (fcntl(recv_fd, F_SETFL, flags | O_NONBLOCK) < 0) {
      printf("Could not set nonblocking mode\n");
      return -1;
    }

    // TODO: set at exit
    init = true;
  }

  // Process incoming messages:
  socklen_t source_addr_len = sizeof(source_addr);
  int len = recvfrom(recv_fd, data, MAX_LENGTH, 0, (struct sockaddr *) &source_addr, &source_addr_len);
  while (len > 0) {
    string msg((const char *) data, len);
    recvQueue.push_back(msg);

    len = recvfrom(recv_fd, data, MAX_LENGTH, 0, (struct sockaddr *) &source_addr, &source_addr_len);
  }

  // Remove older messages:
  while (recvQueue.size() > maxQueueSize) {
    recvQueue.pop_front();
  }

  return 1;
}

static int commSize() {
  return recvQueue.size();
}

static string commReceive() {
  commUpdate();

  //If empty, return 0 (check for this)
  if (recvQueue.empty()) {
    return 0;
  }

  recvQueue.pop_front();
  return recvQueue.front();
}


static int commSend(char* data) {
  commUpdate();
	string header;
  string dataStr;
	string contents(data);
  header.push_back(11);
	dataStr = header + contents;
  return send(send_fd, dataStr.c_str(), dataStr.size(), 0);
}

static int commSend(string data) {
  commUpdate();
	string header;
  string dataStr;
  header.push_back(11);
	dataStr = header + data;
  return send(send_fd, dataStr.c_str(), dataStr.size(), 0);
}

XnBool fileExists(const char *fn)
{
	XnBool exists;
	xnOSDoesFileExist(fn, &exists);
	return exists;
}

// Callback: New user was detected
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
    XnUInt32 epochTime = 0;
    xnOSGetEpochTime(&epochTime);
    printf("%d New User %d\n", epochTime, nId);
    // New user found
    if (g_bNeedPose)
    {
        g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
    }
    else
    {
        g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
    }
}
// Callback: An existing user was lost
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
    XnUInt32 epochTime = 0;
    xnOSGetEpochTime(&epochTime);
    printf("%d Lost user %d\n", epochTime, nId);	
    string s = "{[\"tracking\"]=false}";
    commSend(s);
}
// Callback: Detected a pose
void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie)
{
    XnUInt32 epochTime = 0;
    xnOSGetEpochTime(&epochTime);
    printf("%d Pose %s detected for user %d\n", epochTime, strPose, nId);
    g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
    g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}
// Callback: Started calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie)
{
    XnUInt32 epochTime = 0;
    xnOSGetEpochTime(&epochTime);
    printf("%d Calibration started for user %d\n", epochTime, nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationComplete(xn::SkeletonCapability& capability, XnUserID nId, XnCalibrationStatus eStatus, void* pCookie)
{
    XnUInt32 epochTime = 0;
    xnOSGetEpochTime(&epochTime);
    if (eStatus == XN_CALIBRATION_STATUS_OK)
    {
        // Calibration succeeded
        printf("%d Calibration complete, start tracking user %d\n", epochTime, nId);		
        g_UserGenerator.GetSkeletonCap().StartTracking(nId);
    }
    else
    {
        // Calibration failed
        printf("%d Calibration failed for user %d\n", epochTime, nId);
        if(eStatus==XN_CALIBRATION_STATUS_MANUAL_ABORT)
        {
            printf("Manual abort occured, stop attempting to calibrate!");
            return;
        }
        if (g_bNeedPose)
        {
            g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
        }
        else
        {
            g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
        }
    }
}

int getJoints(XnUserID user){

  //Locals
  XnSkeletonJointTransformation joint;

  g_UserGenerator.GetSkeletonCap().GetSkeletonJoint(user,XN_SKEL_TORSO,joint);
  if(joint.position.fConfidence > .8){
    jointArr[0] = joint;
  } else return 0;

  g_UserGenerator.GetSkeletonCap().GetSkeletonJoint(user,XN_SKEL_RIGHT_HAND,joint);
  if(joint.position.fConfidence > .8){
    jointArr[1] = joint;
  } else return 0;

  g_UserGenerator.GetSkeletonCap().GetSkeletonJoint(user,XN_SKEL_LEFT_HAND,joint);
  if(joint.position.fConfidence > .8){
    jointArr[2] = joint;
  } else return 0;

  //The function succeeded
  return 1;
}

#define CHECK_RC(nRetVal, what)					    \
    if (nRetVal != XN_STATUS_OK)				    \
{								    \
    printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));    \
    return nRetVal;						    \
}

int main(int argc, char **argv)
{
    commInit("96.69.123.255", 54321);
    XnStatus nRetVal = XN_STATUS_OK;
    xn::EnumerationErrors errors;

    const char *fn = NULL;
    if    (fileExists(SAMPLE_XML_PATH)) fn = SAMPLE_XML_PATH;
    else if (fileExists(SAMPLE_XML_PATH_LOCAL)) fn = SAMPLE_XML_PATH_LOCAL;
    else {
        printf("Could not find '%s' nor '%s'. Aborting.\n" , SAMPLE_XML_PATH, SAMPLE_XML_PATH_LOCAL);
        return XN_STATUS_ERROR;
    }
    printf("Reading config from: '%s'\n", fn);

    nRetVal = g_Context.InitFromXmlFile(fn, g_scriptNode, &errors);
    if (nRetVal == XN_STATUS_NO_NODE_PRESENT)
    {
        XnChar strError[1024];
        errors.ToString(strError, 1024);
        printf("%s\n", strError);
        return (nRetVal);
    }
    else if (nRetVal != XN_STATUS_OK)
    {
        printf("Open failed: %s\n", xnGetStatusString(nRetVal));
        return (nRetVal);
    }

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
    CHECK_RC(nRetVal,"No depth");

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
    if (nRetVal != XN_STATUS_OK)
    {
        nRetVal = g_UserGenerator.Create(g_Context);
        CHECK_RC(nRetVal, "Find user generator");
    }

    XnCallbackHandle hUserCallbacks, hCalibrationStart, hCalibrationComplete, hPoseDetected;
    if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
    {
        printf("Supplied user generator doesn't support skeleton\n");
        return 1;
    }
    nRetVal = g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
    CHECK_RC(nRetVal, "Register to user callbacks");
    nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationStart(UserCalibration_CalibrationStart, NULL, hCalibrationStart);
    CHECK_RC(nRetVal, "Register to calibration start");
    nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationComplete(UserCalibration_CalibrationComplete, NULL, hCalibrationComplete);
    CHECK_RC(nRetVal, "Register to calibration complete");

    if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration())
    {
        g_bNeedPose = TRUE;
        if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
        {
            printf("Pose required, but not supported\n");
            return 1;
        }
        nRetVal = g_UserGenerator.GetPoseDetectionCap().RegisterToPoseDetected(UserPose_PoseDetected, NULL, hPoseDetected);
        CHECK_RC(nRetVal, "Register to Pose Detected");
        g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
    }

    g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

    nRetVal = g_Context.StartGeneratingAll();
    CHECK_RC(nRetVal, "StartGenerating");

    XnUserID aUsers[MAX_NUM_USERS];
    XnUInt16 nUsers;
    int j;

    printf("Starting to run\n");
    if(g_bNeedPose)
    {
        printf("Assume calibration pose\n");
    }
    XnUInt32 epochTime = 0;
    bool firstRun = true;
    int rState,lState;
    while (!xnOSWasKeyboardHit())
    {
        g_Context.WaitOneUpdateAll(g_UserGenerator);
        nUsers=MAX_NUM_USERS;
        g_UserGenerator.GetUsers(aUsers, nUsers);
        int numTracked=0;
        int userToPrint=-1; 
        for(XnUInt16 i=0; i<nUsers; i++)
        {
            ostringstream ostr;
            if(g_UserGenerator.GetSkeletonCap().IsTracking(aUsers[i])==FALSE)
              continue;
            ostr<<"{[\"coaching\"]=true,";

            if(getJoints(aUsers[i])){
              if (firstRun) {
                rState = 0;
                lState = 0;
              }

              double xTorso = jointArr[0].position.position.X; 
              double yTorso = jointArr[0].position.position.Y;
              double zTorso = jointArr[0].position.position.Z;
              double xRHand = jointArr[1].position.position.X;
              double yRHand = jointArr[1].position.position.Y;
              double zRHand = jointArr[1].position.position.Z;
              double xLHand = jointArr[2].position.position.X;
              double yLHand = jointArr[2].position.position.Y;
              double zLHand = jointArr[2].position.position.Z;

              //------------------------------------------------------------------
              //Coaching State Machine                                       
              //
              //Robot 1 corresponds to right arm, Robot 2 corresponds to left arm
              //
              //State 0: Initial State
              //State 1: Transitional State
              //State 2: Offensive State
              //State 3: Defensive State
              //State 4: Stationary State
              //
              //Transitions only exist in the form Initial<->Transitional<->State
              //You must return to the Initial state to issue new coaching cues
              //------------------------------------------------------------------

              //State machine for right arm; Robot 1
              if (rState == 0) {
                if ((zTorso-zRHand)>TZ or (xRHand-xTorso)>TX or (yRHand-yTorso)>TY)
                  rState = 1;
              } 
              else if (rState == 1) {
                if ((zTorso-zRHand)<TZ and (xRHand-xTorso)<TX and (yRHand-yTorso)<TY) {
                  rState = 0;
                  printf("Robot 1 INITIAL!");
                }
                if ((zTorso-zRHand)>DZ) {
                  rState = 2;
                  printf("Robot 1 ATTACK!");
                }
                else if ((xRHand-xTorso)>DX) {
                  rState = 3;
                  printf("Robot 1 DEFEND!");
                }
                else if ((yRHand-yTorso)>DY) {
                  rState = 4;
                  printf("Robot 1 STOP!");
                }
                else
                  printf("Robot 1 Transitioning");
              }
              else if (rState == 2) {
                if ((zTorso-zRHand)<DZ)
                  rState = 1;                
              }
              else if (rState == 3) {
                if ((xRHand-xTorso)<DX)
                  rState = 1;
              }
              else if (rState == 4) {
                if ((yRHand-yTorso)<DY)
                  rState = 1;
              }
              else
                printf("The rArm state is %d. How is that possible?!", rState);

              //State machine for left arm; Robot 2
              if (lState == 0) {
                if ((zTorso-zLHand)>TZ or (xTorso-xLHand)>TX or (yLHand-yTorso)>TY)
                  lState = 1;
              } 
              else if (lState == 1) {
                if ((zTorso-zLHand)<TZ and (xTorso-xLHand)<TX and (yLHand-yTorso)<TY) {
                  lState = 0;
                  printf("Robot 1 INITIAL!");
                }
                if ((zTorso-zLHand)>DZ) {
                  lState = 2;
                  printf("Robot 1 ATTACK!");
                }
                else if ((xTorso-xLHand)>DX) {
                  lState = 3;
                  printf("Robot 1 DEFEND!");
                }
                else if ((yLHand-yTorso)>DY) {
                  lState = 4;
                  printf("Robot 1 STOP!");
                }
                else
                  printf("Robot 1 Transitioning");
              }
              else if (lState == 2) {
                if ((zTorso-zLHand)<DZ)
                  lState = 1;                
              }
              else if (lState == 3) {
                if ((xTorso-xLHand)<DX)
                  lState = 1;
              }
              else if (lState == 4) {
                if ((yLHand-yTorso)<DY)
                  lState = 1;
              }
              else
                printf("The lArm state is %d. How is that possible?!", lState);


              ostr << "[\"strat\"]=" << "{" << rState << "," << lState << "}";
              string send = ostr.str();
              cout<<"\n"<<send<<"\n";
              commSend(send);
              usleep(2500);             
            }
        }
        
    }
    g_scriptNode.Release();
    g_DepthGenerator.Release();
    g_UserGenerator.Release();
    g_Context.Release();
}
