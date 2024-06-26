//Required include files
#include <stdio.h>  
#include <string>
#include <iostream>
#include "pubSysCls.h"  
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <sys/types.h>          /* See NOTES */
#include <sys/socket.h>
#include <arpa/inet.h>



int proxyLogged = 0;
int proxyToken = 0;

int motor_state[4] = {0, 0, 0, 0};

time_t heartbeat = 0;

time_t trans_time;

#define MOTOR_LOG_FILE "./motor_performance.dat"

void log_motor_data(int which, double speed, double torque)
{
    FILE *fp;
    time_t now;
    struct tm *ltp;
    
    time(&now);
    trans_time = now;
    
    ltp = gmtime(&now);
    
    fp = fopen(MOTOR_LOG_FILE, "a");
    fprintf (fp, "%04d%02d%02d,%02d%02d%02d,%d,%f,%f\n", ltp->tm_year+1900, ltp->tm_mon+1, ltp->tm_mday,
        ltp->tm_hour, ltp->tm_min, ltp->tm_sec,
        which, speed, torque);
    fclose(fp);
    
}
    

using namespace sFnd;
SysManager* myMgr = NULL;

#define BK_DELAY            5000    // Brake delay

void alarm_handler (int q)
{
    printf ("Got the alarm, leaving\n");
    exit(0);
}

void exit_handler(int q)
{
        int rval;
        
        fprintf (stderr, "Shutting down due to signal\n");
        IPort &myPort = myMgr->Ports(0);
        int cnt = (int)myPort.NodeCount();
        fprintf (stderr, "Shutting down %d motor nodes\n", cnt);
        for (int which = 0; which < (int)myPort.NodeCount(); which++)
        {
            INode &myNode = myPort.Nodes(which);
            myNode.Motion.MoveVelStart(0.0);
            double timeout = myMgr->TimeStampMsec() + 1000;         //define a timeout in case the node is unable to enable
            while (!myNode.Motion.VelocityReachedTarget()) {
                if (myMgr->TimeStampMsec() > timeout) {
                    rval = -3;
                }
            }
            myNode.EnableReq(false);
            motor_state[which] = 0;
            sleep(1);
        }
    myMgr->PortsClose();
    sleep(1);
    exit(0);
}

// Send message and wait for newline
void msgUser(const char *msg) {
    std::cout << msg;
    getchar();
}

// Check if the bus power has been applied.
bool IsBusPowerLow(INode &theNode) {
    return theNode.Status.Power.Value().fld.InBusLoss;
}

//*********************************************************************************
//This program will load configuration files onto each node connected to the port, then executes
//sequential repeated moves on each axis.
//*********************************************************************************

#define ACC_LIM_RPM_PER_SEC 1250
#define VEL_LIM_RPM         1600 
#define TIME_TILL_TIMEOUT   10000   //The timeout used for homing(ms)


int 
SetUpXMLRPC(SysManager *);


int main(int argc, char* argv[])
{
    //msgUser("Motion Server starting. Press Enter to continue.");
    
	size_t portCount = 0;
    std::vector<std::string> comHubPorts;
    
    //Create the SysManager object. This object will coordinate actions among various ports
    // and within nodes. In this example we use this object to setup and open our port.
    myMgr = SysManager::Instance(); 

    //This will try to open the port. If there is an error/exception during the port opening,
    //the code will jump to the catch loop where detailed information regarding the error will be displayed;
    //otherwise the catch loop is skipped over
    try
    {
        printf("Trying to find hubs\n");
        SysManager::FindComHubPorts(comHubPorts);
        printf("Found %d SC Hubs\n", (int)comHubPorts.size());

        for (portCount = 0; portCount < comHubPorts.size() && portCount < NET_CONTROLLER_MAX; portCount++) {
            
            myMgr->ComHubPort(portCount, comHubPorts[portCount].c_str());   //define the first SC Hub port (port 0) to be associated 
                                            // with COM portnum (as seen in device manager)
        }
  
        /*
         * In our application, there should only be 1!!
         */
        if (portCount != 1) {
            
            printf("Incorrect number of SC hub ports\n");
            return -1;  //This terminates the main program
        }
        
        IPort &myPort = myMgr->Ports(0);
        
        /*
         * Open our single SC hub
         */
        myMgr->PortsOpen(portCount);                //Open the port
        
        printf ("Ensuring brakes are engaged on startup\n");
        printf ("First brake AUTOCONTROL\n");
        myPort.BrakeControl.BrakeSetting(0, BRAKE_AUTOCONTROL);
        sleep(1);
        printf ("Second brake AUTOCONTROL\n");
        myPort.BrakeControl.BrakeSetting(1, BRAKE_AUTOCONTROL);
        sleep (1);
        
        
        //Once the code gets past this point, it can be assumed that the Port has been opened without issue
        //Now we can get a reference to our port object which we will use to access the node objects

        for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
            // Create a shortcut reference for a node
            INode &theNode = myPort.Nodes(iNode);
            myMgr->Delay(500);
            
            theNode.Status.RT.Refresh();
			theNode.Status.Alerts.Refresh();
			if (!theNode.Status.RT.Value().cpm.AlertPresent)
			{
				fprintf (stderr, "No alerts present for Motor %d\n", (int)iNode);
            }
            else
            {
				char alertList[512];
				char instr[1024];
				char *retp;
				int cont = 0;
				
				fprintf (stderr, "Motor %d ALERTS PRESENT:\n", (int)iNode);
				
				theNode.Status.Alerts.Value().StateStr(alertList, sizeof(alertList));
				
				fprintf (stderr, "%s\n", alertList);
				
				fprintf (stderr, "Continue?");
				retp = fgets(instr, 1024, stdin);
				if (strcmp (instr, "y\n") == 0 || strcmp(instr, "Y\n") == 0 ||
					strcmp (instr, "yes\n") == 0 || strcmp(instr, "YES\n") == 0)
				{
					cont = 1;
				}
				if (cont != 1)
				{
					fprintf (stderr, "Not continuing\n");
					return -1;
				}
				
			}
            
            //The following statements will attempt to enable the node.  First,
            // any shutdowns or NodeStops are cleared, finally the node is enabled
            theNode.Status.AlertsClear();                   //Clear Alerts on node 
            theNode.Motion.NodeStopClear(); //Clear Nodestops on Node 
            //theNode.Setup.DelayToDisableMsecs = BK_DELAY;
            myMgr->Delay(1200);
            theNode.EnableReq(true);                    //Enable node 
            motor_state[(int)iNode] = 1;
            //At this point the node is enabled
            printf("Node \t%zi enable request pending\n", iNode);
            double timeout = myMgr->TimeStampMsec() + TIME_TILL_TIMEOUT;    //define a timeout in case the node is unable to enable
                                                                        //This will loop checking on the Real time values of the node's Ready status
            while (!theNode.Motion.IsReady()) {
                if (myMgr->TimeStampMsec() > timeout) {
                    if (IsBusPowerLow(theNode)) {
                        printf("Error: Bus Power low. Make sure 75V supply is powered on.\n");
                        return -1;
                    }
                    printf("Error: Timed out waiting for Node %d to enable\n", (int)iNode);
                    //msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
                    return -2;
                }
            }
            printf("Node %zi enabled\n", iNode);
            theNode.Motion.MoveWentDone();                      //Clear the rising edge Move done register
            theNode.AccUnit(INode::RPM_PER_SEC);                //Set the units for Acceleration to RPM/SEC
            theNode.VelUnit(INode::RPM);                        //Set the units for Velocity to RPM
            theNode.Motion.AccLimit = ACC_LIM_RPM_PER_SEC;      //Set Acceleration Limit (RPM/Sec)
            theNode.Motion.VelLimit = VEL_LIM_RPM;              //Set Velocity Limit (RPM)
            theNode.Motion.MoveVelStart(0.0);                   //This should cause it to hold
			sleep(1);
        }
        // Allow brakes time to disengage before taking MOVE orders from anyone
        sleep(2);
        time(&heartbeat);
        SetUpXMLRPC(myMgr);
    }
    catch (mnErr& theErr)
    {
        printf("Failure in node setup\n");
        //This statement will print the address of the error, the error code (defined by the mnErr class), 
        //as well as the corresponding error message.
        printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);

        msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
        return 0;  //This terminates the main program
    }

    // Close down the ports
    myMgr->PortsClose();
    return 0;           //End program
}

#define WIN32_LEAN_AND_MEAN  /* required by xmlrpc-c/server_abyss.hpp */

#include <cassert>
#include <stdexcept>
#include <iostream>
#ifdef _WIN32
#  include <windows.h>
#else
#  include <unistd.h>
#endif

using namespace std;

#include <xmlrpc-c/base.hpp>
#include <xmlrpc-c/registry.hpp>
#include <xmlrpc-c/server_abyss.hpp>

#ifdef _WIN32
  #define SLEEP(seconds) SleepEx(seconds * 1000);
#else
  #define SLEEP(seconds) sleep(seconds);
#endif

double curspeeds[] = {0.0, 0.0, 0.0, 0.0};

class MotorControl : public xmlrpc_c::method {
    SysManager *Manager;
public:
    MotorControl(SysManager *myMgr) {
        // signature and help strings are documentation -- the client
        // can query this information with a system.methodSignature and
        // system.methodHelp RPC.
        this->_signature = "i:id";
            // method's result and two arguments are integers
        this->_help = "This method moves a motor in a direction and speed";
        this->Manager = myMgr;
    }
    void
    execute(xmlrpc_c::paramList const& paramList,
            xmlrpc_c::value *   const  retvalP) {
        
        int rval = 0;
        int const which(paramList.getInt(0));
        double const move(paramList.getDouble(1));
        
        paramList.verifyEnd(2);
        
        IPort &myPort = Manager->Ports(0);
        int cnt = (int)myPort.NodeCount();
        if (which < cnt)
        {
            INode &myNode = myPort.Nodes(which);
            if (motor_state[which] != 1)
            {
				myNode.EnableReq(true);
				while (!myNode.Motion.IsReady())
				{
					usleep(100000);
				}
				sleep(3);
				motor_state[which] = 1;
			}
            try
            {
                    myNode.Motion.MoveVelStart(move);
                    double timeout = Manager->TimeStampMsec() + 2000;           //define a timeout in case the node is unable to enable
                    while (!myNode.Motion.VelocityReachedTarget()) {
                        if (Manager->TimeStampMsec() > timeout) {
                        rval = -3;
                        break;
                    }
                }
                double myTrq = myNode.Motion.TrqMeasured;
                log_motor_data(which, move, myTrq);
            }
            catch (mnErr& myerr)
            {
                 rval = myerr.ErrorCode;
            }
        }
        else
        {
            rval = -2;
        }
        if (rval == 0)
        {
			curspeeds[which] = move;
		}
        *retvalP = xmlrpc_c::value_int(rval);
    }
};

class MotorEnable : public xmlrpc_c::method {
    SysManager *Manager;
public:
    MotorEnable(SysManager *myMgr) {
        // signature and help strings are documentation -- the client
        // can query this information with a system.methodSignature and
        // system.methodHelp RPC.
        this->_signature = "i:i";
            // method's result and two arguments are integers
        this->_help = "This method enables a motor";
        this->Manager = myMgr;
    }
    void
    execute(xmlrpc_c::paramList const& paramList,
            xmlrpc_c::value *   const  retvalP) {
        
        int rval = 0;
        int const which(paramList.getInt(0));
        
        paramList.verifyEnd(1);
        
        IPort &myPort = Manager->Ports(0);
        int cnt = (int)myPort.NodeCount();
        if (which < cnt)
        {
    
            INode &myNode = myPort.Nodes(which);
            myNode.EnableReq(true);
            motor_state[which] = 1;
            usleep(500000);
            myNode.Motion.MoveVelStart(0.0);
            //myPort.BrakeControl.BrakeSetting(which, BRAKE_ALLOW_MOTION);
            double timeout = Manager->TimeStampMsec() + 2000;           //define a timeout in case the node is unable to enable
            while (!myNode.Motion.VelocityReachedTarget()) {
                if (Manager->TimeStampMsec() > timeout) {
                    rval = -3;
                }
            }
            sleep(BK_DELAY/1000);
        }
        else
        {
            rval = -2;
        }
        if (rval == 0)
        {
			curspeeds[which] = 0.0;
		}
        *retvalP = xmlrpc_c::value_int(rval);
    }
};

class AngleMove : public xmlrpc_c::method {
    SysManager *Manager;
public:
    AngleMove(SysManager *myMgr) {
        // signature and help strings are documentation -- the client
        // can query this information with a system.methodSignature and
        // system.methodHelp RPC.
        this->_signature = "i:id";
            // method's result and two arguments are integer/float
        this->_help = "This method moves a motor through some number of degrees";
        this->Manager = myMgr;
    }
    void
    execute(xmlrpc_c::paramList const& paramList,
            xmlrpc_c::value *   const  retvalP) {
        
        int rval = 0;
        int const which(paramList.getInt(0));
        double const angle(paramList.getDouble(1));
        
        paramList.verifyEnd(2);
        
        IPort &myPort = Manager->Ports(0);
        int cnt = (int)myPort.NodeCount();
        if (which < cnt)
        {
    
            INode &myNode = myPort.Nodes(which);
            if (motor_state[which] != 1)
            {
				myNode.EnableReq(true);
				while (!myNode.Motion.IsReady())
				{
					usleep(100000);
				}
				sleep(3);
				motor_state[which] = 1;
			}
            //Get the positioning resolution of the Node
			uint32_t myPosnResolution = myNode.Info.PositioningResolution;
            double target;
            int waitcount = 0;
            
            // Turn angle into rotations, and then into encoder counts
            target = (angle / 360.0) * (double)myPosnResolution;
            myNode.Motion.MovePosnStart((int32_t)target, false, false);
        }
        else
        {
            rval = -2;
        }
        *retvalP = xmlrpc_c::value_int(rval);
    }
};

class MoveWait : public xmlrpc_c::method {
    SysManager *Manager;
public:
    MoveWait (SysManager *myMgr) {
        // signature and help strings are documentation -- the client
        // can query this information with a system.methodSignature and
        // system.methodHelp RPC.
        this->_signature = "i:ii";
            // method's result and two arguments are integer/float
        this->_help = "This method waits for a move to complete";
        this->Manager = myMgr;
    }
    void
    execute(xmlrpc_c::paramList const& paramList,
            xmlrpc_c::value *   const  retvalP) {
        
        int rval = 0;
        int const which(paramList.getInt(0));
        int const waitcount(paramList.getInt(1));
        
        paramList.verifyEnd(2);
        
        IPort &myPort = Manager->Ports(0);
        int cnt = (int)myPort.NodeCount();
        if (which < cnt)
        {
    
            INode &myNode = myPort.Nodes(which);
            
            // Turn angle into rotations, and then into encoder counts
            int wc = waitcount;
            if (wc == 0)
            {
				if (myNode.Motion.MoveIsDone())
				{
					rval = 1;
				}
				else
				{
					rval = 0;
				}
			}
			else
			{
				while (!myNode.Motion.MoveIsDone())
				{
					usleep (100000);
					if (wc-- <= 0)
					{
						rval = -3;
						break;
					}
				}
			}
        }
        else
        {
            rval = -2;
        }
        *retvalP = xmlrpc_c::value_int(rval);
    }
};

class MotorShutdown : public xmlrpc_c::method {
    SysManager *Manager;
public:
    MotorShutdown(SysManager *myMgr) {
        // signature and help strings are documentation -- the client
        // can query this information with a system.methodSignature and
        // system.methodHelp RPC.
        this->_signature = "i:i";
            // method's result and two arguments are integers
        this->_help = "This method shuts down a motor";
        this->Manager = myMgr;
    }
    void
    execute(xmlrpc_c::paramList const& paramList,
            xmlrpc_c::value *   const  retvalP) {
        
        int rval = 0;
        int const which(paramList.getInt(0));
        paramList.verifyEnd(1);
        
        IPort &myPort = Manager->Ports(0);
        int cnt = (int)myPort.NodeCount();
        if (which < cnt)
        {
    
            INode &myNode = myPort.Nodes(which);
            myNode.Motion.MoveVelStart(0.0);
            double timeout = Manager->TimeStampMsec() + 2000;           //define a timeout in case the node is unable to enable
            while (!myNode.Motion.VelocityReachedTarget()) {
                if (Manager->TimeStampMsec() > timeout) {
                    rval = -3;
                }
            }
            //myPort.BrakeControl.BrakeSetting(which, BRAKE_PREVENT_MOTION);
            sleep((BK_DELAY/1000));
            myNode.EnableReq(false);
            motor_state[which] = 0;
            curspeeds[which] = 0.0;
        }
        else
        {
            rval = -2;
        }
        *retvalP = xmlrpc_c::value_int(rval);
    }
};

class SysExit : public xmlrpc_c::method {
    SysManager *Manager;
public:
    SysExit(SysManager *myMgr) {
        // signature and help strings are documentation -- the client
        // can query this information with a system.methodSignature and
        // system.methodHelp RPC.
        this->_signature = "i:i";
            // method's result and two arguments are integers
        this->_help = "This method shuts down the ClearPath system cleanly";
        this->Manager = myMgr;
    }
    void
    execute(xmlrpc_c::paramList const& paramList,
            xmlrpc_c::value *   const  retvalP) {
        
        int rval = 0;
        
        IPort &myPort = Manager->Ports(0);
        INode &myNode = myPort.Nodes(0);
        INode &myNode2 = myPort.Nodes(1);

        myNode.EnableReq(false);
        sleep(1);
        myNode2.EnableReq(false);
        sleep(1);

        Manager->PortsClose();
        alarm(2);
        *retvalP = xmlrpc_c::value_int(rval);
    }
};

class QueryTimestamp: public xmlrpc_c::method {
    SysManager *Manager;
public:
    QueryTimestamp(SysManager *myMgr) {
        // signature and help strings are documentation -- the client
        // can query this information with a system.methodSignature and
        // system.methodHelp RPC.
        this->_signature = "d:i";
            // method's result and two arguments are integers
        this->_help = "This method queries the timestamp";
        this->Manager = myMgr;
    }
    void
    execute(xmlrpc_c::paramList const& paramList,
            xmlrpc_c::value *   const  retvalP) {
        
        double rval = (double)heartbeat;
        fprintf (stderr, "Query time is %lf\n", rval);
        *retvalP = xmlrpc_c::value_double(rval);
    }
};

class QueryAlerts: public xmlrpc_c::method {
    SysManager *Manager;
public:
    QueryAlerts(SysManager *myMgr) {
        // signature and help strings are documentation -- the client
        // can query this information with a system.methodSignature and
        // system.methodHelp RPC.
        this->_signature = "s:i";
            // method's result and two arguments are integers
        this->_help = "This method queries outstanding alerts";
        this->Manager = myMgr;
    }
    void
    execute(xmlrpc_c::paramList const& paramList,
            xmlrpc_c::value *   const  retvalP) {
        
        string rval = "None";
        int const which(paramList.getInt(0));
        
        paramList.verifyEnd(1);
        
        IPort &myPort = Manager->Ports(0);
        int cnt = (int)myPort.NodeCount();
        if (which < cnt)
        {
            INode &theNode = myPort.Nodes(which);
                   
			theNode.Status.RT.Refresh();
			theNode.Status.Alerts.Refresh();
			if (!theNode.Status.RT.Value().cpm.AlertPresent)
			{
                    rval = "None";
            }
            else
            {
				char alertList[256];
				
				theNode.Status.Alerts.Value().StateStr(alertList, sizeof(alertList));
				
				rval = (string)alertList;
			}
        }
        else
        {
            rval = "BADNODE";
        }
 
        *retvalP = xmlrpc_c::value_string(rval);
    }
};

class ResetAlerts: public xmlrpc_c::method {
    SysManager *Manager;
public:
    ResetAlerts(SysManager *myMgr) {
        // signature and help strings are documentation -- the client
        // can query this information with a system.methodSignature and
        // system.methodHelp RPC.
        this->_signature = "i:i";
            // method's result and two arguments are integers
        this->_help = "This method resets outstanding alerts";
        this->Manager = myMgr;
    }
    void
    execute(xmlrpc_c::paramList const& paramList,
            xmlrpc_c::value *   const  retvalP) {
        
        int const which(paramList.getInt(0));
        int rval = 0;
        
        paramList.verifyEnd(1);
        
        IPort &myPort = Manager->Ports(0);
        int cnt = (int)myPort.NodeCount();
        if (which < cnt)
        {
            INode &theNode = myPort.Nodes(which);
            theNode.Status.AlertsClear();
            theNode.Motion.NodeStopClear();
            rval = 0;
        }
        else
        {
            rval = -2;
        }
        *retvalP = xmlrpc_c::value_int(rval);
    }
};

class QueryTorque: public xmlrpc_c::method {
    SysManager *Manager;
public:
    QueryTorque(SysManager *myMgr) {
        // signature and help strings are documentation -- the client
        // can query this information with a system.methodSignature and
        // system.methodHelp RPC.
        this->_signature = "d:i";
            // method's result and two arguments are integers
        this->_help = "This method queries current torque alerts";
        this->Manager = myMgr;
    }
    void
    execute(xmlrpc_c::paramList const& paramList,
            xmlrpc_c::value *   const  retvalP) {
        
        int const which(paramList.getInt(0));
        double rval = 0;
        
        paramList.verifyEnd(1);
        
        IPort &myPort = Manager->Ports(0);
        int cnt = (int)myPort.NodeCount();
        if (which < cnt)
        {
            INode &theNode = myPort.Nodes(which);
            double myTrq = theNode.Motion.TrqMeasured;
            rval = myTrq;
        }
        else
        {
            rval = -2.0;
        }
        *retvalP = xmlrpc_c::value_double(rval);
    }
};

class AccLimit : public xmlrpc_c::method {
    SysManager *Manager;
public:
    AccLimit (SysManager *myMgr) {
        // signature and help strings are documentation -- the client
        // can query this information with a system.methodSignature and
        // system.methodHelp RPC.
        this->_signature = "i:ii";
            // method's result and two arguments are integer/float
        this->_help = "This method sets the acceleration limit for a motor";
        this->Manager = myMgr;
    }
    void
    execute(xmlrpc_c::paramList const& paramList,
            xmlrpc_c::value *   const  retvalP) {
        
        int rval = 0;
        int const which(paramList.getInt(0));
        int const limit(paramList.getInt(1));
        
        paramList.verifyEnd(2);
        
        IPort &myPort = Manager->Ports(0);
        int cnt = (int)myPort.NodeCount();
        if (which < cnt)
        {
    
            INode &myNode = myPort.Nodes(which);
            myNode.Motion.AccLimit = limit;
        }
        else
        {
            rval = -2;
        }
        *retvalP = xmlrpc_c::value_int(rval);
    }
};

class VelLimit : public xmlrpc_c::method {
    SysManager *Manager;
public:
    VelLimit (SysManager *myMgr) {
        // signature and help strings are documentation -- the client
        // can query this information with a system.methodSignature and
        // system.methodHelp RPC.
        this->_signature = "i:ii";
            // method's result and two arguments are integer/float
        this->_help = "This method sets the velocity limit for a motor";
        this->Manager = myMgr;
    }
    void
    execute(xmlrpc_c::paramList const& paramList,
            xmlrpc_c::value *   const  retvalP) {
        
        int rval = 0;
        int const which(paramList.getInt(0));
        int const limit(paramList.getInt(1));
        
        paramList.verifyEnd(2);
        
        IPort &myPort = Manager->Ports(0);
        int cnt = (int)myPort.NodeCount();
        if (which < cnt)
        {
    
            INode &myNode = myPort.Nodes(which);
            myNode.Motion.VelLimit = limit;
        }
        else
        {
            rval = -2;
        }
        *retvalP = xmlrpc_c::value_int(rval);
    }
};

class ProxyLogin : public xmlrpc_c::method {
    SysManager *Manager;
public:
    ProxyLogin () {
        // signature and help strings are documentation -- the client
        // can query this information with a system.methodSignature and
        // system.methodHelp RPC.
        this->_signature = "i:s";
            // method's result and two arguments are integer/float
        this->_help = "This method allows the security proxy to login";
        this->Manager = myMgr;
    }
    void
    execute(xmlrpc_c::paramList const& paramList,
            xmlrpc_c::value *   const  retvalP) {
        
        int rval = 0;
        std::string logstr(paramList.getString(0));
        paramList.verifyEnd(1);
        FILE *fp;
        
        int OK = 0;
        
        logstr = logstr + "\n";
        fp = fopen ("./motion-access.txt", "r");
        if (fp != NULL)
        {
			char mybuf[256];
			
			while (fgets(mybuf, 256, fp) != 0)
			{
				if (mybuf == logstr)
				{
					OK = 1;
				}
			}
			fclose(fp);
		}
        else if (logstr == "Astronomer:Gibbly\n")
        {
			OK = 1;
		}
		
		if (OK != 0)
		{
			time_t t;
			
			time(&t);
			srandom((unsigned int)t);
			proxyToken = random();
			*retvalP = xmlrpc_c::value_int(proxyToken);
		}
		else
		{
			*retvalP = xmlrpc_c::value_int(-1);
		}
    }
};


class HeartBeat : public xmlrpc_c::method {
    SysManager *Manager;
public:
    HeartBeat () {
        // signature and help strings are documentation -- the client
        // can query this information with a system.methodSignature and
        // system.methodHelp RPC.
        this->_signature = "s:i";
            // method's result and two arguments are integer/float
        this->_help = "This allows an app to update the heartbeat time";
        this->Manager = myMgr;
    }
    void
    execute(xmlrpc_c::paramList const& paramList,
            xmlrpc_c::value *   const  retvalP) {
        
        char buf[256];
        int rval = 0;
        int logstr(paramList.getInt(0));
        paramList.verifyEnd(1);
        
        time(&heartbeat);
        sprintf(buf, "%ld,%lf,%lf", (long)heartbeat, curspeeds[0], curspeeds[1]);
        *retvalP = xmlrpc_c::value_string(buf);
    }
};

int 
SetUpXMLRPC(SysManager *myMgr) {

    try {
        xmlrpc_c::registry myRegistry;

        xmlrpc_c::methodPtr const MoveMethodP(new MotorControl(myMgr));
        xmlrpc_c::methodPtr const ShutdownMethodP(new MotorShutdown(myMgr));
        xmlrpc_c::methodPtr const SysExitMethodP(new SysExit(myMgr));
        xmlrpc_c::methodPtr const EnableMethodP(new MotorEnable(myMgr));
        xmlrpc_c::methodPtr const QueryMethodP(new QueryTimestamp(myMgr));
        xmlrpc_c::methodPtr const QueryAlertsP(new QueryAlerts(myMgr));
        xmlrpc_c::methodPtr const ResetAlertsP(new ResetAlerts(myMgr));
        xmlrpc_c::methodPtr const QueryTorqueP(new QueryTorque(myMgr));
        xmlrpc_c::methodPtr const AngleMoveP(new AngleMove(myMgr));
        xmlrpc_c::methodPtr const MoveWaitP(new MoveWait(myMgr));
        xmlrpc_c::methodPtr const AccLimitP(new AccLimit(myMgr));
        xmlrpc_c::methodPtr const VelLimitP(new VelLimit(myMgr));
        xmlrpc_c::methodPtr const ProxyLoginP(new ProxyLogin());
        xmlrpc_c::methodPtr const HeartBeatP(new HeartBeat());
        
        myRegistry.addMethod("Move", MoveMethodP);
        myRegistry.addMethod("Shutdown", ShutdownMethodP);
        myRegistry.addMethod("Enable", EnableMethodP);
        myRegistry.addMethod("SysExit", SysExitMethodP);
        myRegistry.addMethod("QueryTime", QueryMethodP);
        myRegistry.addMethod("QueryAlerts", QueryAlertsP);
        myRegistry.addMethod("ResetAlerts", ResetAlertsP);
        myRegistry.addMethod("QueryTorque", QueryTorqueP);
        myRegistry.addMethod("AngleMove", AngleMoveP);
        myRegistry.addMethod("AccLimit", AccLimitP);
        myRegistry.addMethod("VelLimit", VelLimitP);
        myRegistry.addMethod ("MoveWait", MoveWaitP);
        myRegistry.addMethod ("ProxyLogin", ProxyLoginP);
        myRegistry.addMethod ("HeartBeat", HeartBeatP);
        
        signal(SIGALRM, alarm_handler);
        signal(SIGINT, exit_handler);
        signal(SIGHUP, exit_handler);
        signal(SIGQUIT, exit_handler);
        signal(SIGABRT, exit_handler);

		struct sockaddr_in s;
		s.sin_family = AF_INET;
		s.sin_port = htons(36036);
		inet_pton(AF_INET, "127.0.0.1", &s.sin_addr);

        time(&heartbeat);
        printf ("Going into abyss!\n");
        xmlrpc_c::serverAbyss myAbyssServer(
            xmlrpc_c::serverAbyss::constrOpt()
            .registryP(&myRegistry)
            //.portNumber(36036)
            .sockAddrP((const sockaddr *)&s)
            .sockAddrLen(sizeof(s)));
        
        myAbyssServer.run();
        // xmlrpc_c::serverAbyss.run() never returns
        assert(false);
    } catch (exception const& e) {
        cerr << "Something failed.  " << e.what() << endl;
    }
    return 0;
}

