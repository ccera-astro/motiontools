//Required include files
#include <stdio.h>	
#include <string>
#include <iostream>
#include "pubSysCls.h"	
#include <unistd.h>

using namespace sFnd;

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

#define ACC_LIM_RPM_PER_SEC	5000
#define VEL_LIM_RPM			2000
#define MOVE_DISTANCE_CNTS	10000	
#define NUM_MOVES			5
#define TIME_TILL_TIMEOUT	10000	//The timeout used for homing(ms)
#define BK_DELAY            5000    // Brake delay

int main(int argc, char* argv[])
{
	//msgUser("Motion Server starting. Press Enter to continue.");

size_t portCount = 0;
	std::vector<std::string> comHubPorts;
	
	//Create the SysManager object. This object will coordinate actions among various ports
	// and within nodes. In this example we use this object to setup and open our port.
	SysManager* myMgr = SysManager::Instance();	

	//This will try to open the port. If there is an error/exception during the port opening,
	//the code will jump to the catch loop where detailed information regarding the error will be displayed;
	//otherwise the catch loop is skipped over
	try
	{
		printf("Trying to find hubs\n");
		SysManager::FindComHubPorts(comHubPorts);
		printf("Found %d SC Hubs\n", (int)comHubPorts.size());

		for (portCount = 0; portCount < comHubPorts.size() && portCount < NET_CONTROLLER_MAX; portCount++) {
			
			myMgr->ComHubPort(portCount, comHubPorts[portCount].c_str()); 	//define the first SC Hub port (port 0) to be associated 
											// with COM portnum (as seen in device manager)
		}
  
        /*
         * In our application, there should only be 1!!
         */
		if (portCount != 1) {
			
			printf("Incorrect number of SC hub ports\n");

			//msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key

			return -1;  //This terminates the main program
		}
		
		IPort &myPort = myMgr->Ports(0);
        /*
         * Open our single SC hub
         */
		myMgr->PortsOpen(portCount);				//Open the port

        /* Configure the SC hub's Brakes to be in auto control mode
         *(the brake will automatically activate, and deactivate based on the state of node 0)
         */
		myPort.BrakeControl.BrakeSetting(0,BRAKE_AUTOCONTROL);
		myPort.BrakeControl.BrakeSetting(1,BRAKE_AUTOCONTROL);
        

		//Once the code gets past this point, it can be assumed that the Port has been opened without issue
		//Now we can get a reference to our port object which we will use to access the node objects

		for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
			// Create a shortcut reference for a node
			INode &theNode = myPort.Nodes(iNode);
			theNode.EnableReq(false);				//Ensure Node is disabled before loading config file

			myMgr->Delay(200);
			/*
			 * REVISIT THIS!! We'd like to save the config files
			 *   from ClearView, and load them here.
			 */
			//theNode.Setup.ConfigLoad("Config File path");

			//The following statements will attempt to enable the node.  First,
			// any shutdowns or NodeStops are cleared, finally the node is enabled
			theNode.Status.AlertsClear();					//Clear Alerts on node 
			theNode.Motion.NodeStopClear();	//Clear Nodestops on Node 
			theNode.Setup.DelayToDisableMsecs = BK_DELAY;
			myMgr->Delay(200);			
			theNode.EnableReq(true);					//Enable node 
			//At this point the node is enabled
			printf("Node \t%zi enabled\n", iNode);
			double timeout = myMgr->TimeStampMsec() + TIME_TILL_TIMEOUT;	//define a timeout in case the node is unable to enable
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
			theNode.Motion.MoveWentDone();						//Clear the rising edge Move done register
			theNode.AccUnit(INode::RPM_PER_SEC);				//Set the units for Acceleration to RPM/SEC
			theNode.VelUnit(INode::RPM);						//Set the units for Velocity to RPM
			theNode.Motion.AccLimit = ACC_LIM_RPM_PER_SEC;		//Set Acceleration Limit (RPM/Sec)
			theNode.Motion.VelLimit = VEL_LIM_RPM;				//Set Velocity Limit (RPM)
			theNode.Setup.DelayToDisableMsecs = BK_DELAY;       // Sets brake delay -- our brakes are slow
			theNode.Motion.MoveVelStart(0.0);					//This should cause it to hold
		}
		sleep(20);
		for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++)
		{
			myPort.Nodes(iNode).EnableReq(false);
		}
		INode &Elevation = myPort.Nodes(0);
		INode &Azimuth = myPort.Nodes(1);
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
	return 0;			//End program
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
			myNode.Motion.MoveVelStart(move);
			double timeout = Manager->TimeStampMsec() + 500;			//define a timeout in case the node is unable to enable
			while (!myNode.Motion.VelocityReachedTarget()) {
				if (Manager->TimeStampMsec() > timeout) {
					rval = -3;
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
			myNode.Motion.MoveVelStart(0);
			double timeout = Manager->TimeStampMsec() + 500;			//define a timeout in case the node is unable to enable
			while (!myNode.Motion.VelocityReachedTarget()) {
				if (Manager->TimeStampMsec() > timeout) {
					rval = -3;
				}
			}
			usleep(500000);
			myNode.EnableReq(false);
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
        
        Manager->PortsClose();
        sleep(5);
        exit(0);
		*retvalP = xmlrpc_c::value_int(rval);
    }
};

int 
SetUpXMLRPC(SysManager *myMgr) {

    try {
        xmlrpc_c::registry myRegistry;

        xmlrpc_c::methodPtr const MoveMethodP(new MotorControl(myMgr));
        xmlrpc_c::methodPtr const AbortMethodP(new MotorShutdown(myMgr));
        xmlrpc_c::methodPtr const SysExitMethodP(new SysExit(myMgr));

        myRegistry.addMethod("Move", MoveMethodP);
        myRegistry.addMethod("Shutdown", AbortMethodP);
        myRegistry.addMethod("SysExit", SysExitMethodP);
        
        xmlrpc_c::serverAbyss myAbyssServer(
            xmlrpc_c::serverAbyss::constrOpt()
            .registryP(&myRegistry)
            .portNumber(36036));
        
        myAbyssServer.run();
        // xmlrpc_c::serverAbyss.run() never returns
        assert(false);
    } catch (exception const& e) {
        cerr << "Something failed.  " << e.what() << endl;
    }
    return 0;
}

