#include "rsb-bh8-interface.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <string.h>
#include <deque>
#include "BHand.h"
#include "BHandAppHelper.h" // for connection menu
#include <sys/time.h>       // Needed for ms time.
#include <time.h>           // Needed for ms time.
#include <iostream>
#include <rsc/misc/SignalWaiter.h>
#include <rsb/Handler.h>
#include <rsb/Listener.h>
#include <rsb/Factory.h>
#include <unistd.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// See ../CMakeLists.txt for the generation of this file.
// The generated file can be found in ${BUILD_DIR}/protobuf_converter
#include <rst/dynamics/Wrench.pb.h>

#include <rst/dynamics/Torques.pb.h>
#include <rst/dynamics/Forces.pb.h>

using namespace rsb;

BHand bh; // Handles all hand communication
char buf[100]; // Buffer for reading back hand parameters
int value;     // Some commands use an int* instead of
// a buffer to relay information

int result; // Return value (error) of all BHand calls

bool deactivate_bool = false;

#define MAX_PPS_ELEMENTS 24

// ratios needed to convert from encoder counts to radians of the various joints
const double J2_RATIO = 125.0;
const double J2_ENCODER_RATIO = 50.0;
const double J3_RATIO = 375.0;
const double SPREAD_RATIO = 17.5;

// Pucks are the onboard motor controllers
const int MAX_PUCK_TORQUE = 8191; // max permissible torque

// Encoder counts per motor revolution. Can be read directly for each joint by calling bh.Get("GS", "CTS", &temp);
const int cts = 4096;

const double rpc = 2. * M_PI / cts;   // radians per count
const double cpr = cts / (2. * M_PI); // counts per radian

void Error()
{
    printf("ERROR: %d\n%s\n", result, bh.ErrorMessage(result));
    exit(0);
}

///////////////////////////////////////////////////////////
//  Initialize hand, set timeouts and baud rate          //
///////////////////////////////////////////////////////////
void Initialize()
{
    // Set hardware description before initialization
    // we have a BH8-282, which is identical to the BH8-280 except that the base is smaller
    // the library only knows the older BH8-280 but seems to work fine
    int hwIndex = BHandHardware::getBHandHardwareIndex("BH8-280");
	    if (hwIndex < 0)
    {
        printf("\n\nThe API has not been compiled to include target hand.\n");
        Error();
    }
    printf("hwIndex 1: %d\n", hwIndex);
    bh.setHardwareDesc(hwIndex);
    printf("hwIndex 2: %d\n", hwIndex);
    bool use280Config = (strcmp(bh.getHardwareDesc()->getModelNumber(), "BH8-280") == 0);
    if (!use280Config)
    {
        printf("No config found\n");
    }

    if (result = handInitWithMenu(&bh, "/dev/pcanusb32"))
        Error();

    printf("Initialization...");
    if (result = bh.InitHand(""))
        Error();
    else
        printf(" Done\n");
}

///////////////////////////////////////////////////////////
//  Set parameters, prepare data buffers                 //
///////////////////////////////////////////////////////////
void PrepareRealTime()
{
    // initialize everything in velocity control mode
    // aslo sets which sensors are read out each cycle
    // see API/BHandSupervisoryRealTime.cpp
    if (result = bh.RTSetFlags("SG", 1, 3, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0))
        Error();
    sprintf(buf, "%s", "");

    // set spread to position mode
    // teachIn is not possible with the spread in velocity mode as it is based on a PID position controller on-board
    bh.Set("S", "LCV", 0); // LCV (loop control verlocity) false
    bh.Set("S", "LCP", 1); // LCP (loop control position) true

    // "LCT" is the equivalent call for torques, see API/BHandBH8_280.cpp for a list
}


int After()
{

    printf("DEACTIVATING THE HAND");	
    if (result = bh.Command("GO")) // open all fingers
        Error();                   //print error message
                                   // this just checks for the key press and could be removed
    if (UnbufferedGetChar() != EOF)
        return 1;

    if (result = bh.Command("T")) // terminate all motors
        Error();
    if (UnbufferedGetChar() != EOF)
        return 1;
    DELAY(1);

    return 0;
}

void closeGrasp(){

	char motor[4] = "123";
//	bh.Close(motor);
	result = bh.Command("GC");
	std::cout << result << std::endl;
	if (result)
		Error();
//	return 0;
}

void openGrasp(){
	result = bh.Command("GO");
	if (result)
		Error();
	std::cout << result << std::endl;
//	return 0;
}

void receiveCommands(boost::shared_ptr<std::string> e){
	std::cout << "RECEIVED EVENT" << *e << std::endl;
	if (*e == "open"){
		printf("----- opening -----\n");
		openGrasp();}
	else if (*e =="close"){
		printf("----- closing hands -----\n");
		closeGrasp();}
	else if (*e == "deactivate"){
		printf("----- deactivate ----- \n");
		After();}

}

	

int main(int argc, char *argv[])
{

    printf("\n\n\r\t\tInitializing Software...\n");
    Initialize(); // initializes the hardware

	PrepareRealTime();

	bh.RTStart("GS", BHMotorTorqueLimitProtect);
    bh.RTUpdate();
//    bh.RTAbort();
	   boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::dynamics::Wrench> >
        converter(new rsb::converter::ProtocolBufferConverter<rst::dynamics::Wrench>());
    rsb::converter::converterRepository<std::string>()->registerConverter(converter);

        boost::shared_ptr < rst::dynamics::Wrench > wrench = boost::shared_ptr< rst::dynamics::Wrench > (new rst::dynamics::Wrench());



	rsc::misc::initSignalWaiter();
	
	Factory& factory = getFactory();
	Scope scope((argc >1) ? argv[1] : "/bhand/listener");
	std::string informer_scope = (argc >2) ? argv[2] : "/bhand/informer/wrench";
	ListenerPtr listener = factory.createListener(scope);
	printf("Adding handlers ...\n");
	listener->addHandler(HandlerPtr(new DataFunctionHandler<std::string> (&receiveCommands)));
	printf("Finished adding handlers\n");
	
	Informer<rst::dynamics::Wrench>::Ptr informer = factory.createInformer<rst::dynamics::Wrench>(informer_scope);
    bh.RTTareFT(); // To calibrate
	printf("Finished calibration\n");

	
	printf("Publishing wrench at: ");
	rst::dynamics::Forces* forces = wrench->mutable_forces();

	rst::dynamics::Torques* torques = wrench->mutable_torques();
	double f[3]; //force from ForceTorque sensor
	printf("Created array for force sensor data \n");
	double t[3]; // torque from ForceTorque sensor
	printf("Created array for torue sensor data \n");
	double a[3]; // acceleration from ..
	printf("Created array for acceleration sensor data\n");
	unsigned int microseconds = 1500000;

	while (UnbufferedGetChar() == EOF)
    {
	std::cout << "reading force torque: "  << std::endl;
	bh.RTGetFT(f,t);
	std::cout << " finished reading force torque: "  << std::endl;
//	printf("Force: %7.3f %7.3f %7.3f \n", f[0], f[1], f[2]);
//	printf("Torque: %7.3f %7.3f %7.3f \n", t[0], t[1], t[2]);
//      bh.RTGetA(a);     // get acceleration from the FT sensor
//      printf("Acceleration %7.3f %7.3f %7.3f\n", a[0], a[1], a[2]);
	usleep(microseconds);
	forces->set_x(f[0]);
	forces->set_y(f[1]);
	forces->set_z(f[2]);
	
	torques->set_a(t[0]);
	torques->set_b(t[1]);
	torques->set_c(t[2]);
	informer->publish(wrench);	
	}		

	

	printf("Stopping program");
	After();
	return rsc::misc::suggestedExitCode(rsc::misc::waitForSignal());

	
}
