/* ============================================================
 *
 * This file is a part of the RSB BH8 Hardware Integration (CoSiMA) project
 *
 * Copyright (C) 2019 by Dennis Leroy Wigand <dwigand at techfak dot uni-bielefeld dot de>,
 *                       Sebastian Schneider <sebschne at technfak dot uni-bielefeld dot de>
 *                    based on work of
 *                       Jens Kober,
 *                       Michael Gienger
 *
 * This file may be licensed under the terms of the
 * GNU Lesser General Public License Version 3 (the ``LGPL''),
 * or (at your option) any later version.
 *
 * Software distributed under the License is distributed
 * on an ``AS IS'' basis, WITHOUT WARRANTY OF ANY KIND, either
 * express or implied. See the LGPL for the specific language
 * governing rights and limitations.
 *
 * You should have received a copy of the LGPL along with this
 * program. If not, go to http://www.gnu.org/licenses/lgpl.html
 * or write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The development of this software was supported by:
 *   CoR-Lab, Research Institute for Cognition and Robotics
 *     Bielefeld University
 *
 * ============================================================ */

#include "rsb-bh8-interface.hpp"

// using namespace rsb;
using namespace cosima;

#define MAX_PPS_ELEMENTS 24

RsbBH8Interface::RsbBH8Interface()
{
	wrench = boost::shared_ptr<rst::dynamics::Wrench>(new rst::dynamics::Wrench());
	active = true;
}

RsbBH8Interface::~RsbBH8Interface()
{
	// active = false;
	// ShuttingDown();
}

void RsbBH8Interface::Error(int result, bool exit_program)
{
	BOOST_LOG_TRIVIAL(error) << "ERROR: " << result << ", MSG: " << bh.ErrorMessage(result);
	if (exit_program)
	{
		exit(-1);
	}
}

bool RsbBH8Interface::Initialize(std::string dev)
{
	int result;

	int hwIndex = BHandHardware::getBHandHardwareIndex("BH8-280");
	if (hwIndex < 0)
	{
		BOOST_LOG_TRIVIAL(error) << "The API has not been compiled to include the BH8-280 hand.";
		Error(result);
		return false;
	}
	// printf("hwIndex 1: %d\n", hwIndex);
	bh.setHardwareDesc(hwIndex);
	// printf("hwIndex 2: %d\n", hwIndex);
	bool use280Config = (strcmp(bh.getHardwareDesc()->getModelNumber(), "BH8-280") == 0);
	if (!use280Config)
	{
		BOOST_LOG_TRIVIAL(warning) << "No config found.";
	}

	if (result = handInitWithMenu(&bh, dev))
	{
		Error(result);
		return false;
	}

	BOOST_LOG_TRIVIAL(info) << "Initialization...";
	if (result = bh.InitHand(""))
	{
		Error(result);
		return false;
	}
	else
	{
		BOOST_LOG_TRIVIAL(info) << "Initialization... Done";
	}
	return true;
}

bool RsbBH8Interface::PrepareRealTime()
{
	int result;
	// initialize everything in velocity control mode
	// aslo sets which sensors are read out each cycle
	// see API/BHandSupervisoryRealTime.cpp
	if (result = bh.RTSetFlags("SG", 1, 3, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0))
	{
		Error(result);
		return false;
	}
	sprintf(buf, "%s", "");

	// set spread to position mode
	// teachIn is not possible with the spread in velocity mode as it is based on a PID position controller on-board
	bh.Set("S", "LCV", 0); // LCV (loop control verlocity) false
	bh.Set("S", "LCP", 1); // LCP (loop control position) true

	// "LCT" is the equivalent call for torques, see API/BHandBH8_280.cpp for a list
	return true;
}

int RsbBH8Interface::ShuttingDown()
{
	int result;
	BOOST_LOG_TRIVIAL(info) << "Shutting down.";
	if (result = bh.Command("GO")) // open all fingers
		Error(result);			   //print error message

	if (result = bh.Command("T")) // terminate all motors
		Error(result);

	DELAY(1);

	return 0;
}

void RsbBH8Interface::closeGrasp()
{
	int result;
	// char motor[4] = "123";
	//	bh.Close(motor);
	result = bh.Command("GC");
	if (result)
		Error(result);
}

void RsbBH8Interface::openGrasp()
{
	int result;
	result = bh.Command("GO");
	if (result)
		Error(result);
}

void RsbBH8Interface::receiveCommands(boost::shared_ptr<std::string> e)
{
	if (*e == "open")
	{
		BOOST_LOG_TRIVIAL(info) << "----- opening hand -----\n";
		command = *e;
	}
	else if (*e == "close")
	{
		BOOST_LOG_TRIVIAL(info) << "----- closing hand -----\n";
		command = *e;
	}
	else if (*e == "deactivate")
	{
		BOOST_LOG_TRIVIAL(info) << "----- deactivate hand ----- \n";
		command = *e;
	}
	else if (*e == "tare")
	{
		BOOST_LOG_TRIVIAL(info) << "----- taring F/T ----- \n";
		command = *e;
	}
}

bool RsbBH8Interface::InitializeRSB(std::string commandListenerScope, std::string wrenchInformerScope)
{
	BOOST_LOG_TRIVIAL(info) << "Initializing RSB Interface...";

	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::dynamics::Wrench> /* */> converter(new rsb::converter::ProtocolBufferConverter<rst::dynamics::Wrench>());
	rsb::converter::converterRepository<std::string>()->registerConverter(converter);

	rsb::Factory &factory = rsb::getFactory();

	rsb::Scope scope(commandListenerScope);
	listener = factory.createListener(scope);
	listener->addHandler(rsb::HandlerPtr(new rsb::DataFunctionHandler<std::string>(boost::bind(&RsbBH8Interface::receiveCommands, this, _1))));

	informer = factory.createInformer<rst::dynamics::Wrench>(wrenchInformerScope);

	BOOST_LOG_TRIVIAL(info) << "Initializing RSB Interface... Done";
	return true;
}

void RsbBH8Interface::LoopBlocking()
{
	BOOST_LOG_TRIVIAL(info) << "Entering Loop.";
	rst::dynamics::Forces *forces = wrench->mutable_forces();
	rst::dynamics::Torques *torques = wrench->mutable_torques();
	double f[3]; //force from ForceTorque sensor
	double t[3]; //torque from ForceTorque sensor
	double a[3]; //acceleration from ForceTorque sensor
	// unsigned int microseconds = 1500000; //sleep needed for work in vm

	while (active)
	{
		if (command == "tare")
		{
			BOOST_LOG_TRIVIAL(info) << "----- taring -----\n";
			command = "";
			bh.RTTareFT(); // To calibrate
		}
		else if (command == "open")
		{
			BOOST_LOG_TRIVIAL(info) << "----- opening -----\n";
			command = "";
			openGrasp();
		}
		else if (command == "close")
		{
			BOOST_LOG_TRIVIAL(info) << "----- closing hands -----\n";
			command = "";
			closeGrasp();
		}
		else if (command == "deactivate")
		{
			BOOST_LOG_TRIVIAL(info) << "----- deactivate ----- \n";
			command = "";
			active = false;
			break;
		}

		bh.RTGetFT(f, t);
		//	printf("Force: %7.3f %7.3f %7.3f \n", f[0], f[1], f[2]);
		//	printf("Torque: %7.3f %7.3f %7.3f \n", t[0], t[1], t[2]);
		//bh.RTGetA(a);     // get acceleration from the FT sensor
		//printf("Acceleration %7.3f %7.3f %7.3f\n", a[0], a[1], a[2]);
		//	usleep(microseconds);
		forces->set_x(f[0]);
		forces->set_y(f[1]);
		forces->set_z(f[2]);

		torques->set_a(t[0]);
		torques->set_b(t[1]);
		torques->set_c(t[2]);
		informer->publish(wrench);
	}
	BOOST_LOG_TRIVIAL(info) << "Exiting Loop.";
}

void RsbBH8Interface::SigHandler(int s) {
           printf("Caught signal %d\n",s);
           active=false;
}

std::function<void(int)> callback_wrapper;
void callback_function(int value)
{
  callback_wrapper(value);
}

int main(int argc, char *argv[])
{
	if (argc < 4)
	{
		std::cout << "usage: /scope/listener/cmd /scope/informer/wrench /dev/usb " << std::endl;
		return 1;
	}
	BOOST_LOG_TRIVIAL(info) << "Initializing BH8 Interface...";

	RsbBH8Interface hand;

	// initializes the hardware
	if (!hand.Initialize(argv[3]))
	{
		// TODO deactivate hand or in destructor
		return -1;
	}

	if (!hand.PrepareRealTime())
	{
		// TODO deactivate hand or in destructor
		return -1;
	}

	BOOST_LOG_TRIVIAL(info) << "Initializing BH8 Interface... Done";

	std::string listener_scope = (argc > 1) ? argv[1] : "/bhand/listener";
	std::string informer_scope = (argc > 2) ? argv[2] : "/bhand/informer/wrench";
	if (!hand.InitializeRSB(listener_scope, informer_scope))
	{
		// TODO deactivate hand or in destructor
		return -1;
	}

	hand.bh.RTStart("GS", BHMotorTorqueLimitProtect);
	hand.bh.RTUpdate();

	callback_wrapper = std::bind(&RsbBH8Interface::SigHandler,
                                 &hand,
                                 std::placeholders::_1);

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = callback_function;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

	hand.LoopBlocking();

	// hand.bh.RTAbort();

	if (hand.ShuttingDown() != 0)
	{
		return 1;
	}
	// return rsc::misc::suggestedExitCode(rsc::misc::waitForSignal());
	return 0;
}
