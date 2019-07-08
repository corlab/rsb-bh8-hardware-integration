#include "rsb-bh8-orocos.hpp"
#include <rtt/Operation.hpp>
#include <string>
#include <fstream>
#include <streambuf>
#include <sstream>

// #define N_PUCKS 4;

using namespace cosima;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

BarrettHandRTT::BarrettHandRTT(const std::string &name) : TaskContext(name),
														  is_configured(false),
														  compliance_enabled(false),
														  p_gain(30.0),
														  d_gain(0.1),
														  velocity_gain(0.1),
														  zeroing_active(false),
														  out_converged(true)
{
	this->addOperation("initialize", &BarrettHandRTT::initialize, this, ClientThread);

	this->addOperation("open", &BarrettHandRTT::open, this, ClientThread);
	this->addOperation("close", &BarrettHandRTT::close, this, ClientThread);
	this->addOperation("openSpread", &BarrettHandRTT::openSpread, this, ClientThread);
	this->addOperation("closeSpread", &BarrettHandRTT::closeSpread, this, ClientThread);
	this->addOperation("deactivate_danger", &BarrettHandRTT::deactivate_danger, this, ClientThread);

	this->addOperation("scheduleZeroing", &BarrettHandRTT::scheduleZeroing, this, ClientThread).doc("Zeroing of the sensor is scheduled for the next update cycle from gazebo.");

	this->addProperty("p_gain", p_gain);
	this->addProperty("d_gain", d_gain);

	this->addProperty("velocity_gain", velocity_gain);

	this->addProperty("out_converged", out_converged);
}

bool BarrettHandRTT::startHook()
{
	bool ret = true;
	if (!out_rsb_hand_command_port.connected())
	{
		ret = false;
		RTT::log(RTT::Error) << "[" << this->getName() << "] out_rsb_hand_command_port not connected to hand. Canceling start!" << RTT::endlog();
	}
	if (!in_rsb_hand_converged_port.connected())
	{
		ret = false;
		RTT::log(RTT::Error) << "[" << this->getName() << "] in_rsb_hand_converged_port not connected to hand. Canceling start!" << RTT::endlog();
	}
	return ret;
}

void BarrettHandRTT::updateHook()
{
	in_rsb_hand_converged_flow = in_rsb_hand_converged_port.read(in_rsb_hand_converged);
	if (in_rsb_hand_converged_flow != RTT::NoData)
	{
		out_converged = in_rsb_hand_converged; // TODO not necessary!
		out_converged_port.write(out_converged);
	}

	in_rsb_hand_wrench_flow = in_rsb_hand_wrench_port.read(in_rsb_hand_wrench);
	if (in_rsb_hand_wrench_flow != RTT::NoData)
	{
		out_hand_FT = in_rsb_hand_wrench; // TODO not necessary!
		out_hand_FT_port.write(out_hand_FT);
	}
}

bool BarrettHandRTT::configureHook()
{
	out_converged = true;
	this->addPort("out_converged", out_converged_port).doc("Output port for the convergency criterion.");
	out_converged_port.setDataSample(out_converged);

	this->addPort("in_rsb_hand_converged_port", in_rsb_hand_converged_port).doc("Input port to read feedback via rsb from hand regarding the convergence status.");
	in_rsb_hand_converged_flow = RTT::NoData;
	in_rsb_hand_converged = true;

	this->addPort("in_rsb_hand_wrench_port", in_rsb_hand_wrench_port).doc("Input port to read feedback via rsb from hand regarding the FT wrench.");
	in_rsb_hand_wrench_flow = RTT::NoData;
	in_rsb_hand_wrench = rstrt::dynamics::Wrench();

	out_hand_FT = rstrt::dynamics::Wrench();
	this->addPort("hand_FT", out_hand_FT_port).doc("Output port for the force torque feedback.");
	out_hand_FT_port.setDataSample(out_hand_FT);

	out_rsb_hand_command = -1;
	this->addPort("out_rsb_hand_command_port", out_rsb_hand_command_port).doc("Output port via rsb to send commands to the hand.");
	out_rsb_hand_command_port.setDataSample(out_rsb_hand_command);

	this->is_configured = true;
	return this->is_configured;
}

void BarrettHandRTT::initialize()
{
	out_rsb_hand_command = 0;
	out_rsb_hand_command_port.write(out_rsb_hand_command);
}

double BarrettHandRTT::getOrocosTime()
{
	return 1E-9 * RTT::os::TimeService::ticks2nsecs(
					  RTT::os::TimeService::Instance()->getTicks());
}

void BarrettHandRTT::scheduleZeroing()
{
	out_rsb_hand_command = 5;
	out_rsb_hand_command_port.write(out_rsb_hand_command);
}

bool BarrettHandRTT::doneMoving(const unsigned pair_index)
{
}

bool BarrettHandRTT::doneMovingAll()
{
	return doneMoving(0) && doneMoving(1) && doneMoving(2) && doneMoving(3);
}

void BarrettHandRTT::open()
{
	out_rsb_hand_command = 1;
	out_rsb_hand_command_port.write(out_rsb_hand_command);
}

void BarrettHandRTT::close()
{
	out_rsb_hand_command = 2;
	out_rsb_hand_command_port.write(out_rsb_hand_command);
}

void BarrettHandRTT::openSpread()
{
	out_rsb_hand_command = 3;
	out_rsb_hand_command_port.write(out_rsb_hand_command);
}

void BarrettHandRTT::closeSpread()
{
	out_rsb_hand_command = 4;
	out_rsb_hand_command_port.write(out_rsb_hand_command);
}

void BarrettHandRTT::deactivate_danger()
{
	out_rsb_hand_command = 1337;
	out_rsb_hand_command_port.write(out_rsb_hand_command);
	this->stop();
}

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(cosima::BarrettHandRTT)