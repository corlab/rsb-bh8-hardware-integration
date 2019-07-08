#pragma once

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <rtt/os/Semaphore.hpp>

#include <Eigen/Dense>
#include <Eigen/LU>

#include <vector>

#include <Eigen/Core>
#include <time.h>
#include <rtt/os/TimeService.hpp>
#include <sstream>
#include <rtt/Logger.hpp>

#include <thread>
#include <memory>
#include <string>
#include <fstream>
#include <streambuf>
#include <sstream>

#include <boost/shared_ptr.hpp>

#include <rst-rt/dynamics/Wrench.hpp>

namespace cosima
{

class BarrettHandRTT : public RTT::TaskContext
{
public:
	BarrettHandRTT(std::string const &name);
	bool configureHook();
	bool startHook();
	void updateHook();

protected:
	double getOrocosTime();

	bool registerSensors();

	void scheduleZeroing();
	bool zeroing_active;
	rstrt::dynamics::Wrench zero_hand_FT;

	//    Ports:
	// RTT::OutputPort<rstrt::robot::JointState> out_hand_JointFeedback_port;
	// rstrt::robot::JointState out_hand_JointFeedback;

	RTT::OutputPort<rstrt::dynamics::Wrench> out_hand_FT_port;
	rstrt::dynamics::Wrench out_hand_FT;

	RTT::OutputPort<bool> out_converged_port;
	bool out_converged;
	// int internal_converged_status;

	RTT::InputPort<rstrt::dynamics::Wrench> in_rsb_hand_wrench_port;
	RTT::FlowStatus in_rsb_hand_wrench_flow;
	rstrt::dynamics::Wrench in_rsb_hand_wrench;

	RTT::InputPort<bool> in_rsb_hand_converged_port;
	RTT::FlowStatus in_rsb_hand_converged_flow;
	bool in_rsb_hand_converged;

	RTT::OutputPort<int> out_rsb_hand_command_port;
	int out_rsb_hand_command;

	// int n_allJoints;

	void initialize();
	// void idle();
	// void run();
	void deactivate_danger();

	// void setCompliance(bool enable);
	void open();
	void close();
	void openSpread();
	void closeSpread();

	bool doneMoving(const unsigned pair_index);
	bool doneMovingAll();

	bool compliance_enabled;

	double
		p_gain,
		d_gain,
		velocity_gain;

private:
	bool is_configured;
};

} // namespace cosima