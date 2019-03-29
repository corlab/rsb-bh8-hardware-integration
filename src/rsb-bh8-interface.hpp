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

#pragma once

#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <boost/log/trivial.hpp>
#include <deque>
#include "BHand.h"
#include "BHandAppHelper.h" // for connection menu
#include <sys/time.h>       // Needed for ms time.
#include <time.h>           // Needed for ms time.
#include <iostream>
#include <atomic>
// #include <rsc/misc/SignalWaiter.h>
#include <rsb/Handler.h>
#include <rsb/Listener.h>
#include <rsb/Factory.h>
#include <unistd.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/filter/OriginFilter.h>

#include <boost/shared_ptr.hpp>

#include <rst/dynamics/Wrench.pb.h>
#include <rst/dynamics/Torques.pb.h>
#include <rst/dynamics/Forces.pb.h>

#include <signal.h>

namespace cosima
{

class RsbBH8Interface
{
public:
  RsbBH8Interface();
  ~RsbBH8Interface();
  void Error(int result, bool exit_program = false);

  /**
     * Initialize hand, set timeouts and baud rate.
     * Set hardware description before initialization.
	 * We have a BH8-282, which is identical to the BH8-280 except that the base is smaller.
	 * The library only knows the older BH8-280 but seems to work fine.
     */
  bool Initialize(std::string dev);
  bool PrepareRealTime();
  bool InitializeRSB(std::string commandListenerScope, std::string wrenchInformerScope);

  int ShuttingDown();
  void closeGrasp();
  void openGrasp();
  void receiveCommands(boost::shared_ptr<std::string> e);
  void LoopBlocking();
  void SigHandler(int s);

  /* VERY UNSAFE ACCESS, I HOPE YOU KNOW WHAT YOU ARE DOING WHEN YOU TRY TO CHANGE SOMETHING! */
  BHand bh; // Handles all hand communication
private:
  std::string command;
  char buf[100]; // Buffer for reading back hand parameters
  // int value;	 // Some commands use an int* instead of a buffer to relay information

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

  rsb::ListenerPtr listener;
  rsb::Informer<rst::dynamics::Wrench>::Ptr informer;
  boost::shared_ptr<rst::dynamics::Wrench> wrench;

  std::atomic<bool> active;
};

} // namespace cosima