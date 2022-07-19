/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2021 University of Pretoria
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Jaco Marais <jaco.marais@tuks.co.za>
 */

#ifndef CONGESTION_COMPONENT_H
#define CONGESTION_COMPONENT_H

#include "ns3/object.h"
#include "ns3/log.h"
#include "ns3/packet.h"
#include "ns3/node-container.h"
#include "ns3/network-status.h"
#include "ns3/network-controller-components.h"
#include "ns3/lora-helper.h"
#include "ns3/lora-packet-tracker.h"

namespace ns3 {
namespace lorawan {

////////////////////////////////////////
// Congestion detection and management //
////////////////////////////////////////

class CongestionComponent : public NetworkControllerComponent
{

  enum CombiningMethod
  {
    AVERAGE,
    MAXIMUM
  };

public:
  static TypeId GetTypeId (void);

  //Constructor
  CongestionComponent ();
  //Destructor
  virtual ~CongestionComponent ();

  void OnReceivedPacket (Ptr<const Packet> packet,
                         Ptr<EndDeviceStatus> status,
                         Ptr<NetworkStatus> networkStatus);

  void BeforeSendingReply (Ptr<EndDeviceStatus> status,
                           Ptr<NetworkStatus> networkStatus);

  void OnFailedReply (Ptr<EndDeviceStatus> status,
                      Ptr<NetworkStatus> networkStatus);

  /**
   * Sets the number of gateways.
   */
  void SetGateways (NodeContainer gateways);

  /**
   * Sets the  tracker.
   */
  void SetTracker(LoraPacketTracker& tracker);


  /**
   * Sets the interval duration used for periodic calculations.
   */
  void SetCongestionPeriod(Time interval);


  /**
   * Periodically prints the status of congestion in the network to a file.
   */
  void EnablePeriodicNetworkCongestionStatusPrinting (NodeContainer gateways,
                                           std::string filename);

  /**
   * Print a summary of the congestion of all devices in the network.
   */
  void DoPrintNetworkCongestionStatus (NodeContainer gateways,
                            std::string filename);

  /**
   * Calculate the amount of congestion of all devices in the network.
   */
  void CalculateCongestion (std::string performanceStats);

  std::string
  PrintVector (std::vector<float> vector, int returnString);

private:

  bool m_legendPrinted = false;

  NodeContainer m_gateways;   //!< Set of gateways to monitor with this congestion component

  Time m_lastNetworkCongestionUpdate;

  Time m_congestionInterval;

  LoraPacketTracker* m_packetTracker = 0;


};
}
}

#endif
