/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2018 University of Padova
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
 * Author: Davide Magrin <magrinda@dei.unipd.it>
 * Author: Martina Capuzzo <capuzzom@dei.unipd.it>
 */

#ifndef LORA_PACKET_TRACKER_H
#define LORA_PACKET_TRACKER_H

#include "ns3/packet.h"
#include "ns3/nstime.h"
#include "ns3/lora-frame-header.h"
#include "ns3/lorawan-mac-header.h"

#include <bits/stdint-uintn.h>
#include <map>
#include <string>

namespace ns3 {
namespace lorawan {

enum PhyPacketOutcome
{
  RECEIVED,
  INTERFERED,
  NO_MORE_RECEIVERS,
  UNDER_SENSITIVITY,
  LOST_BECAUSE_TX,
  UNSET
};

struct PacketStatus
{
  Ptr<Packet const> packet;
  uint32_t senderId;
  Time sendTime;
  std::map<int, enum PhyPacketOutcome> outcomes;
  bool txSuccessful;
};

struct MacPacketStatus
{
  Ptr<Packet const> packet;
  uint32_t senderId;
  Time sendTime;
  Time receivedTime;
  std::map<int, Time> receptionTimes;
};

struct RetransmissionStatus
{
  Time firstAttempt;
  Time finishTime;
  uint8_t reTxAttempts;
  bool successful;
};

struct appPacketStatus
{
  Time generatedTime;
  int size;
  LorawanMacHeader::MType mtype;
};

struct NoEnergyPacketStatus
{
  Time generatedTime;
  uint32_t device;
  bool success;
  LorawanMacHeader::MType type;
};

struct NoEnergyPacketExpandedStatus
{
  Time generatedTime;
  uint32_t device;
  bool successValues[8]; //up to 8 transmissions allowed
  int count;
  LorawanMacHeader::MType type;
};


typedef std::map<Ptr<Packet const>, MacPacketStatus> MacPacketData;
typedef std::multimap<Ptr<Packet const>, PacketStatus> PhyPacketData;
typedef std::map<Ptr<Packet const>, RetransmissionStatus> RetransmissionData;
typedef std::map<Ptr<Packet const>, appPacketStatus> AppPacketData;
typedef std::map<Ptr<Packet const>, NoEnergyPacketStatus> TxPacketStatusData;
typedef std::map<Ptr<Packet const>, NoEnergyPacketExpandedStatus> TxPacketStatusDataExpanded;

class LoraPacketTracker
{
public:
  LoraPacketTracker ();
  ~LoraPacketTracker ();

  /////////////////////////
  // PHY layer callbacks //
  /////////////////////////
  // Packet transmission callback
  void TransmissionCallback (Ptr<Packet const> packet, uint32_t systemId);
  // Packet outcome traces
  void PacketReceptionCallback (Ptr<Packet const> packet, uint32_t systemId);
  void InterferenceCallback (Ptr<Packet const> packet, uint32_t systemId);
  void NoMoreReceiversCallback (Ptr<Packet const> packet, uint32_t systemId);
  void UnderSensitivityCallback (Ptr<Packet const> packet, uint32_t systemId);
  void LostBecauseTxCallback (Ptr<Packet const> packet, uint32_t systemId);
  void InterruptedTransmissionCallback (Ptr<Packet const> packet);

  //Helps monitor ACS by keeping track of how many app packets are generated by confirmed nodes participating in ACS.
  //Was not yet tested for simulations in which not all confirmed nodes are part of ACS!                              
  void AppPacketGeneratedCallback (Ptr<Packet const> packet, int size, LorawanMacHeader::MType m_type);

    //See if there is enough energy for a transmission
  void EnoughEnergyToTxCallback (uint32_t nodeId, Ptr<const Packet> packet, Time time, bool boolValue,  LorawanMacHeader::MType type);

  /////////////////////////
  // MAC layer callbacks //
  /////////////////////////
  // Packet transmission at an EndDevice
  void MacTransmissionCallback (Ptr<Packet const> packet);
  void RequiredTransmissionsCallback (uint8_t reqTx, bool success, Time firstAttempt,
                                      Ptr<Packet> packet);
  // Packet reception at the Gateway
  void MacGwReceptionCallback (Ptr<Packet const> packet);

  ///////////////////////////////
  // Packet counting functions //
  ///////////////////////////////
  bool IsUplink (Ptr<Packet const> packet);

  // void CountRetransmissions (Time transient, Time simulationTime, MacPacketData
  //                            macPacketTracker, RetransmissionData reTransmissionTracker,
  //                            PhyPacketData packetTracker);

  /**
   * Count total packets received at the PHY sent of this ED, packets that have
   * been successfullt transmitted at the PHY level and packets that have been
   * interrupted
   */
  std::vector<int> CountPhyPacketsPerEd (Time startTime, Time stopTime, uint edId);

  /**
   * Count packets to evaluate the performance at PHY level of a specific
   * gateway.
   */
  std::vector<int> CountPhyPacketsPerGw (Time startTime, Time stopTime, int systemId);
  /**
   * Count packets to evaluate the performance at PHY level of a specific
   * gateway.
   */
  std::string PrintPhyPacketsPerGw (Time startTime, Time stopTime, int systemId);
  /**
   * Count packets to evaluate the performance at MAC level of a specific
   * gateway.
   */
  std::string CountMacPacketsPerGw (Time startTime, Time stopTime, int systemId);

  /**
   * Count packets to evaluate the performance at MAC level of a specific
   * gateway.
   */
  std::string PrintMacPacketsPerGw (Time startTime, Time stopTime, int systemId);

  /**
   * Count the number of retransmissions that were needed to correctly deliver a
   * packet and receive the corresponding acknowledgment.
   */
  std::string CountRetransmissions (Time startTime, Time stopTime);

  /**
   * Count packets to evaluate the global performance at MAC level of the whole
   * network. In this case, a MAC layer packet is labeled as successful if it
   * was successful at at least one of the available gateways.
   *
   */

  /**
   * Count MAC packets per ED
   */
  std::vector<uint> CountMacPacketsPerEd (Time startTime, Time stopTime, uint32_t edId);

  /**
   * This returns a string containing the number of sent packets and the number
   * of packets that were received by at least one gateway.
   */
  std::string CountMacPacketsGlobally (Time startTime, Time stopTime);

  /**
   * Count packets to evaluate the global performance at MAC level of the whole
   * network. In this case, a MAC layer packet is labeled as successful if it
   * was successful at at least one of the available gateways, and if
   * the corresponding acknowledgment was correctly delivered at the device.
   *
   * This returns a string containing the number of sent packets and the number
   * of packets that generated a successful acknowledgment.
   */
  std::string CountMacPacketsGloballyCpsr (Time startTime, Time stopTime);

  /**
   * Function returning the mean and variance of the time between PHY
   * transmissions for a given ED
   * Output: nPackets, meanT, varianceT
   */
  std::vector<double> TxTimeStatisticsPerEd (Time startTime, Time endTime, uint32_t edId);

  /**
   * Function returning the transmission time of packets that have been successfully transmitted
   */
  std::vector<double> GetTxTimes (Time startTime, Time stopTime, uint32_t edId);
  std::vector<double> GetTxTimeIntervals (Time startTime, Time stopTime, uint32_t edId);

   /**
   * Count app packets for confirmed devices to evaluate the global performance at MAC level of the whole
   * network. This function is added to account for the ACS system's ability to 
   * group packets.
   * In this case, a MAC layer packet is labeled as successful if it
   * was successful at at least one of the available gateways, and if
   * the corresponding acknowledgment was correctly delivered at the device.
   *
   * This returns a string containing the number of sent app packets and the number
   * of successfully extract and tne number with also got a successful acknowledgement.
   */
  std::string CountAppPacketsPacketsGloballyCpsr (Time startTime, Time stopTime);

  /**
   * Count app packets for unconfirmed to evaluate the global performance at MAC level of the whole
   * network. This function is added to account for the ACS system's ability to 
   * group packets.
   * In this case, a MAC layer packet is labeled as successful if it
   * was successful at at least one of the available gateways.
   *
   * This returns a string containing the number of sent app packets and the number
   * of successfully extracted.
   */
   std::string CountAppPacketsPacketsGlobally (Time startTime, Time stopTime);

  std::string DetermineEnergyResults (Time startTime, Time stopTime);
   
  std::string PrintVector (std::vector<int> vector, int returnString = 0);

  std::string PrintSumRetransmissions (std::vector<int> reTxVector, int returnString = 0);

  /**
   * A large performance print looking at PHY and MAC for a time period.
   */
  void PrintPerformance (Time start, Time stop, int gwId);

  /**
   * Check's MAC header to see if MType is confirmed data up.
   * Based off network-controller-component.c
   *
   */
  bool CheckIfUnconfirmed(Ptr<const Packet> packet);
  /**
   * CountRetransmissionsPorted() is a previous metric calculation approach used by Davide's team 
   * and was ported to work with the latest version of lorawan. 
   * Original code came out of the 2018 private version. 
   * It gives more detail and metrics than the approach used by the latest version.
   */
  std::string CountRetransmissionsPorted (Time start, Time stop, int gwId, int returnString = 0);

  std::string getPerformanceLegend();


private:
  PhyPacketData m_packetTracker;
  MacPacketData m_macPacketTracker;
  RetransmissionData m_reTransmissionTracker;
  AppPacketData m_appPacketTracker;
  TxPacketStatusData m_countNoTxTracker; 
  TxPacketStatusDataExpanded m_countNoTxExpandedTracker;
};
} // namespace lorawan
} // namespace ns3
#endif
