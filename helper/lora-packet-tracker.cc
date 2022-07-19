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

#include "lora-packet-tracker.h"
#include "ns3/log-macros-enabled.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/lorawan-mac-header.h"
#include <algorithm>
#include <bits/stdint-uintn.h>
#include <cmath>
#include <numeric>
#include <iostream>
#include <vector>
#include <fstream>
#include <string>

namespace ns3 {
namespace lorawan {
NS_LOG_COMPONENT_DEFINE ("LoraPacketTracker");

LoraPacketTracker::LoraPacketTracker ()
{
  NS_LOG_FUNCTION (this);
}

LoraPacketTracker::~LoraPacketTracker ()
{
  NS_LOG_FUNCTION (this);
}

/////////////////
// MAC metrics //
/////////////////

void
LoraPacketTracker::MacTransmissionCallback (Ptr<Packet const> packet)
{
  if (IsUplink (packet))
    {
      // NS_LOG_INFO ("A new packet was sent by the MAC layer");

      MacPacketStatus status;
      status.packet = packet;
      status.sendTime = Simulator::Now ();
      status.senderId = Simulator::GetContext ();
      status.receivedTime = Time::Max ();

      m_macPacketTracker.insert (std::pair<Ptr<Packet const>, MacPacketStatus> (packet, status));
    }
}

void
LoraPacketTracker::EnoughEnergyToTxCallback(uint32_t nodeId, Ptr<const Packet> packet, Time time, bool boolValue,  LorawanMacHeader::MType type)
{

  //NbTrans > 1 causes the same data packet to trigger this multiple times.
  //A packet is logged the first time and then only updates if it was recorded first as a 0 but later on had enough energy.

  NS_LOG_WARN("Count for " << packet << " is " << m_countNoTxTracker.count(packet));

  if(m_countNoTxExpandedTracker.count(packet) == 1)
  {
    NS_LOG_WARN("Expanded:This packet already exists (NbTrans must be > 1).");

    for (auto it = m_countNoTxExpandedTracker.begin ();
        it != m_countNoTxExpandedTracker.end ();
        ++it)
    {
      if((*it).first == packet)
      {
        (*it).second.successValues[(*it).second.count] = boolValue;
        (*it).second.count = (*it).second.count + 1;
        NS_LOG_WARN("Expanded: Updating to indicate what happened");
      }
    }
  }
  else
  {
 
    NS_LOG_WARN("Expanded: First instance of packet, enough energy: " << boolValue);
    NoEnergyPacketExpandedStatus packetStatus;
    packetStatus.successValues[0] = boolValue;
    packetStatus.device = nodeId;
    packetStatus.count = 1;
    packetStatus.type = type;
    packetStatus.generatedTime = Simulator::Now();
    m_countNoTxExpandedTracker.insert(std::pair<Ptr<Packet const>, NoEnergyPacketExpandedStatus> (packet,packetStatus));
    //insert is used, so only the first instance for a packet will be recorded     

  }

  if(boolValue == 0 && m_countNoTxTracker.count(packet) == 1)
  {
    NS_LOG_WARN ("Not enough E for transmission from D" << (unsigned) nodeId << " " << packet);
    NS_LOG_WARN("This packet already exists (NbTrans must be > 1).");
  }

  else if(boolValue == 1 && m_countNoTxTracker.count(packet) == 1)
  {
    NS_LOG_WARN("This packet already exists (NbTrans must be > 1).");
   
    for (auto it = m_countNoTxTracker.begin ();
        it != m_countNoTxTracker.end ();
        ++it)
    {
      if((*it).first == packet && (*it).second.success==false)
      {
        (*it).second.success = boolValue;
        NS_LOG_WARN("Updating to indicate success");
      }
    }
  }
  else
  {
    NS_LOG_WARN("First instance of packet, enough energy: " << boolValue);
    NoEnergyPacketStatus packetStatus;
    packetStatus.success = boolValue;
    packetStatus.device = nodeId;
    packetStatus.type = type;
    packetStatus.generatedTime = Simulator::Now();
    m_countNoTxTracker.insert(std::pair<Ptr<Packet const>, NoEnergyPacketStatus> (packet,packetStatus));
    //insert is used, so only the first instance for a packet will be recorded
  }
}


void
LoraPacketTracker::RequiredTransmissionsCallback (uint8_t reqTx, bool success, Time firstAttempt,
                                                  Ptr<Packet> packet)
{
  // NS_LOG_INFO ("Finished retransmission attempts for a packet");
  // NS_LOG_DEBUG ("Packet: " << packet << "ReqTx " << unsigned (reqTx) << ", succ: " << success
  //                          << ", firstAttempt: " << firstAttempt.GetSeconds ());

  RetransmissionStatus entry;
  entry.firstAttempt = firstAttempt;
  entry.finishTime = Simulator::Now ();
  entry.reTxAttempts = reqTx;
  entry.successful = success;

  m_reTransmissionTracker.insert (std::pair<Ptr<Packet>, RetransmissionStatus> (packet, entry));
}

void
LoraPacketTracker::MacGwReceptionCallback (Ptr<Packet const> packet)
{
  if (IsUplink (packet))
    {
      // NS_LOG_INFO ("A packet was successfully received"
      //              << " at the MAC layer of gateway " << Simulator::GetContext ());

      // Find the received packet in the m_macPacketTracker
      auto it = m_macPacketTracker.find (packet);
      if (it != m_macPacketTracker.end ())
        {
          (*it).second.receptionTimes.insert (
              std::pair<int, Time> (Simulator::GetContext (), Simulator::Now ()));
        }
      else
        {
          NS_ABORT_MSG ("Packet not found in tracker");
        }
    }
}

/////////////////
// PHY metrics //
/////////////////

void
LoraPacketTracker::TransmissionCallback (Ptr<Packet const> packet, uint32_t edId)
{
  if (IsUplink (packet))
    {
      // NS_LOG_INFO ("PHY packet " << packet
      //                            << " was transmitted by device "
      //                            << edId);
      // Create a packetStatus
      PacketStatus status;
      status.packet = packet;
      status.sendTime = Simulator::Now ();
      status.senderId = edId;
      status.txSuccessful = true;

      m_packetTracker.insert (std::pair<Ptr<Packet const>, PacketStatus> (packet, status));
      // NS_LOG_DEBUG ("Inserted PHY packet");
    }
}


void
LoraPacketTracker::AppPacketGeneratedCallback(Ptr<Packet const> packet, int size, LorawanMacHeader::MType m_type)
{
  NS_LOG_INFO ("An ACS enabled device generated an appPacket of size: " << size << " and type " << m_type);
  appPacketStatus packetStatus;
  packetStatus.size = size;
  packetStatus.mtype = m_type;
  packetStatus.generatedTime = Simulator::Now();
  m_appPacketTracker.insert(std::pair<Ptr<Packet const>, appPacketStatus> (packet,packetStatus));
}

void
LoraPacketTracker::PacketReceptionCallback (Ptr<Packet const> packet, uint32_t gwId)
{
  if (IsUplink (packet))
    {
      // Remove the successfully received packet from the list of sent ones
      NS_LOG_INFO ("PHY packet " << packet
                                 << " was successfully received at gateway "
                                 << gwId);


      if(m_packetTracker.count(packet) == 1) //first reception
        {
          std::map<Ptr<Packet const>, PacketStatus>::iterator it = m_packetTracker.find (packet);
          (*it).second.outcomes.insert (std::pair<int, enum PhyPacketOutcome> (gwId,
                                                                           RECEIVED));
        }
      else
        {
          std::pair <std::multimap<Ptr<Packet const>, PacketStatus>::iterator, std::multimap<Ptr<Packet const>, PacketStatus>::iterator> ret;
          ret = m_packetTracker.equal_range(packet); // find all instances of received packet
          
          int i = 1;

          //loop through all instances and find the one which doesn't yet have an outcome at this gateway
          for(std::multimap<Ptr<Packet const>, PacketStatus>::iterator it=ret.first; it != ret.second; ++it)
            {
                    
              if((*it).second.outcomes.find(gwId) == (*it).second.outcomes.end()) 
              {
                NS_LOG_DEBUG("This is copy " << i <<" of packet "<< packet);                
                NS_LOG_DEBUG("This packet was sent at " << (*it).second.sendTime);
                NS_LOG_DEBUG("It was not yet received by GW " << gwId << " logging it now as RECEIVED.");
                (*it).second.outcomes.insert(std::pair<int, enum PhyPacketOutcome> (gwId,
                                                                           RECEIVED)); 
              }
              i++;           
            }
        }
    }
}

void
LoraPacketTracker::InterferenceCallback (Ptr<Packet const> packet, uint32_t gwId)
{
  if (IsUplink (packet))
    {
      NS_LOG_INFO ("PHY packet " << packet
                                 << " was interfered at gateway "
                                 << gwId);

      if(m_packetTracker.count(packet) == 1) //first reception
        {
          std::map<Ptr<Packet const>, PacketStatus>::iterator it = m_packetTracker.find (packet);
          (*it).second.outcomes.insert (std::pair<int, enum PhyPacketOutcome> (gwId,
                                                                           INTERFERED)); 
        }
      else
        {
          std::pair <std::multimap<Ptr<Packet const>, PacketStatus>::iterator, std::multimap<Ptr<Packet const>, PacketStatus>::iterator> ret;
          ret = m_packetTracker.equal_range(packet); // find all instances of received packet
          
          int i = 1;

          //loop through all instances and find the one which doesn't yet have an outcome at this gateway
          for(std::multimap<Ptr<Packet const>, PacketStatus>::iterator it=ret.first; it != ret.second; ++it)
            {
                    
              if((*it).second.outcomes.find(gwId) == (*it).second.outcomes.end()) 
              {
                NS_LOG_INFO("This is copy " << i <<" of packet "<< packet);                
                NS_LOG_INFO("This packet was sent at " << (*it).second.sendTime);
                NS_LOG_INFO("It was not yet received by GW " << gwId << " logging it now as INTERFERED.");
                (*it).second.outcomes.insert(std::pair<int, enum PhyPacketOutcome> (gwId,
                                                                           INTERFERED)); 
              }
              i++;           
            }
        }
    }
}

void
LoraPacketTracker::NoMoreReceiversCallback (Ptr<Packet const> packet, uint32_t gwId)
{
  if (IsUplink (packet))
    {
      NS_LOG_INFO ("PHY packet " << packet
                                 << " was lost because no more receivers at gateway "
                                 << gwId);

      if(m_packetTracker.count(packet) == 1) //first reception
        {
          std::map<Ptr<Packet const>, PacketStatus>::iterator it = m_packetTracker.find (packet);
          (*it).second.outcomes.insert (std::pair<int, enum PhyPacketOutcome> (gwId,
                                                                           NO_MORE_RECEIVERS)); 
        }
      else
        {
          std::pair <std::multimap<Ptr<Packet const>, PacketStatus>::iterator, std::multimap<Ptr<Packet const>, PacketStatus>::iterator> ret;
          ret = m_packetTracker.equal_range(packet); // find all instances of received packet
          
          int i = 1;

          //loop through all instances and find the one which doesn't yet have an outcome at this gateway
          for(std::multimap<Ptr<Packet const>, PacketStatus>::iterator it=ret.first; it != ret.second; ++it)
            {
                    
              if((*it).second.outcomes.find(gwId) == (*it).second.outcomes.end()) 
              {
                NS_LOG_INFO("This is copy " << i <<" of packet "<< packet);                
                NS_LOG_INFO("This packet was sent at " << (*it).second.sendTime);
                NS_LOG_INFO("It was not yet received by GW " << gwId << " logging it now as NO_MORE_RECEIVERS.");
                (*it).second.outcomes.insert(std::pair<int, enum PhyPacketOutcome> (gwId,
                                                                           NO_MORE_RECEIVERS)); 
              }
              i++;           
            }
        }                                                                     
    }
}
void
LoraPacketTracker::UnderSensitivityCallback (Ptr<Packet const> packet, uint32_t gwId)
{
  if (IsUplink (packet))
    {
      NS_LOG_INFO ("PHY packet " << packet
                                 << " was lost because under sensitivity at gateway "
                                 << gwId);

      if(m_packetTracker.count(packet) == 1) //first reception
        {
          std::map<Ptr<Packet const>, PacketStatus>::iterator it = m_packetTracker.find (packet);
          (*it).second.outcomes.insert (std::pair<int, enum PhyPacketOutcome> (gwId,
                                                                           UNDER_SENSITIVITY)); 
        }
      else
        {
          std::pair <std::multimap<Ptr<Packet const>, PacketStatus>::iterator, std::multimap<Ptr<Packet const>, PacketStatus>::iterator> ret;
          ret = m_packetTracker.equal_range(packet); // find all instances of received packet
          
          int i = 1;

          //loop through all instances and find the one which doesn't yet have an outcome at this gateway
          for(std::multimap<Ptr<Packet const>, PacketStatus>::iterator it=ret.first; it != ret.second; ++it)
            {
                    
              if((*it).second.outcomes.find(gwId) == (*it).second.outcomes.end()) 
              {
                NS_LOG_INFO("This is copy " << i <<" of packet "<< packet);                
                NS_LOG_INFO("This packet was sent at " << (*it).second.sendTime);
                NS_LOG_INFO("It was not yet received by GW " << gwId << " logging it now as UNDER_SENSITIVITY.");
                (*it).second.outcomes.insert(std::pair<int, enum PhyPacketOutcome> (gwId,
                                                                           UNDER_SENSITIVITY)); 
              }
              i++;           
            }
        }    
    }
}

void
LoraPacketTracker::LostBecauseTxCallback (Ptr<Packet const> packet, uint32_t gwId)
{
  if (IsUplink (packet))
    {
      NS_LOG_INFO ("PHY packet " << packet
                                 << " was lost because of GW transmission at gateway "
                                 << gwId);

      if(m_packetTracker.count(packet) == 1) //first reception
        {
          std::map<Ptr<Packet const>, PacketStatus>::iterator it = m_packetTracker.find (packet);
          (*it).second.outcomes.insert (std::pair<int, enum PhyPacketOutcome> (gwId,
                                                                           LOST_BECAUSE_TX)); 
        }
      else
        {
          std::pair <std::multimap<Ptr<Packet const>, PacketStatus>::iterator, std::multimap<Ptr<Packet const>, PacketStatus>::iterator> ret;
          ret = m_packetTracker.equal_range(packet); // find all instances of received packet
          
          int i = 1;

          //loop through all instances and find the one which doesn't yet have an outcome at this gateway
          for(std::multimap<Ptr<Packet const>, PacketStatus>::iterator it=ret.first; it != ret.second; ++it)
            {
                    
              if((*it).second.outcomes.find(gwId) == (*it).second.outcomes.end()) 
              {
                NS_LOG_INFO("This is copy " << i <<" of packet "<< packet);                
                NS_LOG_INFO("This packet was sent at " << (*it).second.sendTime);
                NS_LOG_INFO("It was not yet received by GW " << gwId << " logging it now as LOST_BECAUSE_TX.");
                (*it).second.outcomes.insert(std::pair<int, enum PhyPacketOutcome> (gwId,
                                                                           LOST_BECAUSE_TX)); 
              }
              i++;           
            }
        }    
    }
}

void
LoraPacketTracker::InterruptedTransmissionCallback (Ptr<Packet const> packet)
{
  if (IsUplink (packet))
    {
      NS_LOG_INFO ("PHY packet " << packet << " interrupted");

      if(m_packetTracker.count(packet) == 1) //first reception
      {
        std::map<Ptr<Packet const>, PacketStatus>::iterator it = m_packetTracker.find (packet);
        (*it).second.txSuccessful = false;
      }
      else
        NS_LOG_ERROR("More than one interrupted found");

    }
}

bool
LoraPacketTracker::IsUplink (Ptr<Packet const> packet)
{
  // NS_LOG_FUNCTION (this);

  LorawanMacHeader mHdr;
  Ptr<Packet> copy = packet->Copy ();
  copy->RemoveHeader (mHdr);
  return mHdr.IsUplink ();
}

////////////////////////
// Counting Functions //
////////////////////////

// TODO Count interrupted packets
std::vector<int>
LoraPacketTracker::CountPhyPacketsPerEd (Time startTime, Time stopTime, uint edId)
{
  std::vector<int> packetCounts (3, 0);

  for (auto itPhy = m_packetTracker.begin (); itPhy != m_packetTracker.end (); ++itPhy)
    {
      if ((*itPhy).second.sendTime >= startTime && (*itPhy).second.sendTime <= stopTime)
        {
          // NS_LOG_DEBUG ("Dealing with packet " << (*itPhy).second.packet);
          if ((*itPhy).second.senderId == edId)
            {
              packetCounts.at (0)++;
              if ((*itPhy).second.txSuccessful == true)
                {
                  packetCounts.at (1)++;
                }
              else
                {
                  packetCounts.at (2)++;
                  NS_LOG_DEBUG ("This packet transmission was interrupted at the PHY level");
                }
            }
        }
    }
  return packetCounts;
}

std::vector<int>
LoraPacketTracker::CountPhyPacketsPerGw (Time startTime, Time stopTime, int gwId)
{
  // Vector packetCounts will contain - for the interval given in the input of
  // the function, the following fields: totPacketsSent receivedPackets
  // interferedPackets noMoreGwPackets underSensitivityPackets lostBecauseTxPackets

  std::vector<int> packetCounts (6, 0);

  for (auto itPhy = m_packetTracker.begin ();
       itPhy != m_packetTracker.end ();
       ++itPhy)
    {
      if ((*itPhy).second.sendTime >= startTime && (*itPhy).second.sendTime <= stopTime)
        {
          // NS_LOG_DEBUG ("Dealing with packet " << (*itPhy).second.packet);
          if ((*itPhy).second.txSuccessful == false)
            {
              NS_LOG_DEBUG ("This packet transmission was interrupted at the PHY level");
              // Do nothing and go to next packet
            }
          else
            {
              packetCounts.at (0)++;

              NS_LOG_DEBUG ("Dealing with packet " << (*itPhy).second.packet);
              NS_LOG_DEBUG ("This packet was received by " <<
                            (*itPhy).second.outcomes.size () << " gateways");

              if ((*itPhy).second.outcomes.count (gwId) > 0)
                {
                  switch ((*itPhy).second.outcomes.at (gwId))
                    {
                      case RECEIVED: {
                        packetCounts.at (1)++;
                        break;
                      }
                      case INTERFERED: {
                        packetCounts.at (2)++;
                        break;
                      }
                      case NO_MORE_RECEIVERS: {
                        packetCounts.at (3)++;
                        break;
                      }
                      case UNDER_SENSITIVITY: {
                        packetCounts.at (4)++;
                        break;
                      }
                      case LOST_BECAUSE_TX: {
                        packetCounts.at (5)++;
                        break;
                      }
                      case UNSET: {
                        break;
                      }
                    }
                }
            }

        } // time
    } // iterator

  return packetCounts;
}
std::string
LoraPacketTracker::PrintPhyPacketsPerGw (Time startTime, Time stopTime,
                                         int gwId)
{
  // Vector packetCounts will contain - for the interval given in the input of
  // the function, the following fields: totPacketsSent receivedPackets
  // interferedPackets noMoreGwPackets underSensitivityPackets lostBecauseTxPackets

  std::vector<int> packetCounts = CountPhyPacketsPerGw(startTime, stopTime, gwId);
  std::string output ("");
  for (int i = 0; i < 6; ++i)
    {
      output += std::to_string (packetCounts.at (i)) + " ";
    }

  return output;
}

std::vector<uint>
LoraPacketTracker::CountMacPacketsPerEd (Time startTime, Time stopTime, uint32_t edId)
{
  NS_LOG_FUNCTION (this << startTime << stopTime);

  std::vector<uint> v (2, 0);
  for (auto it = m_macPacketTracker.begin (); it != m_macPacketTracker.end (); ++it)
    {
      if ((*it).second.sendTime >= startTime && (*it).second.sendTime <= stopTime)
        {
          if ((*it).second.senderId == edId)
            {
              v.at (0)++;
              if ((*it).second.receptionTimes.size ())
                {
                  v.at (1)++;
                }
            }
        }
    }

  return v;
}

  std::string
  LoraPacketTracker::CountMacPacketsGlobally (Time startTime, Time stopTime)
  {
    // NS_LOG_FUNCTION (this << startTime << stopTime);

    double sent = 0;
    double received = 0;
    for (auto it = m_macPacketTracker.begin ();
         it != m_macPacketTracker.end ();
         ++it)
      {
        if ((*it).second.sendTime >= startTime && (*it).second.sendTime <= stopTime)
          {
            sent++;
            if ((*it).second.receptionTimes.size ())
              {
                received++;
              }
          }
      }

    return std::to_string (sent) + " " +
      std::to_string (received);
  }

  std::string
  LoraPacketTracker::CountMacPacketsGloballyCpsr (Time startTime, Time stopTime)
  {
    NS_LOG_FUNCTION (this << startTime << stopTime);

    double sent = 0;
    double received = 0;
    for (auto it = m_reTransmissionTracker.begin ();
         it != m_reTransmissionTracker.end ();
         ++it)
      {
        if ((*it).second.firstAttempt >= startTime && (*it).second.firstAttempt <= stopTime)
          {
            sent++;
            // NS_LOG_DEBUG ("Found a packet");
            // NS_LOG_DEBUG ("Number of attempts: " << unsigned(it->second.reTxAttempts) <<
            //               ", successful: " << it->second.successful);
            if (it->second.successful)
              {
                received++;
              }
          }
      }

    return std::to_string (sent) + " " +
      std::to_string (received);
  }

std::string
LoraPacketTracker::CountAppPacketsPacketsGlobally (Time startTime, Time stopTime)
  {

    //This is currently not perfect, it does not test to see if groupedP is in use.
    // 
    NS_LOG_FUNCTION (this << startTime << stopTime);

    int sent = 0;
    int sent_check = 0;
    int successfullyExtractedAppPackets = 0; //grouped packets 
    int generatedAppPackets = 0;

    for (auto it = m_appPacketTracker.begin ();
        it != m_appPacketTracker.end ();
        ++it)
    {
      
      if ((*it).second.generatedTime >= startTime && (*it).second.generatedTime <= stopTime && (*it).second.mtype == LorawanMacHeader::MType::UNCONFIRMED_DATA_UP)
        {
          NS_LOG_DEBUG("This was a unconfirmed packet");
          generatedAppPackets++;
        }
    }
  
    for (auto it = m_macPacketTracker.begin ();
        it != m_macPacketTracker.end ();
        ++it)
    {
      if ((*it).second.sendTime >= startTime && (*it).second.sendTime <= stopTime)
        {
          if((*it).first != 0) //Check if Packet isn't 0. 
          //RequiredTransmissionsCallback() frequently has a Packet: 0 call
          //That should be gone with NbTrans code update 
          {

            auto itRetx = m_reTransmissionTracker.find ((*it).first);
            if (itRetx == m_reTransmissionTracker.end() && CheckIfUnconfirmed((*it).first))
            {

              sent += 1; //(*it).second.numGroupedPackets
              sent_check++;


              if (it->second.receptionTimes.size()) //if it was received at a GW
                {
                  successfullyExtractedAppPackets += 1;                
                }
            }
          }
        }
    }

    NS_LOG_DEBUG(this << " processed " << sent_check << " packets");
    return std::to_string(generatedAppPackets) + " " + std::to_string (sent) + " " +
      std::to_string (successfullyExtractedAppPackets);
  }

std::string
LoraPacketTracker::DetermineEnergyResults (Time startTime, Time stopTime)
  {
    NS_LOG_WARN (startTime.GetSeconds() << " " << stopTime.GetSeconds());
    NS_LOG_WARN("m_countNoTxTracker contains " << m_countNoTxTracker.size());

    int generated = 0;
    int interrupted = 0;
    int noEnergy = 0; 
    int noEnergyExpanded=0;

    int noEnergyExpandedUnconf=0;
    int noEnergyExpandedConf=0;

    for (auto it = m_appPacketTracker.begin ();
        it != m_appPacketTracker.end ();
        ++it)
    {
      
      if ((*it).second.generatedTime >= startTime && (*it).second.generatedTime <= stopTime)
        {
          //NS_LOG_WARN("This was a generated packet");
          generated++;
        }
    }
  
    NS_LOG_WARN("Generated in this period were " << generated << " packets");

    for (auto it = m_countNoTxTracker.begin ();
        it != m_countNoTxTracker.end ();
        ++it)
    {
      if((*it).second.device == 159)
        NS_LOG_WARN("device " << (unsigned) (*it).second.device << " " << (*it).second.success << " " <<(*it).second.generatedTime.GetSeconds());

      if ((*it).second.generatedTime >= startTime  && (*it).second.generatedTime <= stopTime && (*it).second.success==false)
        {

          Ptr<Packet> myPac = (*it).first->Copy();
          LorawanMacHeader macHdr;
          LoraFrameHeader frameHdr;
          myPac->RemoveHeader (macHdr);
          myPac->RemoveHeader (frameHdr);    
 
          NS_LOG_WARN("This packet contains " << (unsigned) 1 << " app packet(s)."); //starts at 0. 0 = 1 packet
          noEnergy = noEnergy + 1; 
          NS_LOG_WARN("This packet could not be sent due to no energy");
         // noEnergy++;
        }
          
        
    }

    //Packets are inserted "twice" if there is not enough Energy. First insertion is to indicate how many were lost and the second to indicate how many were sucessfully sent.
    for (auto it = m_countNoTxExpandedTracker.begin ();
        it != m_countNoTxExpandedTracker.end ();
        ++it)
    {

      if ((*it).second.generatedTime >= startTime  && (*it).second.generatedTime <= stopTime)
        {

          Ptr<Packet> myPac = (*it).first->Copy();
          LorawanMacHeader macHdr;
          LoraFrameHeader frameHdr;
          myPac->RemoveHeader (macHdr);
          myPac->RemoveHeader (frameHdr);    
 
          NS_LOG_WARN("Expanded: this packet contains " << (unsigned) 1 << " app packet(s). Generated at " << (*it).second.generatedTime.GetSeconds()); //starts at 0. 0 = 1 packet
          NS_LOG_WARN("Expanded:There were " << ((*it).second.count) <<  " sending attempts");
         for (int i=0; i < (*it).second.count; i++)
         {
            if((*it).second.successValues[i] == false)
            {
              NS_LOG_WARN("Message type= " << (*it).second.type);
              noEnergyExpanded = noEnergyExpanded + 1; 
              NS_LOG_WARN("Expanded: this packet could not be sent due to no energy, from " << (unsigned) (*it).second.device);


              if ( (int) (*it).second.type == LorawanMacHeader::MType::UNCONFIRMED_DATA_UP)
                noEnergyExpandedUnconf = noEnergyExpandedUnconf + 1; 
              else if( (int) (*it).second.type == LorawanMacHeader::MType::CONFIRMED_DATA_UP)
              noEnergyExpandedConf = noEnergyExpandedConf + 1; 
            }

         }
          
         // noEnergy++;
        }
          
        
    }

    for (auto it = m_packetTracker.begin ();
        it != m_packetTracker.end ();
        ++it)
    {
      
      if ((*it).second.sendTime >= startTime && (*it).second.sendTime <= stopTime && (*it).second.txSuccessful==false)
        {
          NS_LOG_WARN("This was an interrupted packet");
          interrupted++;
        }
    } 


    return std::to_string(generated) + " " + std::to_string (interrupted) + " " +
      std::to_string (noEnergy) + " " + std::to_string(noEnergyExpanded) + " " + std::to_string(noEnergyExpandedUnconf) + " " + std::to_string(noEnergyExpandedConf);
  }

std::string
LoraPacketTracker::CountAppPacketsPacketsGloballyCpsr (Time startTime, Time stopTime)
{
  NS_LOG_FUNCTION (this << startTime << stopTime);

  int sent = 0;
  int sent_check = 0;
  int successfullyExtractedAppPackets = 0; //grouped packets (not looking if they were ACKed)
  int successfullyExtractedAckedAppPackets = 0; //looking at if they were ACKed as well

  int generatedAppPackets = 0;


  for (auto it = m_appPacketTracker.begin ();
        it != m_appPacketTracker.end ();
        ++it)
    {
      if ((*it).second.generatedTime >= startTime && (*it).second.generatedTime <= stopTime && (*it).second.mtype == LorawanMacHeader::MType::CONFIRMED_DATA_UP)
        {
          generatedAppPackets++;
        }
    }

  for (auto it = m_reTransmissionTracker.begin ();
        it != m_reTransmissionTracker.end ();
        ++it)
    {
      if ((*it).second.firstAttempt >= startTime && (*it).second.firstAttempt <= stopTime)
        {

          if((*it).first != 0) //Check if Packet isn't 0. 
          //RequiredTransmissionsCallback() frequently has a Packet: 0 call
          {
            //sent++;


            // m_reTransmissionTracker only contains confirmed packets, so don't need to worry about unconfirmed packets messing up calcs.

            sent += 1;
            sent_check++;



            if (it->second.successful)
              {
                successfullyExtractedAppPackets += 1;
                successfullyExtractedAckedAppPackets += 1;                  
              }
            else
            {
              auto itMac = m_macPacketTracker.find ((*it).first);
              if ((*itMac).second.receptionTimes.size() != 0)
                successfullyExtractedAppPackets += 1;
            }
            
            }
        }
    }

  NS_LOG_DEBUG(this << " processed " << sent_check << " packets");
  return std::to_string(generatedAppPackets) + " " + std::to_string (sent) + " " +
    std::to_string (successfullyExtractedAppPackets) + " " + std::to_string(successfullyExtractedAckedAppPackets);
} 

std::vector<double>
LoraPacketTracker::TxTimeStatisticsPerEd (Time startTime, Time stopTime,
                                          uint32_t edId)
{
  NS_LOG_FUNCTION (this);

  std::vector<double> outputTx (3, 0); // nPackets, meanT, varianceT
  std::vector<double> txTime;
  std::vector<double> txTimeIntervals;
  double meanTxInterval = 0;
  double tmpVariance = 0;

  for (auto itPhy = m_packetTracker.begin (); itPhy != m_packetTracker.end (); ++itPhy)
    {
      if ((*itPhy).second.sendTime >= startTime && (*itPhy).second.sendTime <= stopTime)
        {
          if ((*itPhy).second.senderId == edId)
            {
              // NS_LOG_DEBUG ("Dealing with packet " << (*itPhy).second.packet);
              if ((*itPhy).second.txSuccessful == false)
                {
                  // NS_LOG_DEBUG ("This packet transmission was interrupted at the PHY level");
                  // Do nothing and go to next packet
                }
              else
                {
                  outputTx.at (0)++;

                  // NS_LOG_DEBUG ("This packet was sent at time "
                  //               << (*itPhy).second.sendTime.GetSeconds ());
                  txTime.push_back ((*itPhy).second.sendTime.GetSeconds ());
                }
            }
        }
    }
  std::sort (txTime.begin (), txTime.end ());
  for (uint i = 1; i < txTime.size (); i++)
    {
      double interval = txTime.at (i) - txTime.at (i - 1);
      txTimeIntervals.push_back (interval);
      // NS_LOG_DEBUG ("Interval... " << interval);
    }
  // NS_LOG_DEBUG ("TxTime size= " << txTime.size () << " txTimeIntervals size "
  //                               << txTimeIntervals.size ());
  double tmpMean = 0;
  // Compute variance
  for (uint i = 1; i < txTimeIntervals.size (); i++)
    {
      tmpMean = tmpMean + txTimeIntervals.at (i);
    }
        meanTxInterval = tmpMean/txTimeIntervals.size();
        NS_LOG_DEBUG ("Tmp Mean " << tmpMean);
        NS_LOG_DEBUG ("Mean interval " << meanTxInterval);

        // Compute variance
        for (uint i = 0; i < txTimeIntervals.size (); i++)
          {
            tmpVariance = tmpVariance + std::pow ((txTimeIntervals.at (i) - meanTxInterval), 2);
          }
  outputTx.at (1) = meanTxInterval;
  outputTx.at (2) = std::sqrt (tmpVariance / txTimeIntervals.size ());

  return outputTx;
}


std::vector<double>
LoraPacketTracker::GetTxTimes (Time startTime, Time stopTime,
                                          uint32_t edId)
{
  NS_LOG_FUNCTION (this);

  std::vector<double> txTimes;

  for (auto itPhy = m_packetTracker.begin (); itPhy != m_packetTracker.end (); ++itPhy)
    {
      if ((*itPhy).second.sendTime >= startTime && (*itPhy).second.sendTime <= stopTime)
        {
          if ((*itPhy).second.senderId == edId)
            {
              // NS_LOG_DEBUG ("Dealing with packet " << (*itPhy).second.packet);
              if ((*itPhy).second.txSuccessful == false)
                {
                  // NS_LOG_DEBUG ("This packet transmission was interrupted at the PHY level");
                  // Do nothing and go to next packet
                }
              else
                {
                  // NS_LOG_DEBUG ("This packet was sent at time "
                  //               << (*itPhy).second.sendTime.GetSeconds ());
                  txTimes.push_back ((*itPhy).second.sendTime.GetSeconds ());
                }
            }
        }
    }
  std::sort (txTimes.begin (), txTimes.end ());
  return txTimes;
}

std::vector<double>
LoraPacketTracker::GetTxTimeIntervals (Time startTime, Time stopTime,
                                          uint32_t edId)
{
  NS_LOG_FUNCTION (this);

  std::vector<double> txTime;
  std::vector<double> txTimeIntervals;

  for (auto itPhy = m_packetTracker.begin (); itPhy != m_packetTracker.end (); ++itPhy)
    {
      if ((*itPhy).second.sendTime >= startTime && (*itPhy).second.sendTime <= stopTime)
        {
          if ((*itPhy).second.senderId == edId)
            {
              // NS_LOG_DEBUG ("Dealing with packet " << (*itPhy).second.packet);
              if ((*itPhy).second.txSuccessful == false)
                {
                  // NS_LOG_DEBUG ("This packet transmission was interrupted at the PHY level");
                  // Do nothing and go to next packet
                }
              else
                {
                  // NS_LOG_DEBUG ("This packet was sent at time "
                  //               << (*itPhy).second.sendTime.GetSeconds ());
                  txTime.push_back ((*itPhy).second.sendTime.GetSeconds ());
                }
            }
        }
    }
  std::sort (txTime.begin (), txTime.end ());
  for (uint i = 1; i < txTime.size (); i++)
    {
      double interval = txTime.at (i) - txTime.at (i - 1);
      txTimeIntervals.push_back (interval);
      // NS_LOG_DEBUG ("Interval... " << interval);
    }
  return txTimeIntervals;
}

void
LoraPacketTracker::PrintPerformance (Time start, Time stop, int gwId)
{
  NS_LOG_FUNCTION (this);

  // Statistics ignoring transient
  CountRetransmissionsPorted (start, stop, gwId,0);
}

std::string
LoraPacketTracker::getPerformanceLegend()
{
    return   "Total unconfirmed Successful unconfirmed | Successfully extracted confirmed packets Successfully ACKed confirmed packets | Incomplete Confirmed | Successful with 1 Successful with 2 Successful with 3 Successful with 4 Successful with 5 Successful with 6 Successful with 7 Successful with 8 | Failed after 1 Failed after 2 Failed after 3 Failed after 4 Failed after 5 Failed after 6 Failed after 7 Failed after 8 | Average Delay Average ACK Delay | Total Retransmission amounts || PHY Total PHY Successful PHY Interfered PHY No More Receivers PHY Under Sensitivity PHY Lost Because TX ** CPSR confirmed sent CPSR confirmed ACKed\n";
}

std::string
  LoraPacketTracker::CountRetransmissionsPorted (Time startTime, Time stopTime, int gwId, int returnString)
  {

    NS_LOG_WARN(this << startTime << stopTime);
    std::vector<int> totalReTxAmounts (8, 0);
    std::vector<int> successfulReTxAmounts (8, 0);
    std::vector<int> failedReTxAmounts (8, 0);
    Time delaySum = Seconds (0);
    Time ackDelaySum = Seconds(0);

    int confirmedPacketsOutsideTransient = 0;
    int confirmedMACpacketsOutsideTransient = 0;
    int successfullyExtractedConfirmedPackets = 0;

    int successfulUnconfirmedPackets = 0;
    int incompleteConfirmedPackets = 0;
    int totalUnconfirmedPackets = 0;
    int successfullyAckedConfirmedPackets = 0;

    std::string returnValue="";
  
    for (auto itMac = m_macPacketTracker.begin (); itMac != m_macPacketTracker.end(); ++itMac)
        {

          if ((*itMac).second.sendTime >= startTime && (*itMac).second.sendTime <= stopTime)
            {
              NS_LOG_DEBUG(" ");
              NS_LOG_DEBUG ("Dealing with packet " << (*itMac).first);
              NS_LOG_DEBUG("sendTime " << (*itMac).second.sendTime.GetSeconds());
              NS_LOG_DEBUG("senderId " << (*itMac).second.senderId);
              if((*itMac).second.receptionTimes.size() >0)
                NS_LOG_DEBUG("receptionTimes " << (*itMac).second.receptionTimes.at(gwId).GetSeconds());
              else
              {
                  NS_LOG_DEBUG("Packet sent but never received " << " stopTime was " << stopTime.GetSeconds());
              }
              

              
              // Count retransmissions
              ////////////////////////
              auto itRetx = m_reTransmissionTracker.find ((*itMac).first);
        
              if (itRetx == m_reTransmissionTracker.end() && CheckIfUnconfirmed((*itMac).first))
                {
                  NS_LOG_DEBUG("Packet was a unconfirmed packet");
                  totalUnconfirmedPackets++;

 //                 if ((*itMac).second.receivedTime != Time::Max ()) // Received correctly
                  if ((*itMac).second.receptionTimes.size() != 0) // Received correctly
                    {
                      NS_LOG_DEBUG ("Unconfirmed packet was received");
                      successfulUnconfirmedPackets++;
                      delaySum += ((*itMac).second.receptionTimes.at(gwId)) - (*itMac).second.sendTime;
                    }
                  else
                  {
                         NS_LOG_DEBUG("Unconfirmed packet sent but never received");
                  }
                  
                  // NS_ABORT_MSG ("Searched packet was not found" << "Packet " <<
                  //               (*itMac).first << " not found. Sent at " <<
                  //               (*itMac).second.sendTime.GetSeconds());
                }
              else if(itRetx == m_reTransmissionTracker.end())
                {
                     NS_LOG_DEBUG("Confirmed packet sent but not yet logged in reTransmissionTracker.");
                     NS_LOG_DEBUG("ACK was not yet received/max retrans attempts not yet reached.");
                     incompleteConfirmedPackets++;
                }
              else 
                {
                  NS_LOG_DEBUG("Packet was a confirmed packet");
                  confirmedPacketsOutsideTransient++;
                  confirmedMACpacketsOutsideTransient++;

                  totalReTxAmounts.at ((*itRetx).second.reTxAttempts - 1)++;

                  if ((*itRetx).second.successful)
                    {
                      successfulReTxAmounts.at ((*itRetx).second.reTxAttempts - 1)++;
                      // If this packet was successful at the ED, it means that it
                      // was also received at the GW
                      successfullyExtractedConfirmedPackets++;
                     // successfullyExtractedAppPackets += (*itRetx).second.numGroupedPackets;
                      
                     // successfullyExtractedAckedAppPackets += (*itRetx).second.numGroupedPackets;

                      //Recording to say ED got ACK
                      successfullyAckedConfirmedPackets++;
                    }
                  else
                    {
                      failedReTxAmounts.at ((*itRetx).second.reTxAttempts - 1)++;
                      // Check if, despite failing to get an ACK at the ED, this
                      // packet was received at the GW
                      if ((*itMac).second.receptionTimes.size() != 0)
                        {
                          successfullyExtractedConfirmedPackets++;
                          //successfullyExtractedAppPackets += (*itRetx).second.numGroupedPackets;
                        }
                    }

                  // Compute delays
                  /////////////////
                  if ((*itMac).second.receptionTimes.size() == 0)
                    {
                       NS_LOG_DEBUG ("Confirmed Packet never received, ignoring it");
                      confirmedPacketsOutsideTransient--;
                    }
                  else
                    {
                      delaySum += (*itMac).second.receptionTimes.at(gwId) - (*itMac).second.sendTime;
                      ackDelaySum += (*itRetx).second.finishTime - (*itRetx).second.firstAttempt;
                    }
                }
            }
        }

    // Sum PHY outcomes

    std::string PhyOutcomes = PrintPhyPacketsPerGw(startTime, stopTime, gwId);

    double avgDelay = 0;
    double avgAckDelay = 0;
    if (confirmedPacketsOutsideTransient)
      {
        avgDelay = (delaySum / (confirmedPacketsOutsideTransient +
                                successfulUnconfirmedPackets)).GetSeconds ();
        avgAckDelay = ((ackDelaySum) / confirmedPacketsOutsideTransient).GetSeconds ();
      }
    
    if(returnString)
    {
     returnValue = std::to_string(totalUnconfirmedPackets) + " " + std::to_string(successfulUnconfirmedPackets) + " | ";
     returnValue += std::to_string(successfullyExtractedConfirmedPackets) + " ";
     returnValue += std::to_string(successfullyAckedConfirmedPackets) + " | ";
     returnValue += std::to_string(incompleteConfirmedPackets);
     returnValue += " | ";    
     returnValue += PrintVector (successfulReTxAmounts, 1);
     returnValue +=  " | ";
     returnValue +=  PrintVector (failedReTxAmounts, 1);
     returnValue += " | ";
     returnValue += std::to_string(avgDelay) + " ";
     returnValue += std::to_string(avgAckDelay) + " ";
     returnValue += " | ";
     returnValue += PrintSumRetransmissions (totalReTxAmounts, 1);
     returnValue += " || ";
     returnValue += PhyOutcomes;

     std::string cpsr_result = CountMacPacketsGloballyCpsr(startTime, stopTime);
     std::string appCountConfirmed_result = CountAppPacketsPacketsGloballyCpsr(startTime, stopTime);
     std::string appCountUnconfirmed_result = CountAppPacketsPacketsGlobally(startTime, stopTime);
     returnValue += " ** ";
     returnValue += cpsr_result;
     returnValue += " ** ";
     returnValue += appCountConfirmed_result;
     returnValue += " ";
     returnValue += appCountUnconfirmed_result;

    std::string countGeneratedPackets_result = DetermineEnergyResults(startTime, stopTime);

    returnValue += " $ ";
    returnValue += countGeneratedPackets_result;    
    return returnValue;
    }

    else
    {
      // Print legend
      std::cout << getPerformanceLegend() << std::endl;
      std::cout << totalUnconfirmedPackets << " " << successfulUnconfirmedPackets << " | ";
      std::cout << successfullyExtractedConfirmedPackets << " ";
      std::cout << successfullyAckedConfirmedPackets << " | ";
      std::cout << incompleteConfirmedPackets << " | ";
      PrintVector (successfulReTxAmounts);
      std::cout << " | ";
      PrintVector (failedReTxAmounts);
      std::cout << " | ";
      std::cout << avgDelay << " ";
      std::cout << avgAckDelay << " ";
      std::cout << " | ";
      PrintSumRetransmissions (totalReTxAmounts);
      std::cout << " || ";
      std::cout<< PhyOutcomes;
      std::cout<< " ** ";
      std::string cpsr_result = CountMacPacketsGloballyCpsr(startTime, stopTime);
      std::cout << cpsr_result;
      std::cout<< " ** ";
      std::string appCountConfirmed_result = CountAppPacketsPacketsGloballyCpsr(startTime, stopTime);
      std::string appCountUnconfirmed_result = CountAppPacketsPacketsGlobally(startTime, stopTime);
    
      std::cout << appCountConfirmed_result;
      std::cout<<" ";
      std::cout << appCountUnconfirmed_result;

      std::string countGeneratedPackets_result = DetermineEnergyResults(startTime, stopTime);

      std::cout << " $ ";
      std::cout << countGeneratedPackets_result; 
      std::cout << std::endl;
  
    }
    return "";


  }

std::string
LoraPacketTracker::PrintSumRetransmissions (std::vector<int> reTxVector, int returnString)
{
  int total = 0;

  for (int i = 0; i < int(reTxVector.size ()); i++)
    {
      total += reTxVector[i] * (i + 1);
    }
  if(returnString)
    return std::to_string(total);
  else  
    std::cout << total;

  return "";
}

std::string
LoraPacketTracker::PrintVector (std::vector<int> vector, int returnString)
{
  std::string returnValue="";

  for (int i = 0; i < int(vector.size ()); i++)
    {
      if(returnString)
        returnValue += std::to_string(vector.at (i)) + " ";
      else       
        std::cout << vector.at (i) << " ";
    }
  return returnValue;

}

bool LoraPacketTracker::CheckIfUnconfirmed(Ptr<const Packet> packet)
{
//Based of ConfirmedMessagesComponent::OnReceivedPacket in network-controller-components.cc

  // Check whether the received packet requires an acknowledgment.
  LorawanMacHeader mHdr;

  Ptr<Packet> myPacket = packet->Copy ();
  uint32_t removed = myPacket->RemoveHeader (mHdr);
  NS_LOG_INFO("Removed " << removed << " header bytes. Mtype = " << (unsigned) mHdr.GetMType ());
  //NS_LOG_INFO ("Received packet Mac Header: " << mHdr);


  if (mHdr.GetMType () == LorawanMacHeader::CONFIRMED_DATA_UP)
    {
      NS_LOG_DEBUG ("Packet requires confirmation");
      return false;
    }
    else
    {
      return true;
    }
    
}


} // namespace lorawan
} // namespace ns3
