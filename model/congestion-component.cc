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

#include "ns3/congestion-component.h"
//#include <iostream>
#include <fstream>
#include <math.h>
#include <algorithm>
namespace ns3 {
namespace lorawan {

////////////////////////////////////////
// Congestion detection and management //
////////////////////////////////////////

NS_LOG_COMPONENT_DEFINE ("CongestionComponent");

NS_OBJECT_ENSURE_REGISTERED (CongestionComponent);

TypeId CongestionComponent::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::CongestionComponent")
    .SetGroupName ("lorawan")
    .AddConstructor<CongestionComponent> ()
    .SetParent<NetworkControllerComponent> ()
  ;
  return tid;
}

CongestionComponent::CongestionComponent ()
{
  NS_LOG_DEBUG (this << " " << this->GetTypeId () << " contructor");

  m_lastNetworkCongestionUpdate = Seconds(0);
}

CongestionComponent::~CongestionComponent ()
{
}

void CongestionComponent::OnReceivedPacket (Ptr<const Packet> packet,
                                     Ptr<EndDeviceStatus> status,
                                     Ptr<NetworkStatus> networkStatus)
{

}

void
CongestionComponent::BeforeSendingReply (Ptr<EndDeviceStatus> status,
                                  Ptr<NetworkStatus> networkStatus)
{

}


void CongestionComponent::OnFailedReply (Ptr<EndDeviceStatus> status,
                                  Ptr<NetworkStatus> networkStatus)
{
  NS_LOG_FUNCTION (this->GetTypeId () << networkStatus);
}

void CongestionComponent::SetGateways (NodeContainer gateways)
{
  m_gateways = gateways;
}

void CongestionComponent::SetTracker(LoraPacketTracker & tracker)
{
  NS_LOG_FUNCTION (this);

  m_packetTracker = &tracker;
}

void CongestionComponent::SetCongestionPeriod(Time interval)
{
  NS_LOG_FUNCTION (this);
  m_congestionInterval = interval;
}

void
CongestionComponent::EnablePeriodicNetworkCongestionStatusPrinting (NodeContainer gateways,
                                                std::string filename)
{
  NS_LOG_FUNCTION (this);

  if(m_legendPrinted == false)
  {
    //Going to first print the legend (only want to this once)
    const char * c = filename.c_str ();
    std::ofstream outputFile;
    if (Simulator::Now () == Seconds (0))
      {
        // Delete contents of the file as it is opened
        outputFile.open (c, std::ofstream::out | std::ofstream::trunc);
      }
    else
      {
        // Only append to the file
        outputFile.open (c, std::ofstream::out | std::ofstream::app);
      }

      outputFile << m_packetTracker->getPerformanceLegend();
      outputFile.close();   
      m_legendPrinted = true;
  }

  DoPrintNetworkCongestionStatus (gateways, filename);

  // Schedule periodic printing
  Simulator::Schedule (m_congestionInterval,
                       &CongestionComponent::EnablePeriodicNetworkCongestionStatusPrinting, this,
                       gateways, filename);
}

void
CongestionComponent::DoPrintNetworkCongestionStatus (NodeContainer gateways,
                                 std::string filename)
{

  NS_LOG_FUNCTION (this);

  if( Simulator::Now () == 0)
  {
    NS_LOG_INFO("Skipping printing as this is the 0th interval (0s - 0s)");
    m_lastNetworkCongestionUpdate = Simulator::Now();
    return;
  }
  else if((Simulator::Now()- m_congestionInterval) == Seconds(0))
  {
    NS_LOG_INFO("Skipping printing as this is the end of the 1st interval (0s - " << m_congestionInterval.GetSeconds() << "s)");
    //m_lastNetworkCongestionUpdate = Simulator::Now();
    return;
  }
  else if((Simulator::Now()- 2*m_congestionInterval) == Seconds(0))
  {
    NS_LOG_INFO("Skipping printing as this is the end of the 2nd interval (" << m_lastNetworkCongestionUpdate.GetSeconds() <<  "s -  " << 2*m_congestionInterval.GetSeconds() << "s)");
    return;
  }
  else if((Simulator::Now()- 3*m_congestionInterval) == Seconds(0))
  {
    NS_LOG_INFO("Skipping printing as this is the end of the 3rd interval (" << m_lastNetworkCongestionUpdate.GetSeconds() <<  "s -  " << 3*m_congestionInterval.GetSeconds() << "s)");
    return;
  }
    const char * c = filename.c_str ();
    std::ofstream outputFile;

    // Only append to the file
    outputFile.open (c, std::ofstream::out | std::ofstream::app);
    NS_LOG_INFO ("Congestion tracking the interval " 
                 << m_lastNetworkCongestionUpdate.GetSeconds() << 
                   "s to " << ((Simulator::Now () - 3*m_congestionInterval).GetSeconds())  << "s");

    std::string performanceStats;   //Will only take stats from last gateway (basically only supports 1 GW networks)                           
    for (auto it = gateways.Begin (); it != gateways.End (); ++it)
      {
         int gwId = (*it)->GetId ();
         performanceStats = m_packetTracker->CountRetransmissionsPorted(m_lastNetworkCongestionUpdate,
                                                 (Simulator::Now () - 3*m_congestionInterval),
                                                 gwId, 1);
           outputFile << performanceStats  << std::endl;            
      }
    m_lastNetworkCongestionUpdate = Simulator::Now () - 3*m_congestionInterval;

    outputFile.close();
    CalculateCongestion(performanceStats);
}

void
CongestionComponent::CalculateCongestion (std::string performanceStats)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_INFO(m_packetTracker->getPerformanceLegend());
  NS_LOG_INFO(performanceStats);

  performanceStats.erase(std::remove(performanceStats.begin(), performanceStats.end(), '|'), performanceStats.end()); //remove |
  performanceStats.erase(std::remove(performanceStats.begin(), performanceStats.end(), '*'), performanceStats.end()); //remove *
  NS_LOG_INFO(performanceStats);

  std::stringstream iss(performanceStats);
  float number;
  std::vector<float> asNumbers;
  while ( iss >> number )
    asNumbers.push_back( number );


  float totalUnconfirmed = asNumbers[0];
  float succUnconfirmed = asNumbers[1];
  float succConfirmed = asNumbers[2];
  float succAcked = asNumbers[3];
  float totalConfirmed = asNumbers.end()[-2]; //get second last element (cspr confirmed sent)

  double unconfirmedULPDR = ceil(succUnconfirmed/totalUnconfirmed*10000)/100; //rounding to two decimals by adding (*100 and /100)
  double confirmedULPDR = ceil(succConfirmed/totalConfirmed*10000)/100; 
  double ackedPDR = ceil(succAcked/totalConfirmed*10000)/100; //Gateway will not know this value
  NS_LOG_INFO("unconfirmed ULPDR= " << unconfirmedULPDR <<  " %");
  NS_LOG_INFO("confirmed ULPDR= " << confirmedULPDR <<  " %");
  NS_LOG_INFO("ACKed PDR= " << ackedPDR <<  " %");
}

  std::string
  CongestionComponent::PrintVector (std::vector<float> vector, int returnString)
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

}
}
