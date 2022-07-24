/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2017 University of Padova
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
 *         Martina Capuzzo <capuzzom@dei.unipd.it>
 *
 * Modified by: Peggy Anderson <peggy.anderson@usask.ca>
 */

#include "ns3/class-a-end-device-lorawan-mac.h"
#include "ns3/abort.h"
#include "ns3/end-device-lorawan-mac.h"
#include "ns3/end-device-lora-phy.h"
#include "ns3/log.h"
#include <algorithm>

namespace ns3 {
namespace lorawan {

NS_LOG_COMPONENT_DEFINE ("ClassAEndDeviceLorawanMac");

NS_OBJECT_ENSURE_REGISTERED (ClassAEndDeviceLorawanMac);

TypeId
ClassAEndDeviceLorawanMac::GetTypeId (void)
{
  static TypeId tid =
      TypeId ("ns3::ClassAEndDeviceLorawanMac")
          .SetParent<EndDeviceLorawanMac> ()
          .SetGroupName ("lorawan")
          .AddConstructor<ClassAEndDeviceLorawanMac> ()
          .AddAttribute ("Window2SFSameAsW1",
                          "Whether device should listen with the same SF as W1",
                          BooleanValue (false),
                          MakeBooleanAccessor (&ClassAEndDeviceLorawanMac::m_W2SFSameAsW1),
                          MakeBooleanChecker ())            
          .AddAttribute (
              "FastRx2", "Whether to employ a the same DR in RX1 and RX2", BooleanValue (false),
              MakeBooleanAccessor (&ClassAEndDeviceLorawanMac::m_fastRx2), MakeBooleanChecker ())
          .AddTraceSource ("CloseFirstReceiveWindow", "Callback fired when closing RX1",
                           MakeTraceSourceAccessor (
                               &ClassAEndDeviceLorawanMac::m_closeFirstReceiveWindowCallback),
                           "ns3::ClassAEndDeviceLorawanMac::EmptyCallback")
          .AddTraceSource ("CloseSecondReceiveWindow", "Callback fired when closing RX2",
                           MakeTraceSourceAccessor (
                               &ClassAEndDeviceLorawanMac::m_closeSecondReceiveWindowCallback),
                           "ns3::ClassAEndDeviceLorawanMac::EmptyCallback")
          .AddTraceSource ("OpenFirstReceiveWindow", "Callback fired when closing RX1",
                           MakeTraceSourceAccessor (
                               &ClassAEndDeviceLorawanMac::m_openFirstReceiveWindowCallback),
                           "ns3::ClassAEndDeviceLorawanMac::EmptyCallback")
          .AddTraceSource ("OpenSecondReceiveWindow", "Callback fired when closing RX2",
                           MakeTraceSourceAccessor (
                               &ClassAEndDeviceLorawanMac::m_openSecondReceiveWindowCallback),
                           "ns3::ClassAEndDeviceLorawanMac::EmptyCallback")
          .AddTraceSource (
              "MissedRX1becauseOff", "Callback fired when RX1 not completed because Off",
              MakeTraceSourceAccessor (&ClassAEndDeviceLorawanMac::m_missedRx1becauseOffCallback),
              "ns3::ClassAEndDeviceLorawanMac::EmptyCallback")
          .AddTraceSource (
              "MissedRX2becauseOff", "Callback fired when RX2 not completed because Off",
              MakeTraceSourceAccessor (&ClassAEndDeviceLorawanMac::m_missedRx2becauseOffCallback),
              "ns3::ClassAEndDeviceLorawanMac::EmptyCallback")
  .AddTraceSource (
      "TxFinished", "Callback fired when finishing a Tx",
      MakeTraceSourceAccessor (&ClassAEndDeviceLorawanMac::m_txFinished),
      "ns3::ClassAEndDeviceLorawanMac::EmptyCallback");
  return tid;
}

ClassAEndDeviceLorawanMac::ClassAEndDeviceLorawanMac ()
    : // LoraWAN default
      m_receiveDelay1 (Seconds (1)),
      // LoraWAN default
      m_receiveDelay2 (Seconds (2)),
      m_rx1DrOffset (0),
      m_W2SFSameAsW1(false)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG(" My window 2 usage is " << m_W2SFSameAsW1);

  // Void the two receiveWindow events
  m_closeFirstWindow = EventId ();
  m_closeFirstWindow.Cancel ();
  m_closeSecondWindow = EventId ();
  m_closeSecondWindow.Cancel ();
  m_secondReceiveWindow = EventId ();
  m_secondReceiveWindow.Cancel ();
}

ClassAEndDeviceLorawanMac::~ClassAEndDeviceLorawanMac ()
{
  NS_LOG_FUNCTION_NOARGS ();
}

/////////////////////
// Sending methods //
/////////////////////

void
ClassAEndDeviceLorawanMac::SendToPhy (Ptr<Packet> packetToSend)
{
  /////////////////////////////////////////////////////////
  // Add headers, prepare TX parameters and send the packet
  /////////////////////////////////////////////////////////

  NS_LOG_DEBUG ("PacketToSend: " << packetToSend);

  // Data Rate Adaptation as in LoRaWAN specification, V1.0.2 (2016)
  if (m_enableDRAdapt && (m_dataRate > 0) && (m_retxParams.retxLeft < m_maxNumbTx) &&
      (m_retxParams.retxLeft % 2 == 0))
    {
      m_txPower = 14; // Reset transmission power
      m_dataRate = m_dataRate - 1;
    }

  // Wake up PHY layer and directly send the packet

  Ptr<LogicalLoraChannel> txChannel = GetChannelForTx ();

  NS_LOG_DEBUG("Header disabled " << m_params.headerDisabled);
  NS_LOG_DEBUG ("PacketToSend: " << packetToSend);
  m_phy->Send (packetToSend, m_params, txChannel->GetFrequency (), m_txPower);

  //////////////////////////////////////////////
  // Register packet transmission for duty cycle
  //////////////////////////////////////////////

  // Compute packet duration
  Time duration = m_phy->GetOnAirTime (packetToSend, m_params);

  NS_LOG_DEBUG("++ PACKET DURATION ++" << duration.GetSeconds());

  // Register the sent packet into the DutyCycleHelper
  m_channelHelper.AddEvent (duration, txChannel);

  //////////////////////////////
  // Prepare for the downlink //
  //////////////////////////////

  // Switch the PHY to the channel so that it will listen here for downlink
  m_phy->GetObject<EndDeviceLoraPhy> ()->SetFrequency (txChannel->GetFrequency ());

  // Instruct the PHY on the right Spreading Factor to listen for during the window
  // create a SetReplyDataRate function?
  uint8_t replyDataRate = GetFirstReceiveWindowDataRate ();
  NS_LOG_DEBUG ("m_dataRate: " << unsigned (m_dataRate)
                               << ", m_rx1DrOffset: " << unsigned (m_rx1DrOffset)
                               << ", replyDataRate: " << unsigned (replyDataRate) << ".");

  m_phy->GetObject<EndDeviceLoraPhy> ()->SetSpreadingFactor (GetSfFromDataRate (replyDataRate));
}

//////////////////////////
//  Receiving methods   //
//////////////////////////
void
ClassAEndDeviceLorawanMac::Receive (Ptr<Packet const> packet)
{
  NS_LOG_FUNCTION (this << packet);

  // Work on a copy of the packet
  Ptr<Packet> packetCopy = packet->Copy ();

  // Remove the Mac Header to get some information
  LorawanMacHeader mHdr;
  packetCopy->RemoveHeader (mHdr);

  NS_LOG_DEBUG ("Mac Header: " << mHdr);

  // Only keep analyzing the packet if it's downlink
  if (!mHdr.IsUplink ())
    {
      NS_LOG_INFO ("Found a downlink packet.");

      // Remove the Frame Header
      LoraFrameHeader fHdr;
      fHdr.SetAsDownlink ();
      packetCopy->RemoveHeader (fHdr);

      NS_LOG_DEBUG ("Frame Header: " << fHdr);

      // Determine whether this packet is for us
      bool messageForUs = (m_address == fHdr.GetAddress ());

      if (messageForUs)
        {
          NS_LOG_INFO ("The message is for us!");

          // If it exists, cancel the second receive window event
          // THIS WILL BE GetReceiveWindow()
          Simulator::Cancel (m_secondReceiveWindow);

          // Parse the MAC commands
          ParseCommands (fHdr);

          // TODO Pass the packet up to the NetDevice

          // Call the trace source
          m_receivedPacket (packet);
        }
      else
        {
          NS_LOG_DEBUG ("The message is intended for another recipient.");

          // In this case, we are either receiving in the first receive window
          // and finishing reception inside the second one, or receiving a
          // packet in the second receive window and finding out, after the
          // fact, that the packet is not for us. In either case, if we no
          // longer have any retransmissions left, we declare failure.
          if (m_secondReceiveWindow.IsExpired ())
            {
              if (m_retxParams.retxLeft == 0)
                {
                  uint8_t txs = m_maxNumbTx - (m_retxParams.retxLeft);
                  if(m_retxParams.waitingAck)
                    {
                      m_requiredTxCallback (txs, false, m_retxParams.firstAttempt, m_retxParams.packet);
                      NS_LOG_DEBUG ("Failure: no more retransmissions left for confirmed packet. Used " << unsigned(txs) << " transmissions.");
                    }
                  else if(m_retxParams.sendingMultipleUnconfirmed)
                      NS_LOG_DEBUG ("Failure: no more retransmissions left for unconfirmed packet. Used " << unsigned(txs) << " transmissions.");

                  // Reset retransmission parameters
                  resetRetransmissionParameters ();
                }
              else // Reschedule
                {
                  this->Send (m_retxParams.packet);
                  NS_LOG_INFO ("We have " << unsigned (m_retxParams.retxLeft)
                                          << " retransmissions left: rescheduling transmission.");
                }
            }
        }
    }
  else if (m_secondReceiveWindow.IsExpired ())
    {
      NS_LOG_INFO ("The packet we are receiving is in uplink.");
      if (m_retxParams.retxLeft > 0)
        {
          this->Send (m_retxParams.packet);
          NS_LOG_INFO ("We have " << unsigned (m_retxParams.retxLeft)
                                  << " retransmissions left: rescheduling transmission.");
        }
      else
        {
          uint8_t txs = m_maxNumbTx - (m_retxParams.retxLeft);
          if(m_retxParams.waitingAck)
            {

              m_requiredTxCallback (txs, false, m_retxParams.firstAttempt, m_retxParams.packet);
              NS_LOG_DEBUG ("Failure: no more retransmissions left for confirmed packet. Used " << unsigned(txs) << " transmissions.");
            }
          else if (m_retxParams.sendingMultipleUnconfirmed)
              NS_LOG_DEBUG ("Failure: no more retransmissions left for unconfirmed packet. Used " << unsigned(txs) << " transmissions.");

          // Reset retransmission parameters
          resetRetransmissionParameters ();
        }
    }

  // Successful reception: switch to sleep
  m_phy->GetObject<EndDeviceLoraPhy> ()->SwitchToSleep ();
}

void
ClassAEndDeviceLorawanMac::FailedReception (Ptr<Packet const> packet, bool lostBecauseInterference)
{
  NS_LOG_FUNCTION (this << "lostBecauseInt: " << lostBecauseInterference << "< packet: " << packet);

  if (lostBecauseInterference)
    {
      // Switch to sleep after a failed reception
      m_phy->GetObject<EndDeviceLoraPhy> ()->SwitchToSleep ();
    }

  // If needed, schedule a retransmission
  if (m_secondReceiveWindow.IsExpired ())
    {
      if (m_retxParams.retxLeft > 0)
        {
          this->Send (m_retxParams.packet);
          NS_LOG_INFO ("We have " << unsigned(m_retxParams.retxLeft) << " retransmissions left: rescheduling transmission.");
        }
      else
        {
          uint8_t txs = m_maxNumbTx - (m_retxParams.retxLeft);
          if( m_retxParams.waitingAck)
            {

              m_requiredTxCallback (txs, false, m_retxParams.firstAttempt, m_retxParams.packet);
              NS_LOG_DEBUG ("Failure: no more retransmissions left for confirmed packet. Used " << unsigned(txs) << " transmissions.");              
            }
          else if(m_retxParams.sendingMultipleUnconfirmed)
              NS_LOG_DEBUG ("Failure: no more retransmissions left for unconfirmed packet. Used " << unsigned(txs) << " transmissions.");

          // Reset retransmission parameters
          resetRetransmissionParameters ();
        }
    }
}


void
ClassAEndDeviceLorawanMac::TxFinished (Ptr<const Packet> packet)
{
  NS_LOG_FUNCTION_NOARGS ();

  // Switch the PHY to IDLE (waiting time before RX1)
  bool switchOk = m_phy->GetObject<EndDeviceLoraPhy> ()->SwitchToIdle ();

  // Schedule receive windows only if we were able to switch to idle mode
  if (switchOk)
    {
      // Schedule the opening of the first receive window
      Simulator::Schedule (m_receiveDelay1, &ClassAEndDeviceLorawanMac::OpenFirstReceiveWindow,
                           this);

      // Schedule the opening of the second receive window
      m_secondReceiveWindow = Simulator::Schedule (
          m_receiveDelay2, &ClassAEndDeviceLorawanMac::OpenSecondReceiveWindow, this);
    }
  // Fire the callback
  m_txFinished();
}

void
ClassAEndDeviceLorawanMac::OpenFirstReceiveWindow (void)
{
  NS_LOG_FUNCTION_NOARGS ();

  // Set Phy in Standby mode
  bool switchOk = m_phy->GetObject<EndDeviceLoraPhy> ()->SwitchToStandby ();

  //Calculate the duration of a single symbol for the first receive window DR
  double tSym = pow (2, GetSfFromDataRate (GetFirstReceiveWindowDataRate ())) /
                GetBandwidthFromDataRate (GetFirstReceiveWindowDataRate ());

  // Schedule return to sleep after "at least the time required by the end
  // device's radio transceiver to effectively detect a downlink preamble"
  // (LoraWAN specification)
  if (switchOk)
    {
      // Stay open for Tpreamble
      NS_LOG_DEBUG ("Scheduling closing RX1 after "
                    << (4.25 + m_receiveWindowDurationInSymbols) * tSym << " seconds");
      m_closeFirstWindow = Simulator::Schedule (Seconds ((4.25+m_receiveWindowDurationInSymbols) * tSym),
                                                &ClassAEndDeviceLorawanMac::CloseFirstReceiveWindow,
                                                this); //m_receiveWindowDuration
      // Fire the callback
      m_openFirstReceiveWindowCallback ();
    }
  else
    {
      m_missedRx1becauseOffCallback ();
    }

}

void
ClassAEndDeviceLorawanMac::CloseFirstReceiveWindow (void)
{
  NS_LOG_FUNCTION_NOARGS ();

  Ptr<EndDeviceLoraPhy> phy = m_phy->GetObject<EndDeviceLoraPhy> ();

  // Check the Phy layer's state:
  // - RX -> We are receiving a preamble.
  // - STANDBY -> Nothing was received.
  // - SLEEP -> We have received a packet.
  // We should never be in TX or SLEEP mode at this point
  NS_LOG_DEBUG ("Actual state is " << phy->GetState ());
  switch (phy->GetState ())
    {
    case EndDeviceLoraPhy::TX:
      NS_ABORT_MSG ("PHY was in TX mode when attempting to "
                    << "close a receive window.");
      break;
    case EndDeviceLoraPhy::IDLE:
      NS_ABORT_MSG ("PHY was in IDLE mode when attempting to "
                    << "close a receive window (which was not opened).");
      break;
    case EndDeviceLoraPhy::TURNON:
      NS_ABORT_MSG ("PHY was TURNING ON, when attempting to "
                    << "close a receive window (which was not opened).");
      break;
    case EndDeviceLoraPhy::RX:
      // PHY is receiving: let it finish. The Receive method will switch it back to SLEEP.
      // Fire the callback
      m_closeFirstReceiveWindowCallback ();
      break;
    case EndDeviceLoraPhy::SLEEP:
      // PHY has received, and the MAC's Receive already put the device to sleep
      break;
    case EndDeviceLoraPhy::STANDBY:
      // Turn PHY layer to IDLE (sleep between receive windows)
      phy->SwitchToIdle ();
      // Fire the callback
      m_closeFirstReceiveWindowCallback ();
      break;
    case EndDeviceLoraPhy::OFF:
      // The ED did not have enough energy to keep RX1 open and was put to OFF
      NS_LOG_DEBUG ("Error! Trying to close the first receive window when in OFF mode");
      m_missedRx1becauseOffCallback ();
      // TODO Callback
      break;
    }
}

void
ClassAEndDeviceLorawanMac::OpenSecondReceiveWindow (void)
{
  NS_LOG_FUNCTION_NOARGS ();

  // Check for receiver status: if it's locked on a packet, don't open this
  // window at all.
  if (m_phy->GetObject<EndDeviceLoraPhy> ()->GetState () == EndDeviceLoraPhy::RX)
    {
      NS_LOG_INFO ("Won't open second receive window since we are in RX mode.");

      return;
    }

  // Set Phy in Standby mode
  bool switchOk = m_phy->GetObject<EndDeviceLoraPhy> ()->SwitchToStandby ();

  
  // Switch to appropriate channel and data rate
  NS_LOG_INFO ("Using parameters: " << m_secondReceiveWindowFrequency << "Hz, DR"
                                    << unsigned (GetSecondReceiveWindowDataRate()));

  m_phy->GetObject<EndDeviceLoraPhy> ()->SetFrequency (m_secondReceiveWindowFrequency);
  m_phy->GetObject<EndDeviceLoraPhy> ()->SetSpreadingFactor (
      GetSfFromDataRate (GetSecondReceiveWindowDataRate()));

  //Calculate the duration of a single symbol for the second receive window DR
  double tSym = pow (2, GetSfFromDataRate (GetSecondReceiveWindowDataRate ())) /
                GetBandwidthFromDataRate (GetSecondReceiveWindowDataRate ());

  // Schedule return to sleep after "at least the time required by the end
  // device's radio transceiver to effectively detect a downlink preamble"
  // (LoraWAN specification)
  if (switchOk)
    {
      NS_LOG_DEBUG("RX2 DURATION " << Seconds ((4.25+m_receiveWindowDurationInSymbols) * tSym));
      // Stay open for Tpreamble
      m_closeSecondWindow =
        Simulator::Schedule (Seconds ((4.25+m_receiveWindowDurationInSymbols) * tSym),
                               &ClassAEndDeviceLorawanMac::CloseSecondReceiveWindow, this);
      // Fire the callback
      m_openSecondReceiveWindowCallback ();
    }
  else
    {
      NS_LOG_DEBUG("ERROR switch failed");
      m_missedRx2becauseOffCallback ();
    }
}

void
ClassAEndDeviceLorawanMac::CloseSecondReceiveWindow (void)
{
  NS_LOG_FUNCTION_NOARGS ();

  Ptr<EndDeviceLoraPhy> phy = m_phy->GetObject<EndDeviceLoraPhy> ();

  // NS_ASSERT (phy->m_state != EndDeviceLoraPhy::TX &&
  // phy->m_state != EndDeviceLoraPhy::SLEEP);

  // Check the Phy layer's state:
  // - RX -> We have received a preamble.
  // - STANDBY -> Nothing was detected.
  switch (phy->GetState ())
    {
    case EndDeviceLoraPhy::TX:
      break;
    case EndDeviceLoraPhy::IDLE:
      NS_ABORT_MSG ("PHY was in IDLE mode when attempting to "
                    << "close a receive window (which was not opened).");
      break;
    case EndDeviceLoraPhy::TURNON:
      NS_ABORT_MSG ("PHY was TURNING ON, when attempting to "
                    << "close a receive window (which was not opened).");
      break;
    case EndDeviceLoraPhy::SLEEP:
      break;
    case EndDeviceLoraPhy::RX:
      // PHY is receiving: let it finish
      NS_LOG_DEBUG ("PHY is receiving: Receive will handle the result.");
      // Fire the callback
      m_closeSecondReceiveWindowCallback ();
      return;
    case EndDeviceLoraPhy::STANDBY:
      if (phy->IsEnergyStateOk ())
        {
          // Fire the callback
          m_closeSecondReceiveWindowCallback ();
        }
      // Turn PHY layer to sleep
      phy->SwitchToSleep ();
      break;
    case EndDeviceLoraPhy::OFF:
      // The ED did not have enough energy to keep RX2 open and was put to OFF
      NS_LOG_DEBUG ("Error! Trying to close the second receive window when in OFF mode");
      // TODO Callback
      m_missedRx2becauseOffCallback ();
      break;
    }

    if (m_retxParams.retxLeft > 0 )
      {
      NS_LOG_DEBUG ("No reception initiated by PHY: rescheduling transmission.");
        if(m_retxParams.waitingAck)
           NS_LOG_DEBUG ("No reception initiated by PHY: rescheduling transmission of confirmed packet.");

        if(m_retxParams.sendingMultipleUnconfirmed)
           NS_LOG_DEBUG ("No reception initiated by PHY: rescheduling transmission of unconfirmed packet.");     
           
        NS_LOG_INFO ("We have " << unsigned(m_retxParams.retxLeft) << " retransmissions left: rescheduling transmission.");

        this->Send (m_retxParams.packet);
      }

      else if (m_retxParams.retxLeft == 0 &&
               m_phy->GetObject<EndDeviceLoraPhy> ()->GetState () != EndDeviceLoraPhy::RX)
        {
          uint8_t txs = m_maxNumbTx - (m_retxParams.retxLeft);
          if(m_retxParams.waitingAck)
            {

              m_requiredTxCallback (txs, false, m_retxParams.firstAttempt, m_retxParams.packet);
              NS_LOG_DEBUG ("Failure: no more retransmissions left for confirmed packet. Used " << unsigned(txs) << " transmissions.");
            }
          else if (m_retxParams.sendingMultipleUnconfirmed)
              NS_LOG_DEBUG ("Finished: no more retransmissions left for unconfirmed packet. Used " << unsigned(txs) << " transmissions.");
          // Reset retransmission parameters
          resetRetransmissionParameters ();
        }
      else
        {
          NS_ABORT_MSG ("The number of retransmissions left is negative ! ");
        }

}

/////////////////////////
// Getters and Setters //
/////////////////////////

Time
ClassAEndDeviceLorawanMac::GetNextClassTransmissionDelay (Time waitingTime)
{
  NS_LOG_FUNCTION_NOARGS ();

  if (!m_closeFirstWindow.IsExpired () || !m_closeSecondWindow.IsExpired () ||
      !m_secondReceiveWindow.IsExpired ())
    {
      NS_LOG_WARN ("Attempting to send when there are receive windows:"
                   << " Transmission postponed.");

      //Calculate the duration of a single symbol for the second receive window DR
      double tSym = pow (2, GetSfFromDataRate (GetSecondReceiveWindowDataRate ())) /
                    GetBandwidthFromDataRate (GetSecondReceiveWindowDataRate ());

      Time endSecondRxWindow =
          (m_receiveDelay2 + Seconds (m_receiveWindowDurationInSymbols * tSym));
      waitingTime = std::max (waitingTime, endSecondRxWindow);
    }

  return waitingTime;
}

uint8_t
ClassAEndDeviceLorawanMac::GetFirstReceiveWindowDataRate (void)
{
  return m_replyDataRateMatrix.at (m_dataRate).at (m_rx1DrOffset);
}

void
ClassAEndDeviceLorawanMac::SetSecondReceiveWindowDataRate (uint8_t dataRate)
{
  m_secondReceiveWindowDataRate = dataRate;
}

uint8_t
ClassAEndDeviceLorawanMac::GetSecondReceiveWindowDataRate (void)
{
  if (m_fastRx2 or m_W2SFSameAsW1)
    {
      NS_LOG_DEBUG("ED:Changing window 2 data rate");
      return m_dataRate;
    }
  else
    {
      return m_secondReceiveWindowDataRate;
    }
}

  void ClassAEndDeviceLorawanMac::SetSecondReceiveWindowFrequency (double frequencyMHz)
  {
    m_secondReceiveWindowFrequency = frequencyMHz;
  }

  double ClassAEndDeviceLorawanMac::GetSecondReceiveWindowFrequency (void)
  {
    return m_secondReceiveWindowFrequency;
  }

  /////////////////////////
  // MAC command methods //
  /////////////////////////

  void ClassAEndDeviceLorawanMac::OnRxClassParamSetupReq (Ptr<RxParamSetupReq> rxParamSetupReq)
  {
    NS_LOG_FUNCTION (this << rxParamSetupReq);

    bool offsetOk = true;
    bool dataRateOk = true;

    uint8_t rx1DrOffset = rxParamSetupReq->GetRx1DrOffset ();
    uint8_t rx2DataRate = rxParamSetupReq->GetRx2DataRate ();
    double frequency = rxParamSetupReq->GetFrequency ();

    NS_LOG_FUNCTION (this << unsigned (rx1DrOffset) << unsigned (rx2DataRate) << frequency);

    // Check that the desired offset is valid
    if (!(0 <= rx1DrOffset && rx1DrOffset <= 5))
      {
        offsetOk = false;
      }

    // Check that the desired data rate is valid
    if (GetSfFromDataRate (rx2DataRate) == 0 || GetBandwidthFromDataRate (rx2DataRate) == 0)
      {
        dataRateOk = false;
      }

    // For now, don't check for validity of frequency
    m_secondReceiveWindowDataRate = rx2DataRate;
    m_rx1DrOffset = rx1DrOffset;
    m_secondReceiveWindowFrequency = frequency;

    // Craft a RxParamSetupAns as response
    NS_LOG_INFO ("Adding RxParamSetupAns reply");
    m_macCommandList.push_back (CreateObject<RxParamSetupAns> (offsetOk, dataRateOk, true));
  }

} /* namespace lorawan */
} /* namespace ns3 */
