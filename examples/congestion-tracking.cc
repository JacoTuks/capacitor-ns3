/*
 * This script is the first attempt at having congestion monitoring in the NS and has the advanced metrics and tracking of private lorawan repo.
 * Based off advanced-tracking (for tracking) and adr-example for NS setup.
 
 * 
 *  *  How to run and save output in a file (run this from ns-3 directory)
 * ./waf configure --enable-tests --enable-examples
 * ./waf build
 * ./waf --run "congestion-tracking --confirmedPercentage=15 --appPeriod=4959 --simTimeRatio="30" --RngRun=401" > output.c 2>&1
 */




#include "ns3/end-device-lora-phy.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/class-a-end-device-lorawan-mac.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/lora-packet-tracker.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/lora-helper.h"
#include "ns3/node-container.h"
#include "ns3/mobility-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/double.h"
#include "ns3/random-variable-stream.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/command-line.h"
#include "ns3/network-server-helper.h"
#include "ns3/correlated-shadowing-propagation-loss-model.h"
#include "ns3/building-penetration-loss.h"
#include "ns3/building-allocator.h"
#include "ns3/buildings-helper.h"
#include "ns3/forwarder-helper.h"
#include "ns3/network-status.h"
#include <algorithm>
#include <ctime>

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE ("Congestion-Tracking");

// Network settings
int nDevices = 120; 
int nGateways = 1;
double radius = 6300;

//The next three will be changed by sem script as needed
int appPeriodSeconds = 591; //Data packets will be sent at this interval
int confirmedPercentage = 15; // % of devices sending confirmed traffic
std::string simTimeRatio = "30"; //by default run the sim for 53 periods and use 20-50 periods for calcs

int simulationAppPeriods = 50; 

//These two might be 1 bool in the future. Right now top one controls printing and bottom one changes node side
bool congestionEnabled = true; //Define if the congestion component must be enabled
bool enableACS = false; //Define if the ACS must make changes at confirmed nodes

bool SendAsManyAsPossible = false; //Should node only send new packet or as as many as possible when max size is exceeded.

bool GroupedPEnabled = false;

bool Window2SFSameAsW1Flag = false;

std::string MultiplePacketsCombiningMethod = "max";

std::string congestionType = "ns3::CongestionComponent"; //Currently the only type available

std::string trafficTypesGroupedP = "confirmed"; //whether grouped P should be applied to confirmed only, unconfirmed only or to both. Options are confirmed, unconfirmed or  both

int desiredNumCongestionCalcs = simulationAppPeriods - 20; // how many periodic calcs there must over the entire simulationTime.
                            

// Channel model
//Davide's paper only used the log-distance parts (no shadowing) except for specific sims
bool realisticChannelModel = false;

int basePacketSize = 10;
int randomPSizeMin = 2;
int randomPSizeMax = 8;

int numberOfTransmissions = 3; // The maximum number of transmissions allowed

// Output control
bool print = true; // Save building locations to buildings.txt

int
main (int argc, char *argv[])
{

Packet::EnablePrinting ();

  CommandLine cmd;
  cmd.AddValue ("appPeriod",
                "The period in seconds to be used by periodically transmitting applications",
                appPeriodSeconds);
  cmd.AddValue("nDevices", "Number of devices used in simulation", nDevices);
  cmd.AddValue("confirmedPercentage", "Which percentage of devices should employ confirmed packets", confirmedPercentage);
  cmd.AddValue ("simTimeRatio", "Number of appPeriod multiples over which to calc final results for (string)", simTimeRatio);
  cmd.AddValue ("simulationAppPeriods", "How many appPeriods multiples must be in the total sim", simulationAppPeriods);
  cmd.AddValue("basePacketSize", "Base size for application packets", basePacketSize);
  cmd.AddValue("randomPSizeMin", "Minimum size for randomly sized application packets", randomPSizeMin); 
  cmd.AddValue("randomPSizeMax", "Maximum size for randomly sized application packets", randomPSizeMax); 
  cmd.AddValue("desiredNumCongestionCalcs", "How many periodic congestion calculations must be in simulationTime", desiredNumCongestionCalcs);
  cmd.AddValue ("HistoryRangeACS", "ns3::CongestionComponent::HistoryRangeACS");
  cmd.AddValue ("MultiplePacketsCombiningMethod",
                "should the ACS average or find the max?", MultiplePacketsCombiningMethod);
  cmd.AddValue ("SendAsManyAsPossible",
                "Should node only send new packet or as as many as possible when max size is exceeded.", SendAsManyAsPossible);                
  cmd.AddValue("numberOfTransmissions", "NbTrans parameter (number of transmission per packet).", numberOfTransmissions);
  cmd.AddValue("enableACS","enable adaptive congestion scheme",enableACS);
  cmd.AddValue("GroupedPEnabled", "enable or disable groupedP algorithm",GroupedPEnabled );
  cmd.AddValue("Window2SFSameAsW1Flag", "enable or disable Window 2 SF same as W1",Window2SFSameAsW1Flag );  
  cmd.AddValue("trafficTypesGroupedP", "whether to apply groupedPackets only to unconfirmed, confirmed or both",trafficTypesGroupedP);
  cmd.Parse (argc, argv);


  Config::SetDefault ("ns3::PeriodicSender::SendAsManyAsPossible", BooleanValue (SendAsManyAsPossible));
  Config::SetDefault ("ns3::CongestionComponent::MultiplePacketsCombiningMethod", StringValue(MultiplePacketsCombiningMethod));
  Config::SetDefault ("ns3::CongestionComponent::GroupedPEnabled", BooleanValue(GroupedPEnabled));

  NS_LOG_INFO("Setting window 2 use to "<< Window2SFSameAsW1Flag);
  //Config::SetDefault ("ns3::NetworkStatus::W2SFSameAsW1", BooleanValue(Window2SFSameAsW1Flag));
  Config::SetDefault ("ns3::ClassAEndDeviceLorawanMac::Window2SFSameAsW1", BooleanValue(Window2SFSameAsW1Flag));



  double simulationTime = simulationAppPeriods*appPeriodSeconds; //Updated sim time with new value from sem

  // This is the duration of the window for each congestion calc 
  double congestionPeriod = floor((simulationTime-20*appPeriodSeconds)/desiredNumCongestionCalcs); // Updated sim time with new value from sem. 20 periods are deleted as first 20 will be excluded
 
  // Set up logging
  LogComponentEnable ("Congestion-Tracking", LOG_LEVEL_ALL);
  //LogComponentEnable ("CongestionComponent", LOG_LEVEL_ALL);
 // LogComponentEnable ("NetworkStatus", LOG_LEVEL_ALL);
  //LogComponentEnable ("EndDeviceLorawanMac", LOG_LEVEL_INFO);
  //LogComponentEnable ("ClassAEndDeviceLorawanMac", LOG_LEVEL_ALL);
  //LogComponentEnable("EndDeviceLoraPhy", LOG_LEVEL_ALL);
  //LogComponentEnable("GatewayLorawanMac", LOG_LEVEL_ALL);
  
  
  LogComponentEnableAll (LOG_PREFIX_FUNC);
  LogComponentEnableAll (LOG_PREFIX_NODE);
  LogComponentEnableAll (LOG_PREFIX_TIME);

  NS_LOG_INFO("Setting window 2 use to "<< Window2SFSameAsW1Flag);

  /***********
   *  Setup  *
   ***********/
  // Create the time value from the period
  Time appPeriod = Seconds (appPeriodSeconds);

  // Mobility
  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::UniformDiscPositionAllocator", "rho", DoubleValue (radius),
                                 "X", DoubleValue (0.0), "Y", DoubleValue (0.0));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  /************************
   *  Create the channel  *
   ************************/

  // Create the lora channel object
  Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
  loss->SetPathLossExponent (3.76);
  loss->SetReference (1, 7.7); //can also use 8.1, 8.1 comes from private lorawan repo's rawcompletenetwork example

  if (realisticChannelModel)
    {
      // Create the correlated shadowing component
      Ptr<CorrelatedShadowingPropagationLossModel> shadowing =
          CreateObject<CorrelatedShadowingPropagationLossModel> ();

      // Aggregate shadowing to the logdistance loss
      loss->SetNext (shadowing);

      // Add the effect to the channel propagation loss
      Ptr<BuildingPenetrationLoss> buildingLoss = CreateObject<BuildingPenetrationLoss> ();

      shadowing->SetNext (buildingLoss);
    }

  Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();

  Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);

  /************************
   *  Create the helpers  *
   ************************/

  // Create the LoraPhyHelper
  LoraPhyHelper phyHelper = LoraPhyHelper ();
  phyHelper.SetChannel (channel);

  // Create the LorawanMacHelper
  LorawanMacHelper macHelper = LorawanMacHelper ();

  // Create the LoraHelper
  LoraHelper helper = LoraHelper ();
  helper.EnablePacketTracking (); // Output filename
  // helper.EnableSimulationTimePrinting ();

  //Create the NetworkServerHelper
  NetworkServerHelper nsHelper = NetworkServerHelper ();

  //Create the ForwarderHelper
  ForwarderHelper forHelper = ForwarderHelper ();

  /************************
   *  Create End Devices  *
   ************************/

  // Create a set of nodes
  NodeContainer endDevices;
  endDevices.Create (nDevices);

  // Assign a mobility model to each node
  mobility.Install (endDevices);

  // Make it so that nodes are at a certain height > 0
  for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
    {
      Ptr<MobilityModel> mobility = (*j)->GetObject<MobilityModel> ();
      Vector position = mobility->GetPosition ();
      //NS_LOG_INFO(position);
      position.z = 1.2;
      mobility->SetPosition (position);
    }

  // Create the LoraNetDevices of the end devices
  uint8_t nwkId = 54;
  uint32_t nwkAddr = 1864;
  Ptr<LoraDeviceAddressGenerator> addrGen =
      CreateObject<LoraDeviceAddressGenerator> (nwkId, nwkAddr);

  // Create the LoraNetDevices of the end devices
  macHelper.SetAddressGenerator (addrGen);
  macHelper.SetRegion (LorawanMacHelper::EU);
  phyHelper.SetDeviceType (LoraPhyHelper::ED);
  macHelper.SetDeviceType (LorawanMacHelper::ED_A);
  helper.Install (phyHelper, macHelper, endDevices);

  // Now end devices are connected to the channel

  // Make traffic confirmed if needed

  // Figure out how many devices should employ confirmed traffic
  int confirmedNumber = confirmedPercentage * endDevices.GetN () / 100;
  int i = 0;

  for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
    {
      Ptr<Node> node = *j;

      // Set message type (default is unconfirmed)
      Ptr<LorawanMac> edMac =node->GetDevice (0)->GetObject<LoraNetDevice> ()->GetMac ();
      Ptr<ClassAEndDeviceLorawanMac> edLorawanMac = edMac->GetObject<ClassAEndDeviceLorawanMac> ();

      // Set message type, otherwise the NS does not send ACKs
      if (i < confirmedNumber)
      {
       edLorawanMac->SetMType (LorawanMacHeader::CONFIRMED_DATA_UP);
      }
      edLorawanMac->SetMaxNumberOfTransmissions (numberOfTransmissions);
      i++;
    }
  


  /*********************
   *  Create Gateways  *
   *********************/

  // Create the gateway nodes (allocate them uniformely on the disc)
  NodeContainer gateways;
  gateways.Create (nGateways);

  Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();
  // Make it so that nodes are at a certain height > 0
  allocator->Add (Vector (0.0, 0.0, 15.0));
  mobility.SetPositionAllocator (allocator);
  mobility.Install (gateways);

  // Create a netdevice for each gateway
  phyHelper.SetDeviceType (LoraPhyHelper::GW);
  macHelper.SetDeviceType (LorawanMacHelper::GW);
  helper.Install (phyHelper, macHelper, gateways);

  /**********************
   *  Handle buildings  *
   **********************/

  double xLength = 130;
  double deltaX = 32;
  double yLength = 64;
  double deltaY = 17;
  int gridWidth = 2 * radius / (xLength + deltaX);
  int gridHeight = 2 * radius / (yLength + deltaY);
  if (realisticChannelModel == false)
    {
      gridWidth = 0;
      gridHeight = 0;
    }
  Ptr<GridBuildingAllocator> gridBuildingAllocator;
  gridBuildingAllocator = CreateObject<GridBuildingAllocator> ();
  gridBuildingAllocator->SetAttribute ("GridWidth", UintegerValue (gridWidth));
  gridBuildingAllocator->SetAttribute ("LengthX", DoubleValue (xLength));
  gridBuildingAllocator->SetAttribute ("LengthY", DoubleValue (yLength));
  gridBuildingAllocator->SetAttribute ("DeltaX", DoubleValue (deltaX));
  gridBuildingAllocator->SetAttribute ("DeltaY", DoubleValue (deltaY));
  gridBuildingAllocator->SetAttribute ("Height", DoubleValue (6));
  gridBuildingAllocator->SetBuildingAttribute ("NRoomsX", UintegerValue (2));
  gridBuildingAllocator->SetBuildingAttribute ("NRoomsY", UintegerValue (4));
  gridBuildingAllocator->SetBuildingAttribute ("NFloors", UintegerValue (2));
  gridBuildingAllocator->SetAttribute (
      "MinX", DoubleValue (-gridWidth * (xLength + deltaX) / 2 + deltaX / 2));
  gridBuildingAllocator->SetAttribute (
      "MinY", DoubleValue (-gridHeight * (yLength + deltaY) / 2 + deltaY / 2));
  BuildingContainer bContainer = gridBuildingAllocator->Create (gridWidth * gridHeight);

  BuildingsHelper::Install (endDevices);
  BuildingsHelper::Install (gateways);

  // Print the buildings
  if (print)
    {
      std::ofstream myfile;
      myfile.open ("buildings.txt");
      std::vector<Ptr<Building>>::const_iterator it;
      int j = 1;
      for (it = bContainer.Begin (); it != bContainer.End (); ++it, ++j)
        {  
          Box boundaries = (*it)->GetBoundaries ();
          myfile << "set object " << j << " rect from " << boundaries.xMin << "," << boundaries.yMin
                 << " to " << boundaries.xMax << "," << boundaries.yMax << std::endl;
        }
      myfile.close ();
    }

  /**********************************************
   *  Set up the end device's spreading factor  *
   **********************************************/

  //Calcultes the lowest SF for each device to use whilst ensuring connectivity.
  macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel);

  NS_LOG_DEBUG( "Completed configuration, there are "
                 << endDevices.GetN() << " devices of which "
                 << confirmedNumber << " are confirmed.");

  /*********************************************
   *  Install applications on the end devices  *
   *********************************************/

  Time appStopTime = Seconds (simulationTime) + 3*Seconds(appPeriodSeconds);
  PeriodicSenderHelper appHelper = PeriodicSenderHelper ();
  appHelper.SetPeriod (Seconds (appPeriodSeconds));


  appHelper.SetPacketSize(basePacketSize);
  Ptr<RandomVariableStream> rv = CreateObjectWithAttributes<UniformRandomVariable> (
      "Min", DoubleValue (randomPSizeMin), "Max", DoubleValue (randomPSizeMax)); //randomPSizeMin and randomPSizeMax included

  appHelper.SetPacketSizeRandomVariable(rv); //As indicated by email from Martina

  ApplicationContainer appContainer = appHelper.Install (endDevices);

  appContainer.Start (Seconds (0));
  appContainer.Stop (appStopTime);

  /**************************
   *  Create Network Server  *
   ***************************/

  // Create the NS node
  NodeContainer networkServer;
  networkServer.Create (1);

  // Create a NS for the network
  nsHelper.SetEndDevices (endDevices);
  nsHelper.SetGateways (gateways);
  nsHelper.EnableCongestion (congestionEnabled);
  LoraPacketTracker &track = helper.GetPacketTracker();
  nsHelper.setPacketTracker(track);
  nsHelper.SetCongestion (congestionType);
  nsHelper.SetCongestionTrackingPeriod(Seconds(congestionPeriod));
  nsHelper.EnableWindow2SFChange(Window2SFSameAsW1Flag);

  nsHelper.Install (networkServer);

  //Create a forwarder for each gateway
  forHelper.Install (gateways);

  ////////////////
  // Simulation //
  ////////////////

  Simulator::Stop (appStopTime + Seconds(10)); // adding more time so that tracker can calculate overall metric after 3 periods have passed from time frame of interest

  NS_LOG_INFO( "Congestion is calculated over " << congestionPeriod << " s intervals");
  NS_LOG_INFO( "Stop time is " << (appStopTime + 3*Seconds(appPeriodSeconds)+ Seconds(10)).GetSeconds() << " s ");
  NS_LOG_INFO ("Running simulation...");
  Simulator::Run ();

  Simulator::Destroy ();

  ///////////////////////////
  // Print results to file //
  ///////////////////////////
  NS_LOG_INFO ("Computing performance metrics...");

  //Want to compute using the middle time section of each simulation.
  //Each sim ran for 5*appPeriodSeconds, so computation must use the middle interval
  //PrintPhyPacketsPerGw can take a start and stop time
  LoraPacketTracker &tracker = helper.GetPacketTracker ();
  //nDevices is the id of the gw as 1200 devices would be 0-1199 with gw being device 1200

  if(simTimeRatio == "all")
  {
    NS_LOG_INFO( "Computing over the period "<< 1*appPeriodSeconds << "s to "<< Seconds((simulationAppPeriods-1)*appPeriodSeconds).GetSeconds() << "s");
    tracker.PrintPerformance(Seconds(1*appPeriodSeconds), Seconds((simulationAppPeriods-1)*appPeriodSeconds), nDevices); 
  }
  else if(simTimeRatio == "20")
  {
    NS_LOG_INFO( "Computing over the period "<< 20*appPeriodSeconds << "s to "<< Seconds(40*appPeriodSeconds).GetSeconds() << "s");
    tracker.PrintPerformance(Seconds(20*appPeriodSeconds), Seconds(40*appPeriodSeconds), nDevices); 
  }

  else if(simTimeRatio == "30")
  {
    NS_LOG_INFO( "Computing over the period "<< 20*appPeriodSeconds << "s to "<< Seconds(50*appPeriodSeconds).GetSeconds() << "s");
    tracker.PrintPerformance(Seconds(20*appPeriodSeconds), Seconds(50*appPeriodSeconds), nDevices); 
  }
  else
    {
    NS_LOG_INFO ("Expand .cc file to handle your custom period");
  }



 

  //tracker.PrintPerformance(Seconds (Seconds(0)), Seconds(3*appPeriodSeconds), nDevices); //option for all 3
  //tracker.PrintPerformance(Seconds (appPeriodSeconds), Seconds(2*appPeriodSeconds), nDevices); //option for middle

      //std::cout << tracker.CountMacPacketsGlobally (Seconds (appPeriodSeconds), Seconds(2*appPeriodSeconds)) << std::endl;
      //std::cout << tracker.PrintPhyPacketsPerGw (Seconds (appPeriodSeconds), Seconds(2*appPeriodSeconds), nDevices) << std::endl;
    
  return 0;
}