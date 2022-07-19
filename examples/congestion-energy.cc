/*
 * This script simulates a network to test capacitor source
 * and lora radio energy model.
 */

#include "ns3/basic-energy-harvester.h"
#include "ns3/boolean.h"
#include "ns3/callback.h"
#include "ns3/double.h"
#include "ns3/end-device-lora-phy.h"
#include "ns3/end-device-lorawan-mac.h"
#include "ns3/energy-harvester-container.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/class-a-end-device-lorawan-mac.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/integer.h"
#include "ns3/lora-net-device.h"
#include "ns3/lora-radio-energy-model.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/one-shot-sender-helper.h"
#include "ns3/packet.h"
#include "ns3/random-variable-stream.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/lora-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/node-container.h"
#include "ns3/position-allocator.h"
#include "ns3/periodic-sender-helper.h"
//#include "ns3/energy-aware-sender-helper.h"
#include "ns3/command-line.h"
#include "ns3/basic-energy-source-helper.h"
#include "ns3/basic-energy-harvester-helper.h"
#include "ns3/lora-radio-energy-model-helper.h"
#include "ns3/network-server-helper.h"
#include "ns3/forwarder-helper.h"
#include "ns3/lora-packet-tracker.h"
#include "ns3/file-helper.h"
#include "ns3/names.h"
#include "ns3/config.h"
#include "ns3/string.h"
#include "src/core/model/config.h"
#include "src/lorawan/helper/variable-energy-harvester-helper.h"
#include "src/lorawan/model/capacitor-energy-source.h"
#include "src/lorawan/helper/capacitor-energy-source-helper.h"
#include "src/lorawan/model/lora-tx-current-model.h"
#include "ns3/correlated-shadowing-propagation-loss-model.h"
#include "ns3/building-penetration-loss.h"
#include "ns3/building-allocator.h"
#include "ns3/buildings-helper.h"
#include <algorithm>
#include <bits/stdint-uintn.h>
#include <cmath>
#include <ctime>
#include <iostream>
#include <ostream>
#include <sstream>
#include <string>
#include <sys/types.h>

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE ("Congestion-Energy");

// Inputs
int nGateways = 1;
int simulationAppPeriods = 50; //by default run the sim for 53 periods and use 20-50 periods for calcs
double appPeriodSeconds = 591; //591
double capacitance= 10; // mF
uint nDevices = 1200;
double radius = 500; ///6300
double eh = 0.001;
double E = 3.3; // V
double voltageThLow = 0.545454; // 1.8 V
double voltageThHigh = 0.9090; // 3 V

uint dr = 1;
int confirmedPercentage = 15; // % of devices sending confirmed traffic
bool realisticChannelModel = false; // Channel model
bool print = false; // Output control

//These two might be 1 bool in the future. Right now top one controls printing and bottom one changes node side
bool congestionEnabled = true; //Define if the congestion component must be enabled
bool enableACS = true; //Define if the ACS must make changes at confirmed nodes

bool SendAsManyAsPossible = true; //Should node only send new packet or as as many as possible when max size is exceeded.

bool SendAsManyAsPossibleEnergy = true; //Should nodes send as many as possible when ACS aggregation causes energy issues at larger sizes
bool GroupedPEnabled = true;

bool Window2SFSameAsW1Flag = false;

bool SendgroupedPEnergyACK = false;

std::string MultiplePacketsCombiningMethod = "max";

std::string congestionType = "ns3::CongestionComponent"; //Currently the only type available

std::string trafficTypesGroupedP = "confirmed"; //whether grouped P should be applied to confirmed only, unconfirmed only or to both. Options are confirmed, unconfirmed or  both

int desiredNumCongestionCalcs = simulationAppPeriods - 20; // how many periodic calcs there must over the entire simulationTime.
                            


int basePacketSize = 10;
int randomPSizeMin = 0;
int randomPSizeMax = 0;

int numberOfTransmissions = 1; // The maximum number of transmissions allowed



std::string sender = "periodicSender";
std::string filenameRemainingVoltage = "remainingVoltage.txt";
std::string filenameEnergyConsumption = "energyConsumption.txt";
std::string filenameRemainingEnergy = "remainingEnergy.txt";
std::string filenameState = "deviceStates.txt";
std::string filenameTx = "deviceTx.txt";
std::string filenameEnoughEnergy = "energyEnoughForTx.txt";
std::string pathToInputFile = "/home/marty/work/ua/panels_data";
std::string filenameHarvesterSun = "/outputixys.csv";
std::string filenameHarvesterCloudy = "/outputixys_cloudy.csv";
bool remainingVoltageCallbackFirstCall = true;
bool energyConsumptionCallbackFirstCall = true;
bool enoughEnergyCallbackFirstCall = true;
bool stateChangeCallbackFirstCall = true;
bool TxCallbackFirstCall = true;
int generatedPacketsAPP = 0;

int countNoTx = 0;

//////////////
// Callbcks //
//////////////

void
OnRemainingEnergyChange (double oldRemainingEnergy, double remainingEnergy)
{
  // NS_LOG_DEBUG (Simulator::Now().GetSeconds() << " " << remainingEnergy);
}

void
OnDeviceEnergyConsumption (double oldvalue, double energyConsumption)
{
  const char *c = filenameEnergyConsumption.c_str ();
  std::ofstream outputFile;
  if (energyConsumptionCallbackFirstCall)
    {
      // Delete contents of the file as it is opened
      outputFile.open (c, std::ofstream::out | std::ofstream::trunc);
      // Set the initial sleep state
      outputFile <<  "0 0" << std::endl;
      energyConsumptionCallbackFirstCall = false;
    }
  else
    {
    // Only append to the file
    outputFile.open (c, std::ofstream::out | std::ofstream::app);
    }

  outputFile << Simulator::Now ().GetSeconds () << " " << energyConsumption << std::endl;
  outputFile.close ();
}

void
OnRemainingVoltageChange (std::string context,
                          double oldRemainingVoltage, double remainingVoltage)
{
  //NS_LOG_DEBUG("Callback " << context);
  // if (context == "0")
  // {

  // NS_LOG_DEBUG("Callback " << context);
  // const char *c = filenameRemainingVoltage.c_str ();
  // std::ofstream outputFile;
  // if (remainingVoltageCallbackFirstCall)
  //   {
  //     // Delete contents of the file as it is opened
  //     outputFile.open (c, std::ofstream::out | std::ofstream::trunc);
  //     remainingVoltageCallbackFirstCall = false;
  //   }
  // else
  //   {
  //     // Only append to the file
  //     outputFile.open (c, std::ofstream::out | std::ofstream::app);
  //   }


  // outputFile << context << " "
  //            << Simulator::Now ().GetSeconds ()
  //            << " " << remainingVoltage
  //            << std::endl;
  // outputFile.close ();

 // }
}

void OnEnergyHarvested (double oldvalue, double totalEnergyHarvested)
{
 // NS_LOG_DEBUG("Total energy harvested: " << totalEnergyHarvested);
 // NS_LOG_DEBUG ("Energy harvested in this interval: " << totalEnergyHarvested - oldvalue);
}

void
EnergyDepletionCallback (void)
{
  NS_LOG_DEBUG("Energy depleted callback in main");
}

void
CheckEnoughEnergyCallback (uint32_t nodeId, Ptr<const Packet> packet,
                           Time time, bool boolValue)
{

  // if(boolValue == 1)
  // {
  //     NS_LOG_DEBUG("CheckEnoughEnergyCallback for " << nodeId << ": "<<boolValue);
  // }

  if(boolValue == 0)
  {
    NS_LOG_DEBUG("CheckEnoughEnergyCallback for " << nodeId << ": "<<boolValue);
    countNoTx++;   
 


    // const char *c = filenameEnoughEnergy.c_str ();
    // std::ofstream outputFile;
    // if (enoughEnergyCallbackFirstCall)
    //   {
    //     // Delete contents of the file as it is opened
    //     outputFile.open (c, std::ofstream::out | std::ofstream::trunc);
    //     enoughEnergyCallbackFirstCall = false;
    //   }
    // else
    //   {
    //     // Only append to the file
    //     outputFile.open (c, std::ofstream::out | std::ofstream::app);
    //   }

    // outputFile << nodeId<< " " <<Simulator::Now ().GetSeconds () << " " << boolValue << std::endl;
    // outputFile.close ();
  }
}


void
OnEndDeviceStateChange (std::string context,
                        EndDeviceLoraPhy::State oldstatus,
                        EndDeviceLoraPhy::State status)
{
  const char *c = filenameState.c_str ();
  std::ofstream outputFile;
  if (stateChangeCallbackFirstCall)
    {
      // Delete contents of the file as it is opened
      outputFile.open (c, std::ofstream::out | std::ofstream::trunc);
      // Set the initial sleep state
      //outputFile << 0 << " " << 0 << " " << 0 << std::endl;
      //NS_LOG_DEBUG ("Append initial state inside the callback");
      stateChangeCallbackFirstCall = false;
    }
  else
    {
      // Only append to the file
      outputFile.open (c, std::ofstream::out | std::ofstream::app);
    }

  outputFile << context << " " <<Simulator::Now ().GetSeconds () << " " << status << std::endl;
  outputFile.close ();
}

void
OnEndDeviceTx (std::string context, EndDeviceLoraPhy::State oldstatus,
                        EndDeviceLoraPhy::State status)
{
  if (status == EndDeviceLoraPhy::TX)
    {
      //NS_LOG_DEBUG("One transmission");
      const char *c = filenameTx.c_str ();
      std::ofstream outputFile;
      if (TxCallbackFirstCall)
        {
          outputFile.open (c, std::ofstream::out | std::ofstream::trunc);
          TxCallbackFirstCall = false;
        }
      else
        {
          // Only append to the file
          outputFile.open (c, std::ofstream::out | std::ofstream::app);
        }
      outputFile << context << " " << Simulator::Now ().GetSeconds () << std::endl;
      outputFile.close ();
    }
}

  void OnGeneratedPacket (void)
  {
    // NS_LOG_DEBUG("Generated packet at APP level");
    // In the whole network
    generatedPacketsAPP = generatedPacketsAPP + 1;
    // NS_LOG_DEBUG ("Total number of generated APP packet = " << generatedPacketsAPP);
  }

  void DCRestricted(Ptr<const Packet>, uint32_t device)

  {
      NS_LOG_DEBUG("Device " << (device));

  }

  /***********
 ** MAIN  **
 **********/
  int main (int argc, char *argv[])
  {

    // Inputs
    CommandLine cmd;
    cmd.AddValue("nDevices", "Number of devices used in simulation", nDevices);
    cmd.AddValue("confirmedPercentage", "Which percentage of devices should employ confirmed packets", confirmedPercentage);
    cmd.AddValue ("capacitance", "capacitance[mF]", capacitance);
    cmd.AddValue ("appPeriod",
                "The period in seconds to be used by periodically transmitting applications",
                appPeriodSeconds);
    cmd.AddValue("Window2SFSameAsW1Flag", "Use same SF for window 2", Window2SFSameAsW1Flag);
    cmd.AddValue("numberOfTransmissions", "NBtrans", numberOfTransmissions);
    cmd.AddValue("simulationAppPeriods", "How many appPeriods multiples must be in the total sim", simulationAppPeriods);
    cmd.AddValue ("dr", "dr (dr=-1 = based on the channel)", dr);
    cmd.AddValue ("eh", "Harvested power [W] - eh=-1 data for sunny day, eh=-2 data for cloudy day",
                  eh);
    cmd.Parse (argc, argv);

    // Set up logging
    LogComponentEnable ("Congestion-Energy", LOG_LEVEL_ALL);
//     LogComponentEnable ("LoraPacketTracker", LOG_LEVEL_WARN);
    //LogComponentEnable ("CapacitorEnergySource", LOG_LEVEL_ALL);
    // LogComponentEnable ("CapacitorEnergySourceHelper", LOG_LEVEL_ALL);
//    LogComponentEnable ("LoraRadioEnergyModel", LOG_LEVEL_WARN);
    // LogComponentEnable ("CongestionComponent", LOG_LEVEL_ALL);
 //     LogComponentEnable ("PeriodicSender", LOG_LEVEL_ALL);
    //  LogComponentEnable ("LoraFrameHeader", LOG_LEVEL_ALL);
    // LogComponentEnable ("VariableEnergyHarvester", LOG_LEVEL_ALL);
    // LogComponentEnable ("EnergySource", LOG_LEVEL_ALL);
    // LogComponentEnable ("BasicEnergySource", LOG_LEVEL_ALL);
    // LogComponentEnable ("LoraChannel", LOG_LEVEL_INFO);
 //   LogComponentEnable ("LoraPhy", LOG_LEVEL_ALL);
 //    LogComponentEnable ("EndDeviceLoraPhy", LOG_LEVEL_ALL);
    // LogComponentEnable ("SimpleEndDeviceLoraPhy", LOG_LEVEL_ALL);
    // LogComponentEnable ("GatewayLoraPhy", LOG_LEVEL_ALL);
    // LogComponentEnable ("SimpleGatewayLoraPhy", LOG_LEVEL_ALL);
    // LogComponentEnable ("LoraInterferenceHelper", LOG_LEVEL_ALL);
    // LogComponentEnable ("LorawanMac", LOG_LEVEL_ALL);

 //  LogComponentEnable ("EndDeviceLorawanMac", LOG_LEVEL_ALL);
//
//   LogComponentEnable ("ClassAEndDeviceLorawanMac", LOG_LEVEL_ALL);
 //    LogComponentEnable ("CongestionComponent", LOG_LEVEL_ALL);
    // LogComponentEnable ("LogicalLoraChannel", LOG_LEVEL_ALL);
    // LogComponentEnable ("LogicalLoraChannelHelper", LOG_LEVEL_ALL);
    // LogComponentEnable ("LoraPhyHelper", LOG_LEVEL_ALL);
    // LogComponentEnable ("LorawanMacHelper", LOG_LEVEL_ALL);
  // LogComponentEnable ("EndDeviceLoraMac", LOG_LEVEL_ALL);
    // LogComponentEnable ("OneShotSender", LOG_LEVEL_ALL);
  //   LogComponentEnable ("PeriodicSender", LOG_LEVEL_ALL);
   //  LogComponentEnable ("LoraRadioEnergyModel", LOG_LEVEL_ALL);
   // LogComponentEnable ("LoraTxCurrentModel", LOG_LEVEL_ALL);

    LogComponentEnableAll (LOG_PREFIX_FUNC);
    LogComponentEnableAll (LOG_PREFIX_NODE);
    LogComponentEnableAll (LOG_PREFIX_TIME);

    Config::SetDefault ("ns3::ClassAEndDeviceLorawanMac::Window2SFSameAsW1", BooleanValue(Window2SFSameAsW1Flag));


    double simulationTime = simulationAppPeriods*appPeriodSeconds; //Updated sim time with new value from sem

    

    // This is the duration of the window for each congestion calc 
    double congestionPeriod = appPeriodSeconds; //floor((simulationTime-20*appPeriodSeconds)/desiredNumCongestionCalcs); 
    // Updated sim time with new value from sem. 20 periods are deleted as first 20 will be excluded
 
    // Set SF
    Config::SetDefault ("ns3::EndDeviceLorawanMac::DataRate", UintegerValue (dr));

    // Set MAC behavior
    Config::SetDefault ("ns3::EndDeviceLorawanMac::MacTxIfEnergyOk", BooleanValue (true));


    // Create the time value from the period
    Time appPeriod = Seconds (appPeriodSeconds);


    /************************
  *  Create the channel  *
  ************************/

    // NS_LOG_INFO ("Creating the channel...");

    // Create the lora channel object
    Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
    loss->SetPathLossExponent (3.76);
    loss->SetReference (1, 7.7);

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

    // NS_LOG_INFO ("Setting up helpers...");

    // Create the LoraPhyHelper
    LoraPhyHelper phyHelper = LoraPhyHelper ();
    phyHelper.SetChannel (channel);

    // Create the LorawanMacHelper
    LorawanMacHelper macHelper = LorawanMacHelper ();

    // Create the LoraHelper
    LoraHelper helper = LoraHelper ();
    helper.EnablePacketTracking ();

    /************************
  *  Create End Devices  *
  ************************/

    // NS_LOG_INFO ("Creating the end device...");

    // Create a set of nodes
    NodeContainer endDevices;
    endDevices.Create (nDevices);

    // ED's mobility
    MobilityHelper mobility;
    mobility.SetPositionAllocator ("ns3::UniformDiscPositionAllocator", "rho", DoubleValue (radius),
                                   "X", DoubleValue (0.0), "Y", DoubleValue (0.0));
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    // Assign a mobility model to the node
    mobility.Install (endDevices);

    // Make it so that nodes are at a certain height > 0
    for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
      {
        Ptr<MobilityModel> mobility = (*j)->GetObject<MobilityModel> ();
        Vector position = mobility->GetPosition ();
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
    // Create the LoraNetDevices of the end devices
    phyHelper.SetDeviceType (LoraPhyHelper::ED);
    macHelper.SetDeviceType (LorawanMacHelper::ED_A);
    NetDeviceContainer endDevicesNetDevices = helper.Install (phyHelper, macHelper, endDevices);

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
        if (i < confirmedNumber || (endDevices.GetN() == 1 && confirmedPercentage > 0))
        {
            edLorawanMac->SetMType (LorawanMacHeader::CONFIRMED_DATA_UP);
        }

        edLorawanMac->SetMaxNumberOfTransmissions (numberOfTransmissions);
        i++;
        }

    /*********************
   *  Create Gateways  *
   *********************/
    // NS_LOG_INFO ("Creating the gateway...");
    NodeContainer gateways;
    gateways.Create (nGateways);

    Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();
    allocator->Add (Vector (0, 0, 15.0));
    mobility.SetPositionAllocator (allocator);
    mobility.Install (gateways);

    // Create a netdevice for each gateway
    phyHelper.SetDeviceType (LoraPhyHelper::GW);
    macHelper.SetDeviceType (LorawanMacHelper::GW);
    helper.Install (phyHelper, macHelper, gateways);

    //Calculates the lowest SF for each device to use whilst ensuring connectivity.
    //macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel);


    NS_LOG_DEBUG( "Completed configuration, there are "
                    << endDevices.GetN() << " devices of which "
                    << confirmedNumber << " are confirmed.");

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

    /*********************************************
   *  Install applications on the end devices  *
   *********************************************/



    Time appStopTime = Seconds (simulationTime) + 5*Seconds(appPeriodSeconds);
    PeriodicSenderHelper periodicSenderHelper;
    periodicSenderHelper.SetPeriod (Seconds(appPeriodSeconds));

    periodicSenderHelper.SetPacketSize(basePacketSize);

    Ptr<RandomVariableStream> rv2 = CreateObjectWithAttributes<UniformRandomVariable> (
        "Min", DoubleValue (randomPSizeMin), "Max", DoubleValue (randomPSizeMax)); //randomPSizeMin and randomPSizeMax included

    periodicSenderHelper.SetPacketSizeRandomVariable(rv2); //As indicated by email from Martina

    ApplicationContainer appContainer = periodicSenderHelper.Install (endDevices);
    appContainer.Start (Seconds (0));
    appContainer.Stop (appStopTime);
      

    /************************
   * Install Energy Model *
   ************************/
    std::string rv = "ns3::UniformRandomVariable[Min=3.299|Max=" + std::to_string (E) + "]";
    Config::SetDefault ("ns3::CapacitorEnergySource::RandomInitialVoltage", StringValue (rv));

    CapacitorEnergySourceHelper capacitorHelper;
    capacitorHelper.Set ("Capacitance", DoubleValue (capacitance / 1000));
    capacitorHelper.Set ("CapacitorLowVoltageThreshold", DoubleValue (voltageThLow));
    capacitorHelper.Set ("CapacitorHighVoltageThreshold", DoubleValue (voltageThHigh));
    capacitorHelper.Set ("CapacitorMaxSupplyVoltageV", DoubleValue (3.3));
    // Assumption that the C does not reach full capacitance because of some
    // consumption in the OFF state
    // double Ioff = 0.0000055;
    // double RLoff = E/Ioff;
    // double ri = pow(E, 2)/eh;
    // double Req_off = RLoff;
    // double V0 = E;
    // if (eh != 0)
    // {
    //   Req_off = RLoff * ri / (RLoff + ri);
    //   V0 = E * Req_off / ri;
    //   }
    // NS_LOG_DEBUG ("Initial voltage [V]= " << V0 <<
    //               " RLoff " << RLoff <<
    //               " ri " << ri <<
    //               " R_eq_off " << Req_off);
    // Set initial voltage uniformy random in (0, E)
    capacitorHelper.Set ("RandomInitialVoltage", StringValue (rv));

    capacitorHelper.Set ("PeriodicVoltageUpdateInterval", TimeValue (Seconds (appPeriodSeconds)));
    // capacitorHelper.Set ("FilenameVoltageTracking", StringValue (filenameRemainingVoltage));

    //  // Basic Energy harvesting
    BasicEnergyHarvesterHelper harvesterHelper;
    harvesterHelper.Set ("PeriodicHarvestedPowerUpdateInterval", TimeValue (Seconds (1)));
    // // Constant harvesting rate
    double meanPowerDensity = eh;
    double variancePowerDensity = 0.08;
    std::string power = "ns3::NormalRandomVariable[Mean=" +
      std::to_string (meanPowerDensity) +
      "|Variance=" + std::to_string (variancePowerDensity) +
      "|Bound=" + std::to_string(meanPowerDensity)+"]";

    //constant power
    power = "ns3::UniformRandomVariable[Min=" +std::to_string(eh)+ + "|Max=" +std::to_string(eh)+"]";
    harvesterHelper.Set ("HarvestablePower", StringValue (power));

    LoraRadioEnergyModelHelper radioEnergy;
    radioEnergy.Set ("EnterSleepIfDepleted", BooleanValue (false));
    radioEnergy.Set ("TurnOnDuration", TimeValue (Seconds (0.3)));
    radioEnergy.Set ("TurnOnCurrentA", DoubleValue (0.015));
    radioEnergy.Set ("TxCurrentA", DoubleValue (0.028011));
    radioEnergy.Set ("IdleCurrentA", DoubleValue (0.000007));
    radioEnergy.Set ("RxCurrentA", DoubleValue (0.011011));
    radioEnergy.Set ("SleepCurrentA", DoubleValue (0.0000056));
    radioEnergy.Set ("StandbyCurrentA", DoubleValue (0.010511));
    radioEnergy.Set ("OffCurrentA", DoubleValue (0.0000055));

    // INSTALLATION ON EDs

    // install source on EDs' nodes
    EnergySourceContainer sources = capacitorHelper.Install (endDevices);
    // install device model
    DeviceEnergyModelContainer deviceModels = radioEnergy.Install (endDevicesNetDevices, sources);

    // Names::Add ("/Names/EnergySource", sources.Get(0));

    EnergyHarvesterContainer harvesters = harvesterHelper.Install (sources);

    Names::Add("Names/EnergyHarvester", harvesters.Get (0));
    Ptr<EnergyHarvester> myHarvester = harvesters.Get(0);
    myHarvester -> TraceConnectWithoutContext("TotalEnergyHarvested",
                                              MakeCallback(&OnEnergyHarvested));


    ///////////////////////
    // Connect tracesources
    ///////////////////////
    for (uint j = 0; j < nDevices; ++j)
      {
        Ptr<Node> node = endDevices.Get (j);
        Names::Add ("Names/nodeApp"+std::to_string(j), node->GetApplication (0));
        Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
        Ptr<EnergySource> capacitorES;
        for (uint k = 0; k < nDevices; ++k)
        {
          uint32_t nodeId = sources.Get (k)->GetNode ()->GetId ();
          if (nodeId == j)
            {
              capacitorES = sources.Get(k);
            }
          }
        Ptr<EndDeviceLoraPhy> phy = loraNetDevice->GetPhy ()->GetObject<EndDeviceLoraPhy> ();
        Ptr<EndDeviceLorawanMac> mac = loraNetDevice->GetMac ()->GetObject<EndDeviceLorawanMac> ();

        //Can't use this in large sims
        // mac->TraceConnectWithoutContext ("EnoughEnergyToTx",
        //                                       MakeCallback (&CheckEnoughEnergyCallback));

        // mac->TraceConnectWithoutContext("SendingNotPossible", MakeCallback(&DCRestricted));                                     
        // std::string name = "Names/nodeEdLoraPhy/"
        //   << std::to_string(endDevices.Get(j)->GetId()) << "/";

        //Can't use in large sims

        // phy->TraceConnect ("EndDeviceState", std::to_string (j),
        //                    MakeCallback (&OnEndDeviceStateChange));

        // phy->TraceConnect ("EndDeviceState", std::to_string (j),
        //                    MakeCallback (&OnEndDeviceTx));
        // capacitorES -> TraceConnect("RemainingVoltage", std::to_string(j),
        //                             MakeCallback (&OnRemainingVoltageChange));

        // ns3::Config::ConnectWithoutContext ("/Names/nodeApp"+std::to_string(j)+"/GeneratedPacket",
        //                                     MakeCallback (&OnGeneratedPacket));
        // NS_LOG_DEBUG ("Tracesources connected for ED " << j);
      }

    ///////////////////
    // Packet tracker
    ///////////////////
    LoraPacketTracker &tracker = helper.GetPacketTracker ();

    ////////////
    // Create NS
    ////////////

    NodeContainer networkServers;
    networkServers.Create (1);



    // Install the NetworkServer application on the network server
    NetworkServerHelper networkServerHelper;
    networkServerHelper.SetGateways (gateways);
    networkServerHelper.EnableCongestion(congestionEnabled);
    networkServerHelper.setPacketTracker(tracker);
    networkServerHelper.SetCongestion (congestionType);
    networkServerHelper.SetCongestionTrackingPeriod(Seconds(congestionPeriod));
    networkServerHelper.EnableWindow2SFChange(Window2SFSameAsW1Flag);

    networkServerHelper.SetEndDevices (endDevices);
    networkServerHelper.Install (networkServers);

    // Install the Forwarder application on the gateways
    ForwarderHelper forwarderHelper;
    forwarderHelper.Install (gateways);


    /****************
  *  Simulation  *
  ****************/


    Simulator::Stop (appStopTime + Seconds(10)); // adding more time so that tracker can calculate overall metric after 3 periods have passed from time frame of interest

    NS_LOG_INFO( "Congestion is calculated over " << congestionPeriod << " s intervals");
    NS_LOG_INFO( "Stop time is " << (appStopTime + 5*Seconds(appPeriodSeconds)+ Seconds(10)).GetSeconds() << " s ");
    NS_LOG_INFO ("Running simulation...");
    Simulator::Run ();

    //////////
    // Outputs
    //////////

    //exclude first 20 to allow acs to settle (no random + packet vals so should settle quickly)
    NS_LOG_INFO( "Computing over the period "<< 20*appPeriodSeconds << "s to "<< Seconds(simulationAppPeriods*appPeriodSeconds).GetSeconds() << "s");
    tracker.PrintPerformance(Seconds(20*appPeriodSeconds), Seconds(simulationAppPeriods*appPeriodSeconds), nDevices); 

    if(print)
    {


    
      int confirmed = 1;
      std::string pdr = tracker.CountMacPacketsGlobally (Seconds (0), Seconds (simulationTime));
      std::string cpsr = "0 0";
      if (confirmed > 0)
        {
          cpsr = tracker.CountMacPacketsGloballyCpsr (Seconds (0), Seconds (simulationTime));
        }
      // if (nDevices == 1)
      //   {
      //     std::cout << generatedPacketsAPP << " " << pdr << " " << cpsr << std::endl;
      //     std::vector<uint> v = tracker.CountMacPacketsPerEd(Seconds(0), Seconds(simulationTime), 0);
      //     std::cout << std::to_string(v.at(0)) << " " << std::to_string(v.at(1)) << std::endl;
      //   }

      std::vector<int> edsPhyPerf (3,0);
      std::vector<int> edPhyPerf (3,0);
      for (uint i = 0; i < nDevices; i++)
        {
          edPhyPerf = tracker.CountPhyPacketsPerEd (Seconds (0), appStopTime + Seconds(10),
                                                    endDevices.Get (i)->GetId ());

          edsPhyPerf.at (0) = edsPhyPerf.at (0) + edPhyPerf.at (0);
          edsPhyPerf.at (1) = edsPhyPerf.at (1) + edPhyPerf.at (1);
          edsPhyPerf.at (2) = edsPhyPerf.at (2) + edPhyPerf.at (2);
        }

      NS_LOG_DEBUG("\n" <<"From start PDR: " << pdr << "\nFrom start CPSR: " << cpsr << "\n");
      NS_LOG_DEBUG("PHY from start: "<< tracker.PrintPhyPacketsPerGw (Seconds (0), Seconds (simulationTime),
                                                gateways.Get (0)->GetId ()));
              NS_LOG_DEBUG("\n Generated app: " << generatedPacketsAPP << " Sent "
                << std::to_string (edsPhyPerf.at (0)) << " Recv "
                << std::to_string (edsPhyPerf.at (1)) << "  Interrupted "
                << std::to_string (edsPhyPerf.at (2)) << " Number no Energy events "
                << std::to_string(countNoTx));
              
      //generatedPacketsAPP will be higher than the counts as it doesn't have a time window, it runs till simulation::stop() which is 3 periods +10s more      

      if (print && nDevices == 1)
        {
          // Avoid non-existent files
          NS_LOG_DEBUG ("Create file if not done yet: " << stateChangeCallbackFirstCall);
          // Fix output files if not created
          if (stateChangeCallbackFirstCall)
            {
              const char *c = filenameState.c_str ();
              std::ofstream outputFile;
              // Delete contents of the file as it is opened
              outputFile.open (c, std::ofstream::out | std::ofstream::trunc);
              // Set the initial sleep state
              outputFile << 0 << " " << 0 << std::endl;
              NS_LOG_DEBUG ("Append initial state");
              stateChangeCallbackFirstCall = false;
              outputFile.close ();
            }

          if (energyConsumptionCallbackFirstCall)
            {
              const char *c = filenameEnergyConsumption.c_str ();
              std::ofstream outputFile;
              // Delete contents of the file as it is opened
              outputFile.open (c, std::ofstream::out | std::ofstream::trunc);
              // Set the initial sleep state
              outputFile << 0 << " " << 0 << std::endl;
              NS_LOG_DEBUG ("Append initially not enough energy, because never called");
              energyConsumptionCallbackFirstCall = false;
              outputFile.close ();
            }

          if (enoughEnergyCallbackFirstCall)
            {
              const char *c = filenameEnoughEnergy.c_str ();
              std::ofstream outputFile;
              // Delete contents of the file as it is opened
              outputFile.open (c, std::ofstream::out | std::ofstream::trunc);
              // Set the initial
              outputFile << 0 << " " << 0 << std::endl;
              NS_LOG_DEBUG ("Append initially not enough energy, because never called");
              enoughEnergyCallbackFirstCall = false;
              outputFile.close ();
            }
        }

      int drCounts[6] = {0};
      for (uint j = 0; j < nDevices; ++j)
        {
          
          Ptr<Node> node = endDevices.Get (j);
          Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> (); 
          Ptr<EndDeviceLorawanMac> mac = loraNetDevice->GetMac ()->GetObject<EndDeviceLorawanMac> ();
          uint8_t dr= mac->GetDataRate();

          drCounts[dr]++;
        // NS_LOG_DEBUG("Device " << (unsigned) j << " uses " << (unsigned) dr);

        }
        for (uint j=0; j<= 5 ; ++j)
          NS_LOG_DEBUG(drCounts[j]);
    }


    // Destroy simulator
    Simulator::Destroy ();

    return 0;
  }
