#include "ns3/core-module.h"

#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include "ns3/random-variable-stream.h"
#include <stdlib.h>
#include "ns3/ndnSIM-module.h"
#include "ns3/animation-interface.h"
#include "ns3/ns2-mobility-helper.h"
#include "ns3/vector.h"
#include "ns3/string.h"
#include "ns3/socket.h"
#include "ns3/double.h"
#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/command-line.h"
#include "ns3/mobility-model.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/mobility-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-interface-container.h"
#include <iostream>
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/wave-helper.h"
#include "ns3/wave-module.h"
#include "ns3/ndnSIM/NFD/daemon/face/generic-link-service.hpp"
#include "model/ndn-v2v-net-device-transport.hpp"
#include <algorithm>
#include <vector>
//#include "onoff-application.h"
#include "ns3/applications-module.h"
#include "ctime"
#include "random"


namespace ns3{
NS_LOG_COMPONENT_DEFINE ("FloodingVNDN");

static std::string contentPrefixList [3] ={"/appType1","/appType2","/appType3"}; //List of content types that are provided by selected producers
static vector<string> ConsumercontentPrefixList; //={"/content1","/content2","/content3"}; //List of content types that are for consumers based on randomly selected for producres 
static const std::string effectiveFStrategy = "/localhost/nfd/strategy/flooding-vndn";//flooding-vndn";//flooding-vndn
static const uint32_t minConsumerStartTime = 1; //Minimum time for consumer application to start
static const uint32_t maxConsumerStartTime = 4; //Maximum time for consumer application to start

void installSUMOMobility(std::string TCLFileName)
{
  //Ns2MobilityHelper ns2 = Ns2MobilityHelper ("TCL/Vehicles/man100v60ms.tcl");
  Ns2MobilityHelper ns2 = Ns2MobilityHelper (TCLFileName);
  ns2.Install();
}

// NetDevice Setup - Start
void installWifi(NodeContainer &nodes, NetDeviceContainer &devices)
{
  //Everything created after will have initial queue capacity of 50

  // Setup propagation models
  YansWifiChannelHelper wifiChannel; // = YansWifiChannelHelper::Default ();
  wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::RangePropagationLossModel",
                                  "MaxRange", DoubleValue(250));
  //double freq = 5.9e9;
  //wifiChannel.AddPropagationLoss ("ns3::LogDistancePropagationLossModel", "Frequency", DoubleValue (freq));
  wifiChannel.AddPropagationLoss ("ns3::NakagamiPropagationLossModel");//, "m2",DoubleValue(0.75));
  

  // the channel
  Ptr<YansWifiChannel> channel = wifiChannel.Create();
  
  //Wave
  YansWavePhyHelper wavePhy =  YansWavePhyHelper::Default ();
  wavePhy.SetChannel (channel);
  //wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);
  wavePhy.Set ("TxPowerStart",DoubleValue (16));
  wavePhy.Set ("TxPowerEnd", DoubleValue (16));
  //wavePhy.Set ("TxPowerLevels", UintegerValue (1));

  //802.11p
  YansWifiPhyHelper wifiPhyHelper = YansWifiPhyHelper::Default();
  wifiPhyHelper.SetChannel(channel); //SetChannel(wifiChannel.Create());
  //wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);
  wifiPhyHelper.Set ("TxPowerStart",DoubleValue (16));
  wifiPhyHelper.Set ("TxPowerEnd", DoubleValue (16));
  
  //wifiPhyHelper.Set ("TxPowerLevels", UintegerValue (8));
  
  // Setup WAVE PHY and MAC
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();

  QosWaveMacHelper waveMac = QosWaveMacHelper::Default ();
  WaveHelper waveHelper = WaveHelper::Default ();
    
  // Setup 802.11p stuff
  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode", StringValue ("OfdmRate6MbpsBW10MHz"),
                                      "ControlMode", StringValue ("OfdmRate6MbpsBW10MHz"));
  
  // Setup WAVE-PHY stuff
  waveHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue ("OfdmRate6MbpsBW10MHz"),
                                      "ControlMode",StringValue ("OfdmRate6MbpsBW10MHz"));
   
  //802.11p
  devices = wifi80211p.Install (wifiPhyHelper, wifi80211pMac, nodes);

  //WAVE
  //devices = waveHelper.Install (wavePhy, waveMac, nodes);    
}


/* ****************  Creating Face - Start   *************** */
std::string
constructFaceUri(Ptr<NetDevice> netDevice)
{
  std::string uri = "netdev://";
  Address address = netDevice->GetAddress();
  if (Mac48Address::IsMatchingType(address)) {
    uri += "[" + boost::lexical_cast<std::string>(Mac48Address::ConvertFrom(address)) + "]";
  }

  return uri;
}

shared_ptr<ndn::Face>
V2VNetDeviceFaceCallback (Ptr<Node> node, Ptr<ndn::L3Protocol> ndn, 
			  Ptr<NetDevice> netDevice)
{
  NS_LOG_DEBUG("Creating V2V NetDevice Face on node " << node->GetId());

  // Create an ndnSIM-specific transport instance
  ::nfd::face::GenericLinkService::Options opts;
  opts.allowFragmentation = true;
  opts.allowReassembly = true;
  opts.allowCongestionMarking = true;
  
  auto linkService = make_unique<::nfd::face::GenericLinkService>(opts);
    
  auto v2vTransport = make_unique<ndn::V2VNetDeviceTransport>(node, netDevice,
                                                   constructFaceUri(netDevice),
                                                   "netdev://[ff:ff:ff:ff:ff:ff]");

  auto face = std::make_shared<ndn::Face>(std::move(linkService), std::move(v2vTransport));
  face->setMetric(1);

  ndn->addFace(face);
  NS_LOG_LOGIC("Node " << node->GetId() << ": added Face as face #"
                       << face->getLocalUri());
  
  return face;
}

/* **************** creating Face - End ***************** */

void installNDN(NodeContainer &nodes)
{
  ndn::StackHelper ndnHelper;
  NS_LOG_UNCOND ("Install NDN");  
  ndnHelper.AddFaceCreateCallback(WifiNetDevice::GetTypeId (), MakeCallback(V2VNetDeviceFaceCallback));
  //ndnHelper.SetOldContentStore("ndn::cs::lru", "MaxSize", "1000");
  ndnHelper.setCsSize(1000);
  ndnHelper.setPolicy("nfd::cs::lru");
  ndnHelper.SetDefaultRoutes(true); 
  ndnHelper.Install(nodes);  
  
  for(uint32_t i = 0; i < sizeof(contentPrefixList)/sizeof(contentPrefixList[0]); i++)
  {
    ndn::StrategyChoiceHelper::InstallAll(contentPrefixList[i], effectiveFStrategy); 
    std::cout << std::endl << effectiveFStrategy;
  }  
}


void installConsumer(NodeContainer &nodes, string contentPrefix)
{
  ndn::AppHelper helper("ns3::ndn::ConsumerCbr");
  helper.SetAttribute("Frequency", DoubleValue (10.0)); 
  //helper.SetAttribute("Frequency", DoubleValue (5.0)); 
  helper.SetPrefix(contentPrefix);

 /* ******* To randomly start consumer application on the selected nodes ******* */
  
  ns3::RngSeedManager::SetSeed (std::rand() % maxConsumerStartTime + 1); 
  ns3::RngSeedManager::SetRun (10); 
  ns3::Ptr<ns3::UniformRandomVariable> randomStart = ns3::CreateObject<ns3::UniformRandomVariable> ();
  randomStart->SetAttribute ("Min", ns3::DoubleValue (minConsumerStartTime));
  randomStart->SetAttribute ("Max", ns3::DoubleValue (maxConsumerStartTime));
  double appRandomStartTime = randomStart->GetValue();
  std::cout << std::endl <<"Jitter | " << nodes.Get(0)->GetId() << ": " << appRandomStartTime;
  helper.SetAttribute("StartTime",TimeValue(ns3::Seconds(appRandomStartTime)));

 /* ******* To randomly start consumer application on the selected nodes ******* */

  helper.Install(nodes); 
  std::cout << std::endl << "New Consumer:" << nodes.Get(0)->GetId() << "| Content Prefix :" << contentPrefix;
}


void installProducer(NodeContainer &nodes, string contentPrefix)
{
  ndn::AppHelper producerHelper("ns3::ndn::Producer");
  producerHelper.SetPrefix(contentPrefix);
  producerHelper.SetAttribute("Freshness", TimeValue(Seconds(4.0)));
  producerHelper.Install(nodes); 
  std::cout << std::endl << "New Producer:" << nodes.Get(0)->GetId() << "| Content Prefix :" << contentPrefix;
}

void installProducer1(NodeContainer &c)
{
  ndn::AppHelper producerHelper("ns3::ndn::Producer");
  producerHelper.SetPrefix("/video");
  producerHelper.SetAttribute("Freshness", TimeValue(Seconds(4.0)));
  producerHelper.Install(c);
  NS_LOG_INFO("Producer installed on node " << c.Get(4)->GetId());

}

void VndnForwardingStrategyExperiment(int expriementRunNumber, std::string strategyName, std::string SimulationRunName, uint8_t numNodes, uint8_t producerPercentage, uint8_t totalConsumerPerConetentType, std::string TCLFileName, int simulationEnd, std::string PacketDropResultName)
{
  //std::cout << std::endl << "expriementRunNumber :::::::::" << expriementRunNumber << std::endl;  
  NS_LOG_UNCOND ("Simulation Run of: " << SimulationRunName);
  NodeContainer nodes;
  nodes.Create(numNodes);
  MobilityHelper mobility;

  //installMobility(mobility, nodes, simulationEnd);
  installSUMOMobility(TCLFileName);

  NetDeviceContainer netDevices,netDevices2;
  installWifi(nodes, netDevices);  

  installNDN(nodes);

  std::fstream ExperiementInfo;  // this the file that will hold info about the  new siumlation run name
  std::string ExperiementInfoFileName="ExperiementsInfo.txt";  //change the file name according to the experiement info.
  ExperiementInfo.open(ExperiementInfoFileName, std::ofstream::app);      
  
  /* ************************    Random Selection of Consumers and Producers for each content type  ***************************** */
  std::string consumerNodesFile = "ConsumerProducerNodes/Consumer-" + std::to_string(numNodes) + "-" + std::to_string(expriementRunNumber) + ".txt";
  std::string producerNodesFile = "ConsumerProducerNodes/Producer-" + std::to_string(numNodes) + "-" + std::to_string(expriementRunNumber) + ".txt";

  // Method 1: read consumer and producer nodes info. from a file
  // this method is used when we want use same nodes (as content consumers and produers) as used by other application and forwarding strategy 
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /*
  std::vector<int> producerNodeList;
  std::vector<int> consumerNodeList;

  // Consumers 
  vector<vector<string>> consumerNodes;
  vector<string> consumerNodesRow;
  string consumerlineInfo, consumerWordInfo;
  
  fstream consumerfile (consumerNodesFile, ios::in);
  if(consumerfile.is_open())
  {
    while(getline(consumerfile, consumerlineInfo))
    {
      consumerNodesRow.clear();
      
      stringstream str(consumerlineInfo);
      
      while(getline(str, consumerWordInfo, ','))
      consumerNodesRow.push_back(consumerWordInfo);
      consumerNodes.push_back(consumerNodesRow);
    }
  }
  else
  {
    std::cout<<"Could not open the file (Consumers)\n";
    std::exit(0);
  }
  
  
  for(int i=0;i<consumerNodes.size();i++)
  {
    string ContentPrefix;
    int nodeID;
        
    for(int j=0;j<consumerNodes[i].size() - 1 ;j++)
    {      
      nodeID = std::stoi(consumerNodes[i][j]);
      ContentPrefix = consumerNodes[i][j+1];
      NodeContainer consumers;
      consumers.Add(nodes.Get((uint32_t)nodeID));    
      installConsumer(consumers,ContentPrefix);
      std::cout<< "\n Node: " << nodeID << " as consumer (ContentType: " << ContentPrefix << ")";
      consumerNodeList.push_back(nodeID);   
    }    
    //cout<< "Node: " << nodeID << (uint32_t) nodeID + 5 << " Prefix: "<< ContentPrefix;
    std::cout<<"\n";
  }


  // Producers 
  vector<vector<string>> producerNodes;
  vector<string> producerNodesRow;
  string producerlineInfo, producerWordInfo;
  
  fstream producerfile (producerNodesFile, ios::in);
  if(producerfile.is_open())
  {
    while(getline(producerfile, producerlineInfo))
    {
      producerNodesRow.clear();
      
      stringstream str(producerlineInfo);
      
      while(getline(str, producerWordInfo, ','))
      producerNodesRow.push_back(producerWordInfo);
      producerNodes.push_back(producerNodesRow);
    }
  }
  else
  {
    std::cout<<"Could not open the file (Producers)\n";
    std::exit(0);
  }
  
  
  for(int i=0;i<producerNodes.size();i++)
  {
    string ContentPrefix;
    int nodeID;
        
    for(int j=0;j<producerNodes[i].size() - 1 ;j++)
    {      
      nodeID = std::stoi(producerNodes[i][j]);
      ContentPrefix = producerNodes[i][j+1];
      NodeContainer producers;
      producers.Add(nodes.Get((uint32_t)nodeID));    
      installConsumer(producers,ContentPrefix);
      std::cout<< "\n Node: " << nodeID << " as producer (ContentType: " << ContentPrefix << ")";
      producerNodeList.push_back(nodeID);     
    }    
    //cout<< "Node: " << nodeID << (uint32_t) nodeID + 5 << " Prefix: "<< ContentPrefix;
    std::cout<<"\n";
  }

  //Print list of Consumers and Producers
  std::cout << "\n Consumers: ";
  for (auto j = consumerNodeList.begin(); j != consumerNodeList.end(); j++ )
  {
    std::cout << *j << " ";
  }  
  std::cout << "\n";

  std::cout << "\n Producers: ";
  for (auto j = producerNodeList.begin(); j != producerNodeList.end(); j++ )
  {
    std::cout << *j << " ";
  }  
  std::cout << "\n";  
  */
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////


  // Method 2: To selecte nodes as consumer and procuder randlomly
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////  
  
  std::string consumerNodesFile1 ="ConsumerProducerNodes/Consumer-" + std::to_string(numNodes) + "-" + std::to_string(expriementRunNumber) + ".txt";
  std::string producerNodesFile1 = "ConsumerProducerNodes/Producer-" + std::to_string(numNodes) + "-" + std::to_string(expriementRunNumber) + ".txt";
  std::fstream consumerNodesInfo;
  std::fstream producerNodesInfo;
  consumerNodesInfo.open(consumerNodesFile1, std::ofstream::out | std::ofstream::trunc);  
  producerNodesInfo.open(producerNodesFile1, std::ofstream::out | std::ofstream::trunc);  


  //Selected radom nodes as Conumers and other as Producers
  uint32_t totalContentTypes = sizeof(contentPrefixList)/sizeof(contentPrefixList[0]);
  uint32_t contentTypeNo = 0;

  //uint32_t totalSimulationConsumersNodes = ((consumerPercentage*numNodes)/100) * totalContentTypes;
  uint32_t totalSimulationConsumersNodes = totalConsumerPerConetentType * totalContentTypes;
  //std::vector<int>  totalSimulationConsumersNodes;
  //uint32_t totalConsumerConetentNodes = 1; // this is in case of single consumer
  uint32_t totalConsumerConetentNodes = totalConsumerPerConetentType;//(consumerPercentage*numNodes)/100; // in case of multiple consumers
  //uint32_t consumerNodeList[totalSimulationConsumersNodes];
  std::vector<int> consumerNodeList;
  uint32_t consumerNodeContentList[totalContentTypes][totalConsumerConetentNodes];

      
  uint32_t totalSimulationProducersNodes = ((producerPercentage*numNodes)/100) * totalContentTypes;
  //std::vector<int>  totalSimulationProducersNodes;
  //uint32_t totalProducerNodes = 1; // this is in case of single producer
  uint32_t totalProducerCotentNodes = 3;//producerPercentage; // in case of multiple producers
  //uint32_t producerNodeList[totalSimulationProducersNodes];
  std::vector<int> producerNodeList;
  uint32_t producerNodeContentList[totalContentTypes][totalProducerCotentNodes];

  //ExperiementInfo << "Consumer Percentage: " << std::to_string(consumerPercentage) << "\n";
  ExperiementInfo << "Total Consumers in Simulation: " << std::to_string(totalSimulationConsumersNodes) << "\n";
  ExperiementInfo << "Total Consumers Per Conetent: " << std::to_string(totalConsumerConetentNodes) << "\n";
  ExperiementInfo << "otal Producers in Simulation: " << std::to_string(totalSimulationProducersNodes) << "\n";
  ExperiementInfo << "Total Producers Per Conetent: " << std::to_string(totalProducerCotentNodes) << "\n";
  

  for (contentTypeNo = 0; contentTypeNo < sizeof(contentPrefixList)/sizeof(contentPrefixList[0]); contentTypeNo++ )
  {
    // === Content type consumers ad producers random selection ===============================================
      int selectedNode;
    //srand((unsigned) time(0));
    //std::cout << std::endl << "totalConsumerConetentNodes : " << totalConsumerConetentNodes << std::endl;
  
    // Consumers #####################
    for(uint32_t i = 0; i < totalConsumerConetentNodes;)
    {  
      selectedNode = std::rand() % numNodes;    
      std::cout << std::endl << "selected nodes : " << selectedNode << std::endl;
      bool isduplicated=false;

      //Check if the new random consumer is already selected as CONSUMER node
      for (auto j = consumerNodeList.begin(); j != consumerNodeList.end(); j++ )
      { 
        if ( *j == selectedNode)
        {
          isduplicated = true;
          std::cout << std::endl << "Already exist in Consumer List: " << selectedNode << std::endl;  
          break;
        } 
      }

      if(!isduplicated)
      {      
        //consumerNodeList[i] = selectedNode;
        consumerNodeList.push_back(selectedNode);
        consumerNodeContentList [contentTypeNo][i] = selectedNode;
        consumerNodesInfo << std::to_string(selectedNode) << "," << contentPrefixList[contentTypeNo] << "\n";
        i++;
      }
    }

    std::cout << std::endl << "selected nodes as consumers are : " << std::endl;
    for (auto j = consumerNodeList.begin(); j != consumerNodeList.end(); j++ )
    { 
      std::cout << *j << "  ";
    }

    //Producer #####################
    uint32_t ContentProducerCount = 0;
    for(uint32_t i = 0; i < totalProducerCotentNodes;)
    {
      selectedNode = std::rand() % numNodes;      
      std::cout << std::endl << "selected Producer is: " << selectedNode << std::endl;

      bool isduplicated=false;

      //Check if the new random producer is already selected as PRODUCER node
      for (auto j = producerNodeList.begin(); j != producerNodeList.end(); j++ )
      { 
        if ( *j == selectedNode)
        {         
          isduplicated = true;
          std::cout << std::endl << "already exist in Producer List: " << selectedNode << std::endl;   
          //expriementRunNumber = expriementRunNumber + 1;          
          break;
        }  
      }

      //Check if the new random producer is already selected as CONSUMER node
      for (auto j = consumerNodeList.begin(); j < consumerNodeList.end(); j++ )
      { 
        if ( *j == selectedNode)
        {
          isduplicated = true;
          std::cout << std::endl << "Producer Already exist in Consumer List: " << selectedNode << std::endl;   
          //expriementRunNumber = expriementRunNumber + 1;
          break;
        } 
      }
      //-------------------------------------------------------------------------------------------
      //check if the slected node as producer is at least of 750m in distance
      double DistanceConsumerProducer = 0.0;
      bool isDistanceOk = false;

       std::cout << std::endl << "Distance Check1" << " " << ContentProducerCount;
      
      if(ContentProducerCount != 0)
      {
        for (uint32_t j = 0; j < ContentProducerCount; j++ )
        { 
          uint32_t PreviousNodeContentProducer= producerNodeContentList[contentTypeNo][ContentProducerCount - 1];
          //Location of new selected node as a Producer.
          std::cout << std::endl << "Distance Check2" << " XX " << PreviousNodeContentProducer << " && " << contentTypeNo << " :: " << contentPrefixList[contentTypeNo];
          auto ProducerNodeNew = ns3::NodeList::GetNode(selectedNode);
          auto ProducerNodeNewMobility= ProducerNodeNew->GetObject<ns3::MobilityModel>();
          if (ProducerNodeNewMobility == 0)
          {
            NS_FATAL_ERROR("Mobility model has to be installed on the node: " << selectedNode);        
          }
          ns3::Vector3D ProducerPositionNew = ProducerNodeNewMobility->GetPosition();
          std::cout << std::endl << "Distance Check3";
          //Location of previous Selected node as a producer.
          auto ProducerNodeOld = ns3::NodeList::GetNode(PreviousNodeContentProducer); 
          std::cout << std::endl << "Distance Check4";
          auto ProducerNodeOldMobility= ProducerNodeOld->GetObject<ns3::MobilityModel>();
          if (ProducerNodeOldMobility == 0)
          {
            NS_FATAL_ERROR("Mobility model has to be installed on the node: " << j); 
            std::cout << std::endl << "Distance Check5";       
          }
          ns3::Vector3D ProducerPositionOld = ProducerNodeOldMobility->GetPosition();
          std::cout << std::endl << "Distance Check6";
          
          DistanceConsumerProducer = ns3::CalculateDistance(ProducerPositionOld,ProducerPositionNew);
          std::cout << std::endl << "Distance Check7";
          std::cout << std::endl <<"Distance Previous Selected Producer (" << PreviousNodeContentProducer << ") and New Selected Producer (" << selectedNode << ") " << DistanceConsumerProducer;
          if(DistanceConsumerProducer > 650.0)
          {
            isDistanceOk = true;
            break;
          }
        }  
      }  
    
      if (ContentProducerCount != 0)
      {
        if(!isduplicated && isDistanceOk)
        {      
          //producerNodeList[i] = selectedNode;
          producerNodeList.push_back(selectedNode);
          producerNodeContentList[contentTypeNo][i] = selectedNode;  
          ContentProducerCount++;   
          producerNodesInfo << std::to_string(selectedNode) << "," << contentPrefixList[contentTypeNo] << "\n";;
          i++;
        }
      }
      else
      {
        if(!isduplicated)
        {      
          //producerNodeList[i] = selectedNode;
          producerNodeList.push_back(selectedNode);
          producerNodeContentList[contentTypeNo][i] = selectedNode;  
          producerNodesInfo << std::to_string(selectedNode) << "," << contentPrefixList[contentTypeNo] << "\n";;        
          ContentProducerCount++;   
          i++;
        }
      }
      //-------------------------------------------------------------------------------------------      
    }

    std::cout << std::endl << "selected nodes as producers are : " << std::endl;
    for (auto j = producerNodeList.begin(); j < producerNodeList.end(); j++ )
    {       
      std::cout << *j << "  ";
    }

  } 
 
  // Install Producer and consumer appliations on randomly selected nodes as producers and consumers for each content type
  
  //Producers #########################
   
  for (uint32_t i = 0; i < totalContentTypes; i++ )
  { 
    for (uint32_t j = 0; j < totalProducerCotentNodes; j++ )
    { 
      NodeContainer producers;    
      producers.Add(nodes.Get(producerNodeContentList[i][j]));       
      //ExperiementInfo << "ProducerNode: " << std::to_string(producerNodeContentList[i][j]) << "|ContentType:" << contentPrefixList[i] <<"\n"; 
      installProducer(producers,contentPrefixList[i]);      
    }
  }  
   
  //Consumers ######################### 
  for (uint32_t i = 0; i < totalContentTypes; i++ )
  { 
    for (uint32_t j = 0; j < totalConsumerConetentNodes; j++ )
    {
      NodeContainer consumers;
      consumers.Add(nodes.Get(consumerNodeContentList[i][j]));
      //ExperiementInfo << "ConsumerNode: " << std::to_string(consumerNodeContentList[i][j]) << "|ContentType:" << contentPrefixList[i] <<"\n"; 
      installConsumer(consumers,contentPrefixList[i]);
    }    
  }

  for (uint32_t i = 0; i < totalContentTypes; i++ )
  { 
    ExperiementInfo << "\n" << "# ContentType:" << contentPrefixList[i] ; 
    ExperiementInfo << "\n" << "Producers Nodes: ";
    for (uint32_t j = 0; j < totalProducerCotentNodes; j++ )
    {       
      ExperiementInfo << std::to_string(producerNodeContentList[i][j]) << "  ";            
    }

    ExperiementInfo << "\n" << "Consumers Nodes: ";
    for (uint32_t j = 0; j < totalConsumerConetentNodes; j++ )
    {
      ExperiementInfo << std::to_string(consumerNodeContentList[i][j]) << "  ";       
    }    
  }
    
  producerNodeList.clear();
  consumerNodeList.clear();
  consumerNodesInfo.close();
  producerNodesInfo.close();
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////  

  ExperiementInfo.close();

  // End simulation
  Simulator::Stop(Seconds(simulationEnd));
  
  // Network Animation (NetAnim)
  std::string animFile = "SimulationResults/" + strategyName + "-NetAnim-" + SimulationRunName + ".xml";
  AnimationInterface anim(animFile);

  // Collecting the output results
  ndn::L3RateTracer::InstallAll("SimulationResults/" + strategyName + "-L3Trace-" + SimulationRunName + ".txt", Seconds(1.0));
  ndn::AppDelayTracer::InstallAll("SimulationResults/" + strategyName + "-AppTrace-" + SimulationRunName + ".txt");

  // Start Simulation
  Simulator::Run();
  // return 0;
}

int main (int argc, char *argv[])
{
  //Note: 
  /*
    You need to go >> Forwarder.hpp file, then >> getCSTraceFile() mehtod to change
    the varaibale "strategyName" that holds the name of the current running Forwarding strategy.
    the name you assign to that variable will be in the beginning of the results files
    which is helpful to distinguish between the oupput file results from running different forwarding stratgies. 
  */
  
  std::string strategyName ="BNF"; //basic NDN-based flooding
  std::string outputResultsFolder ="SimulationResults/";  
  std::string TCLFileName;
  std::string SimulationNameFile = "SimulationRunNameFile.txt"; // a file that sotre the name of current simulation run  
  
  int simulationEnd = 300;     
  int MaximumRunTimes = 25;
  

  //Simulation scenarios
  //------------------------------------------------
  // Change Vehicles Number 
  uint8_t numNodes [] = {40,60,80,100,120};
  std::string TCLFilesPath = "TCL/Vehicles/"; 
  uint8_t VehicleSpeed = 60;
  uint8_t producerPercentage = 3; // we used only 3 nodes, not 3% of numNodes as producers
  uint8_t consumerPercentage = 5;
  std::string ExperiementName = "VehicelNumberChange -  Multiple Producers (3 nodes) and Consumers (5% of numNodes)";     
  //------------------------------------------------
   
  //------------------------------------------------
  /*
  // Change Vehicle Speed 
  uint8_t VehicleSpeed [] = {40,50,60,70,80};
  std::string TCLFilesPath = "TCL/Speed/";
  uint8_t numNodes = 80;  
  uint8_t producerPercentage = 3; // we used only 3 nodes, not 3% of numNodes as producers
  uint8_t consumerPercentage = 10;
  std::string ExperiementName = "VehicelNumberChange -  Multiple Producers (3 nodes) and Consumers (5% of numNodes)";  
  */
  //------------------------------------------------


    
  std::fstream ExperiementInfo;  // this the file that will hold info about the  new siumlation run name
  std::string ExperiementInfoFileName = "ExperiementsInfo.txt";
  ExperiementInfo.open(ExperiementInfoFileName, std::ofstream::out | std::ofstream::trunc);      
  ExperiementInfo << "StrategyName: " << strategyName << "\n";
  ExperiementInfo << "ExperiementName: " << ExperiementName << "\n";
  ExperiementInfo.close();
   
  //int arrayLength = sizeof(VehicleSpeed)/sizeof(VehicleSpeed[0]); // uncomment if the speed change scenario is used/wanted
  int arrayLength = sizeof(numNodes)/sizeof(numNodes[0]); // vehilces change scenario
  std::cout << "consumerPercentage array Length: " << arrayLength << std::endl;
  for(int i = 0; i < arrayLength; i++) // this for loop for the main simulation run parameter such as Vehicles Numbers Change, Vehilce Speed Change, Consumer change, etc
  {
    ExperiementInfo.open(ExperiementInfoFileName, std::ofstream::app);
    ExperiementInfo << "\n" << "================================================== " << "\n";
    ExperiementInfo << "Multiple Consumers and Producers " << "\n";  
    ExperiementInfo << "Vehicles Speed: " << std::to_string(VehicleSpeed) << "\n"; 
    ExperiementInfo << "Nodes Number: " << std::to_string(numNodes[i]) << "\n"; 
    //  we need to specify the TCL file for vehicle mobility model
    TCLFileName = TCLFilesPath + "Manhattan" + std::to_string(numNodes[i]) + "v" + std::to_string(VehicleSpeed)+ "kmph.tcl";          
    ExperiementInfo << "TCL-Mobility: " << TCLFileName << "\n";
    ExperiementInfo << "Simulation Run Time: " << simulationEnd << " seconds" <<"\n";
    ExperiementInfo << "Number of Simulation Runs: " << MaximumRunTimes <<"\n";
    ExperiementInfo << "================================================== " << "\n"; 
    ExperiementInfo.close(); 
    //--------------------------------------------------------------------------------------------------------   
    
    for(int j = 1; j <= MaximumRunTimes; j++) // this for the simulation run times (How many times you want to run the simulation)
    {
      
      // Get current Time and Date   
      time_t now = time(0); // Current Date and time 
      std::string dt = ctime(&now); // convert now to string form
      ExperiementInfo.open(ExperiementInfoFileName, std::ofstream::app);
      ExperiementInfo << "\n" << "----------------------------------------" << "\n"; 
      ExperiementInfo << "ExperiementRunTime: " << std::to_string(j) <<"\n";      
      ExperiementInfo << "ExperiementDateTime: " << dt ;//<<"\n";
      ExperiementInfo << "----------------------------------------" << "\n"; 
      ExperiementInfo.close();
      
      //  we need to give the simulation run a name which will be the name of the simulation results files      
      //std::string SimulationRunName = strategyName + "-" + std::to_string(numNodes[i]) + "v" + std::to_string(VehicleSpeed) + "kmph" + std::to_string(producerPercentage) + "p" + std::to_string(consumerPercentage) + "c-Multiple-Man250-" + std::to_string(j);       
      std::string SimulationRunName = std::to_string(numNodes[i]) + "v" + std::to_string(VehicleSpeed) + "kmph" + std::to_string(producerPercentage) + "p" + std::to_string(consumerPercentage) + "c-Multiple-Man250-" + std::to_string(j);       
      //std::string CacheHitResultName = "SimulationResults/" + strategyName + "-CSHitMiss-" + SimulationRunName + ".txt";
      std::string CacheHitResultName = "SimulationResults/" + SimulationRunName + "-CSHitMiss.txt";
      std::string PacketDropResultName = "SimulationResults/" + SimulationRunName + "-PacketDrop.txt";
      std::string HelloPacketResultName = "SimRSimulationResultsesults/" + SimulationRunName + "-HelloPacket.txt";

      //std::string  ForwardingMechanismFileName = "SimulationResults/" + SimulationRunName + "-ForwardingMechanism.txt";      
      
      //--------------------------------------------------------------------------------------------------------
      //  we need to creat a file to store the Simulation Run Name to be used by other scripts
      std::fstream NewSimulationRunNameFile;  // this the file that will hold the new siumlation run name    
      NewSimulationRunNameFile.open(SimulationNameFile, std::ofstream::out | std::ofstream::trunc);      
      NewSimulationRunNameFile << SimulationRunName << "\n";
      NewSimulationRunNameFile.close();

      /*
      std::fstream HelloPacketFileName;  // this the file that will hold the new siumlation run name
      HelloPacketFileName.open(HelloPacketResultName, std::ofstream::out | std::ofstream::trunc);
      HelloPacketFileName << "Time" << ", " << "Rx-Node Id" << ", " << "In/Out" << ", " 
                          << "FaceType" << ", " << "PrefixName" << ", " << "Tx-Node Id" << "\n"; 
      HelloPacketFileName.close();
      */
      // -----------------------------------------------------------------------------------------

      //  Call VndnForwardingStrategyExperiment() to start the
      uint32_t totalConsumerPerConetentType = (consumerPercentage*numNodes[i])/100; 
      VndnForwardingStrategyExperiment(j , strategyName, SimulationRunName, numNodes[i],producerPercentage, totalConsumerPerConetentType, TCLFileName, simulationEnd, PacketDropResultName);
      //VndnForwardingStrategyExperiment(std::string strategyName, std::string SimulationRunName, uint8_t numNodes, std::string TCLFileName)     
      Simulator::Destroy();
    }    
  }

  return 0;
}

} // namespace ns3

int
main(int argc, char* argv[])
{
  srand((unsigned) time(NULL)); // this is to make rand() generats randoms value each time it called during the simulation  
  return ns3::main(argc, argv);
}