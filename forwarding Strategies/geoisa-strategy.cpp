/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014-2019,  Regents of the University of California,
 *                           Arizona Board of Regents,
 *                           Colorado State University,
 *                           University Pierre & Marie Curie, Sorbonne University,
 *                           Washington University in St. Louis,
 *                           Beijing Institute of Technology,
 *                           The University of Memphis.
 *
 * This file is part of NFD (Named Data Networking Forwarding Daemon).
 * See AUTHORS.md for complete list of NFD authors and contributors.
 *
 * NFD is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * NFD is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * NFD, e.g., in COPYING.md file.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "isa-vndn-strategy.hpp"
#include "algorithm.hpp"
#include "common/logger.hpp"
#include "ns3/node-list.h"
#include "ns3/mobility-model.h"
#include "ns3/node.h"
#include <ndn-cxx/lp/tags.hpp>
#include <vector>
#include "ns3/double.h"
#include "ns3/random-variable-stream.h"
#include "ns3/config.h"
#include "interest.hpp"
#include "daemon/table/pit.hpp"
#include "model/ndn-l3-protocol.hpp"
#include <ndn-cxx/lp/pit-token.hpp>
#include "forwarder.hpp"
#include <ndn-cxx/util/scheduler.hpp>
#include <ns3/vector.h>
#include <cmath>
#include "random"

namespace nfd {
namespace fw {

NFD_REGISTER_STRATEGY(ISAVndnStrategy);

NFD_LOG_INIT(ISAVndnStrategy);


ISAVndnStrategy::ISAVndnStrategy(Forwarder& forwarder, const Name& name)
  : Strategy(forwarder)
  , ProcessNackTraits(this)
{
  ParsedInstanceName parsed = parseInstanceName(name);
  if (!parsed.parameters.empty()) {
    NDN_THROW(std::invalid_argument("ISAVndnStrategy does not accept parameters"));
  }
  if (parsed.version && *parsed.version != getStrategyName()[-1].toVersion()) {
    NDN_THROW(std::invalid_argument(
      "ISAVndnStrategy does not support version " + to_string(*parsed.version)));
  }
  this->setInstanceName(makeInstanceName(name, getStrategyName()));

m_retxTime = ns3::Seconds(0.05);

}

ISAVndnStrategy::~ISAVndnStrategy()
{
}


void
ISAVndnStrategy::doPropagateAfterDelay(FaceId inFaceId, weak_ptr<pit::Entry> pitEntryWeak)
{
  Face* inFace = this->getFace(inFaceId);
  if (inFace == nullptr) {
    return;
  }
  shared_ptr<pit::Entry> pitEntry = pitEntryWeak.lock();
  if (pitEntry == nullptr) {
    return;
  }
  auto inRecord = pitEntry->getInRecord(*inFace);
  if (inRecord == pitEntry->in_end()) {
    return;
  }
  const Interest& interest = inRecord->getInterest();
  const fib::Entry& fibEntry = this->lookupFib(*pitEntry);

  for (const auto& nexthop : fibEntry.getNextHops()) {
    Face& face = nexthop.getFace();
    if (!wouldViolateScope(*inFace, interest, face)) { 
      this->sendInterest(pitEntry, FaceEndpoint(face, 0), interest);
      NFD_LOG_DEBUG("Interest Name:" << interest.getName() << " has been sent after propagation delay time is finish");
      break;
    }
  }
}


void
ISAVndnStrategy::afterReceiveInterest(const FaceEndpoint& ingress, const Interest& interest,
                                     const shared_ptr<pit::Entry>& pitEntry)
{
  const fib::Entry& fibEntry = this->lookupFib(*pitEntry);
  const fib::NextHopList& nexthops = fibEntry.getNextHops();
  if (nexthops.size() == 0) 
  {
    lp::NackHeader nackHeader;
    nackHeader.setReason(lp::NackReason::NO_ROUTE);
    this->sendNack(pitEntry, ingress, nackHeader);
    this->rejectPendingInterest(pitEntry);
    NFD_LOG_DEBUG("Interest, Name:" << interest.getName() << " is rejected (no eligible Next Hop/Forwarding Face)");
    return;
  }

  //Check if this Interest is not new and already forwarded by checking OutRecords
  bool isNewPitEntry = !hasPendingOutRecords(*pitEntry);
  if (!isNewPitEntry) {
    return;
  }

  // use first eligible nexthop to forwarder the Interest, else reject it;
  auto firstEligibleNexthop = std::find_if(nexthops.begin(), nexthops.end(),
        [&] (const fib::NextHop& nexthop) {
          Face& outFace = nexthop.getFace();
          return !wouldViolateScope(ingress.face, interest, outFace); 
        });  

  if (firstEligibleNexthop != nexthops.end()) 
  {
    // We check if the Scope of outface. if it is not LOCAL we delay Interest propagation.Otherwise, we go with immediate Interest propagation.
    auto& outForwardingFace = firstEligibleNexthop->getFace();
    if(outForwardingFace.getScope() != ndn::nfd::FACE_SCOPE_LOCAL)
    {   
      // Call Compute-Delay Function then schedule the Interest sending/Propagation
      
      double delayTimer = this->computeDelayTime(interest, ingress); 
      if(ingress.face.getScope() != ndn::nfd::FACE_SCOPE_LOCAL)
      { 
        if (delayTimer >= 0.0)
        {
          ns3::Time waitingTime = ns3::Seconds(delayTimer);
          ns3::EventId EventId = ns3::Simulator::Schedule (waitingTime, &fw::ISAVndnStrategy::doPropagateAfterDelay, this, ingress.face.getId(), weak_ptr<pit::Entry>(pitEntry));
          scheduledInterestInfo new_Schedule;
          new_Schedule.interestName = interest.getName();
          new_Schedule.interestNonce = interest.getNonce();
          new_Schedule.delayTime = ns3::Simulator::Now() + waitingTime;
          new_Schedule.ScheduleEventId = EventId;
          //----------------------------------------------------------------------------------
          /*
          // GET Geo Info. of the last Interest forwarding node from the received Interest packet
            std::shared_ptr<ndn::lp::GeoTag> lastHopGeoTag = interest.getTag<ndn::lp::GeoTag>();     
            if (lastHopGeoTag == nullptr)
            {    
              NFD_LOG_DEBUG("Face Scope:" << ingress.face.getScope() << " Face ID: " << ingress.face.getId());
              NS_FATAL_ERROR(" The received Interest does not have last-hop GeoPosition Information");     
            }
            //Geo info. of Last Interest forwarder
            ns3::Vector3D packetLastHopGeoPosition;
            std::tuple<double, double, double> lastHopGeoCoordinates = lastHopGeoTag->getPos();
            packetLastHopGeoPosition = ns3::Vector3D(std::get<0>(lastHopGeoCoordinates), std::get<1>(lastHopGeoCoordinates), std::get<2>(lastHopGeoCoordinates));
            NFD_LOG_DEBUG("Last Interest Forawarder is loacted at ( X : " << packetLastHopGeoPosition.x << ", Y : " << packetLastHopGeoPosition.y <<")");
            new_Schedule.interestLastForwarderGeoPosition = packetLastHopGeoPosition;
          */
          //----------------------------------------------------------------------------------
          m_forwarder.m_ScheduledInterestList.push_back(new_Schedule);
          NFD_LOG_DEBUG("Scheduled Interest Name:" << interest.getName() << " Estimated Delay Time (s):" << waitingTime.GetSeconds() << ", Will send at (s): " << new_Schedule.delayTime.GetSeconds());
          return;
        }
        else
        {
          this->rejectPendingInterest(pitEntry);
          NFD_LOG_DEBUG(" Interest (Name:"<<interest.getName() << ") is rejecetd (an issue with propagation delayTime calculation)");
          return;
        }
      }
      else
      {
        this->sendInterest(pitEntry, FaceEndpoint(outForwardingFace, 0), interest);
        NFD_LOG_DEBUG("Interest, Name:" << interest.getName() << " has been sent immediatley (no delay) - From Local Consumer App");
        return;
      }
    }
    else
    {      
      this->sendInterest(pitEntry, FaceEndpoint(outForwardingFace, 0), interest);
      NFD_LOG_DEBUG("Interest, Name:" << interest.getName() << " has been sent immediatley (no delay) - TO local Data Provider App.");
      return;
    }
  }
  else
  {
    lp::NackHeader nackHeader;
    nackHeader.setReason(lp::NackReason::NO_ROUTE);
    this->sendNack(pitEntry, ingress, nackHeader);
    this->rejectPendingInterest(pitEntry);
    NFD_LOG_DEBUG("Interest, Name:" << interest.getName() << " is rejected (no eligible Next Hop/Forwarding Face)");
    return;
  }  
}

double
ISAVndnStrategy::computeDelayTime(const Interest& interest, const FaceEndpoint& ingress)
{
  /* computeDelayTime has the following steps:
      1. Check if the Interest Packet is generated locally or not (isLocal = 0 (not local), = 1 (locacl)),
      2. check current ScheduledInterestList to delete/earse all expired scheduled Interest send events,
      3. check if the received Interest is already scheduled with assistance of NAME and NONCE of received Interest,
      4. Compute the Forwarding Delay Time based on Interest type (Local = 0 delay time, non-Local = delay time based on distance )
      This function/method return double value that reprsent how much extra dealy time from now in delaying Interest propagation.
  */
  
  // Final calculated waiting timer will be sotred in waitingTimer. also this value will be as return for this function.
  double forwardingDelayTime = 0.0;
  
  // We chech if Interest has GeoTag. if yes, it is not local.Otherwise, it is local;
  
  if(ingress.face.getScope() != ndn::nfd::FACE_SCOPE_LOCAL)
  {
    // we call this fucntion to do delete all expired scheulded Interest sending events from the rlated list.
  
    removeExpiredElements();  
  
    // Retrive current Node GeoPosition: 1.Node Id -> 2.Node Object -> 3.Node Mobility Model -> 4.Node GeoPosition 
    int32_t currentNodeId = ns3::Simulator::GetContext();
    ns3::Ptr<ns3::Node> currentNodeObj = ns3::NodeList::GetNode(currentNodeId);
    ns3::Ptr<ns3::MobilityModel> currentNodeMobility= currentNodeObj->GetObject<ns3::MobilityModel>();
    if (currentNodeMobility == nullptr) {
      NS_FATAL_ERROR("Current Node does not have a Mobility Model");
      }

    ns3::Vector3D currenNodetGeoPosition = currentNodeMobility->GetPosition();
    
    // Retrive GeoPosition of LastHop node from Interestpacket -> geoTag Position as in the received Interest packet
    std::shared_ptr<ndn::lp::GeoTag> lastHopGeoTag = interest.getTag<ndn::lp::GeoTag>();
    if (lastHopGeoTag == nullptr) 
    {
      NFD_LOG_DEBUG("The received Interest ("<< interest.getName() <<") is generated locally");
    }
    ns3::Vector3D lastHopGeoPosition;
    std::tuple<double, double, double> lastHopGeoCoordinates = lastHopGeoTag->getPos();
    lastHopGeoPosition = ns3::Vector3D(std::get<0>(lastHopGeoCoordinates), std::get<1>(lastHopGeoCoordinates), 0);
    NFD_LOG_DEBUG("Last Forawarder is loacted at ( X : " << lastHopGeoPosition.x << ", Y : " << lastHopGeoPosition.y <<")");

    // Calculate Distance from last hop and also pushing Timer
    double distFromLastHop = ns3::CalculateDistance(lastHopGeoPosition, currenNodetGeoPosition);
    if (distFromLastHop < 0) {
     NS_FATAL_ERROR("Mobility model is not valid | distance can not be less than 0");
     }
  
    NFD_LOG_DEBUG("Distance From Last Hop:" << distFromLastHop);

    // According to "Rapid Traffic Information Dissemination Using Named Data" paper, we set the following parameters    
    double TR = 250.0; // maximum transmission range. Can be retrieve from ns3::RangePropagationLossModel of wireless channel 
    double T = 0.005; // maximum forwarding delay time
    double FDT; // forwarding delay timer

    // Generate a random number [0.005,0.002] to avoid nodes transmission synchronization
    std::random_device rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd());  
    //std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.0,0.002);    
    
    double r = distribution(gen); 
    NFD_LOG_DEBUG("\n random number [0.0, 0.002]" << r);
    

    FDT = (((TR - std::min(TR, distFromLastHop)) / TR) * T) + r;
    
    NFD_LOG_DEBUG("forwarding delay timer:" << FDT) ;  

    forwardingDelayTime = FDT;
  }  
  
  return forwardingDelayTime; 
}


void
ISAVndnStrategy::afterReceiveLoopedInterest(const FaceEndpoint& ingress, const Interest& interest,
                             pit::Entry& pitEntry)
{  
  // we call this fucntion to do delete all expired scheulded Interest sending events from the rlated list.
  m_forwarder.removeExpiredScheduledInterest();  
  NFD_LOG_DEBUG("\n XX: " << m_forwarder.m_ScheduledInterestList.size()); 

  // we check  ScheduledInterestList size as it might be of size Zero(0) due to deleting of the expired scheduled Interest sendning events
  if (m_forwarder.m_ScheduledInterestList.size() > 0)
  {
    //--------------------------------------------------------------------------------------------------------
    //1. Step 1: REP and LEP estimation
    uint32_t LW = 4, EW = 0; //LW:defualt Lane width , EW: extra width
    // Original Interest Forwarder
    ns3::Vector3D S_GPI;
    uint32_t  S_LN,S_LID,S_NTD,S_RTD,S_RWD;
    double S_REPX,S_REPY, S_LEPX,S_LEPY;
    // Duplicate Interest forwarder
    ns3::Vector3D F_GPI;
    uint32_t  F_LN,F_LID,F_NTD, F_RTD, F_RWD;
    double F_REPX,F_REPY, F_LEPX,F_LEPY;

    // ISA Demarcation
    double isaXMin, isaXMax, isaYMin, isaYMax;

    if(m_forwarder.m_ReceivedInterestInfo.size() > 0)
    {
      for(auto iter = m_forwarder.m_ReceivedInterestInfo.begin(); iter != m_forwarder.m_ReceivedInterestInfo.end();)
      {
        if(iter->interestName == interest.getName() && iter->interestNonce == interest.getNonce())
        { 
          NFD_LOG_DEBUG("## Interest Info. as in PIT: "); 
          NFD_LOG_DEBUG("## Content name: " << iter->interestName); 
          NFD_LOG_DEBUG("## Nonce: " << iter->interestNonce); 
          //---------------------------------------------------------------------------------------------------------------------------------------
          // a. get GPI, LN,LID,NTD, RTD, RWD of orginal Interest forwarder from receivedInterestInfo (PIT)
          S_GPI = iter->LastInterestForwarderGeoPosition;
          S_NTD = iter->NTD;
          S_LN = 2;
          S_LID = 1; // in this versio of code we assume vrhicles is moving in lane 1. we remove TraCi from this version

          // RTD and RWD for duplicate Interest forwarder
          // Road Travel Direction (RTD), 11 = horizontal - 22 = vertical
          if (S_NTD == 1 || S_NTD == 2)
          {
            S_RTD = 11; //horizontal
          }
          else
          {
            S_RTD = 22; //vertical
          }

          

          // Road Width Direction (RTD), 11 = horizontal - 22 = vertical
          //RTD and RWD perpendicularity assumption
          if (S_RTD == 11)
          {
            S_RWD = 22; //vertical
          }
          else
          {
            S_RWD = 11; //horizontal
          }

          NFD_LOG_DEBUG("## NTD in S node: " << S_NTD); 
          NFD_LOG_DEBUG("## RTD in S node: " << S_RTD);
          NFD_LOG_DEBUG("## RWD in S node: " << S_RWD);   
          NFD_LOG_DEBUG("## NTD in localnode packet: " << m_forwarder.m_NTD);     
          NFD_LOG_DEBUG("## RTD in localnode packet: " << m_forwarder.m_RTD);          
          NFD_LOG_DEBUG("## RWD in localnode packet: " << m_forwarder.m_RWD);

          // b. get GPI, LN,LID,NTD,RTD,RWD of Loop Interest forwarder from received duplicate Interest
          
          F_LN = 2;
          F_LID = 1;
          
          //F_GPI
          std::shared_ptr<ndn::lp::GeoTag> F_GPIGeoTag = interest.getTag<ndn::lp::GeoTag>();    
          if(F_GPIGeoTag == nullptr)
          {    
            NS_FATAL_ERROR("The received duplicate Interest does not GeoPosition Information");     
          }
          std::tuple<double, double, double> lastHopGeoCoordinates = F_GPIGeoTag->getPos();
          F_GPI = ns3::Vector3D(std::get<0>(lastHopGeoCoordinates), std::get<1>(lastHopGeoCoordinates), 0);

          //F_NTD
          auto F_NTDTag = interest.getTag<ndn::lp::LastHopIDTag>();
          if(F_NTDTag != nullptr)
          {
            F_NTD = *F_NTDTag;
            NFD_LOG_DEBUG("## NTD in Interest packet: " << F_NTD);               
          }

          // RTD and RWD for duplicate Interest forwarder
          // Road Travel Direction (RTD), 11 = horizontal - 22 = vertical
          // NTD = 1 (RTL), NTD = 2 (LRT), NTD = 3 (DTU), NTD = 4 (UTD)
          if (F_NTD == 1 || F_NTD == 2)
          {
            F_RTD = 11; //horizontal
          }
          else
          {
            F_RTD = 22; //vertical
          }
          
          // Road Width Direction (RTD), 11 = horizontal - 22 = vertical
          //RTD and RWD perpendicularity assumption
          if (F_RTD == 11)
          {
            F_RWD = 22; //vertical
          }
          else
          {
            F_RWD = 11; //horizontal
          } 

          NFD_LOG_DEBUG("## NTD in F node: " << F_NTD);
          NFD_LOG_DEBUG("## RTD in F node: " << F_RTD);
          NFD_LOG_DEBUG("## RWD in F node: " << F_RWD);   
          

          // c. calculate REP and LEP for the  orginal Interest forwarder (S) and duplicate Interest forwarder (F)
          // S node
          
          if (S_RWD == 22) // F_RWD vertical
          {
            //REP
            S_REPX = S_GPI.x;
            if(S_NTD == 1)
            {
              S_REPY = S_GPI.y + ((S_LN - S_LID + 0.5) * LW) + EW;
            }
            else
            {
              S_REPY = S_GPI.y - ((S_LN - S_LID + 0.5) * LW) + EW;        
            }

            //LEP
            S_LEPX= S_GPI.x;
            if(S_NTD == 1)
            {
              S_REPY = S_GPI.y - ((S_LN - (S_LN - S_LID + 0.5)) * LW) + EW;  
            }
            else
            {
              S_REPY = S_GPI.y + ((S_LN - (S_LN - S_LID + 0.5)) * LW) + EW;        
            }
          }
          else // F_RWD horizontal
          {
            //REP
            S_REPY = S_GPI.y;
            if(S_NTD == 4)
            {
              S_REPX = S_GPI.x + ((S_LN - S_LID + 0.5) * LW) + EW;
            }
            else
            {
              S_REPX = S_GPI.x - ((S_LN - S_LID + 0.5) * LW) + EW;        
            }
            
            //LEP
            S_LEPY= S_GPI.y;
            if(S_NTD == 4)
            {
              S_REPX = S_GPI.x - ((S_LN - (S_LN - S_LID + 0.5)) * LW) + EW;  
            }
            else
            {
              S_REPX = S_GPI.x + ((S_LN - (S_LN - S_LID + 0.5)) * LW) + EW;        
            }
          }

          // F node
          
          if (F_RWD == 22) // F_RWD vertical
          {
            //REP
            F_REPX = F_GPI.x;
            if(F_NTD == 1)
            {
              F_REPY = F_GPI.y + ((F_LN - F_LID + 0.5) * LW) + EW;
            }
            else
            {
              F_REPY = F_GPI.y - ((F_LN - F_LID + 0.5) * LW) + EW;        
            }

            //LEP
            F_LEPX= F_GPI.x;
            if(F_NTD == 1)
            {
              F_REPY = F_GPI.y - ((F_LN - (F_LN - F_LID + 0.5)) * LW) + EW;  
            }
            else
            {
              F_REPY = F_GPI.y + ((F_LN - (F_LN - F_LID + 0.5)) * LW) + EW;        
            }
          }
          else // F_RWD horizontal
          {
            //REP
            F_REPY = F_GPI.y;
            if(F_NTD == 4)
            {
              F_REPX = F_GPI.x + ((F_LN - F_LID + 0.5) * LW) + EW;
            }
            else
            {
              F_REPX = F_GPI.x - ((F_LN - F_LID + 0.5) * LW) + EW;        
            }
            
            //LEP
            F_LEPY= F_GPI.y;
            if(F_NTD == 4)
            {
              F_REPX = F_GPI.x - ((F_LN - (F_LN - F_LID + 0.5)) * LW) + EW;  
            }
            else
            {
              F_REPX = F_GPI.x + ((F_LN - (F_LN - F_LID + 0.5)) * LW) + EW;        
            }
          }

          // d. ISA demarcation
          
          isaXMin = std::min(std::min(S_REPX, S_LEPX), std::min(F_REPX, F_LEPX));
          isaXMax = std::max(std::max(S_REPX, S_LEPX), std::max(F_REPX, F_LEPX));

          isaYMin = std::min(std::min(S_REPY, S_LEPY), std::min(F_REPY, F_LEPY));
          isaYMax = std::max(std::max(S_REPY, S_LEPY), std::max(F_REPY, F_LEPY));
          NFD_LOG_DEBUG("## ISA Max (X): " << isaXMax);  
          NFD_LOG_DEBUG("## ISA Min (X): " << isaXMin);
          NFD_LOG_DEBUG("## ISA Max (Y): " << isaYMax);  
          NFD_LOG_DEBUG("## ISA Min (Y): " << isaYMin);

           

          // e. check if local node's geo-position belongs to ISA area
          bool isLocatedInISA = false;
            //1. Get GeoPosition of the local node
            auto currentNode = ns3::NodeList::GetNode(ns3::Simulator::GetContext());
            auto nodeMobility= currentNode->GetObject<ns3::MobilityModel>();
            if (nodeMobility == 0)
            {
              NS_FATAL_ERROR("Forwarder onOutgoingInterest:Mobility model has to be installed on the node");
            }      
            ns3::Vector3D currentPosition = nodeMobility->GetPosition();
            NFD_LOG_DEBUG("## Local Node GPI (X): " << currentPosition.x);
            NFD_LOG_DEBUG("## Local Node GPI (Y): " << currentPosition.y);      
            
            if(currentPosition.x >= isaXMin && currentPosition.x <= isaXMax)
            {
              if(currentPosition.y >= isaYMin && currentPosition.y <= isaYMax)
              {
                isLocatedInISA = true;
                NFD_LOG_DEBUG("## Local node located inside ISA.");
              }
            }
            else
            {
              NFD_LOG_DEBUG("## Local node //// is not //// located inside ISA.");
            }

            for (auto iter1 = m_forwarder.m_ScheduledInterestList.begin(); iter1 != m_forwarder.m_ScheduledInterestList.end(); )
            {
              // Check if the received Interest is already scheduled with assistance of Interest NAME and NONCE,
              if(iter1->interestName == interest.getName() && iter1->interestNonce == interest.getNonce())// && iter->lastForwarderID != forwarderID)
              {
                NFD_LOG_DEBUG("## Interest Info. as in Scheduled Interest Table: "); 
                NFD_LOG_DEBUG("## Content name: " << iter->interestName); 
                NFD_LOG_DEBUG("## Nonce: " << iter->interestNonce); 
                
                if(isLocatedInISA) 
                { 
                  if(iter1->ScheduleEventId.IsRunning())
                  {     
                    iter1->ScheduleEventId.Cancel(); 
                    NFD_LOG_DEBUG("Interest forwarding scheduling is cancelled");              
                  }
                  iter1 = m_forwarder.m_ScheduledInterestList.erase(iter1);  
                  break;  
                }
                else
                {
                  NFD_LOG_DEBUG("Interest forwarding scheduling is NOT cancelled");
                  iter1++;
                }
              }// End if the scheduled Interest name not as the received one      
              else
              {
                iter1++;
              }
            }//End for
          //---------------------------------------------------------------------------------------------------------------------------------------
        } 
        iter++;
      }
    }
    

    //--------------------------------------------------------------------------------------------------------      
  }//End if size
  
}//End method

void
ISAVndnStrategy::beforeExpirePendingInterest(shared_ptr<pit::Entry> pitEntry)
{
  NFD_LOG_DEBUG("## pitEntry=" << pitEntry->getName());
  // Delete received Interest info from the list when the interest is satisfied
  if(pitEntry->getInterest().getName().getPrefix(1) != "/localhost")
  {
    NFD_LOG_DEBUG(" Interest Info. : Name (" << pitEntry->getInterest().getName() << "), Nonce (" << pitEntry->getInterest().getNonce() << ")");
    //m_forwarder.deleteReceivedInterestInfo(pitEntry->getInterest().getName().getPrefix(1), pitEntry->getInterest().getNonce());   
    m_forwarder.satisifedReceivedInterestInfo(pitEntry->getInterest());
  }    
}

void
ISAVndnStrategy::beforeSatisfyInterest(const shared_ptr<pit::Entry>& pitEntry, const FaceEndpoint& ingress, const Data& data)
{   
  if(pitEntry->getInterest().getName().getPrefix(1) !="/localhost" && pitEntry->getInterest().getName().getPrefix(1) !="/")
  {
    NFD_LOG_DEBUG("satisfied satisfied satisfied satisfied satisfied satisfied .");
  }   
  
  // Delete received Interest info from the list when the interest is satisfied
  if(pitEntry->getInterest().getName().getPrefix(1) != "/localhost")
  {
    NFD_LOG_DEBUG(" Interest Info. : Name (" << pitEntry->getInterest().getName() << "), Nonce (" << pitEntry->getInterest().getNonce() << ")");
    //m_forwarder.deleteReceivedInterestInfo(pitEntry->getInterest());   
    m_forwarder.satisifedReceivedInterestInfo(pitEntry->getInterest());
  }
    //---------------------------------------------------------------------------------------------------    
}


void ISAVndnStrategy::removeExpiredElements()
{
  NFD_LOG_DEBUG("Before remove: Size of ScheduledInterestList: " << m_forwarder.m_ScheduledInterestList.size());
  int expiredScheduledEvents =0; // this is to count how many scheduled Interest sendning events have expired.
  
  if (m_forwarder.m_ScheduledInterestList.size() > 0)
  {
    
    for (std::vector<scheduledInterestInfo>::iterator iter = m_forwarder.m_ScheduledInterestList.begin(); iter != m_forwarder.m_ScheduledInterestList.end(); )
    {
      NFD_LOG_DEBUG("Name:" << iter->interestName << " |Nonce:" << iter->interestNonce <<" |EventID: " << iter->ScheduleEventId.GetUid() <<  " |Delay: " << iter->delayTime.GetMinutes());

      if((iter->delayTime - ns3::Simulator::Now()).GetSeconds() < 0.0)
      {
        NFD_LOG_DEBUG("Expired scheduled Interest sending event (Name:"<< iter->interestName << ", Nonce: " << iter->interestNonce <<")");
        iter = m_forwarder.m_ScheduledInterestList.erase(iter);  // Note: earse function after earsing the element, it reutnrs : An iterator pointing to the next element. 
        expiredScheduledEvents++;
      }
      else
      {
        iter++; // Note: Husssein do still remember the isseu of iterator incremenet *|^ :).
      }
      
    }
    NFD_LOG_DEBUG("removeExpiredElements - Total expired scheduled sending events:" << expiredScheduledEvents);
  } 
}


void ISAVndnStrategy::afterReceiveNack(const FaceEndpoint& ingress, const lp::Nack& nack,
                                 const shared_ptr<pit::Entry>& pitEntry)
{
  NFD_LOG_DEBUG("Native VNDN Received a NACK Packet");
  this->processNack(ingress.face, nack, pitEntry);
}

const Name&
ISAVndnStrategy::getStrategyName()
{
  static Name strategyName("/localhost/nfd/strategy/isa-vndn/%FD%01");
  return strategyName;
}


} // namespace fw
} // namespace nfd
