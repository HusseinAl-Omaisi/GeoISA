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

#include "native-vndn-strategy.hpp"
#include "algorithm.hpp"
#include "common/logger.hpp"
//Hussein
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
#include "random"


namespace nfd {
namespace fw {

NFD_REGISTER_STRATEGY(NativeVndnStrategy);

NFD_LOG_INIT(NativeVndnStrategy);


NativeVndnStrategy::NativeVndnStrategy(Forwarder& forwarder, const Name& name)
  : Strategy(forwarder)
  , ProcessNackTraits(this)
{
  ParsedInstanceName parsed = parseInstanceName(name);
  if (!parsed.parameters.empty()) {
    NDN_THROW(std::invalid_argument("NativeVndnStrategy does not accept parameters"));
  }
  if (parsed.version && *parsed.version != getStrategyName()[-1].toVersion()) {
    NDN_THROW(std::invalid_argument(
      "NativeVndnStrategy does not support version " + to_string(*parsed.version)));
  }
  this->setInstanceName(makeInstanceName(name, getStrategyName()));

m_retxTime = ns3::Seconds(0.05);

}

NativeVndnStrategy::~NativeVndnStrategy()
{
}


void
NativeVndnStrategy::doPropagateAfterDelay(FaceId inFaceId, weak_ptr<pit::Entry> pitEntryWeak)
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
NativeVndnStrategy::afterReceiveInterest(const FaceEndpoint& ingress, const Interest& interest,
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
          ns3::EventId EventId = ns3::Simulator::Schedule (waitingTime, &fw::NativeVndnStrategy::doPropagateAfterDelay, this, ingress.face.getId(), weak_ptr<pit::Entry>(pitEntry));
          scheduledInterestInfo new_Schedule;
          new_Schedule.interestName = interest.getName();
          new_Schedule.interestNonce = interest.getNonce();
          new_Schedule.delayTime = ns3::Simulator::Now() + waitingTime;
          new_Schedule.ScheduleEventId = EventId;
          m_ScheduledInterestList.push_back(new_Schedule);
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


void
NativeVndnStrategy::afterReceiveNack(const FaceEndpoint& ingress, const lp::Nack& nack,
                                 const shared_ptr<pit::Entry>& pitEntry)
{
  NFD_LOG_DEBUG("Native VNDN Received a NACK Packet");
  this->processNack(ingress.face, nack, pitEntry);
}


double
NativeVndnStrategy::computeDelayTime(const Interest& interest, const FaceEndpoint& ingress)
{
  /* computeDelayTime has the following steps:
      1. Check if the Interest Packet is generated locally or not (isLocal = 0 (not local), = 1 (locacl)),
      2. check current ScheduledInterestList to delete/earse all expired scheduled Interest send events,
      3. check if the received Interest is already scheduled with assistance of NAME and NONCE of received Interest,
      4. Compute the Forwarding Delay Time based on isLocal type (isLocal = 0 : random delay time, isLocal = 1 : delay time based on distance )
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
    std::cout << "\n random number [0.0, 0.002]" << r;
    

    FDT = (((TR - std::min(TR, distFromLastHop)) / TR) * T) + r;
    
    NFD_LOG_DEBUG("forwarding delay timer:" << FDT) ;  

    forwardingDelayTime = FDT;
  }  
  
  return forwardingDelayTime;   
}


void
NativeVndnStrategy::afterReceiveLoopedInterest(const FaceEndpoint& ingress, const Interest& interest,
                             pit::Entry& pitEntry)
{
  // we call this fucntion to do delete all expired scheulded Interest sending events from the rlated list.
  removeExpiredElements();

  // we check  ScheduledInterestList size as it might be of size Zero(0) due to deleting of the expired scheduled Interest sendning events
  if (m_ScheduledInterestList.size() > 0)
  {
    for (std::vector<scheduledInterestInfo>::iterator iter = m_ScheduledInterestList.begin(); iter != m_ScheduledInterestList.end(); )
    {
      
      // Check if the received Interest is already scheduled with assistance of Interest NAME and NONCE,
      if(iter->interestName == interest.getName() && iter->interestNonce == interest.getNonce())
      {
        // stop current related scheduld Interest sending event
        if(iter->ScheduleEventId.IsRunning())
        {
          ns3::Simulator::Cancel(iter->ScheduleEventId);
          NFD_LOG_DEBUG("Interest, Name:"<< interest.getName()<<" is rejecetd (already forwarded by other node");
        }
            
        iter = m_ScheduledInterestList.erase(iter);
        
        // W need here to delete corresponding PIT Entry for the looped Interest:
        // 1. cancel PIT entry expiry timer.
        // 2. delete corresponding PIT Entry from PIT table.
        // Note: here no need to send NACK as
        
        
        //////////////////////////pitEntry.expiryTimer.cancel();
        //////////////////////////auto& PendingTable = m_forwarder.getPit();
        ////////////////////////PendingTable.erase(&pitEntry);
        
        break;
      }
      else{
        iter++;
      }
    }     
  }
 
}

void NativeVndnStrategy::removeExpiredElements()
{
  NFD_LOG_DEBUG("Before remove: Size of ScheduledInterestList: " << m_ScheduledInterestList.size());
  int expiredScheduledEvents =0; // this is to count how many scheduled Interest sendning events have expired.
  
  if (m_ScheduledInterestList.size() > 0)
  {
    
    for (std::vector<scheduledInterestInfo>::iterator iter = m_ScheduledInterestList.begin(); iter != m_ScheduledInterestList.end(); )
    {
      NFD_LOG_DEBUG("Name:" << iter->interestName << " |Nonce:" << iter->interestNonce << " |isLocal: "<< iter->isLocal <<" |EventID: " << iter->ScheduleEventId.GetUid() <<  " |Delay: " << iter->delayTime.GetMinutes());

      if((iter->delayTime - ns3::Simulator::Now()).GetSeconds() < 0.0)
      {
        NFD_LOG_DEBUG("Expired scheduled Interest sending event (Name:"<< iter->interestName << ", Nonce: " << iter->interestNonce <<")");
        iter = m_ScheduledInterestList.erase(iter);  // Note: earse function after earsing the element, it reutnrs : An iterator pointing to the next element. 
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

const Name&
NativeVndnStrategy::getStrategyName()
{
  static Name strategyName("/localhost/nfd/strategy/native-vndn/%FD%01");
  return strategyName;
}


} // namespace fw
} // namespace nfd
