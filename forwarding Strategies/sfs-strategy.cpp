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

#include "ForwardingAngle-Strategy.hpp"
#include "algorithm.hpp"
#include "common/logger.hpp"
//Hussein
#include "ns3/node-list.h"
#include "ns3/mobility-model.h"
#include "ns3/node.h"
#include <ndn-cxx/lp/tags.hpp>
#include <vector>
#include <tuple>
#include "ns3/double.h"
#include "ns3/random-variable-stream.h"
#include "ns3/config.h"
#include "interest.hpp"
#include "daemon/table/pit.hpp"
#include "model/ndn-l3-protocol.hpp"
#include <ndn-cxx/lp/pit-token.hpp>
//#include <ndn-cxx/lp/velocity-tag.hpp>
#include "forwarder.hpp"
#include <ndn-cxx/util/scheduler.hpp>
#include <math.h>
#include <cmath>
#include <random>

namespace nfd {
namespace fw {

NFD_REGISTER_STRATEGY(ForwardingAngleStrategy);

NFD_LOG_INIT(ForwardingAngleStrategy);


ForwardingAngleStrategy::ForwardingAngleStrategy(Forwarder& forwarder, const Name& name)
  : Strategy(forwarder)
  , ProcessNackTraits(this)
{
  ParsedInstanceName parsed = parseInstanceName(name);
  if (!parsed.parameters.empty()) {
    NDN_THROW(std::invalid_argument("ForwardingAngleStrategy does not accept parameters"));
  }
  if (parsed.version && *parsed.version != getStrategyName()[-1].toVersion()) {
    NDN_THROW(std::invalid_argument(
      "LoICen does not support version " + to_string(*parsed.version)));
  }
  this->setInstanceName(makeInstanceName(name, getStrategyName()));

}

ForwardingAngleStrategy::~ForwardingAngleStrategy()
{
}


void
ForwardingAngleStrategy::doPropagateAfterDelay(FaceId inFaceId, weak_ptr<pit::Entry> pitEntryWeak)
{
  std::cout << std::endl << "doPropagateAfterDelay";
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


/*
void
ForwardingAngleStrategy::afterReceiveInterest(const FaceEndpoint& ingress, const Interest& interest,
                                     const shared_ptr<pit::Entry>& pitEntry)
{
  std::cout << std::endl << "afterReceiveInterest";
  
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
    if(outForwardingFace.getScope() != ndn::nfd::FACE_SCOPE_LOCAL && interest.getName().getPrefix(1) != "/hello")
    {
      // Call Compute-Delay Function then schedule the Interest sending/Propagation
      double delayTimer = this->computeDelayTime(interest, ingress);
      if (delayTimer>= 0.0)
      {
        ns3::Time waitingTime = ns3::Seconds(delayTimer);
        ns3::EventId EventId = ns3::Simulator::Schedule (waitingTime, &fw::ForwardingAngleStrategy::doPropagateAfterDelay, this, ingress.face.getId(), weak_ptr<pit::Entry>(pitEntry));
        scheduledInterestInfo new_Schedule;
        new_Schedule.interestName = interest.getName();
        new_Schedule.interestNonce = interest.getNonce();
        new_Schedule.delayTime = ns3::Simulator::Now() + waitingTime;
        new_Schedule.ScheduleEventId = EventId;
        m_forwarder.m_ScheduledInterestList.push_back(new_Schedule);
        NFD_LOG_DEBUG("Scheduled Interest Name:" << interest.getName() << " Estimated Delay Time (s):" << waitingTime.GetSeconds() << ", Will send at (s): " << new_Schedule.delayTime.GetSeconds());
        return;
      }
      else
      {
        //this->rejectPendingInterest(pitEntry); //Note: we cancel this code as if waiting time is less than zero ( negative value), means this node failed to meet the distance condition to forwarded  Interest
        NFD_LOG_DEBUG(" Interest (Name:"<<interest.getName() << ") is rejecetd (an issue with propagation delayTime calculation)");
        return;
      }
    }
    else
    {
      this->sendInterest(pitEntry, FaceEndpoint(outForwardingFace, 0), interest);
      NFD_LOG_DEBUG("Interest, Name:" << interest.getName() << " has been sent immediatley (no delay)");
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
    std::cout << std::endl << "afterReceiveInterest";
    return;
  }  
}
*/

void
ForwardingAngleStrategy::afterReceiveInterest(const FaceEndpoint& ingress, const Interest& interest,
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
      if(ingress.face.getScope() != ndn::nfd::FACE_SCOPE_LOCAL && interest.getName().getPrefix(1) != "/hello")
      { 
        if (delayTimer >= 0.0)
        {
          ns3::Time waitingTime = ns3::Seconds(delayTimer);
          ns3::EventId EventId = ns3::Simulator::Schedule (waitingTime, &fw::ForwardingAngleStrategy::doPropagateAfterDelay, this, ingress.face.getId(), weak_ptr<pit::Entry>(pitEntry));
          scheduledInterestInfo new_Schedule;
          new_Schedule.interestName = interest.getName();
          new_Schedule.interestNonce = interest.getNonce();
          new_Schedule.delayTime = ns3::Simulator::Now() + waitingTime;
          new_Schedule.ScheduleEventId = EventId;
          //----------------------------------------------------------------------------------
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





void
ForwardingAngleStrategy::afterReceiveNack(const FaceEndpoint& ingress, const lp::Nack& nack,
                                 const shared_ptr<pit::Entry>& pitEntry)
{
  std::cout << std::endl << std::endl << "ForwardingAngleStrategy Received a NACK Packet" << std::endl << std::endl;
  this->processNack(ingress.face, nack, pitEntry);
}


double
ForwardingAngleStrategy::computeDelayTime(const Interest& interest, const FaceEndpoint& ingress)
{
  /*     
    # This function/method return double value that represent how much extra dealy time from now to forward the received Interest.
  */
  
  // Final calculated waiting timer will be sotred in forwardingDelayTime. also this value will be as return for this function.
  double FDT = 0.0; 
  double r = 0.0; // random vlaue 
  NFD_LOG_DEBUG("Face Scope:" << ingress.face.getScope() << " Face ID: " << ingress.face.getId());
  if(ingress.face.getScope() != ndn::nfd::FACE_SCOPE_LOCAL)
  {
    // we call this fucntion to do delete all expired scheulded Interest sending events from the rlated list.
    removeExpiredElements();
     
    // Retrive current Node GeoPosition: 1.Node Id -> 2.Node Object -> 3.Node Mobility Model -> 4.Node GeoPosition 
    int32_t currentNodeId = ns3::Simulator::GetContext();
    ns3::Ptr<ns3::Node> currentNodeObj = ns3::NodeList::GetNode(currentNodeId);
    ns3::Ptr<ns3::MobilityModel> currentNodeMobility= currentNodeObj->GetObject<ns3::MobilityModel>();
    if (currentNodeMobility == nullptr)
    {
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
    if (distFromLastHop < 0)
    {
      NS_FATAL_ERROR("Mobility model is not valid | distance can not be less than 0");
    }
      
    NFD_LOG_DEBUG("Distance From Last Hop:" << distFromLastHop);

    // -------------------------------------- xxxXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxx ------------------------------------------
      double ForwardingAngleRadians = 0.0, ForwardingAngleDegrees = 0.0, ForwardingAngleDegrees360 = 0.0;

      ///double angleAtorginal, angleAtCurrentNode,angleAtLoopedNode;
      double PI = 3.14159265;
      ForwardingAngleRadians = std::atan2((currenNodetGeoPosition.x - lastHopGeoPosition.x),(currenNodetGeoPosition.y - lastHopGeoPosition.y));
      ForwardingAngleDegrees = (ForwardingAngleRadians * 180.0) / PI;
      NFD_LOG_DEBUG(" Forwarding Angle (Radians): " << ForwardingAngleRadians << " Radians");
      NFD_LOG_DEBUG(" Forwarding Angle (Degrees): " << ForwardingAngleDegrees << " Degrees");
      if(ForwardingAngleDegrees < 0.0)
      {
        ForwardingAngleDegrees360 = 360.0 + ForwardingAngleDegrees;
      }
      else
      {
        ForwardingAngleDegrees360 = ForwardingAngleDegrees;
      }      
      NFD_LOG_DEBUG(" Forwarding Angle (360): " << ForwardingAngleDegrees360 << " Degrees");
      
      //Compute Delay time based on the forwarding angle
      double chosenRandomNumber, TimeBasedDistance ;
      if((ForwardingAngleDegrees360 >= 0 && ForwardingAngleDegrees360 <= 22.5) || 
         (ForwardingAngleDegrees360 >= 337.5 && ForwardingAngleDegrees360 <= 360.0) || 
         (ForwardingAngleDegrees360 >= 67.5 && ForwardingAngleDegrees360 <= 112.5) || 
         (ForwardingAngleDegrees360 >= 157.5 && ForwardingAngleDegrees360 <= 202.5) ||
         (ForwardingAngleDegrees360 >= 247.5 && ForwardingAngleDegrees360 <= 292.5) )
      {
        NFD_LOG_DEBUG(" xxxXXXXxxx I N S I D E   S  W E E T    S P O T xxxXXXXxxx ");
        NFD_LOG_DEBUG(" Forwarding Angle is I N S I D E   S  W E E T    S P O T ");
        NFD_LOG_DEBUG(" xxxXXXXxxx I N S I D E   S  W E E T    S P O T xxxXXXXxxx ");        

        // Generate a random number [0.000,0.002] to avoid nodes transmission synchronization
        std::random_device rd;  // Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd());  
        //std::default_random_engine generator;
        std::uniform_real_distribution<double> distribution(0.0,0.002);    
        
        r = distribution(gen); 
        NFD_LOG_DEBUG("\n random number [0.0, 0.002]" << r);       
      }
      else
      {

        NFD_LOG_DEBUG(" xxxXXXXxxx O U T S I D E   S  W E E T    S P O T xxxXXXXxxx ");
        NFD_LOG_DEBUG(" O U T S I D E   S  W E E T    S P O T ");
        NFD_LOG_DEBUG(" xxxXXXXxxx O U T S I D E   S  W E E T    S P O T xxxXXXXxxx ");

        // Generate a random number [0.000,0.002] to avoid nodes transmission synchronization
        std::random_device rd;  // Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd());  
        //std::default_random_engine generator;
        std::uniform_real_distribution<double> distribution(0.01,0.02);    
        
        r = distribution(gen); 
        NFD_LOG_DEBUG("\n random number [0.01, 0.02]" << r);        
      }
      
    /////////////////////////////////////////////////////////////////////

    // According to "Rapid Traffic Information Dissemination Using Named Data" paper, we set the following parameters    
    double TR = 250.0; // maximum transmission range. Can be retrieve from ns3::RangePropagationLossModel of wireless channel 
    double T = 0.005; // maximum forwarding delay time
    double FDT; // forwarding delay timer
           
    FDT = (((TR - std::min(TR, distFromLastHop)) / TR) * T) + r;
        
    NFD_LOG_DEBUG("forwarding delay timer:" << FDT) ;  
    
  }

  return FDT;   
}


void
ForwardingAngleStrategy::afterReceiveLoopedInterest(const FaceEndpoint& ingress, const Interest& interest,
                             pit::Entry& pitEntry)
{
  // we call this fucntion to do delete all expired scheulded Interest sending events from the rlated list.
  removeExpiredElements();
  // we check  ScheduledInterestList size as it might be of size Zero(0) due to deleting of the expired scheduled Interest sendning events
  if (m_forwarder.m_ScheduledInterestList.size() > 0)
  {
    for (auto iter = m_forwarder.m_ScheduledInterestList.begin(); iter != m_forwarder.m_ScheduledInterestList.end(); )
    {
      // -------------------------------------- xxxXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxx ------------------------------------------
      // Check if the received Interest is already scheduled with assistance of Interest NAME and NONCE,
      if(iter->interestName == interest.getName() && iter->interestNonce == interest.getNonce())
      {
        // stop current related scheduld Interest sending event
        if(iter->ScheduleEventId.IsRunning())
        {
          ns3::Simulator::Cancel(iter->ScheduleEventId);
          NFD_LOG_DEBUG("Interest, Name:"<< interest.getName()<<" is rejecetd (already forwarded by other node");
        }
              
        iter = m_forwarder.m_ScheduledInterestList.erase(iter);               
        break;
      }
      else
      {
        iter++;
      }
    }     
  } 
}

void ForwardingAngleStrategy::removeExpiredElements()
{
  int expiredScheduledInterestEvents =0; // this is to count how many scheduled Data sendning events have expired. 

  // remove expired form Scheduled Data List
  if (m_forwarder.m_ScheduledInterestList.size() > 0)
  {    
    for (auto iter = m_forwarder.m_ScheduledInterestList.begin(); iter != m_forwarder.m_ScheduledInterestList.end(); )
    {
      if((iter->delayTime - ns3::Simulator::Now()).GetSeconds() <= 0.0 )
      {
        NFD_LOG_DEBUG("Expired scheduled Data (Name:"<< iter->interestName << ")");
        if(iter->ScheduleEventId.IsRunning())
        {
          ns3::Simulator::Cancel(iter->ScheduleEventId);
        }
        iter = m_forwarder.m_ScheduledInterestList.erase(iter);  // Note: earse function after earsing the element, it reutnrs : An iterator pointing to the next element. 
        expiredScheduledInterestEvents++;
      }
      else
      {
        iter++;
      }
    }
  }     
  NFD_LOG_DEBUG("removeExpiredElements - Interest: "<< expiredScheduledInterestEvents);  
}

const Name&
ForwardingAngleStrategy::getStrategyName()
{
  static Name strategyName("/localhost/nfd/strategy/forward-angle/%FD%01");
  return strategyName;
}


} // namespace fw
} // namespace nfd
