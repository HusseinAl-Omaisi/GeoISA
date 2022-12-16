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

#ifndef NFD_DAEMON_FW_ISA_VNDN_STRATEGY_HPP
#define NFD_DAEMON_FW_ISA_VNDN_STRATEGY_HPP

#include "strategy.hpp"
#include "algorithm.hpp"
#include "process-nack-traits.hpp"
#include "forwarder.hpp"


/*
This forwarding strategy is built based on paper "VANET via Named Data Networking" by Giulio Grassi, 2014
it is called Vanilla VNDN
*/

namespace nfd {
namespace fw {

/*
  typedef struct scheduledInterestInfo
    {
        //std::string interestName;
       //////// uint32_t nId;
        ///ndn::Interest intPacket;
       
       // interestNamed and interestNonce are used to check if the same Interest already schedule,
          // and is waiting expiring current scheduled event. 
        ndn::Name interestName;
        uint32_t interestNonce;

        // isLocal is used to check  if the Interest packet is generated locally or received from other nodes.
        int isLocal;

      // eventId is ID of the scheduled event. So, we get access to schculed events.
         //  we use this ID to check if the evenet is still running or to delete/cancel teh event.
        ns3::EventId ScheduleEventId;
        //uint64_t eventId;
        //delayTime stores the delay time (Now() + Extra estimated Delay Time) of scheduled Interest sending event. 
        ns3::Time delayTime;
        scheduledInterestInfo() : interestName(), interestNonce(), isLocal(), ScheduleEventId(), delayTime() {};
        
    } scheduledInterestInfo;
    */

/** \brief a forwarding strategy that forwards Interest to all FIB nexthops
 *
 *  \note This strategy is not EndpointId-aware.
 */
class ISAVndnStrategy : public Strategy
                         , public ProcessNackTraits<ISAVndnStrategy>
{
public:
  ISAVndnStrategy(Forwarder& forwarder, const Name& name = getStrategyName());

  virtual
  ~ISAVndnStrategy() override;

  void
  afterReceiveInterest(const FaceEndpoint& ingress, const Interest& interest,
                       const shared_ptr<pit::Entry>& pitEntry) override;
  
  void
  afterReceiveLoopedInterest(const FaceEndpoint& ingress, const Interest& interest,
                             pit::Entry& pitEntry) override;


  //void
 // afterReceiveNack(const FaceEndpoint& ingress, const lp::Nack& nack,
  //                               const shared_ptr<pit::Entry>& pitEntry) override;
  
  static const Name&
  getStrategyName();

  

 // Hussein 
  void afterReceiveNack(const FaceEndpoint& ingress, const lp::Nack& nack,
                                 const shared_ptr<pit::Entry>& pitEntry) override;
  void beforeExpirePendingInterest(shared_ptr<pit::Entry> pitEntry);
  void beforeSatisfyInterest(const shared_ptr<pit::Entry>& pitEntry, const FaceEndpoint& ingress, const Data& data);
  
  void doPropagateAfterDelay(FaceId inFaceId, weak_ptr<pit::Entry> pitEntryWeak);
  
  double computeDelayTime(const Interest& interest, const FaceEndpoint& ingress); 
  void removeExpiredElements();
 

  private:
  //std::vector <scheduledInterestInfo> m_ScheduledInterestList; // A list that holds all scheduled Interest sending events  
  friend ProcessNackTraits<ISAVndnStrategy>;
  friend class Forwarder;
  
  ns3::Time m_retxTime;    
};

} // namespace fw
} // namespace nfd

#endif // NFD_DAEMON_FW_MULTICAST_STRATEGY_HPP
