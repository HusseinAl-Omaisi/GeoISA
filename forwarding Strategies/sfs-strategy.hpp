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

#ifndef NFD_DAEMON_FW_NATIVE_VNDN_STRATEGY_HPP
#define NFD_DAEMON_FW_NATIVE_VNDN_STRATEGY_HPP

#include "strategy.hpp"
#include "algorithm.hpp"
#include "process-nack-traits.hpp"
#include "forwarder.hpp"
//Hussein
#include <vector>
#include "ns3/core-module.h"
#include <tuple>


/*
This forwarding strategy is built based on paper "LoICen: A novel location-based and information-centric architecture
for content distribution in vehicular networks" by Azzedine Boukerche, May 2019, Ad Hoc Networks
it is called "LoICen"
*/

namespace nfd {
namespace fw {
/*
  typedef struct scheduledInterestInfo
    {
      //interestNamed and interestNonce are used to check if the same Interest already schedule,
      //and is waiting expiring current scheduled event. 
      ndn::Name interestName;
      uint32_t interestNonce;

      //isLocal is used to check  if the Interest packet is generated locally or received from other nodes.
      int isLocal;

      //eventId is ID of the scheduled event. So, we get access to schculed events.
      //we use this ID to check if the evenet is still running or to delete/cancel teh event.
      ns3::EventId ScheduleEventId;

      //delayTime stores the delay time (Now() + Extra estimated Delay Time) of scheduled Interest sending event. 
      ns3::Time delayTime;

      scheduledInterestInfo() : interestName(), interestNonce(), isLocal(), ScheduleEventId(), delayTime() {};  
    
    } scheduledInterestInfo;
    */

   /*

    typedef struct contentLocationTable
    {
        //Partial Name of the content (or contentPrefix)
        ndn::PartialName contentPrefix;
        
        // In contentProviderLocation stores the geolocation of the content provider that
        // might satisfy the corresponding content prefix name 
        //ns3::Vector3D contentProviderLocation;           
        std::tuple<double,double,double> contentProviderLocation;
        
        //Timestamp registers the moment the information was obtained 
        ns3::Time Timestamp;

        contentLocationTable() : contentPrefix(), contentProviderLocation(), Timestamp() {};
    
    } contentLocationTable;
    */


/** \brief a forwarding strategy that forwards Interest to all FIB nexthops
 *
 *  \note This strategy is not EndpointId-aware.
 */
class ForwardingAngleStrategy : public Strategy
                         , public ProcessNackTraits<ForwardingAngleStrategy>
{
public:
  ForwardingAngleStrategy(Forwarder& forwarder, const Name& name = getStrategyName());

  virtual
  ~ForwardingAngleStrategy() override;

  void
  afterReceiveInterest(const FaceEndpoint& ingress, const Interest& interest,
                       const shared_ptr<pit::Entry>& pitEntry) override;
  
  void
  afterReceiveLoopedInterest(const FaceEndpoint& ingress, const Interest& interest,
                             pit::Entry& pitEntry) override;


  void
  afterReceiveNack(const FaceEndpoint& ingress, const lp::Nack& nack,
                                 const shared_ptr<pit::Entry>& pitEntry) override;
  
 // void
 // afterReceiveData(const shared_ptr<pit::Entry>& pitEntry,
   //               const FaceEndpoint& ingress, const Data& data) override;

 // void
  //beforeSatisfyInterest(const shared_ptr<pit::Entry>& pitEntry,
  //                              const FaceEndpoint& ingress, const Data& data) override;


  static const Name&
  getStrategyName();

 // Hussein : Tow new member fuction to schedule Interest sending
  void
  doPropagateAfterDelay(FaceId inFaceId, weak_ptr<pit::Entry> pitEntryWeak);
  
  double
  computeDelayTime(const Interest& interest, const FaceEndpoint& ingress);

 void cacnelDuplicatInterestScheduling(const Interest& interest);
 void removeExpiredElements();

 //void updateContentLocationTable(const FaceEndpoint& ingress, const Data& data);


private:
//std::vector <scheduledInterestInfo> m_ScheduledInterestList; // A list that holds all scheduled Interest sending events  
//std::vector <contentLocationTable> m_CLT; // A list that holds all discovered potential content providerscheduled Interest sending events  

friend ProcessNackTraits<ForwardingAngleStrategy>;
friend class Forwarder; 
  
};

} // namespace fw
} // namespace nfd

#endif // NFD_DAEMON_FW_MULTICAST_STRATEGY_HPP