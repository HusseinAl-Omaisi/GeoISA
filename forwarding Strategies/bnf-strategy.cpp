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

#include "flooding-vndn.hpp"
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


namespace nfd {
namespace fw {

NFD_REGISTER_STRATEGY(FloodingVndnStrategy);

NFD_LOG_INIT(FloodingVndnStrategy);


FloodingVndnStrategy::FloodingVndnStrategy(Forwarder& forwarder, const Name& name)
  : Strategy(forwarder)
  , ProcessNackTraits(this)
{
  ParsedInstanceName parsed = parseInstanceName(name);
  if (!parsed.parameters.empty()) {
    NDN_THROW(std::invalid_argument("FloodingVndnStrategy does not accept parameters"));
  }
  if (parsed.version && *parsed.version != getStrategyName()[-1].toVersion()) {
    NDN_THROW(std::invalid_argument(
      "FloodingVndnStrategy does not support version " + to_string(*parsed.version)));
  }
  this->setInstanceName(makeInstanceName(name, getStrategyName()));

  m_retxTime = ns3::Seconds(0.05);

}

FloodingVndnStrategy::~FloodingVndnStrategy()
{
}

void
FloodingVndnStrategy::afterReceiveInterest(const FaceEndpoint& ingress, const Interest& interest,
                                     const shared_ptr<pit::Entry>& pitEntry)
{
  const fib::Entry& fibEntry = this->lookupFib(*pitEntry);
  const Face& inFace = ingress.face;
  fib::NextHopList nhs;

  std::copy_if(fibEntry.getNextHops().begin(), fibEntry.getNextHops().end(), std::back_inserter(nhs),
               [&] (const auto& nh) { return isNextHopEligible(inFace, interest, nh, pitEntry); });

  if (nhs.empty()) {
    NFD_LOG_DEBUG(interest << " from=" << ingress << " no nexthop");

    lp::NackHeader nackHeader;
    nackHeader.setReason(lp::NackReason::NO_ROUTE);
    this->sendNack(pitEntry, ingress, nackHeader);
    this->rejectPendingInterest(pitEntry);
    return;
  }

  //std::shuffle(nhs.begin(), nhs.end(), ndn::random::getRandomNumberEngine());
  this->sendInterest(pitEntry, FaceEndpoint(nhs.front().getFace(), 0), interest);
  NFD_LOG_DEBUG("Interest, Name:" << interest.getName() << " has been sent immediatley (no delay)");
     
}


void
FloodingVndnStrategy::afterReceiveNack(const FaceEndpoint& ingress, const lp::Nack& nack,
                                 const shared_ptr<pit::Entry>& pitEntry)
{
  NFD_LOG_DEBUG("Flooding VNDN Received a NACK Packet");
  this->processNack(ingress.face, nack, pitEntry);
}



const Name&
FloodingVndnStrategy::getStrategyName()
{
  static Name strategyName("/localhost/nfd/strategy/flooding-vndn/%FD%01");
  return strategyName;
}


} // namespace fw
} // namespace nfd
