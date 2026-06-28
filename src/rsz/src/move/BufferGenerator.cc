// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "BufferGenerator.hh"

#include <memory>
#include <vector>

#include "BufferCandidate.hh"
#include "MoveCandidate.hh"
#include "MoveGenerator.hh"
#include "OptimizerTypes.hh"
#include "db_sta/dbNetwork.hh"
#include "odb/geom.h"
#include "rsz/Resizer.hh"
#include "utl/Logger.h"

namespace rsz {

using utl::RSZ;

namespace {

constexpr int kRebufferMaxFanout = 20;

}  // namespace

BufferGenerator::BufferGenerator(const GeneratorContext& context)
    : MoveGenerator(context)
{
}

bool BufferGenerator::isApplicable(const Target& target) const
{
  return MoveGenerator::isApplicable(target) && target.fanout > 1
         && target.fanout < kRebufferMaxFanout
         && resizer_.okToBufferNet(target.driver_pin);
}

std::vector<std::unique_ptr<MoveCandidate>> BufferGenerator::generate(
    const Target& target)
{
  std::vector<std::unique_ptr<MoveCandidate>> candidates;

  // Skip buffer insertion in congested regions so repair favors area-neutral
  // fixes there instead of adding wires/cells.
  const odb::Point loc = resizer_.dbNetwork()->location(target.driver_pin);
  const double congestion = resizer_.congestionAt(loc);
  if (congestion > resizer_.congestionThreshold()) {
    debugPrint(resizer_.logger(),
               RSZ,
               "buffer_move",
               2,
               "REJECT BufferMove {}: congested region ({:.2f} > {:.2f})",
               resizer_.network()->pathName(target.driver_pin),
               congestion,
               resizer_.congestionThreshold());
    return candidates;
  }

  candidates.push_back(
      std::make_unique<BufferCandidate>(resizer_, target, target.driver_pin));
  return candidates;
}

}  // namespace rsz
