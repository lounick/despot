//
// Created by jim on 30/6/2016.
//

#ifndef DESPOT_COOPERATIVE_MINEFIELD_H
#define DESPOT_COOPERATIVE_MINEFIELD_H

#include <despot/core/pomdp.h>
#include <despot/core/mdp.h>
#include "base/base_cooperative_minefield.h"
#include <despot/util/coord.h>
#include <despot/util/grid.h>

namespace despot{

/* =============================================================================
 * CooperativeMinefield class
 * =============================================================================*/

class CooperativeMinefield: public BaseCooperativeMinefield {
 public:
  CooperativeMinefield(std::string map);
  CooperativeMinefield(int size, int mines);

  bool Step(State& state, double rand_num, int action, double& reward, OBS_TYPE& obs) const;
  int NumActions() const;
  double  ObsProb(OBS_TYPE obs, const State& state, int action) const;
  void PrintObs(const State& state, OBS_TYPE observation, std::ostream& out = std::cout) const;
};

} // namespace despot

#endif //DESPOT_COOPERATIVE_MINEFIELD_H
