//
// Created by jim on 30/6/2016.
//

#ifndef DESPOT_COOPERATIVE_MINEFIELD_H
#define DESPOT_COOPERATIVE_MINEFIELD_H

#include <despot/core/pomdp.h>
#include <despot/core/mdp.h>
//#include "base/base_cooperative_minefield.h"
#include <despot/util/coord.h>
#include <despot/util/grid.h>

#include <mdp_minefield.h>

namespace despot {

/* =============================================================================
 * CooperativeMinefieldState class
 * =============================================================================*/

class CooperativeMinefieldState : public State {
 public:
  Coord agent_pos_;
  std::vector<Coord> mine_pos_;
  std::vector<double> mine_prob_;
  std::vector<Coord> mine_comm_; // TODO: Maybe turn it into a map for easier lookup. Or have also an uncomm mine vector
  std::vector<Coord> searched_pos_;

  CooperativeMinefieldState();
  CooperativeMinefieldState(int _state_id);
  CooperativeMinefieldState(Coord agent_pos,
                            std::vector<Coord> mine_pos,
                            std::vector<double> mine_prob,
                            std::vector<Coord> mine_comm,
                            std::vector<Coord> searched_pos);

  std::string text() const;
};

/* ==============================================================================
 * CooperativeMinefieldBelief class
 * ==============================================================================*/

class CooperativeMinefield;
class CooperativeMinefieldBelief : public ParticleBelief {
 protected:
  const CooperativeMinefield *cooperativeMinefield_;
 public:
  static int num_particles;
  CooperativeMinefieldBelief(std::vector<State *> particles, const DSPOMDP *model, Belief *prior =
  NULL);
  void Update(int action, OBS_TYPE obs);
};

/* =============================================================================
 * CooperativeMinefield class
 * =============================================================================*/

class CooperativeMinefield : public DSPOMDP {
 public:
  bool Step(State &state, double rand_num, int action, double &reward, OBS_TYPE &obs) const;
  int NumActions() const;
  double ObsProb(OBS_TYPE obs, const State &state, int action) const;

  virtual State *CreateStartState(std::string type = "DEFAULT") const;
  virtual Belief *InitialBelief(const State *start,
                                std::string type = "PARTICLE") const;

  inline double GetMaxReward() const {
    return reward_clear_level_;
  }

  inline ValuedAction GetMinRewardAction() const {
    return ValuedAction(0, reward_communicate_);
  }

  ScenarioUpperBound *CreateScenarioUpperBound(std::string name = "DEFAULT",
                                               std::string particle_bound_name = "DEFAULT") const;
  ParticleLowerBound *CreateParticleLowerBound(std::string name = "DEFAULT") const;
  ScenarioLowerBound *CreateScenarioLowerBound(std::string name = "DEFAULT",
                                               std::string particle_bound_name = "DEFAULT") const;

  POMCPPrior *CreatePOMCPPrior(std::string name = "DEFAULT") const;

  virtual void PrintState(const State &state, std::ostream &out = std::cout) const;
  virtual void PrintObs(const State &state, OBS_TYPE observation,
                        std::ostream &out = std::cout) const;
  void PrintBelief(const Belief &belief, std::ostream &out = std::cout) const;
  virtual void PrintAction(int action, std::ostream &out = std::cout) const;

  State *Allocate(int state_id, double weight) const;
  virtual State *Copy(const State *particle) const;
  virtual void Free(State *particle) const;
  int NumActiveParticles() const;

  bool LocalMove(State &state, const History &history, int obs) const;

//  Belief *Tau(const Belief *belief, int action, OBS_TYPE obs) const;
//  void Observe(const Belief *belief, int action, std::map<OBS_TYPE, double> &obss) const;
//  double StepReward(const Belief *belief, int action) const;

//  int NumStates() const;
//  const State *GetState(int index) const;
//  int GetIndex(const State *state) const;

  inline int GetAction(const State &tagstate) const {
    return 0;
  }

//  int GetRobPosIndex(const State *state) const;
//  Coord GetRobPos(const State *state) const;
//  bool GetRock(const State *state, int rock) const;
//  void SampleRock(State *state, int rock) const;
//  int GetX(const State *state) const;
//  void IncX(State *state) const;
//  void DecX(State *state) const;
//  int GetY(const State *state) const;
//  void IncY(State *state) const;
//  void DecY(State *state) const;

 public:
  enum {
    A_COMMUNICATE = 0,
    A_SEARCH = 1,
    A_SENSE = 2,
    A_WAIT = 3
  };
  std::vector<std::string> actionString{"Communicate", "Search", "Sense", "Wait"};

  enum {
    O_NONE = 0,
    O_AGENT_POS = 1,
    O_MINE_POS = 2,
    O_MINE_PROB = 3
  };

  CooperativeMinefield(std::string map);
  CooperativeMinefield(int size, int mines);

  Grid<int> grid_;
  MDPMinefield *movePolicy;
  std::vector<Coord> mine_pos_;
  std::vector<Coord> searched_pos_;
  int size_, num_mines_;
  Coord start_pos_, end_pos_;
  double reward_clear_level_, reward_move_, reward_die_, reward_search_, reward_communicate_;
  CooperativeMinefieldState mine_state_;
  std::vector<CooperativeMinefieldState *> states_;
  Coord NextPos(const Coord &from, int dir) const;
 private:
  void InitGeneral();
  void InitStates();
  bool GetObservation(double rand_num, const CooperativeMinefieldState &minestate, int mine) const;

  std::vector<std::vector<std::vector<State> > > transition_probabilities_; //TODO: Check the size and the dimensions.
  std::vector<std::vector<double> > alpha_vectors_; // For blind policy
  mutable std::vector<ValuedAction> mdp_policy_;

  int MakeObservations(const CooperativeMinefieldState &state) const;

  void InitializeTransitions();
  Coord IndexToCoord(int pos) const;
  int CoordToIndex(Coord c) const;
  std::vector<ValuedAction> &ComputeOptimalSamplingPolicy() const;
  CooperativeMinefieldState *MajorityRockSampleState(const std::vector<State *> &particles) const;

  mutable MemoryPool<CooperativeMinefieldState> memory_pool_;

};

} // namespace despot

#endif //DESPOT_COOPERATIVE_MINEFIELD_H
