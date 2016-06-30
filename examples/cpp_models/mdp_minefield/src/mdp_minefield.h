//
// Created by Nick Tsiogkas on 30/6/2016.
//

#ifndef DESPOT_MDP_MINEFIELD_H
#define DESPOT_MDP_MINEFIELD_H

#include <despot/core/pomdp.h>
#include <despot/solver/pomcp.h>
#include <despot/core/mdp.h>
#include <despot/util/coord.h>
#include <despot/util/grid.h>

namespace despot {

class MinefieldState: public State {
  Coord pos_;
 public:
  MinefieldState();
  MinefieldState(int _state_id);

  std::string text() const;

  Coord getPos();
  void setPos(Coord pos);
};

class MDPMinefield: public MDP, public StateIndexer, public StatePolicy {

 protected:
  Grid<int> grid_;
  std::vector<Coord> mine_pos_;
  int size_, num_mines_;
  Coord start_pos_;
  Coord end_pos_;

  MinefieldState *mine_state_;
  mutable MemoryPool<MinefieldState> memory_pool_;
  std::vector<MinefieldState *> states_;

 protected:
  void InitGeneral();
  void InitStates(); // Do I need that?
  std::vector<std::vector<std::vector<State> > > transition_probabilities_;
  mutable std::vector<ValuedAction> mdp_policy_;

 public:
  enum{
    A_NORTH = 0,
    A_EAST = 1,
    A_SOUTH= 2,
    A_WEST = 3
  };
 public:
  MDPMinefield(int size, int num_mines);
  MDPMinefield();

  bool Step(State &state, double rand_num, int action, double &reward, OBS_TYPE &obs);
  int NumActions();
  double ObsProb(OBS_TYPE obs, const State& state, int action);
  const std::vector<State>& TransitionProbability(int s, int a) const;
  int NextState(int s, int a) const;
  double Reward(int s, int a) const;

  State* CreateStartState(std::string type = "DEFAULT") const;
  std::vector<State*> InitialParticleSet() const;
  std::vector<State*> NoisyInitialParticleSet() const;
  Belief* InitialBelief(const State* start, std::string type = "PARTICLE") const;

  void PrintState(const State& state, std::ostream& out = std::cout) const;
  void PrintBelief(const Belief& belief, std::ostream& out = std::cout) const;
  void PrintObs(const State& state, OBS_TYPE observation, std::ostream& out = std::cout) const;
  void PrintAction(int action, std::ostream& out = std::cout) const;

  Belief* Tau(const Belief* belief, int action, OBS_TYPE obs) const;
  void Observe(const Belief* belief, int action, std::map<OBS_TYPE, double>& obss) const;
  double StepReward(const Belief* belief, int action) const;

  int NumStates() const;
  const State* GetState(int index) const;
  int GetIndex(const State* state) const;

  int GetRobPosIndex(const State* state) const;
  Coord GetRobPos(const State* state) const;
  bool GetRock(const State* state, int rock) const;
  void SampleRock(State* state, int rock) const;
  int GetX(const State* state) const;
  void IncX(State* state) const;
  void DecX(State* state) const;
  int GetY(const State* state) const;
  void IncY(State* state) const;
  void DecY(State* state) const;

 protected:
  void InitializeTransitions();
  Coord IndexToCoord(int pos) const;
  int CoordToIndex(Coord c) const;
};

}

#endif //DESPOT_MDP_MINEFIELD_H
