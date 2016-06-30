#ifndef SIMPLECOOPERATIVEROCKSAMPLE_H
#define SIMPLECOOPERATIVEROCKSAMPLE_H

#include <despot/core/pomdp.h>
#include <despot/core/mdp.h>

namespace despot {

/* =============================================================================
 * SimpleCooperativeState class
 * =============================================================================*/

class SimpleCooperativeState: public State {
 public:
  int rover_position; // positions are numbered 0, 1, 2 from left to right
  int rock_status; // 1 is good, and 0 is bad
  int comms_performed; // Status of data communication. 1 is true, and 0 is false
  int sample_belief;
  int sense_belief;

  SimpleCooperativeState();
  SimpleCooperativeState(int rover_position, int rock_status, int comms_performed, int sample_belief, int sense_belief);
  ~SimpleCooperativeState();

  std::string text() const;
};

/* =============================================================================
 * SimpleCooperativeRockSample class
 * =============================================================================*/

class SimpleCooperativeRockSample: public DSPOMDP {
 protected:
  mutable MemoryPool<SimpleCooperativeState> memory_pool_;

  std::vector<SimpleCooperativeState *> states_;

  mutable std::vector<ValuedAction> mdp_policy_;

 public:
  enum { // action
    /*A_SAMPLE = 0,
    A_EAST = 1,
    A_WEST = 2,
    A_CHECK = 3,*/
    A_SENSING_CHECK = 0,
    A_SENSING_COMMUNICATE = 1,
    A_SENSING_WAIT = 2
  };

  /*enum {
    O_POS_WEST = 0,
    O_POS_MIDDLE = 1,
    O_POS_EAST = 2,
    O_QUALITY_GOOD = 3,
    O_QUALITY_BAD = 4
  };*/

 public:
  SimpleCooperativeRockSample();

  /* Returns total number of actions.*/
  virtual int NumActions() const;

  /* Deterministic simulative model.*/
  virtual bool Step(State &state, double rand_num, int action, double &reward,
                    OBS_TYPE &obs) const;

  /* Functions related to beliefs and starting states.*/
  virtual double ObsProb(OBS_TYPE obs, const State &state, int action) const;
  State *CreateStartState(std::string type = "DEFAULT") const;
  Belief *InitialBelief(const State *start, std::string type = "DEFAULT") const;

  /* Bound-related functions.*/
  double GetMaxReward() const;
  ScenarioUpperBound *CreateScenarioUpperBound(std::string name = "DEFAULT",
                                               std::string particle_bound_name = "DEFAULT") const;
  ValuedAction GetMinRewardAction() const;
  ScenarioLowerBound *CreateScenarioLowerBound(std::string name = "DEFAULT",
                                               std::string particle_bound_name = "DEFAULT") const;

  /* Memory management.*/
  State *Allocate(int state_id, double weight) const;
  State *Copy(const State *particle) const;
  void Free(State *particle) const;
  int NumActiveParticles() const;

  /* Display.*/
  void PrintState(const State &state, std::ostream &out = std::cout) const;
  void PrintBelief(const Belief &belief, std::ostream &out = std::cout) const;
  void PrintObs(const State &state, OBS_TYPE observation,
                std::ostream &out = std::cout) const;
  void PrintAction(int action, std::ostream &out = std::cout) const;
};

} // namespace despot

#endif
//SIMPLECOOPERATIVEROCKSAMPLE_H