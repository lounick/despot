#include "simple_cooperative_rock_sample.h"

using namespace std;

namespace despot {

/* =============================================================================
 * SimpleCooperativeState class
 * =============================================================================*/

SimpleCooperativeState::SimpleCooperativeState() {
}

SimpleCooperativeState::SimpleCooperativeState(int _rover_position,
                                               int _rock_status,
                                               int _comms_performed,
                                               int _sample_belief, int _sense_belief) {
  rover_position = _rover_position;
  rock_status = _rock_status;
  comms_performed = _comms_performed;
  sample_belief = _sample_belief;
  sense_belief = _sense_belief;
}

SimpleCooperativeState::~SimpleCooperativeState() {
}

string SimpleCooperativeState::text() const {
  return "rover position = " + to_string(rover_position) + " rock_status = " +
      to_string(rock_status) + " comms_performed = " + to_string(comms_performed);
}

/* =============================================================================
 * SimpleCooperativeRockSample class
 * =============================================================================*/

SimpleCooperativeRockSample::SimpleCooperativeRockSample() {
}

/* ======
 * Action
 * ======*/

int SimpleCooperativeRockSample::NumActions() const {
  return 3;
}

/* ==============================
 * Deterministic simulative model
 * ==============================*/

bool SimpleCooperativeRockSample::Step(State &state, double rand_num, int action,
                                       double &reward, OBS_TYPE &obs) const {
  SimpleCooperativeState &simple_state = static_cast<SimpleCooperativeState &>(state);
  int &rover_position = simple_state.rover_position;
  int &rock_status = simple_state.rock_status;
  int &comms_performed = simple_state.comms_performed;
  int &sample_belief = simple_state.sample_belief;
  int &sense_belief = simple_state.sense_belief;
  // TODO: Actions should be in pairs (like a tuple) as in each simulation step both agents perform an action.
  // TODO: Or maybe this will model just the sensing agent. And the other agent will act given a default policy given its current position.
  // FIXME: For now we don't take into account the cost of sensing or transmitting

  obs = 1;
  if (comms_performed == 0) {
    // If we haven't performed any communications we can sense, communicate or wait
    if (action == A_SENSING_WAIT) {
      // Just wait nothing changes and the sampling agent follows default policy.
      reward = 0;
      comms_performed = 0;
    } else if (action == A_SENSING_CHECK) {
      reward = -1;
      comms_performed = 0;
      obs = rock_status;
      sense_belief = obs;
    } else if (action == A_SENSING_COMMUNICATE) {
      reward = -2;
      comms_performed = 1;
      sample_belief = sense_belief;
    }
  } else {
    // What happens if we have already communicated
    if (action == A_SENSING_WAIT) {
      // Just wait nothing changes and the sampling agent follows default policy.
      reward = 0;
    } else if (action == A_SENSING_CHECK) {
      obs = rock_status;
      sense_belief = obs;
      reward = -1;
    } else if (action == A_SENSING_COMMUNICATE) {
      reward = -2;
      sample_belief = sense_belief;
    }
  }

  // Now it is the sampling agent turn to act.
  if (rover_position == 0) {
    // It is on the west so it should sample or move east based on the belief of the sample by default thinks it is good
    if (sample_belief == 1) {
      // Thinks it is good so should sample
      reward += rock_status ? 10 : -10; // Get reward based on the actual rock status
      rock_status = 0; // Make rock bad
      sample_belief = 0; // And the sample agent knows it is bad
    }
    else {
      // Thinks the rock is bad and moves to east.
      rover_position = 1;
    }
  } else if (rover_position == 1) {
    if (sample_belief == 1) {
      // Thinks it is good so should go sample
      rover_position = 0;
    }
    else {
      // Thinks the rock is bad and moves to east.
      rover_position = 2;
    }
  } else {
    // Pass
  }

  return rover_position == 2;
}

/* ================================================
 * Functions related to beliefs and starting states
 * ================================================*/

double SimpleCooperativeRockSample::ObsProb(OBS_TYPE obs, const State &state,
                                            int action) const {
  /*if (action == A_CHECK) {
    const SimpleCooperativeState &simple_state = static_cast<const SimpleCooperativeState &>(state);
    int rover_position = simple_state.rover_position;
    int rock_status = simple_state.rock_status;

    if (rover_position == 0) {
      return obs == rock_status;
    } else if (rover_position == 1) {
      return (obs == rock_status) ? 0.8 : 0.2;
    }
  }*/

  if (action == A_SENSING_CHECK) {
    const SimpleCooperativeState &simple_state = static_cast<const SimpleCooperativeState &>(state);
    int rover_position = simple_state.rover_position;
    int rock_status = simple_state.rock_status;
    return obs == rock_status;
  }

  return obs == 1;
}

State *SimpleCooperativeRockSample::CreateStartState(string type) const {
  return new SimpleCooperativeState(1, Random::RANDOM.NextInt(2), 0, 1, 1);
}

Belief *SimpleCooperativeRockSample::InitialBelief(const State *start, string type) const {
  if (type == "DEFAULT" || type == "PARTICLE") {
    vector<State *> particles;

    SimpleCooperativeState *good_rock = static_cast<SimpleCooperativeState *>(Allocate(-1, 0.5));
    good_rock->rover_position = 1;
    good_rock->rock_status = 1;
    good_rock->comms_performed = 0;
    good_rock->sample_belief = 1;
    good_rock->sense_belief = 1;
    particles.push_back(good_rock);

    SimpleCooperativeState *bad_rock = static_cast<SimpleCooperativeState *>(Allocate(-1, 0.5));
    bad_rock->rover_position = 1;
    bad_rock->rock_status = 0;
    bad_rock->comms_performed = 0;
    bad_rock->sample_belief = 1;
    bad_rock->sense_belief = 1;
    particles.push_back(bad_rock);

    return new ParticleBelief(particles, this);
  } else {
    cerr << "[SimpleRockSample::InitialBelief] Unsupported belief type: " << type << endl;
    exit(1);
  }
}

/* ========================
 * Bound-related functions.
 * ========================*/

double SimpleCooperativeRockSample::GetMaxReward() const {
  return 10;
}

class SimpleRockSampleParticleUpperBound: public ParticleUpperBound {
 protected:
  // upper_bounds_[pos][status]:
  //   max possible reward when rover_position = pos, and rock_status = status.
  // TODO: Here should add communications cost etc. Also make it compliant with the action and rewards.
  // TODO: This should also be affected by the sense belief? The upper bound actually is a much more complex structure.
  // TODO: This should reflect what will happen in the different scenarios whether you communicate or not and the state and position of the robot.
  vector<vector<double> > upper_bounds_;

 public:
  SimpleRockSampleParticleUpperBound(const DSPOMDP *model) {
    upper_bounds_.resize(3);
    upper_bounds_[0].push_back(Globals::Discount(1) * 10);
    upper_bounds_[0].push_back(10 + Globals::Discount(2) * 10);
    upper_bounds_[1].push_back(10);
    upper_bounds_[1].push_back(Globals::Discount(1) * 10 + Globals::Discount(3) * 10);
    if (upper_bounds_[1][1] < 10)
      upper_bounds_[1][1] = 10;
    upper_bounds_[2].push_back(0);
    upper_bounds_[2].push_back(0);
  }

  double Value(const State &s) const {
    const SimpleCooperativeState &state = static_cast<const SimpleCooperativeState &>(s);
    return upper_bounds_[state.rover_position][state.rock_status];
  }
};

ScenarioUpperBound *SimpleCooperativeRockSample::CreateScenarioUpperBound(string name,
                                                                          string particle_bound_name) const {
  ScenarioUpperBound *bound = NULL;
  if (name == "TRIVIAL" || name == "DEFAULT") {
    bound = new TrivialParticleUpperBound(this);
  } else if (name == "MAX") {
    bound = new SimpleRockSampleParticleUpperBound(this);
  } else {
    cerr << "Unsupported base upper bound: " << name << endl;
    exit(0);
  }
  return bound;
}

ValuedAction SimpleCooperativeRockSample::GetMinRewardAction() const {
  return ValuedAction(A_SENSING_COMMUNICATE, -2);
}

class SimpleRockSampleEastPolicy: public Policy {
 public:
  SimpleRockSampleEastPolicy(const DSPOMDP *model, ParticleLowerBound *bound) :
      Policy(model, bound) {
  }

  int Action(const vector<State *> &particles, RandomStreams &streams,
             History &history) const {
    return 2; // move east
  }
};

ScenarioLowerBound *SimpleCooperativeRockSample::CreateScenarioLowerBound(string name,
                                                                          string particle_bound_name) const {
  ScenarioLowerBound *bound = NULL;
  if (name == "TRIVIAL" || name == "DEFAULT") {
    bound = new TrivialParticleLowerBound(this);
  } else if (name == "EAST") {
    bound = new SimpleRockSampleEastPolicy(this,
                                           CreateParticleLowerBound(particle_bound_name));
  } else {
    cerr << "Unsupported lower bound algorithm: " << name << endl;
    exit(0);
  }
  return bound;
}

/* =================
 * Memory management
 * =================*/

State *SimpleCooperativeRockSample::Allocate(int state_id, double weight) const {
  SimpleCooperativeState *state = memory_pool_.Allocate();
  state->state_id = state_id;
  state->weight = weight;
  return state;
}

State *SimpleCooperativeRockSample::Copy(const State *particle) const {
  SimpleCooperativeState *state = memory_pool_.Allocate();
  *state = *static_cast<const SimpleCooperativeState *>(particle);
  state->SetAllocated();
  return state;
}

void SimpleCooperativeRockSample::Free(State *particle) const {
  memory_pool_.Free(static_cast<SimpleCooperativeState *>(particle));
}

int SimpleCooperativeRockSample::NumActiveParticles() const {
  return memory_pool_.num_allocated();
}

/* =======
 * Display
 * =======*/

void SimpleCooperativeRockSample::PrintState(const State &state, ostream &out) const {
  const SimpleCooperativeState &simple_state = static_cast<const SimpleCooperativeState &>(state);

  out << "Rover = " << simple_state.rover_position << "; Rock = "
      << (simple_state.rock_status ? "GOOD" : "BAD") << "; SenseBel = " << (simple_state.sense_belief ? "GOOD" : "BAD")
      << "; SampleBel = " << (simple_state.sample_belief ? "GOOD" : "BAD") << endl;
}

void SimpleCooperativeRockSample::PrintObs(const State &state, OBS_TYPE observation,
                                           ostream &out) const {
  out << (observation ? "GOOD" : "BAD") << endl;
}

void SimpleCooperativeRockSample::PrintBelief(const Belief &belief, ostream &out) const {
  const vector<State *> &particles =
      static_cast<const ParticleBelief &>(belief).particles();

  double rock_status = 0;
  vector<double> pos_probs(3);
  for (int i = 0; i < particles.size(); i++) {
    State *particle = particles[i];
    const SimpleCooperativeState *state = static_cast<const SimpleCooperativeState *>(particle);
    rock_status += state->rock_status * particle->weight;
    pos_probs[state->rover_position] += particle->weight;
  }

  out << "Rock belief: " << rock_status << endl;

  out << "Position belief:" << " LEFT" << ":" << pos_probs[0] << " MIDDLE"
      << ":" << pos_probs[1] << " RIGHT" << ":" << pos_probs[2] << endl;
}

void SimpleCooperativeRockSample::PrintAction(int action, ostream &out) const {
  /*if (action == A_SAMPLE)
    out << "Sample" << endl;
  if (action == A_CHECK)
    out << "Check" << endl;
  if (action == A_EAST)
    out << "EAST " << endl;
  if (action == A_WEST)
    out << "West" << endl;*/
  if (action == A_SENSING_CHECK)
    out << "Check" << endl;
  if (action == A_SENSING_COMMUNICATE)
    out << "Transmit" << endl;
  if (action == A_SENSING_WAIT)
    out << "Wait" << endl;
}

} // namespace despot
