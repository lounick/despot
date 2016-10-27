//
// Created by jim on 30/6/2016.
//

#include "cooperative_minefield.h"

namespace despot {

/* ==============================================================================
 * CMSmartParticleUpperBound class
 * ==============================================================================*/

class CMSmartParticleUpperBound : public ParticleUpperBound {
 protected:
  const CooperativeMinefield *cm_;
 public:
  CMSmartParticleUpperBound(const CooperativeMinefield *model) : cm_(model) {}

  double Value(const State &state) const {
    const CooperativeMinefieldState &cmstate = static_cast<const CooperativeMinefieldState &>(state);
    int dist = Coord::ManhattanDistance(cmstate.agent_pos_, cm_->end_pos_);
    return cm_->reward_die_ * (1 - Globals::Discount(dist)) / (1 - Globals::Discount())
        + cm_->reward_clear_level_ * Globals::Discount(dist);
  }
};

/* ==============================================================================
 * CMSmartParticleUpperBound class
 * ==============================================================================*/

class CMApproxScenarioUpperBound : public ScenarioUpperBound {
 protected:
  const CooperativeMinefield *cm_;
 public:
  CMApproxScenarioUpperBound(const CooperativeMinefield *model) : cm_(model) {}

  /*
   * TODO: The upper bound only considers actions and rewards based on one member. It totally ignores the
   * communicate, sense or search actions. We must incorporate somehow as it is formulating the total reward.
   * Also a state that the agent is next to a mine that is not communicated is much less valuable as it may die. On
   * the other hand if a mine is communicated it will never be hit.
   * TODO: Create an approximate upper bound that is more detailed.
   *
   * So it should take into account the position of the agent. And the mines in the vicinity that are not yet
   * communicated. The penalty of the mine should be the probability of being a penalty_die*mine_prob*discount^distance.
   * Also should add the reward_clear*discount^distance (Obviously states closer to the end are more valuable).
   */

  double Value(const std::vector<State *> &particles, RandomStreams &streams, History &history) const {
    double total_value = 0;
    for (int i = 0; i < particles.size(); i++) {
      CooperativeMinefieldState &state = static_cast<CooperativeMinefieldState &>(*(particles[i]));
      double value = 0;

      // For each not communicated mine.
      for (int i = 0; i < state.mine_pos_.size(); i++) {
        std::vector<Coord>::iterator it;
        it = std::find(std::begin(state.mine_comm_), std::end(state.mine_comm_), state.mine_pos_[i]);
        if (it == std::end(state.mine_comm_)) {
          // The mine is not communicated so we need to take it into account.
          value += cm_->reward_die_ * state.mine_prob_[i]
              * Globals::Discount(Coord::ManhattanDistance(state.agent_pos_, state.mine_pos_[i]));
        }
      }

      value += cm_->reward_clear_level_
          * Globals::Discount(Coord::ManhattanDistance(state.agent_pos_, cm_->end_pos_));

      total_value += state.weight * value;

    }
    return total_value;
  }
};

/* ==============================================================================
 * CMLegalParticleLowerBound class
 * ==============================================================================*/

class CMLegalParticleLowerBound : public ParticleLowerBound {
 protected:
  const CooperativeMinefield *cm_;
 public:
  CMLegalParticleLowerBound(const DSPOMDP *model) :
      ParticleLowerBound(model),
      cm_(static_cast<const CooperativeMinefield *>(model)) {
  }

  ValuedAction Value(const std::vector<State *> &particles) const {
    const CooperativeMinefieldState &state =
        static_cast<const CooperativeMinefieldState &>(*particles[0]);
    // FIXME: Should return the reward if the moving agent follows its policy given that the communicating agent always
    // doesn't communicate. Also should try with also communicating.

  }
};

/* =============================================================================
 * CooperativeMinefieldState class
 * =============================================================================*/

CooperativeMinefieldState::CooperativeMinefieldState() {

}
CooperativeMinefieldState::CooperativeMinefieldState(int _state_id) {
  state_id = _state_id;
}
CooperativeMinefieldState::CooperativeMinefieldState(Coord agent_pos,
                                                     std::vector<Coord> mine_pos,
                                                     std::vector<double> mine_prob,
                                                     std::vector<Coord> mine_comm,
                                                     std::vector<Coord> searched_pos) {
  agent_pos_ = agent_pos;
  mine_pos_ = mine_pos;
  mine_prob_ = mine_prob;
  mine_comm_ = mine_comm;
  searched_pos_ = searched_pos;
}
std::string CooperativeMinefieldState::text() const {
  return "id = " + to_string(state_id);
}

/* ==============================================================================
 * CooperativeMinefieldBelief class
 * ==============================================================================*/
int CooperativeMinefieldBelief::num_particles = 50000;

CooperativeMinefieldBelief::CooperativeMinefieldBelief(std::vector<State *> particles,
                                                       const DSPOMDP *model,
                                                       Belief *prior) : ParticleBelief(particles, model, prior) {
  cooperativeMinefield_ = (CooperativeMinefield *) model;
}

void CooperativeMinefieldBelief::Update(int action, OBS_TYPE obs) {
  /*
   * FIXME: This handles only specific actions (i.e. only four defined actions here). Not complex ones. Same for
   * LocalMove. I may have to refactor.
   */
  history_.Add(action, obs);
  std::vector<State *> updated;
  double reward;
  OBS_TYPE o;
  int cur = 0, N = particles_.size(), trials = 0;
  while (updated.size() < num_particles && trials < 10 * num_particles) { // trials < 10 * num_particles is a magic
    // number to make sure that it will stop at some point.
    State *particle = cooperativeMinefield_->Copy(particles_[cur]);
    bool terminal = cooperativeMinefield_->Step(*particle, Random::RANDOM.NextDouble(),
                                                action, reward, o);

    if ((!terminal && o == obs) // Why ignores terminal states? Apparently wants to keep alive only particles that
        // are leading to some future.
        || cooperativeMinefield_->LocalMove(*particle, history_, obs)) { // LocalMove is actually not needed.
      updated.push_back(particle);
    } else {
      cooperativeMinefield_->Free(particle);
    }

    cur = (cur + 1) % N;

    trials++;
  }

  for (int i = 0; i < particles_.size(); i++)
    cooperativeMinefield_->Free(particles_[i]);

  particles_ = updated;

  for (int i = 0; i < particles_.size(); i++)
    particles_[i]->weight = 1.0 / particles_.size();
}

CooperativeMinefield::CooperativeMinefield(std::string map) {
  std::ifstream fin(map.c_str(), std::ifstream::in);
  std::string tmp;
  fin >> tmp >> tmp >> size_ >> size_;

  char tok;
  for (int r = size_ - 1; r >= 0; r--) {
    for (int c = 0; c < size_; c++) {
      fin >> tok;
      if (tok == 'R')
        start_pos_ = Coord(c, r);
      else if (tok == '-') {
        mine_pos_.push_back(Coord(c, r));
      }
    }
  }
  num_mines_ = mine_pos_.size();

  grid_.Resize(size_, size_);
  grid_.SetAllValues(-1);
  for (int i = 0; i < num_mines_; ++i) {
    grid_(mine_pos_[i]) = i;
  }
}

CooperativeMinefield::CooperativeMinefield(int size, int mines) : grid_(size, size), size_(size), num_mines_(mines) {
  InitGeneral();
  // InitStates();
}

bool CooperativeMinefield::Step(State &state, double rand_num, int action_idx, double &reward, OBS_TYPE &obs) const {
  /*
   * FIXME: Add all the functionality
   * This is the step function used to show the game progression over time. The two team members operate one after the
   * other taking the optimal action according to their planning algorithms. We assume that the round lasts enough for
   * communication to be performed and the action to be executed. The following list of steps describes the game sequence.
   *
   * 1. Sensing agent queries for the optimal action. Or maybe he gets it from the function definition. DONE
   * 2. Gets the action and executes.
   * 3. Gets some observation and updates it's belief and state.
   * 4. If the action was communication and was successful the moving agent updates its policy.
   * 5. The moving agent chooses the best action based on its state.
   * 6. It performs the action.
   * 7. The team gets reward based on the actions.
   */
  CooperativeMinefieldState s = static_cast<CooperativeMinefieldState &>(state);
  int action = actions[action_idx].first;
  Coord cell = actions[action_idx].second;

  obs = GetObservation(rand_num, s, action_idx);

  switch (obs){
    case O_AGENT_POS:
      // Successfully communicated a mine. Update lists and agent position.
      break;
    case O_MINE_POS:
      // Found a mine. Add it to mines.
      break;
    case O_MINE_PROB:
      // Imporved a mine classification outcome.
      break;
    case O_NONE:
      // Got Nothing.
      break;
  }

  p = movePolicy->policy();
  int move_action = p[CoordToIndex(s.agent_pos_)].action;

  return false;
}

int CooperativeMinefield::NumActions() const {
  // Clear action vector
  actions.clear();
  //For each non communicated mine insert a communicate action
  for (int i = 0; i < mine_uncomm_.size(); ++i)
    actions.push_back(std::make_pair(A_COMMUNICATE, mine_uncomm_[i]));
  // TODO: Add actions for sensing and searching
  return actions.size();
}

double CooperativeMinefield::ObsProb(OBS_TYPE obs, const State &state, int action) const {
  return 0; //TODO: fix it.
}

State *CooperativeMinefield::CreateStartState(std::string type) const {
  // Just place the mines where they should be. The robot at start position etc.
  CooperativeMinefieldState *startState = memory_pool_.Allocate();
  startState = new CooperativeMinefieldState(start_pos_,
                                             mine_pos_,
                                             std::vector<double>(num_mines_, 1),
                                             std::vector<Coord>(),
                                             searched_pos_);
  return startState;
}
Belief *CooperativeMinefield::InitialBelief(const State *start, std::string type) const {
  /*
   * What is our initial belief for the world? We are sure about the robot but not sure about mine probabilities.
   * In future expansion you won't know where the mines are and you have to search as well.
   * This should be done in the CreateStartState function. It should create some variating belief distribution about
   * the mine probabilities. If we don't know the mine we will have a uniform initial belief that we don't know
   * anything else but the robot start position.
   */
  int N = CooperativeMinefieldBelief::num_particles;
  std::vector<State *> particles(N);
  for (int i = 0; i < N; i++) {
    particles[i] = CreateStartState();
    particles[i]->weight = 1.0 / N;
  }

  return new CooperativeMinefieldBelief(particles, this);
}
ScenarioUpperBound *
CooperativeMinefield::CreateScenarioUpperBound(std::string name, std::string particle_bound_name) const {
  if (name == "TRIVIAL") {
    return new TrivialParticleUpperBound(this);
  } else if (name == "APPROX") {
    return new CMApproxScenarioUpperBound(this);
  } else if (name == "SMART" || name == "DEFAULT") {
    return new CMSmartParticleUpperBound(this);
  } else {
    std::cerr << "Unsupported base upper bound: " << name << std::endl;
    exit(1);
    return NULL;
  }
}

ParticleLowerBound *CooperativeMinefield::CreateParticleLowerBound(std::string name) const {
  if (name == "TRIVIAL") {
    return new TrivialParticleLowerBound(this);
  } else if (name == "LEGAL" || name == "DEFAULT") {
    return new CMLegalParticleLowerBound(this);
  } else {
    std::cerr << "Unsupported base lower bound: " << name << std::endl;
    exit(1);
    return NULL;
  }
}

ScenarioLowerBound *
CooperativeMinefield::CreateScenarioLowerBound(std::string name, std::string particle_bound_name) const {
  if (name == "TRIVIAL") {
    return new TrivialParticleLowerBound(this);
  } else if (name == "LEGAL") {
    return new CMLegalParticleLowerBound(this);
  } else if (name == "SMART" || name == "DEFAULT") {
    return new PocmanSmartPolicy(this, //FIXME
                                 CreateParticleLowerBound(particle_bound_name));
  } else if (name == "RANDOM") {
    return new RandomPolicy(this,
                            CreateParticleLowerBound(particle_bound_name));
  } else {
    std::cerr << "Unsupported lower bound algorithm: " << name << std::endl;
    exit(0);
    return NULL;
  }
}

POMCPPrior *CooperativeMinefield::CreatePOMCPPrior(std::string name) const {
  // TODO: Decide if we need it. I don't think we need POMCP. Maybe for comparisson?
  return DSPOMDP::CreatePOMCPPrior(name);
}

void CooperativeMinefield::PrintState(const State &state, std::ostream &out) const {
  // FIXME: Add print functionality
  /*
   * Print the current state.
   */
}

void CooperativeMinefield::PrintObs(const State &state, OBS_TYPE observation, std::ostream &out) const {
  // FIXME: Add print functionality
  /*
   * Print the observation and the state in text. The observation is a 64bit int and they play with flags. We may
   * have to change it with an enum as we have really different observations each time. Also with the observation you
   * print the state as you understand it.
   */
}

void CooperativeMinefield::PrintBelief(const Belief &belief, std::ostream &out) const {
  // FIXME: Add print functionality
  // Print your belief about the environment. Like Get the average belief of mine probabilities etc. Must refine.
}

void CooperativeMinefield::PrintAction(int action, std::ostream &out) const {
  out << actionString[action] << std::endl;
}

State *CooperativeMinefield::Allocate(int state_id, double weight) const {
  CooperativeMinefieldState *state = memory_pool_.Allocate();
  state->state_id = state_id;
  state->weight = weight;
  return state;
}

State *CooperativeMinefield::Copy(const State *particle) const {
  CooperativeMinefieldState *state = memory_pool_.Allocate();
  *state = *static_cast<const CooperativeMinefieldState *>(particle);
  state->SetAllocated();
  return state;
}

void CooperativeMinefield::Free(State *particle) const {
  memory_pool_.Free(static_cast<CooperativeMinefieldState *>(particle));
}

int CooperativeMinefield::NumActiveParticles() const {
  return memory_pool_.num_allocated();
}

bool CooperativeMinefield::LocalMove(State &state, const History &history, int obs) const {
  // FIXME: Must fill in the function
  /*
   * Move the moving agent in a random square around him (based on the uncertainty sigma of a random distribution) and
   * check for consistency with the observations.
   *
   * Take the same action. Make observation and move the agent again.
   * Then return based on the agent's move and the observation consistency with history.
   *
   * This is done to cover the uncertainty in the position of the agent and the action success.
   */
  CooperativeMinefieldState &mstate = static_cast<CooperativeMinefieldState &>(state);
  Coord dir = Compass::DIRECTIONS[Random::RANDOM.NextInt(4)]; // FIXME: For now it is just one square.

  mstate.agent_pos_ += dir;

  for (int i = 0; i < num_mines_; ++i) {
    if (mine_pos_[i] == mstate.agent_pos_)
      return false;
  }

  int action = history.LastAction();

  // Must add a last affected square from the action.

  if (action == A_COMMUNICATE) {
    /*
     * 1. Reveal communicated target to moving agent if transmission successful. Else nothing changes.
     * 2. If transmission was successful calculate policy from scratch.
     * 3. Get a position obs and reduce the uncertainty to 0.
     */
  } else if (action == A_SEARCH) {
    /*
     * If was on a mine square reveal a mine with some probability. Else obs is none.
     */
  } else if (action == A_SENSE) {
    /*
     * Improve the certainty of a mine being real or not.
     */
  } else if (action == A_WAIT) {
    /*
     * You do nothing obs should be none.
     */
  }

  // Moving agent acts according to policy.
  // If agent didn't hit a mine and the obs matches the history return true else return false.

  return false;
}

Coord CooperativeMinefield::NextPos(const Coord &from, int dir) const {
  Coord ret = from + Compass::DIRECTIONS[dir];
  return ret;
}

void CooperativeMinefield::InitGeneral() {

  // TODO: Initialise variables.

  start_pos_ = Coord(0, 0);
  end_pos_ = Coord(size_ - 1, size_ - 1);
  grid_.SetAllValues(-1);
  for (int i = 0; i < num_mines_; ++i) {
    Coord pos;
    do {
      pos = Coord(Random::RANDOM.NextInt(size_), Random::RANDOM.NextInt(size_));
    } while (grid_(pos) >= 0 && ((pos.x != 0 || pos.y != 0) && ((pos.x != size_ - 1 || pos.y != size_ - 1))));
    grid_(pos) = i;
    mine_pos_.push_back(pos);
    mine_uncomm_.push_back(pos);
  }

  bool GridSearched = true;
  if (GridSearched) { //If GridSearched parameter
    for (int i = 0; i < size_; ++i) {
      for (int j = 0; j < size_; ++j) {
        searched_pos_.push_back(Coord(i, j));
      }
    }
  } else { //You are only sure about the start and the end positions
    searched_pos_.push_back(Coord(0, 0));
    searched_pos_.push_back(Coord(size_ - 1, size_ - 1));
  }
  NumActions();
  movePolicy = new MDPMinefield(size_, 0); //Initially he doesn't know of any mines.
  movePolicy->ComputeOptimalPolicyUsingVI();
  p = movePolicy->policy();

}

void CooperativeMinefield::InitializeTransitions() {
  // FIXME: Do we really need it?
}

Coord CooperativeMinefield::IndexToCoord(int pos) const {
  return Coord(pos % grid_.xsize(), pos / grid_.xsize());
}

int CooperativeMinefield::CoordToIndex(Coord c) const {
  return c.y * grid_.xsize() + c.x;
}

int CooperativeMinefield::GetObservation(double rand_num,
                                         const CooperativeMinefieldState &minestate,
                                         int action_idx) const {

  int action = actions[action_idx].first;
  switch (action) {
    case A_COMMUNICATE:
      if (rand_num < comms_error_rate_)
        return O_NONE;
      else
        return O_AGENT_POS;
    case A_SENSE:
      break; //TODO: Implement it as well. Initial application doesn't have any doubt of mines. Maybe has to do with
      // the sensor sensitivity.
    case A_SEARCH:
      break; //TODO: Use Poisson distribution for mine discovery. For now search is NOT implemented.
      /*
       * There should be a fixed sensor probability of discovering a mine if it actually exists there.
       * Also should check for false positives. I.E. present a mine where there is none.
       */
    case A_WAIT:return O_NONE;
  }
  return 0;
}

}