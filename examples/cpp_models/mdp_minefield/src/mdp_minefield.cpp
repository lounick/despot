//
// Created by jim on 30/6/2016.
//

#include "mdp_minefield.h"

namespace despot {

MinefieldState::MinefieldState() {

}

MinefieldState::MinefieldState(int _state_id) {
  state_id = _state_id;
}

std::string MinefieldState::text() const {
  return "id = " + to_string(state_id);
}

Coord MinefieldState::getPos() {
  return pos_;
}


void MinefieldState::setPos(Coord pos) {
  pos_ = pos;
}


MDPMinefield::MDPMinefield(int size, int num_mines) : grid_(size, size),
                                                      size_(size),
                                                      num_mines_(num_mines) {
  InitGeneral();
}

MDPMinefield::MDPMinefield() : grid_(4, 4),
                               size_(4),
                               num_mines_(3) {
/*
 * Default constructor. Generate a sample minefield problem.
 */
  InitGeneral();
}

void MDPMinefield::InitStates() {
  states_.resize(NumStates());
  for (int s = 0; s < NumStates(); ++s) {
    states_[s] = new MinefieldState(s);
    states_[s]->setPos(IndexToCoord(s));
  }
}

void MDPMinefield::InitGeneral() {
  start_pos_ = Coord(0, 0);
  end_pos_ = Coord(size_ - 1, size_ - 1);
  grid_.SetAllValues(-1);
  grid_(start_pos_) = num_mines_;
  grid_(end_pos_) = num_mines_;
  for (int i = 0; i < num_mines_; i++) {
    Coord pos;
    do {
      pos = Coord(Random::RANDOM.NextInt(size_), Random::RANDOM.NextInt(size_));
    } while (grid_(pos) >= 0);
    grid_(pos) = i;
    mine_pos_.push_back(pos);
  }
}

bool MDPMinefield::Step(State &state, double rand_num, int action, double &reward, OBS_TYPE &obs) {
  return false;
}

int MDPMinefield::NumActions() {
  return 4;
}

double MDPMinefield::ObsProb(OBS_TYPE obs, const State &state, int action) {
  return 0;
}

const std::vector<State> &MDPMinefield::TransitionProbability(int s, int a) const {
  return transition_probabilities_[s][a];
}

int MDPMinefield::NextState(int s, int a) const {
  Coord pos = states_[s]->getPos();

  if (pos == end_pos_)
    return s;

  pos += Compass::DIRECTIONS[a];
}

double MDPMinefield::Reward(int s, int a) const {
  return 0;
}

State *MDPMinefield::CreateStartState(std::string type) const {
  return NULL;
}

std::vector<State *> MDPMinefield::InitialParticleSet() const {
  return std::vector<State *>();
}

std::vector<State *> MDPMinefield::NoisyInitialParticleSet() const {
  return std::vector<State *>();
}

Belief *MDPMinefield::InitialBelief(const State *start, std::string type) const {
  return NULL;
}

void MDPMinefield::PrintState(const State &state, std::ostream &out) const {

}

void MDPMinefield::PrintBelief(const Belief &belief, std::ostream &out) const {

}

void MDPMinefield::PrintObs(const State &state, OBS_TYPE observation, std::ostream &out) const {

}

void MDPMinefield::PrintAction(int action, std::ostream &out) const {

}

Belief *MDPMinefield::Tau(const Belief *belief, int action, OBS_TYPE obs) const {
  return NULL;
}

void MDPMinefield::Observe(const Belief *belief, int action, std::map<OBS_TYPE, double> &obss) const {

}

double MDPMinefield::StepReward(const Belief *belief, int action) const {
  return 0;
}

int MDPMinefield::NumStates() const {
  return grid_.xsize() * grid_.ysize();
}

const State *MDPMinefield::GetState(int index) const {
  return NULL;
}

int MDPMinefield::GetIndex(const State *state) const {
  return 0;
}

int MDPMinefield::GetRobPosIndex(const State *state) const {
  return 0;
}

Coord MDPMinefield::GetRobPos(const State *state) const {
  return despot::Coord();
}

bool MDPMinefield::GetRock(const State *state, int rock) const {
  return false;
}

void MDPMinefield::SampleRock(State *state, int rock) const {

}

int MDPMinefield::GetX(const State *state) const {
  return 0;
}

void MDPMinefield::IncX(State *state) const {

}

void MDPMinefield::DecX(State *state) const {

}

int MDPMinefield::GetY(const State *state) const {
  return 0;
}

void MDPMinefield::IncY(State *state) const {

}

void MDPMinefield::DecY(State *state) const {

}

void MDPMinefield::InitializeTransitions() {
  int num_states = NumStates(), num_actions = NumActions();
  transition_probabilities_.reserve(num_states);
  for (int s = 0; s < num_states; ++s) {
    transition_probabilities_[s].resize(num_actions);
    for (int a = 0; a < num_actions; a++) {
      State state;
      state.state_id = NextState(s, a);
      state.weight = 1.0;
      transition_probabilities_[s][a].push_back(state);
    }
  }
}

Coord MDPMinefield::IndexToCoord(int pos) const {
  return despot::Coord();
}

int MDPMinefield::CoordToIndex(Coord c) const {
  return 0;
}

}