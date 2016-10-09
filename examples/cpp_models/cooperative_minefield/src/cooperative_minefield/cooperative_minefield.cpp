//
// Created by jim on 30/6/2016.
//

#include "cooperative_minefield.h"

despot::CooperativeMinefieldState::CooperativeMinefieldState() {

}
despot::CooperativeMinefieldState::CooperativeMinefieldState(int _state_id) {
  state_id = _state_id;
}
despot::CooperativeMinefieldState::CooperativeMinefieldState(despot::Coord agent_pos,
                                                             std::vector<despot::Coord> mine_pos,
                                                             std::vector<double> mine_prob,
                                                             std::vector<despot::Coord> mine_comm,
                                                             std::vector<despot::Coord> searched_pos) {
  agent_pos_ = agent_pos;
  mine_pos_ = mine_pos;
  mine_prob_ = mine_prob;
  mine_comm_ = mine_comm;
  searched_pos_ = searched_pos;
}
std::string despot::CooperativeMinefieldState::text() const {
  return "id = " + to_string(state_id);
}

despot::CooperativeMinefieldBelief::CooperativeMinefieldBelief(std::vector<State *> particles,
                                                               const despot::DSPOMDP *model,
                                                               despot::Belief *prior) : ParticleBelief(particles,
                                                                                                       model,
                                                                                                       prior) {
  cooperativeMinefield_ = (CooperativeMinefield*) model;
}
void despot::CooperativeMinefieldBelief::Update(int action, despot::OBS_TYPE obs) {
  ParticleBelief::Update(action, obs);
}

despot::CooperativeMinefield::CooperativeMinefield(std::string map) {

}
despot::CooperativeMinefield::CooperativeMinefield(int size, int mines) {

}
bool despot::CooperativeMinefield::Step(despot::State &state,
                                        double rand_num,
                                        int action,
                                        double &reward,
                                        despot::OBS_TYPE &obs) const {
  return false;
}
int despot::CooperativeMinefield::NumActions() const {
  return 4;
}
double despot::CooperativeMinefield::ObsProb(despot::OBS_TYPE obs, const despot::State &state, int action) const {
  return 0;
}
despot::State *despot::CooperativeMinefield::CreateStartState(std::string type) const {
  return NULL;
}
despot::Belief *despot::CooperativeMinefield::InitialBelief(const despot::State *start, std::string type) const {
  return NULL;
}
despot::ScenarioUpperBound *
despot::CooperativeMinefield::CreateScenarioUpperBound(std::string name, std::string particle_bound_name) const {
  return NULL;
}
despot::ParticleLowerBound *despot::CooperativeMinefield::CreateParticleLowerBound(std::string name) const {
  return NULL;
}
despot::ScenarioLowerBound *
despot::CooperativeMinefield::CreateScenarioLowerBound(std::string name, std::string particle_bound_name) const {
  return NULL;
}
despot::POMCPPrior *despot::CooperativeMinefield::CreatePOMCPPrior(std::string name) const {
  return DSPOMDP::CreatePOMCPPrior(name);
}
void despot::CooperativeMinefield::PrintState(const despot::State &state, std::ostream &out) const {

}
void despot::CooperativeMinefield::PrintObs(const despot::State &state,
                                            despot::OBS_TYPE observation,
                                            std::ostream &out) const {

}
void despot::CooperativeMinefield::PrintBelief(const despot::Belief &belief, std::ostream &out) const {

}
void despot::CooperativeMinefield::PrintAction(int action, std::ostream &out) const {

}
despot::State *despot::CooperativeMinefield::Allocate(int state_id, double weight) const {
  return NULL;
}
despot::State *despot::CooperativeMinefield::Copy(const despot::State *particle) const {
  return NULL;
}
void despot::CooperativeMinefield::Free(despot::State *particle) const {

}
int despot::CooperativeMinefield::NumActiveParticles() const {
  return 0;
}
bool despot::CooperativeMinefield::LocalMove(despot::State &state, const despot::History &history, int obs) const {
  return false;
}
despot::Coord despot::CooperativeMinefield::NextPos(const despot::Coord &from, int dir) const {
  return despot::Coord();
}

