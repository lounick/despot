//
// Created by jim on 30/6/2016.
//

#include "base_cooperative_minefield.h"

using namespace std;

namespace despot {

/* ==============================================================================
 * CooperativeMinefieldState class
 * ==============================================================================*/

CooperativeMinefieldState::CooperativeMinefieldState() {
}

CooperativeMinefieldState::CooperativeMinefieldState(int _state_id) {
  state_id = _state_id;
}

CooperativeMinefieldState::CooperativeMinefieldState(Coord agent_pos, std::vector<Coord> mine_pos,
                                                     std::vector<double> mine_prob,
                                                     std::vector<Coord> mine_comm,
                                                     std::vector<Coord> searched_pos)
    : agent_pos_(agent_pos), mine_pos_(mine_pos), mine_prob_(mine_prob), mine_comm_(mine_comm),
      searched_pos_(searched_pos) {

}

string CooperativeMinefieldState::text() const {
  return "id = " + to_string(state_id);
}

/* ==============================================================================
 * BaseCooperativeMinefield class
 * ==============================================================================*/

BaseCooperativeMinefield::BaseCooperativeMinefield(std::string map) {
  ifstream fin(map.c_str(), ifstream::in);
  string tmp;
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

  //InitStates();

  /*
   clock_t start = clock();
   cerr << "Initializing transitions" << endl;
   InitializeTransitions();
   cerr << "Done " << (double (clock() - start) / CLOCKS_PER_SEC) << endl;
   ComputeOptimalPolicyUsingVI();
   */
}

BaseCooperativeMinefield::BaseCooperativeMinefield(int size, int mines) : grid_(size, size), size_(size),
                                                                          num_mines_(mines) {
  InitGeneral();
  // InitStates();
}

void BaseCooperativeMinefield::InitStates() {
  states_.resize(NumStates());
  for (int s = 0; s < NumStates(); ++s) {
    states_[s] = new CooperativeMinefieldState(s);
  }
}

void BaseCooperativeMinefield::InitGeneral() {
  start_pos_ = Coord(0, 0);
  grid_.SetAllValues(-1);
  for (int i = 0; i < num_mines_; ++i) {
    Coord pos;
    do {
      pos = Coord(Random::RANDOM.NextInt(size_), Random::RANDOM.NextInt(size_));
    } while (grid_(pos) >= 0 && ((pos.x != 0 || pos.y != 0) && ((pos.x != size_ - 1 || pos.y != size_ - 1))));
    grid_(pos) = i;
    mine_pos_.push_back(pos);
  }

  bool GridSearched = true;
  if(GridSearched){ //If GridSearched parameter
    for(int i = 0; i < size_; ++i){
      for(int j = 0; j < size_; ++j){
        searched_pos_.push_back(Coord(i,j));
      }
    }
  }else{ //You are only sure about the start and the end positions
    searched_pos_.push_back(Coord(0,0));
    searched_pos_.push_back(Coord(size_-1,size_-1));
  }

}

State *BaseCooperativeMinefield::CreateStartState(std::string type) const {
  return new CooperativeMinefieldState(start_pos_,
                                       mine_pos_,
                                       std::vector<double>(num_mines_, 1),
                                       std::vector<Coord>(),
                                       searched_pos_);
}

}