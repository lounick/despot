//
// Created by Nick Tsiogkas on 30/6/2016.
//

#include "mdp_minefield.h"

using namespace std;
using namespace despot;

int main(){
  MDPMinefield model(4, 3);
  model.ComputeOptimalPolicyUsingVI();
  model.PrintWorld(cout);
  vector<ValuedAction> p = model.policy();
  for (int i  = 0; i < p.size(); ++i){
    cout << p[i].action << " " << p[i].value << endl;
  }
}