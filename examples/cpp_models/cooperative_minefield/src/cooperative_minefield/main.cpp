//
// Created by nick on 15/07/16.
//

#include <despot/simple_tui.h>
#include "cooperative_minefield.h"

using namespace std;
using namespace despot;

class TUI: public SimpleTUI{
 public:
  TUI(){

  }

  DSPOMDP* InitializeModel(option::Option* options){
    DSPOMDP* model = NULL;
    if(options[E_PARAMS_FILE]){
      model = new CooperativeMinefield(options[E_PARAMS_FILE].arg);
    } else {
      int size = 4, number = 2;
      if (options[E_SIZE]){
        size = atoi(options[E_SIZE].arg);
      } else {
        cerr << "Specify map size using --size option" << endl;
        exit(0);
      }
      if(options[E_NUMBER]){
        number = atoi(options[E_NUMBER].arg);
      } else {
        cerr << "Specify number of rocks using --number option" << endl;
        exit(0);
      }
      model = new CooperativeMinefield(size, number);
    }
    return model;
  }

  void InitializeDefaultParameters(){
  }
};

int main (int argc, char* argv[]){
  return TUI().run(argc, argv);
}