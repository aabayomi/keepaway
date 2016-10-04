//
// Created by baj on 10/4/16.
//

#ifndef KEEPAWAY_PLAYER_LINEARSARSALEARNER_H
#define KEEPAWAY_PLAYER_LINEARSARSALEARNER_H

#include <vector>
#include <string>
#include "BasicPlayer.h"
#include <unordered_map>
#include "tiles2.h"

namespace fsm {
class HierarchicalFSM;
}

/**
 * Linear SARSA agent learning to make choice conditioned on env and machine states
 */
class LinearSarsaLearner {
public:
  LinearSarsaLearner(fsm::HierarchicalFSM *machine);

  int step(int num_choices);

private:
  fsm::HierarchicalFSM *machine;
  bool bLearning;

  double alpha;
  double gamma;
  double lambda;
  double epsilon;

  double tileWidths[MAX_RL_STATE_VARS];
  double Q[MAX_RL_ACTIONS];
  double weights[RL_MEMORY_SIZE];

  int tiles[MAX_RL_ACTIONS][RL_MAX_NUM_TILINGS];
  int numTilings;

  double minimumTrace;
  int numNonzeroTraces;

  double traces[RL_MEMORY_SIZE];
  int nonzeroTraces[RL_MAX_NONZERO_TRACES];
  int nonzeroTracesInverse[RL_MEMORY_SIZE];

  collision_table *colTab;

  void loadTiles(
      double state[], const vector<string> &stack, int num_choices);

  int selectChoice(int num_choices);

  double computeQ(int c);

  int argmaxQ(int num_choices);

  void updateWeights(double delta);

  void decayTraces(double decayRate);

  void clearTrace(int f);

  void clearExistentTrace(int f, int loc);

  void setTrace(int f, float newTraceValue);

  void increaseMinTrace();
};


#endif //KEEPAWAY_PLAYER_LINEARSARSALEARNER_H
