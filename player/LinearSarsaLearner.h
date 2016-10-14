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
#include "HierarchicalFSM.h"
#include <cstddef>
#include <functional>

namespace std {

template<>
struct hash<vector<int>> {
  size_t operator()(const vector<int> &vec) const {
    size_t seed = vec.size();
    for (auto &i : vec) {
      seed ^= hash<int>()(i) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

}

namespace fsm {
struct SharedData {
  double Q[MAX_RL_ACTIONS];
  double weights[RL_MEMORY_SIZE];
  double traces[RL_MEMORY_SIZE];
  int nonzeroTraces[RL_MAX_NONZERO_TRACES];
  int nonzeroTracesInverse[RL_MEMORY_SIZE];
  collision_table colTab;

  int numTilings;
  double minimumTrace;
  int numNonzeroTraces;

  int numChoices[OBJECT_MAX_OBJECTS]; // indexed by object type
  char machineStateStr[OBJECT_MAX_OBJECTS][1024];

  int lastJointChoiceIdx;
  int lastJointChoiceTime;
  int lastJointChoice[11]; // indexed by K0..Kn

  vector<int> getNumChoices() const;

  vector<int> getLastJointChoice() const;

  vector<string> getMachineStateStr() const;

  void setBlocked(int i);

  void clearBlocked(int i);

  bool isAllBlocked(int k);

  bool isBlocked(int i);

  void reset();

private:
  int runningStatus;

public:
  int getRunningStatus() const;

public:
  void clearBlocked();
};

class HierarchicalFSM;

/**
 * Linear SARSA agent learning to make choice conditioned on env and machine states
 */
class LinearSarsaLearner {
private:
  LinearSarsaLearner();

  ~LinearSarsaLearner();

public:
  static LinearSarsaLearner &ins();

  void initialize(
      bool learning, double width[], double weight, bool qLearning,
      string loadWeightsFile,
      string saveWeightsFile);

  void shutDown();

  int step(int current_time, int num_choices);

  int step(int current_time);

  void endEpisode(int current_time);

  bool loadSharedData();

  void saveSharedData();

  void wait();

  void notify(int i);

  // Load / Save weights from/to disk
  bool loadWeights(const char *filename);

  bool saveWeights(const char *filename);

public:
  int lastJointChoiceIdx;
  int lastJointChoiceTime;
  vector<string> machineStateStr; // indexed by K0..Kn
  vector<int> numChoices;
  vector<int> lastJointChoice;

private:
  bool bLearning;
  string saveWeightsFile;
  bool bSaveWeights;
  SharedData *sharedData;

private:
  sem_t *semSignal[11];
  sem_t *semSync;

  double alpha;
  double lambda;
  double epsilon;

  double tileWidths[MAX_RL_STATE_VARS];
  int tiles[MAX_RL_ACTIONS][RL_MAX_NUM_TILINGS];

  double *Q;
  double *weights;
  double *traces;
  int *nonzeroTraces;
  int *nonzeroTracesInverse;
  collision_table *colTab;

  int numTilings;
  double minimumTrace;
  int numNonzeroTraces;

  void loadTiles(
      double state[], const vector<string> &machine_state, const vector<int> &num_choices);

  int selectChoice(const vector<int> &num_choices);

  double computeQ(int choice);

  int argmaxQ(const vector<int> &num_choices);

  void updateWeights(double delta);

  void decayTraces(double decayRate);

  void clearTrace(int f);

  void clearExistentTrace(int f, int loc);

  void setTrace(int f, float newTraceValue);

  void increaseMinTrace();

  const vector<int> &validChoices(const vector<int> &num_choices);

  vector<vector<int>> validChoicesRaw(const vector<int> &num_choices);

  double initialWeight;
  string sharedMemoryName;

  unordered_map<vector<int>, vector<int>> validChoicesMap;
  unordered_map<vector<int>, vector<vector<int>>> jointChoicesMap;
  bool qLearning;

  double reward(double tau);
};

}
#endif //KEEPAWAY_PLAYER_LINEARSARSALEARNER_H
