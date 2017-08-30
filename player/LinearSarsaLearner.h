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

template<class T>
struct hash<vector<T>> {
  size_t operator()(const vector<T> &vec) const {
    size_t seed = vec.size();
    for (auto &i : vec) {
      seed ^= hash<T>().operator()(i) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

}

namespace fsm {

typedef vector<string> machine_state_t;
typedef vector<int> num_choice_t;
typedef vector<int> choice_t;
typedef unordered_map<machine_state_t, \
    unordered_map<int, \
        unordered_map<machine_state_t, double>>> transition_t;

struct SharedData {
  double Q[MAX_RL_ACTIONS];
  int tiles[MAX_RL_ACTIONS][RL_MAX_NUM_TILINGS];
  double weights[RL_MEMORY_SIZE];
  double traces[RL_MEMORY_SIZE];
  int nonzeroTraces[RL_MAX_NONZERO_TRACES];
  int nonzeroTracesInverse[RL_MEMORY_SIZE];
  collision_table colTab;

  int numTilings;
  double minimumTrace;
  int numNonzeroTraces;

  int numChoices[OBJECT_MAX_OBJECTS]; // indexed by object type
  char machineState[OBJECT_MAX_OBJECTS][MAX_MSG];

  int lastJointChoiceIdx;
  int lastJointChoiceTime;
  int lastJointChoice[11]; // indexed by K0..Kn
  char lastMachineState[11][MAX_MSG];

  num_choice_t getNumChoices() const;

  choice_t getLastJointChoice() const;

  machine_state_t getMachineState() const;

  machine_state_t getLastMachineState() const;

  void reset();
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
      bool learning, double width[],
      double gamma, double lambda, double alpha,
      double weight, bool qLearning,
      string loadWeightsFile,
      string saveWeightsFile,
      string teamName);

  void shutDown();

  int step(int current_time, int num_choices);

  int step(int current_time);

  void endEpisode(int current_time);

  bool loadSharedData();

  void saveSharedData();

  // Load / Save weights from/to disk
  bool loadWeights(const char *filename);

  bool saveWeights(const char *filename);

public:
  int lastJointChoiceIdx;
  int lastJointChoiceTime;
  choice_t lastJointChoice;
  machine_state_t machineState; // indexed by K0..Kn
  machine_state_t lastMachineState; // indexed by K0..Kn
  num_choice_t numChoices;

private:
  bool bLearning;
  bool qLearning;
  string saveWeightsFile;
  bool bSaveWeights;
  SharedData *sharedData;
  string teamName;

private:
  unordered_map<string, Barrier *> barriers;

  double gamma;
  double lambda;
  double alpha;
  double epsilon;
  double tileWidths[MAX_RL_STATE_VARS];

  double *Q;
  int (*tiles_)[RL_MAX_NUM_TILINGS];
  double *weights;
  double *traces;
  int *nonzeroTraces;
  int *nonzeroTracesInverse;
  collision_table *colTab;

  int numTilings;
  double minimumTrace;
  int numNonzeroTraces;

  int loadTiles(
      double state[],
      const machine_state_t &machine_state,
      const num_choice_t &num_choices,
      int (*tiles)[RL_MAX_NUM_TILINGS]);

  int selectChoice(const num_choice_t &num_choices);

  double computeQ(int choice, int (*tiles)[RL_MAX_NUM_TILINGS], int numTilings);

  double QValue(double *state,
                const machine_state_t &machine_state,
                int choice,
                int (*tiles)[RL_MAX_NUM_TILINGS],
                int num_tilings);

  double Value(double *state, const machine_state_t &machine_state);

  int argmaxQ(const num_choice_t &num_choices);

  void updateWeights(double delta, int num_tilings);

  void decayTraces(double decayRate);

  void clearTrace(int f);

  void clearExistentTrace(int f, int loc);

  void setTrace(int f, float newTraceValue);

  void increaseMinTrace();

  const vector<int> &validChoices(const num_choice_t &num_choices);

  vector<choice_t> validChoicesRaw(const num_choice_t &num_choices);

  double initialWeight;
  string sharedMemory;

  unordered_map<num_choice_t, vector<int>> validChoicesMap;
  unordered_map<num_choice_t, vector<choice_t>> jointChoicesMap;
  unordered_map<machine_state_t, num_choice_t> numChoicesMap;
  unordered_map<size_t, transition_t> staticTransitions;

  double reward(double tau);
};

}
#endif //KEEPAWAY_PLAYER_LINEARSARSALEARNER_H
