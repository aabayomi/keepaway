#ifndef LINEAR_SARSA_AGENT
#define LINEAR_SARSA_AGENT

#include "SMDPAgent.h"
#include "tiles2.h"
#include <vector>
#include <string>
#include <string>
#include "BasicPlayer.h"
#include <unordered_map>
#include "tiles2.h"
#include "HierarchicalFSM.h"
#include <cstddef>
#include <functional>
#include <iostream>
#include <stack>
#include <tuple>

#define RL_MEMORY_SIZE 1048576
#define RL_MAX_NONZERO_TRACES 100000
#define RL_MAX_NUM_TILINGS 6000

template <class T>
struct hash<vector<T>>
{
  size_t operator()(const vector<T> &vec) const
  {
    size_t seed = vec.size();
    for (auto &i : vec)
    {
      seed ^= hash<T>().operator()(i) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

typedef vector<string> machine_state_t;
typedef vector<int> num_choice_t;
typedef vector<int> choice_t;

/**
 * global memory among all machines from the perspective of a single machine similar to bai
 */
class Memory
{
private:
  Memory();

public:
  static Memory &ins();

  void resetState();

  std::string to_string();

  bool bAlive;
  int agentIdx;
  double state[MAX_RL_STATE_VARS]; // current state -- indexed by K0..Kn
  int ballControlState[11];        // current ball state -- indexed by K0..Kn
  ObjectT teammates[11];           // current mapping from index to teammates
  ObjectT opponents[11];           // current mapping from index to opponents

private:
  std::vector<string> stack; // self call stack

public:
  const vector<string> &getStack() const;

  void PushStack(const string &s);

  void PopStack();

  size_t ballControlHash();
};

struct SharedData
{
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

  int num_teammates;
  int num_opponents;

  void reset();
};

class LinearSarsaAgent : public SMDPAgent
{
protected:
  char weightsFile[256];
  bool bLearning;
  bool bSaveWeights;

  bool qLearning;
  string saveWeightsFile;
  SharedData *sharedData;
  string teamName;

  int lastJointChoiceIdx;
  int lastJointChoiceTime;

  /// Hive mind indicator and file descriptor.
  bool hiveMind;
  int hiveFile;

  int epochNum;
  int lastAction;

  // double alpha;
  // double gamma;
  // double lambda;
  // double epsilon;

  // double tileWidths[MAX_STATE_VARS];
  // double Q[MAX_ACTIONS];

  // double *weights;
  // double weightsRaw[RL_MEMORY_SIZE];
  // double traces[RL_MEMORY_SIZE];

  // int tiles[MAX_ACTIONS][RL_MAX_NUM_TILINGS];
  // int numTilings;

  // double minimumTrace;
  // int nonzeroTraces[RL_MAX_NONZERO_TRACES];
  // int numNonzeroTraces;
  // int nonzeroTracesInverse[RL_MEMORY_SIZE];

  // collision_table *colTab;

  // Load / Save weights from/to disk
  // bool loadWeights(const char *filename);
  // bool saveWeights(const char *filename);

  // Value function methods for CMACs
  int selectAction();
  void initializeTileWidths(int numK, int numT);
  double computeQ(int a);
  int argmaxQ();
  void updateWeights(double delta);
  virtual void loadTiles(double state[]);

  // Eligibility trace methods
  void clearTrace(int f);
  void clearExistentTrace(int f, int loc);
  void decayTraces(double decayRate);
  void setTrace(int f, float newTraceValue);
  void increaseMinTrace();

  // double initialWeight;
  // string sharedMemory;

public:
  LinearSarsaAgent(int numFeatures, int numTeammates,
                   int numOpponents,
                   int numActions,
                   bool bLearn,
                   double widths[],
                   double weight, bool qLearning,
                   string loadWeightsFile,
                   string saveWeightsFile,
                   bool hiveMind, string teamName);

  int num_features;
  int num_teammates;
  int num_opponents;

  // Support for extra modes and/or analysis.
  double getQ(int action);
  void setEpsilon(double epsilon);

  // SMDP Sarsa implementation
  int startEpisode(double state[]);
  int step(double reward, double state[]);
  int step(double state[]);
  void endEpisode(double reward);
  void setParams(int iCutoffEpisodes, int iStopLearningEpisodes);
  void shutDown();

  bool loadSharedData();
  void saveSharedData();
  // Load / Save weights from/to disk
  bool loadWeights(const char *filename);
  bool saveWeights(const char *filename);

  std::stack<std::tuple<int, int>> graph();
  double calculateQ(int i, int j, double reward, double state[]);
  double calculateQ(int i, int j, double reward, double state[], std::set<int> s);

private:
  unordered_map<string, Barrier *> barriers;

  double gamma;
  double lambda;
  double alpha;
  double epsilon;
  double tileWidths[MAX_RL_STATE_VARS];

  double *Q;
  int (*tiles)[RL_MAX_NUM_TILINGS];
  double *weights;
  double *traces;
  int *nonzeroTraces;
  int *nonzeroTracesInverse;
  collision_table *colTab;

  int numTilings;
  double minimumTrace;
  int numNonzeroTraces;

  // int selectChoice(const num_choice_t &num_choices);

  double initialWeight;
  string sharedMemory;
};

#endif
