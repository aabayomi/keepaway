#ifndef LINEAR_SARSA_AGENT
#define LINEAR_SARSA_AGENT

#include <iostream>
#include <sstream>
#include "SMDPAgent.h"
#include "tiles2.h"

#define RL_MEMORY_SIZE (2 << 20)
#define RL_MAX_NONZERO_TRACES (2 << 17)
#define RL_MAX_NUM_TILINGS (2 << 14)

class FileLock {
private:
  int lock;
  std::string lockName;

public:
  FileLock(const std::string name);
  ~FileLock();
};

class LinearSarsaAgent : public SMDPAgent {
protected:
  char weightsFile[256];
  bool bLearning;
  bool bSaveWeights;
  bool jointTiling;

  /// Hive mind indicator and file descriptor.
  int hiveMind; // 1: hive mind, 2: full hive mind
  int hiveFile;

  double alpha;
  double gamma;
  double lambda;
  double epsilon;

  double tileWidths[MAX_STATE_VARS];
  double Q[MAX_ACTIONS];

  double *weights;
  double weightsRaw[RL_MEMORY_SIZE];

  int tiles[MAX_ACTIONS][RL_MAX_NUM_TILINGS];
  int numTilings;

  double minimumTrace;
  int numNonzeroTraces;

  double *traces;
  int *nonzeroTraces;
  int *nonzeroTracesInverse;

  double tracesRaw[RL_MEMORY_SIZE];
  int nonzeroTracesRaw[RL_MAX_NONZERO_TRACES];
  int nonzeroTracesInverseRaw[RL_MEMORY_SIZE];

  collision_table *colTab;

  // Load / Save weights from/to disk
  bool loadWeights(char *filename);

  bool saveWeights(char *filename);

  // Value function methods for CMACs
  int selectAction(double state[]);

  double computeQ(int a);

  int argmaxQ(double state[]) const;

  void updateWeights(double delta);

  virtual void loadTiles(double state[]);

  // Eligibility trace methods
  void clearTrace(int f);

  void clearExistentTrace(int f, int loc);

  void decayTraces(double decayRate);

  void setTrace(int f, float newTraceValue);

  void increaseMinTrace();

  size_t mmapSize();

  long *loadSharedData(collision_table *colTab, double *weights);
  virtual void sync();

  double reward(double tau, double gamma);

public:
  LinearSarsaAgent(int numFeatures,
                   bool bLearn,
                   double widths[],
                   char *loadWeightsFile,
                   char *saveWeightsFile,
                   int hiveMind,
                   bool jointTiling,
                   double gamma);

  // Support for extra modes and/or analysis.
  double getQ(int action);

  void setEpsilon(double epsilon);

  // SMDP Sarsa implementation
  int startEpisode(int current_time, double state[]);

  int step(int current_time, double reward_, double state[]);

  void endEpisode(int current_time, double reward_);

  void setParams(int iCutoffEpisodes, int iStopLearningEpisodes);

  void shutDown();
};

#endif
