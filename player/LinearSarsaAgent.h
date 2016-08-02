#ifndef LINEAR_SARSA_AGENT
#define LINEAR_SARSA_AGENT

#include <iostream>
#include <sstream>
#include "SMDPAgent.h"
#include "tiles2.h"

#define RL_MEMORY_SIZE 1048576
#define RL_MAX_NONZERO_TRACES 100000
#define RL_MAX_NUM_TILINGS 6000

class FileLock
{
private:
  int lock;
  std::string lockName;

public:
  FileLock(const std::string name, int ms, int max_loops = 5);
  ~FileLock();
};

class LinearSarsaAgent:public SMDPAgent
{
protected:
  char weightsFile[256];
  bool bLearning;
  bool bSaveWeights;

  /// Hive mind indicator and file descriptor.
  int hiveMind; // 0: hive mind, 1: full hive mind
  int hiveFile;

  int lastAction;
  int lastActionTime;
  bool wait4Episode; // waiting for new episode

  double alpha;
  double gamma;
  double lambda;
  double epsilon;

  double tileWidths[ MAX_STATE_VARS ];
  double Q[ MAX_ACTIONS ];

  double* weights;
  double weightsRaw[ RL_MEMORY_SIZE ];

  int tiles[ MAX_ACTIONS ][ RL_MAX_NUM_TILINGS ];
  int numTilings;

  double minimumTrace;
  double traces[ RL_MEMORY_SIZE ];
  int nonzeroTraces[ RL_MAX_NONZERO_TRACES ];
  int numNonzeroTraces;
  int nonzeroTracesInverse[ RL_MEMORY_SIZE ];

  collision_table *colTab;

  // Load / Save weights from/to disk
  bool loadWeights( char *filename );
  bool saveWeights( char *filename );

  // Value function methods for CMACs
  int  selectAction();
  double computeQ( int a );
  int  argmaxQ();
  void updateWeights( double delta );
  virtual void loadTiles( double state[] );

  // Eligibility trace methods
  void clearTrace( int f );
  void clearExistentTrace( int f, int loc );
  void decayTraces( double decayRate );
  void setTrace( int f, float newTraceValue );
  void increaseMinTrace();

  void reset();

public:
  LinearSarsaAgent                  ( int    numFeatures,
                                      int    numActions,
                                      bool   bLearn,
                                      double widths[],
                                      char   *loadWeightsFile,
                                      char   *saveWeightsFile,
                                      int    hiveMind);

  // Support for extra modes and/or analysis.
  double getQ(int action);
  void setEpsilon(double epsilon);

  // SMDP Sarsa implementation
  int  startEpisode( int current_time, double state[] );
  int  step( int current_time, double reward, double state[] );
  void endEpisode( int current_time, double reward );
  void setParams(int iCutoffEpisodes, int iStopLearningEpisodes);
  void shutDown();

  long* loadSharedData(collision_table *colTab, double *weights);
} ;

#endif
