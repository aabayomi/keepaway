
#ifndef CROSSENTROPYAGENT
#define CROSSENTROPYAGENT

// #ifndef LINEAR_SARSA_AGENT
// #define LINEAR_SARSA_AGENT

#include "SMDPAgent.h"
#include "tiles2.h"
#include <array>

#include <vector>
#include <string>
#include "BasicPlayer.h"
#include <map>
#include "tiles2.h"
#include "HierarchicalFSM.h"
#include <cstddef>
#include <functional>
#include <eigen3/Eigen/Dense>
#include <unordered_map>


// #define RL_MEMORY_SIZE 1048576
// #define RL_MAX_NONZERO_TRACES 100000
// #define RL_MAX_NUM_TILINGS 6000


// namespace std {

// template<class T>
// struct hash<vector<T>> {
//   size_t operator()(const vector<T> &vec) const {
//     size_t seed = vec.size();
//     for (auto &i : vec) {
//       seed ^= hash<T>().operator()(i) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
//     }
//     return seed;
//   }
// };

// }


using namespace Eigen;

class CrossEntropyAgent:public SMDPAgent

{
 protected:
  char weightsFile[256];
  bool bLearning;
  bool bSaveWeights;

  // Hive mind indicator and file descriptor.
  bool hiveMind;
  int hiveFile;
  
  int epochNum;
  int lastAction;
  double epsilon;

  int lastActionsTime;
  
  WorldModel *WM;  /*!< WorldModel that contains information of world   */
  
  // normal distribution variables
  double mean;
  double std;
  int N;
  int counter;
  int k;
  double tempReward;
  double maxReward;

  int popSize;
  int eliteFrac;

  double lastActionTime;

  std::map< array<double, RL_MEMORY_SIZE>, double> samples;
  unordered_map<double, array<double, RL_MEMORY_SIZE>> setOfWeights;

  unordered_map<double, double(*)[RL_MEMORY_SIZE]> S;

  std::map<double, double(*)[RL_MEMORY_SIZE]> setOfW;

  // std::map<double,Triple>R;

  std::vector<double> temp;

  ofstream myFile;
  


  double alpha;
  double gamma;
  double lambda;
  // double epsilon;

  //int batchSize;

  double tileWidths[ MAX_RL_STATE_VARS ];
  double Q[MAX_RL_ACTIONS ];

  double* initialWeights;
  double* t;
  double weightsRaw[ RL_MEMORY_SIZE ];

  array<double,RL_MEMORY_SIZE> weights;
  

  array<double,RL_MEMORY_SIZE> tempWeights;
  

  // RowVectorXd = Eigen::Matrix<double, 1, Eigen::Dynamic>
  // typedef Matrix<double, 1,RL_MEMORY_SIZE > tempWeights;
  // RowVectorXd tempWeights;
  // RowVectorXd initialWeights;
 
  // RowVectorXd newWeights(1048576);


  // array<double,RL_MEMORY_SIZE> tempWeights;


 // double traces[ RL_MEMORY_SIZE ];

  int tiles[MAX_RL_ACTIONS ][ RL_MAX_NUM_TILINGS ];
  int numTilings;

  double minimumTrace;
  int nonzeroTraces[ RL_MAX_NONZERO_TRACES ];
  int numNonzeroTraces;
  int nonzeroTracesInverse[ RL_MEMORY_SIZE ];

  collision_table *colTab;
  
  // Load / Save weights from/to disk
  // bool loadWeights( string filename );
  bool loadWeights(const char *filename);

  // bool saveWeights( char *filename );
  bool saveWeights(const char *filename);


  // bool sortByVal(const pair<array<double, RL_MEMORY_SIZE>, double> &a, 
  //              const pair<array<double, RL_MEMORY_SIZE>, double> &b);

  void weightsLog();
  // Value function methods for CMACs
  int  selectAction();
  //void initializeTileWidths( int numK, int numT );
  void createFile(const   char *filename);
  double computeQ( int a );
  int  argmaxQ();
  void updateWeights();
  void oneUpdate();
  void loadTiles( double state[] );
  void weightsToString(const char* filename);
  void updateweightsEndEpisode();

  // Eligibility trace methods
  double traces[ RL_MEMORY_SIZE ];
  void clearTrace( int f );
  void clearExistentTrace( int f, int loc );
  void decayTraces( double decayRate );
  void setTrace( int f, float newTraceValue );
  void increaseMinTrace();

 public:
  CrossEntropyAgent                  ( int    numFeatures,
                                      int    numActions,
                                      bool   bLearn,
                                      double widths[],
                                      string   loadWeightsFile,
                                      string   saveWeightsFile,
                                      bool   hiveMind,
                                      WorldModel *ws);

  // Support for extra modes and/or analysis.
  double getQ(int action);
  //void setEpsilon(double epsilon);

  // SMDP Sarsa implementation
  // int  startEpisode( int current_time, double state[] );
  // int  step( int current_time, double reward, double state[] );
  // void endEpisode( int current_time, double reward );
  // void setParams(int iCutoffEpisodes, int iStopLearningEpisodes);
  // void shutDown();

  int  startEpisode( double state[] );
  int  step( double reward, double state[] );
  void endEpisode( double reward );
  void setParams(int iCutoffEpisodes, int iStopLearningEpisodes); //*met 8/16/05

  // Optional customization point.
  void shutDown();

} ;

#endif