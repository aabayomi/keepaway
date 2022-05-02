#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <sstream>
#include <random>
#include <vector>
#include <algorithm>
#include <utility>
#include <array>
#include <eigen3/Eigen/Dense>
#include "gzstream.h"
#include <boost/algorithm/string/replace.hpp>
#include "crossEntropyLearner.h"
#include "LoggerDraw.h"


using namespace Eigen;
// If all is well, there should be no mention of anything keepaway- or soccer-
// related in this file. 

/**
 * Designed specifically to match the serialization format for collision_table.
 * See collision_table::save and collision_table::restore.
 */
#pragma pack(push, 1)
struct CollisionTableHeader {
  long m;
  int safe;
  long calls;
  long clearhits;
  long collisions;
};

#pragma pack(pop)
#define VERBOSE_HIVE_MIND false


// /**
//  * Assumes collision table header follows weights.
//  * Returns the memory location after the header because that's useful for colTab
//  * data array.
//  */
// long* loadColTabHeader(collision_table* colTab, double* weights) {
//   CollisionTableHeader* colTabHeader =
//     reinterpret_cast<CollisionTableHeader*>(weights + RL_MEMORY_SIZE);
//   // Do each field individually, since they don't all line up exactly for an
//   // easy copy.
//   colTab->calls = colTabHeader->calls;
//   colTab->clearhits = colTabHeader->clearhits;
//   colTab->collisions = colTabHeader->collisions;
//   colTab->m = colTabHeader->m;
//   colTab->safe = colTabHeader->safe;
//   if (VERBOSE_HIVE_MIND) {
//     cout << "Loaded colTabHeader:" << endl
//       << " calls: " << colTab->calls << endl
//       << " clearhits: " << colTab->clearhits << endl
//       << " collisions: " << colTab->collisions << endl
//       << " m: " << colTab->m << endl
//       << " safe: " << colTab->safe << endl;
//   }
//   return reinterpret_cast<long*>(colTabHeader + 1);
// }


extern LoggerDraw LogDraw;





CrossEntropyAgent::CrossEntropyAgent(int numFeatures, int numActions, bool bLearn,
                                    double widths[],
                                    string loadWeightsFile, string saveWeightsFile ):

  SMDPAgent( numFeatures, numActions )

{
  
  
  bLearning = bLearn;
  bSaveWeights = bLearning && saveWeightsFile.length() > 0;
  saveWeightsFile = saveWeightsFile;
  // teamName = teamName_;



  for ( int i = 0; i < getNumFeatures(); i++ ) {
    tileWidths[ i ] = widths[ i ];
  }

  // number of iterations for updating weights
  mean = 0.0;
  std = 100.0;
  N = 25;
  k = 10; // k best weights
  std::default_random_engine generator;
  std::normal_distribution<double> distribution(mean,std);
  
  counter = 0;
  epsilon = 0.01;

  epochNum = 0;
  lastAction = -1;


  srand( (unsigned int) 0 );
  srand48((unsigned int) 0);
  int tmp[ 2 ];
  float tmpf[ 2 ];

  // colTab = new collision_table( RL_MEMORY_SIZE, 1 );
  // colTab = 
  colTab = new collision_table; colTab;

  GetTiles( tmp, 1, 1, tmpf, 0 );  // A dummy call to set the hashing table    
  srand( time( NULL ) );
  srand48((unsigned int) time(NULL));

  if (bLearning || !bLearning) {

    string exepath = getexepath();
    // exepath += "LinearSarsaLearner::initialize";
    exepath += loadWeightsFile;
    exepath += saveWeightsFile;
    auto h = hash<string>()(exepath); // hashing


      if (loadWeightsFile.empty() || !loadWeights(loadWeightsFile.c_str())) {

          createFile(saveWeightsFile.c_str());
        // fill(weights, weights + RL_MEMORY_SIZE, initialWeight);
          for ( int i = 0; i < RL_MEMORY_SIZE; i++ ) {
            double w = distribution(generator);
            //Log.log(std::to_string(w));
            weights[ i ] = w;
          }
          Log.log("Right before making the weightToString call");
          weightsToString();
          Log.log("Right after");
        colTab->reset();
      }
  }
  //weightsLog();
  //Log.log("Right before weight to String");
  //weightsToString();
}

void CrossEntropyAgent::weightsLog(){
  for(int i = 0; i < RL_MEMORY_SIZE; i++){
    Log.log(std::to_string(weights[i]));
  }
}

void CrossEntropyAgent::createFile(const char *filename){
  myFile.open(filename);
}

void CrossEntropyAgent::weightsToString(){
  Log.log("managed to get inside weightsToString");
  //myFile << "Hello World";
  myFile.write(reinterpret_cast<const char*>(&weights), std::streamsize(RL_MEMORY_SIZE*sizeof(double)));
}

// Q state-action value estimate.
double CrossEntropyAgent::getQ(int action) {
  if (action < 0 || action > getNumActions()) {
    throw "invalid action";
  }
  return Q[action];
}


// At the start of update Q and choose and action
int CrossEntropyAgent::startEpisode(double state[] )

{
    Log.log("start of Episode");
    epochNum++;
    loadTiles( state );
    for ( int a = 0; a < getNumActions(); a++ ) {
        Q[ a ] = computeQ( a );
    }

    lastAction = selectAction();

    char buffer[128];
    sprintf( buffer, "Q[%d] = %.2f", lastAction, Q[lastAction] );
    // LogDraw.logText( "Qmax", VecPosition( 25, -30 ),
    //                  buffer,
    //                  1, COLOR_BROWN );

    // saveWeights(weightsFile);
    return lastAction;
}



bool sortByVal(const pair<array<double, RL_MEMORY_SIZE>, double> &a, 
               const pair<array<double, RL_MEMORY_SIZE>, double> &b) 
{ 
    return (a.second < b.second); 
} 



void CrossEntropyAgent::updateWeights()

{

  /* select the best set of weights 
  
  */

    // create a empty vector of pairs

    std::vector<pair<array<double, RL_MEMORY_SIZE>, double>> vec;
    // copy key-value pairs from the map to the vector
    std::map<array<double, RL_MEMORY_SIZE>, double> :: iterator it;;

    for ( it=samples.begin(); it!=samples.end(); it++) {
        vec.push_back(make_pair(it->first, it->second));
    }
    // Sorts the weights by the reward
    std::sort(vec.begin(), vec.end(), sortByVal); 

    float sumWeights = 0;

    for (std::size_t  i = 0; i < vec.size(); i++){
      if ( i <= k-1) {
          for (std::size_t  j = 0; j < vec[i].first.size(); j++) {
              sumWeights += vec[i].first[j];                    
          }
      }
      else {
          break;
      }
    }
    // update the pointer of mean
    // should we divide by index instead of total selected samples??
    mean = sumWeights / k;

    //Log.log("CrossEntropyAgent::updateWeights mean %.2f", mean);

    // find the std of the weights and update std pointer.
    
    MatrixXd mu = MatrixXd::Constant(1,RL_MEMORY_SIZE,mean);
    // this can be combined with one above.
    float sum;
    float Q;

    for (std::size_t  i = 0; i < vec.size(); i++){
      if ( i <= k-1) {
        RowVectorXd W(RL_MEMORY_SIZE);
        VectorXd P;

          for (std::size_t  j = 0; j < vec[i].first.size(); j++) {
              W(0,j) = vec[0].first[j];      
          }
          P = W - mu;
          Q = P.transpose().dot(P);
          sum += Q;
      }
      else {
          break;
      }
    }
    std = sum / N;

    //Log.log("CrossEntropyAgent::updateWeights std %.2f", std);

    
}



int CrossEntropyAgent::step( double reward, double state[] )
{
 
  // Use either the discounted reward or culmilative reward
  // i have decided to use the culmulative reward

  double totalRewards = reward + Q[ lastAction ];

  loadTiles( state );
  for ( int a = 0; a < getNumActions(); a++ ) {
    Q[ a ] = computeQ( a );
  }

  lastAction = selectAction(); // take actions with the maximal weights

  // I dont know what these does completely yet seems like saving the logging informations. 
  char buffer[128];
  sprintf( buffer, "Q[%d] = %.2f", lastAction, Q[lastAction] );


  //Log.log("CrossEntropyAgent::step reward %.2f", lastAction, Q[lastAction]);

  // LogDraw.logText( "Qmax", VecPosition( 25, -30 ),
  //                  buffer,
  //                  1, COLOR_BROWN );
  //

  if ( !bLearning )
    return lastAction;

  //char buffer[128];
  sprintf( buffer, "reward: %.2f", reward ); 

  //Log.log("CrossEntropyAgent::step reward %.2f", reward);

  // LogDraw.logText( "reward", VecPosition( 25, 30 ),
  //                  buffer,
  //                  1, COLOR_NAVY );

}


// Add the reward and update Mean and Standard deviation
void CrossEntropyAgent::endEpisode(double reward )

{
  // Store weights and reward
  // std::array<double, RL_MEMORY_SIZE> weights;

  
  samples[weights] = reward;  //??

  //Log.log("CrossEntropyAgent::endEpisode weights %f", samples);

  // save reward and weights
  if (counter < N){
    counter++;
    Log.log(std::to_string(counter));
    //Log.log("CrossEntropyAgent::endEpisode counter %f", counter);
  }else{
    // update weights and reset both the counter and samples map.
    weightsToString();
    updateWeights();
    counter = 0;
    samples.clear();
  }
}


void CrossEntropyAgent::shutDown()
{
  // We usually only save weights at random intervals.
  // Always save at shutdown (if we are in saving mode).
  if ( bLearning && bSaveWeights ) {
    cout << "Saving weights at shutdown." << endl;
    //saveWeights(saveWeightsFile.c_str() );
  }
}





// This is a gready approach and do we want to use greedy 
// or Choose the argmax

int CrossEntropyAgent::selectAction()
{
  int action;
  // Epsilon-greedy
  if ( bLearning && drand48() < epsilon ) {     /* explore */
    action = rand() % getNumActions();
  }
  else{
    action = argmaxQ();
  }
  return action;
}


bool CrossEntropyAgent::loadWeights(const char *filename) {
  FileLock lock("loadWeights");
  cerr << "Loading weights from " << filename << endl;

#ifdef _Compress
  igzstream is;
#else
  ifstream is;
#endif

  is.open(filename);
  //myFile.open(filename)
  if (!is.good()) {
    cerr << "failed to open weight file: " << filename << endl;
    return false;
  }
  //myFile.read((char *) &weights, RL_MEMORY_SIZE * sizeof(double)))
  is.read(reinterpret_cast<char*>(&weights), std::streamsize(RL_MEMORY_SIZE * sizeof(double)));
    colTab->restore(is);
    is.close();
    cerr << "...done" << endl;
    return true;
}


bool CrossEntropyAgent::saveWeights(const char *filename) {
  FileLock lock("saveWeights");

#ifdef _Compress
  ogzstream os;
#else
  ofstream os;
#endif

  
  if (!os.good()) {
    
    cerr << "failed to open weight file: " << filename << endl;
    return false;
  }

  os.write((const char*)&weights, RL_MEMORY_SIZE * sizeof(double));
  colTab->save(os);
  os.close();
  return true;
}




// Compute an action value from current F and theta    
double CrossEntropyAgent::computeQ( int a )
{
  double q = 0;
  for ( int j = 0; j < numTilings; j++ ) {
    q += weights[ tiles[ a ][ j ] ];
  }

  return q;
}

// Returns index (action) of largest entry in Q array, breaking ties randomly 
int CrossEntropyAgent::argmaxQ()
{
  int bestAction = 0;
  
  double bestValue = Q[ bestAction ];
  int numTies = 0;
  for ( int a = bestAction + 1; a < getNumActions(); a++ ) {
    double value = Q[ a ];
    if ( value > bestValue ) {
      bestValue = value;
      bestAction = a;
    }
    else if ( value == bestValue ) {
      numTies++;
      if ( rand() % ( numTies + 1 ) == 0 ) {
        bestValue = value;
        bestAction = a;
      }
    }
  }

  return bestAction;
}


void CrossEntropyAgent::loadTiles( double state[] )
{
  int tilingsPerGroup = 32;  /* num tilings per tiling group */
  numTilings = 0;

  /* These are the 'tiling groups'  --  play here with representations */
  /* One tiling for each state variable */
  for ( int v = 0; v < getNumFeatures(); v++ ) {
    for ( int a = 0; a < getNumActions(); a++ ) {
      GetTiles1( &(tiles[ a ][ numTilings ]), tilingsPerGroup, colTab,
                 state[ v ] / tileWidths[ v ], a , v );
    }  
    numTilings += tilingsPerGroup;
  }
  if ( numTilings > RL_MAX_NUM_TILINGS )
    cerr << "TOO MANY TILINGS! " << numTilings << endl;
}



void CrossEntropyAgent::setParams(int iCutoffEpisodes, int iStopLearningEpisodes)
{
  /* set learning parameters */
}

