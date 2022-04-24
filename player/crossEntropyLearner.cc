#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <sstream>
#include <random>
#include "CrossEntropyAgent.h"
#include "LoggerDraw.h"

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

/**
 * Assumes collision table header follows weights.
 * Returns the memory location after the header because that's useful for colTab
 * data array.
 */
long* loadColTabHeader(collision_table* colTab, double* weights) {
  CollisionTableHeader* colTabHeader =
    reinterpret_cast<CollisionTableHeader*>(weights + RL_MEMORY_SIZE);
  // Do each field individually, since they don't all line up exactly for an
  // easy copy.
  colTab->calls = colTabHeader->calls;
  colTab->clearhits = colTabHeader->clearhits;
  colTab->collisions = colTabHeader->collisions;
  colTab->m = colTabHeader->m;
  colTab->safe = colTabHeader->safe;
  if (VERBOSE_HIVE_MIND) {
    cout << "Loaded colTabHeader:" << endl
      << " calls: " << colTab->calls << endl
      << " clearhits: " << colTab->clearhits << endl
      << " collisions: " << colTab->collisions << endl
      << " m: " << colTab->m << endl
      << " safe: " << colTab->safe << endl;
  }
  return reinterpret_cast<long*>(colTabHeader + 1);
}

extern LoggerDraw LogDraw;

CrossEntropyAgent::CrossEntropyAgent( int numFeatures, int numActions, bool bLearn,
                                    double widths[],
                                    char *loadWeightsFile, char *saveWeightsFile, bool hiveMind ):
  SMDPAgent( numFeatures, numActions ), hiveFile(-1)

{
  bLearning = bLearn;

  for ( int i = 0; i < getNumFeatures(); i++ ) {
    tileWidths[ i ] = widths[ i ];
  }

  // Saving weights (including for hive mind) requires learning and a file name.
  this->hiveMind = false;
  if ( bLearning && strlen( saveWeightsFile ) > 0 ) {
    strcpy( weightsFile, saveWeightsFile );
    bSaveWeights = true;
    // Hive mind further requires loading and saving from the same file.
    if (!strcmp(loadWeightsFile, saveWeightsFile)) {
      this->hiveMind = hiveMind;
    }
  }
  else {
    bSaveWeights = false;
  }



  // number of iterations for updating weights
  mean = 0.0;
  std = 100.0;
  N = 25;
  std::default_random_engine generator;
  std::normal_distribution<double> distribution(mean,std);

  int *counter = 0;

  epochNum = 0;
  lastAction = -1;

  weights = weightsRaw;

  // store initial weights by sampling from a distrbution
  for ( int i = 0; i < RL_MEMORY_SIZE; i++ ) {
    double w = distribution(generator);
    weights[ i ] = w;

  }

  // Create an hash map to store weights and current reward.
  // initialized map 
  // incomplete .

  std::map< array<float, RL_MEMORY_SIZE>, float> samples;



  srand( (unsigned int) 0 );
  int tmp[ 2 ];
  float tmpf[ 2 ];
  colTab = new collision_table( RL_MEMORY_SIZE, 1 );

  GetTiles( tmp, 1, 1, tmpf, 0 );  // A dummy call to set the hashing table    
  srand( time( NULL ) );

  if ( strlen( loadWeightsFile ) > 0 )
    loadWeights( loadWeightsFile );
}




// Q state-action value estimate 
double CrossEntropyAgent::getQ(int action) {
  if (action < 0 || action > getNumActions()) {
    throw "invalid action";
  }
  return Q[action];
}



// At the start of update Q and choose and action
int CrossEntropyAgent::startEpisode( double state[] )
{
    epochNum++;
    loadTiles( state );
    for ( int a = 0; a < getNumActions(); a++ ) {
        Q[ a ] = computeQ( a );
    }

    lastAction = selectAction();

    char buffer[128];
    sprintf( buffer, "Q[%d] = %.2f", lastAction, Q[lastAction] );
    LogDraw.logText( "Qmax", VecPosition( 25, -30 ),
                     buffer,
                     1, COLOR_BROWN );

    if (hiveMind) saveWeights(weightsFile);
    return lastAction;
}




// Cross Entropy 
int CrossEntropyAgent::step( double reward, double state[] )
{
  // at each t
  // generate best sets of n weights from distrbution 
  // find the best sets of values with the corresponding weights
  // sort these weights
  // update mean 
  // update std. 


  if (hiveMind) loadColTabHeader(colTab, weights);

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
  LogDraw.logText( "Qmax", VecPosition( 25, -30 ),
                   buffer,
                   1, COLOR_BROWN );
  //
  
  


}


// Add the reward and update Mean and Standard deviation
void CrossEntropyAgent::endEpisode( double reward )

{
  // Store weights and reward

  samples[weights] = reward;

  // save reward and weights
  if (*counter < N){
    counter++;
  }else{

    /// update weights and reset both the counter and samples map.
    updateWeights()
    *counter = 0;
    samples.clear();
  }
  
  





}


void CrossEntropyAgent::shutDown()
{
  // We usually only save weights at random intervals.
  // Always save at shutdown (if we are in saving mode).
  if ( bLearning && bSaveWeights ) {
    cout << "Saving weights at shutdown." << endl;
    saveWeights( weightsFile );
  }
  // Also shut down the hive mind if needed.
  if (hiveMind) {
    size_t mapLength =
      RL_MEMORY_SIZE * sizeof(double) +
      sizeof(CollisionTableHeader) +
      colTab->m * sizeof(long);
    munmap(weights, mapLength);
    close(hiveFile);
    hiveFile = -1;
    
    // Go back to the own arrays, since our map is no longer valid.
    weights = weightsRaw;
    colTab->data = new long[colTab->m];
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


// No changes have been made 

bool CrossEntropyAgent::loadWeights( char *filename )
{
  cout << "Loading weights from " << filename << endl;
  if (hiveMind) {
    if (hiveFile < 0) {
      // First, check the lock file, so we have only one initializer.
      // Later interaction should be approximately synchronized by having only
      // one active player at a time per team, but we can't assume that here.
      stringstream lockNameBuffer;
      lockNameBuffer << filename << ".lock";
      const char* lockName = lockNameBuffer.str().c_str();
      int lock;
      // 10ms delay (times a million to convert from nanos).
      timespec sleepTime = {0, 10 * 1000 * 1000};
      while (true) {
        lock = open(lockName, O_CREAT | O_EXCL, 0664);
        if (lock >= 0) break;
        nanosleep(&sleepTime, NULL);
      }
      // First, see if the file is already there.
      bool fileFound = !access(filename, F_OK);
      // TODO Extract constant for permissions (0664)?
      hiveFile = open(filename, O_RDWR | O_CREAT, 0664);
      size_t mapLength =
        RL_MEMORY_SIZE * sizeof(double) +
        sizeof(CollisionTableHeader) +
        colTab->m * sizeof(long);
      if (!fileFound) {
        // Make the file the right size.
        cout << "Initializing new hive file." << endl;
        if (lseek(hiveFile, mapLength - 1, SEEK_SET) < 0) {
          throw "failed to seek initial file size";
        }
        if (write(hiveFile, "", 1) < 0) {
          throw "failed to expand initial file";
        }
      }
      if (hiveFile < 0) throw "failed to open hive file";
      void* hiveMap =
        mmap(NULL, mapLength, PROT_READ | PROT_WRITE, MAP_SHARED, hiveFile, 0);
      if (hiveMap == MAP_FAILED) throw "failed to map hive file";
      // First the weights.
      weights = reinterpret_cast<double*>(hiveMap);
      // Now the collision table header.
      CollisionTableHeader* colTabHeader =
        reinterpret_cast<CollisionTableHeader*>(weights + RL_MEMORY_SIZE);
      if (fileFound) {
        loadColTabHeader(colTab, weights);
      }
      // Now the collision table data.
      delete[] colTab->data;
      colTab->data = reinterpret_cast<long*>(colTabHeader + 1);
      if (!fileFound) {
        // Clear out initial contents.
        // The whole team might be doing this at the same time. Is that okay?
        for ( int i = 0; i < RL_MEMORY_SIZE; i++ ) {
          weights[ i ] = 0;
        }
        colTab->reset();
        // Make sure the header goes out to the file.
        saveWeights(weightsFile);
      }
      // TODO Separate file lock type with destructor?
      unlink(lockName);
    }
  } else {
    int file = open( filename, O_RDONLY );
    read( file, (char *) weights, RL_MEMORY_SIZE * sizeof(double) );
    colTab->restore( file );
    close( file );
  }
  cout << "...done" << endl;
  return true;
}

//No code changes made here 

bool CrossEntropyAgent::saveWeights( char *filename )
{
  if (hiveMind) {
    // The big arrays should be saved out automatically, but we still need to
    // handle the collision table header.
    CollisionTableHeader* colTabHeader =
      reinterpret_cast<CollisionTableHeader*>(weights + RL_MEMORY_SIZE);
    // Do each field individually, since they don't all line up exactly for an
    // easy copy.
    colTabHeader->calls = colTab->calls;
    colTabHeader->clearhits = colTab->clearhits;
    colTabHeader->collisions = colTab->collisions;
    colTabHeader->m = colTab->m;
    colTabHeader->safe = colTab->safe;
    if (VERBOSE_HIVE_MIND) {
      cout << "Saved colTabHeader:" << endl
        << " calls: " << colTab->calls << endl
        << " clearhits: " << colTab->clearhits << endl
        << " collisions: " << colTab->collisions << endl
        << " m: " << colTab->m << endl
        << " safe: " << colTab->safe << endl;
    }
  } else {
    int file = open( filename, O_CREAT | O_WRONLY, 0664 );
    write( file, (char *) weights, RL_MEMORY_SIZE * sizeof(double) );
    colTab->save( file );
    close( file );
  }
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


// This is change to mean / Standard deviation helper function 
bool CrossEntropyAgent::sortByVal(const pair<array, double> &a, 
               const pair<array, double> &b) 
{ 
    return (a.second < b.second); 
} 



void CrossEntropyAgent::updateWeights( double delta )

{
  // double tmp = delta * alpha / numTilings;
  // for ( int i = 0; i < numNonzeroTraces; i++ ) {
  //   int f = nonzeroTraces[ i ];
  //   if ( f > RL_MEMORY_SIZE || f < 0 )
  //     cerr << "f is too big or too small!!" << f << endl;
  //   weights[ f ] += tmp * traces[ f ];
  //   //cout << "weights[" << f << "] = " << weights[f] << endl;
  // }

  // select the best set of weights



  // find the mean for these weights and update mean pointer



  // find the std of the weights and update std pointer


  
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







// Clear any trace for feature f      
// void CrossEntropyAgent::clearTrace( int f)
// {
//   if ( f > RL_MEMORY_SIZE || f < 0 )
//     cerr << "ClearTrace: f out of range " << f << endl;
//   if ( traces[ f ] != 0 )
//     clearExistentTrace( f, nonzeroTracesInverse[ f ] );
// }

// Clear the trace for feature f at location loc in the list of nonzero traces 
// void CrossEntropyAgent::clearExistentTrace( int f, int loc )
// {
//   if ( f > RL_MEMORY_SIZE || f < 0 )
//     cerr << "ClearExistentTrace: f out of range " << f << endl;
//   traces[ f ] = 0.0;
//   numNonzeroTraces--;
//   nonzeroTraces[ loc ] = nonzeroTraces[ numNonzeroTraces ];
//   nonzeroTracesInverse[ nonzeroTraces[ loc ] ] = loc;
// }

// Decays all the (nonzero) traces by decay_rate, removing those below minimum_trace 
// void CrossEntropyAgent::decayTraces( double decayRate )
// {
//   int f;
//   for ( int loc = numNonzeroTraces - 1; loc >= 0; loc-- ) {
//     f = nonzeroTraces[ loc ];
//     if ( f > RL_MEMORY_SIZE || f < 0 )
//       cerr << "DecayTraces: f out of range " << f << endl;
//     traces[ f ] *= decayRate;
//     if ( traces[ f ] < minimumTrace )
//       clearExistentTrace( f, loc );
//   }
// }

// Set the trace for feature f to the given value, which must be positive   
// void CrossEntropyAgent::setTrace( int f, float newTraceValue )
// {
//   if ( f > RL_MEMORY_SIZE || f < 0 )
//     cerr << "SetTraces: f out of range " << f << endl;
//   if ( traces[ f ] >= minimumTrace )
//     traces[ f ] = newTraceValue;         // trace already exists              
//   else {
//     while ( numNonzeroTraces >= RL_MAX_NONZERO_TRACES )
//       increaseMinTrace(); // ensure room for new trace              
//     traces[ f ] = newTraceValue;
//     nonzeroTraces[ numNonzeroTraces ] = f;
//     nonzeroTracesInverse[ f ] = numNonzeroTraces;
//     numNonzeroTraces++;
//   }
// }

// Try to make room for more traces by incrementing minimum_trace by 10%,
// culling any traces that fall below the new minimum                      
// void CrossEntropyAgent::increaseMinTrace()
// {
//   minimumTrace *= 1.1;
//   cerr << "Changing minimum_trace to " << minimumTrace << endl;
//   for ( int loc = numNonzeroTraces - 1; loc >= 0; loc-- ) { // necessary to loop downwards    
//     int f = nonzeroTraces[ loc ];
//     if ( traces[ f ] < minimumTrace )
//       clearExistentTrace( f, loc );
//   }
// }

void CrossEntropyAgent::setParams(int iCutoffEpisodes, int iStopLearningEpisodes)
{
  /* set learning parameters */
}

