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
// If all is well, there should be no mention of anything keepaway- or soccer- related in this file.

/**
 * Designed specifically to match the serialization format for collision_table.
 * See collision_table::save and collision_table::restore.
 */

#pragma pack(push, 1)
struct CollisionTableHeader
{
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

long *loadColTabHeader(collision_table *colTab, double *weights)
{
  CollisionTableHeader *colTabHeader =
      reinterpret_cast<CollisionTableHeader *>(weights + RL_MEMORY_SIZE);

  // Do each field individually, since they don't all line up exactly for an
  // easy copy.
  colTab->calls = colTabHeader->calls;
  colTab->clearhits = colTabHeader->clearhits;
  colTab->collisions = colTabHeader->collisions;
  colTab->m = colTabHeader->m;
  colTab->safe = colTabHeader->safe;
  if (VERBOSE_HIVE_MIND)
  {
    std::cout << "Loaded colTabHeader:" << endl
              << " calls: " << colTab->calls << endl
              << " clearhits: " << colTab->clearhits << endl
              << " collisions: " << colTab->collisions << endl
              << " m: " << colTab->m << endl
              << " safe: " << colTab->safe << endl;
  }
  return reinterpret_cast<long *>(colTabHeader + 1);
}

extern LoggerDraw LogDraw;

CrossEntropyAgent::CrossEntropyAgent(int numFeatures, int numActions, bool bLearn,
                                     double widths[],
                                     string loadWeightsFile, string saveWeightsFile, bool hiveMind,):

SMDPAgent(numFeatures, numActions)

{
    bLearning = bLearn;

    for ( int i = 0; i < getNumFeatures(); i++ ) {
        tileWidths[ i ] = widths[ i ];
    }

    // Saving weights (including for hive mind) requires learning and a file name.
    this->hiveMind = false;
    if ( bLearning && strlen( saveWeightsFile.c_str() ) > 0 ) {
        strcpy( weightsFile, saveWeightsFile.c_str() );
        bSaveWeights = true;
        // Hive mind further requires loading and saving from the same file.
        if (!strcmp(loadWeightsFile.c_str(), saveWeightsFile.c_str())) {
        this->hiveMind = hiveMind;
        }
    }
    else {
        bSaveWeights = false;
    }

    // alpha = 0.125;
    // gamma = 1.0;
    // lambda = 0;
    // epsilon = 0.01;
    // minimumTrace = 0.01;
    // epochNum = 0;
    // lastAction = -1;
    // numNonzeroTraces = 0;

    mean = 0.01;
    std = 0.1;
    popSize = 25;   // population size .
    eliteFrac = 3;  // elite fraction to select from
  
    maxReward = 0;
    initialWeights = weightsRaw;
    tempReward = 0;

    lastActionsTime = UnknownTime;  // definnes an unknown time.

    WM = WM;

    std::default_random_engine generator;
    std::normal_distribution<double> distribution(mean, std);
    // weights = weightsRaw;

    for ( int i = 0; i < RL_MEMORY_SIZE; i++ ) {
        double w = distribution(generator);
        weights[i] = w;                                                   // template type 
        initialWeights[i] = w;                                           //  array type
    }

    srand( (unsigned int) 0 );
    int tmp[ 2 ];
    float tmpf[ 2 ];
    colTab = new collision_table( RL_MEMORY_SIZE, 1 );

    GetTiles( tmp, 1, 1, tmpf, 0 );  // A dummy call to set the hashing table    
    srand( time( NULL ) );

    if ( strlen( loadWeightsFile.c_str() ) > 0 )
        loadWeights(loadWeightsFile.c_str());
}


int CrossEntropyAgent::startEpisode(double state[] )
{
  if (hiveMind) loadColTabHeader(colTab, initialWeights);               // added the array type.

  loadTiles( state );
  for ( int a = 0; a < getNumActions(); a++ ) {
    Q[ a ] = computeQ( a );
  }

  lastAction = selectAction();
  // lastActionsTime = WM->getCurrentTime();

  // Log.log("Right before making the weightToString call", lastAction.c_str());
  char buffer[128];
  sprintf( buffer, "Q[%d] = %.2f", lastAction, Q[lastAction] );
  LogDraw.logText( "Qmax", VecPosition( 25, -30 ),
                   buffer,
                   1, COLOR_BROWN );
  if (hiveMind) saveWeights(weightsFile);
  return lastAction;

}


// Compute an action value from current F and theta    
double CrossEntropyAgent::computeQ( int a )
{
  double q = 0;
  for ( int j = 0; j < numTilings; j++ ) {
    q += initialWeights[ tiles[ a ][ j ] ];                   // array type
  }

  return q;
}


int CrossEntropyAgent::step( double reward, double state[] )
{
  if (hiveMind) loadColTabHeader(colTab, initialWeights);
  
// Add the difference in time here.
  
//   double delta = reward - Q[ lastAction ];
  
//   loadTiles( state );
//   for ( int a = 0; a < getNumActions(); a++ ) {
//     Q[ a ] = computeQ( a );
//   }

//   lastAction = selectAction();

//   char buffer[128];
//   sprintf( buffer, "Q[%d] = %.2f", lastAction, Q[lastAction] );
//   LogDraw.logText( "Qmax", VecPosition( 25, -30 ),
//                    buffer,
//                    1, COLOR_BROWN );

//   if ( !bLearning )
//     return lastAction;

//   //char buffer[128];
//   sprintf( buffer, "reward: %.2f", reward ); 
//   LogDraw.logText( "reward", VecPosition( 25, 30 ),
//                    buffer,
//                    1, COLOR_NAVY );

//   delta += Q[ lastAction ];
//   updateWeights( delta );
//   Q[ lastAction ] = computeQ( lastAction ); // need to redo because weights changed
//   decayTraces( gamma * lambda );

//   for ( int a = 0; a < getNumActions(); a++ ) {  //clear other than F[a]
//     if ( a != lastAction ) {
//       for ( int j = 0; j < numTilings; j++ )
//         clearTrace( tiles[ a ][ j ] );
//     }
//   }
//   for ( int j = 0; j < numTilings; j++ )      //replace/set traces F[a]
//     setTrace( tiles[ lastAction ][ j ], 1.0 );

//   if (hiveMind) saveWeights(weightsFile);
//   return lastAction;
}



void CrossEntropyAgent::endEpisode(double reward)

{

//   if(rand()%200 == 0){
//     saveWeights(saveWeightsFile.c_str());
//   }


if (counter < popSize)

  {
    // setOfWeights[reward] = initialWeights;
    setOfWeights[reward] = weights;
    // S[reward] = weightsRaw;
    // setOfW.insert(std::pair<double,double*[RL_MEMORY_SIZE]>(reward,weights));
    maxReward = std::max(maxReward, reward);
    //Log.log(std::to_string(reward));
    counter++;
    // Log.log(std::to_string(counter));
    updateweightsEndEpisode();
  }
  else
  {
    oneUpdate();
    counter = 0;
    setOfWeights.clear();
    // samples.clear();
  }
}






