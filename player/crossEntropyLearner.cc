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
                                     string loadWeightsFile, string saveWeightsFile, bool hiveMind, WorldModel *wm) :

                                                                                                                      SMDPAgent(numFeatures, numActions)

{
  bLearning = bLearn;

  for (int i = 0; i < getNumFeatures(); i++)
  {
    tileWidths[i] = widths[i];
  }

  // Saving weights (including for hive mind) requires learning and a file name.
  this->hiveMind = false;
  if (bLearning && strlen(saveWeightsFile.c_str()) > 0)
  {
    strcpy(weightsFile, saveWeightsFile.c_str());
    bSaveWeights = true;
    // Hive mind further requires loading and saving from the same file.
    if (!strcmp(loadWeightsFile.c_str(), saveWeightsFile.c_str()))
    {
      this->hiveMind = hiveMind;
    }
  }
  else
  {
    bSaveWeights = false;
  }

  alpha = 0.125;
  gamma = 1.0;
  lambda = 0;
  epsilon = 0.01;
  minimumTrace = 0.01;
  epochNum = 0;
  lastAction = -1;
  numNonzeroTraces = 0;

  counter = 0;
  mean = 0.01;
  std = 0.1;
  popSize = 25;  // population size .
  eliteFrac = 3; // elite fraction to select from

  maxReward = 0;
  initialWeights = weightsRaw;
  tempReward = 0;

  lastActionsTime = UnknownTime; // definnes an unknown time.
  WM = WM;
  std::default_random_engine generator;
  std::normal_distribution<double> distribution(mean, std);

  for (int j = 0; j < 25; j++)
  {
    for (int i = 0; i < RL_MEMORY_SIZE; i++)
    {
      double w = distribution(generator);
      // Log.log(std::to_string(w));
      weights[i] = w;
      // initialWeights[i] = w;
    }
    // samples.insert(std::make_pair<std::array<double, 10>,double>(j,weights));
    setOfWeights[j] = weights;
    // setOfW.insert(std::make_pair<double,double*[10]>(j,weights));
    // std::cout << "mymap.size() is " << setOfWeights.size() << '\n';
    Log.log(std::to_string(setOfWeights.size()));
  }
  // std::cout << "mymap.size() is " << setOfWeights.size() << '\n';
  // }

  srand((unsigned int)0);
  int tmp[2];
  float tmpf[2];
  colTab = new collision_table(RL_MEMORY_SIZE, 1);

  GetTiles(tmp, 1, 1, tmpf, 0); // A dummy call to set the hashing table
  srand(time(NULL));

  if (strlen(loadWeightsFile.c_str()) > 0)
    loadWeights(loadWeightsFile.c_str());
}

// void CrossEntropyAgent::generateNewWeighs(double state[])
// {
//   for (int j = 0; j < 25; j++)
//   {
//     for (int i = 0; i < RL_MEMORY_SIZE; i++)
//     {
//       double w = distribution(generator);
//       // Log.log(std::to_string(w));
//       weights[i] = w;
//       // initialWeights[i] = w;
//     }
//     // samples.insert(std::make_pair<std::array<double, 10>,double>(j,weights));
//     setOfWeights[j] = weights;
//     // setOfW.insert(std::make_pair<double,double*[10]>(j,weights));
//     // std::cout << "mymap.size() is " << setOfWeights.size() << '\n';
//     Log.log(std::to_string(setOfWeights.size()));
//   }
// }

int CrossEntropyAgent::startEpisode(double state[])
{
  // copy the from the array map to array type

  tempWeights = setOfWeights[counter];
  for (int i = 0; i < RL_MEMORY_SIZE; i++)
  {
    initialWeights[i] = tempWeights[i];
  }

  // Check if array has the right values

  if (hiveMind)
    loadColTabHeader(colTab, initialWeights); // added the array type.

  loadTiles(state);
  for (int a = 0; a < getNumActions(); a++)
  {
    Q[a] = computeQ(a);
  }

  lastAction = selectAction();
  lastActionsTime = WM->getCurrentTime().getTime();
  // Log.log("Right before making the weightToString call", lastAction.c_str());
  char buffer[128];
  sprintf(buffer, "Q[%d] = %.2f", lastAction, Q[lastAction]);
  LogDraw.logText("Qmax", VecPosition(25, -30),
                  buffer,
                  1, COLOR_BROWN);
  if (hiveMind)
    saveWeights(weightsFile);
  return lastAction;
}

// Compute an action value from current F and theta
double CrossEntropyAgent::computeQ(int a)
{
  double q = 0;
  for (int j = 0; j < numTilings; j++)
  {
    q += initialWeights[tiles[a][j]]; // array type
  }

  return q;
}

double CrossEntropyAgent::reward(double tau)
{
  Log.log(101, "CrossEntropyAgent::reward tau=%f", tau);
  double ret = tau;

  if (gamma < 1.0)
  {
    ret = (1.0 - pow(gamma, tau)) / (1.0 - gamma);
  }
  // return teamName == "keepers" ? ret : -ret;
  return ret;
}
int CrossEntropyAgent::step(double current_time, double state[])
{

  // copy into array type
  tempWeights = setOfWeights[counter];
  for (int i = 0; i < RL_MEMORY_SIZE; i++)
  {
    initialWeights[i] = tempWeights[i];
  }
  if (hiveMind)
    loadColTabHeader(colTab, initialWeights);

  // Add the difference in time here.
  double reward = current_time - lastActionsTime;
  double delta = reward - Q[lastAction];
  // rewards[counter] += reward;
  rewardList[counter] += reward;

  // double discountReturn = discountReturn + (tau * pow())
  loadTiles(state);
  for (int a = 0; a < getNumActions(); a++)
  {
    Q[a] = computeQ(a);
  }

  lastAction = selectAction();
  lastActionsTime = WM->getCurrentTime().getTime();

  char buffer[128];
  sprintf(buffer, "Q[%d] = %.2f", lastAction, Q[lastAction]);
  LogDraw.logText("Qmax", VecPosition(25, -30),
                  buffer,
                  1, COLOR_BROWN);

  if (!bLearning)
    return lastAction;

  // char buffer[128];
  sprintf(buffer, "reward: %.2f", reward);
  LogDraw.logText("reward", VecPosition(25, 30),
                  buffer,
                  1, COLOR_NAVY);

  delta += Q[lastAction];
  // updateWeights( delta );

  Q[lastAction] = computeQ(lastAction); // need to redo because weights changed
  decayTraces(gamma * lambda);

  for (int a = 0; a < getNumActions(); a++)
  { // clear other than F[a]
    if (a != lastAction)
    {
      for (int j = 0; j < numTilings; j++)
        clearTrace(tiles[a][j]);
    }
  }
  for (int j = 0; j < numTilings; j++) // replace/set traces F[a]
    setTrace(tiles[lastAction][j], 1.0);

  if (hiveMind)
    saveWeights(weightsFile);
  return lastAction;
}

void CrossEntropyAgent::endEpisode(double reward)

{
  if (counter < popSize)
  {
    counter++;
  }
  else
  {

    // argsort the  rewards
    for (unsigned i = 25; i > 24; i--)
    {
      tempWeights = setOfWeights[argsort(rewardList).at(i)];

      float sumWeights = 0;
      for (std::size_t j = 0; j < RL_MEMORY_SIZE; j++)
      {
        sumWeights += tempWeights[j];
      }
      mean += sumWeights / RL_MEMORY_SIZE;
      // Log.log("New Mean: " + std::to_string(mean));
      Eigen::RowVectorXd b = Eigen::Map<Eigen::RowVectorXd, Eigen::Unaligned>(tempWeights.data(), tempWeights.size());
      RowVectorXd mu = RowVectorXd ::Constant(1, RL_MEMORY_SIZE, mean);
      float sum;
      float Q;
      VectorXd P;
      P = b - mu;
      // std::cout << "Here is the matrix m:\n" <<  P  << std::endl;
      Q = P.transpose().dot(P);
      std += Q / RL_MEMORY_SIZE;
    }
    // find mean
    // generate new samples.

    // oneUpdate();
    // setOfWeights[1];
    counter = 0;
    setOfWeights.clear();
  }
}


// This is a gready approach and do we want to use greedy
// or Choose the argmax

int CrossEntropyAgent::selectAction()
{
  int action;
  // Epsilon-greedy
  if (bLearning && drand48() < epsilon)
  { /* explore */
    action = rand() % getNumActions();
  }
  else
  {
    action = argmaxQ();
  }
  return action;
}


void CrossEntropyAgent::oneUpdate()
{
  // loadWeights(saveWeightsFile.c_str());
  tempWeights = setOfWeights[maxReward];
  // std::cout << "Here is the matrix m:\n" <<  b  << std::endl;
  // Log.log("First number of highest reward: " + std::to_string(tempWeights[0]));
  // Log.log("First number of current weights: " + std::to_string(weights[0]));
  float sumWeights = 0;
  for (std::size_t j = 0; j < RL_MEMORY_SIZE; j++)
  {
    sumWeights += tempWeights[j];
  }
  mean += sumWeights / RL_MEMORY_SIZE;
  // Log.log("New Mean: " + std::to_string(mean));
  Eigen::RowVectorXd b = Eigen::Map<Eigen::RowVectorXd, Eigen::Unaligned>(tempWeights.data(), tempWeights.size());
  RowVectorXd mu = RowVectorXd ::Constant(1, RL_MEMORY_SIZE, mean);
  float sum;
  float Q;
  VectorXd P;
  P = b - mu;
  // std::cout << "Here is the matrix m:\n" <<  P  << std::endl;
  Q = P.transpose().dot(P);
  std += Q / RL_MEMORY_SIZE;
  // Log.log("New std: " + std::to_string(std));
}

// Clear any trace for feature f
void CrossEntropyAgent::clearTrace(int f)
{
  if (f > RL_MEMORY_SIZE || f < 0)
    cerr << "ClearTrace: f out of range " << f << endl;
  if (traces[f] != 0)
    clearExistentTrace(f, nonzeroTracesInverse[f]);
}

// Clear the trace for feature f at location loc in the list of nonzero traces
void CrossEntropyAgent::clearExistentTrace(int f, int loc)
{
  if (f > RL_MEMORY_SIZE || f < 0)
    cerr << "ClearExistentTrace: f out of range " << f << endl;
  traces[f] = 0.0;
  numNonzeroTraces--;
  nonzeroTraces[loc] = nonzeroTraces[numNonzeroTraces];
  nonzeroTracesInverse[nonzeroTraces[loc]] = loc;
}

// Decays all the (nonzero) traces by decay_rate, removing those below minimum_trace
void CrossEntropyAgent::decayTraces(double decayRate)
{
  int f;
  for (int loc = numNonzeroTraces - 1; loc >= 0; loc--)
  {
    f = nonzeroTraces[loc];
    if (f > RL_MEMORY_SIZE || f < 0)
      cerr << "DecayTraces: f out of range " << f << endl;
    traces[f] *= decayRate;
    if (traces[f] < minimumTrace)
      clearExistentTrace(f, loc);
  }
}

// Set the trace for feature f to the given value, which must be positive
void CrossEntropyAgent::setTrace(int f, float newTraceValue)
{
  if (f > RL_MEMORY_SIZE || f < 0)
    cerr << "SetTraces: f out of range " << f << endl;
  if (traces[f] >= minimumTrace)
    traces[f] = newTraceValue; // trace already exists
  else
  {
    while (numNonzeroTraces >= RL_MAX_NONZERO_TRACES)
      increaseMinTrace(); // ensure room for new trace
    traces[f] = newTraceValue;
    nonzeroTraces[numNonzeroTraces] = f;
    nonzeroTracesInverse[f] = numNonzeroTraces;
    numNonzeroTraces++;
  }
}

// Try to make room for more traces by incrementing minimum_trace by 10%,
// culling any traces that fall below the new minimum
void CrossEntropyAgent::increaseMinTrace()
{
  minimumTrace *= 1.1;
  cerr << "Changing minimum_trace to " << minimumTrace << endl;
  for (int loc = numNonzeroTraces - 1; loc >= 0; loc--)
  { // necessary to loop downwards
    int f = nonzeroTraces[loc];
    if (traces[f] < minimumTrace)
      clearExistentTrace(f, loc);
  }
}


void CrossEntropyAgent::shutDown()
{
  // We usually only save weights at random intervals.
  // Always save at shutdown (if we are in saving mode).
  if (bLearning && bSaveWeights)
  {
    cout << "Saving weights at shutdown." << endl;
    saveWeights(saveWeightsFile.c_str());
  }
}


bool CrossEntropyAgent::loadWeights(const char *filename)
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
      initialWeights = reinterpret_cast<double*>(hiveMap);
      // Now the collision table header.
      CollisionTableHeader* colTabHeader =
        reinterpret_cast<CollisionTableHeader*>(initialWeights + RL_MEMORY_SIZE);
      if (fileFound) {
        loadColTabHeader(colTab, initialWeights);
      }
      // Now the collision table data.
      delete[] colTab->data;
      colTab->data = reinterpret_cast<long*>(colTabHeader + 1);
      if (!fileFound) {
        // Clear out initial contents.
        // The whole team might be doing this at the same time. Is that okay?
        for ( int i = 0; i < RL_MEMORY_SIZE; i++ ) {
          initialWeights[ i ] = 0;
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
    read( file, (char *) initialWeights, RL_MEMORY_SIZE * sizeof(double) );
    colTab->restore( file );
    close( file );
  }
  cout << "...done" << endl;
  return true;
}


bool  CrossEntropyAgent::saveWeights(const char *filename)
{
  if (hiveMind) {
    // The big arrays should be saved out automatically, but we still need to
    // handle the collision table header.
    CollisionTableHeader* colTabHeader =
      reinterpret_cast<CollisionTableHeader*>(initialWeights + RL_MEMORY_SIZE);
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
    write( file, (char *) initialWeights, RL_MEMORY_SIZE * sizeof(double) );
    colTab->save( file );
    close( file );
  }
  return true;
}




// Returns index (action) of largest entry in Q array, breaking ties randomly
int CrossEntropyAgent::argmaxQ()
{
  int bestAction = 0;

  double bestValue = Q[bestAction];
  int numTies = 0;
  for (int a = bestAction + 1; a < getNumActions(); a++)
  {
    double value = Q[a];
    if (value > bestValue)
    {
      bestValue = value;
      bestAction = a;
    }
    else if (value == bestValue)
    {
      numTies++;
      if (rand() % (numTies + 1) == 0)
      {
        bestValue = value;
        bestAction = a;
      }
    }
  }

  return bestAction;
}

void CrossEntropyAgent::loadTiles(double state[])
{
  int tilingsPerGroup = 32; /* num tilings per tiling group */
  numTilings = 0;

  /* These are the 'tiling groups'  --  play here with representations */
  /* One tiling for each state variable */
  for (int v = 0; v < getNumFeatures(); v++)
  {
    for (int a = 0; a < getNumActions(); a++)
    {
      GetTiles1(&(tiles[a][numTilings]), tilingsPerGroup, colTab,
                state[v] / tileWidths[v], a, v);
    }
    numTilings += tilingsPerGroup;
  }
  if (numTilings > RL_MAX_NUM_TILINGS)
    cerr << "TOO MANY TILINGS! " << numTilings << endl;
}

void CrossEntropyAgent::setParams(int iCutoffEpisodes, int iStopLearningEpisodes)
{
  /* set learning parameters */
}

void CrossEntropyAgent::updateweightsEndEpisode()
{
  std::default_random_engine generator(counter + 1);
  std::normal_distribution<double> distribution(mean, std);
  for (int i = 0; i < RL_MEMORY_SIZE; i++)
  {
    double w = distribution(generator);
    // Log.log(std::to_string(w));
    weights[i] = w;
  }
}



