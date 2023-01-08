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
#include "neuralnetwork.h"

using namespace Eigen;

// If all is well, there should be no mention of anything keepaway- or soccer-
// related in this file.

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
    cout << "Loaded colTabHeader:" << endl
         << " calls: " << colTab->calls << endl
         << " clearhits: " << colTab->clearhits << endl
         << " collisions: " << colTab->collisions << endl
         << " m: " << colTab->m << endl
         << " safe: " << colTab->safe << endl;
  }
  return reinterpret_cast<long *>(colTabHeader + 1);
}


void initialize_weights(torch::nn::Module& module) {
    torch::NoGradGuard no_grad;

    if (auto* linear = module.as<torch::nn::Linear>()) {
      linear->weight.normal_(mean, dev);
    }
  }

extern LoggerDraw LogDraw;

CrossEntropyAgent::CrossEntropyAgent(int numFeatures, int numActions, bool bLearn,
                                     double widths[],
                                     string loadWeightsFile, string saveWeightsFile , bool hiveMind, WorldModel *wm, TwoLayerNet *m):

SMDPAgent(numFeatures, numActions),hiveFile(-1)

{
  WM = wm;
  bLearning = bLearn;
  bSaveWeights = bLearning && saveWeightsFile.length() > 0;
  saveWeightsFile = saveWeightsFile;
  
  // teamName = teamName_;

  for (int i = 0; i < getNumFeatures(); i++)
  {
    tileWidths[i] = widths[i];
  }

  // number of iterations for updating weights
  mean = 0.0;
  std = 0.02;
  N = 25;
  k = 1; // k best weights
  maxReward = 0;

  initialWeights = weightsRaw;

  tempReward = 0;

  std::default_random_engine generator;
  std::normal_distribution<double> distribution(mean, std);

  lastActionsTime = WM->getCurrentTime().getTime();

  counter = 0;
  epsilon = 0.01;

  epochNum = 0;
  lastAction = -1;

  srand((unsigned int)0);
  srand48((unsigned int)0);
  int tmp[2];
  float tmpf[2];

  srand(time(NULL));
  srand48((unsigned int)time(NULL));
  Log.log("Currently I am in the constructor");

  Log.log("Time " + std::to_string(WM->getCurrentTime().getTime()));

  // TwoLayerNet model;
  M = m;
  // M->apply(initialize_weights);

  torch::Device device(torch::kCPU);

  // Model->to(device);

}

void CrossEntropyAgent::weightsLog()
{
  for (int i = 0; i < RL_MEMORY_SIZE; i++)
  {
    Log.log(std::to_string(weights[i]));
  }
}

void CrossEntropyAgent::createFile(const char *filename)
{
  myFile.open(filename);
}

void CrossEntropyAgent::weightsToString(const char *filename)
{
  // have to open it again so that it overwrites whatever is inside, will be useful if writing more than once into the file per training round
  // myFile.open(filename);
  myFile.write(reinterpret_cast<const char *>(&weights), std::streamsize(RL_MEMORY_SIZE * sizeof(double)));
}

// Q state-action value estimate.
double CrossEntropyAgent::getQ(int action)
{
  if (action < 0 || action > getNumActions())
  {
    throw "invalid action";
  }
  return Q[action];
}

// At the start of update Q and choose and action
int CrossEntropyAgent::startEpisode(double state[])

{

  Log.log("start of Episode");

  std::cout << "start of Episode" << std::endl;

  lastActionTime = WM->getCurrentTime().getTime();

  std::cout << "start of Episode " << WM->getCurrentTime().getTime() << std::endl;

  
  epochNum++;
  loadTiles(state);

  for (int a = 0; a < getNumActions(); a++)
  {
    Q[a] = computeQ(a);
  }

  lastAction = selectAction();

  char buffer[128];
  sprintf(buffer, "Q[%d] = %.2f", lastAction, Q[lastAction]);

  // LogDraw.logText( "Qmax", VecPosition( 25, -30 ),
  //                  buffer,
  //                  1, COLOR_BROWN );

  // saveWeights(weightsFile);

  return lastAction;
}


void CrossEntropyAgent::updateWeights()

{

  /* select the best set of weights

  */

  // create a empty vector of pairs

  std::vector<pair<array<double, RL_MEMORY_SIZE>, double>> vec;
  // copy key-value pairs from the map to the vector
  std::map<array<double, RL_MEMORY_SIZE>, double>::iterator it;
  ;

  for (it = samples.begin(); it != samples.end(); it++)
  {
    vec.push_back(make_pair(it->first, it->second));
  }
  // Sorts the weights by the reward


  float sumWeights = 0;

  for (std::size_t i = 0; i < vec.size(); i++)
  {
    if (i <= k - 1)
    {
      for (std::size_t j = 0; j < vec[i].first.size(); j++)
      {
        sumWeights += vec[i].first[j];
      }
    }
    else
    {
      break;
    }
  }

  // update the pointer of mean
  // should we divide by index instead of total selected samples??
  mean = sumWeights / k;

  // Log.log("CrossEntropyAgent::updateWeights mean %.2f", mean);

  // find the std of the weights and update std pointer.

  MatrixXd mu = MatrixXd::Constant(1, RL_MEMORY_SIZE, mean);
  // this can be combined with one above.
  float sum;
  float Q;

  for (std::size_t i = 0; i < vec.size(); i++)
  {
    if (i <= k - 1)
    {
      RowVectorXd W(RL_MEMORY_SIZE);
      VectorXd P;

      for (std::size_t j = 0; j < vec[i].first.size(); j++)
      {
        W(0, j) = vec[0].first[j];
      }
      P = W - mu;
      Q = P.transpose().dot(P);
      sum += Q;
    }
    else
    {
      break;
    }
  }
  std = sum / N;

  // Log.log("CrossEntropyAgent::updateWeights std %.2f", std);
}

/// weight Update a maximum reward
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
  mean = sumWeights / RL_MEMORY_SIZE;
  // Log.log("New Mean: " + std::to_string(mean));
  Eigen::RowVectorXd  b = Eigen::Map<Eigen::RowVectorXd, Eigen::Unaligned>(tempWeights.data(), tempWeights.size());
  RowVectorXd  mu = RowVectorXd :: Constant(1,RL_MEMORY_SIZE,mean) ;
  float sum;
  float Q;
  VectorXd P;
  P = b - mu;
  // std::cout << "Here is the matrix m:\n" <<  P  << std::endl;
  Q = P.transpose().dot(P);
  std = Q /RL_MEMORY_SIZE;
  // Log.log("New std: " + std::to_string(std));
}

int CrossEntropyAgent::step(double reward, double state[])
{

  // std::cout << "Steps Rewards  " << reward << std::endl;
  // cerr << "steps" << endl;
  // double totalRewards = reward + Q[lastAction];

   if (hiveMind)
    loadColTabHeader(colTab, initialWeights);




  double r = reward - lastActionsTime;
  double delta = r - Q[lastAction];


  loadTiles(state);
  for (int a = 0; a < getNumActions(); a++)
  {
    Q[a] = computeQ(a);
  }
  // take actions with the maximal weights
  lastAction = selectAction(); 
  lastActionsTime = WM->getCurrentTime().getTime();

  char buffer[128];
  sprintf(buffer, "Q[%d] = %.2f", lastAction, Q[lastAction]);

  // Log.log("CrossEntropyAgent::step reward %.2f", lastAction, Q[lastAction]);

  LogDraw.logText( "Qmax", VecPosition( 25, -30 ),
                   buffer,
                   1, COLOR_BROWN );

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



// Add the reward and update Mean and Standard deviation
void CrossEntropyAgent::endEpisode(double reward)

{

  // std::cout << "End Reward" << reward << std::endl;
  Log.log("End of Episode Reward: " + std::to_string(reward));
  // Log.log("CrossEntropyAgent::step reward %.2f", lastAction);
  
  if(rand()%200 == 0){
    saveWeights(saveWeightsFile.c_str());
  }
  if (counter < N)
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

// bool CrossEntropyAgent::loadWeights(const char *filename)
// {
//   FileLock lock("loadWeights");
//   cerr << "Loading weights from " << filename << endl;

// #ifdef _Compress
//   igzstream is;
// #else
//   ifstream is;
// #endif

//   is.open(filename);
//   // myFile.open(filename)
//   if (!is.good())
//   {
//     cerr << "failed to open weight file: " << filename << endl;
//     return false;
//   }
//   // myFile.read((char *) &weights, RL_MEMORY_SIZE * sizeof(double)))
//   is.read(reinterpret_cast<char *>(&weights), std::streamsize(RL_MEMORY_SIZE * sizeof(double)));
//   colTab->restore(is);
//   is.close();
//   cerr << "...done" << endl;
//   return true;
// }

// bool CrossEntropyAgent::saveWeights(const char *filename)
// {
//   FileLock lock("saveWeights");

// #ifdef _Compress
//   ogzstream os;
// #else
//   ofstream os;
// #endif

//   if (!os.good())
//   {

//     cerr << "failed to open weight file: " << filename << endl;
//     return false;
//   }
//   cerr << "saved the weights" << endl;
//   os.write((const char *)&weights, RL_MEMORY_SIZE * sizeof(double));
//   colTab->save(os);
//   os.close();
//   return true;
// }



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




// Compute an action value from current F and theta
double CrossEntropyAgent::computeQ(int a)
{
  double q = 0;

  for (int j = 0; j < numTilings; j++)
  {
    q += weights[tiles[a][j]];
  }

  return q;
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