#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <cmath>
#include <climits>
#include <sstream>
#include <numeric>
#include "LinearSarsaAgent.h"
#include <cassert>

// If all is well, there should be no mention of anything keepaway- or soccer-
// related in this file.

extern Logger Log;

#if USE_DRAW_LOG
extern LoggerDraw LogDraw;
#endif

/**
 * Designed specifically to match the serialization format for collision_table.
 * See collision_table::save and collision_table::restore.
 */
#pragma pack(push, 1)
struct SharedData {
  long m;
  int safe;
  long calls;
  long clearhits;
  long collisions;

  double Q[MAX_ACTIONS];
  int numTilings;
  int lastAction;
  int lastActionTime;
  ObjectT K[11]; // mapping from index to player obj
  double minimumTrace;
  int numNonzeroTraces;
};
#pragma pack(pop)
#define VERBOSE_HIVE_MIND false

void LinearSarsaAgent::sync(bool load) {
  if (!hiveMind) return;

  FileLock lock(fileLockPrefix, weightsFile, "sync");
  Log.log(101, "LinearSarsaAgent::sync %s", load ? "load" : "save");

  if (load) {
    loadSharedData(weights);
  } else {
    saveSharedData(weights);
  }
}

/**
 * Assumes collision table header follows weights.
 * Returns the memory location after the header because that's useful for colTab
 * data array.
 */
void LinearSarsaAgent::loadSharedData(double *weights) {
  SharedData *shared =
      reinterpret_cast<SharedData *>(weights + RL_MEMORY_SIZE);

  // Do each field individually, since they don't all line up exactly for an
  // easy copy.
  colTab->calls = shared->calls;
  colTab->clearhits = shared->clearhits;
  colTab->collisions = shared->collisions;
  *const_cast<long *>(&colTab->m) = shared->m;
  *const_cast<int *>(&colTab->safe) = shared->safe;

  if (hiveMind > 1) {
    memcpy(Q, shared->Q, sizeof(shared->Q));
    numTilings = shared->numTilings;
    lastAction = shared->lastAction;
    lastActionTime = shared->lastActionTime;
    memcpy(K, shared->K, sizeof(shared->K));
    minimumTrace = shared->minimumTrace;
    numNonzeroTraces = shared->numNonzeroTraces;
  }

  if (VERBOSE_HIVE_MIND) {
    cerr << "Loaded shared:" << endl
         << " calls: " << colTab->calls << endl
         << " clearhits: " << colTab->clearhits << endl
         << " collisions: " << colTab->collisions << endl
         << " m: " << colTab->m << endl
         << " safe: " << colTab->safe << endl;
  }
}

void LinearSarsaAgent::saveSharedData(double *weights) {
  SharedData *shared =
      reinterpret_cast<SharedData *>(weights + RL_MEMORY_SIZE);

  // Do each field individually, since they don't all line up exactly for an
  // easy copy.
  shared->calls = colTab->calls;
  shared->clearhits = colTab->clearhits;
  shared->collisions = colTab->collisions;
  shared->m = colTab->m;
  shared->safe = colTab->safe;

  if (hiveMind > 1) {
    memcpy(shared->Q, Q, sizeof(Q));
    shared->numTilings = numTilings;
    shared->lastAction = lastAction;
    shared->lastActionTime = lastActionTime;
    memcpy(shared->K, K, sizeof(K));
    shared->minimumTrace = minimumTrace;
    shared->numNonzeroTraces = numNonzeroTraces;
  }

  if (VERBOSE_HIVE_MIND) {
    cerr << "Saved shared:" << endl
         << " calls: " << colTab->calls << endl
         << " clearhits: " << colTab->clearhits << endl
         << " collisions: " << colTab->collisions << endl
         << " m: " << colTab->m << endl
         << " safe: " << colTab->safe << endl;
  }
}

LinearSarsaAgent::LinearSarsaAgent(
    int numFeatures,
    bool bLearn,
    double widths[],
    string loadWeightsFile,
    string saveWeightsFile,
    int hiveMind,
    bool jointTiling,
    double gamma)
    : SMDPAgent(numFeatures),
      jointTiling(jointTiling),
      hiveFile(-1),
      gamma(gamma) {
  bLearning = bLearn;

  for (int i = 0; i < getNumFeatures(); i++) {
    tileWidths[i] = widths[i];
  }

  // Saving weights (including for hive mind) requires learning and a file name.
  this->hiveMind = 0;
  if (bLearning && saveWeightsFile.length() > 0) {
    const_cast<string &>(weightsFile) = saveWeightsFile; // thie is the weightfile in current dir

    bSaveWeights = true;

    // Hive mind further requires loading and saving from the same file.
    if (loadWeightsFile == saveWeightsFile) {
      // hive in /run/shm dir
      loadWeightsFile = sharedMemoryPrefix + loadWeightsFile;
      saveWeightsFile = sharedMemoryPrefix + saveWeightsFile;

      this->hiveMind = hiveMind;

      bool fileFound = !access(weightsFile.c_str(), F_OK);
      if (fileFound) { // copy weightsFile to /run/shm/weightsFile
        ifstream src(weightsFile, ios::binary);
        ofstream dst(loadWeightsFile, ios::binary);

        dst << src.rdbuf();
        src.close();
        dst.close();
      }
    }
  }
  else {
    bSaveWeights = false;
  }

  alpha = 0.125;
  lambda = 0.0;
  epsilon = 0.01;

  weights = weightsRaw;
  traces = tracesRaw;
  nonzeroTraces = nonzeroTracesRaw;
  nonzeroTracesInverse = nonzeroTracesInverseRaw;

  fill(traces, traces + RL_MEMORY_SIZE, 0.0);

  numTilings = 0;
  minimumTrace = 0.01;
  numNonzeroTraces = 0;

  for (int i = 0; i < RL_MEMORY_SIZE; i++) {
    weights[i] = 0;
    traces[i] = 0;
  }
  srand((unsigned int) 0);
  int tmp[2];
  float tmpf[2];
  colTab = new collision_table(RL_MEMORY_SIZE, 1);

  GetTiles(tmp, 1, 1, tmpf, 0);  // A dummy call to set the hashing table
  srand((unsigned int) time(NULL));

  if (loadWeightsFile.length() > 0)
    loadWeights(loadWeightsFile.c_str());

  // reset in case they have been changed in loadWeights
  numTilings = 0;
  lastAction = -1;
  lastActionTime = UnknownTime;
  minimumTrace = 0.01;
  numNonzeroTraces = 0;
}

void LinearSarsaAgent::setEpsilon(double epsilon) {
  this->epsilon = epsilon;
}

int LinearSarsaAgent::startEpisode(int current_time, double state[]) {
  Log.log(101, "LinearSarsaAgent::startEpisode current_time: %d", current_time);

  if (hiveMind > 1 &&
      lastActionTime != UnknownTime &&
      lastActionTime < current_time &&
      lastAction > 0) { // already started by another teammate
    double tau = current_time - lastActionTime;
    Log.log(101, "LinearSarsaAgent::startEpisode hived lastActionTime: %d", lastActionTime);
    Log.log(101, "LinearSarsaAgent::startEpisode hived tau: %f", tau);
    return step(current_time, tau, state);
  }

  decayTraces(0);
  assert(numNonzeroTraces == 0);

  loadTiles(state);
  for (int a = 0; a < getNumActions(); a++) {
    Q[a] = computeQ(a);
  }

  lastAction = selectAction(state); // share Q[lastAction] in hive mind mode
  lastActionTime = current_time;

  {
    stringstream ss;
    for (int a = 0; a < getNumActions(); ++a) {
      ss << a << ":" << Q[a] << ", ";
    }
    Log.log(101, "LinearSarsaAgent::startEpisode numTilings: %d", numTilings);
    Log.log(101, "LinearSarsaAgent::startEpisode Q: [%s]", ss.str().c_str());
    Log.log(101, "LinearSarsaAgent::startEpisode action: %d", lastAction);
  }

#if USE_DRAW_LOG
  char buffer[128];
  sprintf( buffer, "Q[%d] = %.2f", lastAction, Q[lastAction] );
  LogDraw.logText( "Qmax", VecPosition( 25, -30 ),
                   buffer,
                   1, COLOR_BROWN );
#endif

  for (int j = 0; j < numTilings; j++)
    setTrace(tiles[lastAction][j], 1.0);

  Log.log(101, "LinearSarsaAgent::step saved numNonzeroTraces: %d", numNonzeroTraces);
  return lastAction;
}

double LinearSarsaAgent::reward(double tau, double gamma) {
  double ret = tau;

  if (gamma < 1.0) {
    ret = (1.0 - pow(gamma, tau)) / (1.0 - gamma);
  }

  return ret;
}

int LinearSarsaAgent::step(int current_time, double reward_, double state[]) {
  double tau = reward_; // here reward is actually tau

  Log.log(101, "LinearSarsaAgent::step current_time: %d", current_time);
  Log.log(101, "LinearSarsaAgent::step tau: %f", tau);

  if (hiveMind > 1 && lastActionTime != UnknownTime && lastActionTime < current_time) {
    tau = current_time - lastActionTime;
    Log.log(101, "LinearSarsaAgent::step hived lastActionTime: %d", lastActionTime);
    Log.log(101, "LinearSarsaAgent::step hived lastAction: %d %s", lastAction,
            JointActionSpace::ins().getJointActionName(lastAction));
    Log.log(101, "LinearSarsaAgent::step hived tau: %f", tau);
  }

  assert(lastAction >= 0);
  double delta = reward(tau, gamma) - Q[lastAction]; //r - Q_{t-1}[s_{t-1}, a_{t-1}]

  loadTiles(state); //s_t
  for (int a = 0; a < getNumActions(); a++) {
    Q[a] = computeQ(a); //Q_{t-1}[s_t, *]
  }

  lastAction = selectAction(state); //a_t
  lastActionTime = current_time;

  {
    stringstream ss;
    for (int a = 0; a < getNumActions(); ++a) {
      ss << a << ":" << Q[a] << ", ";
    }
    Log.log(101, "LinearSarsaAgent::step numTilings: %d", numTilings);
    Log.log(101, "LinearSarsaAgent::step Q: [%s]", ss.str().c_str());
    Log.log(101, "LinearSarsaAgent::step action: %d", lastAction);
  }

#if USE_DRAW_LOG
  char buffer[128];
  sprintf( buffer, "Q[%d] = %.2f", lastAction, Q[lastAction] );
  LogDraw.logText( "Qmax", VecPosition( 25, -30 ),
                   buffer,
                   1, COLOR_BROWN );
#endif

  if (!bLearning)
    return lastAction;

#if USE_DRAW_LOG

    //char buffer[128];
  sprintf( buffer, "reward: %.2f", reward );
  LogDraw.logText( "reward", VecPosition( 25, 30 ),
                   buffer,
                   1, COLOR_NAVY );
#endif

  assert(!std::isnan(Q[lastAction]) && !std::isinf(Q[lastAction]));
  delta += pow(gamma, tau) * Q[lastAction]; //delta += Q_{t-1}[s_t, a_t]

  updateWeights(delta); //Q_t <- Q_{t-1}
  Q[lastAction] = computeQ(lastAction); //need to redo because weights changed: Q_t[s_t, a_t]

  decayTraces(gamma * lambda);

  for (int a = 0; a < getNumActions(); a++) {  //clear other than F[a]
    if (a != lastAction) {
      for (int j = 0; j < numTilings; j++)
        clearTrace(tiles[a][j]);
    }
  }

  for (int j = 0; j < numTilings; j++)      //replace/set traces F[a]
    setTrace(tiles[lastAction][j], 1.0);

  Log.log(101, "LinearSarsaAgent::step saved numNonzeroTraces: %d", numNonzeroTraces);
  return lastAction;
}

void LinearSarsaAgent::endEpisode(int current_time, double reward_) {
  double tau = reward_; // here reward is actually tau

  Log.log(101, "LinearSarsaAgent::endEpisode current_time: %d", current_time);
  Log.log(101, "LinearSarsaAgent::endEpisode tau: %f", tau);
  Log.log(101, "LinearSarsaAgent::endEpisode hived lastAction: %d", lastAction);
  Log.log(101, "LinearSarsaAgent::endEpisode hived lastActionTime: %d", lastActionTime);

  if (bLearning && lastAction != -1) { /* otherwise we never ran on this episode */
    if (hiveMind > 1 && lastActionTime != UnknownTime && lastActionTime < current_time) {
      tau = current_time - lastActionTime;
      Log.log(101, "LinearSarsaAgent::endEpisode hived tau: %f", tau);
    }

#if USE_DRAW_LOG
    char buffer[128];
      sprintf( buffer, "reward: %.2f", reward );
      LogDraw.logText( "reward", VecPosition( 25, 30 ),
                       buffer,
                       1, COLOR_NAVY );
#endif

    double delta = reward(tau, gamma) - Q[lastAction];
    updateWeights(delta);
  }

  if (bLearning && bSaveWeights && rand() % 200 == 0 && !hiveMind) {
    saveWeights(weightsFile.c_str());
  }
  lastAction = -1;
  lastActionTime = UnknownTime;
}

size_t LinearSarsaAgent::mmapSize() {
  if (hiveMind > 1) {
    return sizeof(weightsRaw) +
           sizeof(SharedData) +
           colTab->m * sizeof(long) +
           sizeof(tracesRaw) +
           sizeof(nonzeroTracesRaw) +
           sizeof(nonzeroTracesInverseRaw);
  }
  else {
    return sizeof(weightsRaw) +
           sizeof(SharedData) +
           colTab->m * sizeof(long);
  }
}

void LinearSarsaAgent::shutDown() {
  FileLock lock(fileLockPrefix, weightsFile, "shutDown");

  if (bLearning && bSaveWeights) {
    cerr << "Saving weights at shutdown." << endl;
    saveWeights(weightsFile.c_str());
  }

  if (hiveMind) {
    string sharedWeightFile = sharedMemoryPrefix + weightsFile;
    bool fileFound = !access(sharedWeightFile.c_str(), F_OK);
    if (fileFound) { // copy /run/shm/weightsFile to weightsFile
      ifstream src(sharedWeightFile, ios::binary);
      ofstream dst(weightsFile, ios::binary);

      dst << src.rdbuf();
      src.close();
      dst.close();
    }

    size_t mmapLength = mmapSize();
    munmap(weights, mmapLength);
    close(hiveFile);
    hiveFile = -1;

    // Go back to the own arrays, since our map is no longer valid.
    weights = weightsRaw;
    traces = tracesRaw;
    nonzeroTraces = nonzeroTracesRaw;
    nonzeroTracesInverse = nonzeroTracesInverseRaw;
    colTab->data = new long[colTab->m];
  }
}

int LinearSarsaAgent::selectAction(double state[]) {
  int action;

  // Epsilon-greedy
  if (bLearning && drand48() < epsilon) {     /* explore */
    action = JointActionSpace::ins().sample(state, getNumFeatures());
  }
  else {
    action = argmaxQ(state);
  }

  assert(action >= 0);
  return action;
}

bool LinearSarsaAgent::loadWeights(const char *filename) {
  cerr << "Loading weights from " << filename << endl;

  if (hiveMind) {
    if (hiveFile < 0) {
      // First, check the lock file, so we have only one initializer.
      // Later interaction should be approximately synchronized by having only
      // one active player at a time per team, but we can't assume that here.
      FileLock lock(fileLockPrefix, weightsFile, "loadWeights");

      // First, see if the file is already there.
      bool fileFound = !access(filename, F_OK);
      // TODO Extract constant for permissions (0664)?
      hiveFile = open(filename, O_RDWR | O_CREAT, 0664);
      size_t mmapLength = mmapSize();

      if (!fileFound) {
        // Make the file the right size.
        cerr << "Initializing new hive file." << endl;
        if (lseek(hiveFile, mmapLength - 1, SEEK_SET) < 0) {
          throw "failed to seek initial file size";
        }
        if (write(hiveFile, "", 1) < 0) {
          throw "failed to expand initial file";
        }
      }

      if (hiveFile < 0) throw "failed to open hive file";
      void *hiveMap =
          mmap(NULL, mmapLength, PROT_READ | PROT_WRITE, MAP_SHARED, hiveFile, 0);
      if (hiveMap == MAP_FAILED) throw "failed to map hive file";

      // First the weights.
      weights = reinterpret_cast<double *>(hiveMap);

      // Now the collision table header.
      SharedData *shared =
          reinterpret_cast<SharedData *>(weights + RL_MEMORY_SIZE);
      if (fileFound) {
        loadSharedData(weights);
      }

      // Now the collision table data.
      delete[] colTab->data;
      colTab->data = reinterpret_cast<long *>(shared + 1);

      if (hiveMind > 1) {
        traces = reinterpret_cast<double *>(colTab->data + colTab->m);
        nonzeroTraces = reinterpret_cast<int *>(traces + RL_MEMORY_SIZE);
        nonzeroTracesInverse = nonzeroTraces + RL_MAX_NONZERO_TRACES;
      }

      fill(traces, traces + RL_MEMORY_SIZE, 0.0);
      colTab->reset();

      if (!fileFound) {
        fill(weights, weights + RL_MEMORY_SIZE, 0.0);
        saveWeights(weightsFile.c_str());
      }
    }
  } else {
    int file = open(filename, O_RDONLY);
    if (file < 0) {
      cerr << "failed to open weight file: " << filename << endl;
      return false;
    }
    read(file, (char *) weights, RL_MEMORY_SIZE * sizeof(double));
    colTab->restore(file);
    close(file);
  }
  cerr << "...done" << endl;
  return true;
}

bool LinearSarsaAgent::saveWeights(const char *filename) {
  if (hiveMind) {
    // The big arrays should be saved out automatically, but we still need to
    // handle the collision table header.
    saveSharedData(weights);
  } else {
    int file = open(filename, O_CREAT | O_WRONLY, 0664);
    if (file < 0) {
      cerr << "failed to open weight file: " << filename << endl;
      return false;
    }
    write(file, (char *) weights, RL_MEMORY_SIZE * sizeof(double));
    colTab->save(file);
    close(file);
  }
  return true;
}

// Compute an action value from current F and theta
double LinearSarsaAgent::computeQ(int a) {
  double q = 0;
  for (int j = 0; j < numTilings; j++) {
    q += weights[tiles[a][j]];
  }

  return q;
}

// Returns index (action) of largest entry in Q array, breaking ties randomly
int LinearSarsaAgent::argmaxQ(double state[]) const {
  bool tmControllBall = state[getNumFeatures() - 1] > 0.5;

  int bestAction = 0;
  double bestValue = INT_MIN;
  int numTies = 0;
  for (int a = 0; a < getNumActions(); a++) {
    if (JointActionSpace::ins().getJointAction(a)->tmControllBall != tmControllBall)
      continue;

    double value = Q[a];
    if (value > bestValue) {
      bestValue = value;
      bestAction = a;
    }
    else if (value == bestValue) {
      numTies++;
      if (rand() % (numTies + 1) == 0) {
        bestValue = value;
        bestAction = a;
      }
    }
  }

  return bestAction;
}

void LinearSarsaAgent::updateWeights(double delta) {
  assert(numTilings > 0);
  double tmp = delta * alpha / numTilings;

  for (int i = 0; i < numNonzeroTraces; i++) {
    assert(i < RL_MAX_NONZERO_TRACES);

    int f = nonzeroTraces[i];
    assert(f >= 0);
    assert(f < RL_MEMORY_SIZE);

    if (f >= RL_MEMORY_SIZE || f < 0) {
      continue;
    }

    weights[f] += tmp * traces[f];
    assert(!std::isnan(weights[f]));
    assert(!std::isinf(weights[f]));
  }
}

void LinearSarsaAgent::loadTiles(double state[]) // will change colTab->data implictly
{
  const int tilingsPerGroup = 32;

  if (jointTiling) {
    numTilings = tilingsPerGroup * getNumFeatures();

    float state2[MAX_STATE_VARS];
    for (int v = 0; v < getNumFeatures(); v++) {
      state2[v] = (float) (state[v] / tileWidths[v]);
    }

    for (int a = 0; a < getNumActions(); a++) {
      GetTiles(&(tiles[a][0]), numTilings, colTab,
               state2, getNumFeatures(), a);
    }
  }
  else { // tiling per state variable
    numTilings = 0;

    /* These are the 'tiling groups'  --  play here with representations */
    /* One tiling for each state variable */
    for (int v = 0; v < getNumFeatures(); v++) {
      for (int a = 0; a < getNumActions(); a++) {
        GetTiles1(&(tiles[a][numTilings]), tilingsPerGroup, colTab,
                  (float) (state[v] / tileWidths[v]), a, v);
      }
      numTilings += tilingsPerGroup;
    }
  }

  assert(numTilings > 0);
  assert(numTilings < RL_MAX_NUM_TILINGS);
}


// Clear any trace for feature f
void LinearSarsaAgent::clearTrace(int f) {
  if (f >= RL_MEMORY_SIZE || f < 0) {
    cerr << "ClearTrace: f out of range " << f << endl;
    return;
  }

  if (traces[f] != 0)
    clearExistentTrace(f, nonzeroTracesInverse[f]);
}

// Clear the trace for feature f at location loc in the list of nonzero traces
void LinearSarsaAgent::clearExistentTrace(int f, int loc) {
  if (f >= RL_MEMORY_SIZE || f < 0) {
    cerr << "ClearExistentTrace: f out of range " << f << endl;
    return;
  }

  traces[f] = 0.0;

  if (numNonzeroTraces > 0) {
    numNonzeroTraces--;
    nonzeroTraces[loc] = nonzeroTraces[numNonzeroTraces];
    nonzeroTracesInverse[nonzeroTraces[loc]] = loc;
  }
  else {
    fill(traces, traces + RL_MEMORY_SIZE, 0.0);
    assert(numNonzeroTraces == 0);
  }
}

// Decays all the (nonzero) traces by decay_rate, removing those below minimum_trace
void LinearSarsaAgent::decayTraces(double decayRate) {
  for (int loc = numNonzeroTraces - 1; loc >= 0; loc--) {
    int f = nonzeroTraces[loc];
    if (f >= RL_MEMORY_SIZE || f < 0) {
      cerr << "DecayTraces: f out of range " << f << endl;
      continue;
    }

    traces[f] *= decayRate;
    if (traces[f] < minimumTrace)
      clearExistentTrace(f, loc);
  }
}

// Set the trace for feature f to the given value, which must be positive
void LinearSarsaAgent::setTrace(int f, float newTraceValue) {
  if (f >= RL_MEMORY_SIZE || f < 0) {
    cerr << "SetTraces: f out of range " << f << endl;
    return;
  }

  if (traces[f] >= minimumTrace) {
    traces[f] = newTraceValue;         // trace already exists
  }
  else {
    while (numNonzeroTraces >= RL_MAX_NONZERO_TRACES) {
      increaseMinTrace(); // ensure room for new trace
    }

    traces[f] = newTraceValue;
    assert(numNonzeroTraces >= 0);
    assert(numNonzeroTraces < RL_MAX_NONZERO_TRACES);
    nonzeroTraces[numNonzeroTraces] = f;
    nonzeroTracesInverse[f] = numNonzeroTraces;
    numNonzeroTraces++;
  }
}

// Try to make room for more traces by incrementing minimum_trace by 10%,
// culling any traces that fall below the new minimum
void LinearSarsaAgent::increaseMinTrace() {
  minimumTrace *= 1.1;
  cerr << "Changing minimum_trace to " << minimumTrace << endl;
  for (int loc = numNonzeroTraces - 1; loc >= 0; loc--) { // necessary to loop downwards
    int f = nonzeroTraces[loc];
    if (traces[f] < minimumTrace)
      clearExistentTrace(f, loc);
  }
}

void LinearSarsaAgent::setParams(int iCutoffEpisodes, int iStopLearningEpisodes) {
  /* set learning parameters */
}
