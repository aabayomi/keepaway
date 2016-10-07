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

namespace jol {
void LinearSarsaAgent::sync(bool load) {
  if (sharedData) {
    FileLock lock("/run/lock/", sharedMemoryName.substr(1));
    Log.log(101, "LinearSarsaAgent::sync %s", load ? "load" : "save");

    if (load) {
      loadSharedData(sharedData);
    } else {
      saveSharedData(sharedData);
    }
  }
}

void LinearSarsaAgent::loadSharedData(SharedData *address) {
  SharedData *shared = address;
  lastAction = shared->lastAction;
  lastActionTime = shared->lastActionTime;
}

void LinearSarsaAgent::saveSharedData(SharedData *address) {
  SharedData *shared = address;
  shared->lastAction = lastAction;
  shared->lastActionTime = lastActionTime;
}

LinearSarsaAgent::~LinearSarsaAgent() {
  if (sharedData) {
    shm_unlink(sharedMemoryName.c_str());
  }
  delete colTab;
}

LinearSarsaAgent::LinearSarsaAgent(
    WorldModel *wm,
    int numFeatures,
    bool bLearn,
    double widths[],
    string loadWeightsFile,
    string saveWeightsFile,
    double gamma,
    double initialWeight)
    : SMDPAgent(numFeatures, wm),
      saveWeightsFile(saveWeightsFile),
      gamma(gamma),
      initialWeight(initialWeight) {
  bLearning = bLearn;

  string exepath = getexepath();
  exepath += loadWeightsFile;
  exepath += saveWeightsFile;
  auto h = hash<string>().operator()(exepath); //hashing

  fileLockPrefix = "/run/lock/" + to_string(h);
  sharedMemoryName = "/" + to_string(h) + ".shm";
  sharedData = 0;

  for (int i = 0; i < getNumFeatures(); i++) {
    tileWidths[i] = widths[i];
  }

  bSaveWeights = bLearning && saveWeightsFile.length() > 0;

  alpha = 0.125;
  lambda = 0.0;
  epsilon = 0.01;

  fill(traces, traces + RL_MEMORY_SIZE, 0.0);
  fill(weights, weights + RL_MEMORY_SIZE, initialWeight);
  fill(Q, Q + MAX_RL_ACTIONS, 0.0);

  numTilings = 0;
  minimumTrace = 0.01;
  numNonzeroTraces = 0;

  srand((unsigned int) 0);
  srand48((unsigned int) 0);
  int tmp[2];
  float tmpf[2];
  colTab = new collision_table(RL_MEMORY_SIZE, 1);

  GetTiles(tmp, 1, 1, tmpf, 0);  // A dummy call to set the hashing table
  srand((unsigned int) time(NULL));
  srand48((unsigned int) time(NULL));

  if (loadWeightsFile.length() > 0)
    loadWeights(loadWeightsFile.c_str());

  // reset in case they have been changed in loadWeights
  numTilings = 0;
  lastAction = -1;
  lastActionTime = UnknownTime;
  minimumTrace = 0.01;
  numNonzeroTraces = 0;

  if (bLearning) { // setup shared data
    /* create the shared memory segment as if it was a file */
    int shm_fd = shm_open(sharedMemoryName.c_str(), O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
      printf("prod: Shared memory failed: %s\n", strerror(errno));
      exit(1);
    }

    /* configure the size of the shared memory segment */
    ftruncate(shm_fd, sizeof(SharedData));

    /* map the shared memory segment to the address space of the process */
    sharedData = (SharedData *) mmap(0, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (sharedData == MAP_FAILED) {
      printf("prod: Map failed: %s\n", strerror(errno));
      exit(1);
    }
  }
}

void LinearSarsaAgent::setEpsilon(double epsilon) {
  this->epsilon = epsilon;
}

int LinearSarsaAgent::startEpisode(double state[]) {
  Log.log(101, "LinearSarsaAgentstartEpisode");

  decayTraces(0);
  assert(numNonzeroTraces == 0);

  loadTiles(state);
  for (int a : validActions()) {
    Q[a] = computeQ(a);
  }

  lastAction = selectAction();

  if (Log.isInLogLevel(101)) {
    stringstream ss;
    for (int a : validActions()) {
      ss << a << ":" << Q[a] << ", ";
    }
    Log.log(101, "LinearSarsaAgent::startEpisode numTilings: %d", numTilings);
    Log.log(101, "LinearSarsaAgent::startEpisode Q: [%s]", ss.str().c_str());
    Log.log(101, "LinearSarsaAgent::startEpisode action: %d", lastAction);
  }

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

int LinearSarsaAgent::step(double reward_, double state[]) {
  double tau = reward_; // here reward is actually tau
  Log.log(101, "LinearSarsaAgent::step tau: %f", tau);

  assert(lastAction >= 0);
  double delta = reward(tau, gamma) - Q[lastAction]; //r - Q_{t-1}[s_{t-1}, a_{t-1}]

  loadTiles(state); //s_t
  for (int a : validActions()) {
    Q[a] = computeQ(a); //Q_{t-1}[s_t, *]
  }

  lastAction = selectAction(); //a_t

  if (Log.isInLogLevel(101)) {
    stringstream ss;
    for (int a : validActions()) {
      ss << a << ":" << Q[a] << ", ";
    }
    Log.log(101, "LinearSarsaAgent::step numTilings: %d", numTilings);
    Log.log(101, "LinearSarsaAgent::step Q: [%s]", ss.str().c_str());
    Log.log(101, "LinearSarsaAgent::step action: %d", lastAction);
  }

  if (!bLearning)
    return lastAction;

  assert(!std::isnan(Q[lastAction]) && !std::isinf(Q[lastAction]));
  delta += pow(gamma, tau) * Q[lastAction]; //delta += gamma**tau * Q_{t-1}[s_t, a_t]

  updateWeights(delta);
  Q[lastAction] = computeQ(lastAction); //need to redo because weights changed: Q_t[s_t, a_t]

  decayTraces(gamma * lambda);

  for (int a : validActions()) {
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

void LinearSarsaAgent::endEpisode(double reward_) {
  double tau = reward_; // here reward is actually tau

  Log.log(101, "LinearSarsaAgent::endEpisode tau: %f", tau);
  Log.log(101, "LinearSarsaAgent::endEpisode hived lastAction: %d", lastAction);

  if (bLearning && lastAction != -1) { /* otherwise we never ran on this episode */
    double delta = reward(tau, gamma) - Q[lastAction];
    updateWeights(delta);
  }

  if (bLearning && bSaveWeights && rand() % 200 == 0) {
    saveWeights(saveWeightsFile.c_str());
  }
  lastAction = -1;
  lastActionTime = UnknownTime;
}

void LinearSarsaAgent::shutDown() {
  if (bLearning && bSaveWeights) {
    cerr << "Saving weights at shutdown." << endl;
    saveWeights(saveWeightsFile.c_str());
  }
}

int LinearSarsaAgent::selectAction() {
  int &action = lastAction;

  if (agentIdx == 0 || !bLearning) {
    if (bLearning && drand48() < epsilon) {     /* explore */
      action = JointActionSpace::ins().sample(WM->tmControllBall()); // sample
      Log.log(101, "LinearSarsaAgent::selectAction explore action %d", action);
    } else {
      action = argmaxQ();
      Log.log(101, "LinearSarsaAgent::selectAction argmaxQ action %d", action);
    }

    lastActionTime = WM->getCurrentCycle();
    sync(false);
  } else {
    action = -1;
    int loops = 100;
    while (loops--) { // wait for K0
      Log.log(101, "wait for K0 %d", loops);
      static const timespec sleepTime = {0, 1 * 1000 * 1000}; //1ms
      nanosleep(&sleepTime, NULL);
      sync(true); // load shared memory
      if (action != -1 && lastActionTime == WM->getCurrentCycle()) break;
    }

    if (action == -1 || lastActionTime != WM->getCurrentCycle()) {
      Log.log(101, "wait for K0 -- timeout");
      action = JointActionSpace::ins().sample(WM->tmControllBall()); // sample
      Log.log(101, "LinearSarsaAgent::selectAction explore action %d", action);
    }
  }

  assert(action >= 0);
  assert(action < getNumActions());
  assert(JointActionSpace::ins().getJointAction(action)->tmControllBall == WM->tmControllBall());
  assert(find(validActions().begin(), validActions().end(), action) != validActions().end());

  return action;
}

bool LinearSarsaAgent::loadWeights(const char *filename) {
  FileLock lock(fileLockPrefix, "loadWeights");

  cerr << "Loading weights from " << filename << endl;
  int file = open(filename, O_RDONLY);
  if (file < 0) {
    cerr << "failed to open weight file: " << filename << endl;
    return false;
  }
  read(file, (char *) weights, RL_MEMORY_SIZE * sizeof(double));
  colTab->restore(file);
  close(file);
  cerr << "...done" << endl;
  return true;
}

bool LinearSarsaAgent::saveWeights(const char *filename) {
  FileLock lock(fileLockPrefix, "saveWeights");

  int file = open(filename, O_CREAT | O_WRONLY, 0664);
  if (file < 0) {
    cerr << "failed to open weight file: " << filename << endl;
    return false;
  }
  write(file, (char *) weights, RL_MEMORY_SIZE * sizeof(double));
  colTab->save(file);
  close(file);
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

const vector<int> &LinearSarsaAgent::validActions() const {
  static vector<int> actions[2];

  if (actions[0].empty() || actions[1].empty()) {
    for (int a = 0; a < getNumActions(); a++) {
      auto tmControllBall = JointActionSpace::ins().getJointAction(a)->tmControllBall;
      actions[tmControllBall].push_back(a);
    }
  }

  return actions[WM->tmControllBall()];
}

// Returns index (action) of largest entry in Q array, breaking ties randomly
int LinearSarsaAgent::argmaxQ() const {
  int bestAction = 0;
  double bestValue = INT_MIN;
  int numTies = 0;

  for (int a : validActions()) {
    double value = Q[a];
    if (value > bestValue) {
      bestValue = value;
      bestAction = a;
    } else if (value == bestValue) {
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

  Log.log(101, "LinearSarsaAgent::updateWeights delta %f", delta);

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

  numTilings = 0;
  for (int v = 0; v < getNumFeatures(); v++) {
    for (int a : validActions()) {
      GetTiles1(&(tiles[a][numTilings]), tilingsPerGroup, colTab,
                (float) (state[v] / tileWidths[v]), a, v);
    }
    numTilings += tilingsPerGroup;
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
  } else {
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
  } else {
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

}
