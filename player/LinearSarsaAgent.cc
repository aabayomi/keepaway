#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <cmath>
#include <sstream>
#include <numeric>
#include "LinearSarsaAgent.h"

// If all is well, there should be no mention of anything keepaway- or soccer-
// related in this file.

extern Logger Log;

namespace jol {

LinearSarsaAgent::~LinearSarsaAgent() {
  if (sharedData) shm_unlink(sharedMemoryName.c_str());
  delete barrier;
}

LinearSarsaAgent::LinearSarsaAgent(
    WorldModel *wm,
    int numFeatures,
    bool bLearn,
    double widths[],
    string loadWeightsFile,
    string saveWeightsFile,
    double gamma,
    double initialWeight,
    bool qLearning)
    : SMDPAgent(numFeatures, wm),
      saveWeightsFile(saveWeightsFile),
      bLearning(bLearn),
      barrier(0),
      sharedData(0),
      gamma(gamma),
      initialWeight(initialWeight),
      qLearning(qLearning) {
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

  GetTiles(tmp, 1, 1, tmpf, 0);  // A dummy call to set the hashing table
  srand((unsigned int) time(NULL));
  srand48((unsigned int) time(NULL));

  numTilings = 0;
  lastAction = -1;
  lastActionTime = UnknownTime;
  minimumTrace = 0.01;
  numNonzeroTraces = 0;

  if (bLearning) { // setup shared data
    string exepath = getexepath();
    exepath += "LinearSarsaAgent::LinearSarsaAgent";
    exepath += to_string(gamma);
    exepath += to_string(initialWeight);
    exepath += to_string(qLearning);
    auto h = hash<string>()(exepath); //hashing
    sharedMemoryName = "/" + to_string(h) + ".shm";

    int shm_fd = shm_open(sharedMemoryName.c_str(), O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
      printf("prod: Shared memory failed: %s\n", strerror(errno));
      exit(1);
    }

    ftruncate(shm_fd, sizeof(SharedData));
    sharedData = (SharedData *) mmap(0, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (sharedData == MAP_FAILED) {
      printf("prod: Map failed: %s\n", strerror(errno));
      exit(1);
    }

    barrier = new Barrier(AtomicAction::num_keepers, &(sharedData->numBlocked), h);
    sharedData->reset();
  }

  if (loadWeightsFile.length() > 0)
    loadWeights(loadWeightsFile.c_str());
}

void LinearSarsaAgent::setEpsilon(double epsilon) {
  this->epsilon = epsilon;
}

int LinearSarsaAgent::startEpisode(double state[]) {
  Log.log(101, "LinearSarsaAgentstartEpisode");

  decayTraces(0);
  Assert(numNonzeroTraces == 0);

  loadTiles(state);
  for (int a : validActions()) {
    Q[a] = computeQ(a);
  }
  lastAction = selectAction();

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

  Assert(lastAction >= 0);
  double delta = reward(tau, gamma) - Q[lastAction];
  loadTiles(state);
  for (int a : validActions()) {
    Q[a] = computeQ(a);
  }
  lastAction = selectAction();

  if (!bLearning) return lastAction;
  Assert(!std::isnan(Q[lastAction]) && !std::isinf(Q[lastAction]));

  if (qLearning) {
    delta += pow(gamma, tau) * Q[argmaxQ()];
  }
  else {
    delta += pow(gamma, tau) * Q[lastAction];
  }

  updateWeights(delta);
  Q[lastAction] = computeQ(lastAction); //need to redo because weights changed
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

  if (bLearning && lastAction >= 0) { /* otherwise we never ran on this episode */
    double delta = reward(tau, gamma) - Q[lastAction];
    updateWeights(delta);
  }

  if (bLearning && bSaveWeights && rand() % 1000 == 0) {
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
  if (agentIdx == 0 || !bLearning) {
    if (bLearning && drand48() < epsilon) {     /* explore */
      lastAction = JointActionSpace::ins().sample(WM->isTmControllBall()); // sample
      Log.log(101, "LinearSarsaAgent::selectAction explore action %d", lastAction);
    } else {
      lastAction = argmaxQ();
      Log.log(101, "LinearSarsaAgent::selectAction argmaxQ action %d", lastAction);
    }

    lastActionTime = WM->getCurrentCycle();
    if (sharedData) {
      sharedData->lastAction = lastAction;
      sharedData->lastActionTime = lastActionTime;
    }
  }

  if (bLearning) {
    barrier->wait();
    lastAction = sharedData->lastAction;
    lastActionTime = sharedData->lastActionTime;
    Log.log(101, "got %d at %d [isTmControllBall %d]", lastAction, lastActionTime, WM->isTmControllBall());
    barrier->wait();
  }

  if (Log.isInLogLevel(101)) {
    stringstream ss;
    for (int a : validActions()) {
      ss << a << ":" << Q[a] << ", ";
    }
    Log.log(101, "LinearSarsaAgent::selectAction numTilings: %d", numTilings);
    Log.log(101, "LinearSarsaAgent::selectAction Q: [%s]", ss.str().c_str());
    Log.log(101, "LinearSarsaAgent::selectAction action: %d", lastAction);
  }

  Assert(lastAction >= 0);
  Assert(lastAction < getNumActions());

  return lastAction;
}

bool LinearSarsaAgent::loadWeights(const char *filename) {
  cerr << "Loading weights from " << filename << endl;
  int file = open(filename, O_RDONLY);
  if (file < 0) {
    cerr << "failed to open weight file: " << filename << endl;
    return false;
  }
  read(file, (char *) weights, RL_MEMORY_SIZE * sizeof(double));
  colTab.restore(file);
  close(file);
  cerr << "...done" << endl;
  return true;
}

bool LinearSarsaAgent::saveWeights(const char *filename) {
  int file = open(filename, O_CREAT | O_WRONLY, 0664);
  if (file < 0) {
    cerr << "failed to open weight file: " << filename << endl;
    return false;
  }
  write(file, (char *) weights, RL_MEMORY_SIZE * sizeof(double));
  colTab.save(file);
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

  return actions[WM->isTmControllBall()];
}

// Returns index (action) of largest entry in Q array, breaking ties randomly
int LinearSarsaAgent::argmaxQ() const {
  int bestAction = 0;
  double bestValue = numeric_limits<double>::min();
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
  Log.log(101, "LinearSarsaAgent::updateWeights delta %f", delta);

  Assert(numTilings > 0);
  double tmp = delta * alpha / numTilings;

  for (int i = 0; i < numNonzeroTraces; i++) {
    Assert(i < RL_MAX_NONZERO_TRACES);

    int f = nonzeroTraces[i];
    Assert(f >= 0);
    Assert(f < RL_MEMORY_SIZE);

    if (f >= RL_MEMORY_SIZE || f < 0) {
      continue;
    }

    weights[f] += tmp * traces[f];
    Assert(!std::isnan(weights[f]));
    Assert(!std::isinf(weights[f]));
  }
}

void LinearSarsaAgent::loadTiles(double state[]) // will change colTab.data implictly
{
  const int tilingsPerGroup = 32;

  numTilings = 0;
  for (int v = 0; v < getNumFeatures(); v++) {
    for (int a : validActions()) {
      GetTiles1(&(tiles[a][numTilings]), tilingsPerGroup, &colTab,
                (float) (state[v] / tileWidths[v]), a, v);
    }
    numTilings += tilingsPerGroup;
  }

  Assert(numTilings > 0);
  Assert(numTilings < RL_MAX_NUM_TILINGS);
}

// Clear any trace for feature f
void LinearSarsaAgent::clearTrace(int f) {
  if (f >= RL_MEMORY_SIZE || f < 0) {
    Assert(0);
    cerr << "ClearTrace: f out of range " << f << endl;
    return;
  }

  if (traces[f] != 0)
    clearExistentTrace(f, nonzeroTracesInverse[f]);
}

// Clear the trace for feature f at location loc in the list of nonzero traces
void LinearSarsaAgent::clearExistentTrace(int f, int loc) {
  if (f >= RL_MEMORY_SIZE || f < 0) {
    Assert(0);
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
    Assert(numNonzeroTraces == 0);
  }
}

// Decays all the (nonzero) traces by decay_rate, removing those below minimum_trace
void LinearSarsaAgent::decayTraces(double decayRate) {
  for (int loc = numNonzeroTraces - 1; loc >= 0; loc--) {
    int f = nonzeroTraces[loc];
    if (f >= RL_MEMORY_SIZE || f < 0) {
      Assert(0);
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
    Assert(0);
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
    Assert(numNonzeroTraces >= 0);
    Assert(numNonzeroTraces < RL_MAX_NONZERO_TRACES);
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
