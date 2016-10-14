//
// Created by baj on 10/4/16.
//

#include <cassert>
#include <climits>
#include <sstream>
#include "LinearSarsaLearner.h"

namespace fsm {

void SharedData::setBlocked(int i) {
  auto status = runningStatus;
//  assert(!isBlocked(i));
  runningStatus |= 1 << i;
//  assert(isBlocked(i));
  Log.log(101, "SharedData::setBlocked %d (%d => %d)", i, status, runningStatus);
}

void SharedData::clearBlocked(int i) {
  auto status = runningStatus;
//  assert(isBlocked(i));
  runningStatus &= ~(1 << i);
//  assert(!isBlocked(i));
  Log.log(101, "SharedData::clearBlocked %d (%d => %d)", i, status, runningStatus);
}

bool SharedData::isBlocked(int i) {
  return (bool) ((runningStatus >> i) & 1);
}

bool SharedData::isAllBlocked(int k) {
  return runningStatus + 1 == 1 << k;
}

void SharedData::reset() {
  runningStatus = 0;
  lastJointChoiceIdx = -1;
  lastJointChoiceTime = UnknownTime;
  memset(lastJointChoice, 0, sizeof(lastJointChoice));

  memset(Q, 0, sizeof(Q));
  memset(weights, 0, sizeof(weights));
  memset(traces, 0, sizeof(traces));
  memset(nonzeroTraces, 0, sizeof(nonzeroTraces));
  memset(nonzeroTracesInverse, 0, sizeof(nonzeroTracesInverse));

  numTilings = 0;
  minimumTrace = 0.01;
  numNonzeroTraces = 0;

  memset(numChoices, 0, sizeof(numChoices));
  memset(machineStateStr, 0, sizeof(machineStateStr));
  colTab.reset();
}

vector<int> SharedData::getLastJointChoice() const {
  return vector<int>(lastJointChoice, lastJointChoice + HierarchicalFSM::num_keepers);
}

vector<int> SharedData::getNumChoices() const {
  vector<int> ret((unsigned long) HierarchicalFSM::num_keepers);
  for (int i = 0; i < HierarchicalFSM::num_keepers; ++i) {
    ret[i] = numChoices[Memory::ins().K[i]];
  }
  return ret;
}

vector<string> SharedData::getMachineStateStr() const {
  vector<string> ret((unsigned long) HierarchicalFSM::num_keepers);
  for (int i = 0; i < HierarchicalFSM::num_keepers; ++i) {
    ret[i] = machineStateStr[Memory::ins().K[i]];
  }
  return ret;
}

void SharedData::clearBlocked() {
  Log.log(101, "SharedData::clearBlocked to 0");
  runningStatus = 0;
}

int SharedData::getRunningStatus() const {
  return runningStatus;
}

LinearSarsaLearner &LinearSarsaLearner::ins() {
  static LinearSarsaLearner learner;
  return learner;
}

LinearSarsaLearner::~LinearSarsaLearner() {
  shutDown();
}

LinearSarsaLearner::LinearSarsaLearner() {
  bLearning = false;
  qLearning = false;
  sharedData = 0;
  memset(semSignal, 0, sizeof(semSignal));
  semSync = 0;
  lastJointChoiceIdx = -1;
  lastJointChoiceTime = UnknownTime;
}

void LinearSarsaLearner::initialize(
    bool learning, double width[], double weight, bool QLearning,
    string loadWeightsFile,
    string saveWeightsFile_)
{
  bLearning = learning;
  bSaveWeights = bLearning && saveWeightsFile_.length() > 0;
  saveWeightsFile = saveWeightsFile_;
  qLearning = QLearning;

  for (int i = 0; i < HierarchicalFSM::num_features; i++) {
    tileWidths[i] = width[i];
  }

  initialWeight = weight;
  alpha = 0.125;
  lambda = 0.0;
  epsilon = 0.01;

  lastJointChoiceIdx = -1;
  lastJointChoiceTime = UnknownTime;

  srand((unsigned int) 0);
  srand48((unsigned int) 0);
  int tmp[2];
  float tmpf[2];

  GetTiles(tmp, 1, 1, tmpf, 0);  // A dummy call to set the hashing table
  srand((unsigned int) time(NULL));
  srand48((unsigned int) time(NULL));

  numTilings = 0;
  minimumTrace = 0.01;
  numNonzeroTraces = 0;

  if (bLearning || !bLearning) {
    string exepath = getexepath();
    exepath += "LinearSarsaLearner::initialize";
    exepath += to_string(HierarchicalFSM::gamma);
    exepath += to_string(initialWeight);
    exepath += to_string(qLearning);
    auto h = hash<string>()(exepath); //hashing
    sharedMemoryName = "/" + to_string(h) + ".shm";

    for (int i = 0; i < HierarchicalFSM::num_keepers; ++i) {
      if ((semSignal[i] = sem_open(("/signal-" + to_string(i) + "-" + to_string(h)).c_str(), O_CREAT, 0666, 0)) ==
          SEM_FAILED) {
        perror("semaphore initilization");
        exit(1);
      }
    }

    if ((semSync = sem_open(("/sync-" + to_string(h)).c_str(), O_CREAT, 0666, 1)) == SEM_FAILED) {
      perror("semaphore initilization");
      exit(1);
    }

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

    Q = sharedData->Q;
    weights = sharedData->weights;
    traces = sharedData->traces;
    nonzeroTraces = sharedData->nonzeroTraces;
    nonzeroTracesInverse = sharedData->nonzeroTracesInverse;
    colTab = &sharedData->colTab;

    {
      ScopedLock lock(semSync);
      sharedData->reset();
      fill(traces, traces + RL_MEMORY_SIZE, 0.0);
      fill(weights, weights + RL_MEMORY_SIZE, initialWeight);

      if (loadWeightsFile.length() > 0)
        loadWeights(loadWeightsFile.c_str());
    }
  }
}

void LinearSarsaLearner::shutDown()
{
  PRINT_VALUE(bLearning);
  PRINT_VALUE(bSaveWeights);

  if (Memory::ins().agentIdx == 0 && bLearning && bSaveWeights) {
    cerr << "Saving weights at shutdown." << endl;
    saveWeights(saveWeightsFile.c_str());
  }

  if (sharedData) shm_unlink(sharedMemoryName.c_str());
  for (auto s : semSignal) if (s) sem_close(s);
  if (semSync) sem_close(semSync);
}

bool LinearSarsaLearner::loadSharedData() {
  ScopedLock lock(semSync);
  numTilings = sharedData->numTilings;
  minimumTrace = sharedData->minimumTrace;
  numNonzeroTraces = sharedData->numNonzeroTraces;
  lastJointChoiceIdx = sharedData->lastJointChoiceIdx;
  lastJointChoiceTime = sharedData->lastJointChoiceTime;
  lastJointChoice = sharedData->getLastJointChoice();
  numChoices = sharedData->getNumChoices();
  machineStateStr = sharedData->getMachineStateStr();

  bool action_state = true; // action state (when all are in action)
  for (int i = 0; i < HierarchicalFSM::num_keepers; ++i) {
    if (numChoices[i] > 1) {
      action_state = false;
    }
  }

  if (Log.isInLogLevel(101)) {
    stringstream ss;
    PRINT_VALUE_STREAM(ss, Memory::ins().to_string());
    PRINT_VALUE_STREAM(ss, numTilings);
    PRINT_VALUE_STREAM(ss, minimumTrace);
    PRINT_VALUE_STREAM(ss, numNonzeroTraces);
    PRINT_VALUE_STREAM(ss, machineStateStr);
    PRINT_VALUE_STREAM(ss, numChoices);
    PRINT_VALUE_STREAM(ss, action_state);
    PRINT_VALUE_STREAM(ss, lastJointChoiceIdx);
    PRINT_VALUE_STREAM(ss, lastJointChoiceTime);
    PRINT_VALUE_STREAM(ss, lastJointChoice);

    Log.log(101, "LinearSarsaLearner::loadSharedData\n%s", ss.str().c_str());
  }

  return action_state;
}

void LinearSarsaLearner::wait() {
  int val = -1;
  sem_getvalue(semSignal[Memory::ins().agentIdx], &val);
  Log.log(101, "LinearSarsaLearner::wait agent %d wait to be notified, cur val: %d", Memory::ins().agentIdx, val);
  SemTimedWait(semSignal[Memory::ins().agentIdx]);
  Log.log(101, "LinearSarsaLearner::wait agent %d notified", Memory::ins().agentIdx);
}

void LinearSarsaLearner::notify(int i) {
  assert(i != Memory::ins().agentIdx);
  int val = -1;
  sem_getvalue(semSignal[i], &val);
  assert(val == 0);
  Log.log(101, "LinearSarsaLearner::notify agent %d notify %d, cur val: %d", Memory::ins().agentIdx, i, val);
  sem_post(semSignal[i]);
}

void LinearSarsaLearner::saveSharedData() {
  ScopedLock lock(semSync);

  assert(numChoices.size());
  assert(machineStateStr.size());
  assert(lastJointChoice.size());

  sharedData->numTilings = numTilings;
  sharedData->minimumTrace = minimumTrace;
  sharedData->numNonzeroTraces = numNonzeroTraces;
  sharedData->lastJointChoiceIdx = lastJointChoiceIdx;
  sharedData->lastJointChoiceTime = lastJointChoiceTime;

  for (int i = 0; i < HierarchicalFSM::num_keepers; ++i) {
    sharedData->numChoices[Memory::ins().K[i]] = numChoices[i];
    strcpy(sharedData->machineStateStr[Memory::ins().K[i]], machineStateStr[i].c_str());
    sharedData->lastJointChoice[i] = lastJointChoice[i];
  }
}

const vector<int> &LinearSarsaLearner::validChoices(const vector<int> &num_choices) {
  if (!validChoicesMap.count(num_choices)) {
    jointChoicesMap[num_choices] = validChoicesRaw(num_choices);
    for (uint i = 0; i < jointChoicesMap[num_choices].size(); ++i) {
      validChoicesMap[num_choices].push_back(i);
    }
  }
  return validChoicesMap[num_choices];
}

vector<vector<int>> LinearSarsaLearner::validChoicesRaw(const vector<int> &num_choices) {
  if (num_choices.empty()) return {{}};

  vector<vector<int>> ret;
  auto c = validChoicesRaw({num_choices.begin() + 1, num_choices.end()});
  for (int i = 0; i < num_choices[0]; ++i) {
    for (auto &v : c) {
      vector<int> r{i};
      r.insert(r.end(), v.begin(), v.end());
      ret.push_back(r);
    }
  }

  return ret;
}

double LinearSarsaLearner::reward(double tau) {
  Log.log(101, "LinearSarsaLearner::reward tau=%f", tau);
  double gamma = HierarchicalFSM::gamma;
  double ret = tau;

  if (gamma < 1.0) {
    ret = (1.0 - pow(gamma, tau)) / (1.0 - gamma);
  }

  return ret;
}

int LinearSarsaLearner::step(int current_time) {
  ScopedLock lock(semSync);

  int choice = -1;
  if (lastJointChoiceIdx >= 0) {
    assert(lastJointChoiceTime != UnknownTime);
    assert(lastJointChoiceTime <= current_time);
    double tau = current_time - lastJointChoiceTime;
    double delta = reward(tau) - Q[lastJointChoiceIdx];
    loadTiles(Memory::ins().state, machineStateStr, numChoices);
    for (auto c : validChoices(numChoices)) {
      Q[c] = computeQ(c);
    }
    choice = selectChoice(numChoices);

    if (!bLearning) return choice;
    assert(!std::isnan(Q[choice]) && !std::isinf(Q[choice]));

    if (qLearning) {
      delta += pow(HierarchicalFSM::gamma, tau) * Q[argmaxQ(numChoices)];
    }
    else {
      delta += pow(HierarchicalFSM::gamma, tau) * Q[choice];
    }

    updateWeights(delta);
    Q[choice] = computeQ(choice); // update Q[choice]
    decayTraces(HierarchicalFSM::gamma * lambda);

    for (auto a : validChoices(numChoices)) {
      if (a != choice) {
        for (int j = 0; j < numTilings; j++)
          clearTrace(tiles[a][j]);
      }
    }
  } else { // new episode
    decayTraces(0.0);
    assert(numNonzeroTraces == 0);
    loadTiles(Memory::ins().state, machineStateStr, numChoices);
    for (auto c : validChoices(numChoices)) {
      Q[c] = computeQ(c);
    }
    choice = selectChoice(numChoices);
  }

  for (int j = 0; j < numTilings; j++)
    setTrace(tiles[choice][j], 1.0);

  return choice;
}

/**
 * num_choices for agentIdx
 * note: take care of sync among processes
 * @param num_choices
 * @return
 */
int LinearSarsaLearner::step(int current_time, int num_choices) {
  bool last_blocking_agent = false;
  {
    ScopedLock lock(semSync);
    auto stackStr = HierarchicalFSM::getStackStr();
    sharedData->numChoices[Memory::ins().K[Memory::ins().agentIdx]] = num_choices;
    strcpy(sharedData->machineStateStr[Memory::ins().K[Memory::ins().agentIdx]], stackStr.c_str());

    Log.log(101, "LinearSarsaLearner::step agent %d write numChoices %d", Memory::ins().agentIdx, num_choices);
    Log.log(101, "LinearSarsaLearner::step agent %d write machineStateStr %s", Memory::ins().agentIdx,
            stackStr.c_str());

    sharedData->setBlocked(Memory::ins().agentIdx);
    last_blocking_agent = sharedData->isAllBlocked(HierarchicalFSM::num_keepers);
  }

  if (last_blocking_agent) { // leading agent
    Log.log(101, "LinearSarsaLearner::step leading agent %d (running status: %d)", Memory::ins().agentIdx,
            sharedData->getRunningStatus());
    bool action_state = loadSharedData();

    if (action_state) { // no q update
      ScopedLock lock(semSync);
      sharedData->clearBlocked(); // reset to 0

      for (int i = 0; i < HierarchicalFSM::num_keepers; ++i) {
        if (i != Memory::ins().agentIdx) {
          notify(i);
        }
      }
    } else {
      lastJointChoiceIdx = step(current_time);
      lastJointChoiceTime = current_time;
      assert(lastJointChoiceIdx >= 0);
      lastJointChoice = jointChoicesMap[numChoices][lastJointChoiceIdx];
      saveSharedData();

      {
        ScopedLock lock(semSync);
        for (int i = 0; i < HierarchicalFSM::num_keepers; ++i) {
          if (numChoices[i] > 1) {
            sharedData->clearBlocked(i);
          }
        }

        for (int i = 0; i < HierarchicalFSM::num_keepers; ++i) {
          if (numChoices[i] > 1 && i != Memory::ins().agentIdx) { // not blocked in action state
            notify(i);
          }
        }
      }

      if (numChoices[Memory::ins().agentIdx] <= 1) {
        wait(); // block self
      }
    }
  } else {
    wait();
  }

//  assert(!sharedData->isBlocked(agentIdx));
  if (sharedData->isBlocked(Memory::ins().agentIdx)) {
    PRINT_VALUE(sharedData->isBlocked(Memory::ins().agentIdx));
    Log.log(101, "sharedData->isBlocked(agentIdx)");
  }

  if (sharedData->isBlocked(Memory::ins().agentIdx)) sharedData->clearBlocked(Memory::ins().agentIdx);
  bool action_state = loadSharedData();
  if (action_state) { // dummy choice
    return 0;
  } else {
    return lastJointChoice[Memory::ins().agentIdx];
  }
}

void LinearSarsaLearner::endEpisode(int current_time) {
  Log.log(101, "LinearSarsaLearner::endEpisode");

  if (Memory::ins().agentIdx == 0) { // only one agent can update
    loadSharedData();

    if (bLearning && lastJointChoiceIdx >= 0) {
      ScopedLock lock(semSync);
      assert(lastJointChoiceTime != UnknownTime);
      assert(lastJointChoiceTime <= current_time);
      double tau = current_time - lastJointChoiceTime;
      double delta = reward(tau) - Q[lastJointChoiceIdx];
      updateWeights(delta);
    }

    lastJointChoiceIdx = -1;
    lastJointChoiceTime = UnknownTime;
    saveSharedData();

    if (bLearning && bSaveWeights && rand() % 1000 == 0) {
      saveWeights(saveWeightsFile.c_str());
    }
  } else {
    usleep(5000);
  }

  loadSharedData();
  machineStateStr.clear();
  numChoices.clear();
  lastJointChoiceIdx = -1;
  lastJointChoiceTime = UnknownTime;
  lastJointChoice.clear();
  if (sharedData->isBlocked(Memory::ins().agentIdx)) sharedData->clearBlocked(Memory::ins().agentIdx);
  sem_init(semSignal[Memory::ins().agentIdx], PTHREAD_PROCESS_SHARED, 0);
}

void LinearSarsaLearner::loadTiles(
    double state[], const vector<string> &machineState, const vector<int> &num_choices) {
  const int tilingsPerGroup = 32;

  string str = "";
  for (auto &s : machineState) str += s + ", ";
  int h = (int) (hash<string>()(str) % INT_MAX); // joint machine state
  Log.log(101, "LinearSarsaLearner::loadTiles machine state: [%s], #%d, (%d)",
          str.c_str(), validChoices(num_choices).size(), h);

  numTilings = 0;
  for (int v = 0; v < HierarchicalFSM::num_features; v++) {
    for (auto a: validChoices(num_choices)) {
      GetTiles1(&(tiles[a][numTilings]), tilingsPerGroup, colTab,
                (float) (state[v] / tileWidths[v]), a, v, h);
    }
    numTilings += tilingsPerGroup;
  }

  assert(numTilings > 0);
  assert(numTilings < RL_MAX_NUM_TILINGS);
}

double LinearSarsaLearner::computeQ(int choice) {
  double q = 0;
  for (int j = 0; j < numTilings; j++) {
    q += weights[tiles[choice][j]];
  }

  return q;
}

int LinearSarsaLearner::selectChoice(const vector<int> &num_choices) {
  int choice = -1;

  if (Log.isInLogLevel(101)) {
    stringstream ss;
    PRINT_VALUE_STREAM(ss, validChoices(num_choices));
    PRINT_VALUE_STREAM(ss, validChoices(num_choices).size());
    PRINT_VALUE_STREAM(ss, jointChoicesMap[num_choices]);
    PRINT_VALUE_STREAM(ss, vector<double>(Q, Q + validChoices(num_choices).size()));
    Log.log(101, "LinearSarsaLearner::selectChoice %s", ss.str().c_str());
    Log.log(101, "LinearSarsaLearner::selectChoice numTilings: %d", numTilings);
  }

  if (bLearning && drand48() < epsilon) {     /* explore */
    auto &choices = validChoices(num_choices);
    choice = choices[rand() % choices.size()];
    Log.log(101, "LinearSarsaLearner::selectChoice explore choice %d", choice);
  } else {
    choice = argmaxQ(num_choices);
    Log.log(101, "LinearSarsaLearner::selectChoice argmaxQ choice %d", choice);
  }

  return choice;
}

int LinearSarsaLearner::argmaxQ(const vector<int> &num_choices) {
  int bestAction = -1;
  double bestValue = INT_MIN;
  int numTies = 0;

  for (auto a : validChoices(num_choices)) {
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

  assert(bestAction >= 0);
  return bestAction;
}

void LinearSarsaLearner::updateWeights(double delta) {
  Log.log(101, "LinearSarsaLearner::updateWeights delta %f", delta);

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

void LinearSarsaLearner::decayTraces(double decayRate) {
  for (int loc = numNonzeroTraces - 1; loc >= 0; loc--) {
    int f = nonzeroTraces[loc];
    if (f >= RL_MEMORY_SIZE || f < 0) {
      assert(0);
      cerr << "DecayTraces: f out of range " << f << endl;
      continue;
    }

    traces[f] *= decayRate;
    if (traces[f] < minimumTrace)
      clearExistentTrace(f, loc);
  }
}

void LinearSarsaLearner::clearTrace(int f) {
  if (f >= RL_MEMORY_SIZE || f < 0) {
    assert(0);
    cerr << "ClearTrace: f out of range " << f << endl;
    return;
  }

  if (traces[f] != 0)
    clearExistentTrace(f, nonzeroTracesInverse[f]);
}

void LinearSarsaLearner::clearExistentTrace(int f, int loc) {
  if (f >= RL_MEMORY_SIZE || f < 0) {
    assert(0);
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

void LinearSarsaLearner::setTrace(int f, float newTraceValue) {
  if (f >= RL_MEMORY_SIZE || f < 0) {
    assert(0);
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

void LinearSarsaLearner::increaseMinTrace() {
  minimumTrace *= 1.1;
  cerr << "Changing minimum_trace to " << minimumTrace << endl;
  for (int loc = numNonzeroTraces - 1; loc >= 0; loc--) { // necessary to loop downwards
    int f = nonzeroTraces[loc];
    if (traces[f] < minimumTrace)
      clearExistentTrace(f, loc);
  }
}

bool LinearSarsaLearner::loadWeights(const char *filename) {
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

bool LinearSarsaLearner::saveWeights(const char *filename) {
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

}
