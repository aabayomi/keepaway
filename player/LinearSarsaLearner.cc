//
// Created by baj on 10/4/16.
//

#include <cassert>
#include <climits>
#include <sstream>
#include "LinearSarsaLearner.h"

namespace fsm {

void SharedData::incNumBlocked() {
  Log.logWithTime(101, "SharedData::incNumBlocked from %d to %d", numBlocked, numBlocked + 1);
  numBlocked += 1;
  assert(numBlocked <= HierarchicalFSM::num_keepers);
}

void SharedData::decNumBlocked() {
  Log.logWithTime(101, "SharedData::decNumBlocked from %d to %d", numBlocked, numBlocked - 1);
  numBlocked -= 1;
  assert(numBlocked >= 0);
}


void SharedData::resetReward() {
  cumulativeReward = 0.0;
  cumulativeGamma = HierarchicalFSM::gamma;
}

void SharedData::reset() {
  numBlocked = 0;
  lastJointChoiceIdx = -1;
  memset(lastJointChoice, 0, sizeof(lastJointChoice));

  memset(Q, 0, sizeof(Q));
  memset(weights, 0, sizeof(weights));
  memset(traces, 0, sizeof(traces));
  memset(nonzeroTraces, 0, sizeof(nonzeroTraces));
  memset(nonzeroTracesInverse, 0, sizeof(nonzeroTracesInverse));

  numTilings = 0;
  minimumTrace = 0.01;
  numNonzeroTraces = 0;

  fill(numChoices, numChoices + 11, 0);
  colTab.reset();
}

int SharedData::getNumBlocked() const {
  return numBlocked;
}

vector<int> SharedData::getLastJointChoice() const {
  return vector<int>(lastJointChoice, lastJointChoice + HierarchicalFSM::num_keepers);
}

vector<int> SharedData::getNumChoices() const {
  return vector<int>(numChoices, numChoices + HierarchicalFSM::num_keepers);
}

vector<string> SharedData::getMachineStateStr() const {
  return vector<string>(machineStateStr, machineStateStr + HierarchicalFSM::num_keepers);
}

void SharedData::clearNumBlocked() {
  Log.logWithTime(101, "SharedData::clearNumBlocked to 0");
  numBlocked = 0;
}

double SharedData::getCumulativeGamma() const {
  return cumulativeGamma;
}

double SharedData::getCumulativeReward() const {
  return cumulativeReward;
}

LinearSarsaLearner &LinearSarsaLearner::ins() {
  static LinearSarsaLearner learner;
  return learner;
}

LinearSarsaLearner::~LinearSarsaLearner() {
  if (sharedData) shm_unlink(sharedMemoryName.c_str());
  for (auto s : semSignal) if (s) sem_close(s);
}

LinearSarsaLearner::LinearSarsaLearner() {
  sharedData = 0;
  memset(semSignal, 0, sizeof(semSignal));
  semSync = 0;
  newEpisode = true;
}

void LinearSarsaLearner::initialize(bool learning, double width[], double weight)
{
  bLearning = learning;
  for (int i = 0; i < HierarchicalFSM::num_features; i++) {
    tileWidths[i] = width[i];
  }

  initialWeight = weight;
  alpha = 0.125;
  lambda = 0.0;
  epsilon = 0.01;

  lastJointChoiceIdx = -1;

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

  if (bLearning) {
    string exepath = getexepath();
    exepath += to_string(HierarchicalFSM::gamma);
    exepath += to_string(initialWeight);
    auto h = hash<string>().operator()(exepath); //hashing
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

    {
      ScopedLock lock(semSync);
      sharedData->reset();
    }

    Q = sharedData->Q;
    weights = sharedData->weights;
    traces = sharedData->traces;
    nonzeroTraces = sharedData->nonzeroTraces;
    nonzeroTracesInverse = sharedData->nonzeroTracesInverse;
    colTab = &sharedData->colTab;

    fill(traces, traces + RL_MEMORY_SIZE, 0.0);
    fill(weights, weights + RL_MEMORY_SIZE, initialWeight);
  }
}

void LinearSarsaLearner::loadSharedData() {
  ScopedLock lock(semSync);
  numTilings = sharedData->numTilings;
  minimumTrace = sharedData->minimumTrace;
  numNonzeroTraces = sharedData->numNonzeroTraces;
  lastJointChoiceIdx = sharedData->lastJointChoiceIdx;
  lastJointChoice = sharedData->getLastJointChoice();
  numChoices = sharedData->getNumChoices();
  machineStateStr = sharedData->getMachineStateStr();

  if (Log.isInLogLevel(101)) {
    stringstream ss;
    PRINT_VALUE_STREAM(ss, numTilings);
    PRINT_VALUE_STREAM(ss, minimumTrace);
    PRINT_VALUE_STREAM(ss, numNonzeroTraces);
    PRINT_VALUE_STREAM(ss, machineStateStr);
    PRINT_VALUE_STREAM(ss, numChoices);
    PRINT_VALUE_STREAM(ss, lastJointChoiceIdx);
    PRINT_VALUE_STREAM(ss, lastJointChoice);
    Log.logWithTime(101, "LinearSarsaLearner::loadSharedData\n%s", ss.str().c_str());
  }
}

void LinearSarsaLearner::wait() {
  int val = -1;
  sem_getvalue(semSignal[agentIdx], &val);
  Log.logWithTime(101, "LinearSarsaLearner::wait agent %d wait to be notified, cur val: %d", agentIdx, val);
  sem_wait(semSignal[agentIdx]);
  Log.logWithTime(101, "LinearSarsaLearner::wait agent %d notified", agentIdx);
}

void LinearSarsaLearner::notify(int i) {
  assert(i != agentIdx);
  int val = -1;
  sem_getvalue(semSignal[i], &val);
  assert(val == 0);
  Log.logWithTime(101, "LinearSarsaLearner::notify agent %d notify %d, cur val: %d", agentIdx, i, val);
  sem_post(semSignal[i]);
}

void LinearSarsaLearner::saveSharedData() {
  ScopedLock lock(semSync);
  sharedData->numTilings = numTilings;
  sharedData->minimumTrace = minimumTrace;
  sharedData->numNonzeroTraces = numNonzeroTraces;
  sharedData->lastJointChoiceIdx = lastJointChoiceIdx;

  for (int i = 0; i < HierarchicalFSM::num_keepers; ++i) {
    sharedData->numChoices[i] = numChoices.at(i);
    strcpy(sharedData->machineStateStr[i], machineStateStr.at(i).c_str());
    sharedData->lastJointChoice[i] = lastJointChoice.at(i);
  }
}

void LinearSarsaLearner::loadQ(const vector<int> &num_choices) {
  loadTiles(Memory::ins().state, machineStateStr, num_choices);
  Log.logWithTime(101, "LinearSarsaLearner::loadQ numTilings: %d", numTilings);

  for (auto &jc : validChoices(num_choices)) {
    Q[jc] = computeQ(jc);
  }
}

const vector<int> &LinearSarsaLearner::validChoices(const vector<int> &num_choices) {
  if (!validChoicesMap.count(num_choices)) {
    auto choices = validChoicesRaw(num_choices);
    for (uint i = 0; i < choices.size(); ++i) {
      validChoicesMap[num_choices].push_back(i);
      jointChoicesMap[num_choices][i] = choices[i];
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

int LinearSarsaLearner::step() {
  ScopedLock lock(semSync);

  int choice = -1;
  if (Log.isInLogLevel(101)) {
    stringstream ss;
    Log.logWithTime(101, "LinearSarsaLearner::step new episode %d", newEpisode);
    PRINT_VALUE_STREAM(ss, Memory::ins().to_string());
    PRINT_VALUE_STREAM(ss, machineStateStr);
    PRINT_VALUE_STREAM(ss, numChoices);
    PRINT_VALUE_STREAM(ss, lastJointChoiceIdx);
    PRINT_VALUE_STREAM(ss, lastJointChoice);
    PRINT_VALUE_STREAM(ss, sharedData->getCumulativeReward());
    PRINT_VALUE_STREAM(ss, sharedData->getCumulativeGamma());
    Log.logWithTime(101, "LinearSarsaLearner::step memory (before update):\n%s", ss.str().c_str());
  }

  if (!newEpisode) {
    double delta = sharedData->getCumulativeReward() - Q[lastJointChoiceIdx];
    loadQ(numChoices);
    choice = selectChoice(numChoices);

    if (!bLearning) return choice;
    assert(!std::isnan(Q[choice]) && !std::isinf(Q[choice]));
    delta += sharedData->getCumulativeGamma() * Q[choice];

    updateWeights(delta);
    Q[choice] = computeQ(choice); // update Q[choice]
    decayTraces(HierarchicalFSM::gamma * lambda);

    for (auto &a : validChoices(numChoices)) {
      if (a != choice) {
        for (int j = 0; j < numTilings; j++)
          clearTrace(tiles[a][j]);
      }
    }
  } else { // new episode
    decayTraces(0.0);
    assert(numNonzeroTraces == 0);
    loadQ(numChoices);
    choice = selectChoice(numChoices);
    newEpisode = false;
  }

  for (int j = 0; j < numTilings; j++)
    setTrace(tiles[choice][j], 1.0);

  if (Log.isInLogLevel(101)) {
    Log.logWithTime(101, "LinearSarsaLearner::step saved numNonzeroTraces: %d", numNonzeroTraces);
    stringstream ss;
    PRINT_VALUE_STREAM(ss, choice);
    PRINT_VALUE_STREAM(ss, jointChoicesMap[numChoices][choice]);
    Log.logWithTime(101, "LinearSarsaLearner::step choice made\n %s [%d]", ss.str().c_str(), agentIdx);
  }

  return choice;
}

/**
 * num_choices for agentIdx
 * note: take care of sync among processes
 * @param num_choices
 * @return
 */
vector<int> LinearSarsaLearner::step(int num_choices) {
  bool last_blocking_agent = false;
  {
    ScopedLock lock(semSync);
    auto stackStr = HierarchicalFSM::getStackStr();
    sharedData->numChoices[agentIdx] = num_choices;
    strcpy(sharedData->machineStateStr[agentIdx], stackStr.c_str());

    Log.logWithTime(101, "LinearSarsaLearner::step agent %d numChoices %d", agentIdx, num_choices);
    Log.logWithTime(101, "LinearSarsaLearner::step agent %d machineState %s", agentIdx, stackStr.c_str());
    Log.logWithTime(101, "LinearSarsaLearner::step agent %d stack %s", agentIdx, stackStr.c_str());

    sharedData->incNumBlocked();
    last_blocking_agent = sharedData->getNumBlocked() == HierarchicalFSM::num_keepers;
  }

  if (last_blocking_agent || !bLearning) {
    Log.logWithTime(101, "LinearSarsaLearner::step leading agent %d", agentIdx);

    loadSharedData();

    bool action_state = true; // action state (when all are in action)
    for (int i = 0; i < HierarchicalFSM::num_keepers; ++i) {
      if (numChoices.at(i) > 1) {
        action_state = false;
      }
    }
    Log.logWithTime(101, "LinearSarsaLearner::step action state %d", action_state);

    if (action_state) {
      sharedData->cumulativeReward += sharedData->cumulativeGamma * 1.0;
      sharedData->cumulativeGamma *= HierarchicalFSM::gamma;

      sharedData->clearNumBlocked(); // reset to 0
      for (int i = 0; i < HierarchicalFSM::num_keepers; ++i) {
        if (i != agentIdx) {
          notify(i);
        }
      }
    } else {
      lastJointChoiceIdx = step();
      lastJointChoice = jointChoicesMap[numChoices][lastJointChoiceIdx];
      saveSharedData();
      sharedData->resetReward();

      for (int i = 0; i < HierarchicalFSM::num_keepers; ++i) {
        if (numChoices.at(i) > 1) {
          sharedData->decNumBlocked();
        }
      }

      for (int i = 0; i < HierarchicalFSM::num_keepers; ++i) {
        if (numChoices.at(i) > 1 && i != agentIdx) { // not blocked in action state
          notify(i);
        }
      }

      if (numChoices.at(agentIdx) <= 1) {
        wait(); // block self
      }
    }
  } else {
    wait();
  }

  loadSharedData();
  return lastJointChoice;
}

void LinearSarsaLearner::endEpisode() {
  if (Log.isInLogLevel(101)) {
    Log.logWithTime(101, "LinearSarsaLearner::endEpisode memory: \n%s",
                    Memory::ins().to_string().c_str());
    stringstream ss;
    PRINT_VALUE_STREAM(ss, machineStateStr);
    PRINT_VALUE_STREAM(ss, numChoices);
    PRINT_VALUE_STREAM(ss, lastJointChoiceIdx);
    PRINT_VALUE_STREAM(ss, lastJointChoice);
    PRINT_VALUE_STREAM(ss, sharedData->getCumulativeReward());
    PRINT_VALUE_STREAM(ss, sharedData->getCumulativeGamma());
    Log.logWithTime(101, "LinearSarsaLearner::endEpisode machine state/num choices/joint choice\n%s", ss.str().c_str());
  }

  sem_init(semSignal[agentIdx], PTHREAD_PROCESS_SHARED, 0);
  if (agentIdx == 0) { // only one agent can update
    if (bLearning && lastJointChoiceIdx >= 0) {
      double delta = sharedData->getCumulativeReward() - Q[lastJointChoiceIdx];
      updateWeights(delta);
    }
    sharedData->resetReward();
  }

  machineStateStr.clear();
  numChoices.clear();
  lastJointChoiceIdx = -1;
  lastJointChoice.clear();

  newEpisode = true;
}

void LinearSarsaLearner::loadTiles(
    double state[], const vector<string> &machineState, const vector<int> &num_choices) {
  const int tilingsPerGroup = 32;

  string str = "";
  for (auto &s : machineState) str += s;
  int h = (int) hash<string>()(str) % INT_MAX; // joint machine state

  numTilings = 0;
  for (int v = 0; v < HierarchicalFSM::num_features; v++) {
    for (auto &jc : validChoices(num_choices)) {
      GetTiles1(&(tiles[jc][numTilings]), tilingsPerGroup, colTab,
                (float) (state[v] / tileWidths[v]), (int) jc, v, h);
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
  auto vc = validChoices(num_choices);
  int choice = -1;

  if (Log.isInLogLevel(101)) {
    stringstream ss;
    PRINT_VALUE_STREAM(ss, num_choices);
    Log.logWithTime(101, "num_choices/validChoices\n %s", ss.str().c_str());
  }

  if (bLearning && drand48() < epsilon) {     /* explore */
    choice = vc[rand() % vc.size()];
  } else {
    choice = argmaxQ(num_choices);
  }

  return choice;
}

int LinearSarsaLearner::argmaxQ(const vector<int> &num_choices) {
  int bestAction = -1;
  double bestValue = INT_MIN;
  int numTies = 0;

  for (auto &a : validChoices(num_choices)) {
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
  Log.logWithTime(101, "LinearSarsaLearner::updateWeights delta %f", delta);

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
    cerr << "ClearTrace: f out of range " << f << endl;
    return;
  }

  if (traces[f] != 0)
    clearExistentTrace(f, nonzeroTracesInverse[f]);
}

void LinearSarsaLearner::clearExistentTrace(int f, int loc) {
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

void LinearSarsaLearner::setTrace(int f, float newTraceValue) {
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

void LinearSarsaLearner::increaseMinTrace() {
  minimumTrace *= 1.1;
  cerr << "Changing minimum_trace to " << minimumTrace << endl;
  for (int loc = numNonzeroTraces - 1; loc >= 0; loc--) { // necessary to loop downwards
    int f = nonzeroTraces[loc];
    if (traces[f] < minimumTrace)
      clearExistentTrace(f, loc);
  }
}

string LinearSarsaLearner::getQStr(int num_choice) {
  stringstream ss;
//  ss << vector<double>(Q, Q + num_choice);
  return ss.str();
}

}
