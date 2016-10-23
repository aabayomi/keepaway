//
// Created by baj on 10/4/16.
//

#include <climits>
#include <sstream>
#include "LinearSarsaLearner.h"

namespace fsm {

void SharedData::reset() {
  lastJointChoiceIdx = -1;
  lastJointChoiceTime = UnknownTime;
  memset(lastJointChoice, 0, sizeof(lastJointChoice));

  memset(Q, 0, sizeof(Q));
  memset(traces, 0, sizeof(traces));
  memset(nonzeroTraces, 0, sizeof(nonzeroTraces));
  memset(nonzeroTracesInverse, 0, sizeof(nonzeroTracesInverse));

  minimumTrace = 0.01;
  numNonzeroTraces = 0;

  memset(numChoices, 0, sizeof(numChoices));
  memset(machineState, 0, sizeof(machineState));
  memset(lastMachineState, 0, sizeof(lastMachineState));
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

vector<string> SharedData::getMachineState() const {
  vector<string> ret((unsigned long) HierarchicalFSM::num_keepers);
  for (int i = 0; i < HierarchicalFSM::num_keepers; ++i) {
    ret[i] = machineState[Memory::ins().K[i]];
  }
  return ret;
}

vector<string> SharedData::getLastMachineState() const {
  vector<string> ret((unsigned long) HierarchicalFSM::num_keepers);
  for (int i = 0; i < HierarchicalFSM::num_keepers; ++i) {
    ret[i] = lastMachineState[i];
  }
  return ret;
}

LinearSarsaLearner &LinearSarsaLearner::ins() {
  static LinearSarsaLearner learner;
  return learner;
}

LinearSarsaLearner::~LinearSarsaLearner() {
  shutDown();
}

LinearSarsaLearner::LinearSarsaLearner() {
  initialWeight = 0.0;
  alpha = 0.125;
  gamma = 1.0;
  lambda = 0.0;
  epsilon = 0.01;
  bLearning = false;
  qLearning = false;
  sharedData = 0;
  barrier = 0;
  lastJointChoiceIdx = -1;
  lastJointChoiceTime = UnknownTime;
}

void LinearSarsaLearner::initialize(
    bool learning, double width[],
    double Gamma, double Lambda,
    double weight, bool QLearning,
    string loadWeightsFile,
    string saveWeightsFile_) {
  bLearning = learning;
  bSaveWeights = bLearning && saveWeightsFile_.length() > 0;
  saveWeightsFile = saveWeightsFile_;
  qLearning = QLearning;

  for (int i = 0; i < HierarchicalFSM::num_features; i++) {
    tileWidths[i] = width[i];
  }

  initialWeight = weight;
  alpha = 0.125;
  gamma = Gamma;
  lambda = Lambda;
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
    exepath += to_string(gamma);
    exepath += to_string(lambda);
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

    Q = sharedData->Q;
    tiles_ = sharedData->tiles;
    weights = sharedData->weights;
    traces = sharedData->traces;
    nonzeroTraces = sharedData->nonzeroTraces;
    nonzeroTracesInverse = sharedData->nonzeroTracesInverse;
    colTab = &sharedData->colTab;

    barrier = new Barrier(HierarchicalFSM::num_keepers, h);

    sharedData->reset();
    if (loadWeightsFile.empty() || !loadWeights(loadWeightsFile.c_str())) {
      fill(weights, weights + RL_MEMORY_SIZE, initialWeight);
      colTab->reset();
    }

    loadMachineTransitions("transitions.xml");
  }
}

void LinearSarsaLearner::shutDown() {
  if (Memory::ins().agentIdx == 0 && bLearning && bSaveWeights) {
    cerr << "Saving weights at shutdown." << endl;
    saveWeights(saveWeightsFile.c_str());
  }

  if (sharedData) shm_unlink(sharedMemoryName.c_str());
  delete barrier;

  if (Memory::ins().agentIdx == 0) {
    saveMachineTransitions("transitions.xml");

#if DETERMINISTIC_GRAPH
    detTransitionGraph.dump("detTransitionGraph_" + to_string(Memory::ins().agentIdx) + ".dot");
#endif
  }
}

bool LinearSarsaLearner::loadSharedData() {
  numTilings = sharedData->numTilings;
  minimumTrace = sharedData->minimumTrace;
  numNonzeroTraces = sharedData->numNonzeroTraces;
  lastJointChoiceIdx = sharedData->lastJointChoiceIdx;
  lastJointChoiceTime = sharedData->lastJointChoiceTime;
  lastJointChoice = sharedData->getLastJointChoice();
  machineState = sharedData->getMachineState();
  lastMachineState = sharedData->getLastMachineState();
  numChoices = sharedData->getNumChoices();

  Assert(!numChoicesMap.count(machineState) || numChoicesMap[machineState] == numChoices);
  numChoicesMap[machineState] = numChoices;

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
    PRINT_VALUE_STREAM(ss, machineState);
    PRINT_VALUE_STREAM(ss, lastMachineState);
    PRINT_VALUE_STREAM(ss, numChoices);
    PRINT_VALUE_STREAM(ss, action_state);
    PRINT_VALUE_STREAM(ss, lastJointChoiceIdx);
    PRINT_VALUE_STREAM(ss, lastJointChoiceTime);
    PRINT_VALUE_STREAM(ss, lastJointChoice);

    Log.log(101, "LinearSarsaLearner::loadSharedData\n%s", ss.str().c_str());
  }

  return action_state;
}

void LinearSarsaLearner::saveSharedData() {
  Assert(numChoices.size());
  Assert(machineState.size());
  Assert(lastJointChoice.size());

  sharedData->numTilings = numTilings;
  sharedData->minimumTrace = minimumTrace;
  sharedData->numNonzeroTraces = numNonzeroTraces;
  sharedData->lastJointChoiceIdx = lastJointChoiceIdx;
  sharedData->lastJointChoiceTime = lastJointChoiceTime;

  for (int i = 0; i < HierarchicalFSM::num_keepers; ++i) {
    sharedData->numChoices[Memory::ins().K[i]] = numChoices[i];
    strcpy(sharedData->machineState[Memory::ins().K[i]], machineState[i].c_str());
    strcpy(sharedData->lastMachineState[i], lastMachineState[i].c_str());
    sharedData->lastJointChoice[i] = lastJointChoice[i];
  }
}

const vector<int> &LinearSarsaLearner::validChoices(const vector<int> &num_choices) {
  Assert(num_choices.size() == HierarchicalFSM::num_keepers);
  if (!validChoicesMap.count(num_choices) || !jointChoicesMap.count(num_choices)) {
    jointChoicesMap[num_choices] = validChoicesRaw(num_choices);
    for (uint i = 0; i < jointChoicesMap[num_choices].size(); ++i) {
      Assert(jointChoicesMap[num_choices][i].size() == HierarchicalFSM::num_keepers);
      validChoicesMap[num_choices].push_back(i);
    }
  }
  return validChoicesMap[num_choices];
}

void LinearSarsaLearner::saveMachineTransitions(const char *filename) {
  std::ofstream ofs(filename);
  Assert(ofs.good());
  boost::archive::xml_oarchive oa(ofs);
  oa << BOOST_SERIALIZATION_NVP(validChoicesMap);
  oa << BOOST_SERIALIZATION_NVP(jointChoicesMap);
  oa << BOOST_SERIALIZATION_NVP(numChoicesMap);
  oa << BOOST_SERIALIZATION_NVP(detTransitionMap);
}

void LinearSarsaLearner::loadMachineTransitions(const char *filename) {
  std::ifstream ifs(filename);
  if (!ifs.good()) return;
  boost::archive::xml_iarchive ia(ifs);
  ia >> BOOST_SERIALIZATION_NVP(validChoicesMap);
  ia >> BOOST_SERIALIZATION_NVP(jointChoicesMap);
  ia >> BOOST_SERIALIZATION_NVP(numChoicesMap);
  ia >> BOOST_SERIALIZATION_NVP(detTransitionMap);
}

vector<vector<int>> LinearSarsaLearner::validChoicesRaw(const vector<int> &num_choices) {
  if (num_choices.empty()) return {{}};

  vector<vector<int>> ret;
  auto c = validChoicesRaw({num_choices.begin() + 1, num_choices.end()});
  Assert(num_choices[0] > 0);
  for (int i = 0; i < num_choices[0]; ++i) {
    for (auto &v : c) {
      vector<int> r{i};
      r.insert(r.end(), v.begin(), v.end());
      ret.push_back(r);
    }
  }

  return ret;
}

bool LinearSarsaLearner::isDeterministic(const vector<string> &machine_state, int c) {
  return detTransitionMap.count(machine_state) && detTransitionMap[machine_state].count(c);
}

namespace {
bool dfs(
    const vector<string> &root,
    unordered_map<vector<string>, unordered_map<int, vector<string>>> &G,
    unordered_set<vector<string>> &visited,
    unordered_set<vector<string>> &path) {
  if (path.count(root)) {
    return true;
  }

  visited.insert(root);
  path.insert(root);
  if (G.count(root)) {
    for (auto &pa: G[root]) {
      if (dfs(pa.second, G, visited, path)) return true;
    }
  }
  return false;
};
}

bool LinearSarsaLearner::hasCircle(unordered_map<vector<string>, unordered_map<int, vector<string>>> &G) {
  unordered_set<vector<string>> visited;
  for (auto &pa : G) {
    unordered_set<vector<string>> path;
    if (!visited.count(pa.first) && dfs(pa.first, G, visited, path)) return true;
  }
  return false;
}

double LinearSarsaLearner::reward(double tau) {
  Log.log(101, "LinearSarsaLearner::reward tau=%f", tau);
  double ret = tau;

  if (gamma < 1.0) {
    ret = (1.0 - pow(gamma, tau)) / (1.0 - gamma);
  }

  return ret;
}

int LinearSarsaLearner::step(int current_time) {
  int choice = -1;
  auto *state = Memory::ins().state;

  if (lastJointChoiceIdx >= 0) {
    Assert(lastJointChoiceTime != UnknownTime);
    double tau = current_time - lastJointChoiceTime;
    double delta = reward(tau) - Q[lastJointChoiceIdx];
    numTilings = loadTiles(state, machineState, numChoices, tiles_);
    for (auto c : validChoices(numChoices)) {
      Q[c] = QValue(state, machineState, c, tiles_, numTilings);
    }

    choice = selectChoice(numChoices);
    if (!bLearning) return choice;
    Assert(!std::isnan(Q[choice]) && !std::isinf(Q[choice]));

    if (qLearning) {
      delta += pow(gamma, tau) * Q[argmaxQ(numChoices)];
    } else {
      delta += pow(gamma, tau) * Q[choice];
    }

    updateWeights(delta, numTilings);
    Q[choice] = QValue(state, machineState, choice, tiles_, numTilings);
    decayTraces(gamma * lambda);
    for (auto a : validChoices(numChoices)) {
      if (a != choice) {
        for (int j = 0; j < numTilings; j++)
          clearTrace(tiles_[a][j]);
      }
    }
  } else { // new episode
    decayTraces(0.0);
    Assert(numNonzeroTraces == 0);
    numTilings = loadTiles(state, machineState, numChoices, tiles_);
    for (auto c : validChoices(numChoices)) {
      Q[c] = QValue(state, machineState, c, tiles_, numTilings);
    }
    choice = selectChoice(numChoices);
  }

  for (int j = 0; j < numTilings; j++)
    setTrace(tiles_[choice][j], 1.0);

  return choice;
}

/**
 * num_choices for agentIdx
 * note: take care of sync among processes
 * @param num_choices
 * @return
 */
int LinearSarsaLearner::step(int current_time, int num_choices) {
  SCOPED_LOG
  auto stackStr = HierarchicalFSM::getStackStr();
  sharedData->numChoices[Memory::ins().K[Memory::ins().agentIdx]] = num_choices;
  strcpy(sharedData->machineState[Memory::ins().K[Memory::ins().agentIdx]], stackStr.c_str());

  Log.log(101, "LinearSarsaLearner::step agent %d write numChoices %d", Memory::ins().agentIdx, num_choices);
  Log.log(101, "LinearSarsaLearner::step agent %d write machineState %s", Memory::ins().agentIdx,
          stackStr.c_str());

  barrier->wait();
  bool action_state = loadSharedData();
  barrier->wait();

  if (action_state) {
    return 0; // action state
  } else {
    if (Memory::ins().agentIdx == 0) {
      if (current_time == lastJointChoiceTime
          && lastMachineState.size()
          && machineState.size()
          && lastMachineState != machineState) {
        Log.log(101, "LinearSarsaLearner::step: save deterministical transition (%s) + (%d) -> (%s)",
                to_prettystring(lastMachineState).c_str(), lastJointChoiceIdx, to_prettystring(machineState).c_str());
        detTransitionMap[lastMachineState][lastJointChoiceIdx] = machineState;

        if (hasCircle(detTransitionMap)) {
          Log.log(101, "LinearSarsaLearner::step: detTransitionMap has circle");
          detTransitionMap[lastMachineState].erase(lastJointChoiceIdx);
          if (detTransitionMap[lastMachineState].empty()) {
            detTransitionMap.erase(lastMachineState);
          }
        }

#if DETERMINISTIC_GRAPH
        else {
          auto n = detTransitionGraph.getNodes().size();
          detTransitionGraph.addEdge(
              to_prettystring(lastMachineState),
              to_prettystring(machineState), "Blue", to_string(lastJointChoiceIdx));
          if (n != detTransitionGraph.getNodes().size()) {
            PRINT_VALUE(current_time);
            PRINT_VALUE(detTransitionGraph.getNodes().size());
          }
        }
#endif
      }

      lastJointChoiceIdx = step(current_time);
      lastJointChoiceTime = current_time;
      lastJointChoice = jointChoicesMap[numChoices][lastJointChoiceIdx];
      lastMachineState = machineState;
      saveSharedData();
    }

    barrier->wait();
    bool action_state2 = loadSharedData();
    barrier->wait();

    if (action_state2 == action_state) {
      if (numChoices[Memory::ins().agentIdx] <= 1) { // dummy choice
        return step(current_time, num_choices);
      } else {
        assert(lastJointChoice.size() == HierarchicalFSM::num_keepers);
        return lastJointChoice[Memory::ins().agentIdx];
      }
    } else { // race condition?
      return 0;
    }
  }
}

void LinearSarsaLearner::endEpisode(int current_time) {
  SCOPED_LOG
  if (Memory::ins().agentIdx == 0) { // only one agent can update
    loadSharedData();

    if (bLearning && lastJointChoiceIdx >= 0) {
      Assert(numTilings > 0);
      Assert(lastJointChoiceTime != UnknownTime);
      Assert(lastJointChoiceTime <= current_time);
      double tau = current_time - lastJointChoiceTime;
      double delta = reward(tau) - Q[lastJointChoiceIdx];
      updateWeights(delta, numTilings);
    }

    lastJointChoiceIdx = -1;
    lastJointChoiceTime = UnknownTime;
    fill(lastMachineState.begin(), lastMachineState.end(), "");
    fill(machineState.begin(), machineState.end(), "");
    fill(numChoices.begin(), numChoices.end(), 1);
    fill(lastJointChoice.begin(), lastJointChoice.end(), 0);
    saveSharedData();

    if (bLearning && bSaveWeights && rand() % 1000 == 0) {
      saveWeights(saveWeightsFile.c_str());
    }
  }

  barrier->wait();
  loadSharedData();
  barrier->wait();
}

int LinearSarsaLearner::loadTiles(
    double state[],
    const vector<string> &machine_state,
    const vector<int> &num_choices,
    int (*tiles)[RL_MAX_NUM_TILINGS]) {
  const int tilingsPerGroup = 32;

  int h = (int) (hash<string>()(to_prettystring(machine_state)) % INT_MAX); // joint machine state
  Log.log(101, "LinearSarsaLearner::loadTiles machine state: [%s], num_choices=%d, (hash=%d)",
          to_prettystring(machine_state).c_str(), validChoices(num_choices).size(), h);

  int numTilings = 0;
  for (int v = 0; v < HierarchicalFSM::num_features; v++) {
    for (auto a: validChoices(num_choices)) {
      GetTiles1(&(tiles[a][numTilings]), tilingsPerGroup, colTab,
                (float) (state[v] / tileWidths[v]), a, v, h);
    }
    numTilings += tilingsPerGroup;
  }

  Assert(numTilings > 0);
  Assert(numTilings < RL_MAX_NUM_TILINGS);
  return numTilings;
}

/**
 * compute Q vlaue assumming tiles loaded
 * @param state
 * @param machine_state
 * @param choice
 * @param tiles
 * @param num_tilings
 * @return
 */
double LinearSarsaLearner::QValue(
    double *state,
    const vector<string> &machine_state,
    int choice,
    int (*tiles)[RL_MAX_NUM_TILINGS],
    int num_tilings) {
  if (isDeterministic(machine_state, choice) &&
      detTransitionMap[machine_state][choice] != machine_state) {
    Log.log(101, "LinearSarsaLearner::QValue: deterministically Q(s, m=%s, c=%d) = V(s, m=%s)",
            to_prettystring(machine_state).c_str(), choice,
            to_prettystring(detTransitionMap[machine_state][choice]).c_str());
    return Value(state, detTransitionMap[machine_state][choice]);
  } else {
    auto q = computeQ(choice, tiles, num_tilings);
    Log.log(101, "LinearSarsaLearner::QValue: Q(s, m=%s, c=%d) = %f", to_prettystring(machine_state).c_str(), choice,
            q);
    return q;
  }
}

double LinearSarsaLearner::computeQ(int choice, int (*tiles)[RL_MAX_NUM_TILINGS], int numTilings) {
  double q = 0.0;
  for (int j = 0; j < numTilings; j++) {
    q += weights[tiles[choice][j]];
  }

  return q;
}

double LinearSarsaLearner::Value(double *state, const vector<string> &machine_state) {
  auto tiles = new int[MAX_RL_ACTIONS][RL_MAX_NUM_TILINGS];
  double v = numeric_limits<double>::min();

  Assert(numChoicesMap.count(machine_state));
  auto num_choices = numChoicesMap[machine_state];
  int num_tilings = loadTiles(state, machine_state, num_choices, tiles);

  for (auto c : validChoices(num_choices)) {
    double tmp = QValue(state, machine_state, c, tiles, num_tilings);
    if (tmp > v) {
      v = tmp;
    }
  }

  Log.log(101, "LinearSarsaLearner::Value: V(s, m=%s) = %f", to_prettystring(machine_state).c_str(), v);
  delete[] tiles;
  return v;
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
  int bestAction = 0;
  double bestValue = numeric_limits<double>::min();
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

  return bestAction;
}

void LinearSarsaLearner::updateWeights(double delta, int num_tilings) {
  Log.log(101, "LinearSarsaLearner::updateWeights delta %f", delta);

  Assert(num_tilings > 0);
  double tmp = delta * alpha / num_tilings;

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

void LinearSarsaLearner::decayTraces(double decayRate) {
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

void LinearSarsaLearner::clearTrace(int f) {
  if (f >= RL_MEMORY_SIZE || f < 0) {
    Assert(0);
    cerr << "ClearTrace: f out of range " << f << endl;
    return;
  }

  if (traces[f] != 0)
    clearExistentTrace(f, nonzeroTracesInverse[f]);
}

void LinearSarsaLearner::clearExistentTrace(int f, int loc) {
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

void LinearSarsaLearner::setTrace(int f, float newTraceValue) {
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
  FileLock lock("loadWeights");
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
  FileLock lock("saveWeights");
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
