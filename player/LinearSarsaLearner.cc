//
// Created by baj on 10/4/16.
//

#include <cassert>
#include <climits>
#include <sstream>
#include "LinearSarsaLearner.h"
#include "HierarchicalFSM.h"

namespace fsm {

LinearSarsaLearner &LinearSarsaLearner::ins() {
  static LinearSarsaLearner learner;
  return learner;
}

LinearSarsaLearner::LinearSarsaLearner() {
  alpha = 0.125;
  lambda = 0.0;
  epsilon = 0.01;

  fill(traces, traces + RL_MEMORY_SIZE, 0.0);
  fill(weights, weights + RL_MEMORY_SIZE, 1.0);

  srand((unsigned int) 0);
  srand48((unsigned int) 0);
  int tmp[2];
  float tmpf[2];
  colTab = new collision_table(RL_MEMORY_SIZE, 1);

  GetTiles(tmp, 1, 1, tmpf, 0);  // A dummy call to set the hashing table
  srand48((unsigned int) time(NULL));

  numTilings = 0;
  minimumTrace = 0.01;
  numNonzeroTraces = 0;
}

void LinearSarsaLearner::setLearning(bool learning) {
  bLearning = learning;
}

void LinearSarsaLearner::setWidth(double width[]) {
  for (int i = 0; i < HierarchicalFSM::num_features; i++) {
    tileWidths[i] = width[i];
  }
}

void LinearSarsaLearner::loadQ(int num_choices) {
  loadTiles(Memory::ins().state, Memory::ins().stack, Memory::ins().agentIdx, num_choices);
  Log.log(101, "LinearSarsaLearner::loadQ numTilings: %d", numTilings);
  for (int a = 0; a < num_choices; a++) {
    Q[a] = computeQ(a);
  }
}

int LinearSarsaLearner::step(int num_choices) {
  if (Log.isInLogLevel(101))
    Log.log(101, "LinearSarsaLearner::step memory: \n%s",
            Memory::ins().to_string().c_str());

  auto last_choice = Memory::ins().lastChoice; // the last choice
  auto choice = -1;

  if (last_choice >= 0) {
    double delta = Memory::ins().cumulativeReward - Q[last_choice];
    loadQ(num_choices);
    choice = selectChoice(num_choices); // the new choice

    if (!bLearning) return choice;

    assert(!std::isnan(Q[choice]) && !std::isinf(Q[choice]));
    delta += Memory::ins().cumulativeGamma * Q[choice];

    updateWeights(delta, last_choice);
    Q[choice] = computeQ(choice); // update Q[choice]
    decayTraces(HierarchicalFSM::gamma * lambda);

    for (int a = 0; a < num_choices; a++) {
      if (a != choice) {
        for (int j = 0; j < numTilings; j++)
          clearTrace(tiles[a][j]);
      }
    }
  } else { // new episode
    decayTraces(0.0);
    assert(numNonzeroTraces == 0);

    loadQ(num_choices);
    choice = selectChoice(num_choices);
  }

  assert(choice >= 0 && choice < num_choices);
  if (choice >= 0 && choice < num_choices) {
    for (int j = 0; j < numTilings; j++)
      setTrace(tiles[choice][j], 1.0);
  }

  Log.log(101, "LinearSarsaLearner::step saved numNonzeroTraces: %d", numNonzeroTraces);
  Log.log(101, "LinearSarsaLearner::step choice made %d", choice);
  return choice;
}

void LinearSarsaLearner::endEpisode() {
  if (Log.isInLogLevel(101))
    Log.log(101, "LinearSarsaLearner::endEpisode memory: \n%s",
            Memory::ins().to_string().c_str());

  auto last_choice = Memory::ins().lastChoice;
  if (bLearning && last_choice != -1) {
    double delta = Memory::ins().cumulativeReward - Q[last_choice];
    updateWeights(delta, last_choice);
  }
}

void LinearSarsaLearner::loadTiles(
    double state[], const vector<string> &stack, int agentIdx, int num_choices) {
  const int tilingsPerGroup = 32;

  stringstream ss;
  ss << stack;
  int h = (int) (std::hash<string>().operator()(ss.str()) % INT_MAX);
  Log.log(101, "LinearSarsaLearner::loadTiles stack %s hash %d", ss.str().c_str(), h);

  numTilings = 0;

  for (int v = 0; v < HierarchicalFSM::num_features; v++) {
    for (int a = 0; a < num_choices; a++) {
      GetTiles1(&(tiles[a][numTilings]), tilingsPerGroup, colTab,
                (float) (state[v] / tileWidths[v]), a, v, h, agentIdx);
    }
    numTilings += tilingsPerGroup;
  }

  assert(numTilings > 0);
  assert(numTilings < RL_MAX_NUM_TILINGS);
}

double LinearSarsaLearner::computeQ(int a) {
  double q = 0;
  for (int j = 0; j < numTilings; j++) {
    q += weights[tiles[a][j]];
  }

  return q;
}

int LinearSarsaLearner::selectChoice(int num_choices) {
  int action;

  if (bLearning && drand48() < epsilon) {     /* explore */
    action = rand() % num_choices;
    Log.log(101, "explore choice %d in %d", action, num_choices);
  } else {
    action = argmaxQ(num_choices);
    Log.log(101, "argmaxQ choice %d in %d", action, num_choices);
  }

  assert(action >= 0);
  return action;
}

int LinearSarsaLearner::argmaxQ(int num_choices) {
  Log.log(101, "Q[0..%d) %s", num_choices, getQStr(num_choices).c_str());

  int bestAction = 0;
  double bestValue = INT_MIN;
  int numTies = 0;
  for (int a = 0; a < num_choices; a++) {
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

void LinearSarsaLearner::updateWeights(double delta, int last_choice) {
  Log.log(101, "LinearSarsaLearner::updateWeights delta %f", delta);
  Log.log(101, "LinearSarsaLearner::updateWeights before Q[%d] = %f", last_choice, Q[last_choice]);

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

  Q[last_choice] = computeQ(last_choice); // update Q[choice]
  Log.log(101, "LinearSarsaLearner::updateWeights after Q[%d] = %f", last_choice, Q[last_choice]);
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
  ss << vector<double>(Q, Q + num_choice);
  return ss.str();
}

}
