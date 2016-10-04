//
// Created by baj on 10/4/16.
//

#include <cassert>
#include <climits>
#include <sstream>
#include "LinearSarsaLearner.h"
#include "HierarchicalFSM.h"

LinearSarsaLearner::LinearSarsaLearner(fsm::HierarchicalFSM *machine) :
    machine(machine) {
  gamma = machine->gamma;
  alpha = 0.125;
  lambda = 0.0;
  epsilon = 0.01;

  fill(traces, traces + RL_MEMORY_SIZE, 0.0);

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

  numTilings = 0;
  minimumTrace = 0.01;
  numNonzeroTraces = 0;
}

int LinearSarsaLearner::step(int num_choices) {
  assert(machine->lastChoice >= 0);
  double delta = machine->cumulativeReward - Q[machine->lastChoice]; //r - Q_{t-1}[s_{t-1}, a_{t-1}]

  loadTiles(machine->state, machine->stack, num_choices); //s_t, m_t
  for (int a = 0; a < num_choices; a++) {
    Q[a] = computeQ(a); //Q_{t-1}[s_t, *]
  }

  machine->lastChoice = selectChoice(num_choices); //a_t

  if (!bLearning)
    return machine->lastChoice;

  assert(!std::isnan(Q[machine->lastChoice]) && !std::isinf(Q[machine->lastChoice]));
  delta += machine->cumulativeGamma * Q[machine->lastChoice]; //delta += gamma**tau * Q_{t-1}[s_t, a_t]

  updateWeights(delta);
  Q[machine->lastChoice] = computeQ(machine->lastChoice); //need to redo because weights changed: Q_t[s_t, a_t]

  decayTraces(gamma * lambda);

  for (int a = 0; a < num_choices; a++) {  //clear other than F[a]
    if (a != machine->lastChoice) {
      for (int j = 0; j < numTilings; j++)
        clearTrace(tiles[a][j]);
    }
  }

  for (int j = 0; j < numTilings; j++)      //replace/set traces F[a]
    setTrace(tiles[machine->lastChoice][j], 1.0);

  Log.log(101, "LinearSarsaLearner::step saved numNonzeroTraces: %d", numNonzeroTraces);
  return machine->lastChoice;
}

void LinearSarsaLearner::loadTiles(
    double state[], const vector<string> &stack, int num_choices) { // TODO: use stack
  const int tilingsPerGroup = 32;

  stringstream ss;
  ss << stack;
  auto h = std::hash<string>().operator()(ss.str());

  numTilings = 0;

  /* These are the 'tiling groups'  --  play here with representations */
  /* One tiling for each state variable */
  for (int v = 0; v < machine->num_features; v++) {
    for (int a = 0; a < num_choices; a++) {
      GetTiles1(&(tiles[a][numTilings]), tilingsPerGroup, colTab,
                (float) (state[v] / tileWidths[v]), a, v, h);
    }
    numTilings += tilingsPerGroup;
  }

  assert(numTilings > 0);
  assert(numTilings < RL_MAX_NUM_TILINGS);
}

// Compute an action value from current F and theta
double LinearSarsaLearner::computeQ(int a) {
  double q = 0;
  for (int j = 0; j < numTilings; j++) {
    q += weights[tiles[a][j]];
  }

  return q;
}

int LinearSarsaLearner::selectChoice(int num_choices) {
  int action;

  // Epsilon-greedy
  if (bLearning && drand48() < epsilon) {     /* explore */
    action = rand() % num_choices;
  } else {
    action = argmaxQ(num_choices);
  }

  assert(action >= 0);
  return action;
}

// Returns index (action) of largest entry in Q array, breaking ties randomly
int LinearSarsaLearner::argmaxQ(int num_choices) {
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

void LinearSarsaLearner::updateWeights(double delta) {
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


// Clear any trace for feature f
void LinearSarsaLearner::clearTrace(int f) {
  if (f >= RL_MEMORY_SIZE || f < 0) {
    cerr << "ClearTrace: f out of range " << f << endl;
    return;
  }

  if (traces[f] != 0)
    clearExistentTrace(f, nonzeroTracesInverse[f]);
}

// Clear the trace for feature f at location loc in the list of nonzero traces
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
