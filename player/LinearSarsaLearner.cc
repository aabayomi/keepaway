////////////////////////

#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <sstream>
#include "LinearSarsaLearner.h"
#include "KeepawayPlayer.h"
#include "LoggerDraw.h"
#include "gzstream.h"
#include <iostream>
#include <stack>
#include <tuple>

#define DETERMINISTIC_GRAPH 0
// If all is well, there should be no mention of anything keepaway- or soccer-
// related in this file.

void SharedData::reset()
{
  lastJointChoiceIdx = -1;
  lastJointChoiceTime = UnknownTime;
  memset(lastJointChoice, 0, sizeof(lastJointChoice));

  memset(Q, 0, sizeof(Q));
  memset(sharedQ, 0, sizeof(sharedQ));
  memset(traces, 0, sizeof(traces));
  memset(nonzeroTraces, 0, sizeof(nonzeroTraces));
  memset(nonzeroTracesInverse, 0, sizeof(nonzeroTracesInverse));

  minimumTrace = 0.01;
  numNonzeroTraces = 0;

  memset(numChoices, 0, sizeof(numChoices));
};

// size_t Memory::ballControlHash()
// {
//   return hash<vector<int>>().operator()(vector<int>(ballControlState, ballControlState + LinearSarsaAgent::num_teammates));
// }

/**
 * Designed specifically to match the serialization format for collision_table.
 * See collision_table::save and collision_table::restore.
 */
// #pragma pack(push, 1)
// struct CollisionTableHeader
// {
//   long m;
//   int safe;
//   long calls;
//   long clearhits;
//   long collisions;
// };
// #pragma pack(pop)
// #define VERBOSE_HIVE_MIND false

/**
 * Assumes collision table header follows weights.
 * Returns the memory location after the header because that's useful for colTab
 * data array.
 */
// long *loadColTabHeader(collision_table *colTab, double *weights)
// {
//   CollisionTableHeader *colTabHeader =
//       reinterpret_cast<CollisionTableHeader *>(weights + RL_MEMORY_SIZE);
//   // Do each field individually, since they don't all line up exactly for an
//   // easy copy.
//   colTab->calls = colTabHeader->calls;
//   colTab->clearhits = colTabHeader->clearhits;
//   colTab->collisions = colTabHeader->collisions;
//   colTab->m = colTabHeader->m;
//   colTab->safe = colTabHeader->safe;
//   if (VERBOSE_HIVE_MIND)
//   {
//     cout << "Loaded colTabHeader:" << endl
//          << " calls: " << colTab->calls << endl
//          << " clearhits: " << colTab->clearhits << endl
//          << " collisions: " << colTab->collisions << endl
//          << " m: " << colTab->m << endl
//          << " safe: " << colTab->safe << endl;
//   }
//   return reinterpret_cast<long *>(colTabHeader + 1);
// }

extern LoggerDraw LogDraw;

LinearSarsaAgent::LinearSarsaAgent(int numFeatures, int numTeammates,
                                   int numOpponents, int numActions, bool bLearn,
                                   double widths[], double weight, bool QLearning,
                                   string loadWeightsFile, string saveWeightsFile, bool hiveMind, string teamName) : SMDPAgent(numFeatures, numActions), hiveFile(-1)
{
  bLearning = bLearn;
  bSaveWeights = bLearning && saveWeightsFile.length() > 0;
  saveWeightsFile = saveWeightsFile;
  qLearning = QLearning;
  teamName = teamName;

  num_features = numFeatures;
  num_teammates = numTeammates;
  num_opponents = numOpponents;

  for (int i = 0; i < getNumFeatures(); i++)
  {
    tileWidths[i] = widths[i];
  }

  initialWeight = teamName == "keepers" ? weight : -weight;

  alpha = 0.125;
  gamma = 1.0;
  lambda = 0;
  epsilon = 0.01;

  lastJointChoiceIdx = -1;
  lastJointChoiceTime = UnknownTime;

  numNonzeroTraces = 0;
  minimumTrace = 0.01;
  numTilings = 0;

  srand((unsigned int)0);
  srand48((unsigned int)0);
  int tmp[2];
  float tmpf[2];

  GetTiles(tmp, 1, 1, tmpf, 0); // A dummy call to set the hashing table
  srand((unsigned int)time(NULL));
  srand48((unsigned int)time(NULL));

  numTilings = 0;
  minimumTrace = 0.01;
  numNonzeroTraces = 0;

  if (bLearning || !bLearning)
  {
    string exepath = getexepath();
    exepath += "LinearSarsaAgent::LinearSarsaAgent";
    exepath += loadWeightsFile;
    exepath += saveWeightsFile;
    exepath += to_string(gamma);
    exepath += to_string(lambda);
    exepath += to_string(initialWeight);
    exepath += to_string(qLearning);
    exepath += teamName;
    auto h = std::hash<string>()(exepath); // hashing
    sharedMemory = "/" + to_string(h) + ".shm";

    int shm_fd = shm_open(sharedMemory.c_str(), O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1)
    {
      printf("prod: Shared memory failed: %s\n", strerror(errno));
      exit(1);
    }

    ftruncate(shm_fd, sizeof(SharedData));
    sharedData = (SharedData *)mmap(
        0, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (sharedData == MAP_FAILED)
    {
      printf("prod: Map failed: %s\n", strerror(errno));
      exit(1);
    }

    Q = sharedData->Q;
    sharedQ = sharedData->sharedQ;
    tiles = sharedData->tiles;
    weights = sharedData->weights;
    traces = sharedData->traces;
    nonzeroTraces = sharedData->nonzeroTraces;
    nonzeroTracesInverse = sharedData->nonzeroTracesInverse;
    colTab = &sharedData->colTab;

    barriers["enter1"] = new Barrier(num_teammates, h, "enter1");
    barriers["exit1"] = new Barrier(num_teammates, h, "exit1");
    barriers["enter2"] = new Barrier(num_teammates, h, "enter2");
    barriers["exit2"] = new Barrier(num_teammates, h, "exit2");
    barriers["reset"] = new Barrier(num_teammates, h, "reset");

    sharedData->reset();
    if (loadWeightsFile.empty() || !loadWeights(loadWeightsFile.c_str()))
    {
      fill(weights, weights + RL_MEMORY_SIZE, initialWeight);
      colTab->reset();
    }
    loadWeights(loadWeightsFile.c_str());
  }
  cout << "LinearSarsaAgent::LinearSarsaAgent" << endl;
}

int LinearSarsaAgent::startEpisode(double state[])
{
  cout << "startEpisode" << Memory::ins().agentIdx << endl;

  // if (hiveMind)
  //   loadColTabHeader(colTab, weights);
  // epochNum++;
  // decayTraces(0);
  // loadTiles(state);
  // for (int a = 0; a < getNumActions(); a++)
  // {
  //   Q[a] = computeQ(a);
  // }

  // lastAction = selectAction();
  // // Log.log("Right before making the weightToString call", lastAction.c_str());

  // char buffer[128];
  // sprintf(buffer, "Q[%d] = %.2f", lastAction, Q[lastAction]);
  // LogDraw.logText("Qmax", VecPosition(25, -30),
  //                 buffer,
  //                 1, COLOR_BROWN);

  // for (int j = 0; j < numTilings; j++)
  //   setTrace(tiles[lastAction][j], 1.0);
  // if (hiveMind)
  //   saveWeights(weightsFile);
  // return lastAction;
}

bool LinearSarsaAgent::loadSharedData()
{
  numTilings = sharedData->numTilings;
  minimumTrace = sharedData->minimumTrace;
  numNonzeroTraces = sharedData->numNonzeroTraces;
  lastJointChoiceIdx = sharedData->lastJointChoiceIdx;
  lastJointChoiceTime = sharedData->lastJointChoiceTime;
  sharedQ = sharedData->sharedQ;

  // lastJointChoice = sharedData->getLastJointChoice();
  // machineState = sharedData->getMachineState();
  // lastMachineState = sharedData->getLastMachineState();
  // numChoices = sharedData->getNumChoices();

  // localValuesMap = sharedData->localValuesMap;

  // numChoicesMap[machineState] = numChoices;

  bool action_state = true; // action state (when all are in action)
  for (int i = 0; i < num_teammates; ++i)
  {
    // if (numChoices[i] > 1)
    // {
    //   action_state = false;
    // }
  }

  if (Log.isInLogLevel(101))
  {
    stringstream ss;
    PRINT_VALUE_STREAM(ss, Memory::ins().to_string());
    PRINT_VALUE_STREAM(ss, numTilings);
    PRINT_VALUE_STREAM(ss, minimumTrace);
    PRINT_VALUE_STREAM(ss, numNonzeroTraces);
    PRINT_VALUE_STREAM(ss, action_state);

    Log.log(101, "LinearSarsaAgent::loadSharedData\n%s", ss.str().c_str());
  }

  return action_state;
}

void LinearSarsaAgent::saveSharedData()
{

  sharedData->numTilings = numTilings;
  sharedData->minimumTrace = minimumTrace;
  sharedData->numNonzeroTraces = numNonzeroTraces;
  sharedData->lastJointChoiceIdx = lastJointChoiceIdx;
  sharedData->lastJointChoiceTime = lastJointChoiceTime;

  for (int i = 0; i < num_teammates; ++i)
  {
    // sharedData->numChoices[Memory::ins().teammates[i]] = numChoices[i];
    // strcpy(sharedData->machineState[Memory::ins().teammates[i]],
    //        machineState[i].c_str());
    // strcpy(sharedData->lastMachineState[i], lastMachineState[i].c_str());
    // sharedData->lastJointChoice[i] = lastJointChoice[i];
  }
}

// std::stack<std::tuple<int, int>> LinearSarsaAgent::graph()
// {
//   std::stack<std::tuple<int, int>> cordGraph;
//   for (int i = 0; i < num_teammates; ++i)
//   {
//     int j = (i + 1) % num_teammates;
//     cordGraph.push(std::make_tuple(i, j));
//   }
//   return cordGraph;
// }

int LinearSarsaAgent::array_hash(int agent, int action)
{
  return agent * 3 + action;
}
double LinearSarsaAgent::reward(double tau)
{
  Log.log(101, "LinearSarsaLearner::reward tau=%f", tau);
  double ret = tau;
  if (gamma < 1.0)
  {
    ret = (1.0 - pow(gamma, tau)) / (1.0 - gamma);
  }
  return teamName == "keepers" ? ret : -ret;
}

// coordination graph via variable elimination algorithm.

double LinearSarsaAgent::Q1(int a1, int a2, int (*tiles)[RL_MAX_NUM_TILINGS],
                            int num_tilings)
{
  double val = -1e9;
  for (int a2 = 0; a2 < getNumActions(); a2++)
  {
    int index = static_cast<int>(array_hash(1, a2));
    double val1 = sharedData->sharedQ[index];
    if (val1 > val)
    {
      val = val1;
    }
  }
  double v = computeQ(a2, tiles, num_tilings);
  return val + v;
}

double LinearSarsaAgent::Q2(int a1, int a3, int (*tiles)[RL_MAX_NUM_TILINGS],
                            int num_tilings)
{
  double val = -1e9;
  for (int a1 = 0; a1 < getNumActions(); a1++)
  {
    int index = static_cast<int>(array_hash(1, a1));
    auto val1 = sharedData->sharedQ[index];
    if (val1 > val)
    {
      val = val1;
    }
  }
  return val + computeQ(a3, tiles, num_tilings);
}
double LinearSarsaAgent::Q3(int a2, int a3, int (*tiles)[RL_MAX_NUM_TILINGS],
                            int num_tilings)
{
  double val = -1e9;
  for (int a2 = 0; a2 < getNumActions(); a2++)
  {
    int index = static_cast<int>(array_hash(2, a3));
    auto val2 = sharedData->sharedQ[index];
    if (val2 > val)
    {
      val = val2;
    }
  }

  return val + computeQ(a3, tiles, num_tilings);
}

std::tuple<double, int> LinearSarsaAgent::e3(int a1, int a2, int num_tilings)
{
  double maxVal = -1e9;
  int argmax_a3 = -1;
  for (int a3 = 0; a3 < getNumActions(); a3++)
  {
    double current = Q2(a1, a3, tiles, num_tilings) + Q3(a2, a3, tiles, num_tilings);
    if (current > maxVal)
    {
      maxVal = current;
      argmax_a3 = a3;
    }
  }
  return {maxVal, argmax_a3};
}

std::tuple<double, int> LinearSarsaAgent::e2(int a1, int num_tilings)
{
  double maxVal = -1e9;
  int argmax_a2 = -1;
  for (int a2 = 0; a2 <= getNumActions(); a2++)
  {
    double current = Q1(a1, a2, tiles, num_tilings) + std::get<0>(e3(a1, a2, num_tilings));
    if (current > maxVal)
    {
      maxVal = current;
      argmax_a2 = a2;
    }
  }
  return {maxVal, argmax_a2};
}

double LinearSarsaAgent::computeE1(int &optimal_a1)
{
  double maxVal = -1e9;
  for (int a1 = 0; a1 < getNumActions(); a1++)
  {
    double current = std::get<0>(e2(a1, numTilings));
    if (current > maxVal)
    {
      maxVal = current;
      optimal_a1 = a1;
    }
  }
  return maxVal;
}

void LinearSarsaAgent::jointOptimalQ(int numTilings)
{
  // computeE1(lastJointChoiceIdx);
  int optimal_a1;
  std::cout << "E1: " << computeE1(optimal_a1) << std::endl;
  // Backpropagation to recover maximizing actions
  int optimal_a2 = std::get<1>(e2(optimal_a1, numTilings));
  int optimal_a3 = std::get<1>(e3(optimal_a1, optimal_a2, numTilings));

  // fill last joint
  // if (Memory::ins().agentIdx ==)
  sharedData->lastJointChoice[1] = optimal_a1;
  sharedData->lastJointChoice[2] = optimal_a2;
  sharedData->lastJointChoice[3] = optimal_a3;
}

double LinearSarsaAgent::QValue(double *state,
                                int a, int (*tiles)[RL_MAX_NUM_TILINGS],
                                int num_tilings)
{
  auto q = computeQ(a, tiles, num_tilings);
  // Log.log(101, "LinearSarsaAgent::QValue: Q(s, m=%s, c=%d) = %f",
  //         to_prettystring(machine_state).c_str(), a, q);
  return q;
}

double LinearSarsaAgent::computeQ(int a,
                                  int (*tiles)[RL_MAX_NUM_TILINGS],
                                  int numTilings)
{
  double q = 0.0;
  for (int j = 0; j < numTilings; j++)
  {
    q += weights[tiles[a][j]];
  }

  return q;
}

int LinearSarsaAgent::step(double current_time, double state[])
{
  int action = -1;
  int optimal_action = -1;
  // auto *state = Memory::ins().state;

  if (lastJointChoiceIdx >= 0)
  {
    // Assert(lastJointChoiceTime != UnknownTime);
    double tau = current_time - lastJointChoiceTime;
    double delta = reward(tau) - Q[lastJointChoiceIdx];
    numTilings = loadTiles(state, tiles); //

    // compute joint optimal q value and action.

    for (int a = 0; a < getNumActions(); a++)
    {
      Q[a] = computeQ(a, tiles, numTilings);
      int index = array_hash(Memory::ins().agentIdx, a);
      cout << "index " << Memory::ins().agentIdx << endl;
      sharedData->sharedQ[array_hash(Memory::ins().agentIdx, a)] = Q[a];
      // cout << "Q[" << a << "] = " << Q[a] << endl;
    }
    // check same state
    jointOptimalQ(numTilings);

    optimal_action = sharedData->lastJointChoice[Memory::ins().agentIdx];
    action = selectAction(optimal_action);

    if (!bLearning)
      return action;

    if (qLearning)
    {
      delta += pow(gamma, tau) * Q[optimal_action]; // local Q value
    }
    else
    {
      delta += pow(gamma, tau) * Q[action];
    }

    updateWeights(delta, numTilings);

    Q[action] = computeQ(action, tiles, numTilings);

    decayTraces(gamma * lambda);
    for (int a = 0; a < getNumActions(); a++)
    {
      if (a != action)
      {
        for (int j = 0; j < numTilings; j++)
          clearTrace(tiles[a][j]);
      }
    }
  }
  else
  {

    decayTraces(0.0);
    // Assert(numNonzeroTraces == 0);
    numTilings = loadTiles(state, tiles);

    for (int a = 0; a < getNumActions(); a++)
    {
      Q[a] = computeQ(a, tiles, numTilings);
      int index = array_hash(Memory::ins().agentIdx, a);
      // cout << "index " << index << endl;
      sharedData->sharedQ[array_hash(Memory::ins().agentIdx, a)] = Q[a];
      // cout << "Q[" << a << "] = " << Q[a] << endl;
    }
    // choice = selectChoice(numChoices);
    action = selectAction(optimal_action);
  }
  for (int j = 0; j < numTilings; j++)
    setTrace(tiles[action][j], 1.0);

  return action;
}

// int LinearSarsaAgent::step(int current_time, int action)
// {
// }

// int LinearSarsaAgent::step(int current_time, int action)
// {
//   SCOPED_LOG

//   barriers["enter1"]->wait();
//   bool action_state = loadSharedData();
//   barriers["exit1"]->wait();

//   if (action_state)
//   {
//     return 0; // action state
//   }
//   else
//   {
//     if (Memory::ins().agentIdx == 1)
//     {
//       lastJointChoiceIdx = step(current_time);
//       lastJointChoiceTime = current_time;
//       lastJointChoice = jointChoicesMap[numChoices][lastJointChoiceIdx];
//       lastMachineState = machineState;
//       saveSharedData();
//     }

//     barriers["enter2"]->wait();
//     bool action_state2 = loadSharedData();
//     barriers["exit2"]->wait();

//     if (action_state2 == action_state)
//     {
//       if (numChoices[Memory::ins().agentIdx] <= 1)
//       { // dummy choice
//         return step(current_time, num_choices);
//       }
//       else
//       {
//         Assert(lastJointChoice.size() == HierarchicalFSM::num_teammates);
//         return lastJointChoice[Memory::ins().agentIdx];
//       }
//     }
//     else
//     { // race condition?
//       return 0;
//     }
//   }
// }

void LinearSarsaAgent::endEpisode(double current_time)
{
  barriers["reset"]->wait(); // recover from non synchronization (if any)
  if (Memory::ins().agentIdx == 1)
  { // only one agent can update
    loadSharedData();

    if (bLearning && lastJointChoiceIdx >= 0)
    {
      Assert(numTilings > 0);
      Assert(lastJointChoiceTime != UnknownTime);
      Assert(lastJointChoiceTime <= current_time);
      double tau = current_time - lastJointChoiceTime;
      double delta = reward(tau) - Q[lastJointChoiceIdx];
      updateWeights(delta, numTilings);
    }

    lastJointChoiceIdx = -1;
    lastJointChoiceTime = UnknownTime;
    // fill(lastMachineState.begin(), lastMachineState.end(), "");
    // fill(machineState.begin(), machineState.end(), "");
    // fill(numChoices.begin(), numChoices.end(), 1);
    // fill(lastJointChoice.begin(), lastJointChoice.end(), 0);
    saveSharedData();
  }

  barriers["enter2"]->wait();
  loadSharedData();
  barriers["exit2"]->wait();
}

// void LinearSarsaAgent::endEpisode(double reward)
// {
//   if (hiveMind)
//     loadColTabHeader(colTab, weights);
//   if (bLearning && lastAction != -1)
//   { /* otherwise we never ran on this episode */
//     char buffer[128];
//     sprintf(buffer, "reward: %.2f", reward);
//     LogDraw.logText("reward", VecPosition(25, 30),
//                     buffer,
//                     1, COLOR_NAVY);

//     /* finishing up the last episode */
//     /* assuming gamma = 1  -- if not,error*/
//     if (gamma != 1.0)
//       cerr << "We're assuming gamma's 1" << endl;
//     double delta = reward - Q[lastAction];
//     updateWeights(delta);
//     // TODO Actually, there's still possibly risk for trouble here with multiple
//     // TODO players stomping each other. Is this okay?
//     // TODO The weight updates themselves are in order.
//   }
//   if (bLearning && bSaveWeights && rand() % 200 == 0 && !hiveMind)
//   {
//     saveWeights(weightsFile);
//   }
//   if (hiveMind)
//     saveWeights(weightsFile);
//   lastAction = -1;
// }

void LinearSarsaAgent::shutDown()
{
  if (Memory::ins().agentIdx == 1)
  {
    if (bLearning && bSaveWeights)
    {
      cerr << "Saving weights at shutdown." << endl;
      saveWeights(saveWeightsFile.c_str());
    }
  }

  if (sharedData)
    shm_unlink(sharedMemory.c_str());
  for (auto pa : barriers)
    delete pa.second;
}

// void LinearSarsaAgent::shutDown()
// {
//   // We usually only save weights at random intervals.
//   // Always save at shutdown (if we are in saving mode).

//   // if (Memory::ins().agentIdx == 0)
//   // {
//   //   if (bLearning && bSaveWeights)
//   //   {
//   //     cerr << "Saving weights at shutdown." << endl;
//   //     saveWeights(saveWeightsFile.c_str());
//   //   }
//   // }

//   // if (sharedData)
//   //   shm_unlink(sharedMemory.c_str());
//   // for (auto pa : barriers)
//   //   delete pa.second;

//   if (bLearning && bSaveWeights)
//   {
//     cout << "Saving weights at shutdown." << endl;
//     saveWeights(weightsFile);
//   }
//   // Also shut down the hive mind if needed.
//   if (hiveMind)
//   {
//     size_t mapLength =
//         RL_MEMORY_SIZE * sizeof(double) +
//         sizeof(CollisionTableHeader) +
//         colTab->m * sizeof(long);
//     munmap(weights, mapLength);
//     close(hiveFile);
//     hiveFile = -1;
//     // Go back to the own arrays, since our map is no longer valid.
//     weights = weightsRaw;
//     colTab->data = new long[colTab->m];
//   }
// }

int LinearSarsaAgent::selectAction(int a)
{
  int action;

  // Epsilon-greedy
  if (bLearning && drand48() < epsilon)
  { /* explore */
    action = rand() % getNumActions();
  }
  else
  {
    action = a;
  }
  return action;
}

// bool LinearSarsaAgent::loadWeights(const char *filename)
// {
//   cout << "Linear SARSA Loading weights from " << filename << endl;
//   if (hiveMind)
//   {
//     if (hiveFile < 0)
//     {
//       // First, check the lock file, so we have only one initializer.
//       // Later interaction should be approximately synchronized by having only
//       // one active player at a time per team, but we can't assume that here.
//       stringstream lockNameBuffer;
//       lockNameBuffer << filename << ".lock";
//       const char *lockName = lockNameBuffer.str().c_str();
//       int lock;
//       // 10ms delay (times a million to convert from nanos).
//       timespec sleepTime = {0, 10 * 1000 * 1000};
//       while (true)
//       {
//         lock = open(lockName, O_CREAT | O_EXCL, 0664);
//         if (lock >= 0)
//           break;
//         nanosleep(&sleepTime, NULL);
//       }
//       // First, see if the file is already there.
//       bool fileFound = !access(filename, F_OK);
//       // TODO Extract constant for permissions (0664)?
//       hiveFile = open(filename, O_RDWR | O_CREAT, 0664);
//       size_t mapLength =
//           RL_MEMORY_SIZE * sizeof(double) +
//           sizeof(CollisionTableHeader) +
//           colTab->m * sizeof(long);
//       if (!fileFound)
//       {
//         // Make the file the right size.
//         cout << "Initializing new hive file." << endl;
//         if (lseek(hiveFile, mapLength - 1, SEEK_SET) < 0)
//         {
//           throw "failed to seek initial file size";
//         }
//         if (write(hiveFile, "", 1) < 0)
//         {
//           throw "failed to expand initial file";
//         }
//       }
//       if (hiveFile < 0)
//         throw "failed to open hive file";
//       void *hiveMap =
//           mmap(NULL, mapLength, PROT_READ | PROT_WRITE, MAP_SHARED, hiveFile, 0);
//       if (hiveMap == MAP_FAILED)
//         throw "failed to map hive file";
//       // First the weights.
//       weights = reinterpret_cast<double *>(hiveMap);
//       // Now the collision table header.
//       CollisionTableHeader *colTabHeader =
//           reinterpret_cast<CollisionTableHeader *>(weights + RL_MEMORY_SIZE);
//       if (fileFound)
//       {
//         loadColTabHeader(colTab, weights);
//       }
//       // Now the collision table data.
//       delete[] colTab->data;
//       colTab->data = reinterpret_cast<long *>(colTabHeader + 1);
//       if (!fileFound)
//       {
//         // Clear out initial contents.
//         // The whole team might be doing this at the same time. Is that okay?
//         for (int i = 0; i < RL_MEMORY_SIZE; i++)
//         {
//           weights[i] = 0;
//         }
//         colTab->reset();
//         // Make sure the header goes out to the file.
//         saveWeights(weightsFile);
//       }
//       // TODO Separate file lock type with destructor?
//       unlink(lockName);
//     }
//   }
//   else
//   {
//     int file = open(filename, O_RDONLY);
//     read(file, (char *)weights, RL_MEMORY_SIZE * sizeof(double));
//     colTab->restore(file);
//     close(file);
//   }
//   cout << "...done" << endl;
//   return true;
// }

bool LinearSarsaAgent::loadWeights(const char *filename)
{
  FileLock lock("loadWeights");
  cerr << "Loading weights from " << filename << endl;

#ifdef _Compress
  igzstream is;
#else
  ifstream is;
#endif

  is.open(filename);
  if (!is.good())
  {
    cerr << "failed to open weight file: " << filename << endl;
    return false;
  }

  is.read((char *)weights, RL_MEMORY_SIZE * sizeof(double));
  colTab->restore(is);
  is.close();
  cerr << "...done" << endl;
  return true;
}

bool LinearSarsaAgent::saveWeights(const char *filename)
{
  FileLock lock("saveWeights");

#ifdef _Compress
  ogzstream os;
#else
  ofstream os;
#endif

  os.open(filename);
  if (!os.good())
  {
    cerr << "failed to open weight file: " << filename << endl;
    return false;
  }

  os.write((char *)weights, RL_MEMORY_SIZE * sizeof(double));
  colTab->save(os);
  os.close();
  return true;
}

// bool LinearSarsaAgent::saveWeights(const char *filename)
// {
//   if (hiveMind)
//   {
//     // The big arrays should be saved out automatically, but we still need to
//     // handle the collision table header.
//     CollisionTableHeader *colTabHeader =
//         reinterpret_cast<CollisionTableHeader *>(weights + RL_MEMORY_SIZE);
//     // Do each field individually, since they don't all line up exactly for an
//     // easy copy.
//     colTabHeader->calls = colTab->calls;
//     colTabHeader->clearhits = colTab->clearhits;
//     colTabHeader->collisions = colTab->collisions;
//     colTabHeader->m = colTab->m;
//     colTabHeader->safe = colTab->safe;
//     if (VERBOSE_HIVE_MIND)
//     {
//       cout << "Saved colTabHeader:" << endl
//            << " calls: " << colTab->calls << endl
//            << " clearhits: " << colTab->clearhits << endl
//            << " collisions: " << colTab->collisions << endl
//            << " m: " << colTab->m << endl
//            << " safe: " << colTab->safe << endl;
//     }
//   }
//   else
//   {
//     int file = open(filename, O_CREAT | O_WRONLY, 0664);
//     write(file, (char *)weights, RL_MEMORY_SIZE * sizeof(double));
//     colTab->save(file);
//     close(file);
//   }
//   return true;
// }

// Compute an action value from current F and theta
// double LinearSarsaAgent::computeQ(int a)
// {
//   double q = 0;
//   for (int j = 0; j < numTilings; j++)
//   {
//     q += weights[tiles[a][j]];
//   }

//   return q;
// }

// Returns index (action) of largest entry in Q array, breaking ties randomly
// int LinearSarsaAgent::argmaxQ()
// {
//   int bestAction = 0;
//   double bestValue = Q[bestAction];
//   int numTies = 0;
//   for (int a = bestAction + 1; a < getNumActions(); a++)
//   {
//     double value = Q[a];
//     if (value > bestValue)
//     {
//       bestValue = value;
//       bestAction = a;
//     }
//     else if (value == bestValue)
//     {
//       numTies++;
//       if (rand() % (numTies + 1) == 0)
//       {
//         bestValue = value;
//         bestAction = a;
//       }
//     }
//   }

//   return bestAction;
// }

// void LinearSarsaAgent::updateWeights(double delta)
// {
//   double tmp = delta * alpha / numTilings;
//   for (int i = 0; i < numNonzeroTraces; i++)
//   {
//     int f = nonzeroTraces[i];
//     if (f > RL_MEMORY_SIZE || f < 0)
//       cerr << "f is too big or too small!!" << f << endl;
//     weights[f] += tmp * traces[f];
//     // cout << "weights[" << f << "] = " << weights[f] << endl;
//   }
// }

void LinearSarsaAgent::updateWeights(double delta, int num_tilings)
{
  Log.log(101, "LinearSarsaLearner::updateWeights delta %f", delta);

  Assert(num_tilings > 0);
  double tmp = delta * alpha / num_tilings;

  for (int i = 0; i < numNonzeroTraces; i++)
  {
    Assert(i < RL_MAX_NONZERO_TRACES);

    int f = nonzeroTraces[i];
    Assert(f >= 0);
    Assert(f < RL_MEMORY_SIZE);

    if (f >= RL_MEMORY_SIZE || f < 0)
    {
      continue;
    }

    weights[f] += tmp * traces[f];
    Assert(!std::isnan(weights[f]));
    Assert(!std::isinf(weights[f]));
  }
}

int LinearSarsaAgent::loadTiles(double state[], int (*tiles)[RL_MAX_NUM_TILINGS])
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
  Assert(numTilings > 0);
  Assert(numTilings < RL_MAX_NUM_TILINGS);
  return numTilings;
}

// Clear any trace for feature f
void LinearSarsaAgent::clearTrace(int f)
{
  if (f > RL_MEMORY_SIZE || f < 0)
    cerr << "ClearTrace: f out of range " << f << endl;
  if (traces[f] != 0)
    clearExistentTrace(f, nonzeroTracesInverse[f]);
}

// Clear the trace for feature f at location loc in the list of nonzero traces
void LinearSarsaAgent::clearExistentTrace(int f, int loc)
{
  if (f > RL_MEMORY_SIZE || f < 0)
    cerr << "ClearExistentTrace: f out of range " << f << endl;
  traces[f] = 0.0;
  numNonzeroTraces--;
  nonzeroTraces[loc] = nonzeroTraces[numNonzeroTraces];
  nonzeroTracesInverse[nonzeroTraces[loc]] = loc;
}

// Decays all the (nonzero) traces by decay_rate, removing those below minimum_trace
void LinearSarsaAgent::decayTraces(double decayRate)
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
void LinearSarsaAgent::setTrace(int f, float newTraceValue)
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
void LinearSarsaAgent::increaseMinTrace()
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

void LinearSarsaAgent::setParams(int iCutoffEpisodes, int iStopLearningEpisodes)
{
  /* set learning parameters */
}
