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

  // Saving weights (including for hive mind) requires learning and a file name.
  this->hiveMind = false;
  if (bLearning && strlen(saveWeightsFile.c_str()) > 0)
  {
    strcpy(weightsFile, saveWeightsFile.c_str());
    bSaveWeights = true;
    // Hive mind further requires loading and saving from the same file.
    if (!strcmp(loadWeightsFile.c_str(), saveWeightsFile.c_str()))
    {
      this->hiveMind = hiveMind;
    }
  }
  else
  {
    bSaveWeights = false;
  }

  alpha = 0.125;
  gamma = 1.0;
  lambda = 0;
  epsilon = 0.01;
  minimumTrace = 0.01;
  sharedData = 0;
  initialWeight = 0.0;
  bLearning = false;
  qLearning = false;
  lastJointChoiceIdx = -1;
  lastJointChoiceTime = UnknownTime;

  epochNum = 0;
  lastAction = -1;

  numNonzeroTraces = 0;
  weights = weightsRaw;

  for (int i = 0; i < RL_MEMORY_SIZE; i++)
  {
    weights[i] = 0;
    traces[i] = 0;
  }

  // srand((unsigned int)0);
  // int tmp[2];
  // float tmpf[2];
  // colTab = new collision_table(RL_MEMORY_SIZE, 1);

  // GetTiles(tmp, 1, 1, tmpf, 0); // A dummy call to set the hashing table
  // srand(time(NULL));

  // if (strlen(loadWeightsFile.c_str()) > 0)
  //   loadWeights(loadWeightsFile.c_str());

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
    exepath += "LinearSarsaLearner::LinearSarsaAgent";
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

    // Q = sharedData->Q;
    // tiles = sharedData->tiles;
    // weights = sharedData->weights;
    // traces = sharedData->traces;
    // nonzeroTraces = sharedData->nonzeroTraces;
    // nonzeroTracesInverse = sharedData->nonzeroTracesInverse;
    // colTab = &sharedData->colTab;
    // num_teammates = sharedData->num_teammates;
    // num_opponents = sharedData->num_opponents;

    barriers["enter1"] = new Barrier(num_teammates, h, "enter1");
    barriers["exit1"] = new Barrier(num_teammates, h, "exit1");
    barriers["enter2"] = new Barrier(num_teammates, h, "enter2");
    barriers["exit2"] = new Barrier(num_teammates, h, "exit2");
    barriers["reset"] = new Barrier(num_teammates, h, "reset");

    sharedData->reset();
    // if (loadWeightsFile.empty() || !loadWeights(loadWeightsFile.c_str()))
    // {
    //   fill(weights, weights + RL_MEMORY_SIZE, initialWeight);
    //   colTab->reset();
    // }
    loadWeights(loadWeightsFile.c_str());
  }
}

bool LinearSarsaAgent::loadSharedData()
{
  numTilings = sharedData->numTilings;
  minimumTrace = sharedData->minimumTrace;
  numNonzeroTraces = sharedData->numNonzeroTraces;
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

  for (int i = 0; i < num_teammates; ++i)
  {
    // sharedData->numChoices[Memory::ins().teammates[i]] = numChoices[i];
    // strcpy(sharedData->machineState[Memory::ins().teammates[i]],
    //        machineState[i].c_str());
    // strcpy(sharedData->lastMachineState[i], lastMachineState[i].c_str());
    // sharedData->lastJointChoice[i] = lastJointChoice[i];
  }
}

std::stack<std::tuple<int, int>> LinearSarsaAgent::graph()
{
  std::stack<std::tuple<int, int>> cordGraph;

  for (int i = 0; i < num_teammates; ++i)
  {
    int j = (i + 1) % num_teammates;
    cordGraph.push(std::make_tuple(i, j));
  }

  return cordGraph;
}

double LinearSarsaAgent::getQ(int action)
{
  if (action < 0 || action > getNumActions())
  {
    throw "invalid action";
  }
  cout << "getQ " << Q[action];
  return Q[action];
}

void LinearSarsaAgent::setEpsilon(double epsilon)
{
  this->epsilon = epsilon;
}

int LinearSarsaAgent::startEpisode(double state[])
{
  if (hiveMind)
    loadColTabHeader(colTab, weights);

  epochNum++;
  decayTraces(0);
  loadTiles(state);
  for (int a = 0; a < getNumActions(); a++)
  {
    Q[a] = computeQ(a);
  }

  lastAction = selectAction();
  // Log.log("Right before making the weightToString call", lastAction.c_str());

  char buffer[128];
  sprintf(buffer, "Q[%d] = %.2f", lastAction, Q[lastAction]);
  LogDraw.logText("Qmax", VecPosition(25, -30),
                  buffer,
                  1, COLOR_BROWN);

  for (int j = 0; j < numTilings; j++)
    setTrace(tiles[lastAction][j], 1.0);
  if (hiveMind)
    saveWeights(weightsFile);
  return lastAction;
}

// int LinearSarsaAgent::step(double state[])
// {
// }

// Dummy Q functions for the example
// double Q1(int a1, int a2)
// {
// }
// double Q2(int a2, int a4)
// {
//   double val = -1e9;

//   for (int a2 = 0; a2 < getNumActions(); a2++)
//   {
//     auto val2 = Q[a2];
//     if (val2 > val)
//     {
//       val = val2;
//     }
//   }

//   return val + computeQ(a4);
// }
// double Q3(int a1, int a3) { return a1 + a3; }

// double Q4(int a3, int a4)
// {
//   double val = -1e9;

//   for (int a3 = 0; a3 < getNumActions(); a3++)
//   {
//     auto val3 = Q[a3];
//     if (val3 > val)
//     {
//       val = val3;
//     }
//   }

//   return val + computeQ(a4);
// }

// std::tuple<double, int> e4(int a2, int a3)
// {
//   double maxVal = -1e9;
//   int argmax_a4 = -1;
//   for (int a4 = 0; a4 < getNumActions(); a4++)
//   {
//     double current = Q2(a2, a4) + Q4(a3, a4);

//     if (current > maxVal)
//     {
//       maxVal = current;
//       argmax_a4 = a4;
//     }
//   }
//   return {maxVal, argmax_a4};
// }

// std::tuple<double, int> e3(int a1, int a2)
// {
//   double maxVal = -1e9;
//   int argmax_a3 = -1;
//   for (int a3 = 0; a3 <= 10; a3++)
//   {
//     double current = Q3(a1, a3) + std::get<0>(e4(a2, a3));
//     if (current > maxVal)
//     {
//       maxVal = current;
//       argmax_a3 = a3;
//     }
//   }
//   return {maxVal, argmax_a3};
// }

// std::tuple<double, int> e2(int a1)
// {
//   double maxVal = -1e9;
//   int argmax_a2 = -1;
//   for (int a2 = 0; a2 <= 10; a2++)
//   {
//     double current = Q1(a1, a2) + std::get<0>(e3(a1, a2));
//     if (current > maxVal)
//     {
//       maxVal = current;
//       argmax_a2 = a2;
//     }
//   }
//   return {maxVal, argmax_a2};
// }

// double computeE1(int &optimal_a1)
// {
//   double maxVal = -1e9;
//   for (int a1 = 0; a1 <= 10; a1++)
//   {
//     double current = std::get<0>(e2(a1));
//     if (current > maxVal)
//     {
//       maxVal = current;
//       optimal_a1 = a1;
//     }
//   }
//   return maxVal;
// }

// int LinearSarsaAgent::step(int current_time, int action)
// {
// }

int LinearSarsaAgent::step(double reward, double state[])
{

  if (hiveMind)
    loadColTabHeader(colTab, weights);

  double delta = reward - Q[lastAction];
  loadTiles(state);
  cout << " Memory::ins().agentIdx " << Memory::ins().agentIdx << endl;

  if (Memory::ins().agentIdx == num_teammates - 1) // T3.
  {
    for (int a = 0; a < getNumActions(); a++)
    {
      Q[a] = computeQ(a);
      // sharedData->localValuesMap[Memory::ins().agentIdx][a] = computeQ(a);
      cout << "Q[" << a << "] = " << Q[a] << endl;
    }
  }
  else if (Memory::ins().agentIdx == num_teammates - 2) // 2 .
  {
    for (int a = 0; a < getNumActions(); a++)
    {
      Q[a] = computeQ(a);
      cout << "Q[" << a << "] = " << Q[a] << endl;
      // sharedData->localValuesMap[Memory::ins().agentIdx][a] = computeQ(a);
    }
  }
  else
  {
    for (int a = 0; a < getNumActions(); a++)
    {
      Q[a] = computeQ(a);
      cout << "Q[" << a << "] = " << Q[a] << endl;
      // localValuesMap[Memory::ins().agentIdx][a] = computeQ(a);
      // sharedData->localValuesMap[Memory::ins().agentIdx][a] = computeQ(a);
    }
  }

  // // // coordination graph..
  // std::stack<std::tuple<int, int>> g = graph();
  // // // Write algorithm for order of elimination. just pop for now

  // // // std::stack<std::tuple<int, int>> ;
  // std::stack<std::set<int>> factors;
  // std::set<int> s = {}; // clean up later
  // double Q1, Q2, Q3, Q4, e4;

  // // // Right now, we are starting from the last keeper.
  // if (Memory::ins().agentIdx == num_teammates) // T3.
  // {
  //   cout << " Memory::ins().agentIdx " << endl;
  //   cout << Memory::ins().agentIdx << endl;
  //   // while (!g.empty())
  //   // {
  //   //   auto [i, j] = g.top();
  //   //   if (i == Memory::ins().agentIdx || j == Memory::ins().agentIdx)
  //   //   {
  //   //     g.pop();
  //   //     Q4 = calculateQ(i, j, reward, state);
  //   //     if (i != Memory::ins().agentIdx)
  //   //       s.insert(i);
  //   //     else if (j != Memory::ins().agentIdx)
  //   //       s.insert(j);
  //   //     factors.push(s);
  //   //   }
  //   // }
  // }
  // else if (Memory::ins().agentIdx == num_teammates - 1) // 2 .
  // {
  //   cout << " Memory::ins().agentIdx " << Memory::ins().agentIdx << endl;
  //   // while (!g.empty())
  //   // {
  //   //   auto [i, j] = g.top();
  //   //   std::set<int> top_set = factors.top();
  //   //   if (i == Memory::ins().agentIdx || j == Memory::ins().agentIdx)
  //   //   {
  //   //     g.pop();
  //   //     Q3 = calculateQ(i, j, reward, state);
  //   //     // e4 =
  //   //   }
  //   // }
  // }

  // // for (int a = 0; a < getNumActions(); a++)
  // // {
  // //   Q[a] = computeQ(a);
  // // }

  lastAction = selectAction();

  char buffer[128];
  sprintf(buffer, "Q[%d] = %.2f", lastAction, Q[lastAction]);
  LogDraw.logText("Qmax", VecPosition(25, -30),
                  buffer,
                  1, COLOR_BROWN);

  if (!bLearning)
    return lastAction;

  // char buffer[128];
  sprintf(buffer, "reward: %.2f", reward);
  LogDraw.logText("reward", VecPosition(25, 30),
                  buffer,
                  1, COLOR_NAVY);

  delta += Q[lastAction];
  updateWeights(delta);
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

void LinearSarsaAgent::endEpisode(double reward)
{
  if (hiveMind)
    loadColTabHeader(colTab, weights);
  if (bLearning && lastAction != -1)
  { /* otherwise we never ran on this episode */
    char buffer[128];
    sprintf(buffer, "reward: %.2f", reward);
    LogDraw.logText("reward", VecPosition(25, 30),
                    buffer,
                    1, COLOR_NAVY);

    /* finishing up the last episode */
    /* assuming gamma = 1  -- if not,error*/
    if (gamma != 1.0)
      cerr << "We're assuming gamma's 1" << endl;
    double delta = reward - Q[lastAction];
    updateWeights(delta);
    // TODO Actually, there's still possibly risk for trouble here with multiple
    // TODO players stomping each other. Is this okay?
    // TODO The weight updates themselves are in order.
  }
  if (bLearning && bSaveWeights && rand() % 200 == 0 && !hiveMind)
  {
    saveWeights(weightsFile);
  }
  if (hiveMind)
    saveWeights(weightsFile);
  lastAction = -1;
}

void LinearSarsaAgent::shutDown()
{
  // We usually only save weights at random intervals.
  // Always save at shutdown (if we are in saving mode).

  // if (Memory::ins().agentIdx == 0)
  // {
  //   if (bLearning && bSaveWeights)
  //   {
  //     cerr << "Saving weights at shutdown." << endl;
  //     saveWeights(saveWeightsFile.c_str());
  //   }
  // }

  // if (sharedData)
  //   shm_unlink(sharedMemory.c_str());
  // for (auto pa : barriers)
  //   delete pa.second;

  if (bLearning && bSaveWeights)
  {
    cout << "Saving weights at shutdown." << endl;
    saveWeights(weightsFile);
  }
  // Also shut down the hive mind if needed.
  if (hiveMind)
  {
    size_t mapLength =
        RL_MEMORY_SIZE * sizeof(double) +
        sizeof(CollisionTableHeader) +
        colTab->m * sizeof(long);
    munmap(weights, mapLength);
    close(hiveFile);
    hiveFile = -1;
    // Go back to the own arrays, since our map is no longer valid.
    weights = weightsRaw;
    colTab->data = new long[colTab->m];
  }
}

int LinearSarsaAgent::selectAction()
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

  // cout << "This action " << endl;
  // cout << action << endl;

  return action;
}

bool LinearSarsaAgent::loadWeights(const char *filename)
{
  cout << "Linear SARSA Loading weights from " << filename << endl;
  if (hiveMind)
  {
    if (hiveFile < 0)
    {
      // First, check the lock file, so we have only one initializer.
      // Later interaction should be approximately synchronized by having only
      // one active player at a time per team, but we can't assume that here.
      stringstream lockNameBuffer;
      lockNameBuffer << filename << ".lock";
      const char *lockName = lockNameBuffer.str().c_str();
      int lock;
      // 10ms delay (times a million to convert from nanos).
      timespec sleepTime = {0, 10 * 1000 * 1000};
      while (true)
      {
        lock = open(lockName, O_CREAT | O_EXCL, 0664);
        if (lock >= 0)
          break;
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
      if (!fileFound)
      {
        // Make the file the right size.
        cout << "Initializing new hive file." << endl;
        if (lseek(hiveFile, mapLength - 1, SEEK_SET) < 0)
        {
          throw "failed to seek initial file size";
        }
        if (write(hiveFile, "", 1) < 0)
        {
          throw "failed to expand initial file";
        }
      }
      if (hiveFile < 0)
        throw "failed to open hive file";
      void *hiveMap =
          mmap(NULL, mapLength, PROT_READ | PROT_WRITE, MAP_SHARED, hiveFile, 0);
      if (hiveMap == MAP_FAILED)
        throw "failed to map hive file";
      // First the weights.
      weights = reinterpret_cast<double *>(hiveMap);
      // Now the collision table header.
      CollisionTableHeader *colTabHeader =
          reinterpret_cast<CollisionTableHeader *>(weights + RL_MEMORY_SIZE);
      if (fileFound)
      {
        loadColTabHeader(colTab, weights);
      }
      // Now the collision table data.
      delete[] colTab->data;
      colTab->data = reinterpret_cast<long *>(colTabHeader + 1);
      if (!fileFound)
      {
        // Clear out initial contents.
        // The whole team might be doing this at the same time. Is that okay?
        for (int i = 0; i < RL_MEMORY_SIZE; i++)
        {
          weights[i] = 0;
        }
        colTab->reset();
        // Make sure the header goes out to the file.
        saveWeights(weightsFile);
      }
      // TODO Separate file lock type with destructor?
      unlink(lockName);
    }
  }
  else
  {
    int file = open(filename, O_RDONLY);
    read(file, (char *)weights, RL_MEMORY_SIZE * sizeof(double));
    colTab->restore(file);
    close(file);
  }
  cout << "...done" << endl;
  return true;
}

// bool LinearSarsaAgent::loadWeights(const char *filename)
// {
//   FileLock lock("loadWeights");
//   cerr << "Loading weights from " << filename << endl;

// #ifdef _Compress
//   igzstream is;
// #else
//   ifstream is;
// #endif

//   is.open(filename);
//   if (!is.good())
//   {
//     cerr << "failed to open weight file: " << filename << endl;
//     return false;
//   }

//   is.read((char *)weights, RL_MEMORY_SIZE * sizeof(double));
//   colTab->restore(is);
//   is.close();
//   cerr << "...done" << endl;
//   return true;
// }

// bool LinearSarsaAgent::saveWeights(const char *filename)
// {
//   FileLock lock("saveWeights");

// #ifdef _Compress
//   ogzstream os;
// #else
//   ofstream os;
// #endif

//   os.open(filename);
//   if (!os.good())
//   {
//     cerr << "failed to open weight file: " << filename << endl;
//     return false;
//   }

//   os.write((char *)weights, RL_MEMORY_SIZE * sizeof(double));
//   colTab->save(os);
//   os.close();
//   return true;
// }

bool LinearSarsaAgent::saveWeights(const char *filename)
{
  if (hiveMind)
  {
    // The big arrays should be saved out automatically, but we still need to
    // handle the collision table header.
    CollisionTableHeader *colTabHeader =
        reinterpret_cast<CollisionTableHeader *>(weights + RL_MEMORY_SIZE);
    // Do each field individually, since they don't all line up exactly for an
    // easy copy.
    colTabHeader->calls = colTab->calls;
    colTabHeader->clearhits = colTab->clearhits;
    colTabHeader->collisions = colTab->collisions;
    colTabHeader->m = colTab->m;
    colTabHeader->safe = colTab->safe;
    if (VERBOSE_HIVE_MIND)
    {
      cout << "Saved colTabHeader:" << endl
           << " calls: " << colTab->calls << endl
           << " clearhits: " << colTab->clearhits << endl
           << " collisions: " << colTab->collisions << endl
           << " m: " << colTab->m << endl
           << " safe: " << colTab->safe << endl;
    }
  }
  else
  {
    int file = open(filename, O_CREAT | O_WRONLY, 0664);
    write(file, (char *)weights, RL_MEMORY_SIZE * sizeof(double));
    colTab->save(file);
    close(file);
  }
  return true;
}

// Compute an action value from current F and theta
double LinearSarsaAgent::computeQ(int a)
{
  double q = 0;
  for (int j = 0; j < numTilings; j++)
  {
    q += weights[tiles[a][j]];
  }

  return q;
}

// Returns index (action) of largest entry in Q array, breaking ties randomly
int LinearSarsaAgent::argmaxQ()
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

void LinearSarsaAgent::updateWeights(double delta)
{
  double tmp = delta * alpha / numTilings;
  for (int i = 0; i < numNonzeroTraces; i++)
  {
    int f = nonzeroTraces[i];
    if (f > RL_MEMORY_SIZE || f < 0)
      cerr << "f is too big or too small!!" << f << endl;
    weights[f] += tmp * traces[f];
    // cout << "weights[" << f << "] = " << weights[f] << endl;
  }
}

void LinearSarsaAgent::loadTiles(double state[])
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
  // return numTilings;
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
