//
// Created by baj on 10/3/16.
//

#ifndef KEEPAWAY_PLAYER_HIERARCHICALFSM_H
#define KEEPAWAY_PLAYER_HIERARCHICALFSM_H

#include <string>
#include <stack>
#include <vector>
#include "BasicPlayer.h"
#include "LinearSarsaLearner.h"

namespace fsm {
class HierarchicalFSM;
}

namespace std {

const string &to_string(fsm::HierarchicalFSM *m);

}

namespace fsm {

template<class T>
class ChoicePoint;

/**
 * global memory among all machines from the pespective of a single machine
 */
class Memory {
private:
  Memory();

public:
  static Memory &ins();

  void resetState();

  std::string to_string();

  bool bAlive;
  double state[MAX_RL_STATE_VARS]; // current state
  ObjectT K[11]; // current mapping from index to players
  std::vector<string> stack; // self call stack
};

/**
 * hierarchical finite state machines
 */
class HierarchicalFSM {
  template<class T> friend
  class ChoicePoint;

  template<class T> friend
  class MakeChoice;

  friend class Run;

  friend class LinearSarsaLearner;

public:
  HierarchicalFSM(BasicPlayer *p, const std::string name);

  virtual ~HierarchicalFSM();

private:
  virtual void run() = 0;

protected:
  /**
   * send commands to env, and wait for new info
   */
  void action(bool sync = true);

  std::string getState();

  void idle(const std::string error);

  bool running();

  static std::string getStackStr();

protected:
  BasicPlayer *player;
  const std::string name;

public:
  const std::string &getName() const;

  static void initialize(int numFeatures,
                         int numKeepers,
                         bool bLearn,
                         double widths[],
                         double gamma,
                         double initialWeight);

protected:
  ActHandler *ACT; /*!< ActHandler to which commands can be sent        */
  WorldModel *WM;  /*!< WorldModel that contains information of world   */
  ServerSettings *SS;  /*!< All parameters used by the server               */
  PlayerSettings *PS;  /*!< All parameters used for the player              */
  ChoicePoint<int> *dummyChoice;

public:
  static int num_features;
  static int num_keepers;
  static double gamma;
};

class Keeper : public HierarchicalFSM {
public:
  Keeper(BasicPlayer *p);

  ~Keeper();

  virtual void run();

private:
  ChoicePoint<HierarchicalFSM *> *choices[2];
  HierarchicalFSM *pass;
  HierarchicalFSM *hold;
  HierarchicalFSM *move;
  HierarchicalFSM *stay;
  HierarchicalFSM *intercept;
};

class Move : public HierarchicalFSM {
public:
  Move(BasicPlayer *p);

  virtual ~Move();

  virtual void run();

private:
  ChoicePoint<int> *moveToChoice;
};

class Stay : public HierarchicalFSM {
public:
  Stay(BasicPlayer *p);

  virtual void run();
};

class Intercept : public HierarchicalFSM {
public:
  Intercept(BasicPlayer *p);

  virtual void run();
};

class Pass : public HierarchicalFSM {
public:
  Pass(BasicPlayer *p);

  virtual ~Pass();

  virtual void run();

private:
  ChoicePoint<int> *passToChoice;
};

class Hold : public HierarchicalFSM {
public:
  Hold(BasicPlayer *p);

  virtual void run();
};

/**
 * Make choice while taking care of call stack
 */
template<class T>
class MakeChoice {
public:
  MakeChoice(ChoicePoint<T> *cp) : c(cp->choose()) {
    Memory::ins().stack.push_back("#" + to_string(c));
    if (Log.isInLogLevel(101)) {
      stringstream ss;
      PRINT_VALUE_STREAM(ss, Memory::ins().stack);
      Log.logWithTime(101, "stack push:\n%s", ss.str().c_str());
    }
  }

  T operator()() {
    return c;
  }

  ~MakeChoice() {
    Memory::ins().stack.pop_back();
    if (Log.isInLogLevel(101)) {
      stringstream ss;
      PRINT_VALUE_STREAM(ss, Memory::ins().stack);
      Log.logWithTime(101, "stack pop:\n%s", ss.str().c_str());
    }
  }

private:
  T c;
};

/**
 * run a child machine while taking care of call stack
 */
class Run {
public:
  Run(HierarchicalFSM *m) : m(m) {
    Memory::ins().stack.push_back(m->getName());
    if (Log.isInLogLevel(101)) {
      stringstream ss;
      PRINT_VALUE_STREAM(ss, Memory::ins().stack);
      Log.logWithTime(101, "stack push:\n%s", ss.str().c_str());
    }
  }

  void operator()() {
    m->run();
  }

  ~Run() {
    Memory::ins().stack.pop_back();
    if (Log.isInLogLevel(101)) {
      stringstream ss;
      PRINT_VALUE_STREAM(ss, Memory::ins().stack);
      Log.logWithTime(101, "stack pop:\n%s", ss.str().c_str());
    }
  }

private:
  HierarchicalFSM *m;
};

}


#endif //KEEPAWAY_PLAYER_HIERARCHICALFSM_H
