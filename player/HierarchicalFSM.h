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

string to_string(fsm::HierarchicalFSM *m);

}

namespace fsm {

template<class T>
class ChoicePoint;

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
  HierarchicalFSM(BasicPlayer *p, const std::string name) :
      player(p), name(name) {
    ACT = player->ACT;
    WM = player->WM;
    SS = player->SS;
    PS = player->PS;
  }

  virtual ~HierarchicalFSM() {

  }

private:
  virtual void run() = 0;

protected:
  /**
   * send commands to env, and wait for new info
   */
  void action();

  std::string getState();

  void idle(const std::string error);

  static void reset();

  static void choicePoint(int choice);

  bool tmControllBall();

  bool running();

  static std::string getCallStackStr();

protected:
  BasicPlayer *player;
  const std::string name;

public:
  const std::string &getName() const;

protected:
  ActHandler *ACT; /*!< ActHandler to which commands can be sent        */
  WorldModel *WM;  /*!< WorldModel that contains information of world   */
  ServerSettings *SS;  /*!< All parameters used by the server               */
  PlayerSettings *PS;  /*!< All parameters used for the player              */

public:
  static double state[MAX_RL_STATE_VARS]; // current state
  static ObjectT K[11]; // current mapping from index to players
  static std::vector<string> stack; // call stack
  static int agentIdx; // agent index
  static bool bAlive;

  static double lastState[MAX_RL_STATE_VARS]; // last choice state
  static int lastChoice; // index of last choice
  static std::vector<string> lastChoiceStack; // last choice stack
  static double cumulativeReward; // cumulative reward since last choice
  static double cumulativeGamma; // cumulative discount
  static LinearSarsaLearner *learner;

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
  ChoicePoint<int> *moveTo;
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
  ChoicePoint<int> *passTo;
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
    HierarchicalFSM::stack.push_back("#" + to_string(c));
  }

  T operator()() {
    return c;
  }

  ~MakeChoice() {
    HierarchicalFSM::stack.pop_back();
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
    HierarchicalFSM::stack.push_back(m->getName());
  }

  void operator()() {
    m->run();
  }

  ~Run() {
    HierarchicalFSM::stack.pop_back();
  }

private:
  HierarchicalFSM *m;
};

}


#endif //KEEPAWAY_PLAYER_HIERARCHICALFSM_H
