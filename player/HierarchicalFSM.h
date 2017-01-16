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
  int agentIdx;
  double state[MAX_RL_STATE_VARS]; // current state -- indexed by K0..Kn
  ObjectT teammates[11]; // current mapping from index to teammates
  ObjectT opponents[11]; // current mapping from index to opponents

private:
  std::vector<string> stack;  // self call stack

public:
  const vector<string> &getStack() const;

  void PushStack(const string &s);

  void PopStack();
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

  friend class Action;

  friend class LinearSarsaLearner;

public:
  HierarchicalFSM(BasicPlayer *p, const std::string &name);

  virtual ~HierarchicalFSM();

private:
  virtual void run() = 0;

protected:
  /**
   * send commands to env, and wait for new info
   */
  void action(bool sync = true);

  bool isFastestToBall();

  VecPosition refineTarget(VecPosition target, VecPosition backup);

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
                         int numTeammates,
                         int numOpponents,
                         bool bLearn,
                         double widths[],
                         double gamma,
                         double lambda,
                         double initialWeight,
                         bool qLearning,
                         bool UseStaticTransition,
                         string loadWeightsFile,
                         string saveWeightsFile,
                         string teamName);

protected:
  ActHandler *ACT; /*!< ActHandler to which commands can be sent        */
  WorldModel *WM;  /*!< WorldModel that contains information of world   */
  ServerSettings *SS;  /*!< All parameters used by the server               */
  PlayerSettings *PS;  /*!< All parameters used for the player              */
  ChoicePoint<int> *dummyChoice;

public:
  static int num_features;
  static int num_teammates;
  static int num_opponents;
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
  HierarchicalFSM *getopen;
};

class Taker : public HierarchicalFSM {
public:
  Taker(BasicPlayer *p);

  ~Taker();

  virtual void run();

private:
  ChoicePoint<HierarchicalFSM *> *choice_taker;

  HierarchicalFSM *hold;
  HierarchicalFSM *stay;
  HierarchicalFSM *move;
  HierarchicalFSM *intercept;
};

class Move : public HierarchicalFSM {
public:
  Move(BasicPlayer *p);

  virtual ~Move();

  virtual void run();

private:
  ChoicePoint<int> *moveToChoice;
//  ChoicePoint<double> *moveSpeedChoice;
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
  ChoicePoint<PassT> *passSpeedChoice;
};

class Hold : public HierarchicalFSM {
public:
  Hold(BasicPlayer *p);

  ~Hold();

  virtual void run();
};

class GetOpen : public HierarchicalFSM {
public:
  GetOpen(BasicPlayer *p);

  virtual void run();
};

}


#endif //KEEPAWAY_PLAYER_HIERARCHICALFSM_H
