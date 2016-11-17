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
#include "prettyprint.h"

namespace fsm {
class HierarchicalFSM;
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

  std::string getState();

  void idle(const std::string error);

  bool running();

  bool isFastestToBall();

  VecPosition refineTarget(VecPosition target, VecPosition backup);

  static std::string getStackStr();

protected:
  BasicPlayer *player;
  const std::string name;

public:
  const std::string &getName() const;

  static void initialize(int numFeatures,
                         int numKeepers,
                         int numTakers,
                         bool bLearn,
                         double widths[],
                         double gamma,
                         double lambda,
                         double initialWeight,
                         bool qLearning,
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
  static int numFeatures;
  static int numTeammates;
  static int numOpponents;
};

class Keeper : public HierarchicalFSM {
public:
  Keeper(BasicPlayer *p);

  ~Keeper();

  virtual void run();

private:
  ChoicePoint<HierarchicalFSM *> *choice_ball;
  ChoicePoint<HierarchicalFSM *> *choice_free;

  HierarchicalFSM *pass;
  HierarchicalFSM *pass_random;
  HierarchicalFSM *dribble;
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
  ChoicePoint<HierarchicalFSM *> *choice_free;
  ChoicePoint<bool> *choice_tackle;

  HierarchicalFSM *hold;
  HierarchicalFSM *tackle;
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
  ChoicePoint<double> *moveSpeedChoice;
};

class Stay : public HierarchicalFSM {
public:
  Stay(BasicPlayer *p);

  virtual void run();
};

class GetOpen : public HierarchicalFSM {
public:
  GetOpen(BasicPlayer *p);

  virtual void run();
};

class Mark : public HierarchicalFSM {
public:
  Mark(BasicPlayer *p);

  ~Mark();

  virtual void run();

private:
  ChoicePoint<int> *markToChoice;
};

class Intercept : public HierarchicalFSM {
public:
  Intercept(BasicPlayer *p, bool tackle = false);

  virtual void run();

private:
  bool tackle;
};

class Pass : public HierarchicalFSM {
public:
  Pass(BasicPlayer *p, bool random = false);

  virtual ~Pass();

  virtual void run();

  void passTo(int tm, PassT speed);

private:
  bool random;
  ChoicePoint<int> *passToChoice;
  ChoicePoint<PassT> *passSpeedChoice;
};

class Dribble : public HierarchicalFSM {
public:
  Dribble(BasicPlayer *p);

  virtual ~Dribble();

  virtual void run();

private:
  ChoicePoint<double> *dribbleToChoice;
  ChoicePoint<DribbleT> *dribbleSpeedChoice;
};

class Hold : public HierarchicalFSM {
public:
  Hold(BasicPlayer *p);

  ~Hold();

  virtual void run();
};

class Tackle : public HierarchicalFSM {
public:
  Tackle(BasicPlayer *p);

  ~Tackle();

  virtual void run();
};

}

namespace std {
template<>
inline string to_prettystring(fsm::HierarchicalFSM *&o) {
  stringstream ss;
  ss << o->getName();
  return ss.str();
}

template<>
inline string to_prettystring(const fsm::HierarchicalFSM *&o) {
  stringstream ss;
  ss << o->getName();
  return ss.str();
}
}

#endif //KEEPAWAY_PLAYER_HIERARCHICALFSM_H
