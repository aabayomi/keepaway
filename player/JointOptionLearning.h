//
// Created by baj on 10/3/16.
//

#ifndef KEEPAWAY_PLAYER_JOINTACTIONSPACE_H
#define KEEPAWAY_PLAYER_JOINTACTIONSPACE_H

#include "BasicPlayer.h"
#include "SoccerTypes.h"

namespace jol {

enum AtomicActionType {
  AAT_None,
  AAT_Hold,
  AAT_PassTo,
  AAT_Intercept,
  AAT_Stay,
  AAT_Move // Left, Right, In, Out
};

std::ostream &operator<<(std::ostream &out, const AtomicActionType value);

struct AtomicAction {
  AtomicActionType type;
  int parameter;

  AtomicAction(AtomicActionType t = AAT_None, int p = 0) : type(t), parameter(p) {}

  virtual std::string name() const {
    stringstream ss;
    ss << type << "_" << parameter;
    return ss.str();
  }

  virtual std::vector<int> parameters() { return {0}; }

  virtual bool terminated(BasicPlayer *player, ObjectT K[]);

  virtual SoccerCommand execute(BasicPlayer *player, ObjectT K[]) = 0;

  virtual AtomicAction *clone(int parameter) = 0;

  static int num_keepers;
};

#define CLONE(cls) \
  virtual AtomicAction *clone(int parameter) { \
    auto a = new cls(); \
    a->parameter = parameter; \
    return a; \
  }

struct Hold : public AtomicAction {
  Hold() : AtomicAction(AAT_Hold) {}

  virtual bool terminated(BasicPlayer *player, ObjectT K[]);

  virtual SoccerCommand execute(BasicPlayer *player, ObjectT K[]);

  CLONE(Hold)
};

/**
 * PassTo is composed of passto(k) and move(d)/stay
 */
struct PassTo : public AtomicAction {
  PassTo() : AtomicAction(AAT_PassTo) {}

  virtual std::vector<int> parameters();

  virtual bool terminated(BasicPlayer *player, ObjectT K[]);

  virtual SoccerCommand execute(BasicPlayer *player, ObjectT K[]);

  virtual std::string name() const;

  int k() const { return parameter / 5 + 1; }

  int d() const { return parameter % 5; }

  CLONE(PassTo)
};

struct Intercept : public AtomicAction {
  Intercept() : AtomicAction(AAT_Intercept) {}

  virtual bool terminated(BasicPlayer *player, ObjectT K[]);

  virtual SoccerCommand execute(BasicPlayer *player, ObjectT K[]);

  CLONE(Intercept)
};

struct Stay : public AtomicAction {
  Stay() : AtomicAction(AAT_Stay) {}

  virtual SoccerCommand execute(BasicPlayer *player, ObjectT K[]);

  CLONE(Stay)
};

struct Move : public AtomicAction {
  Move(int p = 0) : AtomicAction(AAT_Move, p) {}

  virtual std::vector<int> parameters();

  virtual SoccerCommand execute(BasicPlayer *player, ObjectT K[]);

  CLONE(Move);
};

struct JointAction {
  std::vector<AtomicAction *> actions;
  int id;
  bool tmControllBall;

  JointAction() : id(0), tmControllBall(false) {
    actions.resize((std::size_t) AtomicAction::num_keepers);
  }

  JointAction(const JointAction &ja) :
      actions(ja.actions),
      id(ja.id),
      tmControllBall(ja.tmControllBall) {}

  const JointAction &operator=(const JointAction &ja) {
    if (this != &ja) {
      actions = ja.actions;
      id = ja.id;
      tmControllBall = ja.tmControllBall;
    }
    return *this;
  }

  bool has_intercept(int k) {
    for (int i = 0; i < k; ++i) {
      if (actions[i]->type == AAT_Intercept) return true;
    }
    return false;
  }

  std::string name() const {
    stringstream ss;
    ss << "{";
    for (auto a : actions) ss << a->name() << ", ";
    ss << "}";
    return ss.str();
  }
};

/**
 * high-level joint options
 */
class JointActionSpace {
private:
  JointActionSpace();

  void construct(
      bool tmControlBall, int k,
      std::vector<std::vector<AtomicAction *>> &actions, JointAction &ja);

public:
  static JointActionSpace &ins() {
    static JointActionSpace jas;
    return jas;
  }

  int sample(bool tmControlBall) const {
    return jointActions[tmControlBall][rand() % jointActions[tmControlBall].size()]->id;
  }

  int sample(double state[], int num_features) const {
    bool tmControllBall = state[num_features - 1] > 0.5;
    return sample(tmControllBall);
  }

  int numActions() const { return count; }

  std::string to_string() {
    std::stringstream ss;
    for (int i = 0; i < count; ++i) {
      ss << "id: " << i << " name: " << getJointActionName(i) << std::endl;
    }
    return ss.str();
  }

  const JointAction *getJointAction(int id) {
    if (id >= 0 && id < count) return jaMap[id].first;
    return 0;
  }

  const char *getJointActionName(int id) {
    if (id >= 0 && id < count) return jaMap[id].second.c_str();
    return 0;
  }

private:
  std::vector<JointAction *> jointActions[2];
  std::pair<JointAction *, std::string> jaMap[MAX_RL_ACTIONS];
  int count;
};

}

#endif //KEEPAWAY_PLAYER_JOINTACTIONSPACE_H
