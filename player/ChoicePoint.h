//
// Created by baj on 10/4/16.
//

#ifndef KEEPAWAY_PLAYER_CHOICEPOINT_H
#define KEEPAWAY_PLAYER_CHOICEPOINT_H

#include "HierarchicalFSM.h"
#include <cstring>
#include <functional>
#include <string>
#include <vector>

namespace fsm {

template<class T>
class ChoicePoint {
public:
  ChoicePoint(const std::string &name, const std::vector<T> c)
      : name(name), choices(c) {}

  const string &getName() const { return name; }

  T choose(int current_time) {
    Log.log(101, "ChoicePoint::choose point name: %s", name.c_str());
    auto i = LinearSarsaLearner::ins().step(current_time, (int) choices.size());

    if (i < choices.size()) {
      Log.log(101, "ChoicePoint::choose my choice (agent %d): %s",
              Memory::ins().agentIdx, to_string(choices[i]).c_str());
    } else { // race condition?
      Log.log(101, "ChoicePoint::choose i %d >= choices.size() %d", i,
              choices.size());
    }

    return i < choices.size() ? choices[i] : choices[0];
  }

private:
  std::string name;
  std::vector<T> choices;
};

/**
 * Make choice while taking care of call stack
 */
template<class T>
class MakeChoice {
public:
  MakeChoice(ChoicePoint<T> *cp) : cp(cp) {
    Log.log(101, "MakeChoice::MakeChoice %s", cp->getName().c_str());
    Memory::ins().PushStack(cp->getName());
  }

  T operator()(int current_time) {
    auto c = cp->choose(current_time);
    Log.log(101, "MakeChoice::MakeChoice %s -> %s", cp->getName().c_str(),
            to_string(c).c_str());
    Memory::ins().PushStack("[" + to_string(c) + "]");
    return c;
  }

  ~MakeChoice() {
    Memory::ins().PopStack();
    Memory::ins().PopStack();
  }

private:
  ChoicePoint<T> *cp;
};

/**
 * run a child machine while taking care of call stack
 */
class Run {
public:
  Run(HierarchicalFSM *m) : m(m) {
    Log.log(101, "Run::Run %s", m->getName().c_str());
    Memory::ins().PushStack(m->getName());
  }

  void operator()() { m->run(); }

  ~Run() { Memory::ins().PopStack(); }

private:
  HierarchicalFSM *m;
};

class Action {
public:
  Action(HierarchicalFSM *m, vector<string> parameters = {}) : m(m) {
    Log.log(101, "Action::Action with parameters=%s",
            to_prettystring(parameters).c_str());
    Memory::ins().PushStack("!" + to_prettystring(parameters));
  }

  void operator()() { m->action(); }

  ~Action() { Memory::ins().PopStack(); }

private:
  HierarchicalFSM *m;
};
}

#endif // KEEPAWAY_PLAYER_CHOICEPOINT_H
