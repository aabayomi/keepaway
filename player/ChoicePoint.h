//
// Created by baj on 10/4/16.
//

#ifndef KEEPAWAY_PLAYER_CHOICEPOINT_H
#define KEEPAWAY_PLAYER_CHOICEPOINT_H


#include <string>
#include <vector>
#include <cstring>
#include <functional>
#include "HierarchicalFSM.h"

namespace fsm {

template<class T>
class ChoicePoint {
public:
  ChoicePoint(const std::string &name, const std::vector<T> c) :
      name(name),
      choices(c) {

  }

  const string &getName() const {
    return name;
  }

  T choose(int current_time) {
    Log.log(101, "ChoicePoint::choose point name: %s", name.c_str());
    auto i = LinearSarsaLearner::ins().step(current_time, (int) choices.size());
    assert(i < choices.size());

    Log.log(101, "ChoicePoint::choose my choice (agent %d): %s", Memory::ins().agentIdx, to_string(choices[i]).c_str());
    return choices[i];
  }

private:
  std::string name;
  std::vector<T> choices;
};

}

#endif //KEEPAWAY_PLAYER_CHOICEPOINT_H
