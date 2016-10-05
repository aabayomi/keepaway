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
  ChoicePoint(const std::string name, const std::vector<T> &c) :
      name(name),
      choices(c) {

  }

  T choose() {
    auto i = LinearSarsaLearner::ins().step((int) choices.size());
    Memory::ins().makeChoice(i, "#" + to_string(choices[i]));
    return choices[i];
  }

private:
  std::string name;
  std::vector<T> choices;
};

}

#endif //KEEPAWAY_PLAYER_CHOICEPOINT_H
