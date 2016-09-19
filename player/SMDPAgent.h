/*
Copyright (c) 2004 Gregory Kuhlmann, Peter Stone
University of Texas at Austin
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the University of Amsterdam nor the names of its
contributors may be used to endorse or promote products derived from this
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef SMDP_AGENT
#define SMDP_AGENT

#include <utility>
#include <vector>
#include <zconf.h>
#include <cassert>
#include <cstdlib>
#include <unordered_set>
#include <unordered_map>

#define MAX_STATE_VARS         64
#define MAX_ACTIONS            128

enum AtomicActionType {
  AAT_None,
  AAT_Hold,
  AAT_PassTo, // 2, 3, ..., k
  AAT_Intercept,
  AAT_Stay,
  AAT_Move // Left, Right, In, Out
};

struct AtomicAction {
  AtomicActionType type;
  int parameter;

  AtomicAction(AtomicActionType t = AAT_None, int p = 0): type(t), parameter(p) { }
  virtual std::vector<int> parameters() = 0;
  virtual AtomicAction *clone(int parameter) = 0;

  template <class Derived>
  AtomicAction *clone(int parameter) {
    auto a = new Derived();
    a->parameter = parameter;
    return a;
  }

  static const int keepers = 3; //FIXME: assuming keepers == 3
};

#define CLONE(cls) \
  AtomicAction *clone(int parameter) { \
    auto a = new cls(); \
    a->parameter = parameter; \
    return a; \
  }

struct Hold: public AtomicAction {
  Hold(): AtomicAction(AAT_Hold) { }

  std::vector<int> parameters() {
      return {0};
  }

  CLONE(Hold)
};

struct PassTo: public AtomicAction {
  PassTo(): AtomicAction(AAT_PassTo) { }

  std::vector<int> parameters() {
    std::vector<int> ret;
    for (int k = 2; k <= keepers; ++k) {
        ret.push_back(k);
    }
    return ret;
  }

  CLONE(PassTo)
};

struct Intercept: public AtomicAction {
  Intercept(): AtomicAction(AAT_Intercept) { }

  std::vector<int> parameters() {
    return {0};
  }

  CLONE(Intercept)
};

struct Stay: public AtomicAction {
  Stay(): AtomicAction(AAT_Stay) { }

  std::vector<int> parameters() {
    return {0};
  }

  CLONE(Stay)
};

struct Move: public AtomicAction {
  Move(): AtomicAction(AAT_Move) { }

  std::vector<int> parameters() {
    return {0, 1, 2, 3};
  }

  CLONE(Move)
};

struct JointAction {
  std::vector<AtomicAction*> actions;
  int id;

  JointAction(): id(0) {
      actions.resize(AtomicAction::keepers);
  }

  JointAction(const JointAction &ja): actions(ja.actions), id(ja.id) { }

  const JointAction &operator=(const JointAction &ja) {
    if (this != &ja) {
      actions = ja.actions;
      id = ja.id;
    }
    return *this;
  }
};

class JointActionSpace {
private:
  JointActionSpace(): count(0) {
    std::vector<std::vector<AtomicAction*>> actions[2];
    actions[0].resize(AtomicAction::keepers);
    actions[1].resize(AtomicAction::keepers);

    JointAction ja;

    // tmControlBall
    actions[1][0].push_back(new PassTo);
    actions[1][0].push_back(new Hold);
    for (int k = 1; k < AtomicAction::keepers; ++k) {
      actions[1][k].push_back(new Move);
      actions[1][k].push_back(new Stay);
    }
    construct(true, 0, actions[1], ja);

    // !tmControllBall
    for (int k = 0; k < AtomicAction::keepers; ++k) {
      actions[0][k].push_back(new Intercept);
      actions[0][k].push_back(new Stay);
    }
    construct(false, 0, actions[0], ja);
  }

  void construct(
      bool tmControlBall, int k,
      std::vector<std::vector<AtomicAction*>> &actions, JointAction &ja) {
    if (k >= AtomicAction::keepers) {
      ja.id = count++;
      jointActions[tmControlBall].push_back(ja);
      jaMap[ja.id] = jointActions[tmControlBall].back();
    }
    else {
      for (auto a : actions[k]) {
        for (auto pa : a->parameters()) {
          ja.actions[k] = a->clone(pa);
          construct(tmControlBall, k + 1, actions, ja);
        }
      }
    }
  }

public:
  static JointActionSpace &instance() {
    static JointActionSpace jas;
    return jas;
  }

  const JointAction &sample(bool tmControlBall) const {
    return jointActions[tmControlBall][rand() % jointActions[tmControlBall].size()];
  }

  int numActions() const { return (int) jaMap.size(); }

  const JointAction &getJointAction(int id) { return jaMap[id]; }

private:
  std::vector<JointAction> jointActions[2];
  int count;
  std::unordered_map<int, JointAction> jaMap;
};

class SMDPAgent
{
  int m_numFeatures; /* number of state features <= MAX_STATE_VARS */

protected:

  int getNumFeatures() const { return m_numFeatures; }
  int getNumActions()  const { return JointActionSpace::instance().numActions();  }

public:

  SMDPAgent( int numFeatures)
  { m_numFeatures = numFeatures; }
  virtual ~SMDPAgent() {}

  // abstract methods to be supplied by implementing class
  virtual int  startEpisode( int current_time, double state[] ) = 0;
  virtual int  step( int current_time, double reward, double state[] ) = 0;
  virtual void endEpisode( int current_time, double reward ) = 0;
  virtual void setParams(int iCutoffEpisodes, int iStopLearningEpisodes) = 0; //*met 8/16/05

  // Optional customization point.
  virtual void shutDown() {}
} ;

#endif
