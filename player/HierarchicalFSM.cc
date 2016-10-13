//
// Created by baj on 10/3/16.
//

#include <cstring>
#include <cassert>
#include <sstream>
#include "HierarchicalFSM.h"
#include "ChoicePoint.h"

namespace std {

const string &to_string(fsm::HierarchicalFSM *m) {
  return m->getName();
}

}

namespace fsm {

Memory::Memory() {
  bAlive = true;

  resetState();
}

Memory &Memory::ins() {
  static Memory memory;
  return memory;
}

void Memory::resetState() {
  memset(state, 0, sizeof(state));
  memset(K, 0, sizeof(K));
}

string Memory::to_string() {
  std::stringstream ss;
  PRINT_VALUE_STREAM(ss, vector<double>(state, state + HierarchicalFSM::num_features));
  PRINT_VALUE_STREAM(ss, vector<ObjectT>(K, K + HierarchicalFSM::num_keepers));
  PRINT_VALUE_STREAM(ss, stack);

  return ss.str();
}

const vector<string> &Memory::getStack() const {
  return stack;
}

void Memory::PushStack(const string &s) {
  stack.push_back(s);
  if (Log.isInLogLevel(101)) {
    stringstream ss;
    PRINT_VALUE_STREAM(ss, stack);
    Log.log(101, "Memory::PushStack:\n%s", ss.str().c_str());
  }
}

void Memory::PopStack() {
  stack.pop_back();
  if (Log.isInLogLevel(101)) {
    stringstream ss;
    PRINT_VALUE_STREAM(ss, stack);
    Log.log(101, "Memory::PopStack:\n%s", ss.str().c_str());
  }
}

int HierarchicalFSM::num_features;
int HierarchicalFSM::num_keepers;
double HierarchicalFSM::gamma;

HierarchicalFSM::HierarchicalFSM(BasicPlayer *p, const std::string &name) :
    player(p), name(name) {
  ACT = player->ACT;
  WM = player->WM;
  SS = player->SS;
  PS = player->PS;

  dummyChoice = new ChoicePoint<int>("}", {0});
}

HierarchicalFSM::~HierarchicalFSM() {
  delete dummyChoice;
}

void HierarchicalFSM::action(bool sync) {
  Log.log(101, "action with stack %s", getStackStr().c_str());

  while (Memory::ins().bAlive) {
    if (sync) MakeChoice<int>(dummyChoice).operator()(); // dummy choice for sync purposes

    if (WM->getTimeLastSeeMessage() == WM->getCurrentTime() ||
        (SS->getSynchMode() && WM->getRecvThink())) {
      Log.log(101, "send commands");
      ACT->sendCommands();

      if (SS->getSynchMode()) {
        WM->processRecvThink(false);
        ACT->sendMessageDirect("(done)");
      }
    }

    Memory::ins().bAlive = WM->waitForNewInformation();
    if (!Memory::ins().bAlive) break;
    Log.setHeader(WM->getCurrentCycle());
    WM->updateAll();

    auto e = getState();
    if (e.length()) {
      idle(e);
    } else {
      break;
    }
  }
}

void HierarchicalFSM::idle(const std::string error) {
  SoccerCommand soc;
  if (error != "ball lost") {
    ACT->putCommandInQueue(soc = player->turnBodyToObject(OBJECT_BALL));
    ACT->putCommandInQueue(player->turnNeckToObject(OBJECT_BALL, soc));
  }
  Log.log(101, "idle (error: %s)", error.c_str());
}

string HierarchicalFSM::getState() {
  Memory::ins().resetState();

  if (WM->getConfidence(OBJECT_BALL) < PS->getBallConfThr()) {
    ACT->putCommandInQueue(player->searchBall());
    ACT->putCommandInQueue(player->alignNeckWithBody());
    return "ball lost";
  }

  int numK = WM->getNumKeepers();

  int features = WM->keeperStateVars(Memory::ins().state);
  assert(features == 0 || features == num_features);
  if (features != num_features) return "features != SA->getNumFeatures()";

  for (int i = 0; i < numK; i++)
    Memory::ins().K[i] = SoccerTypes::getTeammateObjectFromIndex(i);

  ObjectT K0 = WM->getClosestInSetTo(OBJECT_SET_TEAMMATES, OBJECT_BALL);
  if (!WM->sortClosestTo(Memory::ins().K, numK, K0)) return "!WM->sortClosestTo(K, numK, K0)";
  if (K0 != Memory::ins().K[0]) return "K0 != K[0]";

  auto &agentIdx = LinearSarsaLearner::ins().agentIdx;
  agentIdx = 0;
  while (agentIdx < numK &&
         Memory::ins().K[agentIdx] != WM->getAgentObjectType())
    agentIdx += 1;
  assert(agentIdx < numK);

  if (agentIdx >= numK) return "agentIdx >= numK";
  return "";
}

const string &HierarchicalFSM::getName() const {
  return name;
}

bool HierarchicalFSM::running() {
  return Memory::ins().bAlive && !WM->isNewEpisode();
}

string HierarchicalFSM::getStackStr() {
  string str;
  for (auto &s : Memory::ins().getStack()) str += s + " ";
  return str;
}

void HierarchicalFSM::initialize(int numFeatures,
                                 int numKeepers,
                                 bool bLearn,
                                 double widths[],
                                 double Gamma,
                                 double initialWeight) {
  num_features = numFeatures;
  num_keepers = numKeepers;
  gamma = Gamma;
  LinearSarsaLearner::ins().initialize(bLearn, widths, initialWeight);
}

Keeper::Keeper(BasicPlayer *p) : HierarchicalFSM(p, "{") {
  pass = new Pass(p);
  hold = new Hold(p);
  move = new Move(p);
  stay = new Stay(p);
  intercept = new Intercept(p);

  choices[0] = new ChoicePoint<HierarchicalFSM *>("ballFree", {intercept, stay, move});
  choices[1] = new ChoicePoint<HierarchicalFSM *>("tmControllBall", {stay, move});
  choices[2] = new ChoicePoint<HierarchicalFSM *>("ballKickable", {pass, hold});
}

Keeper::~Keeper() {
  delete choices[0];
  delete choices[1];
  delete choices[2];

  delete pass;
  delete hold;
  delete move;
  delete stay;
  delete intercept;
}

void Keeper::run() {
  while (WM->getCurrentCycle() != 1) action(false);

  while (Memory::ins().bAlive) {
    assert(Memory::ins().getStack().size() == 1);
    if (WM->isNewEpisode()) {
      LinearSarsaLearner::ins().endEpisode();
      WM->setNewEpisode(false);
    }

    int status = 0;
    if (WM->isBallKickable()) status = 2;
    else if (WM->isTmControllBall()) status = 1;

    Log.log(101, "Keeper::run status %d", status);
    auto m = MakeChoice<HierarchicalFSM *>(choices[status])();
    Run(m).operator()();
  }

  Log.log(101, "Keeper::run exit");
}

Move::Move(BasicPlayer *p) : HierarchicalFSM(p, "Move") {
  moveToChoice = new ChoicePoint<int>("moveTo", {0, 1, 2, 3});
}

Move::~Move() {
  delete moveToChoice;
}

void Move::run() {
  assert(!WM->isBallKickable());
  auto d = MakeChoice<int>(moveToChoice)();

  bool flag = WM->isTmControllBall();
  while (running() && flag == WM->isTmControllBall()) {
    SoccerCommand soc;
    VecPosition target = WM->getAgentGlobalPosition() +
                         (WM->getBallPos() -
                          WM->getAgentGlobalPosition()).rotate(d * 90.0).normalize();

    auto r = WM->getKeepawayRect();
    if (r.isInside(target)) {
      ACT->putCommandInQueue(soc = player->moveToPos(target, 30.0));
    } else {
      double minDist = std::numeric_limits<double>::max();
      VecPosition refinedTarget;
      pair<VecPosition, VecPosition> pt[4];

      pt[0] = {r.getPosLeftTop(), r.getPosRightTop()};
      pt[1] = {r.getPosRightTop(), r.getPosRightBottom()};
      pt[2] = {r.getPosRightBottom(), r.getPosLeftBottom()};
      pt[3] = {r.getPosLeftBottom(), r.getPosLeftTop()};

      for (int i = 0; i < 4; ++i) {
        auto edge = Line::makeLineFromTwoPoints(pt[i].first, pt[i].second);
        auto refined = edge.getPointOnLineClosestTo(target);
        bool inBetween = edge.isInBetween(refined, pt[i].first, pt[i].second);
        if (inBetween) {
          auto dist = refined.getDistanceTo(target);
          if (dist < minDist) {
            minDist = dist;
            refinedTarget = refined;
          }
        }
      }

      if (minDist < std::numeric_limits<double>::max()) {
        ACT->putCommandInQueue(soc = player->moveToPos(refinedTarget, 30.0));
      } else {
        ACT->putCommandInQueue(soc = player->turnBodyToObject(OBJECT_BALL));
        ACT->putCommandInQueue(player->turnNeckToObject(OBJECT_BALL, soc));
      }
    }

    ACT->putCommandInQueue(player->turnNeckToObject(OBJECT_BALL, soc));
    Log.log(101, "Move::run action with d=%d", d);
    Action(this, d)();
  }
}

Stay::Stay(BasicPlayer *p) : HierarchicalFSM(p, "Stay") {

}

void Stay::run() {
  assert(!WM->isBallKickable());
  bool flag = WM->isTmControllBall();
  while (running() && flag == WM->isTmControllBall()) {
    SoccerCommand soc;
    ACT->putCommandInQueue(soc = player->turnBodyToObject(OBJECT_BALL));
    ACT->putCommandInQueue(player->turnNeckToObject(OBJECT_BALL, soc));
    Log.log(101, "Stay::run action");
    Action(this)();
  }
}

Intercept::Intercept(BasicPlayer *p) : HierarchicalFSM(p, "Intercept") {

}

void Intercept::run() {
  assert(!WM->isBallKickable());
  assert(!WM->isTmControllBall());
  while (running() && !WM->isTmControllBall()) {
    SoccerCommand soc;
    ACT->putCommandInQueue(soc = player->intercept(false));
    ACT->putCommandInQueue(player->turnNeckToObject(OBJECT_BALL, soc));
    Log.log(101, "Intercept::run action");
    Action(this)();
  }
}

Pass::Pass(BasicPlayer *p) : HierarchicalFSM(p, "Pass") {
  vector<int> parameters;
  for (int i = 1; i < num_keepers; ++i) {
    parameters.push_back(i);
  }
  passToChoice = new ChoicePoint<int>("passTo", parameters);
}

Pass::~Pass() {
  delete passToChoice;
}

void Pass::run() {
  assert(WM->isBallKickable());
  auto k = MakeChoice<int>(passToChoice)();

  while (running() && WM->isBallKickable()) {
    VecPosition tmPos = WM->getGlobalPosition(Memory::ins().K[k]);
    ACT->putCommandInQueue(player->directPass(tmPos, PASS_NORMAL));
    Log.log(101, "Pass::run action with k=%d", k);
    Action(this, k)();
  }
}

Hold::Hold(BasicPlayer *p) : HierarchicalFSM(p, "Hold") {

}

void Hold::run() {
  assert(WM->isBallKickable());
  SoccerCommand soc;
  ACT->putCommandInQueue(soc = player->holdBall());
  ACT->putCommandInQueue(player->turnNeckToObject(OBJECT_BALL, soc));
  Log.log(101, "Hold::run action");
  Action(this)();
}

}
