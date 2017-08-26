//
// Created by baj on 10/3/16.
//

#include "HierarchicalFSM.h"
#include "ChoicePoint.h"

#define KEEPER_LEARN 1
#define TAKER_LEARN 0

namespace std {
const string &to_string(fsm::HierarchicalFSM *m) { return m->getName(); }
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
  memset(teammates, 0, sizeof(teammates));
  memset(opponents, 0, sizeof(opponents));
}

string Memory::to_string() {
  stringstream ss;
  PRINT_VALUE_STREAM(ss, agentIdx);
  PRINT_VALUE_STREAM(ss, vector<ObjectT>(teammates, teammates + HierarchicalFSM::num_teammates));
  PRINT_VALUE_STREAM(ss, vector<ObjectT>(opponents, opponents + HierarchicalFSM::num_opponents));
  PRINT_VALUE_STREAM(
      ss, vector<double>(state, state + HierarchicalFSM::num_features));
  PRINT_VALUE_STREAM(ss, stack);

  return ss.str();
}

const vector<string> &Memory::getStack() const { return stack; }

void Memory::PushStack(const string &s) {
  stack.push_back(s);
  if (Log.isInLogLevel(101)) {
    Log.log(101, "Memory::PushStack: %s", to_prettystring(stack).c_str());
  }
}

void Memory::PopStack() {
  stack.pop_back();
  if (Log.isInLogLevel(101)) {
    Log.log(101, "Memory::PopStack: %s", to_prettystring(stack).c_str());
  }
}

size_t Memory::ballControlHash() {
  return hash<vector<int>>().operator()(vector<int>(ballControlState, ballControlState + HierarchicalFSM::num_teammates));
}

int HierarchicalFSM::num_features;
int HierarchicalFSM::num_teammates;
int HierarchicalFSM::num_opponents;

HierarchicalFSM::HierarchicalFSM(BasicPlayer *p, const std::string &name)
    : player(p), name(name) {
  ACT = player->ACT;
  WM = player->WM;
  SS = player->SS;
  PS = player->PS;

  dummyChoice = new ChoicePoint<int>("}", {0});
}

HierarchicalFSM::~HierarchicalFSM() { delete dummyChoice; }

void HierarchicalFSM::action(bool sync) {
  Log.log(101, "action with stack=%s", getStackStr().c_str());

  while (Memory::ins().bAlive) {
    if (sync)
      MakeChoice<int>(dummyChoice).operator()(WM->getCurrentCycle());

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
    if (!Memory::ins().bAlive)
      break;
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

bool HierarchicalFSM::isFastestToBall() {
  int minCycle = INT_MAX;
  int agentCycle = INT_MAX;

  for (int i = 0; i < num_teammates; ++i) {
    int iCycle = INT_MAX;
    auto o = Memory::ins().teammates[i];
    SoccerCommand illegal;
    WM->predictCommandToInterceptBall(o, illegal, &iCycle);
    minCycle = min(minCycle, iCycle);
    if (o == WM->getAgentObjectType())
      agentCycle = iCycle;
    Log.log(101, "HierarchicalFSM::isFastestToBall ballFree teammate idx %d iCycle %d", i,
            iCycle);
  }

  if (agentCycle == minCycle && minCycle < INT_MAX) {
    bool fastest = minCycle < 100 || Memory::ins().agentIdx == 0;
    Log.log(101, "HierarchicalFSM::isFastestToBall ballFree agentCycle %d = minCycle %d, fastest = %d",
            agentCycle, minCycle, fastest);
    return fastest;
  }

  Log.log(101, "HierarchicalFSM::isFastestToBall false");
  return false;
}

VecPosition HierarchicalFSM::refineTarget(VecPosition target, VecPosition backup) {
  auto r = WM->getKeepawayRectReduced();
  if (r.isInside(target)) {
    return target;
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
      return refinedTarget;
    }

    return backup;
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

  int features = WM->playerStateVars(Memory::ins().state, num_teammates, num_opponents);
  Assert(features == 0 || features == num_features);
  if (features != num_features)
    return "features != SA->getNumFeatures()";

  for (int i = 0; i < num_teammates; i++)
    Memory::ins().teammates[i] = SoccerTypes::getTeammateObjectFromIndex(i);

  ObjectT T0 = WM->getClosestInSetTo(OBJECT_SET_TEAMMATES, OBJECT_BALL);
  if (!WM->sortClosestTo(Memory::ins().teammates, num_teammates, T0))
    return "!WM->sortClosestTo(teammate, num_teammates, T0)";
  if (T0 != Memory::ins().teammates[0])
    return "T0 != teammate[0]";

  auto ballPos = WM->getBallPos();
  for (int i = 0; i < num_teammates; ++i) {
    auto o = Memory::ins().teammates[i];
    Memory::ins().ballControlState[i]
        = WM->getGlobalPosition(o).getDistanceTo(ballPos) < WM->getMaximalKickDist(o);
  }

  for (int i = 0; i < num_opponents; i++)
    Memory::ins().opponents[i] = SoccerTypes::getOpponentObjectFromIndex(i);

  ObjectT O0 = WM->getClosestInSetTo(OBJECT_SET_OPPONENTS, OBJECT_BALL);
  if (!WM->sortClosestTo(Memory::ins().opponents, num_opponents, O0))
    return "!WM->sortClosestTo(opponent, num_opponents, O0)";
  if (O0 != Memory::ins().opponents[0])
    return "O0 != opponent[0]";

  auto &agentIdx = Memory::ins().agentIdx;
  agentIdx = 0;
  while (agentIdx < num_teammates &&
         Memory::ins().teammates[agentIdx] != WM->getAgentObjectType())
    agentIdx += 1;
  Assert(agentIdx < num_teammates);

  if (agentIdx >= num_teammates)
    return "agentIdx >= num_teammates";
  return "";
}

const string &HierarchicalFSM::getName() const { return name; }

bool HierarchicalFSM::running() {
  return Memory::ins().bAlive && !WM->isNewEpisode();
}

string HierarchicalFSM::getStackStr() {
  string str;
  for (auto &s : Memory::ins().getStack())
    str += s + " ";
  return str;
}

void HierarchicalFSM::initialize(int numFeatures, int numTeammates,
                                 int numOpponents, bool bLearn,
                                 double widths[], double Gamma, double Lambda,
                                 double initialWeight, bool qLearning,
                                 bool UseStaticTransition,
                                 string loadWeightsFile,
                                 string saveWeightsFile, string teamName) {
  num_features = numFeatures;
  num_teammates = numTeammates;
  num_opponents = numOpponents;
  LinearSarsaLearner::ins().initialize(bLearn, widths, Gamma, Lambda,
                                       initialWeight, qLearning, UseStaticTransition,
                                       loadWeightsFile, saveWeightsFile, teamName);
}

Keeper::Keeper(BasicPlayer *p) : HierarchicalFSM(p, "{") {
  pass = new Pass(p);
  hold = new Hold(p);
  move = new Move(p);
  stay = new Stay(p);
  intercept = new Intercept(p);
  getopen = new GetOpen(p);

  choices[0] = new ChoicePoint<HierarchicalFSM *>("@Free", {stay, move});
  choices[1] = new ChoicePoint<HierarchicalFSM *>("@Ball", {pass, hold});
}

Keeper::~Keeper() {
  delete choices[0];
  delete choices[1];

  delete pass;
  delete hold;
  delete move;
  delete stay;
  delete intercept;
  delete getopen;
}

void Keeper::run() {
  while (WM->getCurrentCycle() < 1)
    action(false);

  while (Memory::ins().bAlive) {
    if (WM->isNewEpisode()) {
      LinearSarsaLearner::ins().endEpisode(WM->getCurrentCycle());
      WM->setNewEpisode(false);
    }

    Log.log(101, "Keeper::run agentIdx %d", Memory::ins().agentIdx);
#if KEEPER_LEARN
    if (WM->isBallKickable()) {
      MakeChoice<HierarchicalFSM *> choice(choices[1]);
      auto m = choice(WM->getCurrentCycle());
      Run(m).operator()();
    } else if (!WM->isTmControllBall() && isFastestToBall()) {
      Run(intercept).operator()();
    } else {
      MakeChoice<HierarchicalFSM *> choice(choices[0]);
      auto m = choice(WM->getCurrentCycle());
      Run(m).operator()();
    }
#else
    if (WM->isBallKickable()) {
      Run(hold).operator()();
    } else if (isFastestToBall()) {
      Run(intercept).operator()();
    } else {
      Run(getopen).operator()();
    }
#endif
  }

  Log.log(101, "Keeper::run exit");
}

Taker::Taker(BasicPlayer *p) : HierarchicalFSM(p, "{") {
  hold = new Hold(p);
  stay = new Stay(p);
  move = new Move(p);
  intercept = new Intercept(p);

  choice_taker = new ChoicePoint<HierarchicalFSM *>("@Taker", {stay, move});
}

Taker::~Taker() {
  delete choice_taker;

  delete hold;
  delete stay;
  delete move;
  delete intercept;
}

void Taker::run() {
  while (WM->getCurrentCycle() < 1)
    action(false);

  while (Memory::ins().bAlive) {
    if (WM->isNewEpisode()) {
      LinearSarsaLearner::ins().endEpisode(WM->getCurrentCycle());
      WM->setNewEpisode(false);
    }

    Log.log(101, "Taker::run agentIdx %d", Memory::ins().agentIdx);

#if TAKER_LEARN
    if (WM->isBallKickable()) {
      Run(hold).operator()();
    }
    else if (isFastestToBall()) {
      Run(intercept).operator()();
    }
    else {
      MakeChoice<HierarchicalFSM *> choice(choice_taker);
      auto m = choice(WM->getCurrentCycle());
      Run(m).operator()();
    }
#else
    if (WM->isBallKickable()) {
      Run(hold).operator()();
    } else {
      Run(intercept).operator()();
    }
#endif
  }

  Log.log(101, "Taker::run exit");
}

Move::Move(BasicPlayer *p) : HierarchicalFSM(p, "$Move") {
  moveToChoice = new ChoicePoint<int>("@MoveTo", {0, 1, 2, 3});
}

Move::~Move() {
  delete moveToChoice;
}

void Move::run() {
  MakeChoice<int> choice(moveToChoice);
  auto dir = choice(WM->getCurrentCycle());

  int status = WM->isTmControllBall();
  while (running() && status == WM->isTmControllBall()) {
    VecPosition target = WM->getAgentGlobalPosition() +
                         (WM->getBallPos() - WM->getAgentGlobalPosition())
                             .rotate(dir * 90.0)
                             .normalize();

    target = refineTarget(target, WM->getAgentGlobalPosition());
    auto distance = target.getDistanceTo(WM->getAgentGlobalPosition());
    auto cycles = rint(distance / SS->getPlayerSpeedMax());
    SoccerCommand soc;
    ACT->putCommandInQueue(
        soc = player->moveToPos(target, 25.0, 1.0, false, (int) cycles));
    ACT->putCommandInQueue(player->turnNeckToObject(OBJECT_BALL, soc));
    Log.log(101, "Move::run action with dir=%d, posPassFrom=%s, target=%s",
            dir, to_prettystring(WM->getBallPos()).c_str(), to_prettystring(target).c_str());
    Action(this, {to_prettystring(dir)})();
    if (WM->isBallKickable()) break;
  }
}

Stay::Stay(BasicPlayer *p) : HierarchicalFSM(p, "$Stay") {}

void Stay::run() {
  bool flag = WM->isTmControllBall();
  while (running() && flag == WM->isTmControllBall()) {
    SoccerCommand soc;
    ACT->putCommandInQueue(soc = player->turnBodyToObject(OBJECT_BALL));
    ACT->putCommandInQueue(player->turnNeckToObject(OBJECT_BALL, soc));
    Log.log(101, "Stay::run action");
    Action(this)();
    if (WM->isBallKickable()) break;
  }
}

Intercept::Intercept(BasicPlayer *p) : HierarchicalFSM(p, "$Intercept") {}

void Intercept::run() {
  while (running()) {
    SoccerCommand soc;
    ACT->putCommandInQueue(soc = player->intercept(false));
    ACT->putCommandInQueue(player->turnNeckToObject(OBJECT_BALL, soc));
    Log.log(101, "Intercept::run action");
    Action(this)();
    if (WM->isTmControllBall()) break;
  }
}

Pass::Pass(BasicPlayer *p) : HierarchicalFSM(p, "$Pass") {
  vector<int> parameters;
  for (int i = 1; i < num_teammates; ++i) {
    parameters.push_back(i);
  }
  passToChoice = new ChoicePoint<int>("@PassTo", parameters);
  passSpeedChoice =
      new ChoicePoint<PassT>("@PassSpeed", {PASS_FAST, PASS_NORMAL});
}

Pass::~Pass() {
  delete passToChoice;
  delete passSpeedChoice;
}

void Pass::run() {
  MakeChoice<int> c1(passToChoice);
  auto teammate = c1(WM->getCurrentCycle());

  MakeChoice<PassT> c2(passSpeedChoice);
  auto speed = c2(WM->getCurrentCycle());

  while (running() && WM->isBallKickable()) {
    VecPosition tmPos = WM->getGlobalPosition(Memory::ins().teammates[teammate]);
    VecPosition tmVel = WM->getGlobalVelocity(Memory::ins().teammates[teammate]);
    VecPosition target = WM->predictFinalAgentPos(&tmPos, &tmVel);
    ACT->putCommandInQueue(player->directPass(target, speed));
    Log.log(101, "Pass::run action with teammate=%d speed=%s", teammate,
            speed == PASS_NORMAL ? "normal" : "fast");
    Action(this, {to_string(teammate), to_string(speed)})();
  }
}

Hold::Hold(BasicPlayer *p) : HierarchicalFSM(p, "$Hold") {
}

Hold::~Hold() { }

void Hold::run() {
  SoccerCommand soc;
  ACT->putCommandInQueue(soc = player->holdBall());
  ACT->putCommandInQueue(player->turnNeckToObject(OBJECT_BALL, soc));
  Log.log(101, "Hold::run action");
  Action(this)();
}

GetOpen::GetOpen(BasicPlayer *p) : HierarchicalFSM(p, "$GetOpen") {}

void GetOpen::run() {
  int status = WM->isTmControllBall();
  while (running() && status == WM->isTmControllBall()) {
    SoccerCommand soc;
    ObjectT fastest = WM->getFastestInSetTo(OBJECT_SET_TEAMMATES, OBJECT_BALL);
    int iCycles = WM->predictNrCyclesToObject(fastest, OBJECT_BALL);
    VecPosition posPassFrom = WM->predictPosAfterNrCycles(OBJECT_BALL, iCycles);
    posPassFrom = refineTarget(posPassFrom, WM->getBallPos());
    ACT->putCommandInQueue(soc = player->getOpenForPassFromInRectangle(WM->getKeepawayRect(), posPassFrom));
    ACT->putCommandInQueue(player->turnNeckToObject(OBJECT_BALL, soc));
    Log.log(101, "GetOpen::run action");
    Action(this)();
    if (WM->isBallKickable()) break;
  }
}

}
