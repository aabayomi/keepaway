#include "SMDPAgent.h"
#include "BasicPlayer.h"

using namespace std;

int AtomicAction::keepers = 3;

std::ostream& operator<<(std::ostream& out, const AtomicActionType value) {
  static std::map<AtomicActionType, std::string> strings;

  if (strings.size() == 0){
#define INSERT_ELEMENT(p) strings[p] = #p
    INSERT_ELEMENT(AAT_None);
    INSERT_ELEMENT(AAT_Hold);
    INSERT_ELEMENT(AAT_PassTo);
    INSERT_ELEMENT(AAT_Intercept);
    INSERT_ELEMENT(AAT_Stay);
    INSERT_ELEMENT(AAT_Move);
#undef INSERT_ELEMENT
  }

  return out << strings[value];
}

SoccerCommand Hold::execute(BasicPlayer *player) {
  SoccerCommand soc;
  player->ACT->putCommandInQueue(soc = player->holdBall());
  return soc;
}

std::vector<int> PassTo::parameters() {
  std::vector<int> ret;
  for (int k = 1; k < keepers; ++k) {
    ret.push_back(k);
  }
  return ret;
}

SoccerCommand PassTo::execute(BasicPlayer *player) {
  SoccerCommand soc;
  int numK = player->WM->getNumKeepers();
  ObjectT K[numK];
  for (int i = 0; i < numK; i++)
    K[i] = SoccerTypes::getTeammateObjectFromIndex(i);
  player->WM->sortClosestTo(K, numK, player->WM->getAgentObjectType());
  VecPosition tmPos = player->WM->getGlobalPosition(K[parameter]);
  // Normal Passing
  player->ACT->putCommandInQueue(soc = player->directPass(tmPos, PASS_NORMAL));
  return soc;
}

SoccerCommand Intercept::execute(BasicPlayer *player) {
  SoccerCommand soc;
  player->ACT->putCommandInQueue(soc = player->intercept(false));
  player->ACT->putCommandInQueue(player->turnNeckToObject(OBJECT_BALL, soc));
  return soc;
}

SoccerCommand Stay::execute(BasicPlayer *player) {
  SoccerCommand soc;
  player->ACT->putCommandInQueue(soc = player->moveToPos(player->WM->getAgentGlobalPosition(), 30.0));
  return soc;
}

std::vector<int> Move::parameters() {
  return {0, 1, 2, 3};
}

SoccerCommand Move::execute(BasicPlayer *player) {
  SoccerCommand soc;
  AngDeg ang2ball = (player->WM->getBallPos() - player->WM->getAgentGlobalPosition()).getDirection();

  VecPosition target = player->WM->getAgentGlobalPosition() +
                       VecPosition::getVecPositionFromPolar(1.0, ang2ball + parameter * 90.0);

  if (player->WM->getKeepawayRect().isInside(target)) {
    player->ACT->putCommandInQueue(soc = player->moveToPos(target, 30.0));
  } else {
    player->ACT->putCommandInQueue(soc = player->moveToPos(player->WM->getAgentGlobalPosition(), 30.0));
  }
  return soc;
}

JointActionSpace::JointActionSpace()
    : count(0) {
  std::vector<std::vector<AtomicAction *>> actions[2];
  actions[0].resize((std::size_t) AtomicAction::keepers);
  actions[1].resize((std::size_t) AtomicAction::keepers);

  JointAction ja;

  ja.tmControllBall = true;
  actions[ja.tmControllBall][0].push_back(new Hold);
  actions[ja.tmControllBall][0].push_back(new PassTo);
  for (int k = 1; k < AtomicAction::keepers; ++k) {
    actions[ja.tmControllBall][k].push_back(new Move);
    actions[ja.tmControllBall][k].push_back(new Stay);
  }
  construct(ja.tmControllBall, 0, actions[ja.tmControllBall], ja);

  ja.tmControllBall = false;
  actions[ja.tmControllBall][0].push_back(new Intercept);
  for (int k = 1; k < AtomicAction::keepers; ++k) {
    actions[ja.tmControllBall][k].push_back(new Stay);
  }
  construct(ja.tmControllBall, 0, actions[ja.tmControllBall], ja);
}

void JointActionSpace::construct(
    bool tmControlBall, int k,
    vector<vector<AtomicAction *>> &actions, JointAction &ja) {
  if (k >= AtomicAction::keepers) {
    ja.id = count++;
    jointActions[tmControlBall].push_back(new JointAction(ja));
    jaMap[ja.id] = jointActions[tmControlBall].back();
  } else {
    for (auto a : actions[k]) {
      for (auto pa : a->parameters()) {
        ja.actions[k] = a->clone(pa);
        construct(tmControlBall, k + 1, actions, ja);
      }
    }
  }
}
