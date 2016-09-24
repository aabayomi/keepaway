#include "SMDPAgent.h"
#include "BasicPlayer.h"

using namespace std;

int AtomicAction::keepers = 3;

std::ostream& operator<<(std::ostream& out, const AtomicActionType value) {
  static std::unordered_map<int, std::string> strings;

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

bool Hold::terminated(double state[], int num_features) {
  return true; // always terminate in one cycle
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

  if (player->WM->isBallKickable()) {
    int numK = player->WM->getNumKeepers();
    ObjectT K[numK];
    for (int i = 0; i < numK; i++)
      K[i] = SoccerTypes::getTeammateObjectFromIndex(i);
    player->WM->sortClosestTo(K, numK, player->WM->getAgentObjectType());
    VecPosition tmPos = player->WM->getGlobalPosition(K[parameter]);
    // Normal Passing
    player->ACT->putCommandInQueue(soc = player->directPass(tmPos, PASS_NORMAL));
  }
  else {
    player->ACT->putCommandInQueue(
        soc = player->moveToPos(player->WM->getAgentGlobalPosition(), 30.0));
  }

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
  player->ACT->putCommandInQueue(
      soc = player->moveToPos(player->WM->getAgentGlobalPosition(), 30.0));
  return soc;
}

std::vector<int> Move::parameters() {
  return {0, 1, 2, 3};
}

SoccerCommand Move::execute(BasicPlayer *player) {
  SoccerCommand soc;
  VecPosition target = player->WM->getAgentGlobalPosition() +
      (player->WM->getBallPos() -
          player->WM->getAgentGlobalPosition()).rotate(parameter * 90.0).normalize();

  if (player->WM->getKeepawayRect().isInside(target)) {
    player->ACT->putCommandInQueue(soc = player->moveToPos(target, 30.0));
  } else {
    player->ACT->putCommandInQueue(
        soc = player->moveToPos(player->WM->getAgentGlobalPosition(), 30.0));
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
    actions[ja.tmControllBall][k].push_back(new Intercept);
  }
  construct(ja.tmControllBall, 0, actions[ja.tmControllBall], ja);

  ja.tmControllBall = false;
  actions[ja.tmControllBall][0].push_back(new Intercept);
  for (int k = 1; k < AtomicAction::keepers; ++k) {
    actions[ja.tmControllBall][k].push_back(new Move);
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
    auto j = jointActions[tmControlBall].back();
    jaMap[ja.id] = {j, j->name()};
  } else {
    for (auto a : actions[k]) {
      if (k &&
          ja.actions[0]->type == AAT_PassTo &&
          ja.actions[0]->parameter == k &&
          a->type != AAT_Intercept) continue;
      if (k &&
          ja.actions[0]->type == AAT_PassTo &&
          ja.actions[0]->parameter != k &&
          a->type == AAT_Intercept) continue;
      if (k &&
          ja.actions[0]->type == AAT_Hold &&
          a->type == AAT_Intercept) continue;

      for (auto pa : a->parameters()) {
        ja.actions[k] = a->clone(pa);
        construct(tmControlBall, k + 1, actions, ja);
      }
    }
  }
}
