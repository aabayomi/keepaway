#include <limits>
#include <linux/limits.h>
#include <fcntl.h>

#include "SMDPAgent.h"

using namespace std;

string getexepath() {
  char result[PATH_MAX];
  ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
  if (count > 0) {
    return string(result, (size_t) count);
  } else {
    return string(result, 0);
  }
}

FileLock::FileLock(const string &prefix, const string &file, const string &func) {
  static const timespec sleepTime = {0, 1 * 1000 * 1000}; //1ms
  lockName = prefix + "_" + file + "_" + func + ".flock";

  for (;;) {
    lock = open(lockName.c_str(), O_CREAT | O_EXCL, 0664);
    if (lock >= 0) break;
    nanosleep(&sleepTime, NULL);
  }
}

FileLock::~FileLock() {
  unlink(lockName.c_str());
  close(lock);
}

SMDPAgent::SMDPAgent(int numFeatures) {
  m_numFeatures = numFeatures;
  lastAction = -1;
  lastActionTime = UnknownTime;
  hiveMind = 0;

  lockPrefix = getexepath();
  replace(lockPrefix.begin(), lockPrefix.end(), '/', '_');
  lockPrefix = "/run/shm/" + lockPrefix;
  PRINT_VALUE(lockPrefix);
}

int AtomicAction::keepers = 3;

ostream& operator<<(ostream& out, const AtomicActionType value) {
  static unordered_map<int, string> strings;

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

bool AtomicAction::terminated(BasicPlayer *player) {
  double dDist = numeric_limits<double>::max();
  ObjectT closest = player->WM->getClosestInSetTo( OBJECT_SET_TEAMMATES,
                                                   OBJECT_BALL, &dDist );

  return SoccerTypes::isTeammate(closest ) &&
         dDist < player->WM->getMaximalKickDist(closest);
}

SoccerCommand Hold::execute(BasicPlayer *player) {
  SoccerCommand soc;

  if (player->WM->isBallKickable()) {
    player->ACT->putCommandInQueue(soc = player->holdBall());
    player->ACT->putCommandInQueue(player->turnNeckToObject(OBJECT_BALL, soc));
    return soc;
  }
  else {
    return Intercept().execute(player);
  }
}

bool Hold::terminated(BasicPlayer *player) {
  (void) player;
  return true; // always terminate in one cycle
}

vector<int> PassTo::parameters() {
  vector<int> ret;
  for (int i = 0; i < (keepers - 1) * 5; ++i) {
    ret.push_back(i);
  }
  return ret;
}

string PassTo::name() const {
  stringstream ss;
  ss << type << "_" << k() << "-" << (d() == 4? "Stay": "Move_" + to_string(d()));
  return ss.str();
}

SoccerCommand PassTo::execute(BasicPlayer *player) {
  SoccerCommand soc;

  if (player->WM->isBallKickable()) {
    int numK = player->WM->getNumKeepers();
    ObjectT K[numK];
    for (int i = 0; i < numK; i++)
      K[i] = SoccerTypes::getTeammateObjectFromIndex(i);
    player->WM->sortClosestTo(K, numK, player->WM->getAgentObjectType());
    VecPosition tmPos = player->WM->getGlobalPosition(K[k()]);
    // Normal Passing
    player->ACT->putCommandInQueue(soc = player->directPass(tmPos, PASS_NORMAL));
  }
  else {
    if (d() == 4) {
      return Stay().execute(player);
    }
    else {
      return Move(d()).execute(player);
    }
  }

  player->ACT->putCommandInQueue( player->turnNeckToObject( OBJECT_BALL, soc ) );
  return soc;
}

SoccerCommand Intercept::execute(BasicPlayer *player) {
  SoccerCommand soc;

  // If teammate has it, don't mess with it
  double dDist = std::numeric_limits<double>::max();
  ObjectT closest = player->WM->getClosestInSetTo( OBJECT_SET_TEAMMATES,
                                                   OBJECT_BALL, &dDist );

  if ( SoccerTypes::isTeammate( closest ) &&
       closest != player->WM->getAgentObjectType() &&
       dDist < player->WM->getMaximalKickDist(closest)) {
    return Stay().execute(player);
  }
  else {
    player->ACT->putCommandInQueue(soc = player->intercept(false));
    player->ACT->putCommandInQueue(player->turnNeckToObject(OBJECT_BALL, soc));
    return soc;
  }
}

SoccerCommand Stay::execute(BasicPlayer *player) {
  SoccerCommand soc;
  Log.log(101, "turnBodyToObject( OBJECT_BALL )");
  player->ACT->putCommandInQueue( soc = player->turnBodyToObject( OBJECT_BALL ) );
  player->ACT->putCommandInQueue( player->turnNeckToObject( OBJECT_BALL, soc ) );
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

  auto r = player->WM->getKeepawayRect();
  if (r.isInside(target)) {
    Log.log(101, "move to target %s", target.str().c_str());
    player->ACT->putCommandInQueue(soc = player->moveToPos(target, 30.0));
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
        auto d = refined.getDistanceTo(target);
        if (d < minDist) {
          minDist = d;
          refinedTarget = refined;
        }
      }
    }

    if (minDist < std::numeric_limits<double>::max()) {
      Log.log(101, "move to refinedTarget %s", refinedTarget.str().c_str());
      player->ACT->putCommandInQueue(soc = player->moveToPos(refinedTarget, 30.0));
    }
    else {
      Log.log(101, "player->turnBodyToObject( OBJECT_BALL )");
      return Stay().execute(player);
    }
  }

  player->ACT->putCommandInQueue( player->turnNeckToObject( OBJECT_BALL, soc ) );
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
    actions[ja.tmControllBall][k].push_back(new Intercept);
    actions[ja.tmControllBall][k].push_back(new Stay);
    actions[ja.tmControllBall][k].push_back(new Move);
  }
  construct(ja.tmControllBall, 0, actions[ja.tmControllBall], ja);

  ja.tmControllBall = false;
  actions[ja.tmControllBall][0].push_back(new Intercept);
  for (int k = 1; k < AtomicAction::keepers; ++k) {
    actions[ja.tmControllBall][k].push_back(new Stay);
    actions[ja.tmControllBall][k].push_back(new Move);
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
      if (tmControlBall) {
        if (k && ja.actions[0]->type == AAT_PassTo &&
            ja.actions[0]->parameter / 5 + 1 == k &&
            a->type != AAT_Intercept)
          continue;
        if (k && ja.actions[0]->type == AAT_PassTo &&
            ja.actions[0]->parameter / 5 + 1 != k &&
            a->type == AAT_Intercept)
          continue;
        if (k && ja.actions[0]->type == AAT_Hold &&
            a->type == AAT_Intercept)
          continue;
      }
      else {
        if (k && a->type == AAT_Intercept &&
            ja.has_intercept(k))
          continue;
      }

      for (auto pa : a->parameters()) {
        ja.actions[k] = a->clone(pa);
        construct(tmControlBall, k + 1, actions, ja);
      }
    }
  }
}
