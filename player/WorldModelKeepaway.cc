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

#include "WorldModel.h"

int WorldModel::getNumKeepers() {
  return m_numKeepers;
}

void WorldModel::setNumKeepers(int iNum) {
  m_numKeepers = iNum;
}

int WorldModel::getNumTakers() {
  return m_numTakers;
}

void WorldModel::setNumTakers(int iNum) {
  m_numTakers = iNum;
}

double WorldModel::congestion(VecPosition pos, bool considerMe) {
  double congest = 0;
  if (considerMe && pos != getAgentGlobalPosition())
    congest = 1 / getAgentGlobalPosition().getDistanceTo(pos);

  VecPosition playerPos;

  int iIndex;
  for (ObjectT obj = iterateObjectStart(iIndex, OBJECT_SET_PLAYERS);
       obj != OBJECT_ILLEGAL;
       obj = iterateObjectNext(iIndex, OBJECT_SET_PLAYERS)) {
    if ((playerPos = getGlobalPosition(obj)) != pos)
      if (obj != getAgentObjectType())
        /* Don't want to count a player in its own congestion measure */
        congest += 1 / playerPos.getDistanceTo(pos);
  }

  return congest;
}

void WorldModel::resetEpisode() {
  Ball.setTimeLastSeen(-1);
  for (int i = 0; i < MAX_TEAMMATES; i++)
    Teammates[i].setTimeLastSeen(-1);
  for (int i = 0; i < MAX_OPPONENTS; i++)
    Opponents[i].setTimeLastSeen(-1);
  for (int i = 0; i < MAX_TEAMMATES + MAX_OPPONENTS; i++)
    UnknownPlayers[i].setTimeLastSeen(-1);
  iNrUnknownPlayers = 0;
  for (int i = 0; i < MAX_FLAGS; i++)
    Flags[i].setTimeLastSeen(-1);
  for (int i = 0; i < MAX_LINES; i++)
    Lines[i].setTimeLastSeen(-1);

  setNewEpisode(true);
}

void WorldModel::setNewEpisode(bool bNewEp) {
  m_newEpisode = bNewEp;
}

bool WorldModel::isNewEpisode() {
  return m_newEpisode;
}

double WorldModel::keeperReward(int lastActionTime) {
  double reward = getCurrentCycle() - lastActionTime;
  return reward;
}

int WorldModel::keeperStateVars(double state[]) {
  return getStateVars(state, getNumKeepers(), getNumTakers());
}

int WorldModel::takerStateVars(double state[]) {
  return getStateVars(state, getNumTakers(), getNumKeepers());
}

int WorldModel::getStateVars(double state[], int numTeammates, int numOpponents) {
  ObjectT K0 = getClosestInSetTo(OBJECT_SET_TEAMMATES, OBJECT_BALL);
  VecPosition C = getKeepawayRect().getPosCenter();

  VecPosition B = getBallPos();
  double dist_to_K0_B = getGlobalPosition(K0).getDistanceTo(B);
  double dist_to_C_B = B.getDistanceTo(C);

  double WK0_dist_to_C = getGlobalPosition(K0).getDistanceTo(C);

  ObjectT K[numTeammates];
  for (int i = 0; i < numTeammates; i++)
    K[i] = SoccerTypes::getTeammateObjectFromIndex(i);

  ObjectT T[numOpponents];
  for (int i = 0; i < numOpponents; i++)
    T[i] = SoccerTypes::getOpponentObjectFromIndex(i);

  double dist_to_K0_K[numTeammates];
  if (!sortClosestTo(K, numTeammates, K0, dist_to_K0_K)) {
    Log.log(101, "WorldModel::getStateVars !sortClosestTo(K, numTeammates, K0, dist_to_K0_K)");
    return 0;
  }

  double dist_to_K0_T[numOpponents];
  if (!sortClosestTo(T, numOpponents, K0, dist_to_K0_T)) {
    Log.log(101, "WorldModel::getStateVars !sortClosestTo(T, numOpponents, K0, dist_to_K0_T)");
    return 0;
  }

  double dist_to_C_K[numTeammates];
  for (int i = 1; i < numTeammates; i++) {
    dist_to_C_K[i] = getGlobalPosition(K[i]).getDistanceTo(C);
  }

  double dist_to_C_T[numOpponents];
  for (int i = 0; i < numOpponents; i++) {
    dist_to_C_T[i] = getGlobalPosition(T[i]).getDistanceTo(C);
  }

  double nearest_Opp_dist_K[numTeammates];
  VecPosition posPB = getGlobalPosition(K0);
  for (int i = 1; i < numTeammates; i++) {
    VecPosition pos = getGlobalPosition(K[i]);
    for (int j = 0; j < numOpponents; j++) {
      double tmp = getGlobalPosition(T[j]).getDistanceTo(pos);
      if (j == 0 || tmp < nearest_Opp_dist_K[i]) {
        nearest_Opp_dist_K[i] = tmp;
      }
    }
  }

  double nearest_Opp_ang_K[numTeammates];
  for (int i = 1; i < numTeammates; i++) {
    VecPosition pos = getGlobalPosition(K[i]);
    for (int j = 0; j < numOpponents; j++) {
      double tmp = posPB.getAngleBetweenPoints(pos, getGlobalPosition(T[j]));
      if (j == 0 || tmp < nearest_Opp_ang_K[i]) {
        nearest_Opp_ang_K[i] = tmp;
      }
    }
  }

  int j = 0;
  state[j++] = WK0_dist_to_C;

  for (int i = 1; i < numTeammates; i++)
    state[j++] = dist_to_K0_K[i];

  for (int i = 0; i < numOpponents; i++)
    state[j++] = dist_to_K0_T[i];

  for (int i = 1; i < numTeammates; i++)
    state[j++] = dist_to_C_K[i];

  for (int i = 0; i < numOpponents; i++)
    state[j++] = dist_to_C_T[i];

  for (int i = 1; i < numTeammates; i++)
    state[j++] = nearest_Opp_dist_K[i];

  for (int i = 1; i < numTeammates; i++)
    state[j++] = nearest_Opp_ang_K[i];

  state[j++] = dist_to_K0_B;

  state[j++] = dist_to_C_B;

  return j;
}

// Greg:
// This really doesn't belong here,
// because it is related to a specific
// learner.  I don't know exactly
// where to put it, though because
// I want to keep the LinearSarsa
// class generic.

int WorldModel::getStateRangesAndResolutions(double ranges[],
                                             double minValues[],
                                             double resolutions[],
                                             int numTeammates, int numOpponents) {
  int j = 0;

  double maxRange = hypot(25, 25);

  ranges[j] = maxRange / 2.0;        // dist_to_K0_center
  minValues[j] = 0;
  resolutions[j++] = 2.0;

  for (int i = 1; i < numTeammates; i++) {     // dist_to_K0_Teammate
    ranges[j] = maxRange;
    minValues[j] = 0;
    resolutions[j++] = 2.0 + (i - 1) / (numTeammates - 1);
  }

  for (int i = 0; i < numOpponents; i++) {     // dist_to_K0_Opponent
    ranges[j] = maxRange;
    minValues[j] = 0;
    resolutions[j++] = 3.0 + (i - 1) / (numOpponents - 1);
  }

  for (int i = 1; i < numTeammates; i++) {     // dist_to_center_Teammate
    ranges[j] = maxRange / 2.0;
    minValues[j] = 0;
    resolutions[j++] = 2.0 + (i - 1) / (numTeammates - 1);
  }

  for (int i = 0; i < numOpponents; i++) {     // dist_to_center_Opponent
    ranges[j] = maxRange / 2.0;
    minValues[j] = 0;
    resolutions[j++] = 3.0;
  }

  for (int i = 1; i < numTeammates; i++) {     // nearest_Opp_dist_Teammate
    ranges[j] = maxRange / 2.0;
    minValues[j] = 0;
    resolutions[j++] = 4.0;
  }

  for (int i = 1; i < numTeammates; i++) {     // nearest_Opp_ang_Teammate
    ranges[j] = 180;
    minValues[j] = 0;
    resolutions[j++] = 10.0;
  }

  ranges[j] = maxRange;        // dist_to_K0_B
  minValues[j] = 0.0;
  resolutions[j++] = 1.0;

  ranges[j] = maxRange / 2.0;        // dist_to_C_B
  minValues[j] = 0.0;
  resolutions[j++] = 2.0;

  return j;
}

// Yaxin: changed from keeperTileWidths to keeperResolutions and keeperRanges,
int WorldModel::keeperStateRangesAndResolutions(
    double ranges[],
    double minValues[],
    double resolutions[],
    int numK, int numT) {
  if (numK < 3) {
    cerr << "keeperTileWidths: num keepers must be at least 3, found: "
         << numK << endl;
    return 0;
  }

  if (numT < 2) {
    cerr << "keeperTileWidths: num takers must be at least 2, found: "
         << numT << endl;
    return 0;
  }

  return getStateRangesAndResolutions(ranges, minValues, resolutions, numK, numT);
}

int WorldModel::takerStateRangesAndResolutions(
    double ranges[],
    double minValues[],
    double resolutions[],
    int numK, int numT) {
  if (numK < 3) {
    cerr << "keeperTileWidths: num keepers must be at least 3, found: "
         << numK << endl;
    return 0;
  }

  if (numT < 2) {
    cerr << "keeperTileWidths: num takers must be at least 2, found: "
         << numT << endl;
    return 0;
  }

  return getStateRangesAndResolutions(ranges, minValues, resolutions, numT, numK);
}

void WorldModel::setMoveSpeed(double speed) {
  m_moveSpeed = speed;
}

double WorldModel::getMoveSpeed() {
  return m_moveSpeed;
}

void WorldModel::setKeepawayRectReduced(VecPosition pos1, VecPosition pos2) {
  m_keepawayRectReduced.setRectanglePoints(pos1, pos2);
}

Rect WorldModel::getKeepawayRectReduced() {
  return m_keepawayRectReduced;
}

void WorldModel::setKeepawayRect(VecPosition pos1, VecPosition pos2) {
  m_keepawayRect.setRectanglePoints(pos1, pos2);
}

Rect WorldModel::getKeepawayRect() {
  return m_keepawayRect;
}
