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

#include "KeepawayPlayer.h"
#include "SayMsgEncoder.h"
#include "HierarchicalFSM.h"
#include "ChoicePoint.h"


KeepawayPlayer::KeepawayPlayer(jol::SMDPAgent *sa, ActHandler *act, WorldModel *wm,
                               ServerSettings *ss, PlayerSettings *ps,
                               char* strTeamName, int iNumKeepers, int iNumTakers,
                               double dVersion, int iReconnect )

{
  char str[MAX_MSG];

  SA            = sa;
  ACT           = act;
  WM            = wm;
  SS            = ss;
  PS            = ps;
  WM->setTeamName( strTeamName );
  WM->setNumKeepers( iNumKeepers );
  WM->setNumTakers( iNumTakers );
  WM->setNewEpisode( false );
  m_timeLastSay = -5;

  // create initialisation string
  if( iReconnect != -1 )
    sprintf( str, "(reconnect %s %d)", strTeamName, iReconnect );
  else
    sprintf( str, "(init %s (version %f))", strTeamName, dVersion );
  ACT->sendMessage( str );
}

/*! This is the main loop of the agent. This method calls the update methods
    of the world model after it is indicated that new information has arrived.
    After this, the correct main loop of the player type is called, which
    puts the best soccer command in the queue of the ActHandler. */
void KeepawayPlayer::mainLoop( )
{
  Log.setHeader(-1);
  bool bContLoop = true;

  // do initialization stuff
  if (!WM->waitForNewInformation()) bContLoop = false;
  if (bContLoop) WM->updateAll();

  if (/*WM->getSide() == SIDE_RIGHT ||*/ SA) { // joint option learner or taker
    if (!WM->waitForNewInformation()) bContLoop = false;

    while (bContLoop)                                 // as long as server alive
    {
      Log.logWithTime(3, "  start update_all");
      Log.setHeader(WM->getCurrentCycle());

      if (WM->updateAll()) {
        SoccerCommand soc;

        if (WM->getSide() == SIDE_LEFT)
          soc = keeper();
        else
          soc = taker();

        if (shallISaySomething())           // shall I communicate
        {
          m_timeLastSay = WM->getCurrentTime();
          char strMsg[MAX_SAY_MSG];
          makeSayMessage(soc, strMsg);
          if (strlen(strMsg) != 0)
            Log.log(600, "send communication string: %s", strMsg);
          WM->setCommunicationString(strMsg);
        }

        Log.logWithTime(3, "  determined action; waiting for new info");
        // directly after see message, will not get better info, so send commands
        if (WM->getTimeLastSeeMessage() == WM->getCurrentTime() ||
            (SS->getSynchMode() && WM->getRecvThink())) {
          Log.logWithTime(3, "  send messages directly");
          ACT->sendCommands();
          Log.logWithTime(3, "  sent messages directly");
          if (SS->getSynchMode()) {
            WM->processRecvThink(false);
            ACT->sendMessageDirect("(done)");
          }
        }
      } else {
        Log.logWithTime(3, "  HOLE no action determined; waiting for new info");
      }

      // wait for new information from the server cannot say
      // bContLoop=WM->wait... since bContLoop can be changed elsewhere
      if (!WM->waitForNewInformation())
        bContLoop = false;
    }
  } else { // hierarchical FSM learner
    fsm::HierarchicalFSM *player = 0;
    if (WM->getSide() == SIDE_LEFT) {
      player = new fsm::Keeper(this);
    } else {
      player = new fsm::Taker(this);
    }
    fsm::Run(player).operator()();
    delete player;
  }

  // shutdown, print hole and number of players seen statistics
  if (SA) SA->shutDown();
  printf("Shutting down player %d\n", WM->getPlayerNumber() );
  printf("   Number of holes: %d (%f)\n", WM->iNrHoles,
         ((double)WM->iNrHoles/WM->getCurrentCycle())*100 );
  printf("   Teammates seen: %d (%f)\n", WM->iNrTeammatesSeen,
         ((double)WM->iNrTeammatesSeen/WM->getCurrentCycle()));
  printf("   Opponents seen: %d (%f)\n", WM->iNrOpponentsSeen,
         ((double)WM->iNrOpponentsSeen/WM->getCurrentCycle()));
}


/********************** SAY **************************************************/

/*!This method determines whether a player should say something.
   \return bool indicating whether the agent should say a message */
bool KeepawayPlayer::shallISaySomething()
{
  bool        bReturn;

  bReturn  = ((WM->getCurrentTime() - m_timeLastSay) >= SS->getHearDecay());
  bReturn  &= (WM->getCurrentCycle() > 0 );

  return bReturn;
}

void KeepawayPlayer::makeSayMessage( SoccerCommand soc, char * strMsg )
{
  VecPosition posBall = WM->getGlobalPosition( OBJECT_BALL );
  VecPosition velBall = WM->getGlobalVelocity( OBJECT_BALL );
  int iDiff = 0;
  SayMsgEncoder myencoder;

  VecPosition posBallPred;
  WM->predictBallInfoAfterCommand( soc, &posBallPred );
  VecPosition posAgentPred = WM->predictAgentPosAfterCommand( soc );

  // If we have good information about the ball
  if( ( WM->getTimeChangeInformation(OBJECT_BALL) == WM->getCurrentTime() &&
        WM->getRelativeDistance( OBJECT_BALL ) < 20.0 &&
        WM->getTimeLastSeen( OBJECT_BALL ) == WM->getCurrentTime() )
      ||
      (
          WM->getRelativeDistance( OBJECT_BALL ) < SS->getVisibleDistance() &&
          WM->getTimeLastSeen( OBJECT_BALL ) == WM->getCurrentTime()
      )
      ||
      (
          WM->getRelativeDistance( OBJECT_BALL ) < SS->getMaximalKickDist() &&
          posBallPred.getDistanceTo( posAgentPred ) > SS->getMaximalKickDist()
      )
      )
  {
    // If we are kicking the ball
    if( WM->getRelativeDistance( OBJECT_BALL ) < SS->getMaximalKickDist() )
    {
      // if kick and a pass
      if( soc.commandType == CMD_KICK )
      {
        WM->predictBallInfoAfterCommand( soc, &posBall, &velBall );
        VecPosition posAgent = WM->predictAgentPos( 1, 0 );
        if( posBall.getDistanceTo( posAgent ) > SS->getMaximalKickDist() + 0.2 )
          iDiff = 1;
      }

      if( iDiff == 0 )
      {
        posBall = WM->getGlobalPosition( OBJECT_BALL );
        velBall.setVecPosition( 0, 0 );
      }
    }

#if USE_DRAW_LOG
    LogDraw.logCircle( "ball sending", posBall,
                       1.1, 90, false, COLOR_BLUE );
#endif
    myencoder.add( new BallInfo((float) posBall.getX(), (float) posBall.getY(),
                                (float) velBall.getX(), (float) velBall.getY(), (unsigned short) (1 - iDiff)) );
  }

  // Find closest opponent that was seen this cycle
  int numT = WM->getNumTakers();
  ObjectT T[ numT ];
  int numSeen = 0;
  for ( int i = 0; i < numT; i++ ) {
    T[ numSeen ] = SoccerTypes::getOpponentObjectFromIndex( i );
    if ( WM->getRelativeDistance( T[ numSeen ] ) < SS->getVisibleDistance() &&
         WM->getTimeLastSeen( T[ numSeen ] ) == WM->getCurrentTime() )
      numSeen++;  // store this opponent if we just saw him
  }
  WM->sortClosestTo( T, numSeen, WM->getAgentObjectType() );

  if ( numSeen > 0 ) { // add closest
    VecPosition posOpp = WM->getGlobalPosition( T[ 0 ] );
    myencoder.add( new OppPos((unsigned short) (SoccerTypes::getIndex(T[ 0 ] ) + 1),
                              (float) posOpp.getX(), (float) posOpp.getY(), 1 ) );
  }

  if ( myencoder.getSize() <= 7 &&  // if there is room
       WM->getConfidence( WM->getAgentObjectType() ) > PS->getPlayerHighConfThr() ) {
    myencoder.add( new OurPos((float) posAgentPred.getX(), (float) posAgentPred.getY()) );
  }

  strcpy( strMsg, myencoder.getEncodedStr().c_str() );
  myencoder.clear();
}

/********************** Keepaway ***************************************/

SoccerCommand KeepawayPlayer::keeper()
{
  SoccerCommand soc;

  // If we don't know where the ball is, search for it.
  if (WM->getCurrentCycle() < 1 || WM->getConfidence(OBJECT_BALL) < PS->getBallConfThr()) {
    ACT->putCommandInQueue( soc = searchBall() );
    ACT->putCommandInQueue( alignNeckWithBody() );
    return soc;
  }

  return jolKeepers();
}

SoccerCommand KeepawayPlayer::jolKeepers()
{
  static ObjectT savedK[11];
  static double state[MAX_RL_STATE_VARS];
  memset(state, 0, sizeof(state));

  int numK = WM->getNumKeepers();

  int features = WM->keeperStateVars(state);
  Assert(features == 0 || features == SA->getNumFeatures());
  if (features != SA->getNumFeatures()) return idle("features != SA->getNumFeatures()"); // do nothing
  Log.log(101, "isTmControllBall %d", WM->isTmControllBall());

  ObjectT K[11];
  memset(K, 0, sizeof(K));

  for (int i = 0; i < numK; i++)
    K[i] = SoccerTypes::getTeammateObjectFromIndex(i);

  ObjectT K0 = WM->getClosestInSetTo(OBJECT_SET_TEAMMATES, OBJECT_BALL);
  if (!WM->sortClosestTo(K, numK, K0)) return idle("!WM->sortClosestTo(teammate, numK, K0)");
  if (K0 != K[0]) return idle("K0 != teammate[0]");

  int &agentIdx = SA->agentIdx;
  agentIdx = 0;
  while (agentIdx < numK && K[agentIdx] != WM->getAgentObjectType()) agentIdx += 1;
  Assert(agentIdx < numK);
  Log.log(101, "agentIdx %d", agentIdx);

  if (agentIdx >= numK) return idle("agentIdx >= numK");

  // if running an option
  if (SA->lastAction >= 0 && !WM->isNewEpisode()) {
    auto ja = jol::JointActionSpace::ins().getJointAction(SA->lastAction);
    auto aa = ja->actions[0]; // the action of the leading agent

    int idx = 0;
    while (idx < numK && savedK[idx] != WM->getAgentObjectType()) idx += 1;
    Assert(idx < numK);
    if (idx >= numK) return idle("idx >= numK");

    if (!aa->terminated(this, savedK)) {
      Log.log(101, "continue option %d:%s [%d]", SA->lastAction,
              jol::JointActionSpace::ins().getJointActionName(SA->lastAction), idx);
      return jolExecute(SA->lastAction, idx, savedK);
    } else {
      Log.log(101, "terminated option %d:%s [%d]", SA->lastAction,
              jol::JointActionSpace::ins().getJointActionName(SA->lastAction), idx);
    }
  }

  Assert(agentIdx != 0 || !WM->isTmControllBall() || WM->isBallKickable());

  if (WM->isNewEpisode()) {
    SA->endEpisode(WM->keeperReward(SA->lastActionTime));
  }

  int action;
  if (WM->isNewEpisode() || SA->lastActionTime == UnknownTime || SA->lastAction == -1) {
    Assert(SA->lastAction == -1);
    if (WM->isNewEpisode()) WM->setNewEpisode(false);
    action = SA->startEpisode(state);
  } else { // Call step() on all but first SMDP step
    action = SA->step(WM->keeperReward(SA->lastActionTime), state);
  }

  Log.log(101, "execute option %d:%s [%d]", action,
          jol::JointActionSpace::ins().getJointActionName(action), SA->agentIdx);

  memcpy(savedK, K, sizeof(K)); // save teammate[]
  Log.log(101, "sync save K0:%d K1:%d K2:%d", savedK[0], savedK[1], savedK[2]);

  SA->lastActionTime = WM->getCurrentCycle();
  return jolExecute(action, SA->agentIdx, savedK);
}

SoccerCommand KeepawayPlayer::jolExecute(int action, int agentIdx, ObjectT K[])
{
  auto ja = jol::JointActionSpace::ins().getJointAction(action);
  auto aa = ja->actions[agentIdx];

  return aa->execute(this, K);
}

SoccerCommand KeepawayPlayer::idle(string error)
{
  SoccerCommand soc;
  ACT->putCommandInQueue( soc = turnBodyToObject( OBJECT_BALL ) );
  ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
  Log.log(101, "idle (error: %s)", error.c_str());
  return soc;
}

SoccerCommand KeepawayPlayer::taker()
{
  SoccerCommand soc;

  // If we don't know where the ball is, search for it.
  if ( WM->getConfidence( OBJECT_BALL ) <
       PS->getBallConfThr() ) {
    ACT->putCommandInQueue( soc = searchBall() );
    ACT->putCommandInQueue( alignNeckWithBody() );
    return soc;
  }

  // Maintain possession if you have the ball.
  if ( WM->isBallKickable() &&
       WM->getClosestInSetTo( OBJECT_SET_TEAMMATES, OBJECT_BALL) ==
       WM->getAgentObjectType() ) {
    ACT->putCommandInQueue( soc = holdBall( 0.3 ) );
    Log.log(101, "holdBall( 0.3 )");
    return soc;
  }

  // If not first (or second) closest, then mark open opponent
  int numT = WM->getNumTakers();
  ObjectT T[ 11 ];
  for ( int i = 0; i < numT; i++ )
    T[ i ] = SoccerTypes::getTeammateObjectFromIndex( i );
  WM->sortClosestTo( T, numT, OBJECT_BALL );

  if (numT > 1 && T[0] != WM->getAgentObjectType() &&
      T[ 1 ] != WM->getAgentObjectType() ) {
    ObjectT withBall = WM->getFastestInSetTo( OBJECT_SET_OPPONENTS,
                                              OBJECT_BALL );
    Log.log(101, "markMostOpenOpponent( withBall )");
    ACT->putCommandInQueue( soc = markMostOpenOpponent( withBall ) );
    ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
    return soc;
  }

  // If teammate has it, don't mess with it
  double dDist = std::numeric_limits<double>::max();
  ObjectT closest = WM->getClosestInSetTo( OBJECT_SET_PLAYERS,
                                           OBJECT_BALL, &dDist );

  if ( SoccerTypes::isTeammate( closest ) &&
       closest != WM->getAgentObjectType() &&
       dDist < WM->getMaximalKickDist(closest)) {
    Log.log(101, "turnBodyToObject( OBJECT_BALL )");
    ACT->putCommandInQueue( soc = turnBodyToObject( OBJECT_BALL ) );
    ACT->putCommandInQueue( alignNeckWithBody() );
    return soc;
  }

  closest = WM->getClosestInSetTo(OBJECT_SET_OPPONENTS,
                                  OBJECT_BALL, &dDist);

  if (SoccerTypes::isOpponent(closest) &&
      dDist < WM->getMaximalKickDist(closest)) { // opp can kick
    Log.log(101, "move to ball");
    soc = moveToPos(WM->getBallPos(), 30.0);
  }
  else {
    Log.log(101, "intercept");
    soc = intercept( false );
  }

  ACT->putCommandInQueue( soc );
  ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
  return soc;
}
