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
#include <cstring>

#if USE_DRAW_LOG
extern LoggerDraw LogDraw;
#endif

KeepawayPlayer::KeepawayPlayer( SMDPAgent* sa, ActHandler* act, WorldModel *wm,
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
  bContLoop     = true;
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
  Timing timer;

  // wait for new information from the server
  // cannot say bContLoop=WM->wait... since bContLoop can be changed elsewhere
  if(!WM->waitForNewInformation())
    bContLoop =  false;

  while( bContLoop )                                 // as long as server alive
  {
    Log.logWithTime( 3, "  start update_all" );
    Log.setHeader( WM->getCurrentCycle() );

#if USE_DRAW_LOG
    LogDraw.setTime( WM->getCurrentCycle() );
#endif

    if(WM->updateAll( ))
    {
      //Log.log( 101, "KeepawayPlayer::mainLoop: keeper/taker main loop" );

      timer.restartTime();
      SoccerCommand soc;

      if ( WM->getSide() == SIDE_LEFT )
        soc = keeper();
      else
        soc = taker();

      if(shallISaySomething())           // shall I communicate
      {
        m_timeLastSay = WM->getCurrentTime();
        char strMsg[MAX_SAY_MSG];
        makeSayMessage( soc, strMsg );
        if( strlen( strMsg ) != 0 )
          Log.log( 600, "send communication string: %s", strMsg );
        WM->setCommunicationString( strMsg );
      }

      Log.logWithTime( 3, "  determined action; waiting for new info" );
      // directly after see message, will not get better info, so send commands
      if( WM->getTimeLastSeeMessage() == WM->getCurrentTime() ||
          (SS->getSynchMode() && WM->getRecvThink()))
      {
        Log.logWithTime( 3, "  send messages directly" );
        ACT->sendCommands( );
        Log.logWithTime( 3, "  sent messages directly" );
        if(SS->getSynchMode())
        {
          WM->processRecvThink( false );
          ACT->sendMessageDirect( "(done)" );
        }
      }
    }
    else
      Log.logWithTime( 3, "  HOLE no action determined; waiting for new info");

#if USE_DRAW_LOG
    int iIndex;
    double dConfThr = PS->getPlayerConfThr();
    char buffer[128];
    for( ObjectT o = WM->iterateObjectStart( iIndex, OBJECT_SET_TEAMMATES, dConfThr);
         o != OBJECT_ILLEGAL;
         o = WM->iterateObjectNext ( iIndex, OBJECT_SET_TEAMMATES, dConfThr ) ) {
      LogDraw.logCircle( "Players", WM->getGlobalPosition( o ), 1.6, 80,
                         false,
                         COLOR_ORANGE, WM->getConfidence( o ) );
      sprintf( buffer, "%d", SoccerTypes::getIndex( o ) + 1 );
      LogDraw.logText( "Players", WM->getGlobalPosition( o ),
                       buffer,
                       80, COLOR_ORANGE );
    }
    // Me
    ObjectT o = WM->getAgentObjectType();
    LogDraw.logCircle( "Players", WM->getGlobalPosition( o ), 1.6, 81,
                       false,
                       COLOR_PURPLE, WM->getConfidence( o ) );
    sprintf( buffer, "%d", SoccerTypes::getIndex( o ) + 1 );
    LogDraw.logText( "Players", WM->getGlobalPosition( o ),
                     buffer,
                     81, COLOR_PURPLE );
    for( ObjectT o = WM->iterateObjectStart( iIndex, OBJECT_SET_OPPONENTS, dConfThr);
         o != OBJECT_ILLEGAL;
         o = WM->iterateObjectNext ( iIndex, OBJECT_SET_OPPONENTS, dConfThr ) ) {
      LogDraw.logCircle( "Players", WM->getGlobalPosition( o ), 1.6, 80,
                         false,
                         COLOR_PINK, WM->getConfidence( o ) );
      sprintf( buffer, "%d", SoccerTypes::getIndex( o ) + 1 );
      LogDraw.logText( "Players", WM->getGlobalPosition( o ),
                       buffer,
                       80, COLOR_PINK );
    }

    Log.logWithTime( 604, "time for action: %f", timer.getElapsedTime()*1000 );
#endif

    // wait for new information from the server cannot say
    // bContLoop=WM->wait... since bContLoop can be changed elsewhere
    if(!WM->waitForNewInformation())
      bContLoop =  false;
  }

  // shutdown, print hole and number of players seen statistics
  SA->shutDown();
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
  //Log.log( 101, "KeepawayPlayer::keeper: decision" );

  SoccerCommand soc;

  if ( WM->isNewEpisode() ) {
    SA->endEpisode( WM->getCurrentCycle(), WM->keeperReward(SA->lastActionTime) );
    WM->setNewEpisode( false );
  }

#if USE_DRAW_LOG
  LogDraw.logCircle( "ball belief", WM->getBallPos(),
                     1.1, 11, false, COLOR_RED,
                     WM->getConfidence( OBJECT_BALL ) );
  char buffer[128];
  sprintf( buffer, "%.2f", WM->getConfidence( OBJECT_BALL ) );
  LogDraw.logText( "ball belief", WM->getBallPos(),
                   buffer,
                   11, COLOR_RED );
#endif

  // If we don't know where the ball is, search for it.
  if (WM->getConfidence(OBJECT_BALL) < PS->getBallConfThr()) {
    ACT->putCommandInQueue( soc = searchBall() );
    ACT->putCommandInQueue( alignNeckWithBody() );
#if USE_DRAW_LOG
    LogDraw.logText( "state", VecPosition( 25, 25 ),
                     "lost ball",
                     1, COLOR_WHITE );
#endif
    return soc;
  }

  return fullTeamKeepers();
}

SoccerCommand KeepawayPlayer::fullTeamKeepers()
{
  double state[ MAX_STATE_VARS ];
  if ( WM->keeperStateVars( state ) <= 0 ) {
    return interpretKeeperAction(0); // do nothing
  }

  int numK = WM->getNumKeepers();
  ObjectT K[ numK ];
  for ( int i = 0; i < numK; i++ )
    K[ i ] = SoccerTypes::getTeammateObjectFromIndex( i );

  ObjectT K0 = WM->getClosestInSetTo(OBJECT_SET_TEAMMATES, OBJECT_BALL);
  if (!WM->sortClosestTo(K, numK, K0))
    return interpretKeeperAction(0);

  int agentIdx = 0;
  while (agentIdx < numK && K[agentIdx] != WM->getAgentObjectType()) agentIdx += 1;
  if (agentIdx >= numK) return interpretKeeperAction(0);

  // if within an option
  if (SA->lastAction >= 0) {
    Log.log(101, "in option %s agentIdx %d",
            JointActionSpace::ins().getJointAction(SA->lastAction)->name().c_str(), agentIdx);
    auto ja = JointActionSpace::ins().getJointAction(SA->lastAction);
    auto aa = ja->actions[0]; // the action of the leading agent

    bool passing = SA->lastActionTime == WM->getCurrentCycle() - 1 &&
        JointActionSpace::ins().getJointAction(SA->lastAction)->actions[0]->type == AAT_PassTo;

    if (passing || !aa->terminated(state, SA->getNumFeatures())) {
      return interpretFullTeamKeeperAction(SA->lastAction, agentIdx);
    }
    else {
      SA->lastAction = -1;
    }
  }

  // K0 makes the decision
  if (agentIdx == 0) {
    int action;
    if ( SA->lastActionTime == UnknownTime ) {
      action = SA->startEpisode(WM->getCurrentCycle(), state);
    }
    else { // Call step() on all but first SMDP step
      action = SA->step( WM->getCurrentCycle(), WM->keeperReward(SA->lastActionTime), state );
    }
    Log.log(101, "execute %s agentIdx %d",
            JointActionSpace::ins().getJointAction(action)->name().c_str(), agentIdx);
    return interpretFullTeamKeeperAction(action, agentIdx);
  }
  else {
    return interpretKeeperAction(0); // do nothing at this cycle
  }
}

SoccerCommand KeepawayPlayer::interpretFullTeamKeeperAction( int action, int agentIdx )
{
  auto ja = JointActionSpace::ins().getJointAction(action);
  auto aa = ja->actions[agentIdx];

  return aa->execute(this);
}

SoccerCommand KeepawayPlayer::interpretKeeperAction( int action )
{
  SoccerCommand soc;

  if ( action == 0 ) { // interpret HOLD action
    ACT->putCommandInQueue( soc = moveToPos(WM->getAgentGlobalPosition(), 30.0) );
  }

  return soc;
}

SoccerCommand KeepawayPlayer::taker()
{
  SoccerCommand soc;

//  LogDraw.logCircle( "ball pos", WM->getBallPos(),
//                     1.1, 11, false, COLOR_RED, WM->getConfidence( OBJECT_BALL ) );

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
    return soc;
  }

  // If not first (or second) closest, then mark open opponent
  int numT = WM->getNumTakers();
  ObjectT T[ numT ];
  for ( int i = 0; i < numT; i++ )
    T[ i ] = SoccerTypes::getTeammateObjectFromIndex( i );
  WM->sortClosestTo( T, numT, OBJECT_BALL );

  if (numT > 1 && T[0] != WM->getAgentObjectType()
    /*T[ 1 ] != WM->getAgentObjectType()*/ ) {
    ObjectT withBall = WM->getFastestInSetTo( OBJECT_SET_OPPONENTS,
                                              OBJECT_BALL );
    ACT->putCommandInQueue( soc = markMostOpenOpponent( withBall ) );
    ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
    return soc;
  }

  // If teammate has it, don't mess with it
  double dDist;
  ObjectT closest = WM->getClosestInSetTo( OBJECT_SET_PLAYERS,
                                           OBJECT_BALL, &dDist );

  if ( SoccerTypes::isTeammate( closest ) &&
       closest != WM->getAgentObjectType() &&
       dDist < SS->getMaximalKickDist() ) {
    ACT->putCommandInQueue( soc = turnBodyToObject( OBJECT_BALL ) );
    ACT->putCommandInQueue( alignNeckWithBody() );
    return soc;
  }


  // Otherwise try to intercept the ball
  ACT->putCommandInQueue( soc = intercept( false ) );
  ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
  return soc;
}
