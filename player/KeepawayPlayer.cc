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
#include <cstring>
#include "Parse.h"


// KeepawayPlayer::KeepawayPlayer(SMDPAgent* sa, ActHandler *act, WorldModel *wm,
//                                ServerSettings *ss, PlayerSettings *ps,
//                                char* strTeamName, int iNumKeepers, int iNumTakers,
//                                double dVersion, int iReconnect )

// {
//   // char str[MAX_MSG];

//   // ACT           = act;
//   // WM            = wm;
//   // SS            = ss;
//   // PS            = ps;
//   // WM->setTeamName( strTeamName );
//   // WM->setNumKeepers( iNumKeepers );
//   // WM->setNumTakers( iNumTakers );
//   // WM->setNewEpisode( false );
//   // m_timeLastSay = -5;

//   char str[MAX_MSG];
  
//   SA            = sa;
//   ACT           = act;
//   WM            = wm;
//   SS            = ss;
//   PS            = ps;
//   bContLoop     = true;
//   WM->setTeamName( strTeamName );
//   WM->setNumKeepers( iNumKeepers );
//   WM->setNumTakers( iNumTakers ); 
//   WM->setNewEpisode( false );
//   // WM->setLastAction( UnknownIntValue );
//   m_timeLastSay = -5;
//   m_timeStartEpisode = -5;


//   // create initialisation string
//   if( iReconnect != -1 )
//     sprintf( str, "(reconnect %s %d)", strTeamName, iReconnect );
//   else
//     sprintf( str, "(init %s (version %f))", strTeamName, dVersion );
//   ACT->sendMessage( str );
// }

// /*! This is the main loop of the agent. This method calls the update methods
//     of the world model after it is indicated that new information has arrived.
//     After this, the correct main loop of the player type is called, which
//     puts the best soccer command in the queue of the ActHandler. */
// void KeepawayPlayer::mainLoop( )
// {

//   Log.setHeader(-1);
//   bool bContLoop = true;

//   // do initialization stuff
//   // if (!WM->waitForNewInformation()) bContLoop = false;
//   // if (bContLoop) WM->updateAll();

//   // fsm::HierarchicalFSM *player = 0;
//   // if (WM->getSide() == SIDE_LEFT) {
//   //   player = new fsm::Keeper(this);
//   // } else {
//   //   player = new fsm::Taker(this);
//   // }
//   // fsm::Run(player).operator()();
//   // delete player;




//   // // shutdown, print hole and number of players seen statistics
//   // printf("Shutting down player %d\n", WM->getPlayerNumber() );
//   // printf("   Number of holes: %d (%f)\n", WM->iNrHoles,
//   //        ((double)WM->iNrHoles/WM->getCurrentCycle())*100 );
//   // printf("   Teammates seen: %d (%f)\n", WM->iNrTeammatesSeen,
//   //        ((double)WM->iNrTeammatesSeen/WM->getCurrentCycle()));
//   // printf("   Opponents seen: %d (%f)\n", WM->iNrOpponentsSeen,
//   //        ((double)WM->iNrOpponentsSeen/WM->getCurrentCycle()));


//   Timing timer;

//   // wait for new information from the server
//   // cannot say bContLoop=WM->wait... since bContLoop can be changed elsewhere
//   if(  WM->waitForNewInformation() == false )
//     bContLoop =  false;

//   while( bContLoop )                                 // as long as server alive
//   {
//     Log.logWithTime( 3, "  start update_all" );
//     Log.setHeader( WM->getCurrentCycle() );
//     // LogDraw.setTime( WM->getCurrentCycle() );

//     if( WM->updateAll( ) == true )
//     {
//       timer.restartTime();
//       SoccerCommand soc;

//       if ( WM->getSide() == SIDE_LEFT )
//         soc = keeper();
//       else
//         soc = taker();

//       if( shallISaySomething() == true )           // shall I communicate
//         {
//           m_timeLastSay = WM->getCurrentTime();
//           char strMsg[MAX_SAY_MSG];
//           makeSayMessage( soc, strMsg );
//           if( strlen( strMsg ) != 0 )
//             Log.log( 600, "send communication string: %s", strMsg );
//           WM->setCommunicationString( strMsg );
//         }
//       Log.logWithTime( 3, "  determined action; waiting for new info" );
//       // directly after see message, will not get better info, so send commands
//       if( WM->getTimeLastSeeMessage() == WM->getCurrentTime() ||
//           (SS->getSynchMode() == true && WM->getRecvThink() == true ))
//       {
//         Log.logWithTime( 3, "  send messages directly" );
//         ACT->sendCommands( );
//         Log.logWithTime( 3, "  sent messages directly" );
//         if( SS->getSynchMode() == true  )
//         {
//           WM->processRecvThink( false );
//           ACT->sendMessageDirect( "(done)" );
//         }
//       }
//     }  
//     else
//       Log.logWithTime( 3, "  HOLE no action determined; waiting for new info");

//     int iIndex;
//     double dConfThr = PS->getPlayerConfThr();
//     char buffer[128];
//     for( ObjectT o = WM->iterateObjectStart( iIndex, OBJECT_SET_TEAMMATES, dConfThr);
//            o != OBJECT_ILLEGAL;
//            o = WM->iterateObjectNext ( iIndex, OBJECT_SET_TEAMMATES, dConfThr ) ) {
//       // LogDraw.logCircle( "Players", WM->getGlobalPosition( o ), 1.6, 80,
//       //                    false,
//       //                    COLOR_ORANGE, WM->getConfidence( o ) );
//       sprintf( buffer, "%d", SoccerTypes::getIndex( o ) + 1 );
//       // LogDraw.logText( "Players", WM->getGlobalPosition( o ),
//       //                  buffer,
//       //                  80, COLOR_ORANGE );
//     }
//     // Me
//     ObjectT o = WM->getAgentObjectType();
//     // LogDraw.logCircle( "Players", WM->getGlobalPosition( o ), 1.6, 81,
//     //                    false,
//     //                    COLOR_PURPLE, WM->getConfidence( o ) );
//     sprintf( buffer, "%d", SoccerTypes::getIndex( o ) + 1 );

//     // LogDraw.logText( "Players", WM->getGlobalPosition( o ),
//     //                  buffer,
//     //                  81, COLOR_PURPLE );
//     for( ObjectT o = WM->iterateObjectStart( iIndex, OBJECT_SET_OPPONENTS, dConfThr);
//            o != OBJECT_ILLEGAL;
//            o = WM->iterateObjectNext ( iIndex, OBJECT_SET_OPPONENTS, dConfThr ) ) {
//       // LogDraw.logCircle( "Players", WM->getGlobalPosition( o ), 1.6, 80,
//       //                    false,
//       //                    COLOR_PINK, WM->getConfidence( o ) );
//       sprintf( buffer, "%d", SoccerTypes::getIndex( o ) + 1 );
//       // LogDraw.logText( "Players", WM->getGlobalPosition( o ),
//       //                  buffer,
//       //                  80, COLOR_PINK );
//     }

//     Log.logWithTime( 604, "time for action: %f", timer.getElapsedTime()*1000 );
           
//     // wait for new information from the server cannot say
//     // bContLoop=WM->wait... since bContLoop can be changed elsewhere
//     if(  WM->waitForNewInformation() == false )
//         bContLoop =  false;
//   }

//   // shutdown, print hole and number of players seen statistics
//   SA->shutDown();
//   printf("Shutting down player %d\n", WM->getPlayerNumber() );
//   printf("   Number of holes: %d (%f)\n", WM->iNrHoles,
//                          ((double)WM->iNrHoles/WM->getCurrentCycle())*100 );
//   printf("   Teammates seen: %d (%f)\n", WM->iNrTeammatesSeen,
//                          ((double)WM->iNrTeammatesSeen/WM->getCurrentCycle()));
//   printf("   Opponents seen: %d (%f)\n", WM->iNrOpponentsSeen,
//                          ((double)WM->iNrOpponentsSeen/WM->getCurrentCycle()));



// }


// /********************** SAY **************************************************/

// /*!This method determines whether a player should say something.
//    \return bool indicating whether the agent should say a message */
// bool KeepawayPlayer::shallISaySomething()
// {
//   bool        bReturn;

//   bReturn  = ((WM->getCurrentTime() - m_timeLastSay) >= SS->getHearDecay());
//   bReturn  &= (WM->getCurrentCycle() > 0 );

//   return bReturn;
// }

// void KeepawayPlayer::makeSayMessage( SoccerCommand soc, char * strMsg )
// {
//   VecPosition posBall = WM->getGlobalPosition( OBJECT_BALL );
//   VecPosition velBall = WM->getGlobalVelocity( OBJECT_BALL );
//   int iDiff = 0;
//   SayMsgEncoder myencoder;

//   VecPosition posBallPred;
//   WM->predictBallInfoAfterCommand( soc, &posBallPred );
//   VecPosition posAgentPred = WM->predictAgentPosAfterCommand( soc );

//   // If we have good information about the ball
//   if( ( WM->getTimeChangeInformation(OBJECT_BALL) == WM->getCurrentTime() &&
//         WM->getRelativeDistance( OBJECT_BALL ) < 20.0 &&
//         WM->getTimeLastSeen( OBJECT_BALL ) == WM->getCurrentTime() )
//       ||
//       (
//           WM->getRelativeDistance( OBJECT_BALL ) < SS->getVisibleDistance() &&
//           WM->getTimeLastSeen( OBJECT_BALL ) == WM->getCurrentTime()
//       )
//       ||
//       (
//           WM->getRelativeDistance( OBJECT_BALL ) < SS->getMaximalKickDist() &&
//           posBallPred.getDistanceTo( posAgentPred ) > SS->getMaximalKickDist()
//       )
//       )
//   {
//     // If we are kicking the ball
//     if( WM->getRelativeDistance( OBJECT_BALL ) < SS->getMaximalKickDist() )
//     {
//       // if kick and a pass
//       if( soc.commandType == CMD_KICK )
//       {
//         WM->predictBallInfoAfterCommand( soc, &posBall, &velBall );
//         VecPosition posAgent = WM->predictAgentPos( 1, 0 );
//         if( posBall.getDistanceTo( posAgent ) > SS->getMaximalKickDist() + 0.2 )
//           iDiff = 1;
//       }

//       if( iDiff == 0 )
//       {
//         posBall = WM->getGlobalPosition( OBJECT_BALL );
//         velBall.setVecPosition( 0, 0 );
//       }
//     }

// #if USE_DRAW_LOG
//     // LogDraw.logCircle( "ball sending", posBall,
//     //                    1.1, 90, false, COLOR_BLUE );
// #endif
//     myencoder.add( new BallInfo((float) posBall.getX(), (float) posBall.getY(),
//                                 (float) velBall.getX(), (float) velBall.getY(), (unsigned short) (1 - iDiff)) );
//   }

//   // Find closest opponent that was seen this cycle
//   int numT = WM->getNumTakers();
//   ObjectT T[ numT ];
//   int numSeen = 0;
//   for ( int i = 0; i < numT; i++ ) {
//     T[ numSeen ] = SoccerTypes::getOpponentObjectFromIndex( i );
//     if ( WM->getRelativeDistance( T[ numSeen ] ) < SS->getVisibleDistance() &&
//          WM->getTimeLastSeen( T[ numSeen ] ) == WM->getCurrentTime() )
//       numSeen++;  // store this opponent if we just saw him
//   }
//   WM->sortClosestTo( T, numSeen, WM->getAgentObjectType() );

//   if ( numSeen > 0 ) { // add closest
//     VecPosition posOpp = WM->getGlobalPosition( T[ 0 ] );
//     myencoder.add( new OppPos((unsigned short) (SoccerTypes::getIndex(T[ 0 ] ) + 1),
//                               (float) posOpp.getX(), (float) posOpp.getY(), 1 ) );
//   }

//   if ( myencoder.getSize() <= 7 &&  // if there is room
//        WM->getConfidence( WM->getAgentObjectType() ) > PS->getPlayerHighConfThr() ) {
//     myencoder.add( new OurPos((float) posAgentPred.getX(), (float) posAgentPred.getY()) );
//   }

//   strcpy( strMsg, myencoder.getEncodedStr().c_str() );
//   myencoder.clear();
// }

// SoccerCommand KeepawayPlayer::idle(string error)
// {
//   SoccerCommand soc;
//   ACT->putCommandInQueue( soc = turnBodyToObject( OBJECT_BALL ) );
//   ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
//   Log.log(101, "idle (error: %s)", error.c_str());
//   return soc;
// }









KeepawayPlayer::KeepawayPlayer(SMDPAgent* sa, ActHandler *act, WorldModel *wm,
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

  fsm::HierarchicalFSM *player = 0;
  if (WM->getSide() == SIDE_LEFT) {
    player = new fsm::Keeper(this);
  } else {
    player = new fsm::Taker(this);
  }
  fsm::Run(player).operator()();
  delete player;


  

  // shutdown, print hole and number of players seen statistics
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

SoccerCommand KeepawayPlayer::idle(string error)
{
  SoccerCommand soc;
  ACT->putCommandInQueue( soc = turnBodyToObject( OBJECT_BALL ) );
  ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
  Log.log(101, "idle (error: %s)", error.c_str());
  return soc;
}