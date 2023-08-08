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

#ifndef KEEPAWAY_PLAYER
#define KEEPAWAY_PLAYER

#include "BasicPlayer.h"
#include "SMDPAgent.h"

// /*! This class is a superclass from BasicPlayer and contains a more
//     sophisticated decision procedure to determine the next action. */
// class KeepawayPlayer : public BasicPlayer {
// protected:
//   Time m_timeLastSay;           /*!< last time communicated         */

//   // methods associated with saying (defined in KeepawayPlayer.cc)
//   bool shallISaySomething();

//   void makeSayMessage(SoccerCommand soc,
//                       char *str);

// public:
//   KeepawayPlayer(ActHandler *a,
//                  WorldModel *wm,
//                  ServerSettings *ss,
//                  PlayerSettings *cs,
//                  char *strTeamName,
//                  int iNumKeepers,
//                  int iNumTakers,
//                  double dVersion,
//                  int iReconnect = -1);

//   void mainLoop();

//   // behaviors
//   SoccerCommand keeper();

//   SoccerCommand jolKeepers();

//   SoccerCommand idle(std::string error);

//   SoccerCommand jolExecute(int action, int agentIdx, ObjectT K[]);

//   SoccerCommand taker();

// };

// #endif

/*! This class is a superclass from BasicPlayer and contains a more
    sophisticated decision procedure to determine the next action. */

class Memory
{
private:
  Memory();

public:
  static Memory &ins();

  void resetState();

  std::string to_string();

  bool bAlive;
  int agentIdx;
  double state[MAX_RL_STATE_VARS]; // current state -- indexed by K0..Kn
  int ballControlState[11];        // current ball state -- indexed by K0..Kn
  ObjectT teammates[11];           // current mapping from index to teammates
  ObjectT opponents[11];           // current mapping from index to opponents

private:
  std::vector<string> stack; // self call stack

public:
  const vector<string> &getStack() const;

  void PushStack(const string &s);

  void PopStack();

  size_t ballControlHash();
};

class KeepawayPlayer : public BasicPlayer
{
protected:
  bool bContLoop; /*!< is server is alive             */

  Time m_timeLastSay; /*!< last time communicated         */
  Time m_timeStartEpisode;
  SMDPAgent *SA;

  // methods associated with saying (defined in KeepawayPlayer.cc)
  bool shallISaySomething();
  void makeSayMessage(SoccerCommand soc,
                      char *str);

public:
  KeepawayPlayer(SMDPAgent *sa,
                 ActHandler *a,
                 WorldModel *wm,
                 ServerSettings *ss,
                 PlayerSettings *cs,
                 char *strTeamName,
                 int iNumKeepers,
                 int iNumTakers,
                 double dVersion,
                 int numFeatures,
                 int iReconnect = -1);

  // KeepawayPlayer(ActHandler *a,
  //                WorldModel *wm,
  //                ServerSettings *ss,
  //                PlayerSettings *cs,
  //                char *strTeamName,
  //                int iNumKeepers,
  //                int iNumTakers,
  //                double dVersion,
  //                int iReconnect = -1);

  static int num_features;
  static int num_teammates;
  static int num_opponents;

  void mainLoop();

  // behaviors
  SoccerCommand keeper();
  SoccerCommand keeperWithBall();
  SoccerCommand keeperSupport(ObjectT fastest);
  SoccerCommand interpretKeeperAction(int action);

  ObjectT chooseLookObject(double ballThr);

  SoccerCommand taker();

  bool isFastestToBall();
  string getState();
  VecPosition refineTarget(VecPosition target, VecPosition backup);

  // SoccerCommand jolKeepers();
  // SoccerCommand idle(std::string error);
  // SoccerCommand jolExecute(int action, int agentIdx, ObjectT K[]);
};

#endif