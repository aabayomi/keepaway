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

#ifndef SMDP_AGENT
#define SMDP_AGENT

#include <utility>
#include <vector>
#include <zconf.h>
#include <cassert>
#include <cstdlib>
#include <sstream>
#include <unordered_set>
#include <unordered_map>
#include <map>
#include "SoccerTypes.h"
#include "Logger.h"
#include "JointActionSpace.h"

class SMDPAgent
{
public:
  int m_numFeatures; /* number of state features <= MAX_STATE_VARS */
  int lastAction;
  int lastActionTime;
  ObjectT K[11]; // mapping from index to player obj
  int hiveMind; // 1: hive mind, 2: full hive mind
  string fileLockPrefix;
  string sharedMemoryPrefix;

  int getNumFeatures() const { return m_numFeatures; }
  int getNumActions()  const { return JointActionSpace::ins().numActions();  }

public:
  SMDPAgent(int numFeatures);
  virtual ~SMDPAgent() {}

  // abstract methods to be supplied by implementing class
  virtual void sync(bool load) = 0;
  virtual int  startEpisode( int current_time, double state[] ) = 0;
  virtual int  step( int current_time, double reward, double state[] ) = 0;
  virtual void endEpisode( int current_time, double reward ) = 0;

  // Optional customization point.
  virtual void shutDown() {}
} ;

#endif
