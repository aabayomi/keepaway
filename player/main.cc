/*
Copyright (c) 2004, Gregory Kuhlmann, Peter Stone
University of Texas at Austin
All right reserved

Based On:

Copyright (c) 2000-2003, Jelle Kok, University of Amsterdam
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

/*! \file main.cpp
<pre>
<b>File:</b>          main.cpp
<b>Project:</b>       Robocup Soccer Simulation Team: UvA Trilearn
<b>Authors:</b>       Jelle Kok
<b>Created:</b>       28/11/2000
<b>Last Revision:</b> $ID$
<b>Contents:</b>      This file contains the main of the program that is used
               to start the agent. It creates all classes, starts the different
               threads and calls the mainloop. Furthermore it parses the
               specified arguments to adjust the variables.
<hr size=2>
<h2><b>Changes</b></h2>
<b>Date</b>             <b>Author</b>          <b>Comment</b>
28/11/2000       Jelle Kok       Initial version created
</pre>
*/


#include "SenseHandler.h"
#include "ActHandler.h"
#include "KeepawayPlayer.h"
#include "HandCodedAgent.h"
#include "HierarchicalFSM.h"
#include "crossEntropyLearner.h"

#include "Parse.h"
#include "gzstream.h"
#include <dlfcn.h>    // needed for extension loading.

extern Logger Log;     /*!< This is a reference to the normal Logger class   */

void printOptions();

/*! This is the main function and creates and links all the different classes.
    First it reads in all the parameters from the command prompt
    (<program name> -help) and uses these values to create the classes. After
    all the classes are linked, the mainLoop in the Player class is called. */
int main(int argc, char *argv[]) {
  pthread_t sense;
  ServerSettings ss;
  PlayerSettings cs;

  // define variables for command options and initialize with default values
  char strTeamName[MAX_TEAM_NAME_LENGTH] = "UvA_Trilearn";
  int iPort = ss.getPort();
  int iMinLogLevel;
  int iMaxLogLevel;
  char strHost[128] = "127.0.0.1";
  // Changes recommended by celibertojr include changing dVersion to 12. See:
  // https://utlists.utexas.edu/sympa/arc/keepaway/2011-05/msg00006.html
  double dVersion = 12.0;
  int iMode = 0;
  int iNr = 2;
  int iReconnect = -1;
  int iNumKeepers = 3;
  int iNumTakers = 2;
  char strPolicy[128] = "random";
  bool bLearn = false;
  string loadWeightsFile;
  string saveWeightsFile;
  bool bInfo = false;
  bool bSuppliedLogFile = false;
  bool hierarchicalFSM = false;
  bool crossEntropy = false;
  double gamma = 1.0;
  double lambda = 0.0;
  double alpha = 0.125;
  double initialWeight = 0.0;
  bool qLearning = false;
  string loggingFile;

#ifdef _Compress
  ogzstream os;
#else
  ofstream os;
#endif

  // read in all the command options and change the associated variables
  // assume every two values supplied at prompt, form a duo
  char *str;
  for (int i = 1; i < argc; i = i + 2) {
    // help is only option that does not have to have an argument
    if (i + 1 >= argc && strncmp(argv[i], "-help", 3) != 0) {
      cout << "Need argument for option: " << argv[i] << endl;
      exit(0);
    }

    // read a command option
    if (argv[i][0] == '-' && strlen(argv[i]) > 1) {
      switch (argv[i][1]) {
        case '?':                                   // print help
          printOptions();
          exit(0);
          break;
        case 'c':                                   // clientconf file
          if (!cs.readValues(argv[i + 1], ":"))
            cerr << "Error in reading client file: " << argv[i + 1] << endl;
          break;
        case 'd':
          str = &argv[i + 1][0];
          break;
        case 'e': // enable learning 0/1
          str = &argv[i + 1][0];
          bLearn = Parse::parseFirstInt(&str) == 1;
          break;
        case 'f':
          saveWeightsFile = argv[i + 1];
          break;
        case 'g': // gamma
          str = &argv[i + 1][0];
          gamma = Parse::parseFirstDouble(&str);
          break;
        case 'h':                                   // host server or help
          if (strlen(argv[i]) > 2 && argv[i][2] == 'e') {
            printOptions();
            exit(0);
          }
          else
            strcpy(strHost, argv[i + 1]);
          break;
        case 'i':                                   // info 1 0
          str = &argv[i + 1][0];
          bInfo = Parse::parseFirstInt(&str) == 1;
          break;
        case 'I': // initialWeight
          str = &argv[i + 1][0];
          initialWeight = Parse::parseFirstDouble(&str);
          break;
        case 'j':
          str = &argv[i + 1][0];
          iNumTakers = Parse::parseFirstInt(&str);
          break;
        case 'k':
          str = &argv[i + 1][0];
          iNumKeepers = Parse::parseFirstInt(&str);
          break;
        case 'l':                                   // loglevel int[..int]
          str = &argv[i + 1][0];
          iMinLogLevel = Parse::parseFirstInt(&str);
          while (iMinLogLevel != 0) {
            if (*str == '.' || *str == '-') // '.' or '-' indicates range
            {
              *str += 1;
              iMaxLogLevel = Parse::parseFirstInt(&str);
              if (iMaxLogLevel == 0) iMaxLogLevel = iMinLogLevel;
              Log.addLogRange(iMinLogLevel, iMaxLogLevel);
            }
            else
              Log.addLogLevel(iMinLogLevel);
            iMinLogLevel = Parse::parseFirstInt(&str);
          }
          break;
        case 'L': // lambda
          str = &argv[i + 1][0];
          lambda = Parse::parseFirstDouble(&str);
          break;
        case 'A': // alpha 
          str = &argv[i + 1][0];
          alpha = Parse::parseFirstDouble(&str);
          break;
        case 'm':                                   // mode int
          str = &argv[i + 1][0];
          iMode = Parse::parseFirstInt(&str);
          break;
        case 'n':                                   // number in formation int
          str = &argv[i + 1][0];
          iNr = Parse::parseFirstInt(&str);
          break;
        case 'o':                                   // output file log info
# ifdef _Compress
          //os.open((string(argv[i + 1]) + ".gz").c_str());
          os.open((string(argv[i + 1])).c_str());
          loggingFile = string(argv[i+1]);

#else
          os.open(argv[i + 1]);
#endif
          bSuppliedLogFile = true;
          break;
        case 'p':                                   // port
          str = &argv[i + 1][0];
          iPort = Parse::parseFirstInt(&str);
          break;
        case 'q':
          strcpy(strPolicy, argv[i + 1]);
          break;
        case 'Q': // enable qlearning 0/1
          str = &argv[i + 1][0];
          qLearning = Parse::parseFirstInt(&str) == 1;
          break;
        case 'r':                                   // reconnect 1 0
          str = &argv[i + 1][0];
          iReconnect = Parse::parseFirstInt(&str);
          break;
        case 's':                                   // serverconf file
          if (!ss.readValues(argv[i + 1], ":"))
            cerr << "Error in reading server file: " << argv[i + 1] << endl;
          break;
        case 't':                                   // teamname name
          strcpy(strTeamName, argv[i + 1]);
          break;
        case 'v':                                   // version version
          str = &argv[i + 1][0];
          dVersion = Parse::parseFirstDouble(&str);
          break;
        case 'w':
          loadWeightsFile = argv[i + 1];
          break;
        case 'z':
          str = &argv[i + 1][0];
          hierarchicalFSM = Parse::parseFirstInt(&str) == 1;
          break;
        case 'E':
          str = &argv[i + 1][0];
          crossEntropy = Parse::parseFirstInt(&str) == 1;
          break;
        default:
          cerr << "(main) Unknown command option: " << argv[i] << endl;
      }
    }
  }

  if (bInfo) {
    cout << "team : " << strTeamName << endl <<
         "port : " << iPort << endl <<
         "host : " << strHost << endl <<
         "version : " << dVersion << endl <<
         "mode : " << iMode << endl <<
         "playernr : " << iNr << endl <<
         "reconnect : " << iReconnect << endl <<
         "hierarchical FSM : " << hierarchicalFSM << endl <<
         "cross entropy method : " << crossEntropy << endl <<
         "gamma : " << gamma << endl <<
         "lambda : " << lambda << endl <<
         "alpha : " << alpha << endl <<
         "initialWeight: " << initialWeight << endl <<
         "qlearning: " << qLearning << endl <<
         "be learning : " << bLearn << endl;
  }

  if (bSuppliedLogFile){
    Log.setOutputStream(os);                   // initialize logger
    Log.setOutputStream(loggingFile);
  }
  else{
    Log.setOutputStream(cout);
    Log.setOutputStream(loggingFile);
  }

  Log.restartTimer();

  WorldModel wm(&ss, &cs, NULL);              // create worldmodel
  Connection c(strHost, iPort, MAX_MSG);     // make connection with server
  ActHandler a(&c, &wm, &ss);                // link actHandler and worldmodel
  SenseHandler s(&c, &wm, &ss, &cs);         // link senseHandler with wm
  Log.log("It is working");
  
  SMDPAgent *sa = NULL;

  double ranges[MAX_RL_STATE_VARS];
  double minValues[MAX_RL_STATE_VARS];
  double resolutions[MAX_RL_STATE_VARS];

  int numFeatures = 0;
  if (string(strTeamName) == "keepers") {
    numFeatures = wm.keeperStateRangesAndResolutions(ranges, minValues, resolutions,
                                                     iNumKeepers, iNumTakers);
  }
  else {
    numFeatures = wm.takerStateRangesAndResolutions(ranges, minValues, resolutions,
                                                    iNumKeepers, iNumTakers);
  }

   int numActions = iNumKeepers;

  
  // cout << "crossEntropy nnnnnnnn"<< crossEntropy << endl;


 if (!crossEntropy) {
    cerr << "No agent!" << endl;
    return EXIT_FAILURE;
  } else {
    if (string(strTeamName) == "keepers") {
      Log.log("Keepers here");
      CrossEntropyAgent *variable = new CrossEntropyAgent(numFeatures, numActions, bLearn, resolutions,loadWeightsFile, saveWeightsFile);

      sa = variable;


      // fsm::HierarchicalFSM::initialize(
      //     numFeatures, iNumKeepers, iNumTakers, bLearn,
      //     resolutions, gamma, lambda, alpha, initialWeight, qLearning,
      //     loadWeightsFile, saveWeightsFile, strTeamName);
    }
    else {

        

        CrossEntropyAgent *variable = new CrossEntropyAgent(numFeatures, numActions, bLearn, resolutions,loadWeightsFile, saveWeightsFile);

        sa = variable;


      //  CrossEntropyAgent* CrossEntropyAgent = new CrossEntropyAgent(
      //   numFeatures, numActions, bLearn, resolutions,
      //   loadWeightsFile, saveWeightsFile);

      //   sa = CrossEntropyAgent;
    };
  }


  




  // if (!hierarchicalFSM) {
  //   cerr << "No agent!" << endl;
  //   return EXIT_FAILURE;
  // } else {
  //   if (string(strTeamName) == "keepers") {
  //     fsm::HierarchicalFSM::initialize(
  //         numFeatures, iNumKeepers, iNumTakers, bLearn,
  //         resolutions, gamma, lambda, alpha, initialWeight, qLearning,
  //         loadWeightsFile, saveWeightsFile, strTeamName);
  //   }
  //   else {
  //     fsm::HierarchicalFSM::initialize(
  //         numFeatures, iNumTakers, iNumKeepers, bLearn,
  //         resolutions, gamma, lambda, alpha, initialWeight, qLearning,
  //         loadWeightsFile, saveWeightsFile, strTeamName);
  //   };
  // }

  KeepawayPlayer bp( sa, &a, &wm, &ss, &cs, strTeamName,
                    iNumKeepers, iNumTakers, dVersion, iReconnect);

  pthread_create(&sense, NULL, sense_callback, &s); // start listening

  if (iMode == 0)
    bp.mainLoop();

  c.disconnect();
  os.close();
}

/*! This function prints the command prompt options that can be supplied to the
    program. */
void printOptions() {
  cout << "Command options:" << endl <<
       " a file                - write drawing log info to " << endl <<
       " c(lientconf) file     - use file as client conf file" << endl <<
       " d(rawloglevel) int[..int] - level(s) of drawing debug info" << endl <<
       " e(nable) learning 0/1  - turn learning on/off" << endl <<
       " f save weights file   - use file to save weights" << endl <<
       " h(ost) hostname       - host to connect with" << endl <<
       " he(lp)                - print this information" << endl <<
       " hi(ve) 0/1            - use mmap to hive mind the team" << endl <<
       " i(nfo) 0/1            - print variables used to start" << endl <<
       " j takers  int         - number of takers" << endl <<
       " k(eepers) int         - number of keepers" << endl <<
       " l(oglevel) int[..int] - level of debug info" << endl <<
       " m(ode) int            - which mode to start up with" << endl <<
       " n(umber) int          - player number in formation" << endl <<
       " o(utput) file         - write log info to (screen is default)" << endl <<
       " p(ort)                - port number to connect with" << endl <<
       " q policy name         - policy to play with" << endl <<
       " r(econnect) int       - reconnect as player nr" << endl <<
       " s(erverconf) file     - use file as server conf file" << endl <<
       " t(eamname) name       - name of your team" << endl <<
       " w(eights) file        - use file to load weights" << endl <<
       " x exit after running for this many episodes" << endl <<
       " y enable learning after not learning for this many episodes" << endl;
}
