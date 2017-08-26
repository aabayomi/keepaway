# RoboCup Keepaway benchmark player framework

## Concurrent HAMQ Learning

This is an ongoing project on concurrent reinforcement learning following the HAM framework. More details can be found on following papers:
-   **Efficient Reinforcement Learning with Hierarchies of Machines by Leveraging Internal Transitions**, *Aijun Bai*, and Stuart Russell,\
    *Proceedings of the Twenty-Fifth International Joint Conference on Artificial Intelligence (**IJCAI**), Melbourne, Australia, August 19 - 25, 2017.*
    \[[pdf](https://aijunbai.github.io/publications/IJCAI17-Bai.pdf)\]
-   **Concurrent Hierarchical Reinforcement Learning for RoboCup Keepaway**, *Aijun
    Bai*, Stuart Russell, and Xiaoping Chen,\
    *RoboCup-2017: Robot Soccer World Cup XX, Lecture Notes in
    Artificial Intelligence (**RoboCup**), Springer Verlag, Berlin, 2017.*
    \[[pdf](https://aijunbai.github.io/publications/LNAI17-Bai.pdf)\]
-   **Speeding Up HAM Learning with Internal Transitions**, *Aijun Bai*, and Stuart Russell,\
    *The Multi-disciplinary Conference on Reinforcement Learning and Decision Making (**RLDM**)
    2017, Ann Arbor, Michigan, USA, June 11-14, 2017.*
    \[[pdf](https://aijunbai.github.io/publications/RLDM17-Bai.pdf)\]


### Initial policy
[![HAMQ-INT on RoboCup Keepaway (initial policy)](gifs/keepaway_initialized.gif)](https://www.youtube.com/watch?v=GFd0b3wV0Rk "initial policy")

### Converged policy
[![HAMQ-INT on RoboCup Keepaway (converged policy)](gifs/keepaway_converged.gif)](https://www.youtube.com/watch?v=ob249J8Kxi0 "converged policy")

## Full Hive Mind Learning

Full-hive-mind learning extends the original work of Stone and et. al. 
In a scenario where A at t1 passes to B, B at t2 passes to A, A at t3 passes to C,
C at t4 passes to B and B at t5 recievs the ball, the original single agent learning 
or hive mind learning have only 2 updates: t1-t3 for A and it2-t5 for B; the new 
full-hive-mind algorithm has 4 updates: t1-t2 for B, t2-t3 for A, t3-t4 for C 
and t4-t5 for B. Weights as well as traces are shared for this purpose. Global 
optimality can be ensured given fixed opponents.

- Aijun Bai

## Usages

- Train a group of keepaway players using a linear SARSA algorithm: ```./train.sh [-h HIVE_MODE] [-g GAMMA] [-l] [-f] [-m] [-s]```
 - ```-h HIVE_MODE```: set hive mind mode for ```keepaway_player```: 0 - single agent learning; 1 - single agent learning with shared weights; 2 - multi-agent distributed learning with shared weights and traces
 - ```-g GAMMA```: specify the discount factor used when learning
 - ```-l```: turn on logging for ```keepaway_player``` and ``rcssserver```
 - ```-f```: turn on fullstate perception within ```rcssserver```
 - ```-m```: launch a monitor, technically ```rcsssmonitor```
 - ```-s```: turn on synch mode within ```rcssserver```

- Evaluate a specified policy by simulation:```./evaluate.sh [-p POLICY] [-q QFILE] [-l] [-f] [-m] [-s]```
 - ```-p POLICY```: specify the policy to evaluate, for example: hand, hold, random or learn
 - ```-q QFILE```: speficy the learned Q function for a learned policy
  - ```-l```: turn on logging for ```keepaway_player``` and ``rcssserver```
 - ```-f```: turn on fullstate perception within ```rcssserver```
 - ```-m```: launch a monitor, technically ```rcsssmonitor```
 - ```-s```: turn on synch mode within ```rcssserver```

# Old READMEs

This project is intended as a public but unofficial repository for updates to
the Keepaway benchmark player framework created at UT Austin by Gregory
Kuhlmann and Peter Stone.

The intention is that this project can serve as a source for updates and
compatibility with ongoing changes to the RoboCup Soccer Server. However, no
promises are made as to the fitness of this software for any purpose
whatsoever.

I have also modified the launch scripts to expect working with in-place builds
of rcss tools rather than system installed ones. The latter might still work,
but I haven't checked on that. I expect sibling dirs to keepaway called
rcssmonitor_qt4 and rcssserver. Use the keepaway.py launch script for most
convenience. I haven't done much to maintain the original keepaway.sh. As an
example, to kick off with default settings and see the action, run like so (if
your system has python and knows shebangs):

```
./keepaway.py --monitor
```

The original README from Gregory Kuhlmann and Peter Stone follows. Following
that is the original README for the learning code, including the list of
contributors to that code base. See these READMEs also for papers to reference
when using this code.

Also included in this combined code base is the Tile Coding Software version 2
from Richard S. Sutton, included with permission from him. This software, as
well as more recent versions, is also available directly from the author
(http://webdocs.cs.ualberta.ca/~sutton/tiles2.html).

The combined, updated code base has also included assistance from Stephen Kelly
(initial integration of learning code) and Luiz Celiberto (my updates in support
of latest rcss and gcc checked against his), among others.

I have also made some minimal changes to support dynamic loading of shared
library code for agent behavior, thus avoiding the need to change the original
code. See the "ext=" hackery in main.cc for more details.

- Tom Palmer

-------------------
README for Keepaway
-------------------

This package contains the source code for the Keepaway benchmark 
player framework and the source code for some keepaway utility
programs.  The framework includes all low- and mid-level keepaway
behaviors.  A few example high-level policies are included.  Not
included is any learning code.  However, the framework was designed
to make it easy to insert your own learning code.  Also, not included
is the soccer simulator (see REQUIREMENTS).

The framework was created by

 Gregory Kuhlmann
 and Peter Stone

in the Department of Computer Sciences
at the University of Texas at Austin:
http://www.cs.utexas.edu/~AustinVilla/sim/keepaway/

Copyright 2004 Gregory Kuhlmann, Peter Stone
All Rights Reserved

This package was created as part of our research using
the keepaway domain.  The following is a list of selected 
publications:

Keepaway Soccer: From Machine Learning Testbed to Benchmark.
Peter Stone, Gregory Kuhlmann, Matthew E. Taylor, and Yaxin Liu.
In Itsuki Noda, Adam Jacoff, Ansgar Bredenfeld, and Yasutake Takahashi, editors, 
RoboCup-2005: Robot Soccer World Cup IX, Springer Verlag, Berlin, 2006.

Reinforcement Learning for RoboCup-Soccer Keepaway.
Peter Stone, Richard S. Sutton, and Gregory Kuhlmann.
Adaptive Behavior, 2005. 

These and other keepaway papers can be found here:
http://www.cs.utexas.edu/~pstone/Papers/bib2html/

We ask that you please cite these papers if you publish work that builds
off of this framework.

The players are built upon version 3.3 of the UvA Basic team that is 
publicly available from:
http://staff.science.uva.nl/~jellekok/robocup/2003/index_en.html

The communication code is based on:
saymsglib - a library to generate and parse messages in the Coachable agent
communication standard created by Carnegie Mellon University.  It is 
described here:
http://www-2.cs.cmu.edu/~robosoccer/simulator/comm_standard.html

We provide this code as-is with no support implied.
It has been tested under Debian Linux 3.1 (testing)

REQUIREMENTS:

The RoboCup Soccer Server, available from:
http://sserver.sf.net

The following package versions are strongly recommended:
  rcssbase    9.4.5
  rcssserver  9.4.5
  rcssmonitor 9.3.7

Gnuplot (optional)


FILES and DIRECTORIES:

```
README        - This file
COPYING       - GNU Public License
keepaway.sh   - script to start players
player/       - keepaway player source code
logs/         - dir to store generated log files (initially empty)
weights/      - dir to store learned weights (initially empty)
tools/        - some useful utilities for keepaway experiments
```


INSTALLATION:

In player/:
```
make depend
make
```

In tools/:
```
make
```
(copy all binaries to some directory in your PATH)


USAGE:

(Modify "keepaway_dir" in keepaway.sh to match installation dir)
```
./keepaway.sh
```

To view players:
```
monitor
```

To stop:
```
killserver
```


---------------------------------
README for Keepaway learning code
---------------------------------

This package contains the learning code to accompany the Keepaway
benchmark player framework.

This is a particular snapshot of the code, and different published
results have used different learning algorithms.  We don't guarantee
that this particular instantiation will line up with all of the
published papers using keepaway.  Variations on the learning algorithm
are described in the papers indexed here:
http://www.cs.utexas.edu/~pstone/Papers/bib2html/

We provide this code as-is with no support implied. This code has not
been tested and we make no guarantees that it will even compile.

The learning code was created by

 Peter Stone
 Gregory Kuhlmann
 Matthew E. Taylor
 Yaxin Liu
 and Shivaram Kalyanakrishnan

in the Department of Computer Sciences
at the University of Texas at Austin:
http://www.cs.utexas.edu/~AustinVilla/sim/keepaway/

This package was created as part of our research using
the keepaway domain.  The following is a list of selected 
publications:

Keepaway Soccer: From Machine Learning Testbed to Benchmark.
Peter Stone, Gregory Kuhlmann, Matthew E. Taylor, and Yaxin Liu.
In Itsuki Noda, Adam Jacoff, Ansgar Bredenfeld, and Yasutake Takahashi, editors,
RoboCup-2005: Robot Soccer World Cup IX, Springer Verlag, Berlin, 2006. To
appear.

Reinforcement Learning for RoboCup-Soccer Keepaway.
Peter Stone, Richard S. Sutton, and Gregory Kuhlmann.
Adaptive Behavior, 2005. 

We ask that you please cite these papers if you publish work that builds
off of this code.

The players are built upon version 3.3 of the UvA Basic team that is 
publicly available from:
http://staff.science.uva.nl/~jellekok/robocup/2003/index_en.html

The communication code is based on:
saymsglib - a library to generate and parse messages in the Coachable agent
communication standard created by Carnegie Mellon University.  It is 
described here:
http://www-2.cs.cmu.edu/~robosoccer/simulator/comm_standard.html

REQUIREMENTS:

The Keepaway Player Framework
http://www.cs.utexas.edu/~AustinVilla/sim/keepaway/

The RoboCup Soccer Server, available from:
http://sserver.sf.net
