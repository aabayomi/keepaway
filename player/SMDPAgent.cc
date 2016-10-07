#include <limits>

#include "SMDPAgent.h"

using namespace std;

namespace jol {

SMDPAgent::SMDPAgent(int numFeatures, WorldModel *WM) : WM(WM) {
  m_numFeatures = numFeatures;
  lastAction = -1;
  lastActionTime = UnknownTime;
  agentIdx = 0;
}

}
