#include <limits>

#include "SMDPAgent.h"
#include <linux/limits.h>

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

SMDPAgent::SMDPAgent(int numFeatures) {
  m_numFeatures = numFeatures;
  lastAction = -1;
  lastActionTime = UnknownTime;
  hiveMind = 0;

  string exepath = getexepath();
  replace(exepath.begin(), exepath.end(), '/', '_');

  fileLockPrefix = "/run/lock/" + exepath + "_";
  sharedMemoryPrefix = "/run/shm/" + exepath + "_";
}
