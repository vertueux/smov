#include <ctime>
#include <unistd.h>
#include <cstdlib>

#include <smov/base.h>

namespace smov {

void delay(int time) {
  struct timespec ts = {0,0}; 
  ts.tv_sec = time / 1000; 
  ts.tv_nsec = (time % 1000) * 1000000; 
  nanosleep(&ts, NULL);
}

}
