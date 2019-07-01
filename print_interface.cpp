#include <arpa/inet.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <limits.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <cerrno>
#include <ctime>
#include <iomanip>

// pauses caller
bool customSleep(int sec, int nsec)
{
  struct timespec req, rem;
  req.tv_sec = sec;
  req.tv_nsec = nsec;
  if (nanosleep(&req, &rem) != 0)
  {
    printf("\x1b[31m rem %d:%d strerror:%s \x1b[0m\n", (int)rem.tv_sec, (int)rem.tv_nsec, strerror(errno));
    return false;
  }
  return true;
}

int main(void)
{
  // unsigned int max = -1;
  // printf("UINT_MAX = %u = 0x%x\n", UINT_MAX, UINT_MAX);
  // printf("max      = %u = 0x%x\n", max, max);
  // printf("int32max      = %d\n", INT_MAX);
  // printf("long max      = %ld\n", LONG_MAX);
  // customSleep(3, 3);
  int a = 1;
  uint b = a;
  printf("%d %u\n", a, b);

  return 0;
}
