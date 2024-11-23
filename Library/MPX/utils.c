#include "utils.h"


void clearmemory(unsigned char*in,unsigned int sz){
  while(sz--){
    *in=0;
    in++;
  }
}

void copymemory(unsigned char*to,unsigned char *from,unsigned int sz){
  while(sz--){
    *to = *from;
    to++;
    from++;
  }
}