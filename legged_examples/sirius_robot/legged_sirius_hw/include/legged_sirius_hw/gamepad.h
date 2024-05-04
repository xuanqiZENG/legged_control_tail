/* define ----------------------------------------------------------------*/
#ifndef Logitech_DRIVER
#define Logitech_DRIVER

/* Includes ---------------------------------------------------------------*/
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/input.h>
#include <string.h>
#include <stdio.h>
#include <map>

#define JSKEY_A     0x001
#define JSKEY_B     0x101
#define JSKEY_X     0x201
#define JSKEY_Y     0x301

#define JSKEY_LB    0x401
#define JSKEY_RB    0x501
#define JSKEY_BACK  0x601
#define JSKEY_START 0x701
#define JSKEY_HOME  0x801

#define JSKEY_LT 0x202
#define JSKEY_RT 0x502

#define JSKEY_CROSS_X 0x602
#define JSKEY_CROSS_Y 0x702
#define JSKEY_LEFTSTICK_X 0x002
#define JSKEY_LEFTSTICK_Y 0x102
#define JSKEY_RIGHTSTICK_X 0x302
#define JSKEY_RIGHTSTICK_Y 0x402

#define JSKEY_PRESS 0x001    
#define JSKEY_RELEASE 0x0    

#define JSKEY_CROSS_LOW_VALUE 0xffff8001 
#define JSKEY_CROSS_HIGH_VALUE 0x7fff   

using namespace std;

class Logitech{
private:
    char *dev;
    ssize_t n;
    int fd;
    int buf[2];
public:
    Logitech(char* device);
    map<int, int> Keystate_map;
    int  init();
    void listen_input();
    void print_key_state();
    float x;
    float y;
    float z;
    float alpha;
    float belta;
    float gama;
    int enable;
    int start;
};
#endif
