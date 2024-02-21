#ifndef _DEAD_ZONE_H
#define _DEAD_ZONE_H

#include "Remote_Control.h"

typedef struct
{
    double ch[5];
    char s[2];
}REMOTE;

double dead_zone_change(double _input, char _set);
REMOTE dead_zone_output(RC_ctrl_t *_rc);


#endif

