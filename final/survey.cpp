#include "survey.h"

Survey::Survey(char flame_port, char turret_motor, char turret_pot, float p,
               float i, float d)
    : Loop(1e4 /*100Hz*/), turret_(turret_motor, turret_pot, p, i, d) {}
