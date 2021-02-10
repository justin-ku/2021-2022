#include "main.h"

Controller master(CONTROLLER_MASTER);

Motor LF(9, E_MOTOR_GEARSET_06, true);
Motor LB(2, E_MOTOR_GEARSET_06, true);
Motor RF(10, E_MOTOR_GEARSET_06, false);
Motor RB(1, E_MOTOR_GEARSET_06, false);

Motor intakeLeft(19, E_MOTOR_GEARSET_06, true);
Motor intakeRight(18, E_MOTOR_GEARSET_06, false);

Motor rollerTop(11, E_MOTOR_GEARSET_06, true);
Motor rollerBottom(12, E_MOTOR_GEARSET_06, false);

ADIEncoder leftE('G', 'H', false);
ADIEncoder rightE('E', 'F', false);
Imu imu(8);

Distance topDist(16);
Distance midDist(15);
Distance bottomDist(14);
Optical optical(17);
