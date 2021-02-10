#include "main.h"

void autonomous() {
  Pose initialState{0, 0, 0};
  ChassisConfig robotConfig{3, 3, 10};
  PurePursuit p (initialState, robotConfig, 10, 10, 10);
}
