#include "types.h"

const pins_t pins_v0_4_n20 = {
  .revSwitch = 15,
  .triggerSwitch = 32,
  .cycleSwitch = 23,
  .flywheel = 2,
  .pusher = 12,
  .pusherBrake = 13,
  .esc1 = 19,
  .esc2 = 18,
  .esc3 = 5,
  .esc4 = 17,
  .telem = 16,
  .button = 0,
  .batteryADC = 35,
}; 

const pins_t pins_v0_4_noid = {
  .revSwitch = 15,
  .triggerSwitch = 32,
  .pusher = 2,
  .esc1 = 19,
  .esc2 = 18,
  .esc3 = 5,
  .esc4 = 17,
  .telem = 16,
  .button = 0,
  .batteryADC = 35,
};

const pins_t pins_v0_3_n20 = {
  .revSwitch = 15,
  .triggerSwitch = 32,
  .cycleSwitch = 23,
  .flywheel = 2,
  .pusher = 12,
  .pusherBrake = 13,
  .esc1 = 19,
  .esc2 = 18,
  .esc3 = 5,
  .esc4 = 17,
  .telem = 16,
  .button = 0,
  .batteryADC = 33,
};

const pins_t pins_v0_3_noid = {
  .revSwitch = 15,
  .triggerSwitch = 32,
  .pusher = 2,
  .esc1 = 19,
  .esc2 = 18,
  .esc3 = 5,
  .esc4 = 17,
  .telem = 16,
  .button = 0,
  .batteryADC = 33,
};

const pins_t pins_v0_2 = {
  .revSwitch = 15,
  .esc1 = 19,
  .esc2 = 18,
  .esc3 = 5,
  .esc4 = 17,
  .telem = 16,
  .button = 0,
  .batteryADC = 12,
};

const pins_t pins_v0_1 = {
  .revSwitch = 12,
  .esc1 = 4,
  .esc2 = 2,
  .esc3 = 15,
  .esc4 = 13,
};
