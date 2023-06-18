#include "types.h"

const pins_t pins_v0_5 = {
    .pusherDriverType = HBRIDGE_DRIVER,
    .revSwitch = 15,
    .triggerSwitch = 33,
    .cycleSwitch = 25,
    .flywheel = 2,
    .pusher1L = 14,
    .pusher2L = 27,
    .pusher1H = 13,
    .pusher2H = 12,
    .esc1 = 19,
    .esc2 = 18,
    .esc3 = 5,
    .esc4 = 17,
    .telem = 16,
    .button = 0,
    .batteryADC = 35,
};

const pins_t pins_v0_4_n20 = {
    .pusherDriverType = AT8870_DRIVER,
    .revSwitch = 15,
    .triggerSwitch = 32,
    .cycleSwitch = 23,
    .flywheel = 2,
    .pusher1L = 12,
    .pusher2L = 13,
    .esc1 = 19,
    .esc2 = 18,
    .esc3 = 5,
    .esc4 = 17,
    .telem = 16,
    .button = 0,
    .batteryADC = 35,
};

const pins_t pins_v0_4_noid = {
    .pusherDriverType = FET_DRIVER,
    .revSwitch = 15,
    .triggerSwitch = 32,
    .pusher1H = 2,
    .esc1 = 19,
    .esc2 = 18,
    .esc3 = 5,
    .esc4 = 17,
    .telem = 16,
    .button = 0,
    .batteryADC = 35,
};

const pins_t pins_v0_3_n20 = {
    .pusherDriverType = AT8870_DRIVER,
    .revSwitch = 15,
    .triggerSwitch = 32,
    .cycleSwitch = 23,
    .flywheel = 2,
    .pusher1L = 12,
    .pusher2L = 13,
    .esc1 = 19,
    .esc2 = 18,
    .esc3 = 5,
    .esc4 = 17,
    .telem = 16,
    .button = 0,
    .batteryADC = 33,
};

const pins_t pins_v0_3_noid = {
    .pusherDriverType = FET_DRIVER,
    .revSwitch = 15,
    .triggerSwitch = 32,
    .pusher1H = 2,
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
