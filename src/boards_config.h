#include "types.h"

const boards_t board_v0_7 = {
    .pusherDriverType = DRV_DRIVER,
    .pusherCoastHigh = true,
    .nSleep = 27,
    .flywheel = 2,
    .pusher1L = 12,
    .pusher2L = 13,
    .esc1 = 17,
    .esc2 = 5,
    .esc3 = 18,
    .esc4 = 19,
    .telem = 16,
    .button = 0,
    .batteryADC = 35,
    .pusherShunt = 14,
};

const boards_t board_v0_8 = board_v0_7;
const boards_t board_v0_9 = board_v0_7;
const boards_t board_v0_10 = board_v0_7;
const boards_t board_v0_11 = board_v0_7;

const boards_t board_v0_5 = {
    .pusherDriverType = HBRIDGE_DRIVER,
    .flywheel = 2,
    .pusher1L = 14,
    .pusher2L = 27,
    .pusher1H = 13,
    .pusher2H = 12,
    .esc1 = 17,
    .esc2 = 5,
    .esc3 = 18,
    .esc4 = 19,
    .telem = 16,
    .button = 0,
    .batteryADC = 35,
};

const boards_t board_v0_6 = board_v0_5;

const boards_t board_v0_4_n20 = {
    .pusherDriverType = DRV_DRIVER,
    .pusherCoastHigh = false,
    .flywheel = 2,
    .pusher1L = 12,
    .pusher2L = 13,
    .esc1 = 17,
    .esc2 = 5,
    .esc3 = 18,
    .esc4 = 19,
    .telem = 16,
    .button = 0,
    .batteryADC = 35,
};

const boards_t board_v0_4_noid = {
    .pusherDriverType = FET_DRIVER,
    .pusher1H = 2,
    .esc1 = 17,
    .esc2 = 5,
    .esc3 = 18,
    .esc4 = 19,
    .telem = 16,
    .button = 0,
    .batteryADC = 35,
};

const boards_t board_v0_3_n20 = {
    .pusherDriverType = DRV_DRIVER,
    .pusherCoastHigh = false,
    .flywheel = 2,
    .pusher1L = 12,
    .pusher2L = 13,
    .esc1 = 17,
    .esc2 = 5,
    .esc3 = 18,
    .esc4 = 19,
    .telem = 16,
    .button = 0,
    .batteryADC = 33,
};

const boards_t board_v0_3_noid = {
    .pusherDriverType = FET_DRIVER,
    .pusher1H = 2,
    .esc1 = 17,
    .esc2 = 5,
    .esc3 = 18,
    .esc4 = 19,
    .telem = 16,
    .button = 0,
    .batteryADC = 33,
};

const boards_t board_v0_2 = {
    .esc1 = 17,
    .esc2 = 5,
    .esc3 = 18,
    .esc4 = 19,
    .telem = 16,
    .button = 0,
    .batteryADC = 12,
};

const boards_t board_v0_1 = {
    .esc1 = 4,
    .esc2 = 2,
    .esc3 = 15,
    .esc4 = 13,
};
