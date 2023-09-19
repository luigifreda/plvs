#pragma once
#ifndef __UTILS_HPP__
#define __UTILS_HPP__

#include <chrono>
#include <iostream>

#define SET_CLOCK(t0) \
        std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

#define TIME_DIFF(t1, t0) \
        (std::chrono::duration_cast<std::chrono::duration<double>>((t1) - (t0)).count())

#define PRINT_CLOCK(msg, t1, t0) \
        std::cerr << msg << TIME_DIFF(t1, t0) << endl;

#ifdef USE_NVTX
#include "nvToolsExt.h"

static const uint32_t colors[] = { 0x0000ff00, 0x000000ff, 0x00ffff00, 0x00ff00ff, 0x0000ffff, 0x00ff0000, 0x00ffffff };
static const int num_colors = sizeof(colors)/sizeof(uint32_t);

#define PUSH_RANGE(name,cid) { \
      int color_id = cid; \
      color_id = color_id%num_colors;\
      nvtxEventAttributes_t eventAttrib = {0}; \
      eventAttrib.version = NVTX_VERSION; \
      eventAttrib.size = NVTX_EVENT_ATTRIB_STRUCT_SIZE; \
      eventAttrib.colorType = NVTX_COLOR_ARGB; \
      eventAttrib.color = colors[color_id]; \
      eventAttrib.messageType = NVTX_MESSAGE_TYPE_ASCII; \
      eventAttrib.message.ascii = name; \
      nvtxRangePushEx(&eventAttrib); \
}
#define POP_RANGE nvtxRangePop();
#else
#define PUSH_RANGE(name,cid)
#define POP_RANGE
#endif

#endif
