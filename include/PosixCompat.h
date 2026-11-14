/**
 * This file is part of PLVS.
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 *
 * Portable replacement for the POSIX <unistd.h> facilities the project uses
 * (only usleep). On POSIX platforms it simply includes <unistd.h>; on Windows,
 * where <unistd.h> does not exist, it provides usleep via the C++ standard
 * library so the rest of the code base is unchanged.
 *
 * \author Jeff Daily <jeff.daily@amd.com>
 */

#ifndef PLVS_POSIX_COMPAT_H
#define PLVS_POSIX_COMPAT_H

#if defined(_WIN32)

#include <chrono>
#include <thread>

inline void usleep(unsigned long usec)
{
    std::this_thread::sleep_for(std::chrono::microseconds(usec));
}

#else

#include <unistd.h>

#endif

#endif // PLVS_POSIX_COMPAT_H
