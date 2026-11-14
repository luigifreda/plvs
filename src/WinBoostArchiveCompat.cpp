/**
 * This file is part of PLVS.
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 *
 * Windows/Clang boost compatibility shim.
 *
 * The boost.serialization binaries shipped by vcpkg are built with MSVC, which
 * never emits the separate "virtual base destructor" (mangled ??_D...) that
 * Clang's MSVC-ABI code generator requests when it destroys a boost archive
 * exception (these classes virtually derive from std::exception). The boost DLL
 * thus exports the regular destructor (??1...) but not the vbase variant, so
 * Clang's reference to it -- and the __imp_ entry the dllimport callers use --
 * is unresolved at link time.
 *
 * Provide the vbase destructor here (forwarding to the regular destructor the
 * DLL does export) and publish the matching __imp_ pointer the dllimport callers
 * dereference. No-op on non-Windows.
 *
 * \author Jeff Daily <jeff.daily@amd.com>
 */

#if defined(_WIN32)
#include <boost/archive/archive_exception.hpp>

// The vbase destructor Clang expects, with external linkage so every translation
// unit's reference resolves. Its job is to run the object's destructor, which the
// boost DLL exports, so forward to it.
void plvs_archive_exception_vbase_dtor(boost::archive::archive_exception* self)
    asm("??_Darchive_exception@archive@boost@@QEAAXXZ");
void plvs_archive_exception_vbase_dtor(boost::archive::archive_exception* self)
{
    self->~archive_exception();
}

// dllimport callers read the function address from __imp_<symbol>; define that
// pointer to point at the definition above.
extern "C" void (*const plvs_imp_vbase_dtor)(boost::archive::archive_exception*)
    asm("__imp_??_Darchive_exception@archive@boost@@QEAAXXZ");
extern "C" void (*const plvs_imp_vbase_dtor)(boost::archive::archive_exception*)
    = &plvs_archive_exception_vbase_dtor;
#endif // _WIN32
