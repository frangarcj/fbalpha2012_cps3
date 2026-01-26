#ifndef PICO_INT_H
#define PICO_INT_H

#include <stdio.h>
#include <string.h> // memset
#include "pico_types.h"
#include "pico_port.h"

// Minimal stub for PicoDrive internal definitions required by SH2 DRC

#define PICO_INTERNAL

// Log levels
#define EL_STATUS (1 << 0)
#define EL_ANOMALY (1 << 1)
#define EL_32X (1 << 2)

// Redirect elprintf to printf or FBA log
#ifdef __cplusplus
extern "C"
{
#endif

// Simple logging wrapper
#if defined(VITA)
#include <psp2/kernel/clib.h>
#define elprintf(w, f, ...) sceClibPrintf("SH2DRC: " f "\n", ##__VA_ARGS__)
#else
#define elprintf(w, f, ...) printf("SH2DRC: " f "\n", ##__VA_ARGS__)
#endif

#define elprintf_sh2(sh2, w, f, ...) elprintf(w, "sh2: " f, ##__VA_ARGS__)

// SH2 DRC Constants
#define SH2_READ_SHIFT 25
#define SH2_WRITE_SHIFT 25
#define SH2_DRCBLK_RAM_SHIFT 1
#define SH2_DRCBLK_DA_SHIFT 1
#define P32XF_DRC_ROM_C (1 << 8)

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

    // Stub Structures
    typedef struct
    {
        unsigned int emu_flags;
    } Pico32x_t;

    typedef struct
    {
        unsigned char *drcblk_da[2];
        unsigned char *drcblk_ram;
        unsigned char *drclit_ram;
        unsigned char *drclit_da[2];
    } Pico32xMem_t;

    typedef struct
    {
        int opt;
    } PicoIn_t;
#define POPT_EN_DRC 1

    // Externs (defined in sh2_drc_fba.inl)
    extern Pico32x_t Pico32x;
    extern Pico32xMem_t *Pico32xMem;
    extern PicoIn_t PicoIn;

    // Helper functions (dummies)
    void cache_flush_d_inval_i(void *start, void *end);
    int plat_mem_set_exec(void *ptr, size_t size);
    void *plat_mem_get_for_drc(size_t size);
    void memset32(void *dest, int val, int count);

    struct SH2_DRC_;
    typedef struct SH2_DRC_ SH2_DRC;
    // extern SH2_DRC sh2s[2]; // Moved to picodrive_sh2.h

    // Missing Prototypes
    u32 REGPARM(3) p32x_sh2_poll_memory8(u32 a, u32 d, SH2_DRC *sh2);
    u32 REGPARM(3) p32x_sh2_poll_memory16(u32 a, u32 d, SH2_DRC *sh2);
    u32 REGPARM(3) p32x_sh2_poll_memory32(u32 a, u32 d, SH2_DRC *sh2);
    u32 REGPARM(2) p32x_sh2_fetch16(u32 a, SH2_DRC *sh2);
    void *p32x_sh2_get_mem_ptr(u32 a, u32 *mask, SH2_DRC *sh2);
    int p32x_sh2_mem_is_rom(u32 a, SH2_DRC *sh2);

#ifdef __cplusplus
}
#endif

#endif // PICO_INT_H
