// Adapter for PicoDrive SH2 DRC (included from sh2.cpp)

#ifdef VITA
// DRC_STEP logging for compare_traces.py
#define DRC_STEP_LOG 1
#define DRC_STEP_MAX 5000000

// Vita includes for RWX memory
#include <psp2/kernel/sysmem.h>
#include <psp2/io/fcntl.h>
#include <psp2/kernel/clib.h>
#define printf sceClibPrintf

// Defines for DRC
extern "C"
{
#include "drc/pico/pico_int.h"
#include "drc/picodrive_sh2.h"
#include "drc/compiler.h"
}

// From RetroArch bootstrap/vita/sbrk.c
extern "C" int getVMBlock();

// The JIT context
static SH2_DRC sh2_drc_ctx;
static bool sh2_drc_initialized = false;
static int sceBlock = 0;

#if DRC_STEP_LOG
static void sh2_drc_log_step(const SH2_DRC *ctx)
{
    printf("DRC_STEP PC:%08x SR:%08x R0:%08x R1:%08x R2:%08x R3:%08x R4:%08x R5:%08x R6:%08x R7:%08x R8:%08x R9:%08x R10:%08x R11:%08x R12:%08x R13:%08x R14:%08x R15:%08x PR:%08x MACH:%08x MACL:%08x GBR:%08x VBR:%08x\n",
           ctx->pc, ctx->sr,
           ctx->r[0], ctx->r[1], ctx->r[2], ctx->r[3],
           ctx->r[4], ctx->r[5], ctx->r[6], ctx->r[7],
           ctx->r[8], ctx->r[9], ctx->r[10], ctx->r[11],
           ctx->r[12], ctx->r[13], ctx->r[14], ctx->r[15],
           ctx->pr, ctx->mach, ctx->macl, ctx->gbr, ctx->vbr);
}
#endif

// Globals required by DRC (linked via pico_int.h externs)
Pico32x_t Pico32x;
Pico32xMem_t *Pico32xMem = NULL;
PicoIn_t PicoIn;
SH2_DRC sh2s[2]; // Dummy array, we use local context mostly but compiler refers to this

static Pico32xMem_t dummy_mem; // Storage for Pico32xMem

extern "C" u32 REGPARM(2) p32x_sh2_read16(u32 a, SH2_DRC *sh2);

// Opcode fetch uses FETCH map, not READ map
u32 REGPARM(2) p32x_sh2_fetch16(u32 a, SH2_DRC *sh2)
{
    if (!pSh2Ext)
        return 0xFFFF;

    unsigned char *pr = pSh2Ext->MemMap[(a >> SH2_SHIFT) + SH2_WADD * 2];
    if ((uintptr_t)pr >= SH2_MAXHANDLER)
    {
#ifndef MSB_FIRST
        a ^= 0x02;
#endif
        return *((unsigned short *)(pr + (a & SH2_PAGEM)));
    }
    if ((uintptr_t)pr < SH2_MAXHANDLER && pSh2Ext->ReadWord[(uintptr_t)pr])
    {
        unsigned short v = pSh2Ext->ReadWord[(uintptr_t)pr](a);
#ifdef VITA
        // FBA handlers on Little Endian hosts return byte-swapped values (1234 -> 3412)
        // We MUST un-swap to get correct native value
        if ((uintptr_t)pr == 2)
            v = (v << 8) | (v >> 8);
#endif
        return v;
    }

    return 0xFFFF;
}

// Last-op trace ring buffer (for debugging PC=0 issues)
#define SH2_SR_MASK 0x00000FF3u
#define SH2_DRC_TRACEBUF_SIZE 64
struct Sh2DrcTraceEntry
{
    u32 pc;
    u16 op;
    u16 pad;
    u32 sr;
    u32 r3, r4, r5, r10, r15, pr;
};
static Sh2DrcTraceEntry sh2_drc_tracebuf[SH2_DRC_TRACEBUF_SIZE];
static int sh2_drc_tracepos = 0;
static int sh2_drc_tracefilled = 0;

static void sh2_drc_record_trace(SH2_DRC *ctx)
{
    if (!ctx)
        return;

    Sh2DrcTraceEntry e = {};
    e.pc = ctx->pc;
    if (ctx->pc != 0)
        e.op = (u16)p32x_sh2_read16(ctx->pc, ctx);
    else
        e.op = 0xFFFF;
    e.sr = ctx->sr;
    e.r3 = ctx->r[3];
    e.r4 = ctx->r[4];
    e.r5 = ctx->r[5];
    e.r10 = ctx->r[10];
    e.r15 = ctx->r[15];
    e.pr = ctx->pr;

    sh2_drc_tracebuf[sh2_drc_tracepos] = e;
    sh2_drc_tracepos = (sh2_drc_tracepos + 1) % SH2_DRC_TRACEBUF_SIZE;
    if (sh2_drc_tracefilled < SH2_DRC_TRACEBUF_SIZE)
        sh2_drc_tracefilled++;
}

static void sh2_drc_dump_trace(const char *reason)
{
    if (sh2_drc_tracefilled == 0)
    {
        printf("SH2DRC: LASTOPS dump (%s): <empty>\n", reason ? reason : "?");
        return;
    }

    printf("SH2DRC: LASTOPS dump (%s):\n", reason ? reason : "?");
    int count = sh2_drc_tracefilled;
    int start = (sh2_drc_tracepos - count + SH2_DRC_TRACEBUF_SIZE) % SH2_DRC_TRACEBUF_SIZE;
    for (int i = 0; i < count; i++)
    {
        int idx = (start + i) % SH2_DRC_TRACEBUF_SIZE;
        const Sh2DrcTraceEntry *e = &sh2_drc_tracebuf[idx];
        printf("  %02d: PC=%08x OP=%04x SR=%08x R3=%08x R4=%08x R5=%08x R10=%08x R15=%08x PR=%08x\n",
               i, e->pc, e->op, e->sr, e->r3, e->r4, e->r5, e->r10, e->r15, e->pr);
    }
}

void cache_flush_d_inval_i(void *start, void *end)
{
    size_t len = (char *)end - (char *)start;
    sceKernelSyncVMDomain(sceBlock, start, len);
}

int plat_mem_set_exec(void *ptr, size_t size)
{
    return sceKernelOpenVMDomain();
}

void *plat_mem_get_for_drc(size_t size)
{
    void *mem = NULL;
    if (sceBlock <= 0)
    {
        printf("SH2DRC: Error - invalid sceBlock %x\n", sceBlock);
        return NULL;
    }
    int ret = sceKernelGetMemBlockBase(sceBlock, &mem);
    if (ret < 0)
        printf("SH2DRC: sceKernelGetMemBlockBase failed: 0x%x\n", ret);
    else
        printf("SH2DRC: MemBase: %p, size: %zu\n", mem, size);
    return mem;
}

void memset32(void *dest, int val, int count)
{
    int *d = (int *)dest;
    while (count--)
        *d++ = val;
}

// Interface handlers required by PicoDrive DRC
// They are called from generated code (ASM) or C helper functions
extern "C"
{

    // Safe read/write implementations that check pSh2Ext validity
    // and properly dispatch to FBA handlers

    // Read handlers - use pSh2Ext->MemMap for reads with validation
    u32 REGPARM(2) p32x_sh2_read8(u32 a, SH2_DRC *sh2)
    {
        // Debug first few calls
        static int r8_count = 0;
        if (r8_count++ < 5)
            printf("SH2DRC: Read8 a=%08x pSh2Ext=%p\n", a, (void *)pSh2Ext);

        if (!pSh2Ext)
            return 0xFFFFFFFF; // Sign-extended

        unsigned char *pr = pSh2Ext->MemMap[(a >> SH2_SHIFT)];
        if ((uintptr_t)pr >= SH2_MAXHANDLER)
        {
#ifndef MSB_FIRST
            // CPS3 stores bytes swapped, XOR address like CPS3 handlers do
            u32 addr_swapped = a ^ 0x03;
            return (int)(signed char)(*((unsigned char *)(pr + (addr_swapped & SH2_PAGEM))));
#else
            return (int)(signed char)(*((unsigned char *)(pr + (a & SH2_PAGEM))));
#endif
        }
        // Handler mode
        if ((uintptr_t)pr < SH2_MAXHANDLER && pSh2Ext->ReadByte[(uintptr_t)pr])
        {
            return (int)(signed char)(pSh2Ext->ReadByte[(uintptr_t)pr](a));
        }
        printf("SH2DRC: Read8 unmapped %08x\n", a);
        return 0xFFFFFFFF; // Sign-extended 0xFF
    }

    u32 REGPARM(2) p32x_sh2_read16(u32 a, SH2_DRC *sh2)
    {
        if (!pSh2Ext)
            return 0xFFFFFFFF; // Sign-extended

        int trace_060008 = (sh2 && sh2->pc >= 0x06000840 && sh2->pc <= 0x06000900 && a >= 0x06000900);

        unsigned char *pr = pSh2Ext->MemMap[(a >> SH2_SHIFT)];
        if ((uintptr_t)pr >= SH2_MAXHANDLER)
        {
// Direct memory - CPS3 handlers use address XOR for endian correction
#ifndef MSB_FIRST
            // On little-endian host (Vita), apply same XOR as CPS3 handlers
            u32 addr_swapped = a ^ 0x02;
            unsigned short v = *((unsigned short *)(pr + (addr_swapped & SH2_PAGEM)));
            if (trace_060008)
            {
                static int r16_trace = 0;
                if (r16_trace++ < 200)
                    printf("SH2DRC: R16 PC=%08x A=%08x D=%04x\n", sh2->pc, a, v);
            }
            return (int)(signed short)v;
#else
            unsigned short v = *((unsigned short *)(pr + (a & SH2_PAGEM)));
            if (trace_060008)
            {
                static int r16_trace = 0;
                if (r16_trace++ < 200)
                    printf("SH2DRC: R16 PC=%08x A=%08x D=%04x\n", sh2->pc, a, v);
            }
            return (int)(signed short)v;
#endif
        }
        if ((uintptr_t)pr < SH2_MAXHANDLER && pSh2Ext->ReadWord[(uintptr_t)pr])
        {
            unsigned short v = pSh2Ext->ReadWord[(uintptr_t)pr](a);
#ifdef VITA
            // FBA handlers on Little Endian hosts return byte-swapped values (1234 -> 3412)
            // We MUST un-swap to get correct native value
            if ((uintptr_t)pr == 2)
                v = (v << 8) | (v >> 8);
#endif
            if (trace_060008)
            {
                static int r16_trace = 0;
                if (r16_trace++ < 200)
                    printf("SH2DRC: R16 PC=%08x A=%08x D=%04x\n", sh2->pc, a, v);
            }
            return (int)(signed short)v;
        }
        return 0xFFFFFFFF; // Sign-extended 0xFFFF
    }

    u32 REGPARM(2) p32x_sh2_read32(u32 a, SH2_DRC *sh2)
    {
        if (!pSh2Ext)
            return 0xFFFFFFFF;

        unsigned char *pr = pSh2Ext->MemMap[(a >> SH2_SHIFT)];
        if ((uintptr_t)pr >= SH2_MAXHANDLER)
        {
            // Direct memory - 32-bit reads are naturally aligned and don't need word-swap
            // because FBA memory for CPS3 is already word-swapped for 16-bit access,
            // which effectively makes 32-bit reads correct Big Endian on Little Endian hosts.
            unsigned int val = *((unsigned int *)(pr + (a & SH2_PAGEM)));
            
            static int r32_log = 0;
            if (r32_log++ < 20 && a >= 0x06000000 && a < 0x06000200)
            {
                printf("SH2DRC: R32 Direct PC=%08x A=%08x -> %08x\n", sh2 ? sh2->pc : 0, a, val);
            }
            return val;
        }
        if ((uintptr_t)pr < SH2_MAXHANDLER && pSh2Ext->ReadLong[(uintptr_t)pr])
        {
            // FBA ROM/RAM handlers on Little Endian hosts return data with 16-bit halves swapped (scrambled).
            // We MUST un-swap it to get the correct native value.
            typedef unsigned int (*ReadLongHandler)(unsigned int);
            unsigned int val = ((ReadLongHandler)pSh2Ext->ReadLong[(uintptr_t)pr])(a);
            
#ifdef VITA
            // Undo FBA's half-swap to get correct Big Endian value
            if ((uintptr_t)pr == 2)
                val = (val << 16) | (val >> 16);
#endif

            static int r32h_log = 0;
            if (r32h_log++ < 20 && a >= 0x06000000 && a < 0x06000200)
            {
                printf("SH2DRC: R32 Handler PC=%08x A=%08x -> %08x (h=%u)\n", sh2 ? sh2->pc : 0, a, val, (unsigned)(uintptr_t)pr);
            }
            return val;
        }
        return 0xFFFFFFFF;
    }

    // Write handlers - use pSh2Ext->MemMap for writes with validation
    void REGPARM(3) p32x_sh2_write8(u32 a, u32 d, SH2_DRC *sh2)
    {
        if (!pSh2Ext)
            return;

        unsigned char *pr = pSh2Ext->MemMap[(a >> SH2_SHIFT) + SH2_WADD];
        if ((uintptr_t)pr >= SH2_MAXHANDLER)
        {
#ifndef MSB_FIRST
            u32 addr_swapped = a ^ 0x03;
            *((unsigned char *)(pr + (addr_swapped & SH2_PAGEM))) = (unsigned char)d;
#else
            *((unsigned char *)(pr + (a & SH2_PAGEM))) = (unsigned char)d;
#endif
            return;
        }
        if ((uintptr_t)pr < SH2_MAXHANDLER && pSh2Ext->WriteByte[(uintptr_t)pr])
        {
            pSh2Ext->WriteByte[(uintptr_t)pr](a, d);
            return;
        }
        static int w8_unmapped = 0;
        if (w8_unmapped++ < 10)
            printf("SH2DRC: Write8 unmapped %08x = %02x\n", a, d);
    }

    void REGPARM(3) p32x_sh2_write16(u32 a, u32 d, SH2_DRC *sh2)
    {
        if (!pSh2Ext)
            return;

        if (sh2 && sh2->pc >= 0x06000840 && sh2->pc <= 0x06000900)
        {
            static int w16_trace = 0;
            if (w16_trace++ < 200)
                printf("SH2DRC: W16 PC=%08x A=%08x D=%04x\n", sh2->pc, a, d & 0xFFFF);
        }

        unsigned char *pr = pSh2Ext->MemMap[(a >> SH2_SHIFT) + SH2_WADD];
        
        // Force handler for Palette RAM (0x0408xxxx) to trigger update logic
        if (a >= 0x04080000 && a < 0x040c0000)
        {
#ifdef VITA
             // FBA handers expect swapped bytes on Little Endian hosts
             d = (d << 8) | (d >> 8);
#endif
             // HW Palette Handler is index 4 in cps3run.cpp
             if (pSh2Ext->WriteWord[4])
                 pSh2Ext->WriteWord[4](a, d);
             else
                 printf("SH2DRC: ERROR - Palette Handler 4 missing!\n");
             return;
        }

        if ((uintptr_t)pr >= SH2_MAXHANDLER)
        {
// Apply same swap as read16
#ifndef MSB_FIRST
            u32 addr_swapped = a ^ 0x02;
            *((unsigned short *)(pr + (addr_swapped & SH2_PAGEM))) = (unsigned short)d;
#else
            *((unsigned short *)(pr + (a & SH2_PAGEM))) = (unsigned short)d;
#endif
            return;
        }
        if ((uintptr_t)pr < SH2_MAXHANDLER && pSh2Ext->WriteWord[(uintptr_t)pr])
        {
#ifdef VITA
            // FBA handers expect swapped bytes on Little Endian hosts
            if ((uintptr_t)pr == 2)
                d = (d << 8) | (d >> 8);
#endif
            pSh2Ext->WriteWord[(uintptr_t)pr](a, d);
            return;
        }
        static int w16_unmapped = 0;
        if (w16_unmapped++ < 10)
            printf("SH2DRC: Write16 unmapped %08x = %04x\n", a, d);
    }

    void REGPARM(3) p32x_sh2_write32(u32 a, u32 d, SH2_DRC *sh2)
    {
        if (!pSh2Ext)
            return;

        if (sh2 && sh2->pc >= 0x06000840 && sh2->pc <= 0x06000900)
        {
            static int w32_trace = 0;
            if (w32_trace++ < 200)
            {
                u32 op = p32x_sh2_read16(sh2->pc, sh2);
                printf("SH2DRC: W32 PC=%08x OP=%04x A=%08x D=%08x\n", sh2->pc, op & 0xFFFF, a, d);
            }
        }
        if (a >= 0x06000000 && a < 0x06000100)
        {
            static int w32_vec_trace = 0;
            if (w32_vec_trace++ < 50)
                printf("SH2DRC: W32V PC=%08x A=%08x D=%08x\n", sh2 ? sh2->pc : 0, a, d);
        }

        unsigned char *pr = pSh2Ext->MemMap[(a >> SH2_SHIFT) + SH2_WADD];

        // Force handler for Palette RAM (0x0408xxxx) to trigger update logic
        int force_handler = (a >= 0x04080000 && a < 0x040c0000);

        if (!force_handler && (uintptr_t)pr >= SH2_MAXHANDLER)
        {
            // Direct memory
#ifndef MSB_FIRST
            // CPS3 stores 16-bit words swapped on little-endian hosts
            // u32 sd = (d << 16) | (d >> 16); // FIX: No swap needed for 32-bit access!
            *((unsigned int *)(pr + (a & SH2_PAGEM))) = (unsigned int)d;
#else
            *((unsigned int *)(pr + (a & SH2_PAGEM))) = (unsigned int)d;
#endif
            return;
        }

        // Try handler
        if ((uintptr_t)pr < SH2_MAXHANDLER && pSh2Ext->WriteLong[(uintptr_t)pr])
        {
            pSh2Ext->WriteLong[(uintptr_t)pr](a, d);
            return;
        }
    }

    u32 REGPARM(3) p32x_sh2_poll_memory8(u32 a, u32 d, SH2_DRC *sh2) { return p32x_sh2_read8(a, sh2); }
    u32 REGPARM(3) p32x_sh2_poll_memory16(u32 a, u32 d, SH2_DRC *sh2) { return p32x_sh2_read16(a, sh2); }
    u32 REGPARM(3) p32x_sh2_poll_memory32(u32 a, u32 d, SH2_DRC *sh2) { return p32x_sh2_read32(a, sh2); }

    // Memory mapping helper (used by DRC for instruction fetching)
    // Returns pointer TO the data at address 'a', or NULL if not directly mappable
    void *p32x_sh2_get_mem_ptr(u32 a, u32 *mask, SH2_DRC *sh2)
    {
#ifdef VITA
        // Force handler-based access on Vita to avoid endian/alias issues
        static int mp_log = 0;
        if (mp_log++ < 5)
            printf("SH2DRC: p32x_sh2_get_mem_ptr disabled on Vita (a=%08x)\n", a);
        return NULL;
#else
        if (!pSh2Ext)
        {
            printf("SH2DRC: p32x_sh2_get_mem_ptr - pSh2Ext is NULL!\n");
            return NULL;
        }

        // Get the pointer from FBA's FETCH map (for fetching instructions)
        // FBA's MemMap layout:
        //   Offset 0:           READ pointers
        //   Offset SH2_WADD:    WRITE pointers
        //   Offset SH2_WADD*2:  FETCH pointers (what we need for JIT instruction fetching)
        unsigned char *pr = pSh2Ext->MemMap[(a >> SH2_SHIFT) + SH2_WADD * 2];

        // Check if it's a valid host pointer (not a handler index)
        if ((uintptr_t)pr >= SH2_MAXHANDLER)
        {
            // Valid host pointer
            // pr points to the start of the page containing 'a'
            // We need to return the actual pointer to the data at 'a'
            if (mask)
                *mask = SH2_PAGEM;

            // pr + (a & SH2_PAGEM) = pointer to data at address 'a'
            void *ptr = (void *)(pr + (a & SH2_PAGEM));
            // printf("SH2DRC: p32x_sh2_get_mem_ptr a=%08x -> %p\n", a, ptr);
            return ptr;
        }

        printf("SH2DRC: p32x_sh2_get_mem_ptr a=%08x -> NULL (pr=%p, handler=%d)\n",
               a, (void *)pr, (int)(uintptr_t)pr);
        return NULL;
#endif
    }

    // Polling detection
    void REGPARM(4) p32x_sh2_poll_detect(u32 a, SH2_DRC *sh2, u32 flags, int maxcnt)
    {
        // If we are polling too much, yield the timeslice to let other devices run (and trigger IRQs)
        if (maxcnt > 5) // Threshold for loop detection
        {
            static int poll_log = 0;
            if (poll_log++ < 20)
                printf("SH2DRC: Poll detect at PC=%08x (cnt=%d) - Yielding\n", sh2->pc, maxcnt);
            
            // Abort current timeslice
            sh2->cycles_timeslice = 0;
        }
    }

    void REGPARM(4) p32x_sh2_poll_event(u32 a, SH2_DRC *sh2, u32 flags, u32 m68k_cycles)
    {
        // Check if an interrupt is pending that should break the poll
        if (sh2->pending_level >= 0)
        {
             static int poll_ev_log = 0;
             if (poll_ev_log++ < 20)
                 printf("SH2DRC: Poll event at PC=%08x - IRQ pending %d\n", sh2->pc, sh2->pending_level);
             
             // Abort to process IRQ
             sh2->cycles_timeslice = 0;
        }
    }

    int p32x_sh2_mem_is_rom(u32 a, SH2_DRC *sh2)
    {
        return 0;
    }

} // extern "C"

// Memory map tables for DRC - 128 entries (2^(32-25))
#define SH2_MAP_SIZE (1 << (32 - SH2_READ_SHIFT))

// Read map: each entry is 8 bytes [handler_ptr | 0x80000000, mask]
// The MSB of handler_ptr tells DRC this is a handler, not direct memory
static uintptr_t sh2_drc_read8_map[SH2_MAP_SIZE * 2];
static uintptr_t sh2_drc_read16_map[SH2_MAP_SIZE * 2];
static uintptr_t sh2_drc_read32_map[SH2_MAP_SIZE * 2];

// Write map: each entry is a function pointer
static const void *sh2_drc_write8_tab[SH2_MAP_SIZE];
static const void *sh2_drc_write16_tab[SH2_MAP_SIZE];
static const void *sh2_drc_write32_tab[SH2_MAP_SIZE];

static void sh2_drc_setup_mem_maps()
{
    printf("SH2DRC: Setting up memory maps (%d entries)...\n", SH2_MAP_SIZE);

    // The JIT's emith_sh2_rcall does: func = func + func (doubles)
    // Then checks carry flag - if set, it's a handler, if clear it's direct memory
    // After doubling, the result is used as the function pointer to call
    // So we store: (func_ptr >> 1) | 0x80000000
    // After doubling: ((func_ptr >> 1) | 0x80000000) * 2 = func_ptr (with carry set)

    uintptr_t read8_half = ((uintptr_t)(void *)p32x_sh2_read8 >> 1) | 0x80000000u;
    uintptr_t read16_half = ((uintptr_t)(void *)p32x_sh2_read16 >> 1) | 0x80000000u;
    uintptr_t read32_half = ((uintptr_t)(void *)p32x_sh2_read32 >> 1) | 0x80000000u;

    printf("SH2DRC: read8=%p, half=%08lx\n", (void *)p32x_sh2_read8, (unsigned long)read8_half);

    for (int i = 0; i < SH2_MAP_SIZE; i++)
    {
        // Read maps: [handler_half | MSB, mask]
        // handler_half = (func_ptr >> 1) | 0x80000000
        // When doubled: becomes func_ptr with carry set
        sh2_drc_read8_map[i * 2] = read8_half;
        sh2_drc_read8_map[i * 2 + 1] = 0xFFFFFFFF; // mask
        sh2_drc_read16_map[i * 2] = read16_half;
        sh2_drc_read16_map[i * 2 + 1] = 0xFFFFFFFF;
        sh2_drc_read32_map[i * 2] = read32_half;
        sh2_drc_read32_map[i * 2 + 1] = 0xFFFFFFFF;

        // Write maps: simple function pointer (wcall uses different mechanism)
        sh2_drc_write8_tab[i] = (const void *)p32x_sh2_write8;
        sh2_drc_write16_tab[i] = (const void *)p32x_sh2_write16;
        sh2_drc_write32_tab[i] = (const void *)p32x_sh2_write32;
    }

    // Assign to DRC context
    sh2_drc_ctx.read8_map = sh2_drc_read8_map;
    sh2_drc_ctx.read16_map = sh2_drc_read16_map;
    sh2_drc_ctx.read32_map = sh2_drc_read32_map;
    sh2_drc_ctx.write8_tab = sh2_drc_write8_tab;
    sh2_drc_ctx.write16_tab = sh2_drc_write16_tab;
    sh2_drc_ctx.write32_tab = sh2_drc_write32_tab;

    printf("SH2DRC: Memory maps configured.\n");
}

// IRQ Callback
static int REGPARM(2) drc_irq_callback(SH2_DRC *sh2, int level)
{
    if (level == 16)
    {
        printf("SH2DRC: NMI requested\n");
        return 11; // NMI
    }
    if (sh2 && sh2->pending_int_irq == level)
    {
        printf("SH2DRC: Pending interrupt %d acknowledged, vector=%d\n", level, sh2->pending_int_vector);
        return sh2->pending_int_vector;
    }

    // FBA SH2 core uses (64 + level / 2) for external IRQ pins (IRQ0-IRQ7)
    // This differs from default SH2 autovectors (64 + level).
    int vector = 64 + level / 2;
    static int cb_log = 0;
    if (cb_log++ < 20)
    {
        printf("SH2DRC: IRQ callback for level %d - returning vector %d (FBA mapping)\n", level, vector);
    }
    return vector;
}

// Initialize DRC
void Sh2DrcInit()
{
    if (sh2_drc_initialized)
        return;

    if (!sh2)
    {
        if (pSh2Ext)
            sh2 = &pSh2Ext->sh2;
        if (!sh2)
        {
            printf("SH2DRC: Init deferred (sh2 is NULL)\n");
            return;
        }
    }

    printf("SH2DRC: Initializing...\n");

    // Get the RWX memory block from RetroArch
    sceBlock = getVMBlock();
    printf("SH2DRC: getVMBlock returned 0x%x\n", sceBlock);

    // Init globals
    memset(&Pico32x, 0, sizeof(Pico32x));
    memset(&dummy_mem, 0, sizeof(dummy_mem));

    // Allocate DRC block tracking arrays (for SMC detection)
    // RAM_SIZE(0) = 0x40000, RAM_SIZE(1) = 0x1000
    static unsigned char drcblk_ram_storage[0x40000];
    static unsigned char drcblk_da0_storage[0x1000];
    static unsigned char drcblk_da1_storage[0x1000];
    static unsigned char drclit_ram_storage[0x40000];
    static unsigned char drclit_da0_storage[0x1000];
    static unsigned char drclit_da1_storage[0x1000];

    memset(drcblk_ram_storage, 0, sizeof(drcblk_ram_storage));
    memset(drcblk_da0_storage, 0, sizeof(drcblk_da0_storage));
    memset(drcblk_da1_storage, 0, sizeof(drcblk_da1_storage));
    memset(drclit_ram_storage, 0, sizeof(drclit_ram_storage));
    memset(drclit_da0_storage, 0, sizeof(drclit_da0_storage));
    memset(drclit_da1_storage, 0, sizeof(drclit_da1_storage));

    dummy_mem.drcblk_ram = drcblk_ram_storage;
    dummy_mem.drcblk_da[0] = drcblk_da0_storage;
    dummy_mem.drcblk_da[1] = drcblk_da1_storage;
    dummy_mem.drclit_ram = drclit_ram_storage;
    dummy_mem.drclit_da[0] = drclit_da0_storage;
    dummy_mem.drclit_da[1] = drclit_da1_storage;

    printf("SH2DRC: Allocated DRC tracking arrays\n");

    Pico32xMem = &dummy_mem;
    PicoIn.opt = POPT_EN_DRC; // Enable DRC

    memset(&sh2_drc_ctx, 0, sizeof(sh2_drc_ctx));
    sh2_drc_ctx.irq_callback = drc_irq_callback;

    // Setup memory maps BEFORE sh2_drc_init so the DRC has valid pointers
    sh2_drc_setup_mem_maps();

    // Initialize DRC context with starting state from FBA sh2
    sh2_drc_ctx.pc = sh2->pc;
    sh2_drc_ctx.r[15] = sh2->r[15];
    sh2_drc_ctx.sr = sh2->sr & SH2_SR_MASK;
    sh2_drc_ctx.vbr = sh2->vbr;

    sh2_drc_init(&sh2_drc_ctx);

    sh2_drc_initialized = true;
    printf("SH2DRC: Initialization complete. Initial PC=0x%08x SP=0x%08x\n",
           sh2_drc_ctx.pc, sh2_drc_ctx.r[15]);
}

// Reset DRC cache (called when SH2 is reset)
void Sh2DrcReset()
{
    if (!sh2_drc_initialized)
        return;

    printf("SH2DRC: Reset called. Flushing code cache...\n");

    // Flush the entire DRC cache (takes no arguments)
    sh2_drc_flush_all();

    // Reset DRC context to match FBA state
    sh2_drc_ctx.pc = sh2->pc;
    memcpy(sh2_drc_ctx.r, sh2->r, 16 * 4);
    sh2_drc_ctx.pr = sh2->pr;
    sh2_drc_ctx.sr = sh2->sr & SH2_SR_MASK;
    sh2_drc_ctx.gbr = sh2->gbr;
    sh2_drc_ctx.vbr = sh2->vbr;
    sh2_drc_ctx.mach = sh2->mach;
    sh2_drc_ctx.macl = sh2->macl;

    printf("SH2DRC: Reset complete. PC=0x%08x\n", sh2_drc_ctx.pc);
}

// Run DRC
INT32 Sh2RunDrc(INT32 cycles)
{
    if (!sh2_drc_initialized)
        Sh2DrcInit();

    // Validate DRC entry points to avoid jumping to NULL
    static int ep_checked = 0;
    if (!ep_checked)
    {
        uintptr_t entry = 0, dispatcher = 0, exitp = 0;
        sh2_drc_get_entry_points(&entry, &dispatcher, &exitp);
        printf("SH2DRC: entry=%p dispatcher=%p exit=%p\n",
               (void *)entry, (void *)dispatcher, (void *)exitp);
        ep_checked = 1;
    }

    if (sh2_drc_ctx.pc == 0)
    {
        printf("SH2DRC: ERROR - ctx PC is 0 before execute\n");
        sh2_drc_dump_trace("pc=0 before execute");
        return 0;
    }

    // 1. Sync FBA state -> DRC state
    // Regs
    memcpy(sh2_drc_ctx.r, sh2->r, 16 * 4);
    sh2_drc_ctx.pc = sh2->pc;
    sh2_drc_ctx.pr = sh2->pr;
    sh2_drc_ctx.sr = sh2->sr & SH2_SR_MASK;
    sh2_drc_ctx.gbr = sh2->gbr;
    sh2_drc_ctx.vbr = sh2->vbr;
    sh2_drc_ctx.mach = sh2->mach;
    sh2_drc_ctx.macl = sh2->macl;

    // DEBUG: Dump RAM on first run to verify code
    static bool ram_dumped = false;
    if (!ram_dumped && sh2_drc_ctx.pc == 0x06000ea0) // Wait for start PC
    {
        printf("SH2DRC: Dumping RAM 06000000-06040000...\n");
        int fd = sceIoOpen("ux0:data/retroarch/sh2_ram.bin", 0x602, 0777); // WRONLY|CREAT|TRUNC
        if (fd >= 0)
        {
            // Dump 256KB
            for (u32 i = 0; i < 0x40000; i++)
            {
                u8 b = p32x_sh2_read8(0x06000000 + i, &sh2_drc_ctx);
                sceIoWrite(fd, &b, 1);
            }
            sceIoClose(fd);
            printf("SH2DRC: RAM Dumped!\n");
        }
        else
        {
            printf("SH2DRC: Failed to open dump file!\n");
        }
        ram_dumped = true;
    }

    // The DRC expects 'cycles_timeslice' to be set
    int cycles_to_run = cycles;
    // Debug: force tiny timeslice in target ranges to capture execution trace
    static int trace_left = 0;
    static int trace_mode = 0;
    if (sh2_drc_ctx.pc >= 0x06000800 && sh2_drc_ctx.pc <= 0x06000a00)
    {
        if (trace_mode != 1)
        {
            trace_mode = 1;
            trace_left = 2000;
            printf("SH2DRC: TRACE mode=060008xx enabled\n");
        }
    }
    else if (sh2_drc_ctx.pc >= 0x06000ea0 && sh2_drc_ctx.pc <= 0x06001000)
    {
        if (trace_mode != 2)
        {
            trace_mode = 2;
            trace_left = 2000;
            printf("SH2DRC: TRACE mode=06000exx enabled\n");
        }
    }

    sh2_drc_ctx.cycles_timeslice = cycles_to_run;


    // Sync interrupts: Convert FBA's pending_irq bitmask to level (include internal IRQs)

    int level = -1;
    if (sh2->pending_irq)
    {
        for (int i = 15; i >= 0; i--)
        {
            if (sh2->pending_irq & (1 << i))
            {
                level = i;
                break;
            }
        }
    }
    if (sh2->internal_irq_level > level)
        level = sh2->internal_irq_level;
    
    // Log IRQ transitions to debug synchronization issues
    if (sh2_drc_ctx.pending_level != level)
    {
        static int irq_trans_log = 0;
        if (irq_trans_log++ < 100)
            printf("SH2DRC: IRQ level transition: %d -> %d (pending_irq=0x%04x internal=%d)\n", 
                   sh2_drc_ctx.pending_level, level, sh2->pending_irq, sh2->internal_irq_level);
    }

    sh2_drc_ctx.pending_level = level;

    // Sync internal IRQ vector/level for correct exception vectoring
    sh2_drc_ctx.pending_int_irq = sh2->internal_irq_level;
    sh2_drc_ctx.pending_int_vector = sh2->internal_irq_vector;

    // Clear other IRQ fields
    sh2_drc_ctx.pending_irl = 0;

    static int run_count = 0;
    printf("SH2DRC: Run %d cycles. PC=0x%08x IRQ=%d\n", cycles_to_run, sh2_drc_ctx.pc, level);

    // Execute
    sh2_drc_record_trace(&sh2_drc_ctx);

    static u32 last_pc = 0;
    static int trace_dumped_060008 = 0;
    static int trace_dumped_06000cdx = 0;
    static int trace_dumped_06000840 = 0;

    int cycles_remaining = 0;
#if 0
    int total_executed = 0;
    while (total_executed < cycles_to_run)
    {
        sh2_drc_ctx.cycles_timeslice = 1;
        int rem = sh2_execute_drc(&sh2_drc_ctx, 1);
        int executed = 1 - rem;
        if (executed <= 0)
            executed = 1;
        total_executed += executed;
        last_pc = sh2_drc_ctx.pc;

        static int drc_step_count = 0;
        if (DRC_STEP_MAX <= 0 || drc_step_count < DRC_STEP_MAX)
        {
            sh2_drc_log_step(&sh2_drc_ctx);
            drc_step_count++;
        }
    }
    cycles_remaining = cycles_to_run - total_executed;
#else
    cycles_remaining = sh2_execute_drc(&sh2_drc_ctx, cycles_to_run);
    last_pc = sh2_drc_ctx.pc;
#endif

    if (sh2_drc_ctx.pc == 0)
    {
        printf("SH2DRC: ERROR - ctx PC became 0 (last PC=%08x)\n", last_pc);
        sh2_drc_dump_trace("pc=0 after execute");
        abort();
    }

    // Debug: dump if PC looks corrupt
    if (sh2_drc_ctx.pc >= 0xF0000000 || sh2_drc_ctx.pc == 0)
    {
        printf("SH2DRC: WARNING - Invalid PC after exec: 0x%08x\n", sh2_drc_ctx.pc);
        printf("  r0-3:  %08x %08x %08x %08x\n", sh2_drc_ctx.r[0], sh2_drc_ctx.r[1], sh2_drc_ctx.r[2], sh2_drc_ctx.r[3]);
        printf("  r4-7:  %08x %08x %08x %08x\n", sh2_drc_ctx.r[4], sh2_drc_ctx.r[5], sh2_drc_ctx.r[6], sh2_drc_ctx.r[7]);
        printf("  r8-11: %08x %08x %08x %08x\n", sh2_drc_ctx.r[8], sh2_drc_ctx.r[9], sh2_drc_ctx.r[10], sh2_drc_ctx.r[11]);
        printf("  r12-15:%08x %08x %08x %08x\n", sh2_drc_ctx.r[12], sh2_drc_ctx.r[13], sh2_drc_ctx.r[14], sh2_drc_ctx.r[15]);
        printf("  PR=%08x SR=%08x VBR=%08x GBR=%08x\n", sh2_drc_ctx.pr, sh2_drc_ctx.sr, sh2_drc_ctx.vbr, sh2_drc_ctx.gbr);
        abort();
    }

    // 3. Sync DRC state -> FBA state
    memcpy(sh2->r, sh2_drc_ctx.r, 16 * 4);
    sh2->pc = sh2_drc_ctx.pc;
    sh2->pr = sh2_drc_ctx.pr;
    sh2->sr = sh2_drc_ctx.sr & SH2_SR_MASK;
    sh2->gbr = sh2_drc_ctx.gbr;
    sh2->vbr = sh2_drc_ctx.vbr;
    sh2->mach = sh2_drc_ctx.mach;
    sh2->macl = sh2_drc_ctx.macl;

    return cycles - cycles_remaining; // Cycles executed
}

// Finish/Exit
void Sh2DrcExit()
{
    if (sh2_drc_initialized)
    {
        sh2_drc_finish(&sh2_drc_ctx);
        sh2_drc_initialized = false;
    }
}

#endif // VITA
