// Adapter for PicoDrive SH2 DRC (included from sh2.cpp)

#ifdef VITA
// DRC_STEP logging for compare_traces.py
#define DRC_STEP_LOG 0
#define DRC_STEP_MAX 10000000

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
    printf("REF %08x: R0:%08x R1:%08x R2:%08x R3:%08x R4:%08x R5:%08x R13:%08x R14:%08x R15:%08x SR:%08x PR:%08x MACH:%08x MACL:%08x GBR:%08x VBR:%08x\n",
           ctx->pc,
           ctx->r[0], ctx->r[1], ctx->r[2], ctx->r[3],
           ctx->r[4], ctx->r[5], ctx->r[13], ctx->r[14], ctx->r[15],
           ctx->sr, ctx->pr, ctx->mach, ctx->macl, ctx->gbr, ctx->vbr);
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
    unsigned char *pr = pSh2Ext->MemMap[(a >> SH2_SHIFT) + SH2_WADD * 2];
    if ((uintptr_t)pr >= SH2_MAXHANDLER)
    {
#ifndef MSB_FIRST
        a ^= 0x02;
#endif
        return *((unsigned short *)(pr + (a & SH2_PAGEM)));
    }
    return pSh2Ext->ReadWord[(uintptr_t)pr](a);
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
        return (int)(signed char)(pSh2Ext->ReadByte[(uintptr_t)pr](a));
    }


    u32 REGPARM(2) p32x_sh2_read16(u32 a, SH2_DRC *sh2)
    {
        unsigned char *pr = pSh2Ext->MemMap[(a >> SH2_SHIFT)];
        if ((uintptr_t)pr >= SH2_MAXHANDLER)
        {
// Direct memory - CPS3 handlers use address XOR for endian correction
#ifndef MSB_FIRST
            // On little-endian host (Vita), apply same XOR as CPS3 handlers
            u32 addr_swapped = a ^ 0x02;
            unsigned short v = *((unsigned short *)(pr + (addr_swapped & SH2_PAGEM)));
            return (int)(signed short)v;
#else
            unsigned short v = *((unsigned short *)(pr + (a & SH2_PAGEM)));
            return (int)(signed short)v;
#endif
        }
        return (int)(signed short)pSh2Ext->ReadWord[(uintptr_t)pr](a);
    }

    u32 REGPARM(2) p32x_sh2_read32(u32 a, SH2_DRC *sh2)
    {

        unsigned char *pr = pSh2Ext->MemMap[(a >> SH2_SHIFT)];
        
        if ((uintptr_t)pr >= SH2_MAXHANDLER){
		    return *((unsigned int *)(pr + (a & SH2_PAGEM)));
	    }
	    u32 val = pSh2Ext->ReadLong[(uintptr_t)pr](a);
        return val;
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
        pSh2Ext->WriteByte[(uintptr_t)pr](a, d);
        return;
    }

    void REGPARM(3) p32x_sh2_write16(u32 a, u32 d, SH2_DRC *sh2)
    {
        unsigned char *pr = pSh2Ext->MemMap[(a >> SH2_SHIFT) + SH2_WADD];


        if ((uintptr_t)pr >= SH2_MAXHANDLER)
        {
#ifndef MSB_FIRST
            u32 addr_swapped = a ^ 0x02;
            *((unsigned short *)(pr + (addr_swapped & SH2_PAGEM))) = (unsigned short)d;
#else
            *((unsigned short *)(pr + (a & SH2_PAGEM))) = (unsigned short)d;
#endif
            return;
        }
        pSh2Ext->WriteWord[(uintptr_t)pr](a, d);
        return;
    }
    
    void REGPARM(3) p32x_sh2_write32(u32 a, u32 d, SH2_DRC *sh2)
    {

        unsigned char *pr = pSh2Ext->MemMap[(a >> SH2_SHIFT) + SH2_WADD];

        if ((uintptr_t)pr >= SH2_MAXHANDLER)
        {
            *((unsigned int *)(pr + (a & SH2_PAGEM))) = (unsigned int)d;
            return;
        }
        pSh2Ext->WriteLong[(uintptr_t)pr](a, d);
        return;
    }

    u32 REGPARM(3) p32x_sh2_poll_memory8(u32 a, u32 d, SH2_DRC *sh2) { return p32x_sh2_read8(a, sh2); }
    u32 REGPARM(3) p32x_sh2_poll_memory16(u32 a, u32 d, SH2_DRC *sh2) { return p32x_sh2_read16(a, sh2); }
    u32 REGPARM(3) p32x_sh2_poll_memory32(u32 a, u32 d, SH2_DRC *sh2) { return p32x_sh2_read32(a, sh2); }

    // Memory mapping helper (used by DRC for instruction fetching)
    // Returns pointer TO the data at address 'a', or NULL if not directly mappable
    void *p32x_sh2_get_mem_ptr(u32 a, u32 *mask, SH2_DRC *sh2)
    {

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
        // ROM = address has no write handler (write map entry is a handler index < SH2_MAXHANDLER)
        //unsigned char *pr_write = pSh2Ext->MemMap[(a >> SH2_SHIFT) + SH2_WADD];
        //return ((uintptr_t)pr_write < SH2_MAXHANDLER) ? 1 : 0;
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

    // Setup memory region pointers for the DRC (used for constant data optimization)
    // CPS3 memory map:
    //   0x00000000 = BIOS (RomBios)
    //   0x02000000 = Main RAM (RamMain)
    //   0x06000000 = Game ROM decrypted (RomGame_D)
    //   0xE0000000 = Data Array (internal cache)
    sh2_drc_ctx.p_bios  = pSh2Ext->MemMap[0x00000000 >> SH2_SHIFT];
    sh2_drc_ctx.p_sdram = pSh2Ext->MemMap[0x02000000 >> SH2_SHIFT];
    sh2_drc_ctx.p_rom   = pSh2Ext->MemMap[0x06000000 >> SH2_SHIFT];
    sh2_drc_ctx.p_da    = sh2_drc_ctx.data_array;

    printf("SH2DRC: p_bios=%p p_sdram=%p p_rom=%p p_da=%p\n",
           sh2_drc_ctx.p_bios, sh2_drc_ctx.p_sdram, sh2_drc_ctx.p_rom, sh2_drc_ctx.p_da);

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

    // Internal regs (m[] -> peri_regs, assuming peri_regs[0..63] maps to m[0..63])
    memcpy(sh2_drc_ctx.peri_regs, sh2->m, 64 * 4);

    // Timers and DMAC state (if DRC needs them directly)
    // Note: DRC may not use these directly, but sync for consistency
    // Add more if needed, e.g., sh2_drc_ctx.frc = sh2->frc; if field exists

    // DEBUG: Dump RAM on first run to verify code
    static bool ram_dumped = true;
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
    //printf("SH2DRC: Run %d cycles. PC=0x%08x IRQ=%d\n", cycles_to_run, sh2_drc_ctx.pc, level);

    // Execute
    //sh2_drc_record_trace(&sh2_drc_ctx);

    int cycles_remaining = 0;
#if DRC_STEP_LOG
    int cycles_left = cycles_to_run;
    static int logged_steps = 0;

    while (cycles_left > 0)
    {
        if (logged_steps < DRC_STEP_MAX)
        {
            sh2_drc_log_step(&sh2_drc_ctx);
            logged_steps++;
            
            // Step 1 cycle
            int chunk = 1;
            int rem = sh2_execute_drc(&sh2_drc_ctx, chunk);
            int executed = chunk - rem;
            
            // Failsafe
            if (executed <= 0) executed = 1;
            
            cycles_left -= executed;
        }
        else
        {
            abort();
            // Run rest
            int rem = sh2_execute_drc(&sh2_drc_ctx, cycles_left);
            cycles_left = rem; 
            break;
        }
    }
    cycles_remaining = cycles_left;
#else
    cycles_remaining = sh2_execute_drc(&sh2_drc_ctx, cycles_to_run);
#endif

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

    // Internal regs (peri_regs -> m[])
    memcpy(sh2->m, sh2_drc_ctx.peri_regs, 64 * 4);

    // Update total cycles (like interpreter)
    sh2->sh2_total_cycles += cycles - cycles_remaining;

    // Check timers and DMA (like interpreter does after each opcode)
    {
        unsigned int cy = sh2->sh2_total_cycles;
        if (sh2->dma_timer_active[0] && (cy - sh2->dma_timer_base[0]) >= sh2->dma_timer_cycles[0])
            sh2_dmac_callback(0);
        if (sh2->dma_timer_active[1] && (cy - sh2->dma_timer_base[1]) >= sh2->dma_timer_cycles[1])
            sh2_dmac_callback(1);
        if (sh2->timer_active && (cy - sh2->timer_base) >= sh2->timer_cycles)
            sh2_timer_callback();
    }

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
