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

// The JIT context IS the global sh2 pointer now
static bool sh2_drc_initialized = false;
static int sceBlock = 0;

// Globals required by DRC (linked via pico_int.h externs)
Pico32x_t Pico32x;
Pico32xMem_t *Pico32xMem = NULL;
PicoIn_t PicoIn;
// SH2_DRC sh2s[2]; // Removed, we use FBA sh2

static Pico32xMem_t dummy_mem; // Storage for Pico32xMem

// Prototypes
extern "C" u32 REGPARM(2) p32x_sh2_read16(u32 a, SH2_DRC *sh2);

// Sync icount from SR (for wrappers)
static inline void drc_sync_icount(SH2 *ctx)
{
    // Extract cycles from SR (bits 12-31)
    // Note: sh2_execute_drc packs (cycles-1) << 12
    ctx->icount = (ctx->sr >> 12);
}

// Opcode fetch uses FETCH map, not READ map
u32 REGPARM(2) p32x_sh2_fetch16(u32 a, SH2_DRC *ctx)
{
    SH2* sh2_ptr = (SH2*)ctx;
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

    // Read handlers - use pSh2Ext->MemMap for reads with validation
    u32 REGPARM(2) p32x_sh2_read8(u32 a, SH2_DRC *ctx)
    {
        SH2* sh2_ptr = (SH2*)ctx;
        drc_sync_icount(sh2_ptr); // Sync time before IO access

        unsigned char *pr = pSh2Ext->MemMap[(a >> SH2_SHIFT)];
        if ((uintptr_t)pr >= SH2_MAXHANDLER)
        {
#ifndef MSB_FIRST
            u32 addr_swapped = a ^ 0x03;
            return (int)(signed char)(*((unsigned char *)(pr + (addr_swapped & SH2_PAGEM))));
#else
            return (int)(signed char)(*((unsigned char *)(pr + (a & SH2_PAGEM))));
#endif
        }
        return (int)(signed char)(pSh2Ext->ReadByte[(uintptr_t)pr](a));
    }


    u32 REGPARM(2) p32x_sh2_read16(u32 a, SH2_DRC *ctx)
    {
        SH2* sh2_ptr = (SH2*)ctx;
        drc_sync_icount(sh2_ptr); // Sync unconditionally

        unsigned char *pr = pSh2Ext->MemMap[(a >> SH2_SHIFT)];
        if ((uintptr_t)pr >= SH2_MAXHANDLER)
        {
#ifndef MSB_FIRST
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

    u32 REGPARM(2) p32x_sh2_read32(u32 a, SH2_DRC *ctx)
    {
        SH2* sh2_ptr = (SH2*)ctx;
        drc_sync_icount(sh2_ptr);

        unsigned char *pr = pSh2Ext->MemMap[(a >> SH2_SHIFT)];
        
        if ((uintptr_t)pr >= SH2_MAXHANDLER){
		    return *((unsigned int *)(pr + (a & SH2_PAGEM)));
	    }
	    u32 val = pSh2Ext->ReadLong[(uintptr_t)pr](a);
        return val;
    }

    // Write handlers - use pSh2Ext->MemMap for writes with validation
    void REGPARM(3) p32x_sh2_write8(u32 a, u32 d, SH2_DRC *ctx)
    {
        SH2* sh2_ptr = (SH2*)ctx;
        drc_sync_icount(sh2_ptr); // Critical for Timer/DMAC activation time

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

    void REGPARM(3) p32x_sh2_write16(u32 a, u32 d, SH2_DRC *ctx)
    {
        SH2* sh2_ptr = (SH2*)ctx;
        drc_sync_icount(sh2_ptr);

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
    
    void REGPARM(3) p32x_sh2_write32(u32 a, u32 d, SH2_DRC *ctx)
    {
        SH2* sh2_ptr = (SH2*)ctx;
        drc_sync_icount(sh2_ptr);

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

    void *p32x_sh2_get_mem_ptr(u32 a, u32 *mask, SH2_DRC *ctx)
    {
        unsigned char *pr = pSh2Ext->MemMap[(a >> SH2_SHIFT) + SH2_WADD * 2];
        if ((uintptr_t)pr >= SH2_MAXHANDLER)
        {
            if (mask) *mask = SH2_PAGEM;
            return (void *)(pr + (a & SH2_PAGEM));
        }
        return NULL;
    }

    void REGPARM(4) p32x_sh2_poll_detect(u32 a, SH2_DRC *ctx, u32 flags, int maxcnt)
    {
        SH2* sh2_ptr = (SH2*)ctx;
        if (maxcnt > 5)
        {
            // Yield timeslice
            sh2_ptr->cycles_timeslice = 0;
            // Also zero icount so GetTotalCycles reflects end of slice
            // sh2_ptr->icount = 0; // Not stricly needed if cycles_timeslice checked
        }
    }

    void REGPARM(4) p32x_sh2_poll_event(u32 a, SH2_DRC *ctx, u32 flags, u32 m68k_cycles)
    {
        SH2* sh2_ptr = (SH2*)ctx;
        if (sh2_ptr->pending_level >= 0)
        {
            sh2_ptr->cycles_timeslice = 0;
        }
    }

    int p32x_sh2_mem_is_rom(u32 a, SH2_DRC *sh2) { return 0; }

} // extern "C"

// Memory map tables for DRC
#define SH2_MAP_SIZE (1 << (32 - SH2_READ_SHIFT))
static uintptr_t sh2_drc_read8_map[SH2_MAP_SIZE * 2];
static uintptr_t sh2_drc_read16_map[SH2_MAP_SIZE * 2];
static uintptr_t sh2_drc_read32_map[SH2_MAP_SIZE * 2];
static const void *sh2_drc_write8_tab[SH2_MAP_SIZE];
static const void *sh2_drc_write16_tab[SH2_MAP_SIZE];
static const void *sh2_drc_write32_tab[SH2_MAP_SIZE];

static void sh2_drc_setup_mem_maps(SH2* sh2_ptr)
{
    printf("SH2DRC: Setting up memory maps (%d entries)...\n", SH2_MAP_SIZE);

    uintptr_t read8_half = ((uintptr_t)(void *)p32x_sh2_read8 >> 1) | 0x80000000u;
    uintptr_t read16_half = ((uintptr_t)(void *)p32x_sh2_read16 >> 1) | 0x80000000u;
    uintptr_t read32_half = ((uintptr_t)(void *)p32x_sh2_read32 >> 1) | 0x80000000u;

    for (int i = 0; i < SH2_MAP_SIZE; i++)
    {
        sh2_drc_read8_map[i * 2] = read8_half;
        sh2_drc_read8_map[i * 2 + 1] = 0xFFFFFFFF; // mask
        sh2_drc_read16_map[i * 2] = read16_half;
        sh2_drc_read16_map[i * 2 + 1] = 0xFFFFFFFF;
        sh2_drc_read32_map[i * 2] = read32_half;
        sh2_drc_read32_map[i * 2 + 1] = 0xFFFFFFFF;

        sh2_drc_write8_tab[i] = (const void *)p32x_sh2_write8;
        sh2_drc_write16_tab[i] = (const void *)p32x_sh2_write16;
        sh2_drc_write32_tab[i] = (const void *)p32x_sh2_write32;
    }

    sh2_ptr->read8_map = sh2_drc_read8_map;
    sh2_ptr->read16_map = sh2_drc_read16_map;
    sh2_ptr->read32_map = sh2_drc_read32_map;
    sh2_ptr->write8_tab = sh2_drc_write8_tab;
    sh2_ptr->write16_tab = sh2_drc_write16_tab;
    sh2_ptr->write32_tab = sh2_drc_write32_tab;
}

// IRQ Callback compatible with JIT
static int REGPARM(2) drc_irq_callback(SH2_DRC *ctx, int level)
{
    // FBA SH2 core uses (64 + level / 2) for external IRQ pins (IRQ0-IRQ7)
    if (level == 16) return 11; // NMI
    return 64 + level / 2;
}

// Initialize DRC
static bool drc_global_init_done = false;
static bool drc_cpu_init_done[2] = {false, false};

// Helper to setup JIT context pointers (maps, memory regions)
static void Sh2DrcSetupContext(SH2* sh2)
{
    // Maps
    sh2_drc_setup_mem_maps(sh2);

    // Memory regions
    sh2->p_bios  = pSh2Ext->MemMap[0x00000000 >> SH2_SHIFT];
    sh2->p_sdram = pSh2Ext->MemMap[0x02000000 >> SH2_SHIFT];
    sh2->p_rom   = pSh2Ext->MemMap[0x06000000 >> SH2_SHIFT];
    sh2->p_da    = sh2->data_array; 

    // Assist compiler.c single context optimization
    sh2_drc_ctx = (SH2_DRC*)sh2;
    printf("SH2DRC: Context pointers restored for CPU %d. sh2_drc_ctx=%p\n", sh2->is_slave, sh2_drc_ctx);
}

void Sh2DrcInit()
{
    SH2 *sh2 = &pSh2Ext->sh2; // Ensure local sh2 is valid
    if (!sh2) {
        printf("SH2DRC: FATAL: sh2 is NULL in Sh2DrcInit\n");
        return;
    }
    int cpu_idx = sh2->is_slave; // 0 or 1
    
    printf("SH2DRC: Sh2DrcInit called for cpu %d. pSh2Ext=%p\n", cpu_idx, pSh2Ext);

    if (drc_cpu_init_done[cpu_idx]) {
         printf("SH2DRC: CPU %d already initialized. sh2_drc_ctx=%p\n", cpu_idx, sh2_drc_ctx);
         // Ensure pointers are correct even if marked initialized
         Sh2DrcSetupContext(sh2);
         return;
    }

    if (!drc_global_init_done)
    {
        printf("SH2DRC: Initializing global DRC resources...\n");

        // Get the RWX memory block
        sceBlock = getVMBlock();
        
        // Init globals
        memset(&Pico32x, 0, sizeof(Pico32x));
        memset(&dummy_mem, 0, sizeof(dummy_mem));

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

        Pico32xMem = &dummy_mem;
        PicoIn.opt = POPT_EN_DRC; 
        
        drc_global_init_done = true;
    }

    printf("SH2DRC: Initializing context for CPU %d...\n", cpu_idx);

    // Setup function pointers
    sh2->irq_callback = (int (*)(struct SH2_ *, int))drc_irq_callback;

    // Setup context pointers
    Sh2DrcSetupContext(sh2);

    int ret = sh2_drc_init((SH2_DRC*)sh2);
    printf("SH2DRC: sh2_drc_init returned %d\n", ret);

    drc_cpu_init_done[cpu_idx] = true;
    sh2_drc_initialized = true; // For compatibility, though we rely on drc_cpu_init_done now
}

void Sh2DrcReset()
{
    if (!drc_global_init_done) return;
    printf("SH2DRC: Resetting DRC state (restoring pointers after memset)...\n");
    
    // Restore pointers cleared by Sh2Reset's memset
    SH2 *sh2 = &pSh2Ext->sh2;
    if (sh2) {
        Sh2DrcSetupContext(sh2);
        sh2->irq_callback = (int (*)(struct SH2_ *, int))drc_irq_callback;
    }

    sh2_drc_flush_all();
    // Don't reset regs here, FBA does it
}

INT32 Sh2RunDrc(INT32 cycles)
{
    SH2 *sh2 = &pSh2Ext->sh2; // Ensure local sh2 is valid
    Sh2DrcInit(); // Check per-cpu init
    
    // 1. Setup execution
    sh2->cycles_timeslice = cycles;
    sh2->icount = cycles; // Start count
    
    // Sync pending interrupts for DRC
    // FBA uses bitmask pending_irq. DRC needs pending_level.
    // Logic from FBA CHECK_PENDING_IRQ:
    int level = -1;
    if (sh2->pending_irq)
    {
        for (int i = 15; i >= 0; i--) {
            if (sh2->pending_irq & (1 << i)) {
                level = i;
                break;
            }
        }
    }
    // Check internal
    if (sh2->internal_irq_level > level)
        level = sh2->internal_irq_level;
        
    sh2->pending_level = level;
    sh2->pending_int_irq = sh2->internal_irq_level;
    sh2->pending_int_vector = sh2->internal_irq_vector;
    
    // 2. Execute
    int cycles_remaining = sh2_execute_drc((SH2_DRC*)sh2, cycles);
    
    // 3. Update total cycles (FBA)
    // cycles_remaining is what's left.
    // executed = cycles - cycles_remaining.
    // If JIT exited early (e.g. poll), cycles_remaining > 0.
    
    int executed = cycles - cycles_remaining;
    
    // Update FBA counters
    sh2->cycle_counts += executed;
    sh2->sh2_total_cycles += executed;
    
    // Check timers
    // We do this at block end.
    // The JIT execution might have called p32x_sh2_writeXX -> timer activated.
    // Timers might need servicing now.
    
    unsigned int cy = sh2->sh2_total_cycles; // Updated!
    if (sh2->dma_timer_active[0] && (cy - sh2->dma_timer_base[0]) >= sh2->dma_timer_cycles[0])
        sh2_dmac_callback(0);
    if (sh2->dma_timer_active[1] && (cy - sh2->dma_timer_base[1]) >= sh2->dma_timer_cycles[1])
        sh2_dmac_callback(1);
    if (sh2->timer_active && (cy - sh2->timer_base) >= sh2->timer_cycles)
        sh2_timer_callback();

    // Cleanup for FBA state consistency
    // So that sh2_GetTotalCycles returns accurate 'cycle_counts' between runs
    sh2->cycles_timeslice = 0;
    sh2->icount = 0;

    return executed;
}

void Sh2DrcExit()
{
    if (drc_global_init_done)
    {
        // Finish all contexts? We only have current sh2 pointer?
        // Ideally loop through 0 and 1, but we rely on FBA calling Sh2DrcExit for each CPU or once?
        // FBA calls Sh2Exit -> Sh2DrcExit. Sh2Exit is called per CPU?
        // Actually Sh2Exit in sh2_intf.cpp is usually called once or loops?
        // Assuming current sh2 is valid.
        
        sh2_drc_finish((SH2_DRC*)sh2);
        
        int idx = sh2->is_slave;
        drc_cpu_init_done[idx] = false;
        
        // If both done, clear global? 
        if (!drc_cpu_init_done[0] && !drc_cpu_init_done[1]) {
             drc_global_init_done = false;
        }
        
        sh2_drc_initialized = false;
    }
}

#endif // VITA
