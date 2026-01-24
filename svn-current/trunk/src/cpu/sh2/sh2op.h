/*
 * SH2 Opcode Jump Table
 * Auto-generated opcode dispatch table for SH2 interpreter
 * 
 * This file is included AFTER op0000-op1111 are defined.
 * It creates a 65536-entry jump table mapping each opcode to its handler.
 */

typedef void (*sh2_opfunc)(UINT16);

/* The 65536-entry jump table, indexed by full opcode */
static sh2_opfunc opcode_jumptable[65536];

/* Initialize the jump table by mapping each opcode to its group handler */
static void sh2_init_jumptable(void)
{
    static int initialized = 0;
    if (initialized) return;
    
    for (int i = 0; i < 65536; i++)
    {
        switch (i >> 12)
        {
            case  0: opcode_jumptable[i] = op0000; break;
            case  1: opcode_jumptable[i] = op0001; break;
            case  2: opcode_jumptable[i] = op0010; break;
            case  3: opcode_jumptable[i] = op0011; break;
            case  4: opcode_jumptable[i] = op0100; break;
            case  5: opcode_jumptable[i] = op0101; break;
            case  6: opcode_jumptable[i] = op0110; break;
            case  7: opcode_jumptable[i] = op0111; break;
            case  8: opcode_jumptable[i] = op1000; break;
            case  9: opcode_jumptable[i] = op1001; break;
            case 10: opcode_jumptable[i] = op1010; break;
            case 11: opcode_jumptable[i] = op1011; break;
            case 12: opcode_jumptable[i] = op1100; break;
            case 13: opcode_jumptable[i] = op1101; break;
            case 14: opcode_jumptable[i] = op1110; break;
            default: opcode_jumptable[i] = op1111; break;
        }
    }
    
    initialized = 1;
}
