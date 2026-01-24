#ifndef CPS3_NEON_H
#define CPS3_NEON_H

#ifdef VITA
#include <arm_neon.h>

// NEON optimized drawer for case 0 (Opaque) - 1:1 scale
static void cps3_drawgfxzoom_2_neon_opaque(
    UINT8 *source, UINT32 *dest, INT32 width, UINT32 pal)
{
    // Process 16 pixels at a time
    int i = 0;
    
    // Create vector for palette (splat high bits)
    // Destination format: index | palette
    // But logic is: if (src) dest = src | pal
    // pal is usually like 0x8000 | (color << something) ?
    // No, pal contains the high bits for the color index.
    
    uint32x4_t v_pal = vdupq_n_u32(pal);
    
    // Main loop 16 pixels
    for (; i <= width - 16; i += 16) {
        // Load 16 source bytes (indexes)
        uint8x16_t v_src = vld1q_u8(&source[i]);
        
        // Expand 8-bit src to 16-bit
        uint8x8_t v_src_low = vget_low_u8(v_src);
        uint8x8_t v_src_high = vget_high_u8(v_src);
        
        uint16x8_t v_src16_low = vmovl_u8(v_src_low);
        uint16x8_t v_src16_high = vmovl_u8(v_src_high);
        
        // Expand 16-bit to 32-bit (destination format is UINT32*)
        uint32x4_t v_src32_0 = vmovl_u16(vget_low_u16(v_src16_low));
        uint32x4_t v_src32_1 = vmovl_u16(vget_high_u16(v_src16_low));
        uint32x4_t v_src32_2 = vmovl_u16(vget_low_u16(v_src16_high));
        uint32x4_t v_src32_3 = vmovl_u16(vget_high_u16(v_src16_high));
        
        // Check for 0 (transparency) on 32-bit values to get proper 32-bit mask (0xFFFFFFFF)
        uint32x4_t v_zero32 = vdupq_n_u32(0);
        uint32x4_t v_mask32_0 = vceqq_u32(v_src32_0, v_zero32);
        uint32x4_t v_mask32_1 = vceqq_u32(v_src32_1, v_zero32);
        uint32x4_t v_mask32_2 = vceqq_u32(v_src32_2, v_zero32);
        uint32x4_t v_mask32_3 = vceqq_u32(v_src32_3, v_zero32);
        
        // Combine with palette: src | pal
        uint32x4_t v_res_0 = vorrq_u32(v_src32_0, v_pal);
        uint32x4_t v_res_1 = vorrq_u32(v_src32_1, v_pal);
        uint32x4_t v_res_2 = vorrq_u32(v_src32_2, v_pal);
        uint32x4_t v_res_3 = vorrq_u32(v_src32_3, v_pal);
        
        // Load current destination (to keep pixels where mask is 0xFFFFFFFF)
        uint32x4_t v_dst_0 = vld1q_u32(&dest[i + 0]);
        uint32x4_t v_dst_1 = vld1q_u32(&dest[i + 4]);
        uint32x4_t v_dst_2 = vld1q_u32(&dest[i + 8]);
        uint32x4_t v_dst_3 = vld1q_u32(&dest[i + 12]);
        
        // Select: if mask is 1 (src==0), keep dest. Else write result.
        // vbsl(mask, a, b) selects bits from 'a' where mask is 1, 'b' where mask is 0
        v_res_0 = vbslq_u32(v_mask32_0, v_dst_0, v_res_0);
        v_res_1 = vbslq_u32(v_mask32_1, v_dst_1, v_res_1);
        v_res_2 = vbslq_u32(v_mask32_2, v_dst_2, v_res_2);
        v_res_3 = vbslq_u32(v_mask32_3, v_dst_3, v_res_3);
        
        // Store result
        vst1q_u32(&dest[i + 0], v_res_0);
        vst1q_u32(&dest[i + 4], v_res_1);
        vst1q_u32(&dest[i + 8], v_res_2);
        vst1q_u32(&dest[i + 12], v_res_3);
    }
    
    // Handle remaining pixels
    for (; i < width; i++) {
        UINT8 c = source[i];
        if (c) dest[i] = pal | c;
    }
}

// NEON optimized drawer for case 6 (Alpha/Shadow) - 1:1 scale
// dest[x] |= ((c&0x0000f) << 13);
static void cps3_drawgfxzoom_2_neon_alpha6(
    UINT8 *source, UINT32 *dest, INT32 width)
{
    int i = 0;
    uint8x16_t v_mask_f = vdupq_n_u8(0x0F);
    
    for (; i <= width - 16; i += 16) {
        // Load 16 source bytes
        uint8x16_t v_src = vld1q_u8(&source[i]);
        
        // Apply mask 0x0F
        v_src = vandq_u8(v_src, v_mask_f);
        
        // Expand 8-bit to 16-bit
        uint16x8_t v_src16_low = vmovl_u8(vget_low_u8(v_src));
        uint16x8_t v_src16_high = vmovl_u8(vget_high_u8(v_src));
        
        // Expand to 32-bit
        uint32x4_t v_src32_0 = vmovl_u16(vget_low_u16(v_src16_low));
        uint32x4_t v_src32_1 = vmovl_u16(vget_high_u16(v_src16_low));
        uint32x4_t v_src32_2 = vmovl_u16(vget_low_u16(v_src16_high));
        uint32x4_t v_src32_3 = vmovl_u16(vget_high_u16(v_src16_high));
        
        // Shift left 13
        v_src32_0 = vshlq_n_u32(v_src32_0, 13);
        v_src32_1 = vshlq_n_u32(v_src32_1, 13);
        v_src32_2 = vshlq_n_u32(v_src32_2, 13);
        v_src32_3 = vshlq_n_u32(v_src32_3, 13);
        
        // Load, OR, Store
        uint32x4_t v_dst_0 = vld1q_u32(&dest[i+0]);
        uint32x4_t v_dst_1 = vld1q_u32(&dest[i+4]);
        uint32x4_t v_dst_2 = vld1q_u32(&dest[i+8]);
        uint32x4_t v_dst_3 = vld1q_u32(&dest[i+12]);
        
        vst1q_u32(&dest[i+0], vorrq_u32(v_dst_0, v_src32_0));
        vst1q_u32(&dest[i+4], vorrq_u32(v_dst_1, v_src32_1));
        vst1q_u32(&dest[i+8], vorrq_u32(v_dst_2, v_src32_2));
        vst1q_u32(&dest[i+12], vorrq_u32(v_dst_3, v_src32_3));
    }
    
    // Handle remaining
    for (; i < width; i++) {
        UINT8 c = source[i];
        dest[i] |= ((c&0x0000f) << 13);
    }
}

// NEON optimized drawer for zoomed opaque sprites (Manual Gather)
// src/dst ops are same as opaque 1:1, but src is gathered via x_index logic
static void cps3_drawgfxzoom_2_neon_zoom_opaque(
    UINT8 *source_base_line, UINT32 *dest, INT32 width, UINT32 pal, INT32 x_index, INT32 dx)
{
    int i = 0;
    uint32x4_t v_pal = vdupq_n_u32(pal);
    uint32x4_t v_zero32 = vdupq_n_u32(0);
    uint8_t buffer[8];
    
    // Process 8 pixels at a time
    for (; i <= width - 8; i += 8) {
        // Manual gather
        buffer[0] = source_base_line[x_index >> 16]; x_index += dx;
        buffer[1] = source_base_line[x_index >> 16]; x_index += dx;
        buffer[2] = source_base_line[x_index >> 16]; x_index += dx;
        buffer[3] = source_base_line[x_index >> 16]; x_index += dx;
        buffer[4] = source_base_line[x_index >> 16]; x_index += dx;
        buffer[5] = source_base_line[x_index >> 16]; x_index += dx;
        buffer[6] = source_base_line[x_index >> 16]; x_index += dx;
        buffer[7] = source_base_line[x_index >> 16]; x_index += dx;
        
        // Load gathered pixels
        uint8x8_t v_src = vld1_u8(buffer);
        
        // Expand to 16-bit
        uint16x8_t v_src16 = vmovl_u8(v_src);
        
        // Expand to 32-bit (low/high)
        uint32x4_t v_src32_0 = vmovl_u16(vget_low_u16(v_src16));
        uint32x4_t v_src32_1 = vmovl_u16(vget_high_u16(v_src16));
        
        // Mask generation (check full 32-bit for 0xFFFFFFFF mask)
        uint32x4_t v_mask32_0 = vceqq_u32(v_src32_0, v_zero32);
        uint32x4_t v_mask32_1 = vceqq_u32(v_src32_1, v_zero32);
        
        // Combine with palette
        uint32x4_t v_res_0 = vorrq_u32(v_src32_0, v_pal);
        uint32x4_t v_res_1 = vorrq_u32(v_src32_1, v_pal);
        
        // Load destination
        uint32x4_t v_dst_0 = vld1q_u32(&dest[i]);
        uint32x4_t v_dst_1 = vld1q_u32(&dest[i + 4]);
        
        // Select
        v_res_0 = vbslq_u32(v_mask32_0, v_dst_0, v_res_0);
        v_res_1 = vbslq_u32(v_mask32_1, v_dst_1, v_res_1);
        
        // Store
        vst1q_u32(&dest[i], v_res_0);
        vst1q_u32(&dest[i + 4], v_res_1);
    }
    
    // Handle remaining pixels scalar
    for (; i < width; i++) {
        UINT8 c = source_base_line[x_index >> 16];
        if (c) dest[i] = pal | c;
        x_index += dx;
    }
}
#endif // VITA

#endif // CPS3_NEON_H
