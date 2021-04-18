#pragma once

#include <arm_neon.h>

struct Vuint8x16 {
    Vuint8x16() = default;
    Vuint8x16(int v) { operator=(v); }
    Vuint8x16(bool v) { operator=(v); }

    void load(const void *src) {
        values = vld1q_u8((const std::uint8_t*)src);
    }
    void store(void *dst) const {
        vst1q_u8((std::uint8_t*)dst, values);
    }

    void operator=(int v) {
        values = vdupq_n_u8(v);
    }

    void operator=(bool v) {
        values = vdupq_n_u8(v?~0ull:0ull);
    }

    Vuint8x16 operator&(const Vuint8x16 &rhs) const {
        Vuint8x16 res;
        res.values = vandq_u8(values, rhs.values);
        return res;
    }

    void operator&=(const Vuint8x16 &rhs) {
        values = vandq_u8(values, rhs.values);
    }

    void operator>>=(int v) {
        values = vshrq_n_u8(values, v);
    }

    Vuint8x16 operator~() const {
        Vuint8x16 res;
        res.values = vmvnq_u8(values);
        return res;
    }

    Vuint8x16 operator<=(const Vuint8x16 &rhs) const {
        Vuint8x16 res;
        res.values = vcleq_u8(values, rhs.values);
        return res;
    }
    Vuint8x16 operator>=(const Vuint8x16 &rhs) const {
        Vuint8x16 res;
        res.values = vcgeq_u8(values, rhs.values);
        return res;
    }
    Vuint8x16 operator>(const Vuint8x16 &rhs) const {
        Vuint8x16 res;
        res.values = vcgtq_u8(values, rhs.values);
        return res;
    }

    void shiftLeftInsert(const Vuint8x16 &insert, unsigned shiftAmount) {
        values = vsliq_n_u8(values, insert.values, shiftAmount);
    }

    template<unsigned offset>
    std::uint32_t extract32B() const {
        return vgetq_lane_u32((const uint32x4_t)values, offset);
    }


    uint8x16_t values;
};

struct Vuint16x8 {
    Vuint16x8() = default;
    Vuint16x8(int v) { operator=(v); }
    Vuint16x8(bool v) { operator=(v); }

    void operator=(int v) {
        values = vdupq_n_u16(v);
    }

    void operator=(bool v) {
        values = vdupq_n_u16(v?~0ull:0ull);
    }

    void operator&=(const Vuint16x8 &rhs) {
        values = vandq_u16(values, rhs.values);
    }

    Vuint16x8 bitTest(const Vuint16x8 &rhs) const {
        Vuint16x8 res;
        res.values = vtstq_u16(values, rhs.values);
        return res;
    }


    uint16x8_t values;
};


inline Vuint8x16 saturatingAdd(const Vuint8x16 &lhs, const Vuint8x16 &rhs) {
    Vuint8x16 res;
    res.values = vqaddq_u8(lhs.values, rhs.values);
    return res;
}


inline Vuint8x16 saturatingAdd(const Vuint8x16 &lhs, int rhs) {
    Vuint8x16 res;
    res.values = vqaddq_u8(lhs.values, vdupq_n_u8(rhs));
    return res;
}

inline Vuint8x16 saturatingSub(const Vuint8x16 &lhs, int rhs) {
    Vuint8x16 res;
    res.values = vqsubq_u8(lhs.values, vdupq_n_u8(rhs));
    return res;
}


inline void zip(const Vuint8x16 &a, const Vuint8x16 &b, Vuint16x8 &lower, Vuint16x8 &upper) {
    auto zipped = vzipq_u8(a.values, b.values);

    lower.values = (uint16x8_t&)zipped.val[0];
    upper.values = (uint16x8_t&)zipped.val[1];
}


inline Vuint8x16 unzipLower(const Vuint16x8 &a, const Vuint16x8 &b) {
    auto unzipped = vuzpq_u8((uint8x16_t&)a.values, (uint8x16_t&)b.values);
    Vuint8x16 res;
    res.values = unzipped.val[0];
    return res;
}


inline uint32_t is_not_zero(uint32x4_t v)
{
    uint32x2_t tmp = vorr_u32(vget_low_u32(v), vget_high_u32(v));
    return vget_lane_u32(vpmax_u32(tmp, tmp), 0);
}

inline bool any(const Vuint8x16 &v) {
    return is_not_zero((uint32x4_t)v.values);
}



inline Vuint8x16 absDiff(const Vuint8x16 &lhs, const Vuint8x16 &rhs) {
    Vuint8x16 res;
    res.values = vabdq_u8(lhs.values, rhs.values);
    return res;
}