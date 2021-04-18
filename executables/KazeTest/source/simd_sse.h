#pragma once

#include <cstdint>
#include <immintrin.h>

struct Vuint8x16 {
    Vuint8x16() = default;
    Vuint8x16(int v) { operator=(v); }
    Vuint8x16(bool v) { operator=(v); }

    void load(const void *src) {
        values = _mm_loadu_si128((__m128i const*)src);
    }
    void store(void *dst) const {
        _mm_storeu_si128((__m128i*)dst, values);
    }

    void operator=(int v) {
        values = _mm_set1_epi8(v);
    }

    void operator=(bool v) {
        values = _mm_set1_epi8(v?~0ull:0ull);
    }

    Vuint8x16 operator&(const Vuint8x16 &rhs) const {
        Vuint8x16 res;
        res.values = _mm_and_si128(values, rhs.values);
        return res;
    }

    void operator&=(const Vuint8x16 &rhs) {
        values = _mm_and_si128(values, rhs.values);
    }

    void operator|=(const Vuint8x16 &rhs) {
        values = _mm_or_si128(values, rhs.values);
    }

    void operator>>=(int v) {
        values = _mm_srli_epi16(values, v);
        values = _mm_and_si128(values,  _mm_set1_epi8(255u >> v));
    }

    Vuint8x16 operator~() const {
        Vuint8x16 res;
        res.values = _mm_andnot_si128(values,  _mm_set1_epi8(255u));
        return res;
    }

    Vuint8x16 operator<=(const Vuint8x16 &rhs) const {
        Vuint8x16 res;
        res.values = _mm_or_si128(_mm_cmplt_epi8(values, rhs.values),
            _mm_cmpeq_epi8(values, rhs.values));
        return res;
    }
    Vuint8x16 operator>=(const Vuint8x16 &rhs) const {
        Vuint8x16 res;
        res.values = _mm_or_si128(_mm_cmpgt_epi8(values, rhs.values),
            _mm_cmpeq_epi8(values, rhs.values));
        return res;
    }
    Vuint8x16 operator>(const Vuint8x16 &rhs) const {
        Vuint8x16 res;
        res.values = _mm_cmpgt_epi8(values, rhs.values);
        return res;
    }

    template<unsigned offset>
    std::uint32_t extract32B() const {
        return _mm_extract_epi32(values, offset);
    }
    __m128i values;
};

struct Vuint16x8 {
    Vuint16x8() = default;
    Vuint16x8(int v) { operator=(v); }
    Vuint16x8(bool v) { operator=(v); }

    void operator=(int v) {
        values = _mm_set1_epi16(v);
    }

    void operator=(bool v) {
        values = _mm_set1_epi16(v?~0ull:0ull);
    }

    void operator&=(const Vuint16x8 &rhs) {
        values = _mm_and_si128(values, rhs.values);
    }

    Vuint16x8 bitTest(const Vuint16x8 &rhs) const {
        Vuint16x8 res;
        res.values = _mm_cmpgt_epi16(_mm_and_si128(values, rhs.values),
            _mm_setzero_si128());
        return res;
    }
    
    __m128i values;
};


inline Vuint8x16 saturatingAdd(const Vuint8x16 &lhs, const Vuint8x16 &rhs) {
    Vuint8x16 res;
    res.values = _mm_adds_epu8(lhs.values, rhs.values);
    return res;
}

inline Vuint8x16 saturatingAdd(const Vuint8x16 &lhs, int rhs) {
    Vuint8x16 res;
    res.values = _mm_adds_epu8(lhs.values, _mm_set1_epi8( (char)rhs));
    return res;
}

inline Vuint8x16 saturatingSub(const Vuint8x16 &lhs, int rhs) {
    Vuint8x16 res;
    res.values = _mm_subs_epu8(lhs.values, _mm_set1_epi8( (char)rhs));
    return res;
}

inline void zip(const Vuint8x16 &a, const Vuint8x16 &b, 
        Vuint16x8 &lower, Vuint16x8 &upper) {
    lower.values = _mm_unpacklo_epi8(a.values, b.values);
    upper.values = _mm_unpackhi_epi8(a.values, b.values);
}

inline Vuint8x16 unzipLower(const Vuint16x8 &a, const Vuint16x8 &b) {
    Vuint8x16 res;
    res.values = _mm_packus_epi16(a.values,b.values);
    return res;
}

inline bool any(const Vuint8x16 &v) {
    return _mm_testz_si128(v.values,v.values);
}

inline Vuint8x16 absDiff(const Vuint8x16 &lhs, const Vuint8x16 &rhs) {
    Vuint8x16 res;

    const __m128i ab = _mm_subs_epu8(lhs.values, rhs.values);
    const __m128i ba = _mm_subs_epu8(rhs.values, lhs.values);
    res.values = _mm_or_si128(ab, ba);

    return res;
}