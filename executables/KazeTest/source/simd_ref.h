#pragma once

#include <array>
#include <cstdint>
#include <cstring>

template<typename type, unsigned dim>
struct Vuint {
    Vuint() = default;
    Vuint(int v) { operator=(v); }
    Vuint(bool v) { operator=(v); }

    void load(const type *src) {
        for (unsigned i = 0; i < dim; i++)
            values[i] = src[i];
    }
    void store(type *dst) const {
        for (unsigned i = 0; i < dim; i++)
            dst[i] = values[i];
    }

    void operator=(int v) {
        for (unsigned i = 0; i < dim; i++)
            values[i] = v;
    }

    void operator=(bool v) {
        for (unsigned i = 0; i < dim; i++)
            values[i] = v?~0ull:0ull;
    }

    Vuint<type, dim> operator+(int i) const {
        auto res = *this;
        for (auto &v : res.values) v += i;
        return res;
    }
    Vuint<type, dim> operator-(int i) const {
        auto res = *this;
        for (auto &v : res.values) v -= i;
        return res;
    }
    Vuint<type, dim> operator+(const Vuint<type, dim> &rhs) const {
        Vuint<type, dim> res;
        for (unsigned i = 0; i < dim; i++)
            res.values[i] = values[i] + rhs.values[i];
        return res;
    }
    Vuint<type, dim> operator-(const Vuint<type, dim> &rhs) const {
        Vuint<type, dim> res;
        for (unsigned i = 0; i < dim; i++)
            res.values[i] = values[i] - rhs.values[i];
        return res;
    }

    Vuint<type, dim> operator&(const Vuint<type, dim> &rhs) const {
        Vuint<type, dim> res;
        for (unsigned i = 0; i < dim; i++)
            res.values[i] = values[i] & rhs.values[i];
        return res;
    }

    Vuint<type, dim> operator|(const Vuint<type, dim> &rhs) const {
        Vuint<type, dim> res;
        for (unsigned i = 0; i < dim; i++)
            res.values[i] = values[i] | rhs.values[i];
        return res;
    }

    void operator&=(const Vuint<type, dim> &rhs) {
        for (unsigned i = 0; i < dim; i++)
            values[i] &= rhs.values[i];
    }

    void operator|=(const Vuint<type, dim> &rhs) {
        for (unsigned i = 0; i < dim; i++)
            values[i] |= rhs.values[i];
    }

    void operator>>=(int v) {
        for (unsigned i = 0; i < dim; i++)
            values[i] >>= v;
    }

    Vuint<type, dim> operator~() const {
        Vuint<type, dim> res;
        for (unsigned i = 0; i < dim; i++)
            res.values[i] = ~values[i];
        return res;
    }

    Vuint<type, dim> operator<=(const Vuint<type, dim> &rhs) const {
        Vuint<type, dim> res;
        for (unsigned i = 0; i < dim; i++)
            res.values[i] = (values[i] <= rhs.values[i])?~0ull:0ull;
        return res;
    }
    Vuint<type, dim> operator>=(const Vuint<type, dim> &rhs) const {
        Vuint<type, dim> res;
        for (unsigned i = 0; i < dim; i++)
            res.values[i] = (values[i] >= rhs.values[i])?~0ull:0ull;
        return res;
    }
    Vuint<type, dim> operator>(const Vuint<type, dim> &rhs) const {
        Vuint<type, dim> res;
        for (unsigned i = 0; i < dim; i++)
            res.values[i] = (values[i] > rhs.values[i])?~0ull:0ull;
        return res;
    }

    void shiftLeftInsert(const Vuint<type, dim> &insert, unsigned shiftAmount) {
        for (unsigned i = 0; i < dim; i++) {
            type shifted = insert.values[i] << shiftAmount;
            type mask = ~(~0ull << shiftAmount);
            values[i] = (values[i] & mask) | shifted;
        }
    }

    Vuint<type, dim> bitTest(const Vuint<type, dim> &rhs) const {
        Vuint<type, dim> res;
        for (unsigned i = 0; i < dim; i++)
            res.values[i] = (values[i] & rhs.values[i])?~0ull:0ull;
        return res;
    }

    template<unsigned offset>
    std::uint32_t extract32B() const {
        #if 0
            const std::uint32_t *ptr = (const std::uint32_t *)&values;
            return ptr[offset];
        #else
            std::uint32_t res;
            memcpy(&res, ((const char*)values.data()) + offset * 4, 4);
            return res;
        #endif
    }

    std::array<type, dim> values;
};


template<typename type, unsigned dim>
Vuint<type, dim> saturatingAdd(const Vuint<type, dim> &lhs, const Vuint<type, dim> &rhs) {
    Vuint<type, dim> res;
    for (unsigned i = 0; i < dim; i++)
        res.values[i] = std::min<std::int64_t>((std::int64_t) lhs.values[i] + (std::int64_t)rhs.values[i], std::numeric_limits<type>::max());
    return res;
}

template<typename type, unsigned dim>
Vuint<type, dim> saturatingSub(const Vuint<type, dim> &lhs, const Vuint<type, dim> &rhs) {
    Vuint<type, dim> res;
    for (unsigned i = 0; i < dim; i++)
        res.values[i] = std::max<std::int64_t>((std::int64_t) lhs.values[i] - (std::int64_t)rhs.values[i], std::numeric_limits<type>::min());
    return res;
}

template<typename type, unsigned dim>
Vuint<type, dim> saturatingAdd(const Vuint<type, dim> &lhs, int rhs) {
    Vuint<type, dim> res;
    for (unsigned i = 0; i < dim; i++)
        res.values[i] = std::min<std::int64_t>((std::int64_t) lhs.values[i] + rhs, std::numeric_limits<type>::max());
    return res;
}

template<typename type, unsigned dim>
Vuint<type, dim> saturatingSub(const Vuint<type, dim> &lhs, int rhs) {
    Vuint<type, dim> res;
    for (unsigned i = 0; i < dim; i++)
        res.values[i] = std::max<std::int64_t>((std::int64_t) lhs.values[i] - rhs, std::numeric_limits<type>::min());
    return res;
}


template<typename type, unsigned dim>
Vuint<type, dim> absDiff(const Vuint<type, dim> &lhs, const Vuint<type, dim> &rhs) {
    Vuint<type, dim> res;
    for (unsigned i = 0; i < dim; i++)
        if (lhs.values[i] > rhs.values[i])
            res.values[i] = lhs.values[i] - rhs.values[i];
        else
            res.values[i] = rhs.values[i] - lhs.values[i];
    return res;
}



using Vuint8x16 = Vuint<std::uint8_t, 16>;
using Vuint16x8 = Vuint<std::uint16_t, 8>;

template<class Functor>
void unroll8times(Functor functor) {
    functor(0);
    functor(1);
    functor(2);
    functor(3);
    functor(4);
    functor(5);
    functor(6);
    functor(7);
}


inline void zip(const Vuint8x16 &a, const Vuint8x16 &b, Vuint16x8 &lower, Vuint16x8 &upper) {
    for (unsigned i = 0; i < 8; i++)
        lower.values[i] = (((std::uint16_t) a.values[i]) << 8) | b.values[i];
#if 1
    for (unsigned i = 0; i < 8; i++){
//        std::cout << "i " << i << std::endl;
        upper.values[i] = (((std::uint16_t) a.values[8+i]) << 8) | b.values[8+i];
    }
#else
    unroll8times([&](unsigned i){
        upper.values[i] = (((std::uint16_t) a.values[8+i]) << 8) | b.values[8+i];
    });
#endif
}

inline  Vuint8x16 unzipLower(const Vuint16x8 &a, const Vuint16x8 &b) {
    Vuint8x16 res;
    for (unsigned i = 0; i < 8; i++){
        res.values[i] = a.values[i] & 0xFF;
    }

    for (unsigned i = 0; i < 8; i++){
        res.values[8+i] = b.values[i] & 0xFF;
    }
    return res;
}

inline bool any(const Vuint8x16 &v) {
    bool res = false;
    for (unsigned i = 0; i < 16; i++)
        res |= v.values[i];
    return res;
}

