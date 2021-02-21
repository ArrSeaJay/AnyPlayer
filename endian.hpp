//    AnyPlayer - Demonstration of how a FLAC and OggVorbis decoder works
//    Copyright (C) 2021 Ralf-Christian Jürgensen

//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.

//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef ENDIAN_HPP
#define ENDIAN_HPP

#include <cstdint>

namespace anyplayer
{

inline std::uint8_t swap(std::uint8_t x)
{
    return x;
}

inline std::uint16_t swap(std::uint16_t x)
{
#if defined(__GNUC__) && defined(__i386__)
    asm("xchgb %b0,%h0": "=Q"(x):"0"(x));
    return x;
#elif defined(__GNUC__) && defined(__x86_64__)
    asm("xchgb %b0,%h0": "=Q"(x):"0"(x));
    return x;
#else
    return (x << 8) | (x >> 8);
#endif
}

inline std::uint32_t swap(std::uint32_t x)
{
#if defined(__GNUC__) && defined(__i386__)
    asm("bswap %0": "=r"(x):"0"(x));
    return x;
#elif defined(__GNUC__) && defined(__x86_64__)
    asm("bswapl %0": "=r"(x):"0"(x));
    return x;
#else
    return ((x << 24) | ((x << 8) & 0x00FF0000) | ((x >> 8) & 0x0000FF00) | (x >> 24));
#endif
}

inline std::uint64_t swap(std::uint64_t x)
{
#if defined(__GNUC__) && defined(__i386__)
    union
    {
        struct
        {
            std::uint32_t a, b;
        } s;
        std::uint64_t u;
    } v;
    v.u = x;
    asm("bswapl %0 ; bswapl %1 ; xchgl %0,%1": "=r"(v.s.a), "=r"(v.s.b):"0"(v.s.a), "1"(v.s.b));
    return v.u;
#elif defined(__GNUC__) && defined(__x86_64__)
    asm("bswapq %0": "=r"(x):"0"(x));
    return x;
#else
    std::uint32_t lo = x & 0xFFFFFFFF;
    std::uint32_t hi = (x >> 32) & 0xFFFFFFFF;
    x = swap(lo);
    x <<= 32;
    x |= swap(hi);
    return x;
#endif
}

template<class T>
inline extern T swap(const std::uint8_t* p)
{
    return swap(*reinterpret_cast<const T*>(p));
}

// currently this is compiled on LE only!
#define TARGET_LE
#ifdef TARGET_LE

template<class T>
T swap_le(T v) { return v; }

template<class T>
T swap_be(T v) { return swap(v); }

template<class T>
T swap_le(const std::uint8_t* p)
{
    return *reinterpret_cast<const T*>(p);
}

template<class T>
T swap_be(const std::uint8_t* p)
{
    return swap<T>(p);
}

#else

template<class T>
T swap_le(T v) { return swap(v); }

template<class T>
T swap_be(T v) { return v; }

template<class T>
T swap_le(const std::uint8_t* p)
{
    return swap<T>(p);
}

template<class T>
T swap_be(const std::uint8_t* p)
{
    return *reinterpret_cast<const T*>(p);
}

#endif

}

#endif // ENDIAN_HPP
