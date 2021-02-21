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

#ifndef CRC_HPP
#define CRC_HPP

#include <cstdint>
#include <array>
#include "meta.hpp"

namespace anyplayer
{

template<std::size_t Bits, std::uint32_t Poly>
class crc
{
public:
    typedef
    typename std::conditional<Bits == 8, std::uint8_t,
    typename std::conditional<Bits == 16, std::uint16_t,
    typename std::conditional<Bits == 32, std::uint32_t,
    void
    >::type>::type>::type T;

private:
    typedef
    typename std::conditional<Bits == 8, std::uint8_t,
    typename std::conditional<Bits == 16, std::uint32_t,
    typename std::conditional<Bits == 32, std::uint64_t,
    void
    >::type>::type>::type U;

    static constexpr U mask = static_cast<U>(1) << (Bits - 1);

    template<unsigned int j, class U, U entry>
    struct Body
    {
        static constexpr U value = Body<j - 1, U, (entry & mask) == mask ? static_cast<U>((entry << 1) ^ Poly) : entry << 1>::value;
    };

    template<class U, U entry>
    struct Body<0, U, entry>
    {
        static constexpr U value = entry;
    };

    template<std::size_t N>
    struct Func
    {
        static constexpr T value = static_cast<T>(Body<8, U, N << (Bits - 8)>::value);
    };

    typedef typename generate_array<T, 256, Func>::result table;

public:
    template<typename It>
    T hash(It begin, It end, T seed)
    {
        T crc = seed;

        while (begin != end)
        {
            crc = static_cast<T>((crc << 8) ^ table::data[(*begin++ ^ (crc >> (Bits - 8))) & 0xff]);
        }

        return crc;
    }
};

template<std::size_t Bits, std::uint32_t Poly>
class reverse_crc
{
public:
    typedef
    typename std::conditional<Bits == 8, std::uint8_t,
    typename std::conditional<Bits == 16, std::uint16_t,
    typename std::conditional<Bits == 32, std::uint32_t,
    void
    >::type>::type>::type T;

private:
    typedef
    typename std::conditional<Bits == 8, std::uint8_t,
    typename std::conditional<Bits == 16, std::uint32_t,
    typename std::conditional<Bits == 32, std::uint64_t,
    void
    >::type>::type>::type U;

    template<unsigned int j, class U, U entry>
    struct Body
    {
        static constexpr U value = Body<j - 1, U, (entry & 1) == 1 ? static_cast<U>((entry >> 1) ^ Poly) : (entry >> 1)>::value;
    };

    template<class U, U entry>
    struct Body<0, U, entry>
    {
        static constexpr U value = entry;
    };

    template<std::size_t N>
    struct Func
    {
        static constexpr T value = static_cast<T>(Body<8, U, N>::value);
    };

    typedef typename generate_array<T, 256, Func>::result table;

public:
    template<typename It>
    T hash(It begin, It end, T seed)
    {
        T crc = seed;

        for (It i = begin; i != end; ++i)
        {
            crc = static_cast<T>((crc >> 8) ^ table::data[(*i ^ crc) & 0xff]);
        }

        return crc;
    }
};

}

#endif // CRC_HPP
