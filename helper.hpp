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

#ifndef HELPER_HPP
#define HELPER_HPP

#include <limits>
#include <cstdint>

namespace anyplayer::helper
{

inline constexpr
std::int32_t float_to_signed(float x)
{
    // float -> 16bit int
    int v = x * (1 << 15);

    // check if out-of-bounds
    if (static_cast<std::uint32_t>(v) - static_cast<std::int32_t>(std::numeric_limits<std::int16_t>::min()) > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max()))
       v = v < 0 ? std::numeric_limits<std::int16_t>::min() : std::numeric_limits<std::int16_t>::max();

    // adapt 16bit int to 32bit int
    return v * (1 << 15);
}

inline constexpr
std::int32_t unsigned_to_signed(std::uint32_t uval, std::size_t bits)
{
    return (uval < (1U << (bits - 1))) ?
                static_cast<std::int32_t>(uval) :
               -static_cast<std::int32_t>((1 << bits) - uval);
}

inline constexpr
int ilog(int x)
{
    int result = 0;

    while (x > 0)
    {
        result++;
        x >>= 1;
    }

    return result;
}

inline constexpr
unsigned int reverse_bits(unsigned int n)
{
    n = ((n & 0xAAAAAAAA) >> 1) | ((n & 0x55555555) << 1);
    n = ((n & 0xCCCCCCCC) >> 2) | ((n & 0x33333333) << 2);
    n = ((n & 0xF0F0F0F0) >> 4) | ((n & 0x0F0F0F0F) << 4);
    n = ((n & 0xFF00FF00) >> 8) | ((n & 0x00FF00FF) << 8);

    return (n >> 16) | (n << 16);
}

}

#endif // HELPER_HPP
