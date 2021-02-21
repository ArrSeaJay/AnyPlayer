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

#ifndef ADLER32_HPP
#define ADLER32_HPP

#include <cstdint>

namespace anyplayer
{
template<typename It>
std::uint32_t adler_32(It begin, It end)
{
    std::uint32_t s1 = 1;
    std::uint32_t s2 = 0;

    while (begin != end)
    {
        s1 += static_cast<std::uint8_t>(*begin++);
        s1 %= 65521;
        s2 += s1;
        s2 %= 65521;
    }

    return (s2 << 16) | s1;
}
}

#endif // ADLER32_HPP
