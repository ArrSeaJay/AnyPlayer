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

#include "base64.hpp"

namespace
{
    char to_base_64(std::uint8_t b)
    {
        if (b < 26) return 'A' + b;
        if (b < 52) return 'a' + (b - 26);
        if (b < 62) return '0' + (b - 52);
        if (b == 62) return '+';
        if (b == 63) return '/';
        return '=';
    }
}

std::string to_base_64(const std::vector<uint8_t>& data)
{
    int n = data.size();

    int fill = n % 3;

    std::string base64;
    base64.reserve(4 * (n + 2 - ((n + 2) % 3)) / 3);

    int i = 0;
    while (i + fill < n)
    {
        std::uint8_t a = data[i++];
        std::uint8_t b = data[i++];
        std::uint8_t c = data[i++];

        char A = to_base_64(a >> 2);
        char B = to_base_64(((a & 3) << 6) | (b >> 4));
        char C = to_base_64(((b & 15) << 4) | (c >> 6));
        char D = to_base_64(c & 63);

        base64.push_back(A);
        base64.push_back(B);
        base64.push_back(C);
        base64.push_back(D);
    }

    if (fill > 0)
    {
        std::uint8_t a = data[i++];
        std::uint8_t b = fill == 1 ? data[i++] : 0;

        char A = to_base_64(a >> 2);
        char B = to_base_64(((a & 3) << 6) | (b >> 4));
        char C = fill == 1 ? to_base_64(((b & 15) << 4)) : '=';
        char D = '=';

        base64.push_back(A);
        base64.push_back(B);
        base64.push_back(C);
        base64.push_back(D);
    }

    return base64;
}
