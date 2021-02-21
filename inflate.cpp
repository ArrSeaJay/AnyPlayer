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

#include "inflate.hpp"

#include "huffman.hpp"
#include <vector>

namespace anyplayer
{

namespace helper
{

huffman<int> create_huffman_tree(std::vector<int> lengths)
{
    return create_huffman_tree(lengths.begin(), lengths.end());
}

std::vector<int> prepare_value_lengths()
{
    std::vector<int> vl(288, 8);

    for (int i = 144; i < 256; ++i)
    {
        vl[i] = 9;
    }
    for (int i = 256; i < 280; ++i)
    {
        vl[i] = 7;
    }

    return vl;
}

std::vector<int> prepare_distances()
{
    return std::vector<int>(32, 5);
}

const huffman<int> fixedValueLengthTree = create_huffman_tree(prepare_value_lengths());
const huffman<int> fixedDistanceTree = create_huffman_tree(prepare_distances());

const std::vector<int> lengthBase
{
    3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258
};

const std::vector<int> distanceBase
{
    1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577
};

const std::vector<int> codeLengthAlphabetIndices
{
    16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15
};

}

}
