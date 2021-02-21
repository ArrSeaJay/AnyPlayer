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

#ifndef DEFLATE_HPP
#define DEFLATE_HPP

#include <vector>
#include <cstdint>
#include <algorithm>
#include <iterator>
#include <vector>
#include <tuple>
#include <iostream>

#include "huffman.hpp"
#include "bititerator.hpp"

namespace anyplayer
{

namespace helper
{

extern const huffman<int> fixedValueLengthTree;
extern const huffman<int> fixedDistanceTree;

extern const std::vector<int> lengthBase;
extern const std::vector<int> distanceBase;
extern const std::vector<int> codeLengthAlphabetIndices;

template<typename Iterator>
huffman<int> create_huffman_tree(Iterator begin, Iterator end)
{
    auto maxBits = *std::max_element(begin, end);

    std::vector<int> bl_count(maxBits + 1);
    for (auto i = begin; i != end; ++i)
    {
        bl_count[*i]++;
    }

    std::vector<int> next_code { 0 };
    next_code.reserve(bl_count.size() + 1);
    int code = 0;
    bl_count[0] = 0;
    for (size_t bits = 1; bits <= bl_count.size(); ++bits)
    {
        code = (code + bl_count[bits-1]) << 1;
        next_code.push_back(code);
    }

    huffman<int> root;

    for (int n = 0; begin != end; ++n, ++begin)
    {
        int len = *begin;
        if (len != 0)
        {
            int c = next_code[len]++;

            auto* node = &root;
            int depth = len - 1;
            while (depth != 0)
            {
                bool bit = ((c >> depth) & 1) == 1;
                auto tmp = node->get_branch(bit);
                if (tmp == nullptr)
                    node = node->append(bit);
                else
                    node = tmp;
                --depth;
            }
            node->append((c & 1) == 1, n);
        }
    }

    return root;
}

template<typename Iterator>
int get_length(bit_iterator<Iterator, false>& it, int value)
{
    if (value > 285)
        throw std::runtime_error("");

    int extra;
    if (value < 261 || value == 285)
        extra = 0;
    else
        extra = ((value - 257) >> 2) - 1;

    auto i = read_int(it, extra);
    return lengthBase[value - 257] + i;
}

template<typename Iterator>
int decode_distance(bit_iterator<Iterator, false>& it, const huffman<int>& huffman)
{
    auto value = huffman.decode([&]() { return *it++; }).value();

    if (value > 29)
        throw std::runtime_error("");

    int extra = 0;
    if (value > 1)
        extra = (value >> 1) - 1;

    auto i = read_int(it, extra);
    return distanceBase[value] + i;
}

template<typename Iterator>
std::vector<int> decode_code_lengths(int size, bit_iterator<Iterator, false>& it, const huffman<int>& tree)
{
    std::vector<int> lengths(size);

    for (int i = 0; i < size; )
    {
        auto value = tree.decode([&]() { return *it++; }).value();
        if (value < 16)
        {
            lengths[i++] = value;
        }
        else if (value == 16)
        {
            int count = read_int<2>(it) + 3;
            auto prev = lengths[i - 1];
            for (int j = 0; j < count; ++j)
            {
                lengths[i++] = prev;
            }
        }
        else if (value == 17)
        {
            int count = read_int<3>(it) + 3;
            i += count;
        }
        else if (value == 18)
        {
            int count = read_int<7>(it) + 11;
            i += count;
        }
    }

    return lengths;
}

template<typename Iterator>
std::tuple<huffman<int>, huffman<int> > read_huffman_tables(bit_iterator<Iterator, false>& it)
{
    auto hlit = read_int<5>(it) + 257;
    auto hdist = read_int<5>(it) + 1;
    auto hclen = read_int<4>(it) + 4;

    std::vector<int> codeLengthAlphabet(codeLengthAlphabetIndices.size());
    for (int i = 0; i < hclen; ++i)
    {
        auto l = read_int<3>(it);
        auto j = codeLengthAlphabetIndices[i];
        codeLengthAlphabet[j] = l;
    }
    auto codeLengthsTree = create_huffman_tree(codeLengthAlphabet.begin(), codeLengthAlphabet.end());

    auto lengths = decode_code_lengths(hlit + hdist, it, codeLengthsTree);

    auto valueLenghtsTree = create_huffman_tree(lengths.begin(), lengths.begin() + hlit);
    auto distancesTree = create_huffman_tree(lengths.begin() + hlit, lengths.end());

    return std::make_tuple(std::move(valueLenghtsTree), std::move(distancesTree));
}

}

template<typename Iterator>
std::vector<std::uint8_t> inflate(bit_iterator<Iterator, false>& bitstream, std::size_t maxbits, std::size_t sizehint)
{
    auto begin = bitstream;

    std::vector<std::uint8_t> output;
    output.reserve(sizehint);

    bool final = false;

    do
    {
        final = *bitstream++;

        auto compression = read_int<2>(bitstream);
        if (compression == 0)
        {
            // no compression
            int restBits = 8 - bitstream.current_bit();
            read_int(bitstream, restBits);
            std::uint16_t len = read_int<16>(bitstream);
            std::uint16_t nlen = read_int<16>(bitstream);

            if (len != ~nlen)
            {
                // error
                final = true;
                break;
            }

            std::copy_n(make_byte_iterator(bitstream), len, std::back_inserter(output));
            bitstream += len * 8;
        }
        else if (compression != 3)
        {
            huffman<int> valueLengthsD;
            huffman<int> distancesD;
            if (compression == 2)
            {
                std::tie(valueLengthsD, distancesD) = helper::read_huffman_tables(bitstream);
            }

            const auto& valueLengths =
                    compression == 1 ? helper::fixedValueLengthTree : valueLengthsD;
            const auto& distances =
                    compression == 1 ? helper::fixedDistanceTree : distancesD;

            bool eob = false;
            while (!eob)
            {
                if (std::distance(begin, bitstream) >= maxbits)
                {
                    // possible error
                    eob = true;
                    final = true;
                    break;
                }

                int value = valueLengths.decode([&]() { return *bitstream++; }).value();
                if (value < 256)
                {
                    output.push_back(value);
                }
                else if (value == 256)
                {
                    eob = true;
                }
                else
                {
                    int length = helper::get_length(bitstream, value);
                    int distance = helper::decode_distance(bitstream, distances);
                    // make sure output is big enough for copy_n
                    output.reserve(output.capacity() + length);
                    auto copyStart = output.end() - distance;
                    std::copy_n(copyStart, length, std::back_inserter(output));
                }
            }
        }
        else
        {
            // error; just return what we have
            final = true;
        }
    } while (!final);

    return output;
}

template<typename Iterator>
std::vector<std::uint8_t> inflate(const Iterator& begin, const Iterator& end, std::size_t sizehint)
{
    auto bitstream = make_bit_iterator(begin);
    auto endstream = make_bit_iterator(end);
    auto bits = std::distance(bitstream, endstream);
    return inflate(bitstream, bits, sizehint);
}

}

#endif // DEFLATE_HPP
