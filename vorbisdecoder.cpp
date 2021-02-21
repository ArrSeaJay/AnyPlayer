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

/* Implementation of a vorbis decoder.
 *
 * Most algorithms in this file are one-to-one conversions from the vorbis specification.
 *
 * And some are even untested like the complete floor0 code.
 *
 * The code is written for readability and not with performance in mind.
 *
 * Although it is indeed quite efficient by using C++ constructs that are introduced in the
 * C++11 standard (i.e. move semantics) and by using standard containers that are much better than
 * their reputation.
 * So the code tries to be modern C++.
 */

#include "vorbisdecoder.hpp"

#include "oggstream.hpp"

#include "memorystream.hpp"
#include "filestream.hpp"

#include "mdct.hpp"
#include "helper.hpp"

#include <memory>
#include "huffman.hpp"

#include <cmath>
#include <valarray>
#include <utility>
#include <limits>
#include <array>
#include <fstream>
#include <functional>
#include <iostream>
#include <iomanip>
#include <numbers>

namespace // vorbis internals
{

template<typename T>
constexpr auto square(T a)
{
    return a * a;
}

const auto pi = std::numbers::pi_v<float>;
const auto pi2 = pi / 2.0f;

int lookup1_values(int entries, int dimensions)
{
   int r = std::floor(std::exp(std::log(entries) / dimensions));
   if (std::floor(std::pow(r + 1, dimensions)) <= entries)
      ++r;
   return r;
}

// Helper class to pass current- and end-iterator for the current vorbis packet to the next decoder level.
// Doesn't own the vorbis packet as that is passed only downwards the stack; it doesn't lie on the heap.
struct packet
{
    typedef anyplayer::bit_iterator<std::vector<std::uint8_t>::const_iterator, false> read_iterator;

    packet(const std::vector<std::uint8_t>& packet)
        : current(anyplayer::make_bit_iterator<false>(packet.cbegin()))
        , end(anyplayer::make_bit_iterator<false>(packet.cend()))
    {
    }

    int available_bits() const
    {
        return end - current;
    }

    read_iterator current;
    read_iterator end;
};

template<typename iterator>
float read_float(anyplayer::bit_iterator<iterator, false>& reader)
{
    uint32_t x = anyplayer::read_int<32>(reader);
    uint32_t mantissa = x & 0x1fffff;
    uint32_t sign = x & 0x80000000;
    uint32_t exp = (x & 0x7fe00000) >> 21;
    auto res = sign ? -static_cast<float>(mantissa) : static_cast<float>(mantissa);
    return std::ldexp(res, exp - 788);
}

anyplayer::huffman<int> create_huffman_tree(std::vector<int> codeword_lengths)
{
    if (codeword_lengths.size() == 1)
        return {codeword_lengths.front()};

    anyplayer::huffman<int> root;
    anyplayer::huffman<int>* node = nullptr;

    int size = codeword_lengths.size();
    for (int entry = 0; entry < size; ++entry)
    {
        int depth = codeword_lengths[entry];
        node = &root;
        while (depth > 0)
        {
            if (depth == 1)
            {
                if (node->is_free(false))
                {
                    node->append(false, entry);
                    depth--;
                }
                else if (node->is_free(true))
                {
                    node->append(true, entry);
                    depth--;
                }
                else
                {
                    throw anyplayer::decoder_exception();
                }
            }
            else if (node->is_free(false))
            {
                node = node->append(false);
                depth--;
            }
            else if (!node->is_full(depth, false))
            {
                node = node->get_branch(false);
                depth--;
            }
            else if (node->is_free(true))
            {
                node = node->append(true);
                depth--;
            }
            else if (!node->is_full(depth, true))
            {
                node = node->get_branch(true);
                depth--;
            }
            else
            {
                throw anyplayer::decoder_exception();
            }
        }
    }

    return root;
}

class codebook
{
private:
    anyplayer::huffman<int> huffman;
    std::vector<std::valarray<float>> lookup_tables;
    int dimensions;

public:
    template<typename iterator>
    codebook(anyplayer::bit_iterator<iterator, false>& reader)
    {
        auto sync = anyplayer::read_int<24>(reader);
        if (sync != 0x564342)
        {
            throw anyplayer::decoder_exception();
        }

        dimensions = anyplayer::read_int<16>(reader);
        int entries = anyplayer::read_int<24>(reader);

        huffman = read_codebook(reader, entries);

        lookup_tables = read_lookup_tables(reader, entries);
    }

    int get_dimensions() const { return dimensions; }

private:
    template<typename iterator>
    anyplayer::huffman<int> read_codebook(anyplayer::bit_iterator<iterator, false>& reader, int entries)
    {
        std::vector<int> codeword_lengths(entries);

        if (*reader++) // ordered
        {
            int current_entry = 0;
            int current_length =  anyplayer::read_int<5>(reader) + 1;
            while (current_entry < entries)
            {
                int bits = anyplayer::helper::ilog((int)entries - current_entry);
                int number = anyplayer::read_int(reader, bits);
                for (int i = 0; i < number; i++)
                {
                    codeword_lengths[current_entry] = current_length;
                    current_entry++;
                }
                current_length++;
            }
        }
        else
        {
            auto sparse = *reader++;
            for (int i = 0; i < entries; ++i)
            {
                if (sparse)
                {
                    if (*reader++)
                    {
                        codeword_lengths[i] = anyplayer::read_int<5>(reader) + 1;
                    }
                    else
                    {
                        codeword_lengths[i] = 0;
                    }
                }
                else
                {
                    codeword_lengths[i] = anyplayer::read_int<5>(reader) + 1;
                }
            }
        }

        return create_huffman_tree(codeword_lengths);
    }

    template<typename iterator>
    std::vector<std::valarray<float>> read_lookup_tables(anyplayer::bit_iterator<iterator, false>& reader, int entries)
    {
        std::vector<std::valarray<float>> lt;

        auto lookup_type = anyplayer::read_int<4>(reader);
        if (lookup_type != 0)
        {
            auto codebook_minimum_value = read_float(reader);
            auto codebook_delta_value = read_float(reader);
            auto codebook_value_bits = anyplayer::read_int<4>(reader) + 1;
            auto codebook_sequence_p = *reader++;

            int codebook_lookup_values;
            if (lookup_type == 1)
                codebook_lookup_values = lookup1_values(entries, dimensions);
            else
                codebook_lookup_values = (int)(entries * dimensions);

            std::vector<int> multiplicands(codebook_lookup_values);
            for (int i = 0; i < codebook_lookup_values; ++i)
                multiplicands[i] = anyplayer::read_int(reader, codebook_value_bits);

            lt.reserve(entries);
            if (lookup_type == 1)
            {
                for (int lookup_offset = 0; lookup_offset < entries; ++lookup_offset)
                {
                    std::valarray<float> value_vector(dimensions);
                    float last = 0.0;
                    int index_divisor = 1;
                    for (int i = 0; i < dimensions; ++i)
                    {
                        int multiplicand_offset = (lookup_offset / index_divisor) % codebook_lookup_values;

                        value_vector[i] = multiplicands[multiplicand_offset] * codebook_delta_value + codebook_minimum_value + last;

                        if (codebook_sequence_p)
                            last = value_vector[i];

                        index_divisor = index_divisor * codebook_lookup_values;
                    }
                    lt.push_back(value_vector);
                }
            }
            else
            {
                for (int lookup_offset = 0; lookup_offset < entries; ++lookup_offset)
                {
                    std::valarray<float> value_vector(dimensions);
                    float last = 0.0;
                    int multiplicand_offset = lookup_offset * dimensions;
                    for (int i = 0; i < dimensions; ++i)
                    {
                        value_vector[i] = multiplicands[multiplicand_offset] * codebook_delta_value + codebook_minimum_value + last;

                        if (codebook_sequence_p)
                            last = value_vector[i];

                        multiplicand_offset++;
                    }
                    lt.push_back(value_vector);
                }
            }
        }

        return lt;
    }

public:
    std::optional<int> decode(packet& reader) const
    {
        auto code = huffman.decode([&reader]()
        {
            return *reader.current++;
        });

        if (reader.available_bits() < 0)
        {
            return {};
        }

        return code;
    }

    const std::valarray<float>& decode_vector(packet& reader) const
    {
        static std::valarray<float> empty;

        auto index = decode(reader);
        if (index)
        {
            return lookup_tables[*index];
        }
        else
        {
            return empty;
        }
    }
};

// Function returned by floor decode that is applied with the residues as an argument.
// The in-out argument is by design to save copying a lot.
typedef std::function<void(std::valarray<float>&)> floor_application;

/**
 * @brief The Floor decoder interface
 */
class floor
{
public:
    virtual ~floor() = default;

    /**
     * @brief decode the floor data from given packet
     * @param reader The packet data reader for the packet
     * @param n The size of the fully decoded block. Should be divided by 2 to get the size of the
     * floor data.
     * @return A function to apply floor data with the residues data (i.e. multiply)
     */
    virtual floor_application decode(packet& reader, int n) const = 0;
};

class floor0 : public floor
{
    int order;
    int rate;
    float bark_map_size;
    int amplitude_bits;
    float amplitude_offset;
    std::vector<int> books;
    int book_bits;
    const std::vector<codebook>& codebooks;

public:
    template<typename iterator>
    floor0(anyplayer::bit_iterator<iterator, false>& reader, const std::vector<codebook>& cb)
        : codebooks(cb)
    {
        order = anyplayer::read_int<8>(reader);
        rate = anyplayer::read_int<16>(reader);
        bark_map_size = static_cast<float>(anyplayer::read_int<16>(reader));
        amplitude_bits = anyplayer::read_int<6>(reader);
        amplitude_offset = static_cast<float>(anyplayer::read_int<8>(reader));
        int num_books = anyplayer::read_int<4>(reader);
        for (int i = 0; i < num_books; ++i)
        {
            books.push_back(anyplayer::read_int<8>(reader));
        }
        book_bits = anyplayer::helper::ilog(num_books);
    }

private:
    float bark(float x) const
    {
        return 13.1f * std::atan(.00074f * x) + 2.24f * std::atan(.0000000185f * square(x) + .0001f * x);
    }

    float foobar(int n, int i) const
    {
        return bark(rate * i / (2.f * n)) * bark_map_size / bark(.5f * rate);
    }

public:

    virtual floor_application decode(packet& reader, int n) const override
    {
        n /= 2;

        float amplitude = static_cast<float>(anyplayer::read_int(reader.current, amplitude_bits));
        if (amplitude > 0.f)
        {
            std::valarray<float> coefficients(order);
            auto current = std::begin(coefficients);

            auto book = anyplayer::read_int(reader.current, book_bits);
            if (book >= books.size())
                return {};

            float last = 0.f;

            while (current != std::end(coefficients))
            {
                auto temp_vector = codebooks[books[book]].decode_vector(reader);

                if (temp_vector.size() == 0)
                    return {};

                temp_vector += last;
                last = temp_vector[temp_vector.size() - 1];

                current = std::copy_n(std::begin(temp_vector), std::min(temp_vector.size(), static_cast<std::size_t>(std::distance(current, std::end(coefficients)))), current);
            }

            auto cos_coeff = std::cos(coefficients);

            std::valarray<float> map(n);
            for (int i = 0; i < (n - 1); ++i)
            {
                map[i] = std::min(bark_map_size - 1.f, foobar(n, i));
            }
            map[n - 1] = -1;

            return [&](std::valarray<float>& output)
            {
                for (int i = 0; i < n; )
                {
                    auto omega = pi * map[i] / bark_map_size;
                    auto cos_omega = std::cos(omega);
                    float p, q;
                    if ((order % 2) == 1)
                    {
                        p = 1.f;
                        for (int j = 1; j < (order - 3) / 2; j += 2)
                        {
                            p *= square(cos_coeff[j] - cos_omega) * 4.f;
                        }
                        p *= 1.f - square(cos_omega);

                        q = 1.f;
                        for (int j = 0; j < (order - 1) / 2; j += 2)
                        {
                            q *= square(cos_coeff[j] - cos_omega) * 4.f;
                        }
                        q /= 4.f;
                    }
                    else
                    {
                        p = 1.f;
                        for (int j = 1; j < (order - 2) / 2; j += 2)
                        {
                            p *= square(cos_coeff[j] - cos_omega) * 4.f;
                        }
                        p *= 1.f - square(cos_omega);
                        p /= 2.f;

                        q = 1.f;
                        for (int j = 0; j < (order - 2) / 2; j += 2)
                        {
                            q *= square(cos_coeff[j] - cos_omega) * 4.f;
                        }
                        q *= 1.f + square(cos_omega);
                        q /= 2.f;
                    }

                    auto linear_floor_value = std::exp(.11512925f * ((amplitude * amplitude_offset / ((std::exp2(amplitude_bits) - 1.f) * std::sqrt(p + q))) - amplitude_offset));
                    auto iteration_condition = map[i];
                    do
                    {
                        output[i++] *= linear_floor_value;
                    }
                    while (map[i] == iteration_condition);
                }
            };
        }

        return {};
    }
};

int render_point(int x0, int y0, int x1, int y1, int x)
{
    int dy = y1 - y0;
    int adx = x1 - x0;
    int ady = std::abs(dy);
    int err = ady * (x - x0);
    int off = err / adx;
    return dy < 0 ? y0 - off : y0 + off;
}

float inverse_db_table[256] =
{
  1.0649863e-07f, 1.1341951e-07f, 1.2079015e-07f, 1.2863978e-07f,
  1.3699951e-07f, 1.4590251e-07f, 1.5538408e-07f, 1.6548181e-07f,
  1.7623575e-07f, 1.8768855e-07f, 1.9988561e-07f, 2.1287530e-07f,
  2.2670913e-07f, 2.4144197e-07f, 2.5713223e-07f, 2.7384213e-07f,
  2.9163793e-07f, 3.1059021e-07f, 3.3077411e-07f, 3.5226968e-07f,
  3.7516214e-07f, 3.9954229e-07f, 4.2550680e-07f, 4.5315863e-07f,
  4.8260743e-07f, 5.1396998e-07f, 5.4737065e-07f, 5.8294187e-07f,
  6.2082472e-07f, 6.6116941e-07f, 7.0413592e-07f, 7.4989464e-07f,
  7.9862701e-07f, 8.5052630e-07f, 9.0579828e-07f, 9.6466216e-07f,
  1.0273513e-06f, 1.0941144e-06f, 1.1652161e-06f, 1.2409384e-06f,
  1.3215816e-06f, 1.4074654e-06f, 1.4989305e-06f, 1.5963394e-06f,
  1.7000785e-06f, 1.8105592e-06f, 1.9282195e-06f, 2.0535261e-06f,
  2.1869758e-06f, 2.3290978e-06f, 2.4804557e-06f, 2.6416497e-06f,
  2.8133190e-06f, 2.9961443e-06f, 3.1908506e-06f, 3.3982101e-06f,
  3.6190449e-06f, 3.8542308e-06f, 4.1047004e-06f, 4.3714470e-06f,
  4.6555282e-06f, 4.9580707e-06f, 5.2802740e-06f, 5.6234160e-06f,
  5.9888572e-06f, 6.3780469e-06f, 6.7925283e-06f, 7.2339451e-06f,
  7.7040476e-06f, 8.2047000e-06f, 8.7378876e-06f, 9.3057248e-06f,
  9.9104632e-06f, 1.0554501e-05f, 1.1240392e-05f, 1.1970856e-05f,
  1.2748789e-05f, 1.3577278e-05f, 1.4459606e-05f, 1.5399272e-05f,
  1.6400004e-05f, 1.7465768e-05f, 1.8600792e-05f, 1.9809576e-05f,
  2.1096914e-05f, 2.2467911e-05f, 2.3928002e-05f, 2.5482978e-05f,
  2.7139006e-05f, 2.8902651e-05f, 3.0780908e-05f, 3.2781225e-05f,
  3.4911534e-05f, 3.7180282e-05f, 3.9596466e-05f, 4.2169667e-05f,
  4.4910090e-05f, 4.7828601e-05f, 5.0936773e-05f, 5.4246931e-05f,
  5.7772202e-05f, 6.1526565e-05f, 6.5524908e-05f, 6.9783085e-05f,
  7.4317983e-05f, 7.9147585e-05f, 8.4291040e-05f, 8.9768747e-05f,
  9.5602426e-05f, 0.00010181521f, 0.00010843174f, 0.00011547824f,
  0.00012298267f, 0.00013097477f, 0.00013948625f, 0.00014855085f,
  0.00015820453f, 0.00016848555f, 0.00017943469f, 0.00019109536f,
  0.00020351382f, 0.00021673929f, 0.00023082423f, 0.00024582449f,
  0.00026179955f, 0.00027881276f, 0.00029693158f, 0.00031622787f,
  0.00033677814f, 0.00035866388f, 0.00038197188f, 0.00040679456f,
  0.00043323036f, 0.00046138411f, 0.00049136745f, 0.00052329927f,
  0.00055730621f, 0.00059352311f, 0.00063209358f, 0.00067317058f,
  0.00071691700f, 0.00076350630f, 0.00081312324f, 0.00086596457f,
  0.00092223983f, 0.00098217216f, 0.0010459992f,  0.0011139742f,
  0.0011863665f,  0.0012634633f,  0.0013455702f,  0.0014330129f,
  0.0015261382f,  0.0016253153f,  0.0017309374f,  0.0018434235f,
  0.0019632195f,  0.0020908006f,  0.0022266726f,  0.0023713743f,
  0.0025254795f,  0.0026895994f,  0.0028643847f,  0.0030505286f,
  0.0032487691f,  0.0034598925f,  0.0036847358f,  0.0039241906f,
  0.0041792066f,  0.0044507950f,  0.0047400328f,  0.0050480668f,
  0.0053761186f,  0.0057254891f,  0.0060975636f,  0.0064938176f,
  0.0069158225f,  0.0073652516f,  0.0078438871f,  0.0083536271f,
  0.0088964928f,  0.009474637f,   0.010090352f,   0.010746080f,
  0.011444421f,   0.012188144f,   0.012980198f,   0.013823725f,
  0.014722068f,   0.015678791f,   0.016697687f,   0.017782797f,
  0.018938423f,   0.020169149f,   0.021479854f,   0.022875735f,
  0.024362330f,   0.025945531f,   0.027631618f,   0.029427276f,
  0.031339626f,   0.033376252f,   0.035545228f,   0.037855157f,
  0.040315199f,   0.042935108f,   0.045725273f,   0.048696758f,
  0.051861348f,   0.055231591f,   0.058820850f,   0.062643361f,
  0.066714279f,   0.071049749f,   0.075666962f,   0.080584227f,
  0.085821044f,   0.091398179f,   0.097337747f,   0.10366330f,
  0.11039993f,    0.11757434f,    0.12521498f,    0.13335215f,
  0.14201813f,    0.15124727f,    0.16107617f,    0.17154380f,
  0.18269168f,    0.19456402f,    0.20720788f,    0.22067342f,
  0.23501402f,    0.25028656f,    0.26655159f,    0.28387361f,
  0.30232132f,    0.32196786f,    0.34289114f,    0.36517414f,
  0.38890521f,    0.41417847f,    0.44109412f,    0.46975890f,
  0.50028648f,    0.53279791f,    0.56742212f,    0.60429640f,
  0.64356699f,    0.68538959f,    0.72993007f,    0.77736504f,
  0.82788260f,    0.88168307f,    0.9389798f,     1.0
};

void render_line(int x0, int y0, int x1, int y1, std::valarray<float>& v)
{
    int dy = y1 - y0;
    int adx = x1 - x0;
    int ady = std::abs(dy);
    int base = dy / adx;
    int x = x0;
    int y = y0;
    int err = -adx;
    int sy = 1 - (((dy >> 31) & 1) * 2);
    ady -= std::abs(base) * adx;
    v[x0] *= inverse_db_table[y0];
    while(++x < x1)
    {
        y += base;
        err += ady;
        if (err >= 0)
        {
            err -= adx;
            y += sy;
        }
        v[x] *= inverse_db_table[y];
    }
}

class floor1 : public floor
{
private:
    int partitions;
    std::vector<int> partition_class_list;
    std::vector<int> class_dimensions;
    std::vector<int> class_subclasses;
    std::vector<int> class_masterbooks;
    std::vector<std::vector<int>> subclass_books;
    int multiplier;
    std::vector<int> x_list;
    std::vector<std::pair<int, int>> neighbors;
    std::vector<std::pair<int, int>> sorted_x;

    const std::vector<codebook>& codebooks;

    static std::array<int, 4> ranges;

public:
    template<typename iterator>
    floor1(anyplayer::bit_iterator<iterator, false>& reader, const std::vector<codebook>& cb)
        : codebooks(cb)
    {
        partitions = anyplayer::read_int<5>(reader);
        int maximum_class = -1;
        partition_class_list.reserve(partitions);
        for (int i = 0; i < partitions; ++i)
        {
            int partition_class = anyplayer::read_int<4>(reader);
            partition_class_list.push_back(partition_class);
            if (partition_class > maximum_class)
                maximum_class = partition_class;
        }

        ++maximum_class;

        class_dimensions.resize(maximum_class);
        class_subclasses.resize(maximum_class);
        class_masterbooks.resize(maximum_class);

        subclass_books.reserve(maximum_class);

        for (int i = 0; i < maximum_class; ++i)
        {
            class_dimensions[i] = anyplayer::read_int<3>(reader) + 1;
            int subclass = anyplayer::read_int<2>(reader);
            class_subclasses[i] = subclass;
            if (subclass != 0)
            {
                class_masterbooks[i] = anyplayer::read_int<8>(reader);
            }
            int k = 1 << subclass;
            std::vector<int> subclassbook(k);
            for (int j = 0; j < k; ++j)
            {
                subclassbook[j] = anyplayer::read_int<8>(reader) - 1;
            }
            subclass_books.push_back(std::move(subclassbook));
        }

        x_list.reserve(65);
        multiplier = anyplayer::read_int<2>(reader) + 1;
        int range_bits = anyplayer::read_int<4>(reader);
        x_list.push_back(0);
        x_list.push_back(1 << range_bits);
        for (int i = 0; i < partitions; ++i)
        {
            int current_class_number = partition_class_list[i];
            for (int j = 0; j < class_dimensions[current_class_number]; ++j)
            {
                x_list.push_back(anyplayer::read_int(reader, range_bits));
            }
        }

        int x_list_size = x_list.size();
        neighbors.reserve(x_list_size);
        for (int i = 0; i < x_list_size; ++i)
        {
            neighbors.push_back(get_neighbors(i));
        }

        // presort
        sorted_x.reserve(x_list_size);
        for (int i = 0; i < x_list_size; ++i)
        {
            sorted_x.emplace_back(i, x_list[i]);
        }
        std::sort(sorted_x.begin(), sorted_x.end(), [](const std::pair<int,int>& a, const std::pair<int,int>& b){return a.second < b.second;});
    }

    std::pair<int, int> get_neighbors(int n)
    {
        int min = std::numeric_limits<int>::min();
        int max = std::numeric_limits<int>::max();

        int result_min = 0;
        int result_max = 0;

        for (int i = 0; i < n; ++i)
        {
            if (x_list[i] > min && x_list[i] < x_list[n])
            {
                min = x_list[i];
                result_min = i;
            }
            if (x_list[i] < max && x_list[i] > x_list[n])
            {
                max = x_list[i];
                result_max = i;
            }
        }

        return {result_min, result_max};
    }

    virtual floor_application decode(packet& reader, int n) const override
    {
        if (*reader.current++)
        {
            int range = ranges[multiplier - 1];
            auto y_list = decode_packet(reader, range);
            if (y_list.size() == 0)
                return {};
            auto points = amplitude_value_synthesis(range, y_list);
            return [n, points, this](std::valarray<float>& res) { curve_synthesis(n, points, res); };
        }

        return {};
    }

private:
    std::vector<int> decode_packet(packet& reader, int range) const
    {
        int bits = anyplayer::helper::ilog(range - 1);

        std::vector<int> y_list;
        y_list.reserve(x_list.size());

        y_list.push_back(anyplayer::read_int(reader.current, bits));
        y_list.push_back(anyplayer::read_int(reader.current, bits));

        if (reader.available_bits() <= 0)
        {
            return {};
        }

        for (int i = 0; i < partitions; ++i)
        {
            int class_ = partition_class_list[i];
            int cdim = class_dimensions[class_];
            int cbits = class_subclasses[class_];
            int csub = (1 << cbits) - 1;
            int cval = 0;
            if (cbits > 0)
            {
                int book = class_masterbooks[class_];
                auto tmp = codebooks[book].decode(reader);
                if (!tmp)
                    return {};
                cval = *tmp;
            }
            for (int j = 0; j < cdim; j++)
            {
                int book = subclass_books[class_][cval & csub];
                cval >>= cbits;
                if (book < 0)
                {
                    y_list.push_back(0);
                }
                else
                {
                    auto tmp = codebooks[book].decode(reader);
                    if (!tmp)
                        return {};
                    y_list.push_back(*tmp);
                }
            }
        }

        return y_list;
    }

    std::vector<std::pair<int, bool> > amplitude_value_synthesis(int range, const std::vector<int>& y_list) const
    {
        int values = x_list.size();
        std::vector<std::pair<int, bool>> points(values);
        points[0].second = true;
        points[0].first = y_list[0];
        points[1].second = true;
        points[1].first = y_list[1];
        for (int i = 2; i < values; ++i)
        {
            int low_neighbor_offset = neighbors[i].first;
            int high_neighbor_offset = neighbors[i].second;
            int predicted = render_point(x_list[low_neighbor_offset],
                                        points[low_neighbor_offset].first,
                                        x_list[high_neighbor_offset],
                                        points[high_neighbor_offset].first,
                                        x_list[i]);

            int val = y_list[i];
            int highroom = range - predicted;
            int lowroom = predicted;
            int room = (highroom < lowroom ? highroom : lowroom) * 2;
            if (val != 0)
            {
                points[low_neighbor_offset].second = true;
                points[high_neighbor_offset].second = true;
                points[i].second = true;
                if (val >= room)
                {
                    if (highroom > lowroom)
                    {
                        points[i].first = val - lowroom + predicted;
                    }
                    else
                    {
                        points[i].first =  predicted - val + highroom - 1;
                    }
                }
                else
                {
                    if ((val % 2) == 1)
                    {
                        points[i].first = predicted - ((val + 1) / 2);
                    }
                    else
                    {
                        points[i].first = predicted + (val / 2);
                    }
                }
            }
            else
            {
                points[i].second = false;
                points[i].first = predicted;
            }
        }

        return points;
    }

    void curve_synthesis(int n, std::vector<std::pair<int,bool>> points, std::valarray<float>& residues) const
    {
        int n2 = n / 2;

        int lx = 0;
        int ly = points[0].first * multiplier;
        int values = points.size();
        for (int j = 1; j < values && lx < n2; ++j)
        {
            int i = sorted_x[j].first;
            if (points[i].second)
            {
                int hy = points[i].first * multiplier;
                int hx = x_list[i];
                if (lx < n2) render_line(lx, ly, std::min(hx, n2), hy, residues);
                lx = hx;
                ly = hy;
            }
        }

        if (lx < n2)
        {
            render_line(lx, ly, n2, ly, residues);
        }
    }
};

std::array<int, 4> floor1::ranges {256, 128, 86, 64};

/**
 * @brief The Residue decoder interface
 */
class residue
{
protected:
    virtual bool decode_single(packet& reader, std::valarray<float>& v, int offset, int book) const = 0;

public:
    virtual ~residue() = default;

    virtual std::vector<std::valarray<float>> decode(packet& reader, int n, int ch, const std::vector<bool>& do_not_decode) const = 0;
};

class residue0 : public residue
{
protected:
    int begin;
    int end;
    int partition_size;
    int classifications;
    int classbook;
    std::vector<int> cascade;
    std::vector<int> books;

    const std::vector<codebook>& codebooks;

public:
    template<typename iterator>
    residue0(anyplayer::bit_iterator<iterator, false>& reader, const std::vector<codebook>& cb)
        : codebooks(cb)
    {
        begin = anyplayer::read_int<24>(reader);
        end = anyplayer::read_int<24>(reader);
        partition_size = anyplayer::read_int<24>(reader) + 1;
        classifications = anyplayer::read_int<6>(reader) + 1;
        classbook = anyplayer::read_int<8>(reader);

        cascade.reserve(classifications);
        for (int i = 0; i < classifications; ++i)
        {
            int low_bits = anyplayer::read_int<3>(reader);
            int high_bits = *reader++ ? anyplayer::read_int<5>(reader) : 0;

            cascade.push_back(high_bits * 8 + low_bits);
        }

        books.resize(classifications * 8);
        for (int i = 0, offset = 0; i < classifications; ++i, offset += 8)
        {
            for (int j = 0; j < 8; ++j)
            {
                if (((cascade[i] >> j) & 1) == 1)
                {
                    books[offset + j] = anyplayer::read_int<8>(reader);
                }
                else
                {
                    books[offset + j] = -1;
                }
            }
        }
    }

protected:
    virtual bool decode_single(packet& reader, std::valarray<float>& v, int offset, int book) const override
    {
        auto& codebook = codebooks[book];
        int dim = codebook.get_dimensions();
        int step = partition_size / codebook.get_dimensions();
        for (int i = 0; i < step; ++i)
        {
            auto dv = codebook.decode_vector(reader);
            if (dv.size() == 0)
                return false;
            v[std::slice(offset + i, dim, step)] += dv;
        }

        return true;
    }

public:
    virtual std::vector<std::valarray<float>> decode(packet& reader, int n, int ch, const std::vector<bool>& do_not_decode) const override
    {
        int actual_size = n / 2;

        int classwords_per_codeword = codebooks[classbook].get_dimensions();
        auto begin_ = begin;
        auto end_ = std::min(end, actual_size);
        int n_to_read = end_ - begin_;

        std::vector<std::valarray<float> > output(ch, std::valarray<float>(actual_size));

        if (n_to_read == 0)
        {
            return output;
        }

        int partitions_to_read = n_to_read / partition_size;
        int x = classwords_per_codeword * partitions_to_read;
        std::vector<int> classifications(ch * x);

        for (int pass = 0; pass < 8; ++pass)
        {
            int partition_count = 0;

            while (partition_count < partitions_to_read)
            {
                if (pass == 0)
                {
                    for (int j = 0; j < ch; ++j)
                    {
                        if (!do_not_decode[j])
                        {
                            int temp = codebooks[classbook].decode(reader).value();
                            for (int i = classwords_per_codeword - 1; i >= 0; --i)
                            {
                                classifications[(j * x) + i + partition_count] = temp % this->classifications;
                                temp /= this->classifications;
                            }
                        }
                    }
                }

                for (int i = 0; i < classwords_per_codeword && partition_count < partitions_to_read; ++i, ++partition_count)
                {
                    for (int j = 0; j < ch; ++j)
                    {
                        if (!do_not_decode[j])
                        {
                            int vqclass = classifications[(j * x) + partition_count];
                            int vqbook = books[(vqclass * 8) + pass];
                            if (vqbook != -1)
                            {
                                auto& v = output[j];
                                if (!decode_single(reader, v, begin_ + partition_count * partition_size, vqbook))
                                {
                                    return output;
                                }
                            }
                        }
                    }
                }
            }
        }

        return output;
    }
};

class residue1 : public residue0
{
public:
    using residue0::residue0;

protected:
    virtual bool decode_single(packet& reader, std::valarray<float>& v, int offset, int book) const override
    {
        auto& codebook = codebooks[book];
        int dim = codebook.get_dimensions();
        int i = 0;
        while (i < partition_size)
        {
            auto dv = codebook.decode_vector(reader);
            if (dv.size() == 0)
                return false;
            v[std::slice(offset + i, dim, 1)] += dv;
            i += dim;
        }

        return true;
    }
};

class residue2 : public residue1
{
public:
    using residue1::residue1;

    virtual std::vector<std::valarray<float>> decode(packet& reader, int n, int ch, const std::vector<bool>& do_not_decode) const override
    {
        std::vector<std::valarray<float>> output(ch, std::valarray<float>(n / 2));

        if (std::all_of(do_not_decode.begin(), do_not_decode.end(), std::identity{}))
        {
            return output;
        }

        auto v = residue1::decode(reader, n * ch, 1, { false }).front();

        for (int j = 0; j < ch; ++j)
        {
            output[j] = v[std::slice(j, n / 2, ch)];
        }

        return output;
    }
};

class mapping
{
    const std::vector<std::unique_ptr<floor>>& floors;
    const std::vector<std::unique_ptr<residue>>& residues;

    std::vector<std::pair<int, int>> coupling_steps;
    std::vector<int> mux;
    std::vector<std::pair<int, int>> submaps;
public:
    template<typename iterator>
    mapping(anyplayer::bit_iterator<iterator, false>& reader, int channels, const std::vector<std::unique_ptr<floor>>& f, const std::vector<std::unique_ptr<residue>>& r)
        : floors(f)
        , residues(r)
    {
        int type = anyplayer::read_int<16>(reader);
        if (type != 0)
            throw anyplayer::decoder_exception();

        int mapping_submaps = *reader++ ? anyplayer::read_int<4>(reader) + 1 : 1;

        int cs = *reader++ ? anyplayer::read_int<8>(reader) + 1 : 0;
        int bits = anyplayer::helper::ilog(channels - 1);
        coupling_steps.reserve(cs);
        for (int j = 0; j < cs; ++j)
        {
            int magnitudes = anyplayer::read_int(reader, bits);
            int angles = anyplayer::read_int(reader, bits);
            coupling_steps.emplace_back(magnitudes, angles);
        }

        if (anyplayer::read_int<2>(reader) != 0)
            throw anyplayer::decoder_exception();

        mux.resize(channels);
        if (mapping_submaps > 1)
        {
            for (int j = 0; j < channels; ++j)
            {
                mux[j] = anyplayer::read_int<4>(reader);
                if (mux[j] > mapping_submaps - 1)
                    throw anyplayer::decoder_exception();
            }
        }

        submaps.reserve(mapping_submaps);
        for (int j = 0; j < mapping_submaps; ++j)
        {
            anyplayer::read_int<8>(reader);
            auto floor = anyplayer::read_int<8>(reader);
            auto residue = anyplayer::read_int<8>(reader);
            if (floor >= floors.size() || residue >= residues.size())
                throw anyplayer::decoder_exception();

            submaps.emplace_back(floor, residue);
        }
    }

    floor_application decode_floor(packet& reader, int channel, int n) const
    {
        return floors[submaps[mux[channel]].first]->decode(reader, n);
    }

    void nonzero_vector_propagate(std::vector<bool>& no_residues) const
    {
        for (const auto& cs : coupling_steps)
        {
            bool tmp = no_residues[cs.first] && no_residues[cs.second];

            no_residues[cs.first] = tmp;
            no_residues[cs.second] = tmp;
        }
    }

    std::vector<std::valarray<float>> decode_residues(packet& reader, int n, int channels, std::vector<bool> no_residues) const
    {
        std::vector<bool> do_not_decode(channels);
        std::vector<std::valarray<float>> rv(channels);

        int size = submaps.size();
        for (int i = 0; i < size; ++i)
        {
            int ch = 0;
            for (int j = 0; j < channels; ++j)
            {
                if (mux[j] == i)
                {
                    do_not_decode[ch] = no_residues[j];
                    ++ch;
                }
            }

            auto residue_vectors = residues[submaps[i].second]->decode(reader, n, ch, do_not_decode);

            ch = 0;
            for (int j = 0; j < channels; ++j)
            {
                if (mux[j] == i)
                {
                    rv[j] = std::move(residue_vectors[ch]);
                    ++ch;
                }
            }
        }

        return rv;
    }

    void reverse_coupling(std::vector<std::valarray<float>>& residue_vectors) const
    {
        for (auto cs = coupling_steps.crbegin(); cs != coupling_steps.crend(); ++cs)
        {
            auto& magnitude_vector = residue_vectors[cs->first];
            auto& angle_vector = residue_vectors[cs->second];
            int rs = magnitude_vector.size() / 2;
            for (int i = 0; i < rs; ++i)
            {
                auto m = magnitude_vector[i];
                auto a = angle_vector[i];

                float new_m, new_a;

                if (m > 0.0)
                {
                    if (a > 0.0)
                    {
                        new_m = m;
                        new_a = m - a;
                    }
                    else
                    {
                        new_a = m;
                        new_m = m + a;
                    }
                }
                else
                {
                    if (a > 0.0)
                    {
                        new_m = m;
                        new_a = m + a;
                    }
                    else
                    {
                        new_a = m;
                        new_m = m - a;
                    }
                }

                magnitude_vector[i] = new_m;
                angle_vector[i] = new_a;
            }
        }
    }
};

class mode
{
    int channels;
    bool blockflag;
    int windowtype;
    int transformtype;
    const class mapping& mapping;
    int blocksize;
    anyplayer::mdct<float> mdct;

public:
    template<typename iterator>
    mode(anyplayer::bit_iterator<iterator, false>& reader, int ch, int blocksize0, int blocksize1, const std::vector<class mapping>& mappings)
        : channels(ch)
        , blockflag(*reader++)
        , windowtype(anyplayer::read_int<16>(reader))
        , transformtype(anyplayer::read_int<16>(reader))
        , mapping(mappings[anyplayer::read_int<8>(reader)])
        , blocksize(blockflag ? blocksize1 : blocksize0)
        , mdct(blocksize)
    {
        if (windowtype != 0)
            throw anyplayer::decoder_exception();
        if (transformtype != 0)
            throw anyplayer::decoder_exception();
    }

    int get_blocksize() const
    {
        return blocksize;
    }

    std::vector<std::valarray<float>> decode(packet& reader) const
    {
        if (blockflag)
        {
            reader.current++;
            reader.current++;
        }

        std::vector<bool> no_residues(channels);

        std::vector<floor_application> floors;
        floors.reserve(channels);
        for (int i = 0; i < channels; ++i)
        {
            floors.push_back(mapping.decode_floor(reader, i, blocksize));
            no_residues[i] = !floors.back();

            if (reader.available_bits() <= 0)
                return std::vector<std::valarray<float>>(channels, std::valarray<float>(blocksize));
        }

        mapping.nonzero_vector_propagate(no_residues);

        auto residues = mapping.decode_residues(reader, blocksize, channels, no_residues);

        mapping.reverse_coupling(residues);

        std::vector<std::valarray<float>> output;
        output.reserve(channels);

        for (int ch = 0; ch < channels; ++ch)
        {
            floor_application floor = floors[ch];
            if (floor)
            {
                floor(residues[ch]);
            }

            output.push_back(mdct.inverse(residues[ch]));
        }

        return output;
    }
};

float slope(int i, int n)
{
    return std::sin(pi2 * square(std::sin((i + 0.5f) / n * pi2)));
}

}

struct anyplayer::vorbis_decoder::internal
{
    std::unique_ptr<anyplayer::stream> stream_;

    ogg_decoder ogg;

    int channels;
    int sample_rate;
    int bitrate_max;
    int bitrate_nom;
    int bitrate_min;
    int blocksize0;
    int blocksize1;

    bool primed = false;

    std::multimap<std::string, std::string> comments;

    std::vector<codebook> codebooks;
    std::vector<std::unique_ptr<class floor>> floors;
    std::vector<std::unique_ptr<residue>> residues;
    std::vector<mapping> mappings;
    std::vector<mode> modes;

    std::vector<std::valarray<float>> previous;

    std::vector<std::valarray<float>> raw_samples;
    int samples_read = -1;

    std::valarray<float> window0;
    std::valarray<float> window1;

    std::uint64_t total_samples;
    std::uint64_t position;

    internal(std::unique_ptr<anyplayer::stream> s)
        : stream_(std::move(s))
        , ogg(*stream_)
        , total_samples(-1)
        , position(0)
    {
        read_identification_packet();
        read_comments_packet();
        read_setup_packet();

        if (stream_->seekable())
        {
            total_samples = get_total_samples();
            reset();
        }
    }

    void reset()
    {
        ogg.reset();
        ogg.get_packet();
        ogg.get_packet();
        ogg.get_packet();
    }

    void read_identification_packet()
    {
        auto packet = ogg.get_packet();
        auto reader = anyplayer::make_bit_iterator(packet.cbegin());

        if (anyplayer::read_int<8>(reader) != 1)
        {
            throw decoder_exception();
        }

        validate_vorbis_id(reader);

        if (anyplayer::read_int<32>(reader) != 0)
        {
            throw decoder_exception();
        }

        channels = anyplayer::read_int<8>(reader);
        sample_rate = anyplayer::read_int<32>(reader);
        bitrate_max = anyplayer::read_int<32>(reader);
        bitrate_nom = anyplayer::read_int<32>(reader);
        bitrate_min = anyplayer::read_int<32>(reader);
        blocksize0 = 1 << anyplayer::read_int<4>(reader);
        blocksize1 = 1 << anyplayer::read_int<4>(reader);

        // short slope
        int n02 = blocksize0 / 2;
        window0.resize(n02);
        for (int i = 0; i < n02; ++i)
        {
            window0[i] = slope(i, n02);
        }

        // long slope
        int n12 = blocksize1 / 2;
        window1.resize(n12);
        for (int i = 0; i < n12; ++i)
        {
            window1[i] = slope(i, n12);
        }

        if (!*reader++)
            throw decoder_exception();
    }

    void read_comments_packet()
    {
        auto packet = ogg.get_packet();
        auto reader = anyplayer::make_bit_iterator(packet.cbegin());

        if (anyplayer::read_int<8>(reader) != 3)
        {
            throw decoder_exception();
        }

        validate_vorbis_id(reader);

        comments = read_vorbis_comments(reader);

        if (!*reader++)
            throw decoder_exception();
    }

    void read_setup_packet()
    {
        auto packet = ogg.get_packet();
        auto reader = anyplayer::make_bit_iterator(packet.cbegin());

        if (anyplayer::read_int<8>(reader) != 5)
        {
            throw decoder_exception();
        }

        validate_vorbis_id(reader);

        setup_codebooks(reader);
        setup_times(reader);
        setup_floors(reader);
        setup_residues(reader);
        setup_mappings(reader);
        setup_modes(reader);

        if (!*reader++)
            throw decoder_exception();
    }

    template<typename iterator>
    void setup_codebooks(anyplayer::bit_iterator<iterator, false>& reader)
    {
        int codebook_count = anyplayer::read_int<8>(reader) + 1;
        codebooks.reserve(codebook_count);

        for (int i = 0; i < codebook_count; ++i)
        {
            codebooks.emplace_back(reader);
        }
    }

    template<typename iterator>
    void setup_times(anyplayer::bit_iterator<iterator, false>& reader)
    {
        int time_count = anyplayer::read_int<6>(reader) + 1;
        for (int i = 0; i < time_count; i++)
        {
            auto time = anyplayer::read_int<16>(reader);
            if (time != 0)
                throw decoder_exception();
        }
    }

    template<typename iterator>
    void setup_floors(anyplayer::bit_iterator<iterator, false>& reader)
    {
        int floor_count = anyplayer::read_int<6>(reader) + 1;
        floors.reserve(floor_count);

        for (int i = 0; i < floor_count; ++i)
        {
            auto type = anyplayer::read_int<16>(reader);
            switch (type)
            {
            case 0:
                floors.emplace_back(std::make_unique<floor0>(reader, codebooks));
                break;
            case 1:
                floors.emplace_back(std::make_unique<floor1>(reader, codebooks));
                break;
            default:
                throw decoder_exception();
            }
        }
    }

    template<typename iterator>
    void setup_residues(anyplayer::bit_iterator<iterator, false>& reader)
    {
        int residue_count = anyplayer::read_int<6>(reader) + 1;
        residues.reserve(residue_count);

        for (int i = 0; i < residue_count; ++i)
        {
            auto type = anyplayer::read_int<16>(reader);
            switch (type)
            {
            case 0:
                residues.emplace_back(std::make_unique<residue0>(reader, codebooks));
                break;
            case 1:
                residues.emplace_back(std::make_unique<residue1>(reader, codebooks));
                break;
            case 2:
                residues.emplace_back(std::make_unique<residue2>(reader, codebooks));
                break;
            default:
                throw decoder_exception();
            }
        }
    }

    template<typename iterator>
    void setup_mappings(anyplayer::bit_iterator<iterator, false>& reader)
    {
        int count = anyplayer::read_int<6>(reader) + 1;
        mappings.reserve(count);

        for (int i = 0; i < count; ++i)
        {
            mappings.emplace_back(reader, channels, floors, residues);
        }
    }

    template<typename iterator>
    void setup_modes(anyplayer::bit_iterator<iterator, false>& reader)
    {
        int count = anyplayer::read_int<6>(reader) + 1;
        modes.reserve(count);

        for (int i = 0; i < count; ++i)
        {
            modes.emplace_back(reader, channels, blocksize0, blocksize1, mappings);
        }
    }

    template<typename iterator>
    void validate_vorbis_id(anyplayer::bit_iterator<iterator, false>& reader)
    {
        auto vorbis = anyplayer::read_int<6 * 8>(reader);
        if (vorbis != 0x736962726f76)
            throw decoder_exception();
    }

    std::vector<std::valarray<float>> read_packet()
    {
        auto packet = ogg.get_packet();

        if (packet.size() == 0)
            return std::vector<std::valarray<float>>(channels);

        struct packet reader(packet);

        if (*reader.current++)
        {
            throw decoder_exception();
        }

        int mode_number = anyplayer::read_int(reader.current, anyplayer::helper::ilog(modes.size() - 1));
        return modes[mode_number].decode(reader);
    }

    std::uint64_t get_total_samples()
    {
        auto packet = ogg.get_packet();

        std::vector<int> bs;
        while (packet.size() != 0)
        {
            auto reader = anyplayer::make_bit_iterator(packet.cbegin());
            ++reader;
            int mode_number = anyplayer::read_int(reader, anyplayer::helper::ilog(modes.size() - 1));
            bs.push_back(modes[mode_number].get_blocksize());
            packet = ogg.get_packet();
        }

        auto it = bs.begin();
        std::uint64_t count = 0;
        int prev = *it++;
        do
        {
            count += (prev + *it) / 4;

            prev = *it;
        }
        while (++it != bs.end());

        return count;
    }

    std::valarray<float> overlap(const std::valarray<float>& current, const std::valarray<float>& previous) const
    {
        int size = previous.size() / 4 + current.size() / 4;
        std::valarray<float> overlapped(size);

        if (current.size() == previous.size())
        {
            const auto& w = current.size() == (std::size_t)blocksize0 ? window0 : window1;

            int center = size;

            for (int i = 0; i < size; ++i)
            {
                auto o = previous[i + center] * w[center - i - 1] + current[i] * w[i];
                overlapped[i] = o;
            }
        }
        else if (current.size() < previous.size())
        {
            const auto& w = window0;

            // long - short
            int center = previous.size() / 2;
            int prev_n = previous.size() / 4 - current.size() / 4;

            for (int i = 0; i < prev_n; ++i)
            {
                auto o = previous[i + center];
                overlapped[i] = o;
            }

            int next_n = current.size() / 2;
            for (int i = 0; i < next_n; ++i)
            {
                auto o = previous[i + center + prev_n] * w[next_n - i - 1] + current[i] * w[i];
                overlapped[i + prev_n] = o;
            }
        }
        else // current > previous
        {
            const auto& w = window0;

            // short - long
            int center = previous.size() / 2;
            int prev_n = previous.size() / 2;
            int begin = current.size() / 4 - previous.size() / 4;

            for (int i = 0; i < prev_n; ++i)
            {
                auto o = previous[center + i] * w[prev_n - i - 1] + current[begin + i] * w[i];
                overlapped[i] = o;
            }

            int next_n = size - prev_n;
            for (int i = 0; i < next_n; ++i)
            {
                auto o = current[begin + prev_n + i];
                overlapped[i + prev_n] = o;
            }
        }

        return overlapped;
    }

    void decode()
    {
        if (!primed)
        {
            previous = read_packet();
            primed = true;
        }

        auto current = read_packet();

        raw_samples.clear();
        raw_samples.reserve(channels);

        if (current[0].size() == 0)
        {
            for (int i = 0; i < channels; ++i)
                raw_samples.emplace_back(0);

            samples_read = 0;
            return;
        }

        for (int i = 0; i < channels; ++i)
        {
            raw_samples.push_back(overlap(current[i], previous[i]));
        }

        previous = std::move(current);
        samples_read = 0;
    }

    int decode(std::int32_t* data, std::size_t samples)
    {
        if (samples_read == -1)
            decode();

        int samples_needed = samples;
        int block_size = raw_samples[0].size();
        int sample_count = 0;
        while (samples_needed > 0 && block_size != 0)
        {
            int samples_available = block_size - samples_read;
            if (samples_available <= 0)
            {
                decode();
                block_size = raw_samples[0].size();
                samples_available = block_size;
            }

            int samples_to_be_read = std::min(samples_available, samples_needed);

            int s1 = samples_read;
            int s2 = samples_read + samples_to_be_read;
            for (int ch = 0; ch < channels; ++ch)
            {
                auto i = data + ch;
                const auto& rs = raw_samples[ch];
                for (int s = s1; s < s2; ++s, i += channels, ++sample_count)
                {
                    *i = anyplayer::helper::float_to_signed(rs[s]);
                }
            }

            samples_read += samples_to_be_read;
            samples_needed -= samples_to_be_read;
            data += samples_to_be_read * channels;
            position += samples_to_be_read;
        }

        return sample_count / channels;
    }

};

anyplayer::vorbis_decoder::vorbis_decoder(std::string path)
    : internal_(std::make_unique<internal>(std::make_unique<anyplayer::file_stream>(path, anyplayer::file_stream::r)))

{
}

anyplayer::vorbis_decoder::vorbis_decoder(std::unique_ptr<anyplayer::stream> stream)
    : internal_(std::make_unique<internal>(std::move(stream)))
{

}

anyplayer::vorbis_decoder::~vorbis_decoder()
{

}

int anyplayer::vorbis_decoder::get_sample_rate() const noexcept
{
    return internal_->sample_rate;
}

int anyplayer::vorbis_decoder::get_channels() const noexcept
{
    return internal_->channels;
}

int anyplayer::vorbis_decoder::decode(std::int32_t* data, std::size_t samples)
{
    return internal_->decode(data, samples);
}

bool anyplayer::vorbis_decoder::seek(int sample)
{
    internal_->reset();
    internal_->samples_read = -1;
    internal_->primed = false;
    internal_->position = sample;

    return true;
}

std::multimap<std::string, std::string> anyplayer::vorbis_decoder::get_info() const noexcept
{
    return internal_->comments;
}

std::uint64_t anyplayer::vorbis_decoder::get_length() const noexcept
{
    return (internal_->total_samples * 1000) / get_sample_rate();
}

std::uint64_t anyplayer::vorbis_decoder::get_position() const noexcept
{
    return (internal_->position * 1000) / get_sample_rate();
}

