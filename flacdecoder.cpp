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

#include "flacdecoder.hpp"

#include <string>
#include <algorithm>
#include <assert.h>
#include <functional>

#include "stream.hpp"
#include "filestream.hpp"

#include "memorystream.hpp"
#include "crc.hpp"
#include "vorbisdecoder.hpp"
#include "base64.hpp"
#include "copyiterator.hpp"

#include "helper.hpp"

namespace
{
anyplayer::crc<8, 0x7> crc8;
anyplayer::crc<16, 0x8005> crc16;
}

struct anyplayer::flac_decoder::internal
{
    enum class channel_assignment
    {
        independent,
        right_side,
        left_side,
        mid_side
    };

    anyplayer::stream_iterator stream_;

    int min_blocksize;
    int max_blocksize;
    int min_frame_size;
    int max_frame_size;
    int sample_rate;
    int channels;
    int bits_per_sample;
    uint64_t total_samples;
    std::vector<std::uint8_t> md5;
    std::multimap<std::string, std::string> comments;

    int sample_size;
    int block_size;
    channel_assignment assignment;
    int wasted_bits;

    std::vector<std::vector<int>> raw_samples;
    int samples_read;

    std::uint64_t position;

    long stream_begin_pos;

    internal(std::unique_ptr<anyplayer::stream> s)
        : stream_(std::move(s))
        , block_size(-1)
        , position(0)
    {
        auto id = anyplayer::read_be<32>(stream_);
        if (id != 0x664c6143)
        {
            throw decoder_exception("'fLaC' in header not found.");
        }

        decode_meta_data();

        stream_begin_pos = stream_;
    }

    void decode_meta_data()
    {
        bool last_meta_data_block = false;
        auto reader = anyplayer::make_ref_bit_iterator<true>(stream_);

        while (!last_meta_data_block)
        {
            last_meta_data_block = *reader++;
            auto block_type = anyplayer::read_int<7>(reader);
            int len = anyplayer::read_int<24>(reader);

            switch(block_type)
            {
            case 0:
                min_blocksize = anyplayer::read_int<16>(reader);
                max_blocksize = anyplayer::read_int<16>(reader);
                min_frame_size = anyplayer::read_int<24>(reader);
                max_frame_size = anyplayer::read_int<24>(reader);
                sample_rate = anyplayer::read_int<20>(reader);
                channels = anyplayer::read_int<3>(reader) + 1;
                bits_per_sample = anyplayer::read_int<5>(reader) + 1;
                total_samples = anyplayer::read_int<36>(reader);
                md5 = anyplayer::read_ints<8>(reader, 128 / 8);
                break;

            case 1:
            {
                auto zero = anyplayer::read_ints<8>(reader, len);
                auto count = std::count(zero.begin(), zero.end(), 0);
                if (count != len)
                {
                    throw decoder_exception();
                }
            }
                break;

            case 4:
            {
                auto le_reader = anyplayer::make_ref_bit_iterator<false>(stream_);
                auto c = read_vorbis_comments(le_reader);
                comments.insert(c.begin(), c.end());
            }
                break;

            case 6:
                comments.emplace("METADATA_BLOCK_PICTURE", to_base_64(anyplayer::read_ints<8>(reader, len)));
                break;

            default:
                anyplayer::read_ints<8>(reader, len);
                break;
            }
        }
    }

    template<typename iterator>
    int get_block_size(int bs, anyplayer::bit_iterator<iterator, true>& copy_reader)
    {
        switch (bs)
        {
        case 1:
            return 192;
        case 2:
        case 3:
        case 4:
        case 5:
            return 576 << (bs - 2);
        case 6:
            return anyplayer::read_int<8>(copy_reader) + 1;
        case 7:
            return anyplayer::read_int<16>(copy_reader) + 1;
        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        case 13:
        case 14:
        case 15:
            return 256 << (bs - 8);
        }

        throw decoder_exception();
    }

    template<typename iterator>
    int get_sample_rate(int sr, anyplayer::bit_iterator<iterator, true>& copy_reader)
    {
        switch (sr)
        {
        case 0:
            return this->sample_rate;
        case 1:
            return 88200;
        case 2:
            return 176400;
        case 3:
            return 192000;
        case 4:
            return 8000;
        case 5:
            return 16000;
        case 6:
            return 22050;
        case 7:
            return 24000;
        case 8:
            return 32000;
        case 9:
            return 44100;
        case 10:
            return 48000;
        case 11:
            return 96000;
        case 12:
            return anyplayer::read_int<8>(copy_reader) * 1000;
        case 13:
            return anyplayer::read_int<16>(copy_reader);
        case 14:
            return anyplayer::read_int<16>(copy_reader) / 10;
        case 15:
            throw decoder_exception();
        }

        return 0;
    }

    template<typename iterator>
    channel_assignment get_assignment(anyplayer::bit_iterator<iterator, true>& copy_reader)
    {
        int channel_assignment = anyplayer::read_int<4>(copy_reader);

        if (channel_assignment <= 7)
        {
            if (channel_assignment + 1 == channels)
            {
                return channel_assignment::independent;
            }
        }

        if (channels == 2)
        {
            switch (channel_assignment)
            {
                case 8:
                    return channel_assignment::left_side;
                case 9:
                    return channel_assignment::right_side;
                case 10:
                    return channel_assignment::mid_side;
            }
        }

        throw decoder_exception();
    }

    template<typename iterator>
    int get_sample_size(anyplayer::bit_iterator<iterator, true>& copy_reader)
    {
        auto ss = anyplayer::read_int<3>(copy_reader);
        switch (ss)
        {
            case 0:
                return bits_per_sample;
            case 1:
                return 8;
            case 2:
                return 12;
            case 4:
                return 16;
            case 5:
                return 20;
            case 6:
                return 24;
            default:
                throw decoder_exception();
        }
    }

    template<typename iterator>
    std::vector<int> get_residuals(int block_size, int predictor_order, anyplayer::bit_iterator<iterator, true>& copy_reader)
    {
        auto residual_coding_method = anyplayer::read_int<2>(copy_reader);
        if (residual_coding_method != 0 && residual_coding_method != 1)
        {
            throw decoder_exception();
        }
        int partition_order = anyplayer::read_int<4>(copy_reader);
        int partitions = 1 << partition_order;
        int partition_samples = partition_order > 0 ? block_size >> partition_order : block_size - predictor_order;
        std::vector<int> residues(block_size - predictor_order);
        int sample = 0;
        int rice_precision = residual_coding_method == 0 ? 4 : 5;
        for (int partition = 0; partition < partitions; partition++)
        {
            int rice_param = anyplayer::read_int(copy_reader, rice_precision);
            if ((rice_param == 15 && residual_coding_method == 0) || (rice_param == 31 && residual_coding_method == 1))
            {
                int bits_per_sample_partition = anyplayer::read_int<5>(copy_reader);
                for (int u = (partition_order == 0 || partition > 0) ? 0 : predictor_order; u < partition_samples; u++, sample++)
                {
                    residues[sample] = anyplayer::read_int(copy_reader, bits_per_sample_partition);
                }
            }
            else
            {
                int u = (partition_order == 0 || partition > 0) ? partition_samples : partition_samples - predictor_order;

                for (int i = 0; i < u; ++i)
                {
                    // Rice coding
                    auto q = anyplayer::consecutive(copy_reader, false);
                    auto r = anyplayer::read_int(copy_reader, rice_param);
                    auto n = (q << rice_param) | r;
                    residues[sample + i] = n % 2 == 0 ? n >> 1 : -(int)((n >> 1) + 1);
                }

                sample += u;
            }
        }

        assert(sample + predictor_order == block_size);

        return residues;
    }

    template<typename iterator>
    std::vector<int> decode_fixed_subframe(int block_size, int order, int bps, anyplayer::bit_iterator<iterator, true>& copy_reader)
    {
        std::vector<int> samples(block_size);

        for (int i = 0; i < order; ++i)
        {
            samples[i] = anyplayer::helper::unsigned_to_signed(anyplayer::read_int(copy_reader, bps), bps);
        }

        auto residues = get_residuals(block_size, order, copy_reader);

        switch (order)
        {
            case 0:
                std::copy(residues.begin(), residues.end(), samples.begin());
                break;
            case 1:
                for (size_t i = 0; i < residues.size(); i++)
                {
                    samples[i + 1] = residues[i] + samples[i];
                }
                break;
            case 2:
                for (size_t i = 0; i < residues.size(); i++)
                {
                    samples[i + 2] = residues[i] + 2 * samples[i + 1] - samples[i];
                }
                break;
            case 3:
                for (size_t i = 0; i < residues.size(); i++)
                {
                    samples[i + 3] = residues[i] + 3 * samples[i + 2] - 3 * samples[i + 1] + samples[i];
                }
                break;
            case 4:
                for (size_t i = 0; i < residues.size(); i++)
                {
                    samples[i + 4] = residues[i] + 4 * samples[i + 3] - 6 * samples[i + 2] + 4 * samples[i + 1] - samples[i];
                }
                break;
            default:
                throw decoder_exception();
        }

        return samples;
    }

    template<typename iterator>
    std::vector<int> decode_lpcsubframe(int block_size, int order, int bps, anyplayer::bit_iterator<iterator, true>& copy_reader)
    {
        std::vector<int> samples(block_size);

        for (int i = 0; i < order; ++i)
        {
            samples[i] = anyplayer::helper::unsigned_to_signed(anyplayer::read_int(copy_reader, bps), bps);
        }

        auto precision = anyplayer::read_int<4>(copy_reader) + 1;
        if (precision == 16)
        {
            throw decoder_exception();
        }

        auto quantization_level = anyplayer::helper::unsigned_to_signed(anyplayer::read_int<5>(copy_reader), 5);

        std::vector<int> coefficients(order);
        for (int i = 0; i < order; i++)
        {
            coefficients[i] = anyplayer::helper::unsigned_to_signed(anyplayer::read_int(copy_reader, precision), precision);
        }

        auto residues = get_residuals(block_size, order, copy_reader);

        for (size_t i = 0; i < residues.size(); i++)
        {
            int sum = 0;
            int k = order + i;

            for (int j = 0; j < order; j++)
            {
                sum += coefficients[j] * samples[k - j - 1];
            }

            samples[k] = residues[i] + (sum >> quantization_level);
        }

        return samples;
    }

    int get_bits_per_sample(int channel, channel_assignment assignment, int sample_size)
    {
        return ((assignment == channel_assignment::left_side || assignment == channel_assignment::mid_side)
        && channel == 1)
        || (assignment == channel_assignment::right_side && channel == 0)
        ? sample_size + 1
        : sample_size;
    }

    template<typename iterator>
    std::vector<int> decode_subframe(int block_size, int ch, channel_assignment assignment, int sample_size, anyplayer::bit_iterator<iterator, true>& copy_reader)
    {
        int bps = get_bits_per_sample(ch, assignment, sample_size);

        if (*copy_reader++)
        {
            throw decoder_exception();
        }

        auto sftype = anyplayer::read_int<6>(copy_reader);

        wasted_bits = 0;
        if (*copy_reader++)
        {
            wasted_bits = anyplayer::consecutive(copy_reader, false) + 1;
        }

        if (sftype == 0)
        {
            int constant = anyplayer::helper::unsigned_to_signed(anyplayer::read_int(copy_reader, bps), bps);
            return std::vector<int>(block_size, constant);
        }
        else if (sftype == 1)
        {
            std::vector<int> samples;
            samples.reserve(block_size);
            for (int i = 0; i < block_size; ++i)
            {
                samples.push_back(anyplayer::helper::unsigned_to_signed(anyplayer::read_int(copy_reader, bps), bps));
            }
            return samples;
        }
        else if (sftype >= 8 && sftype <= 12)
        {
            return decode_fixed_subframe(block_size, sftype - 8, bps, copy_reader);
        }
        else
        {
            return decode_lpcsubframe(block_size, sftype - 31, bps, copy_reader);
        }
    }

    void decode()
    {
        std::vector<std::uint8_t> copy_buffer;
        copy_buffer.reserve(std::max(max_frame_size, 4096));
        auto copy = anyplayer::make_ref_copy(stream_, std::back_inserter(copy_buffer));
        auto copy_reader = anyplayer::make_ref_bit_iterator<true>(copy);

        // read header
        auto sync = anyplayer::read_int<14>(copy_reader);

        if (!stream_)
        {
            block_size = 0;
            samples_read = 0;
            return;
        }

        if (sync != 0x3ffe)
        {
            throw decoder_exception();
        }

        if (*copy_reader++)
        {
            throw decoder_exception();
        }

        /*auto blockingStrategy = *copy_reader++;*/
        ++copy_reader;
        int bs = anyplayer::read_int<4>(copy_reader);
        int sr = anyplayer::read_int<4>(copy_reader);

        assignment = get_assignment(copy_reader);
        sample_size = get_sample_size(copy_reader);

        if (*copy_reader++)
        {
            throw decoder_exception();
        }

        /*auto currentFrame = */anyplayer::read_utf8([&](){return anyplayer::read_int<8>(copy_reader);});

        block_size = get_block_size(bs, copy_reader);
        /*int sampleRate = */get_sample_rate(sr, copy_reader);

        auto crc_header = anyplayer::read_int<8>(copy_reader);
        if (crc_header != crc8.hash(copy_buffer.begin(), copy_buffer.end() - 1, 0))
        {
            throw decoder_exception();
        }

        // read subframes
        raw_samples.clear();
        raw_samples.reserve(channels);
        for (int ch = 0; ch < channels; ++ch)
        {
            raw_samples.push_back(decode_subframe(block_size, ch, assignment, sample_size, copy_reader));
        }

        // read zero padding
        if (copy_reader.current_bit() != 0)
        {
            auto zero_padding = anyplayer::read_int(copy_reader, 8 - copy_reader.current_bit());
            if (zero_padding != 0)
            {
                throw decoder_exception();
            }
        }

        auto crc_frame = anyplayer::read_int<16>(copy_reader);
        if (crc_frame != crc16.hash(copy_buffer.begin(), copy_buffer.end() - 2, 0))
        {
            throw decoder_exception();
        }

        samples_read = 0;
    }

    std::int32_t transpose(int sample)
    {
        return sample << (32 - bits_per_sample);
    }

    int decode(std::int32_t* data, std::size_t samples)
    {
        if (block_size == 0)
            return 0;
        if (block_size == -1)
            decode();

        int samples_needed = samples;
        int sample_count = 0;
        while (samples_needed > 0 && block_size != 0)
        {
            int samples_available = block_size - samples_read;
            if (samples_available <= 0)
            {
                decode();
                samples_available = block_size;
            }

            int samples_to_be_read = std::min(samples_available, samples_needed);

            int s1 = samples_read;
            int s2 = samples_read + samples_to_be_read;
            switch (assignment)
            {
            case channel_assignment::independent:
                for (int ch = 0; ch < channels; ++ch)
                {
                    auto i = data + ch;
                    const auto& rs = raw_samples[ch];

                    for (int s = s1; s < s2; ++s, i += channels, ++sample_count)
                    {
                        *i = transpose(rs[s]);
                    }
                }
                data += samples_to_be_read * channels;
                break;
            case channel_assignment::left_side:
                for (int s = s1; s < s2; ++s)
                {
                    auto left = raw_samples[0][s];
                    auto difference = raw_samples[1][s];
                    auto right = left - difference;
                    data[0] = transpose(left);
                    data[1] = transpose(right);
                    data += 2;
                    sample_count += 2;
                }
                break;
            case channel_assignment::right_side:
                for (int s = s1; s < s2; ++s)
                {
                    auto difference = raw_samples[0][s];
                    auto right = raw_samples[1][s];
                    auto left = difference + right;
                    data[0] = transpose(left);
                    data[1] = transpose(right);
                    data += 2;
                    sample_count += 2;
                }
                break;
            case channel_assignment::mid_side:
                for (int s = s1; s < s2; ++s)
                {
                    auto mid = raw_samples[0][s];
                    auto side = raw_samples[1][s];
                    auto left = (((mid << 1) | (side & 1)) + side) >> 1;
                    auto right = (((mid << 1) | (side & 1)) - side) >> 1;
                    data[0] = transpose(left);
                    data[1] = transpose(right);
                    data += 2;
                    sample_count += 2;
                }
                break;
            }

            samples_read += samples_to_be_read;
            samples_needed -= samples_to_be_read;
            position += samples_to_be_read;
        }

        return sample_count / channels;
    }
};

anyplayer::flac_decoder::flac_decoder(std::string path)
    : internal_(std::make_unique<internal>(std::make_unique<anyplayer::file_stream>(path, anyplayer::file_stream::r)))
{
}

anyplayer::flac_decoder::flac_decoder(std::unique_ptr<anyplayer::stream> stream_)
    : internal_(std::make_unique<internal>(std::move(stream_)))
{
}

anyplayer::flac_decoder::~flac_decoder()
{

}

int anyplayer::flac_decoder::get_sample_rate() const noexcept
{
    return internal_->sample_rate;
}

int anyplayer::flac_decoder::get_channels() const noexcept
{
    return internal_->channels;
}

int anyplayer::flac_decoder::decode(std::int32_t* data, std::size_t samples)
{
    return internal_->decode(data, samples);
}

bool anyplayer::flac_decoder::seek(int sample)
{
    internal_->stream_ = internal_->stream_begin_pos;
    internal_->block_size = -1;
    internal_->samples_read = -1;
    internal_->position = sample;

    return true;
}

std::multimap<std::string, std::string> anyplayer::flac_decoder::get_info() const noexcept
{
    return internal_->comments;
}

std::uint64_t anyplayer::flac_decoder::get_length() const noexcept
{
    return (internal_->total_samples * 1000) / get_sample_rate();
}

std::uint64_t anyplayer::flac_decoder::get_position() const noexcept
{
    return (internal_->position * 1000) / get_sample_rate();
}

