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

#include "oggstream.hpp"

#include <vector>
#include <algorithm>
#include <numeric>
#include <map>

#include "crc.hpp"
#include <memory>
#include "endian.hpp"

namespace
{
    struct ogg_page
    {
    public:
        struct segment
        {
            int offset;
            int count;
        };

        ogg_page() {}
        ogg_page(anyplayer::stream& stream);

        bool continued;
        bool begin_of_stream;
        bool end_of_stream;
        std::uint64_t absolute_granule_position;
        std::uint32_t serial_number;
        std::uint32_t page_counter;
        std::vector<segment> segments;
        bool has_continuing_pages;
        std::vector<std::uint8_t> data;
        bool is_valid = false;
        int length;
    };

    anyplayer::crc<32, 0x04c11db7> crc;

    std::vector<ogg_page::segment> get_segments(const std::vector<std::uint8_t>& buffer, std::uint8_t lacing_values)
    {
        std::vector<ogg_page::segment> segments;
        segments.reserve(255);

        int current_segment_offset = 0;
        int current_segment_size = 0;

        for (uint8_t i = 0; i < lacing_values; ++i)
        {
            current_segment_size += buffer[i];

            if (buffer[i] < 255 || i == lacing_values - 1)
            {
                segments.push_back({current_segment_offset, current_segment_size});

                current_segment_offset += current_segment_size;
                current_segment_size = 0;
            }
        }

        return segments;
    }
}

ogg_page::ogg_page(anyplayer::stream &stream)
{
    auto header = anyplayer::read<std::uint8_t>(stream, 27);

    if (   header.size() == 27
        && header[0] == 'O'
        && header[1] == 'g'
        && header[2] == 'g'
        && header[3] == 'S')
    {
        auto version = header[4];
        if (version == 0)
        {
            continued = (header[5] & 0b001) == 0b001;
            begin_of_stream = (header[5] & 0b010) == 0b010;
            end_of_stream = (header[5] & 0b100) == 0b100;

            absolute_granule_position = anyplayer::swap_le<std::uint64_t>(&header[6]);
            serial_number = anyplayer::swap_le<std::uint32_t>(&header[14]);
            page_counter = anyplayer::swap_le<std::uint32_t>(&header[18]);
            auto page_crc = anyplayer::swap_le<std::uint32_t>(&header[22]);
            auto lacing_values = header[26];
            auto lacing_buffer = anyplayer::read<std::uint8_t>(stream, lacing_values);
            if (lacing_buffer.size() == lacing_values)
            {
                segments = get_segments(lacing_buffer, lacing_values);

                has_continuing_pages = lacing_buffer[lacing_values - 1] == 255;

                auto data_size = std::accumulate(segments.begin(), segments.end(), 0u, [](uint32_t a, const segment& s) { return a + s.count; });
                data = anyplayer::read<std::uint8_t>(stream, data_size);
                if (data.size() == data_size)
                {
                    length = 27 + lacing_values + data_size;

                    *reinterpret_cast<uint32_t*>(&header[22]) = 0;

                    is_valid = crc.hash(data.begin(), data.end(),
                              crc.hash(lacing_buffer.begin(), lacing_buffer.end(),
                              crc.hash(header.begin(), header.end(), 0))) == page_crc;
                }
            }
        }
    }
}

struct anyplayer::ogg_decoder::internal
{
    anyplayer::stream& stream;

    std::unique_ptr<ogg_page> current_page;
    size_t current_segment = 0;

    internal(anyplayer::stream& s) : stream(s), current_page(std::make_unique<ogg_page>()) {}

    void reset()
    {
        if (stream.seekable())
        {
            stream.seek(0, anyplayer::stream::begin);
            current_page = std::make_unique<ogg_page>();
        }
    }

    std::vector<std::uint8_t> get_packet()
    {
        // ATTENTION: we ignore more than one logical stream

        if (!current_page->is_valid)
            current_page = std::make_unique<ogg_page>(stream);

        std::vector<std::unique_ptr<ogg_page>> pages;
        while (   current_page->is_valid
               && current_page->has_continuing_pages
               && current_page->segments.size() == current_segment + 1)
        {
            pages.push_back(std::move(current_page));
            current_page = std::make_unique<ogg_page>(stream);
            current_segment = 0;
        }

        if (pages.size() > 0)
        {
            // we have a multi page packet

            uint32_t packet_size = std::accumulate(std::begin(pages), std::end(pages), 0u,
                                                   [](auto sum, const auto& page)
            {
                return sum + page->segments.back().count;
            });

            ogg_page::segment current_seg;
            if (current_page->is_valid)
            {
                current_seg = current_page->segments.front();
                packet_size += current_seg.count;
            }

            std::vector<uint8_t> packet(packet_size);

            int offset = 0;
            for (const auto& page : pages)
            {
                auto segment = page->segments.back();
                std::copy_n(page->data.begin() + segment.offset, segment.count, packet.begin() + offset);
                offset += segment.count;
            }

            if (current_page->is_valid)
            {
                std::copy_n(current_page->data.begin(), current_seg.count, packet.begin() + offset);
            }

            if (current_page->is_valid && current_page->segments.size() > 1)
            {
                current_segment = 1;
            }
            else
            {
                if (!current_page->end_of_stream)
                {
                    current_page = std::make_unique<ogg_page>(stream);
                    current_segment = 0;
                }
                else
                {
                    current_page = std::make_unique<ogg_page>();
                    current_segment = 0;
                }
            }

            return packet;
        }

        if (current_page->is_valid)
        {
            // we have a single page packet

            auto current_seg = current_page->segments[current_segment];

            std::vector<uint8_t> packet(current_seg.count);
            std::copy_n(current_page->data.begin() + current_seg.offset, current_seg.count, packet.begin());

            if (current_segment < current_page->segments.size() - 1)
            {
                ++current_segment;
            }
            else
            {
                if (!current_page->end_of_stream)
                {
                    current_page = std::make_unique<ogg_page>(stream);
                    current_segment = 0;
                }
                else
                {
                    current_page = std::make_unique<ogg_page>();
                    current_segment = 0;
                }
            }

            return packet;
        }

        return {};
    }
};

anyplayer::ogg_decoder::ogg_decoder(anyplayer::stream &stream)
    : internal_(std::make_unique<internal>(stream))
{
}

anyplayer::ogg_decoder::~ogg_decoder() {}

void anyplayer::ogg_decoder::reset()
{
    internal_->reset();
}

std::vector<std::uint8_t> anyplayer::ogg_decoder::get_packet()
{
    return internal_->get_packet();
}
