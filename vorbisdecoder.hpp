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

#ifndef VORBISDECODER_HPP
#define VORBISDECODER_HPP

#include <string>
#include <memory>
#include <vector>

#include "decoder.hpp"
#include "bititerator.hpp"

namespace anyplayer
{
class stream;

class vorbis_decoder : public decoder
{
public:
    vorbis_decoder(std::string path);
    vorbis_decoder(std::unique_ptr<anyplayer::stream> stream);
    virtual ~vorbis_decoder();

    virtual int get_sample_rate() const noexcept final;
    virtual int get_channels() const noexcept final;
    virtual std::multimap<std::string, std::string> get_info() const noexcept final;
    virtual std::uint64_t get_length() const noexcept final;
    virtual std::uint64_t get_position() const noexcept final;

    virtual int decode(std::int32_t* data, std::size_t samples) final;

    virtual bool seek(int sample) final;

private:
    struct internal;
    std::unique_ptr<internal> internal_;
};

template<typename Iterator>
std::multimap<std::string, std::string> read_vorbis_comments(anyplayer::bit_iterator<Iterator, false>& reader)
{
    std::multimap<std::string, std::string> comments;

    auto vendorLength = anyplayer::read_int<32>(reader);
    auto vendorBuffer = anyplayer::read_ints<8>(reader, vendorLength);
    comments.insert({"vendor", {vendorBuffer.begin(), vendorBuffer.end()}});

    auto commentLength = anyplayer::read_int<32>(reader);
    for (size_t i = 0; i < commentLength; ++i)
    {
        auto len = anyplayer::read_int<32>(reader);

        auto buffer = anyplayer::read_ints<8>(reader, len);

        auto eq = std::find(buffer.begin(), buffer.end(), '=');

        std::string key {buffer.begin(), eq};

        if (eq != buffer.end())
        {
            ++eq;
        }

        comments.insert({key, {eq, buffer.end()}});
    }

    return comments;
}

}

#endif // VORBISDECODER_HPP
