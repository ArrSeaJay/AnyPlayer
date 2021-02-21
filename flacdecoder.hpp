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

#ifndef FLAC_DECODER_HPP
#define FLAC_DECODER_HPP

#include <memory>
#include <string>

#include "decoder.hpp"

namespace anyplayer
{
class stream;

class flac_decoder : public decoder
{
public:
    flac_decoder(std::string path);
    flac_decoder(std::unique_ptr<anyplayer::stream> Stream);
    virtual ~flac_decoder();

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

}

#endif // FLAC_DECODER_HPP
