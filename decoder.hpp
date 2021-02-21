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

#ifndef DECODER_HPP
#define DECODER_HPP

#include <string>
#include <map>
#include <memory>

namespace anyplayer
{

class decoder_exception
{
    std::string message_;

public:
    decoder_exception() : message_("unspecified") {}
    decoder_exception(const std::string& message) : message_(message) {}

    std::string get_message() const { return message_; }
};

/**
 * @brief The decoder class
 *
 * Interface to generalize access to decoders of different kinds.
 *
 *
 */
class decoder
{
public:
    // must be implemented by all decoders
    /// Get sample rate in Hz
    virtual int get_sample_rate() const noexcept = 0;
    /// Get number of channels
    virtual int get_channels() const noexcept = 0;
    /// Decode samples
    virtual int decode(std::int32_t* data, std::size_t samples) = 0;

    // for seekable formats (loaded from file)
    /// Get length of current track/song in ms.
    virtual std::uint64_t get_length() const noexcept { return 0; }
    /// Get current position in current track/song in ms.
    virtual std::uint64_t get_position() const noexcept { return 0; }
    /// seek to position in ms.
    virtual bool seek(int /* ms */) { return false; }

    // for formats with subtracks (CD, Cuesheet, sid, tfmx, sc68)
    /// Get number of tracks available; must be at least 1!
    virtual int get_tracks() const noexcept { return 1; }
    /// Get current track being played; is at least 0 for trackless formats.
    virtual int get_track() const noexcept { return 0; }
    /// Change current track. Track is guaranteed to be in range [0, Tracks[
    virtual void set_track(int /* track */) {}

    /// Get information of current track/song
    virtual std::multimap<std::string, std::string> get_info() const noexcept { return {}; }

    virtual bool get_loop() const noexcept { return false; }
    virtual void set_loop(bool /* loop */) {}

    virtual ~decoder() {}
};

std::unique_ptr<decoder> create_decoder(const std::string& path);

}

#endif // DECODER_HPP
