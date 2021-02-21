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

#include "decoder.hpp"

#include "vorbisdecoder.hpp"
#include "flacdecoder.hpp"

namespace anyplayer
{

std::unique_ptr<decoder> create_decoder(const std::string& path)
{
    if (path.ends_with("ogg"))
    {
        return std::make_unique<vorbis_decoder>(path);
    }
    else if (path.ends_with("flac"))
    {
        return std::make_unique<flac_decoder>(path);
    }

    return {};
}

}
