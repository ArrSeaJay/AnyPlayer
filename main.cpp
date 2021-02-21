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

#define SDL_MAIN_HANDLED
#include <SDL2/SDL.h>

#include <iostream>
#include <iomanip>
#include <cstdlib>

#include "decoder.hpp"

void callback(void* user, Uint8* data, int len)
{
    SDL_memset(data, 0, len);
    auto decoder = static_cast<anyplayer::decoder*>(user);

    auto bytesPerSample = decoder->get_channels() * sizeof (std::int32_t);

    decoder->decode(reinterpret_cast<std::int32_t*>(data), len / bytesPerSample);
}

void print_time(std::uint64_t t)
{
    auto md = std::div(t, 60 * 1000);
    auto sd = std::div(md.rem, 1000);

    int m = md.quot;
    int s = sd.quot;
    int ms = sd.rem;

    std::cout << std::setfill('0')
              << std::setw(2) << m
              << ':'
              << std::setw(2) << s
              << ':'
              << std::setw(3) << ms;
}

int main(int argc, char** argv)
{
    SDL_SetMainReady();
    SDL_Init(SDL_INIT_AUDIO);

    if (argc != 2)
    {
        return 0;
    }

    auto decoder = anyplayer::create_decoder(argv[1]);
    if (!decoder)
    {
        return 0;
    }

    auto info = decoder->get_info();
    std::for_each(info.cbegin(), info.cend(), [](const auto& t)
    {
        std::cout << t.first << "\t" << t.second << std::endl;
    });

    SDL_AudioSpec want;

    SDL_memset(&want, 0, sizeof(want));
    want.freq = decoder->get_sample_rate();
    want.format = AUDIO_S32;
    want.channels = decoder->get_channels();
    want.samples = 4096;
    want.userdata = decoder.get();
    want.callback = callback;

    auto dev = SDL_OpenAudioDevice(nullptr, 0, &want, nullptr, 0);

    SDL_PauseAudioDevice(dev, 0);

    while (decoder->get_position() < decoder->get_length())
    {
        std::cout << "\r";
        print_time(decoder->get_position());
        std::cout << " / ";
        print_time(decoder->get_length());
        std::cout << std::flush;

        SDL_Delay(100);
    }

    SDL_CloseAudioDevice(dev);

    SDL_Quit();

    return 0;
}
