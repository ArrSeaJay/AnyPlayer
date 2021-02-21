# AnyPlayer - Demonstration of how a FLAC and OggVorbis decoder works
# Copyright (C) 2021 Ralf-Christian Jürgensen

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.17)

project(AnyPlayer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(SDL2 REQUIRED)

add_executable(AnyPlayer
    base64.cpp
    decoder.cpp
    inflate.cpp
    flacdecoder.cpp
    main.cpp
    oggstream.cpp
    vorbisdecoder.cpp
    adler32.hpp
    base64.hpp
    bititerator.hpp
    buffer.hpp
    copyiterator.hpp
    crc.hpp
    decoder.hpp
    inflate.hpp
    endian.hpp
    filestream.hpp
    flacdecoder.hpp
    huffman.hpp
    iteratorhelper.hpp
    mdct.hpp
    memorystream.hpp
    meta.hpp
    oggstream.hpp
    stream.hpp
    vorbisdecoder.hpp
    helper.hpp
)
target_link_libraries(AnyPlayer PRIVATE SDL2::SDL2 SDL2::SDL2main)
