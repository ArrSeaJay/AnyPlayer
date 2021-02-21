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

#ifndef MEMORY_STREAM_HPP
#define MEMORY_STREAM_HPP

#include "stream.hpp"

#include <iterator>
#include <algorithm>
#include <memory>

namespace anyplayer
{
class memory_stream : public stream
{
public:
    memory_stream(size_t size)
        : memory_(size)
    {
        position_ = memory_.begin();
    }

    memory_stream(std::vector<uint8_t>&& buffer)
        : memory_(std::move(buffer))
    {
        position_ = memory_.begin();
    }

    memory_stream& operator=(std::vector<uint8_t>&& buffer)
    {
        memory_ = std::move(buffer);
        position_ = memory_.begin();
        return *this;
    }

    const std::vector<uint8_t>& Data() const
    {
        return memory_;
    }

    virtual size_t read(void* buffer, size_t offset, size_t bytes) override
    {
        bytes = std::min(bytes, (size_t)std::distance(position_, memory_.end()));

        std::copy_n(position_, bytes, reinterpret_cast<std::uint8_t*>(buffer) + offset);
        position_ += bytes;

        return bytes;
    }

/*
    virtual size_t write(std::vector<uint8_t> buffer, size_t offset, size_t bytes) override
    {
        bytes = std::min(buffer.size() - offset, bytes);

        int rest = std::distance(position_, memory_.end());
        int dist = bytes - rest;

        auto begin = buffer.begin() + offset;

        if (dist <= 0)
        {
            position_ = std::copy_n(begin, bytes, position_);
        }
        else
        {
            std::copy_n(begin, dist, position_);

            std::copy_n(begin + dist, rest, std::back_inserter(memory_));

            position_ = memory_.end();
        }

        return bytes;
    }
*/
    virtual long position() override
    {
        return std::distance(position_, memory_.begin());
    }

    virtual void seek(long offset, origin o) override
    {
        switch(o)
        {
        case current:
            position_ += offset;
            break;
        case begin:
            position_ = memory_.begin() + offset;
            break;
        case end:
            position_ = memory_.end() - offset;
        }
    }

    virtual bool eos() override
    {
        return position_ == memory_.end();
    }

    virtual std::uint8_t peek() override
    {
        return *position_;
    }

    virtual bool seekable() const override
    {
        return true;
    }

private:
    std::vector<uint8_t> memory_;
    std::vector<uint8_t>::iterator position_;
};
}

#endif // MEMORY_STREAM_HPP
