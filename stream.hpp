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

#ifndef STREAM_HPP
#define STREAM_HPP

#include <vector>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <algorithm>
#include <iterator>

#include "endian.hpp"
#include "bititerator.hpp"
#include "buffer.hpp"

namespace anyplayer
{

class stream
{
public:
    virtual ~stream() = default;

    virtual long position() = 0;

    enum origin { begin, current, end };

    virtual void seek(long offset, origin o) = 0;

    virtual bool eos() = 0;

    virtual std::uint8_t peek() = 0;

    virtual bool seekable() const = 0;

    virtual size_t read(void* buffer, size_t offset, size_t bytes) = 0;
};

template<typename T>
size_t read(stream& s, std::vector<T>& buffer, size_t offset, size_t n)
{
    return s.read(reinterpret_cast<void*>(buffer.data()), offset * sizeof(T), n * sizeof(T)) / sizeof(T);
}

template<typename T>
size_t read(stream& s, anyplayer::buffer<T>& buffer, size_t offset, size_t n)
{
    return s.read(buffer.raw(), offset * sizeof(T), n * sizeof(T)) / sizeof(T);
}

template<typename T>
std::vector<T> read(stream& s, size_t n)
{
    std::vector<T> buffer(n);

    auto read_size = read(s, buffer, 0, n);
    buffer.resize(read_size);

    return buffer;
}

template<typename T>
std::vector<T> read_all(stream& s)
{
    if (s.seekable())
    {
        s.seek(0, anyplayer::stream::end);
        auto length = s.position() / sizeof(T);
        s.seek(0, anyplayer::stream::begin);
        return anyplayer::read<T>(s, length);
    }

    return {};
}

template<std::size_t Bits, bool BigEndian>
struct stream_read_int
{
    typedef typename bit_type<Bits>::type T;

    static T read(stream& s)
    {
        static thread_local T buffer;
        s.read(&buffer, 0, sizeof(T));
        return swap_le(buffer);
    }
};

template<std::size_t Bits>
struct stream_read_int<Bits, true>
{
    typedef typename bit_type<Bits>::type T;

    static T read(stream& s)
    {
        static thread_local T buffer;
        s.read(&buffer, 0, sizeof(T));
        return swap_be(buffer);
    }
};

inline std::uint8_t read_byte(stream& s)
{
    static thread_local std::uint8_t buffer;
    s.read(&buffer, 0, 1);
    return buffer;
}

template<std::size_t Bits>
typename bit_type<Bits>::type read_le(stream& s)
{
    return stream_read_int<Bits, false>::read(s);
}

template<std::size_t Bits>
typename bit_type<Bits>::type read_be(stream& s)
{
    return stream_read_int<Bits, true>::read(s);
}

class stream_iterator : public std::iterator<std::input_iterator_tag, std::uint8_t, long, void, void>
{
    static constexpr auto BufferSize = 4096;

public:
    stream_iterator()
        : stream_(nullptr)
        , buffer_()
        , pos_(buffer_.cbegin())
    {
    }

    stream_iterator(std::unique_ptr<stream>&& s)
        : stream_(std::move(s))
        , buffer_(BufferSize)
    {
        fill_buffer();
    }

    std::uint8_t operator *() const
    {
        return *pos_;
    }

    stream_iterator& operator ++()
    {
        if (pos_ != buffer_.cend() &&
          ++pos_ == buffer_.cend())
        {
            fill_buffer();
        }

        return *this;
    }

    dereferencable<std::uint8_t> operator ++(int)
    {
        dereferencable<std::uint8_t> tmp {*pos_};

        ++(*this);

        return tmp;
    }

    operator bool() const
    {
        return pos_ != buffer_.cend();
    }

    bool operator !() const
    {
        return pos_ == buffer_.cend();
    }

    operator long() const
    {
        if (stream_->seekable())
            return stream_->position() - std::distance(pos_, buffer_.cend());
        else
            return 0;
    }

    bool operator =(long position)
    {
        if (stream_->seekable())
        {
            stream_->seek(position, stream::begin);

            fill_buffer();

            return true;
        }

        return false;
    }

    friend bool operator ==(const stream_iterator& lhs, const stream_iterator& rhs)
    {
        if (lhs.stream_ && rhs.stream_)
        {
            // not possible
            return false;
        }
        else if (!lhs.stream_ && !rhs.stream_)
        {
            return true;
        }
        else if (lhs.stream_)
        {
            return lhs.pos_ == lhs.buffer_.cend();
        }
        else
        {
            return rhs.pos_ == rhs.buffer_.cend();
        }
    }

    friend std::uint8_t read_byte(stream_iterator& stream);

    template<std::size_t Bits>
    friend typename bit_type<Bits>::type read_le(stream_iterator& stream);

    template<std::size_t Bits>
    friend typename bit_type<Bits>::type read_be(stream_iterator& stream);

private:
    void advance(std::size_t bytes)
    {
        pos_ += bytes;
        if (pos_ == buffer_.cend())
        {
            fill_buffer();
        }
    }

    const std::uint8_t* get_current_data() const
    {
        return buffer_.data() + std::distance(buffer_.cbegin(), pos_);
    }

    void ensure(std::size_t bytes_needed)
    {
        std::size_t bytes_available = std::distance(pos_, buffer_.cend());

        if (bytes_available < bytes_needed)
        {
            std::copy(pos_, buffer_.cend(), buffer_.begin());

            fill_buffer(bytes_available);
        }
    }

    void fill_buffer(std::size_t offset = 0)
    {
        auto s = read(*stream_, buffer_, offset, BufferSize - offset);
        if (s != (BufferSize - offset))
        {
            buffer_.resize(s);
        }

        pos_ = buffer_.cbegin();
    }

    std::unique_ptr<stream> stream_;
    std::vector<std::uint8_t> buffer_;
    std::vector<std::uint8_t>::const_iterator pos_;
};

inline bool operator !=(const stream_iterator& lhs, const stream_iterator& rhs)
{
    return !(lhs == rhs);
}

inline std::uint8_t read_byte(stream_iterator& stream)
{
    std::uint8_t byte = *stream;
    ++stream;
    return byte;
}

template<std::size_t Bits>
typename bit_type<Bits>::type read_le(stream_iterator& stream)
{
    stream.ensure(Bits / 8);

    auto v = read_le<Bits>(stream.get_current_data());

    stream.advance(Bits / 8);

    return v;
}

template<std::size_t Bits>
typename bit_type<Bits>::type read_be(stream_iterator& stream)
{
    stream.ensure(Bits / 8);

    auto v = read_be<Bits>(stream.get_current_data());

    stream.advance(Bits / 8);

    return v;
}

}

#endif // STREAM_HPP
