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

#ifndef FILE_STREAM_HPP
#define FILE_STREAM_HPP

#include <string>
#include <memory>

#include "stream.hpp"

namespace anyplayer
{
class file_stream : public stream
{
public:
    enum mode { r, w };

    file_stream(std::string path, mode file_mode)
        : path_(path)
    {
        const char* m;
        switch(file_mode)
        {
        case r:
            m = "rb";
            break;
        case w:
            m = "wb";
            break;
        default:
            m = "r+b";
            break;
        }

        handle = fopen(path.c_str(), m);
        if (handle == nullptr)
        {
            std::perror(path.c_str());
        }
    }

    file_stream(file_stream&& other)
    {
        handle = other.handle;
        other.handle = nullptr;
    }

    file_stream& operator =(file_stream&& other)
    {
        handle = other.handle;
        other.handle = nullptr;

        return *this;
    }

    file_stream(const file_stream&) = delete;
    file_stream& operator =(const file_stream&) = delete;

    virtual ~file_stream()
    {
        if (handle != nullptr)
        {
            fclose(handle);
        }
    }

    virtual size_t read(void* buffer, size_t offset, size_t bytes) override
    {
        return fread(reinterpret_cast<std::uint8_t*>(buffer) + offset, 1, bytes, handle);
    }
/*
    virtual size_t write(std::vector<uint8_t> buffer, size_t offset, size_t bytes) override
    {
        bytes = std::min(buffer.size() - offset, bytes);

        return fwrite(buffer.data() + offset, 1, bytes, handle);
    }
*/

    virtual long position() override
    {
        return ftell(handle);
    }

    virtual void seek(long offset, origin o) override
    {
        fseek(handle, offset, static_cast<int>(o));
    }

    virtual bool eos() override
    {
        auto ch = fgetc(handle);
        ungetc(ch, handle);
        return ch == EOF;
    }

    virtual std::uint8_t peek() override
    {
        auto ch = fgetc(handle);
        ungetc(ch, handle);
        return static_cast<std::uint8_t>(ch);
    }

    virtual bool seekable() const override
    {
        return true;
    }

private:
    std::FILE* handle;
    std::string path_;
};

}

#endif // FILE_STREAM_HPP
