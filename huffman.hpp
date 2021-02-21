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

#ifndef HUFFMAN_HPP
#define HUFFMAN_HPP

#include <memory>
#include <utility>
#include <iostream>
#include <sstream>
#include <optional>

namespace anyplayer
{
template<typename T>
class huffman
{
private:
    enum { leaf, node } type;

    union data_t
    {
        T leaf;
        std::pair<std::unique_ptr<huffman<T>>, std::unique_ptr<huffman<T>>> node;

        data_t() : node() {}
        data_t(T l) : leaf(l) {}
        ~data_t() {}
    } data;

    void destroy()
    {
        switch(type)
        {
        case leaf:
            data.leaf.~T();
            break;
        case node:
            data.node.~pair();
            break;
        }
    }

    void move(huffman<T>&& other)
    {
        type = other.type;

        switch(type)
        {
        case leaf:
            data.leaf = std::move(other.data.leaf);
            break;
        case node:
            data.node = std::move(other.data.node);
            break;
        }
    }

    void copy(const huffman<T>& other)
    {
        type = other.type;

        switch(type)
        {
        case leaf:
            data.leaf = other.data.leaf;
            break;
        case node:
            data.node = std::make_pair(std::make_unique<huffman<T>>(*other.data.node.first), std::make_unique<huffman<T>>(*other.data.node.second));
            break;
        }
    }

public:
    constexpr huffman() : type(node), data() {}
    constexpr huffman(T value) : type(leaf), data(value) {}

    huffman(huffman<T>&& other)
    {
        move(std::move(other));
    }

    huffman& operator =(huffman<T>&& other)
    {
        destroy();

        move(std::move(other));

        return *this;
    }

    huffman(const huffman<T> &other)
    {
        copy(other);
    }

    huffman& operator =(const huffman<T>& other)
    {
        destroy();

        copy(other);

        return *this;
    }    

    huffman<T>* append(bool bit)
    {
        if (type == node)
        {
            if (bit)
            {
                data.node.first = std::make_unique<huffman<T>>();
                return data.node.first.get();
            }
            else
            {
                data.node.second = std::make_unique<huffman<T>>();
                return data.node.second.get();
            }
        }
        else
        {
            return nullptr;
        }
    }

    void append(bool bit, T value)
    {
        if (type == node)
        {
            if (bit)
            {
                data.node.first = std::make_unique<huffman<T>>(value);
            }
            else
            {
                data.node.second = std::make_unique<huffman<T>>(value);
            }
        }
    }

    bool is_free(bool bit) const
    {
        return type == node && (bit ? data.node.first == nullptr : data.node.second == nullptr);
    }

    bool is_full(int depth) const
    {
        if (type == leaf) return true;

        if (depth == 1) return true;

        return data.node.first != nullptr
            && data.node.second != nullptr
            && data.node.first->is_full(depth - 1)
            && data.node.second->is_full(depth - 1);
    }

    bool is_full(int depth, bool bit) const
    {
        if (type == node)
        {
            return bit ? data.node.first->is_full(depth) : data.node.second->is_full(depth);
        }
        else
        {
            return true;
        }
    }

    huffman<T>* get_branch(bool bit) const
    {
        if (type == node)
        {
            return bit ? data.node.first.get() : data.node.second.get();
        }
        else
        {
            return nullptr;
        }
    }

    template<typename F>
    std::optional<T> decode(F getBit) const
    {
        static_assert(std::is_same<typename std::result_of<F()>::type, bool>::value, "bool getBit() required");

        auto current = this;

        while (current != nullptr && current->type == node)
        {
            if (getBit())
                current = current->data.node.first.get();
            else
                current = current->data.node.second.get();
        }

        if (current != nullptr)
            return current->data.leaf;
        else
            return {};
    }

private:
    void print_helper(std::string a, std::string& s) const
    {
        switch(type)
        {
        case leaf:
            {
                std::stringstream ss;
                ss << a << "=" << data.leaf << std::endl;
                s.append(ss.str());
                break;
            }
        case node:
            if (data.node.second != nullptr)
            {
                data.node.second->print(a + "0", s);
            }
            if (data.node.first != nullptr)
            {
                data.node.first->print(a + "1", s);
            }
            break;
        }
    }

public:
    void print() const
    {
        std::string s;
        print_helper("", s);
        std::cout << s;
    }

    ~huffman()
    {
        destroy();
    }
};
}

#endif // HUFFMAN_HPP
