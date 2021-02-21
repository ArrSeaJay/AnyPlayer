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

#ifndef BUFFER_HPP
#define BUFFER_HPP

#include <memory>

namespace anyplayer
{

/** @brief Uninitialized buffer
 *
 * Does not construct and destroy contained elements!
 *
 * It is intended to be filled by C-code that need void* and read by
 * c++ algorithms.
 *
 * In contrast a simple vector should be used in the opposite direction.
 */
template<typename T, typename Allocator = std::allocator<T>>
class buffer
{
public:
    typedef T value_type;
    typedef Allocator allocator_type;
    typedef size_t size_type;
    typedef std::ptrdiff_t difference_type;
    typedef value_type& reference;
    typedef const value_type& const_reference;
    typedef typename std::allocator_traits<allocator_type>::pointer pointer;
    typedef typename std::allocator_traits<allocator_type>::const_pointer const_pointer;
    typedef pointer iterator;
    typedef const_pointer const_iterator;
    typedef std::reverse_iterator<iterator> reverse_iterator;
    typedef std::reverse_iterator<const_iterator> const_reverse_iterator;

    buffer()
        : n_(0)
        , p_(nullptr)
    {
    }

    explicit buffer(size_type count)
        : n_(count)
        , p_(allocator_type().allocate(n_))
    {
    }

    ~buffer()
    {
        if (p_ != nullptr)
        {
            allocator_type a;
            a.deallocate(p_, n_);
        }
    }

    size_type size() const { return n_; }

    size_type bytes() const { return n_ * sizeof(T); }

    void resize(size_type count)
    {
        allocator_type a;

        if (p_ != nullptr)
        {
            a.deallocate(p_, n_);
        }

        n_ = count;
        p_ = a.allocate(n_);
    }

    reference operator[](size_type pos) { return *(p_ + pos); }
    const_reference operator[](size_type pos) const { return *(p_ + pos); }

    pointer data() { return p_; }
    const_pointer data() const { return p_; }

    void* raw() { return reinterpret_cast<void*>(p_); }
    const void* raw() const { return reinterpret_cast<void*>(p_); }
    const void* craw() const { return reinterpret_cast<void*>(p_); }

    iterator begin() { return p_; }
    const_iterator begin() const { return p_; }
    const_iterator cbegin() const { return p_; }

    iterator end() { return p_ + n_; }
    const_iterator end() const { return p_ + n_; }
    const_iterator cend() const { return p_ + n_; }

private:
    size_type n_;
    pointer p_;
};

}

#endif // BUFFER_HPP
