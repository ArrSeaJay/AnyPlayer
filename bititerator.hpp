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

#ifndef BITITERATOR_HPP
#define BITITERATOR_HPP

#include <type_traits>
#include <iostream>
#include <iterator>
#include <array>
#include <vector>
#include <algorithm>
#include <cassert>

#include "meta.hpp"

#include "endian.hpp"
#include "iteratorhelper.hpp"

namespace anyplayer
{

template<typename T, std::size_t byte_size>
struct __mask
{
    template<std::size_t n> struct func
    {
        static constexpr T value = (1 << n) - 1;
    };

    typedef typename generate_array<T, byte_size + 1, func>::result array;
};

template<typename Iterator, typename Descendant>
class bit_iterator_base : public std::iterator<
        typename std::iterator_traits<typename std::remove_reference<Iterator>::type>::iterator_category,
        bool, int, void, bool>
{
public:
    typedef std::iterator_traits<typename std::remove_reference<Iterator>::type> given_traits;
    typedef typename given_traits::iterator_category given_category;

    typedef Iterator iterator_type;
    typedef typename given_traits::value_type byte_type;
    static constexpr auto byte_size = sizeof(byte_type) * CHAR_BIT;

    static_assert(std::is_base_of<std::input_iterator_tag, given_category>::value, "bit_iterator requires input iterator as base iterator");
    static_assert(std::is_integral<byte_type>::value, "bit_iterator requires base iterator for integral types");
    static_assert(sizeof(byte_type) == 1, "bit_iterator requires iterator with value_type of size 1");

    typedef int difference_type;

    bit_iterator_base(iterator_type iterator)
        : iterator_(iterator)
    {}

    bit_iterator_base(iterator_type iterator, std::size_t current_bit)
        : iterator_(iterator)
        , current_bit_(current_bit)
    {}

    bool operator [](difference_type n) const
    {
        return *(*this + n);
    }

    // simpler version of += operator
    // for 0 <= n <= byte_size
    void advance(std::size_t n)
    {
        assert(n >= 0 && n <= byte_size);

        difference_type new_bit = current_bit_ + n;
        auto div = std::div(new_bit, byte_size);
        if (div.quot > 0)
            next();

        current_bit_ = div.rem;
    }

    friend Descendant& operator ++(Descendant& self)
    {
        if (++self.current_bit_ == byte_size)
        {
            self.next();

            self.current_bit_ = 0;
        }

        return self;
    }

    friend Descendant& operator --(Descendant& self)
    {
        if (self.current_bit_-- == 0)
        {
            self.next(-1);

            self.current_bit_ = byte_size - 1;
        }

        return self;
    }

    friend dereferencable<bool> operator ++(Descendant& self, int)
    {
        dereferencable<bool> tmp(*self);

        ++self;

        return tmp;
    }

    friend dereferencable<bool> operator --(Descendant& self, int)
    {
        dereferencable<bool> tmp(*self);

        --self;

        return tmp;
    }

    friend Descendant& operator +=(Descendant& self, difference_type n)
    {
        difference_type new_bit = self.current_bit_ + n;

        auto div = std::div(new_bit, byte_size);
        if (div.rem >= 0)
        {
            self.next(div.quot);
            self.current_bit_ = div.rem;
        }
        else
        {
            self.next(div.quot - 1);
            self.current_bit_ = byte_size + div.rem;
        }

        return self;
    }

    friend Descendant& operator -=(Descendant& self, difference_type n)
    {
        return self += -n;
    }

    int operator -(const bit_iterator_base& other) const
    {
        auto diff = (base() - other.base()) * byte_size;

        return diff + current_bit() - other.current_bit();
    }

    bool equal(const bit_iterator_base& other) const
    {
        return base() == other.base()
            && current_bit() == other.current_bit();
    }

    typedef typename std::remove_reference<iterator_type>::type copy_type;

    const copy_type& base() const {return iterator_; }
    std::size_t current_bit() const { return current_bit_; }

private:
    // this next is for basic input_iterators
    void next()
    {
        ++iterator_;
    }

    void next(difference_type n)
    {
        iterator_ += n;
    }

    iterator_type iterator_;
    std::size_t current_bit_ = 0;
};

/**
 * @brief bit_iterator is an iterator adapter to iterate at bit level
 *
 * The base iterator can be any input iterator that iterate on itegral
 * type with a sizeof(T) == 1.
 *
 * The template parameter big_endian specifies the endianess of the base
 * byte stream.
 *
 * This adapter inherits the iterator traits of the base iterator in the
 * sense that a random access iterator will result in an equivalent
 * bit_iterator adapter.
 */
template<typename Iterator, bool big_endian = false>
class bit_iterator;

template<typename Iterator>
class bit_iterator<Iterator, false> : public bit_iterator_base<Iterator,
        bit_iterator<Iterator, false>>
{
    typedef bit_iterator_base<Iterator,
    bit_iterator<Iterator, false>> base_;

public:
    using base_::bit_iterator_base;

    constexpr bool operator *() const
    {
        return ((*base_::base() >> base_::current_bit()) & 1) == 1;
    }
};

template<typename Iterator>
class bit_iterator<Iterator, true> : public bit_iterator_base<Iterator,
        bit_iterator<Iterator, true>>
{
    typedef bit_iterator_base<Iterator,
    bit_iterator<Iterator, true>> base_;

public:
    using base_::bit_iterator_base;

    constexpr bool operator *() const
    {
        return ((*base_::base() >> (base_::byte_size - 1 - base_::current_bit())) & 1) == 1;
    }
};

template<typename Iterator, bool big_endian>
bit_iterator<Iterator, big_endian> operator +(const bit_iterator<Iterator, big_endian>& lhs, typename bit_iterator<Iterator, big_endian>::difference_type rhs)
{
    auto tmp = lhs;

    tmp += rhs;

    return tmp;
}

template<typename Iterator, bool big_endian>
bit_iterator<Iterator, big_endian> operator +(typename bit_iterator<Iterator, big_endian>::difference_type lhs, const bit_iterator<Iterator, big_endian>& rhs)
{
    auto tmp = rhs;

    tmp += lhs;

    return tmp;
}

template<typename Iterator, bool big_endian>
bit_iterator<Iterator, big_endian> operator -(const bit_iterator<Iterator, big_endian>& lhs, typename bit_iterator<Iterator, big_endian>::difference_type rhs)
{
    auto tmp = lhs;

    tmp -= rhs;

    return tmp;
}

template<typename Iterator, bool big_endian>
typename bit_iterator<Iterator, big_endian>::difference_type operator -(const bit_iterator<Iterator, big_endian>& lhs, const bit_iterator<Iterator, big_endian>& rhs)
{
    auto diff = (lhs.base() - rhs.base()) * bit_iterator<Iterator, big_endian>::byte_size;

    return diff + lhs.current_bit() - rhs.current_bit();
}

template<typename Iterator, bool big_endian>
bool operator ==(const bit_iterator<Iterator, big_endian>& lhs, const bit_iterator<Iterator, big_endian>& rhs)
{
    return lhs.equal(rhs);
}

template<typename Iterator, bool big_endian>
bool operator !=(const bit_iterator<Iterator, big_endian>& lhs, const bit_iterator<Iterator, big_endian>& rhs)
{
    return !lhs.equal(rhs);
}

template<typename Iterator, bool big_endian>
bool operator <(const bit_iterator<Iterator, big_endian>& lhs, const bit_iterator<Iterator, big_endian>& rhs)
{
    return (rhs - lhs) > 0;
}

template<typename Iterator, bool big_endian>
bool operator >(const bit_iterator<Iterator, big_endian>& lhs, const bit_iterator<Iterator, big_endian>& rhs)
{
    return rhs < lhs;
}

template<typename Iterator, bool big_endian>
bool operator >=(const bit_iterator<Iterator, big_endian>& lhs, const bit_iterator<Iterator, big_endian>& rhs)
{
    return !(lhs < rhs);
}

template<typename Iterator, bool big_endian>
bool operator <=(const bit_iterator<Iterator, big_endian>& lhs, const bit_iterator<Iterator, big_endian>& rhs)
{
    return !(lhs > rhs);
}

template<typename Iterator, bool big_endian>
std::size_t consecutive(bit_iterator<Iterator, big_endian>& iterator, bool bit)
{
    std::size_t count = 0;

    while (*iterator++ == bit)
    {
        ++count;
    }

    return count;
}

template<bool big_endian = false, typename Iterator>
bit_iterator<Iterator, big_endian> make_bit_iterator(const Iterator& it)
{
    return {it};
}

template<bool big_endian = false, typename Iterator>
bit_iterator<Iterator&, big_endian> make_ref_bit_iterator(Iterator& it)
{
    return bit_iterator<Iterator&, big_endian>(it);
}

template<std::size_t Bits>
struct bit_type
{
    static_assert(Bits <= 64, "Number of bits must be in range [0, 64]");

    typedef
    typename std::conditional<Bits <=  8, std::uint8_t,
    typename std::conditional<Bits <= 16, std::uint16_t,
    typename std::conditional<Bits <= 32, std::uint32_t,
    typename std::conditional<Bits <= 64, std::uint64_t,
    void
    >::type>::type>::type>::type type;
};

template<typename Iterator>
std::uint8_t read_bits(bit_iterator<Iterator, false>& it, std::size_t bits)
{
    typedef __mask<typename bit_iterator<Iterator, false>::byte_type, bit_iterator<Iterator, false>::byte_size> Masks;

    constexpr auto byte_size = bit_iterator<Iterator, false>::byte_size;
    if ((byte_size - it.current_bit()) >= bits)
    {
        std::uint8_t byte = (*it.base() >> it.current_bit()) & Masks::array::data[bits];
        it.advance(bits);
        return byte;
    }
    else
    {
        auto first = byte_size - it.current_bit();
        auto rest = bits - first;
        std::uint8_t byte = *it.base() >> it.current_bit();
        it.advance(byte_size - it.current_bit());
        byte |= (*it.base() & Masks::array::data[rest]) << first;
        it.advance(rest);
        return byte;
    }
}

template<typename Iterator>
std::uint8_t read_bits(bit_iterator<Iterator, true>& it, std::size_t bits)
{
    typedef __mask<typename bit_iterator<Iterator, true>::byte_type, bit_iterator<Iterator, true>::byte_size> Masks;

    constexpr auto byte_size = bit_iterator<Iterator, true>::byte_size;
    if ((byte_size - it.current_bit()) >= bits)
    {
        auto avail = byte_size - it.current_bit();
        auto shift = avail - bits;
        std::uint8_t byte = *it.base() >> shift;
        byte &= Masks::array::data[bits];
        it.advance(bits);
        return byte;
    }
    else
    {
        auto first = byte_size - it.current_bit();
        std::uint8_t byte = *it.base() & Masks::array::data[first];
        auto rest = bits - first;
        byte <<= rest;
        it.advance(bits);
        auto low = *it.base() >> (byte_size - rest);
        byte |= low;
        return byte;
    }
}

template<typename Iterator, std::size_t Bits, bool big_endian>
struct int_reader_even
{
    typedef typename bit_type<Bits>::type T;

    typedef int_reader_even<Iterator, Bits / 2, big_endian> reader;

    static T read(bit_iterator<Iterator, big_endian>& it)
    {
        T lo = reader::read(it);
        T hi = reader::read(it);

        return lo | (hi << (sizeof(T) * 4));
    }
};

template<typename Iterator>
struct int_reader_even<Iterator, 8, false>
{
    typedef typename bit_type<8>::type T;

    static T read(bit_iterator<Iterator, false>& it)
    {
        T byte = *it.base() >> it.current_bit();

        it.advance(8);

        if (it.current_bit() != 0)
        {
            auto high_bits = bit_iterator<Iterator, false>::byte_size - it.current_bit();
            byte |= *it.base() << high_bits;
        }

        return byte;
    }
};

template<typename Iterator>
struct int_reader_even<Iterator, 8, true>
{
    typedef typename bit_type<8>::type T;

    static T read(bit_iterator<Iterator, true>& it)
    {
         T byte = *it.base() << it.current_bit();

        it.advance(8);

        if (it.current_bit() != 0)
        {
            auto low_bits = bit_iterator<Iterator, true>::byte_size - it.current_bit();
            byte |= *it.base() >> low_bits;
        }

        return byte;
    }
};

template<typename Iterator, std::size_t Bits>
struct int_reader_even<Iterator, Bits, true>
{
    typedef typename bit_type<Bits>::type T;

    typedef int_reader_even<Iterator, Bits / 2, true> reader;

    static T read(bit_iterator<Iterator, true>& it)
    {
        T hi = reader::read(it);
        T lo = reader::read(it);

        return lo | (hi << (sizeof(T) * 4));
    }
};

template<typename Iterator, std::size_t Bits, bool big_endian>
struct int_reader_odd;

template<typename Iterator, std::size_t full, std::size_t rest, bool big_endian>
struct int_reader_odd_with_rest
{
    typedef typename bit_type<full + rest>::type T;

    static T read(bit_iterator<Iterator, big_endian>& it)
    {
        T lo = int_reader_even<Iterator, full, big_endian>::read(it);
        T hi = int_reader_odd<Iterator, rest, big_endian>::read(it);

        return lo | (hi << full);
    }
};

template<typename Iterator, bool big_endian>
struct int_reader_odd_with_rest<Iterator, 0, 0, big_endian>
{
    typedef typename bit_type<0>::type T;

    static T read(bit_iterator<Iterator, big_endian>&)
    {
        return 0;
    }
};

template<typename Iterator, std::size_t full, std::size_t rest>
struct int_reader_odd_with_rest<Iterator, full, rest, true>
{
    typedef typename bit_type<full + rest>::type T;

    static T read(bit_iterator<Iterator, true>& it)
    {
        T hi = int_reader_even<Iterator, full, true>::read(it);
        T lo = int_reader_odd<Iterator, rest, true>::read(it);

        return (hi << rest) | lo;
    }
};

template<typename Iterator, std::size_t Bits>
struct int_reader_odd_with_rest<Iterator, 0, Bits, false>
{
    typedef typename bit_type<Bits>::type T;

    static constexpr T read(bit_iterator<Iterator, false>& it)
    {
        return read_bits(it, Bits);
    }
};

template<typename Iterator, std::size_t Bits>
struct int_reader_odd_with_rest<Iterator, 0, Bits, true>
{
    typedef typename bit_type<Bits>::type T;

    static constexpr T read(bit_iterator<Iterator, true>& it)
    {
        return read_bits(it, Bits);
    }
};

template<typename Iterator, std::size_t Bits, bool big_endian>
struct int_reader_odd
{
    static_assert(Bits < 64, "Number of bits must be in range [0, 64)");

    typedef typename bit_type<Bits>::type T;

    static constexpr auto full =
             std::conditional<(Bits > 32), std::integral_constant<std::size_t, 32>,
    typename std::conditional<(Bits > 16), std::integral_constant<std::size_t, 16>,
    typename std::conditional<(Bits > 8),  std::integral_constant<std::size_t, 8>,
                                           std::integral_constant<std::size_t, 0>
    >::type>::type>::type::value;

    static constexpr auto rest = Bits - full;

    typedef int_reader_odd_with_rest<Iterator, full, rest, big_endian> reader;

    static T read(bit_iterator<Iterator, big_endian>& it)
    {
        return reader::read(it);
    }
};

template<typename Iterator, std::size_t Bits, bool big_endian>
struct int_reader
{
    static_assert(Bits <= 64, "Number of bits must be in range [0, 64]");

    typedef typename bit_type<Bits>::type T;

    static constexpr auto even =
            (Bits == 64)
         || (Bits == 32)
         || (Bits == 16)
         || (Bits ==  8);

    typedef
    typename std::conditional<even,
        int_reader_even<Iterator, Bits, big_endian>,
        int_reader_odd<Iterator, Bits, big_endian>
    >::type reader;

    static T read(bit_iterator<Iterator, big_endian>& it)
    {
        return reader::read(it);
    }
};

template<std::size_t Bits, bool big_endian = false>
struct u8_int_reader
{
    static_assert(Bits <= 64, "Number of bits must be in range [0, 64]");

    typedef typename bit_type<Bits>::type T;

    static T read(const std::uint8_t* p)
    {
        return swap_le<T>(p);
    }
};

template<std::size_t Bits>
struct u8_int_reader<Bits, true>
{
    static_assert(Bits <= 64, "Number of bits must be in range [0, 64]");

    typedef typename bit_type<Bits>::type T;

    static T read(const std::uint8_t* p)
    {
        return swap_be<T>(p);
    }
};

inline std::uint8_t read_byte(std::uint8_t*& p)
{
    return *p++;
}

template<std::size_t Bits>
typename bit_type<Bits>::type read_le(std::uint8_t*& p)
{
    auto t = p;
    p += Bits / 8;
    return u8_int_reader<Bits, false>::read(t);
}

template<std::size_t Bits>
typename bit_type<Bits>::type read_be(std::uint8_t*& p)
{
    auto t = p;
    p += Bits / 8;
    return u8_int_reader<Bits, true>::read(t);
}

inline std::uint8_t read_byte(const std::uint8_t* p)
{
    return *p;
}

template<std::size_t Bits>
typename bit_type<Bits>::type read_le(const std::uint8_t* p)
{
    return u8_int_reader<Bits, false>::read(p);
}

template<std::size_t Bits>
typename bit_type<Bits>::type read_be(const std::uint8_t* p)
{
    return u8_int_reader<Bits, true>::read(p);
}

template<typename Iterator, bool big_endian>
typename std::uint8_t read_byte(bit_iterator<Iterator, big_endian>& it)
{
    return int_reader<Iterator, 8, big_endian>::read(it);
}

template<std::size_t Bits, bool big_endian, typename Iterator>
typename bit_type<Bits>::type read_int(bit_iterator<Iterator, big_endian>& it)
{
    return int_reader<Iterator, Bits, big_endian>::read(it);
}

template<std::size_t Bits, bool big_endian, typename Iterator>
std::vector<typename bit_type<Bits>::type> read_ints(bit_iterator<Iterator, big_endian>& it, std::size_t ints)
{
    std::vector<typename bit_type<Bits>::type> result;

    result.reserve(ints);

    std::generate_n(std::back_inserter(result), ints,
                    [&](){ return int_reader<Iterator, Bits, big_endian>::read(it); });

    return result;
}

template<typename Iterator>
std::uint64_t read_int(bit_iterator<Iterator, false> &it, std::size_t bits)
{
    if (bits > 64)
        return 0;

    if (bits == 64)
        return read_int<64>(it);

    std::uint64_t result = 0;
    std::size_t shift = 0;

    if (bits >= 32)
    {
        result |= read_int<32>(it) << shift;
        bits -= 32;
        shift += 32;
    }

    if (bits >= 16)
    {
        result |= read_int<16>(it) << shift;
        bits -= 16;
        shift += 16;
    }

    if (bits >= 8)
    {
        result |= read_int<8>(it) << shift;
        bits -= 8;
        shift += 8;
    }

    if (bits > 0)
    {
        result |= read_bits(it, bits) << shift;
    }

    return result;
}

template<typename Iterator>
std::uint64_t read_int(bit_iterator<Iterator, true> &it, std::size_t bits)
{
    if (bits > 64)
        return 0;

    if (bits == 64)
        return read_int<64>(it);

    std::uint64_t result = 0;

    if (bits >= 32)
    {
        result <<= 32;
        result |= read_int<32>(it);
        bits -= 32;
    }

    if (bits >= 16)
    {
        result <<= 16;
        result |= read_int<16>(it);
        bits -= 16;
    }

    if (bits >= 8)
    {
        result <<= 8;
        result |= read_int<8>(it);
        bits -= 8;
    }

    if (bits > 0)
    {
        result <<= bits;
        result |= read_bits(it, bits);
    }

    return result;
}

template<typename Iterator, std::size_t Bits, bool big_endian = false>
class int_iterator : public std::iterator<typename std::iterator_traits<Iterator>::iterator_category, typename bit_type<Bits>::type, int, void, typename bit_type<Bits>::type>
{
public:
    typedef std::iterator_traits<Iterator> given_traits;
    typedef typename given_traits::iterator_category given_category;

    typedef Iterator iterator_type;
    typedef typename given_traits::value_type byte_type;
    static constexpr auto byte_size = sizeof(byte_type) * CHAR_BIT;

    typedef int difference_type;
    typedef typename bit_type<Bits>::type T;

public:
    int_iterator(bit_iterator<Iterator, big_endian> begin)
        : iterator_(begin)
        , last_pos_(begin)
    {
        read(1);
    }

    int_iterator(const int_iterator& other) = default;
    int_iterator& operator =(const int_iterator& other) = default;

    int_iterator make(const iterator_type& other)
    {
        // implicit cast
        return {other};
    }

    T operator *()
    {
        if (iterator_ != last_pos_)
            read(1);
        return current_element_;
    }

    T operator [](difference_type n)
    {
        return *(*this + n);
    }

    int_iterator& operator ++()
    {
        read(1);

        return *this;
    }

    dereferencable<T> operator ++(int)
    {
        dereferencable<T> tmp(**this);

        read(1);

        return tmp;
    }

    int_iterator& operator --()
    {
        read(-1);

        return *this;
    }

    dereferencable<T> operator --(int)
    {
        dereferencable<T> tmp(**this);

        read(-1);

        return tmp;
    }

    int_iterator& operator +=(difference_type n)
    {
        if (n == 0)
            return *this;

        read(n);

        return *this;
    }

    int_iterator& operator -=(difference_type n)
    {
        return *this += -n;
    }

    int_iterator operator +(difference_type n) const
    {
        auto tmp = *this;
        tmp += n;
        return tmp;
    }

    int_iterator operator -(difference_type n) const
    {
        auto tmp = *this;
        tmp += -n;
        return tmp;
    }

    bool equal(const int_iterator& other) const
    {
        return iterator_.equal(other.iterator_);
    }

    const bit_iterator<Iterator, big_endian>& base() const { return iterator_; }

private:
    typedef int_reader<Iterator, Bits, big_endian> reader;

    void read(difference_type n)
    {
        iterator_ += (n - 1) * Bits;

        current_element_ = reader::read(iterator_);

        last_pos_ = iterator_;
    }

    bit_iterator<Iterator, big_endian> iterator_;
    bit_iterator<Iterator, big_endian> last_pos_;
    T current_element_;
};

template<typename Iterator, std::size_t Bits, bool big_endian>
int_iterator<Iterator, Bits, big_endian> operator +(typename int_iterator<Iterator, Bits, big_endian>::difference_type lhs, const int_iterator<Iterator, Bits, big_endian>& rhs)
{
    auto tmp = rhs;
    tmp += lhs;
    return tmp;
}

template<typename Iterator, std::size_t Bits, bool big_endian>
typename int_iterator<Iterator, Bits, big_endian>::difference_type operator -(const int_iterator<Iterator, Bits, big_endian>& lhs, const int_iterator<Iterator, Bits, big_endian>& rhs)
{
    return (lhs.base() - rhs.base()) / Bits;
}

template<typename Iterator, bool big_endian>
int_iterator<Iterator, 8, big_endian> make_byte_iterator(const bit_iterator<Iterator, big_endian>& base)
{
    return {base};
}

template<std::size_t Bits, typename Iterator>
int_iterator<Iterator, Bits, false> make_le_iterator(const bit_iterator<Iterator, false>& base)
{
    return {base};
}

template<std::size_t Bits, typename Iterator>
int_iterator<Iterator, Bits, true> make_be_iterator(const bit_iterator<Iterator, true>& base)
{
    return {base};
}

template<typename Iterator, bool big_endian = false>
int_iterator<Iterator, 8, big_endian> make_byte_iterator(const Iterator& base)
{
    return {base};
}

template<std::size_t Bits, typename Iterator>
int_iterator<Iterator, Bits, false> make_le_iterator(const Iterator& base)
{
    return {base};
}

template<std::size_t Bits, typename Iterator>
int_iterator<Iterator, Bits, true> make_be_iterator(const Iterator& base)
{
    return {base};
}

template<typename Iterator, std::size_t Bits, bool big_endian>
bool operator ==(const int_iterator<Iterator, Bits, big_endian>& lhs, const int_iterator<Iterator, Bits, big_endian>& rhs)
{
    return lhs.equal(rhs);
}

template<typename Iterator, std::size_t Bits, bool big_endian>
bool operator !=(const int_iterator<Iterator, Bits, big_endian>& lhs, const int_iterator<Iterator, Bits, big_endian>& rhs)
{
    return !lhs.equal(rhs);
}

template<typename Iterator, std::size_t Bits, bool big_endian>
bool operator <(const int_iterator<Iterator, Bits, big_endian>& lhs, const int_iterator<Iterator, Bits, big_endian>& rhs)
{
    return (rhs - lhs) > 0;
}

template<typename Iterator, std::size_t Bits, bool big_endian>
bool operator >(const int_iterator<Iterator, Bits, big_endian>& lhs, const int_iterator<Iterator, Bits, big_endian>& rhs)
{
    return rhs < lhs;
}

template<typename Iterator, std::size_t Bits, bool big_endian>
bool operator >=(const int_iterator<Iterator, Bits, big_endian>& lhs, const int_iterator<Iterator, Bits, big_endian>& rhs)
{
    return !(lhs < rhs);
}

template<typename Iterator, std::size_t Bits, bool big_endian>
bool operator <=(const int_iterator<Iterator, Bits, big_endian>& lhs, const int_iterator<Iterator, Bits, big_endian>& rhs)
{
    return !(lhs > rhs);
}

template<typename F>
std::uint64_t read_utf8(F get_byte)
{
    std::uint64_t v = 0;
    auto x = get_byte();
    int i;

    if ((x & 0x80) == 0)
    {
        v = x;
        i = 0;
    }
    else if ((x & 0xC0) == 0xC0 && (x & 0x20) == 0)
    {
        v = x & 0x1F;
        i = 1;
    }
    else if ((x & 0xE0) == 0xE0 && (x & 0x10) == 0)
    {
        v = x & 0x0F;
        i = 2;
    }
    else if ((x & 0xF0) == 0xF0 && (x & 0x08) == 0)
    {
        v = x & 0x07;
        i = 3;
    }
    else if ((x & 0xF8) == 0xF8 && (x & 0x04) == 0)
    {
        v = x & 0x03;
        i = 4;
    }
    else if ((x & 0xFC) == 0xFC && (x & 0x02) == 0)
    {
        v = x & 0x01;
        i = 5;
    }
    else
    {
        return 0xFFFFFFFFFFFFFFFF;
    }

    for (; i != 0; i--)
    {
        x = get_byte();

        if ((x & 0x80) == 0 || (x & 0x40) == 0x40)
            return 0xFFFFFFFFFFFFFFFF;

        v <<= 6;
        v |= x & 0x3F;
    }

    return v;
}

}

#endif // BITITERATOR_HPP
