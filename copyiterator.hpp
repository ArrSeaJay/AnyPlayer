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

#ifndef COPYITERATOR_HPP
#define COPYITERATOR_HPP

#include <iterator>
#include <type_traits>
#include "iteratorhelper.hpp"

namespace anyplayer
{

template<typename Input, typename Output>
class copy_iterator : public std::iterator<
        std::input_iterator_tag,
        typename std::iterator_traits<typename std::remove_reference<Input>::type>::value_type,
        typename std::iterator_traits<typename std::remove_reference<Input>::type>::difference_type,
        typename std::iterator_traits<typename std::remove_reference<Input>::type>::pointer,
        typename std::iterator_traits<typename std::remove_reference<Input>::type>::reference>
{
public:
    typedef typename std::iterator_traits<typename std::remove_reference<Input>::type>::value_type value_type;
    typedef typename std::iterator_traits<typename std::remove_reference<Input>::type>::pointer pointer;

    copy_iterator(Input input, const Output& output)
        : input_(input)
        , output_(output)
    {
    }

    value_type operator *() const
    {
        return *input_;
    }

    copy_iterator& operator ++()
    {
        *output_ = *input_;
        ++output_;
        ++input_;

        return *this;
    }

    dereferencable<value_type> operator ++(int)
    {
        dereferencable<value_type> tmp(*input_);

        *output_ = *input_;
        ++output_;
        ++input_;

        return tmp;
    }

private:
    Input input_;
    Output output_;
};

template<typename Input, typename Output>
copy_iterator<Input, Output> make_copy(const Input& input, const Output& output)
{
    return {input, output};
}

template<typename Input, typename Output>
copy_iterator<Input&, Output> make_ref_copy(Input& input, const Output& output)
{
    return {input, output};
}

}

#endif // COPYITERATOR_HPP
