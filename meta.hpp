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

#ifndef META_HPP
#define META_HPP

#include <array>

namespace anyplayer
{

template<typename T, T... args> struct __array_holder
{
    static constexpr std::array<T, sizeof...(args)> data = { args... };
};

template<typename T, T... args>
constexpr std::array<T, sizeof...(args)> __array_holder<T, args...>::data;

template<typename T, std::size_t N, template<std::size_t> class F, T... args>
struct __generate_array_impl
{
    typedef typename __generate_array_impl<T, N-1, F, F<N>::value, args...>::result result;
};

template<typename T, template<std::size_t> class F, T... args>
struct __generate_array_impl<T, 0, F, args...>
{
    typedef __array_holder<T, F<0>::value, args...> result;
};

template<typename T, std::size_t N, template<std::size_t> class F>
struct generate_array
{
    typedef typename __generate_array_impl<T, N-1, F>::result result;
};

}

#endif // META_HPP
