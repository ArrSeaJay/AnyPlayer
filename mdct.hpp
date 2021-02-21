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

#ifndef MDCT_HPP
#define MDCT_HPP

#include <valarray>
#include <cmath>
#include <numbers>
#include "helper.hpp"

namespace anyplayer
{

template<class T = float>
class mdct
{
private:
    int n;
    int n2;
    int n4;
    int n8;
    int n34;
    int ld;

    std::valarray<T> A;
    std::valarray<T> B;
    std::valarray<T> C;

public:
    mdct(int N)
        : n(N)
        , n2(n >> 1)
        , n4(n >> 2)
        , n8(n >> 3)
        , n34(n - n4)
        , ld(helper::ilog(n) - 1)
        , A(n2)
        , B(n2)
        , C(n4)
    {
        const auto pi = std::numbers::pi_v<T>;

        for (int k = 0, k2 = 0, k4 = 0; k < n4; k++, k2 += 2, k4 += 4)
        {
            A[k2]     =  std::cos(k4 * pi / n);
            A[k2 + 1] = -std::sin(k4 * pi / n);

            B[k2]     = std::cos((k2 + 1) * pi / n / 2);
            B[k2 + 1] = std::sin((k2 + 1) * pi / n / 2);
        }

        for (int k = 0, k2 = 0; k < n8; k++, k2 += 2)
        {
            C[k2]     =  std::cos(2 * (k2 + 1) * pi / n);
            C[k2 + 1] = -std::sin(2 * (k2 + 1) * pi / n);
        }
    }

    std::valarray<T> inverse(const std::valarray<T>& Y) const
    {
        // get spectral coefficients
        std::valarray<T> u(n);
        u[std::slice( 0, n2, 1)] =  Y[std::slice(     0, n2,  1)];
        u[std::slice(n2, n2, 1)] = -Y[std::slice(n2 - 1, n2, -1)];

        const std::valarray<T> X = kernel_computation(u);

        std::valarray<T> y(n);
        y[std::slice(  0, n4, 1)] = 0.5 *  X[std::slice(    n4, n4,  1)];
        y[std::slice( n4, n2, 1)] = 0.5 * -X[std::slice(n2 - 1, n2, -1)];
        y[std::slice(n34, n4, 1)] = 0.5 * -X[std::slice(     0, n4,  1)];

        return y;
    }

    std::valarray<T> kernel_computation(const std::valarray<T>& u) const
    {
        // step 1
        std::valarray<T> v(n);
        for (int k = 0, k2 = 0, k4 = 0; k < n4; k++, k2 += 2, k4 += 4)
        {
            v[n - k4 - 1] = (u[k4] - u[n - k4 - 1]) * A[k2]     - (u[k4 + 2] - u[n - k4 - 3]) * A[k2 + 1];
            v[n - k4 - 3] = (u[k4] - u[n - k4 - 1]) * A[k2 + 1] + (u[k4 + 2] - u[n - k4 - 3]) * A[k2];
        }

        // step 2
        std::valarray<T> w(n);
        for (int k = 0, k4 = 0; k < n8; k++, k4 += 4)
        {
            w[n2 + 3 + k4] =  v[n2 + 3 + k4] + v[k4 + 3];
            w[n2 + 1 + k4] =  v[n2 + 1 + k4] + v[k4 + 1];
            w[k4 + 3]      = (v[n2 + 3 + k4] - v[k4 + 3]) * A[n2 - 4 - k4] - (v[n2 + 1 + k4] - v[k4 + 1]) * A[n2 - 3 - k4];
            w[k4 + 1]      = (v[n2 + 1 + k4] - v[k4 + 1]) * A[n2 - 4 - k4] + (v[n2 + 3 + k4] - v[k4 + 3]) * A[n2 - 3 - k4];
        }

        // step 3
        std::valarray<T> u_(n);
        for (int l = 0; l < ld - 3; l++)
        {
            int k0 = n >> (l + 2);
            int k1 = 1 << (l + 3);

            int rmax = n >> (l + 4);
            int smax = 1 << (l + 2);

            for (int r = 0, r4 = 0; r < rmax; r++, r4 += 4)
            {
                int rk1 = r * k1;

                for (int s2 = 0; s2 < smax; s2 += 2)
                {
                    int k0s2 = k0 * s2;
                    int k0s2p1 = k0 * (s2 + 1);

                    u_[n - 1 - k0s2 - r4] = w[n - 1 - k0s2 - r4] + w[n - 1 - k0s2p1 - r4];
                    u_[n - 3 - k0s2 - r4] = w[n - 3 - k0s2 - r4] + w[n - 3 - k0s2p1 - r4];
                    u_[n - 1 - k0s2p1 - r4] = (w[n - 1 - k0s2 - r4] - w[n - 1 - k0s2p1 - r4]) * A[rk1]
                                            - (w[n - 3 - k0s2 - r4] - w[n - 3 - k0s2p1 - r4]) * A[rk1 + 1];
                    u_[n - 3 - k0s2p1 - r4] = (w[n - 3 - k0s2 - r4] - w[n - 3 - k0s2p1 - r4]) * A[rk1]
                                            + (w[n - 1 - k0s2 - r4] - w[n - 1 - k0s2p1 - r4]) * A[rk1 + 1];
                }
            }

            if (l + 1 < ld - 3)
            {
                w = u_;
            }
        }

        // step 4
        std::valarray<T> v_(n);
        for (unsigned int i = 0; i < static_cast<unsigned int>(n8); i++)
        {
            unsigned int j = helper::reverse_bits(i) >> (32 - ld + 3);
            unsigned int i8 = i << 3;

            if (i < j)
            {
                unsigned int j8 = j << 3;

                v_[j8 + 1] = u_[i8 + 1];
                v_[i8 + 1] = u_[j8 + 1];
                v_[j8 + 3] = u_[i8 + 3];
                v_[i8 + 3] = u_[j8 + 3];
                v_[j8 + 5] = u_[i8 + 5];
                v_[i8 + 5] = u_[j8 + 5];
                v_[j8 + 7] = u_[i8 + 7];
                v_[i8 + 7] = u_[j8 + 7];
            }
            else if (i == j)
            {
                v_[i8 + 1] = u_[i8 + 1];
                v_[i8 + 3] = u_[i8 + 3];
                v_[i8 + 5] = u_[i8 + 5];
                v_[i8 + 7] = u_[i8 + 7];
            }
        }

        // step 5
        std::valarray<T> w_(n);
        for (int k = 0, k2 = 0; k < n2; k++, k2 += 2)
        {
            w_[k] = v_[k2 + 1];
        }

        // step 6
        std::valarray<T> u__(n);
        for (int k = 0, k2 = 0, k4 = 0; k < n8; k++, k2 += 2, k4 += 4)
        {
            u__[  n - 1 - k2] = w_[k4];
            u__[  n - 2 - k2] = w_[k4 + 1];
            u__[n34 - 1 - k2] = w_[k4 + 2];
            u__[n34 - 2 - k2] = w_[k4 + 3];
        }

        // step 7
        std::valarray<T> v__(n);
        for (int k = 0, k2 = 0; k < n8; k++, k2 += 2)
        {
            v__[n2 + k2] = (u__[n2 + k2] + u__[n - 2 - k2]
                            + C[k2 + 1] * (u__[n2 + k2] - u__[n - 2 - k2])
                            + C[k2] * (u__[n2 + k2 + 1] + u__[n - 2 - k2 + 1])) / 2;

            v__[n - 2 - k2] = (u__[n2 + k2] + u__[n - 2 - k2]
                              - C[k2 + 1] * (u__[n2 + k2] - u__[n - 2 - k2])
                              - C[k2] * (u__[n2 + k2 + 1] + u__[n - 2 - k2 + 1])) / 2;

            v__[n2 + 1 + k2] = (u__[n2 + 1 + k2] - u__[n - 1 - k2]
                               + C[k2 + 1] * (u__[n2 + 1 + k2] + u__[n - 1 - k2])
                               - C[k2] * (u__[n2 + k2] - u__[n - 2 - k2])) / 2;

            v__[n - k2 - 1] = (-u__[n2 + 1 + k2] + u__[n - 1 - k2]
                              + C[k2 + 1] * (u__[n2 + 1 + k2] + u__[n - 1 - k2])
                              - C[k2] * (u__[n2 + k2] - u__[n - 2 - k2])) / 2;
        }

        // step 8
        std::valarray<T> X(n);
        for (int k = 0, k2 = 0; k < n4; k++, k2 += 2)
        {
            X[k]          = v__[k2 + n2] * B[k2]     + v__[k2 + 1 + n2] * B[k2 + 1];
            X[n2 - 1 - k] = v__[k2 + n2] * B[k2 + 1] - v__[k2 + 1 + n2] * B[k2];
        }

        return X;
    }

};

}

#endif // MDCT_HPP
