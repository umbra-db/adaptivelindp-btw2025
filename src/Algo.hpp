#pragma once
//---------------------------------------------------------------------------
#include <bit>
#include <cstddef>
#include <functional>
//---------------------------------------------------------------------------
namespace fastoptim {
//---------------------------------------------------------------------------
// Taken from https://orlp.net/blog/bitwise-binary-search.
/*
    Copyright (c) 2023 Orson Peters

    This software is provided 'as-is', without any express or implied warranty. In
    no event will the authors be held liable for any damages arising from the use of
    this software.

    Permission is granted to anyone to use this software for any purpose, including
    commercial applications, and to alter it and redistribute it freely, subject to
    the following restrictions:

    1. The origin of this software must not be misrepresented; you must not claim
        that you wrote the original software. If you use this software in a product,
        an acknowledgment in the product documentation would be appreciated but is
        not required.

    2. Altered source versions must be plainly marked as such, and must not be
        misrepresented as being the original software.

    3. This notice may not be removed or altered from any source distribution.
*/
template<typename It, typename T, typename Cmp = std::less<>>
It lower_bound_fast(It begin, It end, const T& value, Cmp cmp = std::less<>{})
// Find first index >= value
{
    size_t n = end - begin;
    if (n == 0) return begin;

    size_t two_k = std::bit_floor(n);
    size_t b = cmp(begin[n / 2], value) ? n - two_k : -1;
    for (size_t bit = two_k >> 1; bit != 0; bit >>= 1) {
        if (cmp(begin[b + bit], value)) b += bit;
    }
    return begin + (b + 1);
}
//---------------------------------------------------------------------------
}