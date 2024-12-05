#include "BumpAlloc.hpp"
//---------------------------------------------------------------------------
/* adaptivelindp: Reference implementation of the adaptive LinDP algorithm
 * Copyright (C) 2024 Altan Birler
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
//---------------------------------------------------------------------------
using namespace std;
//---------------------------------------------------------------------------
namespace fastoptim {
//---------------------------------------------------------------------------
std::byte* BumpAllocBase::allocateInternal(size_t sz)
// Allocate show path
{
    auto targetCapacity = max(sz, (4 << 10) - sizeof(Chunk));
    if (current)
        targetCapacity = max(targetCapacity, min(current->capacity + current->capacity / 4, (2 << 20) - sizeof(Chunk)));
    auto newChunk = unique_ptr<Chunk, Deleter>(new (malloc(sizeof(Chunk) + targetCapacity)) Chunk{});
    newChunk->capacity = targetCapacity;
    newChunk->prev = move(current);
    current = move(newChunk);

    current->size += sz;
    return current->storage;
}
//---------------------------------------------------------------------------
}
//---------------------------------------------------------------------------