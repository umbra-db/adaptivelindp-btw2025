#pragma once
//---------------------------------------------------------------------------
#include <cstddef>
#include <cstdlib>
#include <memory>
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
namespace fastoptim {
//---------------------------------------------------------------------------
/// Simple allocator
class BumpAllocBase {
    /// Deleted
    using Deleter = decltype([](void* ptr) { free(ptr); });
    /// A chunk
    struct Chunk {
        /// Previous chunk
        std::unique_ptr<Chunk, Deleter> prev{};
        /// Size
        size_t size = 0;
        /// Capacity
        size_t capacity = 0;
        /// Storage
        std::byte storage[];
    };

    /// The current chunk
    std::unique_ptr<Chunk, Deleter> current{};

    /// Allocate show path
    std::byte* allocateInternal(size_t sz);

    public:
    /// Constructor
    constexpr BumpAllocBase() noexcept = default;
    /// Destructor
    constexpr ~BumpAllocBase() noexcept {
        // Prefer iterative destruction to recursion
        while (current)
            current = move(current->prev);
    }
    /// Move constructor
    constexpr BumpAllocBase(BumpAllocBase&&) noexcept = default;
    /// Move assignment
    constexpr BumpAllocBase& operator=(BumpAllocBase&&) noexcept = default;
    /// Deleted copy
    BumpAllocBase(const BumpAllocBase&) = delete;

    /// Allocate
    std::byte* allocate(size_t sz) {
        // Ensure 8 byte alignment
        sz = (sz + 7) & ~7;
        if (current && (current->size + sz <= current->capacity)) {
            auto off = current->size;
            current->size += sz;
            return &current->storage[off];
        }
        return allocateInternal(sz);
    }
};
//---------------------------------------------------------------------------
/// Simple allocator
template <typename T>
class BumpAlloc : BumpAllocBase {
    public:
    /// Allocate
    T* allocate(size_t n = 1) {
        return reinterpret_cast<T*>(BumpAllocBase::allocate(n * sizeof(T)));
    }
    /// Emplace
    template <typename... Args>
    T& emplace(Args&&... args) {
        return *new (allocate()) T(std::forward<Args>(args)...);
    }
};
//---------------------------------------------------------------------------
}