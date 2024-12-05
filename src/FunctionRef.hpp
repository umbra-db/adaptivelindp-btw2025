#pragma once
//---------------------------------------------------------------------------
#include <utility>
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
/// Reference to an invocable object
template <typename T>
class FunctionRef;
/// Reference to an invocable object
template <typename R, typename... Args>
class FunctionRef<R(Args...)> {
    private:
    using FuncPtr = R (*)(const void*, Args...);
    /// The invocable
    FuncPtr func = nullptr;
    /// The object argument
    const void* obj = nullptr;

    public:
    /// Constructor
    constexpr FunctionRef() noexcept = default;
    /// Constructor
    template <typename F>
    constexpr FunctionRef(const F& f) {
        func = [](const void* o, Args... args) -> R {
            return (*static_cast<const F*>(o))(std::forward<Args>(args)...);
        };
        obj = &f;
    }
    /// Constructor
    constexpr FunctionRef(R (*f)(Args...)) {
        func = [](const void* o, Args... args) -> R {
            return static_cast<decltype(f)>(o)(std::forward<Args>(args)...);
        };
        obj = f;
    }

    /// Invoke
    R operator()(Args... args) const {
        return func(obj, std::forward<Args>(args)...);
    }
    /// Is set?
    explicit operator bool() const noexcept {
        return func != nullptr;
    }
};
//---------------------------------------------------------------------------
}
//---------------------------------------------------------------------------