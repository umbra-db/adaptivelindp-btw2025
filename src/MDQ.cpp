#include "MDQ.hpp"
#include <iostream>
#include <iomanip>
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
template <typename T>
ostream& operator<<(ostream& out, const MDQ<T>& mdq) {
    for (size_t i = 1; i < mdq.data.size(); i *= 2) {
        for (size_t k = i; k < min(i * 2, mdq.data.size()); k++) {
            out << setw(2) << k << ":";
            if (mdq.data[k] == numeric_limits<int32_t>::max()) out << "-";
            else out << mdq.data[k];
            out << "\t";
        }
        out << endl;
    }
    return out;
}
template ostream& operator<<(ostream& out, const MDQ<int32_t>& mdq);
//---------------------------------------------------------------------------
}
//---------------------------------------------------------------------------