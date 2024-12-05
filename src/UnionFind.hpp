#pragma once
//---------------------------------------------------------------------------
#include <vector>
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
/// A union-find data structure
class UnionFind {
   /// An entry
   struct Entry {
      /// The parent entry
      unsigned parent;
      /// The rank representing the subset size
      unsigned rank;
   };
   /// The entries
   std::vector<Entry> entries;

   public:
   /// Constructor
   explicit UnionFind(unsigned n);

   /// Find
   [[nodiscard]] unsigned find(unsigned x);
   /// Merge
   unsigned merge(unsigned x, unsigned y);
};
//---------------------------------------------------------------------------
}
//---------------------------------------------------------------------------