#include "UnionFind.hpp"
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
UnionFind::UnionFind(unsigned n)
// Constructor
{
   entries.reserve(n);
   for (unsigned i = 0; i < n; ++i)
      entries.push_back({i, 1});
}
//---------------------------------------------------------------------------
unsigned UnionFind::find(unsigned x)
// Find
{
   while (entries[x].parent != x) {
      auto gp = entries[entries[x].parent].parent;
      entries[x].parent = gp;
      x = gp;
   }
   return x;
}
//---------------------------------------------------------------------------
unsigned UnionFind::merge(unsigned x, unsigned y)
// Merge
{
   x = find(x);
   y = find(y);
   if (x == y)
      return x;
   if (entries[x].rank < entries[y].rank)
      swap(x, y);
   entries[y].parent = x;
   entries[x].rank += entries[y].rank;
   return x;
}
//---------------------------------------------------------------------------
}
//---------------------------------------------------------------------------