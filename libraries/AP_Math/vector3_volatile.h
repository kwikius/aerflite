// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// Copyright 2010 Michael Smith, all rights reserved.

// Derived closely from:
/****************************************
* 3D Vector Classes
* By Bill Perone (billperone@yahoo.com)
* Original: 9-16-2002
* Revised: 19-11-2003
*          11-12-2003
*          18-12-2003
*          06-06-2004
*
* © 2003, This code is provided "as is" and you can use it freely as long as
* credit is given to Bill Perone in the application it is used in
*
* Notes:
* if a*b = 0 then a & b are orthogonal
* a%b = -b%a
* a*(b%c) = (a%b)*c
* a%b = a(cast to matrix)*b
* (a%b).length() = area of parallelogram formed by a & b
* (a%b).length() = a.length()*b.length() * sin(angle between a & b)
* (a%b).length() = 0 if angle between a & b = 0 or a.length() = 0 or b.length() = 0
* a * (b%c) = volume of parallelpiped formed by a, b, c
* vector triple product: a%(b%c) = b*(a*c) - c*(a*b)
* scalar triple product: a*(b%c) = c*(a%b) = b*(c%a)
* vector quadruple product: (a%b)*(c%d) = (a*c)*(b*d) - (a*d)*(b*c)
* if a is unit vector along b then a%b = -b%a = -b(cast to matrix)*a = 0
* vectors a1...an are linearly dependant if there exists a vector of scalars (b) where a1*b1 + ... + an*bn = 0
*           or if the matrix (A) * b = 0
*
****************************************/

#ifndef VECTOR3_VOLATILE_H
#define VECTOR3_VOLATILE_H

#include "vector3.h"

template <typename T>
class Vector3<volatile T>
{

public:
    volatile T        x;
    volatile T        y;
    volatile T        z;

    Vector3(): x{T{0}},y{T{0}},z{T{0}}{ }

    // setting ctor
    Vector3(const T & x0, const T & y0, const T & z0) : x{x0}, y{y0}, z{z0} {}
    // non volatile
    Vector3 (Vector3<T> const &v) : x{v.x},y{v.y},z{v.z}{}

    Vector3& operator = (Vector3<T> const & in)
    {
       x = in.x; y= in.y; z = in.z;
       return *this;
    }

    operator Vector3<T> () { return Vector3<T>{x,y,z};}

};

template <typename T>
inline
Vector3<T> operator *(Vector3<volatile T> const & lhs,T const & rhs)
{
   return Vector3<T>{ lhs.x * rhs, lhs.y * rhs, lhs.z * rhs};
}

template <typename T>
inline
Vector3<T> operator +(Vector3<volatile T> const & lhs,Vector3<T> const & rhs)
{
   return Vector3<T>{ lhs.x + rhs.x, lhs.y +rhs.y, lhs.z +rhs.z};
}

template <typename T>
inline
Vector3<T> operator +(Vector3<T> const & lhs,Vector3<volatile T> const & rhs)
{
   return Vector3<T>{ lhs.x + rhs.x, lhs.y+rhs.y, lhs.z+rhs.z};
}

template <typename T>
inline
Vector3<T> operator -(Vector3<volatile T> const & lhs,Vector3<T> const & rhs)
{
   return Vector3<T>{ lhs.x-rhs.x, lhs.y-rhs.y, lhs.z-rhs.z};
}

template <typename T>
inline
Vector3<T> operator -(Vector3<T> const & lhs,Vector3<volatile T> const & rhs)
{
   return Vector3<T>{ lhs.x-rhs.x, lhs.y-rhs.y, lhs.z-rhs.z};
}

#endif // VECTOR3_VOLATILE_H
