//
//  geometry.h
//  robotcontrol2
//
//  Created by Sebastian Theophil on 24.09.15.
//  Copyright © 2015 Sebastian Theophil. All rights reserved.
//

#ifndef geometry_h
#define geometry_h

#include <boost/operators.hpp>
#include <cstdint>
#include <cmath>

#include "math.h"

namespace rbt {
    template<typename T> struct size;

    template<typename T>
    struct point :
        boost::equality_comparable<point<T>,
        boost::additive< point<T>, size<T> > >
    {
        T x;
        T y;
        
        point() = default;
        point(T _x, T _y) : x(_x), y(_y) {}
        
        static point<T> zero() { return point(0, 0); }
        
        friend point<T> operator+=(point<T>& pt, size<T> const& sz);
        friend point<T> operator-=(point<T>& pt, size<T> const& sz);
        friend size<T> operator-(point<T> const& lhs, point<T> const& rhs);
        friend bool operator==(point<T> const& lhs, point<T> const& rhs);
    };
    
    template<typename T>
    struct size {
        T x;
        T y;
        
        size(T _x, T _y) : x(_x), y(_y) {}
        
        template<typename S>
        explicit size(S _x, S _y) : x(rbt::numeric_cast<T>(_x)), y(rbt::numeric_cast<T>(_y))
        {}
        
        static size<T> fromAngleAndDistance(double fYaw, T distance);
        
        T Abs() const;
    };

    template<typename T>
    point<T> operator+=(point<T>& pt, size<T> const& sz) {
        pt.x += sz.x;
        pt.y += sz.y;
        return pt;
    }
    
    template<typename T>
    point<T> operator-=(point<T>& pt, size<T> const& sz) {
        pt.x -= sz.x;
        pt.y -= sz.y;
        return pt;
    }
    
    template<typename T>
    size<T> operator-(point<T> const& lhs, point<T> const& rhs) {
        return size<T>(lhs.x - rhs.x, lhs.y - rhs.y);
    }
    
    template<typename T>
    bool operator==(point<T> const& lhs, point<T> const& rhs) {
        return lhs.x == rhs.x && lhs.y == rhs.y;
    }
    
    template<typename T>
    size<T> size<T>::fromAngleAndDistance(double fYaw, T distance) {
        return size<T>(distance * std::cos(fYaw),
                       distance * std::sin(fYaw));
    }
    
    template<typename T>
    T size<T>::Abs() const {
        return rbt::numeric_cast<T>(sqrt(x*x + y*y));
    }
}
#endif /* geometry_h */
