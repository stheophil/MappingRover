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
#include <boost/range/algorithm/for_each.hpp>
#include <opencv2/core.hpp>

#include <cstdint>
#include <cmath>
#include <algorithm>

#include "math.h"

namespace rbt {
    template<typename T> struct size;

    template<typename T>
    struct point :
        boost::equality_comparable<point<T>,
        boost::additive< point<T>, size<T>,
        boost::multiplicative< point<T>, T >>>
    {
        T x;
        T y;
        
        point() = default;
        point(T _x, T _y) : x(_x), y(_y) {}

        template<typename S>
        explicit point(S _x, S _y) : x(rbt::numeric_cast<T>(_x)), y(rbt::numeric_cast<T>(_y))
        {}
        
        template<typename S>
        explicit point(point<S> const& pt) : x(rbt::numeric_cast<T>(pt.x)), y(rbt::numeric_cast<T>(pt.y))
        {}
        
        explicit point(cv::Point_<T> const& pt) : x(pt.x), y(pt.y)
        {}
        
        static point<T> zero() { return point(0, 0); }
        static point<T> invalid() { return point(std::numeric_limits<T>::lowest(), std::numeric_limits<T>::lowest()); }
        
        point<T>& operator+=(size<T> const& sz);
        point<T>& operator-=(size<T> const& sz);
        
        point<T>& operator*=(T t);
        point<T>& operator/=(T t);
        
        size<T> operator-(point<T> const& rhs) const;
        bool operator==(point<T> const& rhs) const;
        
        operator cv::Point_<T>() const;
    };
    
    template<typename T>
    struct size :
        boost::multiplicative2<size<T>, T>
    {
        T x;
        T y;
        
        size(T _x, T _y) : x(_x), y(_y) {}
        
        template<typename S>
        explicit size(S _x, S _y) : x(rbt::numeric_cast<T>(_x)), y(rbt::numeric_cast<T>(_y))
        {}
        
        template<typename S>
        explicit size(size<S> const& sz) : x(rbt::numeric_cast<T>(sz.x)), y(rbt::numeric_cast<T>(sz.y))
        {}
        
        static size<T> fromAngleAndDistance(double fYaw, double fDistance);
    
        size<T>& operator*=(T t);
        size<T>& operator/=(T t);
        
        T operator*(size<T> const& rhs) const;
        
        // < 0 -> v is left of this
        // > 0 -> v is right of this
        int compare(size const& sz) const;
        int quadrant() const;
        size<T> rotated(double fAngle) const;
            
        T Abs() const;
        T SqrAbs() const;
    };
    
    template<typename T>
    struct rect {
        T left;
        T bottom;
        T right;
        T top;
        
        static rect<T> empty(); // neutral element for operator|=
        static rect<T> bound(std::initializer_list<point<T>> const& cpt);
        
        rect<T>& operator|=(point<T> const& pt);
    };
    
    template<typename T>
    struct interval {
        T begin;
        T end;
        
        interval() = default;
        interval(T _begin, T _end) : begin(_begin), end(_end) {}
        
        interval<T>& operator|=(T const& t);
    };

    template<typename T>
    point<T>& point<T>::operator+=(size<T> const& sz) {
        x += sz.x;
        y += sz.y;
        return *this;
    }
    
    template<typename T>
    point<T>& point<T>::operator-=(size<T> const& sz) {
        x -= sz.x;
        y -= sz.y;
        return *this;
    }
    
    template<typename T>
    point<T>& point<T>::operator*=(T t) {
        x *= t;
        y *= t;
        return *this;
    }
    
    template<typename T>
    point<T>& point<T>::operator/=(T t) {
        x /= t;
        y /= t;
        return *this;
    }

    template<typename T>
    point<T>::operator cv::Point_<T>() const {
        return cv::Point_<T>(x, y);
    }
    
    template<typename T>
    size<T> point<T>::operator-(point<T> const& rhs) const {
        return size<T>(x - rhs.x, y - rhs.y);
    }
    
    template<typename T>
    bool point<T>::operator==(point<T> const& rhs) const {
        return x == rhs.x && y == rhs.y;
    }
    
    template<typename T>
    size<T> size<T>::fromAngleAndDistance(double fYaw, double fDistance) {
        return size<T>(fDistance * std::cos(fYaw),
                       fDistance * std::sin(fYaw));
    }
    
    template<typename T>
    size<T>& size<T>::operator*=(T t) {
        x *= t;
        y *= t;
        return *this;
    }
    
    template<typename T>
    size<T>& size<T>::operator/=(T t) {
        x /= t;
        y /= t;
        return *this;
    }
    
    template<typename T>
    T size<T>::operator*(size<T> const& rhs) const {
        return x*rhs.x + y*rhs.y;
    }
    
    template<typename T>
    int size<T>::compare(size<T> const& sz) const {
        auto d = y * sz.x - x * sz.y;
        return rbt::sign(d);
    }
    
    template<typename T>
    int size<T>::quadrant() const {
        return x < 0
            ? (y < 0 ? 2 : 1)
            : (y < 0 ? 3 : 0);
    }

    template<typename T>
    size<T> size<T>::rotated(double fAngle) const {
        auto c = std::cos(fAngle);
        auto s = std::sin(fAngle);
        return {x * c - y * s, x * s + y * c};
    }
    
    template<typename T>
    T size<T>::Abs() const {
        return rbt::numeric_cast<T>(std::sqrt(SqrAbs()));
    }
    
    template<typename T>
    T size<T>::SqrAbs() const {
        return rbt::sqr(x) + rbt::sqr(y);
    }
    
    template<typename T>
    rect<T> rect<T>::empty() {
        return rect<T>{std::numeric_limits<T>::max(),
                        std::numeric_limits<T>::max(),
                        std::numeric_limits<T>::lowest(),
                        std::numeric_limits<T>::lowest()};
    }
    
    template<typename T>
    rect<T> rect<T>::bound(std::initializer_list<point<T>> const& cpt) {
        auto rect = rbt::rect<T>::empty();
        boost::for_each(cpt, [&](rbt::point<T> const& pt) {
            rect |= pt;
        });
        return rect;
    }
    
    template<typename T>
    rect<T>& rect<T>::operator|=(rbt::point<T> const& pt) {
        left = std::min(left, pt.x);
        bottom = std::min(bottom, pt.y);
        right = std::max(right, pt.x);
        top = std::max(top, pt.y);
        return *this;
    }
    
    template<typename T>
    interval<T>& interval<T>::operator|=(T const& t) {
        begin = std::min(begin, t);
        end = std::max(end, t);
        return *this;
    }
}
#endif /* geometry_h */
