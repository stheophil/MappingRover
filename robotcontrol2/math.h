//
//  math.h
//  rover
//
//  Created by Sebastian Theophil on 24.09.15.
//
//

#ifndef math_h
#define math_h

#include <type_traits>
#include <limits>
#include <cmath>
#include <cfenv>
#include <assert.h>

namespace rbt {
    template<typename T>
    T numeric_cast(T t) {
        return t;
    }
    
    template<typename Dst, typename Src>
    std::enable_if_t<std::is_integral<Dst>::value && std::is_floating_point<Src>::value,
    Dst> numeric_cast(Src t) {
        assert(std::numeric_limits<Dst>::lowest()<=t && t<=std::numeric_limits<Dst>::max());
        std::feclearexcept(FE_ALL_EXCEPT);
        auto tRounded = std::lround(t);
        assert(0==std::fetestexcept(FE_INVALID));
        
        assert(std::numeric_limits<Dst>::lowest()<=tRounded && tRounded<=std::numeric_limits<Dst>::max());
        return static_cast<Dst>(tRounded);
    }
    
    template<typename Dst, typename Src>
    std::enable_if_t<std::is_floating_point<Dst>::value && std::is_integral<Src>::value,
    Dst> numeric_cast(Src t) {
        assert(std::numeric_limits<Dst>::lowest()<=t && t<=std::numeric_limits<Dst>::max());
        return static_cast<Dst>(t);
    }
    
    template<typename T>
    int sign(T const& t) {
        return 0<t
            ? 1
            : (t<0 ? -1 : 0);
    }
    
    template<typename T>
    T sqr(T const& t) { // TODO: introduce rbt::sqr_type<T>, so int * int -> int64?
        return t*t;
    }
    
    double angularDistance( double fAngleA, double fAngleB);
}

#endif /* math_h */
