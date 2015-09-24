//
//  nonmoveable.h
//  robotcontrol2
//
//  Created by Sebastian Theophil on 24.09.15.
//  Copyright © 2015 Sebastian Theophil. All rights reserved.
//

#ifndef nonmoveable_h
#define nonmoveable_h

namespace rbt {
    struct nonmoveable {
        nonmoveable() = default;
        
        nonmoveable(nonmoveable&&) = delete;
        nonmoveable(nonmoveable const&) = delete;
        
        void operator=(nonmoveable&&) = delete;
        void operator=(nonmoveable const&) = delete;
    };
}

#endif /* nonmoveable_h */
