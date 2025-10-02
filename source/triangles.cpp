#include "triangles.hpp"

Point intersection(const Line& first, const Line& second) {
    if (first.parallel(second)) {
        return Point{NAN, NAN, NAN};
    }

    
}