#include "triangles.hpp"

// Point intersection(const Line& first, const Line& second) {
//     if (first.parallel(second)) {
//         return Point{NAN, NAN, NAN};
//     }

    
// }
 
bool intersect_triangles(const Triangle& first_tri, const Triangle& second_tri) {
    Plane first_pl  = first_tri.find_plane();
    Plane second_pl = second_tri.find_plane();

    if (first_pl.parallel(second_pl)) { return false; }

    if (first_pl.match(second_pl)) {
        //intersect_2d(); 
    }

    else {
        //intersect_3d();
    }
}