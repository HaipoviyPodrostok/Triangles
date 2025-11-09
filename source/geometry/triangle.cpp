#include "geometry/triangle.hpp"
#include "geometry/plane.hpp"
#include "geometry/section.hpp"
#include "geometry/vector_3d.hpp"
#include "math/math_utils.hpp"
#include <cassert>
#include <cstddef>
#include <iostream>
#include <vector>

namespace geometry {

Triangle::Triangle(const Vector3D& a, const Vector3D& b, const Vector3D& c)
    : a(a), b(b), c(c) {
        centroid = get_centre();
    };

bool Triangle::is_valid() const {
    const Section ab {a, b};
    const Section bc {b, c};
    const Section ac {a, c};
    
    return a.is_valid()  && b.is_valid()  && c.is_valid() &&
           ab.is_valid()  && ac.is_valid()  && bc.is_valid() &&
           (ab.length() + bc.length() > ac.length())         &&
           (ab.length() + ac.length() > bc.length())         &&
           (bc.length() + ac.length() > ab.length());
}

void Triangle::print() const {
    std::cout << "A = ";   a.print();
    std::cout << ", B = "; b.print();
    std::cout << ", C = "; c.print();
}

bool Triangle::intersection(const Triangle& other) const {
    assert(this->is_valid());
    assert(other.is_valid());
    
    const Plane first_pl  = this->get_plane();
    const Plane second_pl = other.get_plane();
    
    if (first_pl.is_match(second_pl)) {
        return is_intersect_2d(other);
    }

    if (first_pl.is_parallel(second_pl)) { return false; }

    return is_intersect_3d(other);
}

Plane Triangle::get_plane() const {
    assert(this->is_valid());
    
    const Vector3D ab = b - a;
    const Vector3D ac = c - a;

    const Vector3D normal = ab.cross(ac);

    return Plane{a, normal};
}

bool Triangle::is_inside(const Vector3D& p) const {
    assert(this->is_valid());
    assert(p.is_valid());
    assert(this->get_plane().is_contains(p));

    const Vector3D ab = b - a;
    const Vector3D bc = c - b;
    const Vector3D ca = a - c;
    const Vector3D ap = p - a;
    const Vector3D bp = p - b;
    const Vector3D cp = p - c;
    
    const Vector3D normal = ab.cross(bc);

    const float side_ab = ( normal.cross(ab) ).scalar(ap);
    const float side_bc = ( normal.cross(bc) ).scalar(bp);
    const float side_ca = ( normal.cross(ca) ).scalar(cp);

    return (side_ab >= - math::eps && side_bc >= - math::eps && side_ca >= - math::eps) ||
           (side_ab <=   math::eps && side_bc <=   math::eps && side_ca <=   math::eps);
}

bool Triangle::is_intersect_2d(const Triangle& other) const {
    assert(this->is_valid());
    assert(other.is_valid());
    if ( !(this->get_plane().is_match(other.get_plane())) ) { return false;}

    const Section ab {a, b};
    const Section bc {b, c};
    const Section ca {c, a};
    const Section other_ab {other.a, other.b};
    const Section other_bc {other.b, other.c};
    const Section other_ca {other.c, other.a};

    const Section* sides1[3] = {&ab, &bc, &ca};
    const Section* sides2[3] = {&other_ab, &other_bc, &other_ca};
    
    if (is_inside(other.a) || is_inside(other.b) || is_inside(other.c)) {
        return true;
    }

    if (other.is_inside(a) || other.is_inside(b) || other.is_inside(c)) {
        return true;
    }
    
    for (size_t i = 0; i < 3; i++) {
        for (size_t j = 0; j < 3; j++) {
            if (sides1[i]->is_intersect(*sides2[j])) {
                return true;
            };
        }
    }

    return false;
}

bool Triangle::is_intersect_3d(const Triangle& other) const {
    assert(this->is_valid());
    assert(other.is_valid());

    const Section ab {a, b};
    const Section bc {b, c};
    const Section ca {c, a};
    const Section other_ab {other.a, other.b};
    const Section other_bc {other.b, other.c};
    const Section other_ca {other.c, other.a};
    
    const Line intersect_line = get_intersect_line(other);   
    assert(intersect_line.is_valid());
    
    const Section* sides1[3] = {&ab, &bc, &ca};
    std::vector<Vector3D> intersect_points1;

    for (size_t i = 0; i < 3; i++) {
        if (sides1[i]->is_belong(intersect_line)) {
            intersect_points1.push_back(sides1[i]->a_);
            intersect_points1.push_back(sides1[i]->b_);
        }
        else if (sides1[i]->is_intersect(intersect_line)) {         
            Vector3D p = sides1[i]->intersect_point(intersect_line);
            assert(p.is_valid());
            intersect_points1.push_back(p);
        }
    }

    if (intersect_points1.size() < 2) {
        return false;
    }

    const Section* sides2[3] = {&other_ab, &other_bc, &other_ca};
    std::vector<Vector3D> intersect_points2;

    for (size_t i = 0; i < 3; i++) {
        if (sides2[i]->is_belong(intersect_line)) {
            intersect_points2.push_back(sides2[i]->a_);
            intersect_points2.push_back(sides2[i]->b_);
        }
        else if (sides2[i]->is_intersect(intersect_line)) {
            Vector3D p = sides2[i]->intersect_point(intersect_line);
            assert(p.is_valid());
            intersect_points2.push_back(p);
        }
    }

    if (intersect_points2.size() < 2) {
        return false;
    }

    if (intersect_points1[0].is_match(intersect_points1[1])) {
        Vector3D p1 = intersect_points1[0];

        if (intersect_points2[0].is_match(intersect_points2[1])) {
            Vector3D p2 = intersect_points2[0];
            return p1.is_match(p2);
        }

        Section s2(intersect_points2[0], intersect_points2[1]);
        return s2.is_contains(p1);
    }

    if (intersect_points2[0].is_match(intersect_points2[1])) {
        Vector3D p2 = intersect_points2[0];
        Section s1(intersect_points1[0], intersect_points1[1]);
        return s1.is_contains(p2);
    }

    const Section s1{intersect_points1[0], intersect_points1[1]};
    const Section s2{intersect_points2[0], intersect_points2[1]};

    return s1.is_intersect(s2);
}

Line Triangle::get_intersect_line(const Triangle& other) const {
    assert(this->is_valid());
    assert(other.is_valid());
    
    const Plane pl1 = get_plane();
    const Plane pl2 = other.get_plane();
    
    assert(!pl1.is_parallel(pl2)); 
    assert(!pl1.is_match(pl2));

    const float D1 = pl1.D();
    const float D2 = pl2.D();
    
    const Vector3D n1 = pl1.normal();
    const Vector3D n2 = pl2.normal(); 

    const Vector3D dir = n1.cross(n2);

    const Vector3D pl_intersect_p = (n2 * D1 - n1 * D2).cross(n1.cross(n2)) /
                                        math::sqr(( n1.cross(n2) ).length());

    return {pl_intersect_p, dir};
}

Vector3D Triangle::get_centre() const {
    assert(this->is_valid());

    return {(a.x + b.x + c.x) / 3.0f, 
            (a.y + b.y + c.y) / 3.0f,
            (a.z + b.z + c.z) / 3.0f };
}
} // namespace geometry