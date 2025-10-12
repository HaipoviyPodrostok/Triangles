#include "geometry/section.hpp"
#include "geometry/line.hpp"
#include "geometry/vector_3d.hpp"
#include "math/math_utils.hpp"
#include <cassert>

namespace geometry {

Section::Section(const Vector3D& a, const Vector3D& b)
    : a_(a), b_(b) { }

bool Section::is_valid() const {
    return a_.is_valid() &&
           b_.is_valid() &&
           !((a_ - b_).is_zero());
}

Line Section::get_line() const {
    return Line{a_, b_ - a_};
}

bool Section::is_intersect(const Section& other) const {
    assert(is_valid());
    assert(other.is_valid());

    Line first_line  = get_line();
    Line second_line = other.get_line();

    // 1️⃣ Если линии не параллельны — обычное 3D пересечение
    if (first_line.is_intersect(second_line)) {
        Vector3D p = first_line.intersect_point(second_line);
        if (is_contains(p) && other.is_contains(p))
            return true;
        return false;
    }

    // 2️⃣ Если линии параллельны — проверяем коллинеарность
    if (first_line.is_parallel(second_line)) {
        // Если не лежат на одной прямой — нет пересечения
        if (!first_line.is_contains(other.a_))
            return false;

        // Коллинеарные: проверяем перекрытие проекций на главную ось
        Vector3D dir = first_line.dir();
        float ax = std::fabs(dir.x());
        float ay = std::fabs(dir.y());
        float az = std::fabs(dir.z());

        int axis = 0;
        if (ay > ax && ay > az) axis = 1;
        else if (az > ax && az > ay) axis = 2;

        float a1, a2, b1, b2;

        switch (axis) {
            case 0: // X
                a1 = a_.x(); a2 = b_.x();
                b1 = other.a_.x(); b2 = other.b_.x();
                break;
            case 1: // Y
                a1 = a_.y(); a2 = b_.y();
                b1 = other.a_.y(); b2 = other.b_.y();
                break;
            default: // Z
                a1 = a_.z(); a2 = b_.z();
                b1 = other.a_.z(); b2 = other.b_.z();
                break;
        }

        if (a1 > a2) std::swap(a1, a2);
        if (b1 > b2) std::swap(b1, b2);

        // Перекрываются ли проекции (учитываем допуск)
        return !(a2 < b1 - flt_tolerance || b2 < a1 - flt_tolerance);
    }

    // 3️⃣ В остальных случаях (разные плоскости) — нет пересечения
    return false;
}

float Section::length() const {
    return (a_ - b_).length();
}

Side Section::get_side(const Vector3D& p) const {
    Vector3D ab = b_ - a_;
    Vector3D ap = p  - a_;

    Vector3D n = ab.cross(ap);
    float n_length = n.length();

    if (n_length > 0) { return LEFT_SIDE; }
    if (n_length < 0) { return  RIGHT_SIDE; }

    return INTER_SIDE;
}

bool Section::is_contains(const Vector3D& p) const { // TODO failed test
    Vector3D ap = p - a_;
    Vector3D ab = b_ - a_;
    Vector3D bp = p - b_;

    if ((ap).is_collinear(ab)) {
        float scalar_ab_bp = ap.scalar(bp);
        if (scalar_ab_bp < 0 || math::is_zero(scalar_ab_bp)) {
            return true;
        }
    }

    return false;
}

void Section::print() const {
    std::cout << "p1 = ";   a_.print();
    std::cout << ", p2 = "; b_.print();
    std::cout << std::endl;
}
} // namespace geometry 