#ifndef _QUATERNION_HPP_
#define _QUATERNION_HPP_

#include "typedefs.hpp"
#include "Vector3.hpp"
#include "Matrix3x3.hpp"
#include <cmath>

class Quaternion {

public:

    Quaternion(tReal a, Vec3f omega) {
        s = a;
        x = omega.x;
        y = omega.y;
        z = omega.z;
    }

    // assignment operators
   
    Quaternion& operator*=(const tReal& r) {
        s *= r; 
        x *= r; 
        y *= r; 
        z *= r;
        return *this;
    }
    Quaternion& operator/=(const tReal& r) {
        s /= r; 
        x /= r; 
        y /= r; 
        z /= r;
        return *this;
    }
    Quaternion& operator+=(const Quaternion& q) {
        s += q.s;
        x += q.x;
        y += q.y;
        z += q.z;
        return *this;
    }
    Quaternion& operator-=(const Quaternion& q) {
        s -= q.s;
        x -= q.x;
        y -= q.y;
        z -= q.z;
        return *this;
    }
    Quaternion operator*=(const Quaternion& q) {
        return Quaternion(*this) * q;
    }

    Quaternion conjugate() const {
        return Quaternion(s, Vec3f(-x, -y, -z));
    }
    Quaternion operator/=(const Quaternion& q) {
        return *this *= q.conjugate();
    }

    // binary operators

    Quaternion operator*(const tReal& r) const { return Quaternion(*this) *= r; }
    Quaternion operator/(const tReal& r) const { return Quaternion(*this) /= r; }

    Quaternion operator+(const Quaternion& q) const { return Quaternion(*this) += q; }
    Quaternion operator-(const Quaternion& q) const { return Quaternion(*this) -= q; }
    Quaternion operator*(const Quaternion& q) const {
        tReal a = s * q.s - x * q.x - y * q.y - z * q.z;
        Vec3f omega = Vec3f(s * q.x + x * q.s - y * q.z + z * q.y,
                            s * q.y + x * q.z + y * q.s - z * q.x,
                            s * q.z - x * q.y + y * q.x - z * q.s);
        return Quaternion(a, omega);
    }
    Quaternion operator/(Quaternion& q) const {return Quaternion(*this) /= q; }
   


    Mat3f ToRotMat() {
        return Mat3f(
            1 - 2 * pow(y, 2) - 2 * pow(z, 2), 2 * x * y - 2 * s * z, 2 * x * z + 2 * s * y,
            2 * x * y + 2 * s * z, 1 - 2 * pow(x, 2) - 2 * pow(z, 2), 2 * y * z - 2 * s * x,
            2 * x * z - 2 * s * y, 2 * y * z + 2 * s * x, 1 - 2 * pow(x, 2) - 2 * pow(y, 2)
        );
    }


    Quaternion& normalize() { return *this = normalized(); }
    Quaternion normalized() {
        return Quaternion(*this) / norm();
    }
    
    tReal norm() { //calcola la norma (radice quadrata della norma al quadrato)
        return sqrt(pow(s, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2));
    }

    tReal s, x, y, z;
};


#endif  /* _MATRIX3X3_HPP_ */