#pragma once

#include "crl-basic/utils/mathUtils.h"

namespace crl::utils {

class Segment {
public:
    P3D a = P3D(0, 0, 0);
    P3D b = P3D(0, 0, 0);

    Segment() {}

    Segment(const P3D &a_, const P3D &b_) {
        this->origin = a_;
        this->dir = b_;
    }

private:
    P3D origin;
    P3D dir;
};

class Plane {
public:
    // a plane is defined by its normal, and a point that lies on it
    V3D n = V3D(0, 1, 0);
    P3D p = P3D(0, 0, 0);

public:
    Plane(void);
    Plane(const P3D &p, const V3D &n) {
        this->n = n.normalized();
        this->p = p;
    }
    Plane(const P3D &p1, const P3D &p2, const P3D &p3) {
        this->p = p1;
        this->n = (V3D(p1, p2).cross(V3D(p2, p3))).normalized();
    }
    ~Plane(void) {}

    double getSignedDistanceToPoint(const P3D &pt) const {
        return V3D(p, pt).dot(n);
    }

    Plane &operator=(const Plane &other);

    // get the coefficients that define the cartesian equation of the plane: ax
    // + by + cz + d = 0
    void getCartesianEquationCoefficients(double &a, double &b, double &c, double &d) {
        a = n[0];
        b = n[1];
        c = n[2];
        d = -(n[0] * p.x + n[1] * p.y + n[2] * p.z);
    }
};

class Ray {
public:
    P3D origin = P3D(0, 0, 0);
    V3D dir = V3D(0, 0, 1);

    Ray() = default;

    Ray(const P3D &o, const V3D &d);

    /**
     * returns the point on the ray with distance 't' from the origin
     */
    P3D getPointAt(double t) const;

    /**
     * returns the 't' value away from origin where p can be found
     */
    double getRayParameterFor(const P3D &p) const;

    /**
     * returns the smallest distance between the line segment and the ray.
     * If ClosestPtOnRay is not nullptr, it will be set to the point on the ray that is closest to the segment
     */
    double getDistanceToSegment(const P3D &p1, const P3D &p2, P3D *closestPtOnRay) const;

    /**
     * returns the point on the ray that is closest to c.
     * If ClosestPtOnRay is not nullptr, it will be set to the point on the ray that is closest to c
     */
    double getDistanceToPoint(const P3D &p, P3D *closestPtOnRay) const;

    /**
     * returns the smallest distance between the line segment and the ray.
     * and if ClosestPtOnRay is not NULL, it will be set to the point on the ray that is closest to c
     */
    double getDistanceToPlane(const Plane &plane, P3D *closestPtOnRay) const;
};

};  // namespace crl::utils