//
// Created by Dongho Kang on 08.07.23.
//

#include "crl-basic/utils/geoms.h"

namespace crl::utils {

Ray::Ray(const P3D &o, const V3D &d) {
    this->origin = o;
    this->dir = d;
}

P3D Ray::getPointAt(double t) const {
    return origin + dir * t;
}

double Ray::getRayParameterFor(const P3D &p) const {
    return dir.dot(V3D(origin, p));
}

double Ray::getDistanceToSegment(const P3D &p1, const P3D &p2, P3D *closestPtOnRay) const {
    // (c) Dan Sunday, http://geomalgorithms.com/a07-_distance.html
    V3D u = V3D(p1, origin) + dir * 10000;
    V3D v(p1, p2);
    V3D w(p1, origin);

    double a = u.squaredNorm();
    double b = u.dot(v);
    double c = v.squaredNorm();
    double d = u.dot(w);
    double e = v.dot(w);
    double denom = a * c - b * b;

    double sN, sD = denom;
    double tN, tD = denom;

    if (denom <= 1e-10) {
        // lines are almost parallel
        sN = 0;
        sD = 1;
        tN = e;
        tD = c;
    } else {
        sN = (b * e - c * d);
        tN = (a * e - b * d);
        if (sN < 0) {
            // the closest point is behind the ray's starting point
            sN = 0;
            tN = e;
            tD = c;
        }
    }

    if (tN < 0) {
        // the closest point is behind t1
        tN = 0;
        if (-d < 0)
            sN = 0;
        else if (-d > a)
            sN = sD;
        else {
            sN = -d;
            sD = a;
        }
    } else if (tN > tD) {
        // the closest point is behind t2
        tN = tD;
        if ((-d + b) < 0)
            sN = 0;
        else if ((-d + b) > a)
            sN = sD;
        else {
            sN = -d + b;
            sD = a;
        }
    }

    double sc = (abs(sN) < 1e-10) ? 0 : sN / sD;
    double tc = (abs(tN) < 1e-10) ? 0 : tN / tD;

    if (closestPtOnRay != nullptr)
        *closestPtOnRay = origin + dir * (sc * 10000);

    return (w + u * sc - v * tc).norm();
}

double Ray::getDistanceToPoint(const P3D &p, P3D *closestPtOnRay) const {
    double d = dir.dot(V3D(origin, p));
    P3D res = origin;
    if (d > 0)
        res = getPointAt(d);

    if (closestPtOnRay)
        *closestPtOnRay = res;

    return V3D(p, res).norm();
}

double Ray::getDistanceToPlane(const Plane &plane, P3D *closestPtOnRay) const{
    // check to see if the ray is parallel to the plane...
    if (fabs(dir.dot(plane.n)) < 10e-10) {
        if (closestPtOnRay)
            *closestPtOnRay = origin;
        return fabs(plane.getSignedDistanceToPoint(origin));
    }

    // we know that p = origin + t * direction lies on the plane, which
    // means that dot product between plane p and p, dotted with n is 0...
    V3D tmpV(origin, plane.p);
    double t = tmpV.dot(plane.n) / dir.dot(plane.n);

    // check the solution now...
    assert(IS_ZERO(V3D(plane.p, origin + dir * t).dot(plane.n)));

    if (t < 0) {
        if (closestPtOnRay)
            *closestPtOnRay = origin;
        return fabs(plane.getSignedDistanceToPoint(origin));
    }

    if (closestPtOnRay)
        *closestPtOnRay = origin + dir * t;

    return t;
}

}  // namespace crl::utils