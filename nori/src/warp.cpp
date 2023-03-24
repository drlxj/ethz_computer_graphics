/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

Vector3f Warp::sampleUniformHemisphere(Sampler *sampler, const Normal3f &pole) {
    // Naive implementation using rejection sampling
    Vector3f v;
    do {
        v.x() = 1.f - 2.f * sampler->next1D();
        v.y() = 1.f - 2.f * sampler->next1D();
        v.z() = 1.f - 2.f * sampler->next1D();
    } while (v.squaredNorm() > 1.f);

    if (v.dot(pole) < 0.f)
        v = -v;
    v /= v.norm();

    return v;
}

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    float r = sqrt(sample(0));
    float theta = 2 * M_PI * sample(1);
    return Point2f(r * cos(theta), r * sin(theta));
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
    return (p.norm() < 1) ? 1.0f / M_PI : 0.0f;
}

Vector3f Warp::squareToUniformSphereCap(const Point2f &sample, float cosThetaMax) {
    float w_z = cosThetaMax + (1 - cosThetaMax) * sample(0);
    float r = sqrt(1 - pow(w_z, 2));
    float phi = 2 * M_PI * sample(1);
    float w_x = r * cos(phi);
    float w_y = r * sin(phi);
    return Vector3f(w_x, w_y, w_z);
}

float Warp::squareToUniformSphereCapPdf(const Vector3f &v, float cosThetaMax) {
    Vector3f normal = Vector3f(0, 0, 1);
    return ((abs(1 - v.norm()) < Epsilon) && (v.z() > cosThetaMax)) ? 1.0f / (2 * M_PI * (1 - cosThetaMax)) : 0.0f;
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    float w_z = 2 * sample(0) - 1;
    float r = sqrt(1 - pow(w_z, 2));
    float phi = 2 * M_PI * sample(1);
    float w_x = r * cos(phi);
    float w_y = r * sin(phi);
    return Vector3f(w_x, w_y, w_z);
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    return (abs(1 - v.norm()) < Epsilon) ? 1.0f / (4 * M_PI) : 0.0f;
    // return 1.0f / (4 * M_PI);
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    float w_z = sample(0);
    float r = sqrt(1 - pow(w_z, 2));
    float phi = 2 * M_PI * sample(1);
    float w_x = r * cos(phi);
    float w_y = r * sin(phi);
    return Vector3f(w_x, w_y, w_z);
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    Vector3f normal = Vector3f(0, 0, 1);
    return ((v.dot(normal) > 0.f) && (abs(1 - v.norm()) < Epsilon)) ? 1.0f / (2 * M_PI) : 0.0f;
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    auto phi = 2 * M_PI * sample.x();
    auto theta =  acos(sqrt(sample.y()));
    return {sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta)};
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    Vector3f normal = Vector3f(0, 0, 1);
    float cosTheta = normal.dot(v);
    return ((v.dot(normal) > 0.f) && (abs(1 - v.norm()) < Epsilon)) ? cosTheta / M_PI : 0.0f;
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    float phi = 2 * M_PI * sample.x();
    float theta = atan(sqrt(-pow(alpha, 2.f) * log(1 - sample.y())));
    return {sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta)};
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    float theta = acos(m.z());
    if (abs(1 - m.norm()) > Epsilon || m.z() < 0) {
        return 0;
    }
    return exp(-pow(tan(theta), 2) / pow(alpha, 2)) / (M_PI * pow(alpha, 2) * pow(cos(theta), 3));
}

Vector3f Warp::squareToUniformTriangle(const Point2f &sample) {
    float su1 = sqrtf(sample.x());
    float u = 1.f - su1, v = sample.y() * su1;
    return Vector3f(u,v,1.f-u-v);
}

NORI_NAMESPACE_END
