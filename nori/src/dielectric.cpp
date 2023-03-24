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

#include <nori/bsdf.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

/// Ideal dielectric BSDF
class Dielectric : public BSDF {
public:
    Dielectric(const PropertyList &propList) {
        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);
    }

    virtual Color3f eval(const BSDFQueryRecord &) const override {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return Color3f(0.0f);
    }

    virtual float pdf(const BSDFQueryRecord &) const override {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return 0.0f;
    }

    virtual Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const override {

        float cosThetaI = Frame::cosTheta(bRec.wi);
        float F = fresnel(cosThetaI, m_extIOR, m_intIOR);

        // The sampling code will make a random choice to determine how light is scattered:
        // the probability that a reflection or refraction event takes place 
        // is proportional to the reflection coefficient provided by the Fresnel equations
        if (sample.x() <= F) {
            // Reflection
            bRec.wo = Vector3f(
            -bRec.wi.x(),
            -bRec.wi.y(),
             bRec.wi.z()
            );
            bRec.eta = 1;
        } else {
            Vector3f n(0, 0, 1);
            float eta = m_extIOR / m_intIOR;
            if (cosThetaI < 0) {
                // inside
                n = -n;
                eta = 1/eta;
            }
            bRec.wo = ( -eta * (bRec.wi - bRec.wi.dot(n) * n) 
                - n * sqrt(1 - pow(eta, 2) * (1 - pow(bRec.wi.dot(n), 2)))
                ).normalized();
            bRec.eta = eta;
        }

        bRec.measure = EDiscrete;

        return Color3f(1.0f);
    }

    virtual std::string toString() const override {
        return tfm::format(
            "Dielectric[\n"
            "  intIOR = %f,\n"
            "  extIOR = %f\n"
            "]",
            m_intIOR, m_extIOR);
    }
private:
    float m_intIOR, m_extIOR;
};

NORI_REGISTER_CLASS(Dielectric, "dielectric");
NORI_NAMESPACE_END
