#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class DirectIntegrator : public Integrator {
public:
    DirectIntegrator(const PropertyList &props) {}

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        /* Find the surface that is visible in the requested direction */
        Intersection its;
        if (!scene->rayIntersect(ray, its)){
            return Color3f(0);
        }

        Color3f color(0);

        for (auto light : scene->getLights()) {
            EmitterQueryRecord lRec;
            lRec.ref = its.p;
            Color3f value = light->sample(lRec, sampler->next2D());

            // No direct ray to light source possible
            if (scene->rayIntersect(lRec.shadowRay)) {
                continue;
            }

            // Convert to local frame
            auto localRay = its.shFrame.toLocal(-ray.d);
            auto localLRec = its.shFrame.toLocal(lRec.wi);

            // Cosine value between shading normal and lRec
            auto cosineTerm = Frame::cosTheta(localLRec);

            // Evaluate the BSDF for a pair of directions (lRec and ray)
            BSDFQueryRecord bsdfRec(localLRec, localRay, ESolidAngle);
            bsdfRec.uv = its.uv;
            auto fr = its.mesh->getBSDF()->eval(bsdfRec);

            color += value * cosineTerm * fr;
        }

        return color;
    }

    std::string toString() const {
        return "DirectIntegrator[]";
    }

};

NORI_REGISTER_CLASS(DirectIntegrator, "direct");
NORI_NAMESPACE_END