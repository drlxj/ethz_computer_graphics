#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class DirectEmsIntegrator : public Integrator {
public:
    DirectEmsIntegrator(const PropertyList &props) {}

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        /* Find the surface that is visible in the requested direction */
        Intersection its;
        if (!scene->rayIntersect(ray, its)){
            return Color3f(0);
        }

        Color3f Lo(0);

        // Le(p, w0)
        if (its.mesh->isEmitter()) {
            EmitterQueryRecord lRec(ray.o, its.p, its.shFrame.n);
            auto Le = its.mesh->getEmitter()->eval(lRec);
            Lo += Le;
        }

        for (auto light : scene->getLights()) {
            //reflected
            EmitterQueryRecord lRec;
            lRec.ref = its.p;
            Color3f value = light->sample(lRec, sampler->next2D());

            // If ray is occluded
            if (scene->rayIntersect(lRec.shadowRay)) {
                continue;
            }

            // Convert to local frame
            auto localLRec = its.shFrame.toLocal(lRec.wi);
            auto localRay = its.shFrame.toLocal(-ray.d);

            // Cosine value between shading normal and lRec
            auto cosineTerm = Frame::cosTheta(localLRec);

            // Evaluate the BSDF for a pair of directions (lRec and ray)
            BSDFQueryRecord bsdfRec(localRay, localLRec, ESolidAngle);
            bsdfRec.uv = its.uv;
            auto bsdf = its.mesh->getBSDF()->eval(bsdfRec);

            Lo += value * cosineTerm * bsdf;
        }

        return Lo;
    }

    std::string toString() const {
        return "DirectEmsIntegrator[]";
    }

};

NORI_REGISTER_CLASS(DirectEmsIntegrator, "direct_ems");
NORI_NAMESPACE_END