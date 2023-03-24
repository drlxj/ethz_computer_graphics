#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>

NORI_NAMESPACE_BEGIN

class DirectMis : public Integrator {
public:
    DirectMis(const PropertyList &props) {
        /* No parameters this time */

    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        /* Find the surface that is visible in the requested direction */
        Intersection its;
        if (!scene->rayIntersect(ray, its)){
            return Color3f(0.0f);
        }
            
        // check if the direction is emitter
        Color3f radiance_e(0);
        if (its.mesh->isEmitter()){
            EmitterQueryRecord lRec(ray.o, its.p, its.shFrame.n);
            radiance_e = its.mesh->getEmitter()->eval(lRec);
            // radiance_r += radiance_e;

        }

        // Ems
        Color3f radiance_ems(0);

        // intersect with object
        auto light = scene->getLights();
        for (unsigned long i = 0; i < light.size(); i++){
            // 1. get light radiance over pdf
            EmitterQueryRecord lRec;
            lRec.ref = its.p;
            auto Li_pdf = light[i]->sample(lRec, sampler->next2D());
            auto w_em = light[i]->pdf(lRec);
                // if occlusion (intersect before its.p)
            if (scene->rayIntersect(lRec.shadowRay)) {
                continue;
            }
            // 2. set BRDF property
                // frame transformation
            auto local_wo = its.shFrame.toLocal(lRec.wi);
            auto local_wi = its.shFrame.toLocal(-ray.d);
            BSDFQueryRecord bRec(local_wi, local_wo, ESolidAngle);
            bRec.uv = its.uv;
            auto bsdf = its.mesh->getBSDF()->eval(bRec);
            auto w_mat = its.mesh->getBSDF()->pdf(bRec);
        
            if (w_em + w_mat >= Epsilon){
                radiance_ems += w_em / (w_em + w_mat) * Li_pdf * bsdf * Frame::cosTheta(local_wo);
            }
            

        }
        

        //Mats
        Color3f radiance_mat(0);
        
        // sample BRDF
        // 1. bRec: local_wo, frame
        auto local_wo = its.shFrame.toLocal(-ray.d);
        BSDFQueryRecord bRec(local_wo);
        bRec.uv = its.uv;
        // 2. sample
        auto bsdf_cos_pdf = its.mesh->getBSDF()->sample(bRec, sampler->next2D());   //return bsdf * cos(theta_o) / pdf, also assign the wo
        auto w_mat = its.mesh->getBSDF()->pdf(bRec);
        
        // Get Emitter (from BRDF sample)
        // 1. get ref point from bRec.wi
        Ray3f ray_wi(its.p, its.shFrame.toWorld(bRec.wo));
        Intersection its_wi;
        if (scene->rayIntersect(ray_wi, its_wi) && its_wi.mesh->isEmitter()){
            // 2.lRec
            EmitterQueryRecord lRec(its.p, its_wi.p, its_wi.shFrame.n);
            // 3. get value
            Color3f radiance_i = its_wi.mesh->getEmitter()->eval(lRec);
            auto w_em = its_wi.mesh->getEmitter()->pdf(lRec);

            if (w_em + w_mat >= Epsilon){
                    radiance_mat += w_mat / (w_em + w_mat) * radiance_i * bsdf_cos_pdf;
                }
        }
        
        
        // return radiance_r;
        return radiance_e + radiance_ems + radiance_mat;

    }

    std::string toString() const {
        return "DirectMis[]";
    }
};

NORI_REGISTER_CLASS(DirectMis, "direct_mis");
NORI_NAMESPACE_END