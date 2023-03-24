#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/warp.h>
#include <nori/sampler.h>

NORI_NAMESPACE_BEGIN

class PathMatsIntegrator : public Integrator {
public:
    PathMatsIntegrator(const PropertyList) {}

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const override {
        // Initial radiance and throughput
        Color3f Li(0);
        Color3f t(1);
        
        Intersection x_o;
        Ray3f pathRay = ray;

        while(true) {

            bool is_intersectsScene = scene->rayIntersect(pathRay, x_o);

            // Surface has intersection
            if (is_intersectsScene) {
                if (x_o.mesh->isEmitter()) {
                    EmitterQueryRecord lRec(ray.o, x_o.p, x_o.shFrame.n);
                    Color3f Le = x_o.mesh->getEmitter()->eval(lRec);
                    Li += t * Le;
                }  

                // Sample from BSDF
                BSDFQueryRecord bRec(x_o.shFrame.toLocal(-pathRay.d)); // wi: -pathRay.d
                bRec.uv = x_o.uv;
                auto frCosThetaOverPdf = x_o.mesh->getBSDF()->sample(bRec, sampler->next2D());

                // Cast ray based on sample
                pathRay = Ray3f(x_o.p, x_o.shFrame.toWorld(bRec.wo)); // origin, direction
                t = t * frCosThetaOverPdf;

                                         
            } else {
                break;
            }  
            
            // Russian Rouelette
            float p = std::min(t.maxCoeff(), .99f);
            if (sampler->next1D() > p) {
                break;
            } 
            t = t / p;    
        }
        return Li;
    }

    std::string toString() const override {
        return "PathMatsIntegrator[]";
    }

};

NORI_REGISTER_CLASS(PathMatsIntegrator, "path_mats");
NORI_NAMESPACE_END