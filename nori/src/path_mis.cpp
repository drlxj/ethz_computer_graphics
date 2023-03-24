#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>
#include <nori/warp.h> 

NORI_NAMESPACE_BEGIN

class PathMisIntegrator : public Integrator {
public:
    PathMisIntegrator(const PropertyList &props) {}

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const override {
        // Initial radiance and throughput
        Color3f Li(0);
        Color3f t(1);

        Intersection x_o;
        Ray3f pathRay = ray;

        auto w_em = 1.f;
        auto w_mat = 1.f;

        while (true) {
            if (scene->rayIntersect(pathRay, x_o)) {
                // Contribution from mats
                if (x_o.mesh->isEmitter()) {
                    EmitterQueryRecord lRec(ray.o, x_o.p, x_o.shFrame.n);
                    Color3f Le = x_o.mesh->getEmitter()->eval(lRec);
                    Li += w_mat * t * Le;
                } 

                // Russian Rouelette
                float p = std::min(t.maxCoeff(), .99f);
                if (sampler->next1D() > p) {
                    break;
                } else {t = t / p;}   

                // Contribution from emitter sampling
                auto light = scene->getRandomEmitter(sampler->next1D());
                EmitterQueryRecord lRec(x_o.p);
                Color3f LeOverPdf = light->sample(lRec, sampler->next2D()) * scene->getLights().size();
                // Visibility test uses shadowRay
                if (!scene->rayIntersect(lRec.shadowRay)) {
                    auto wi = x_o.shFrame.toLocal(-pathRay.d);
                    auto wo = x_o.shFrame.toLocal(lRec.wi);
                    auto cosTheta = Frame::cosTheta(wo);

                    BSDFQueryRecord bsdfRec(wi, wo, ESolidAngle);
                    bsdfRec.uv = x_o.uv;
                    auto fr = x_o.mesh->getBSDF()->eval(bsdfRec);

                    auto pdfEm = light->pdf(lRec);
                    auto pdfMat = x_o.mesh->getBSDF()->pdf(bsdfRec);

                    if (pdfEm + pdfMat >= Epsilon) {
                        w_em = pdfEm / (pdfEm + pdfMat);
                    }

                    Li += w_em * t * (fr * LeOverPdf * cosTheta);
                }
                
                // Sample from BSDF
                BSDFQueryRecord bRec(x_o.shFrame.toLocal(-pathRay.d)); // wi: -pathRay.d
                bRec.uv = x_o.uv;
                auto frCosThetaOverPdf = x_o.mesh->getBSDF()->sample(bRec, sampler->next2D());

                // Cast ray based on sample
                pathRay = Ray3f(x_o.p, x_o.shFrame.toWorld(bRec.wo)); // origin, direction
                t *= frCosThetaOverPdf;

                // Compute new wMat
                if (bRec.measure == EDiscrete) {
                    w_mat = 1;
                } else {
                    Intersection its;
                    if (scene->rayIntersect(pathRay, its)) {
                        if (its.mesh->isEmitter()) {
                            EmitterQueryRecord itsERec(x_o.p, its.p, its.shFrame.n);
                            auto pdfMat = x_o.mesh->getBSDF()->pdf(bRec);
                            auto pdfEm = its.mesh->getEmitter()->pdf(itsERec);
                            if (pdfEm + pdfMat > 0) {
                                w_mat = pdfMat / (pdfEm + pdfMat);
                            }
                        }
                    }
                }
            } else { 
                break;
            }     
        }        
        return Li;
    }

    std::string toString() const override {
        return "PathMisIntegrator[]";
    }
};

NORI_REGISTER_CLASS(PathMisIntegrator, "path_mis");
NORI_NAMESPACE_END
