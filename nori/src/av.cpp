#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class AverageVisibilityIntegrator : public Integrator {
public:
    AverageVisibilityIntegrator(const PropertyList &props) {
        length = props.getFloat("length");
        std::cout << "Length value was : " << length << std::endl;
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        /* Find the surface that is visible in the requested direction */
        Intersection its;
        if (!scene->rayIntersect(ray, its)) {
            return Color3f(1.0f);
        } 

        
        Vector3f v  = Warp::sampleUniformHemisphere(sampler, its.shFrame.n);  
        Ray3f irray(its.p, v); 
        irray.maxt = length;

        if (!scene->rayIntersect(irray, its)) {
            return Color3f(1.0f);
        } 
        
        return Color3f(0.0f);
               
    }

    
    /// Return a human-readable description for debugging purposes
    std::string toString() const {
        return tfm::format(
            "AverageVisibilityIntegrator[\n"
            "  length = \"%s\"\n"
            "]",
            length
        );
    }

protected:
    float length; 
    
};

NORI_REGISTER_CLASS(AverageVisibilityIntegrator, "av");
NORI_NAMESPACE_END