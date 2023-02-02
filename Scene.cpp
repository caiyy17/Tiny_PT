//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

float Scene::pdfLight(Intersection &inter) const {
    float emit_power_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            Vector3f E = objects[k]->getE();
            float power = E.norm();
            emit_power_sum += objects[k]->getArea() * power;
        }
    }
    float emit_power_sum_0 = emit_power_sum;
    Vector3f E = inter.obj->getE();
    float power = E.norm();
    emit_power_sum = inter.obj->getArea() * power;
    return (1/inter.obj->getArea()) * emit_power_sum / emit_power_sum_0;
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_power_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            Vector3f E = objects[k]->getE();
            float power = E.norm();
            emit_power_sum += objects[k]->getArea() * power;
        }
    }
    float emit_power_sum_0 = emit_power_sum;
    float p = get_random_float() * emit_power_sum;
    emit_power_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            Vector3f E = objects[k]->getE();
            float power = E.norm();
            emit_power_sum += objects[k]->getArea() * power;
            if (p <= emit_power_sum){
                objects[k]->Sample(pos, pdf);
                pdf = pdf * (objects[k]->getArea() * power) / emit_power_sum_0;
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    /*
    1 shade (p, wo) //
    2 sampleLight (inter , pdf_light )
    3 Get x, ws , NN , emit from inter
    4 Shoot a ray from p to x
    5 If the ray is not blocked in the middle
    6 L_dir = emit * eval(wo , ws , N) * dot(ws , N) * dot(ws ,
                                                           NN) / |x-p|^2 / pdf_light
    7
    8 L_indir = 0.0
    9 Test Russian Roulette with probability RussianRoulette
    10 wi = sample (wo , N)
    11 Trace a ray r(p, wi)
    12 If ray r hit a non - emitting object at q
    13 L_indir = shade (q, wi) * eval (wo , wi , N) * dot(wi , N)
                 / pdf(wo , wi , N) / RussianRoulette
    14
    15 Return L_dir + L_indir

    */
    Intersection intersection = Scene::intersect(ray);
    Material *m = intersection.m;
    Object *hitObject = intersection.obj;
    Vector3f hitColor(0,0,0);
    //hitColor = this->backgroundColor;
//    float tnear = kInfinity;
    Vector2f uv;
    uint32_t index = 0;
    if(intersection.happened) {
        Vector3f hitPoint = intersection.coords;
        Vector3f N = intersection.normal; // normal
        Vector2f st; // st coordinates
        hitObject->getSurfaceProperties(hitPoint, ray.direction, index, uv, N, st);
        auto wo = ray.direction;
        if (m->hasEmission()) {
            if(depth == 0){
                return m->m_emission;
            }
            return Vector3f(0,0,0);
        }
        //sampleLight
        float probability = 0.5;
        Vector3f L_dir = Vector3f(0,0,0);
        Intersection inter;
        if (get_random_float() < probability){
            float pdf_light;
            sampleLight(inter, pdf_light);
            auto x = inter.coords;
            auto emit = inter.emit;
            auto NN = inter.normal;
            auto lightDir = x - hitPoint;
            float lightDistance2 = dotProduct(lightDir, lightDir);
            lightDir = normalize(lightDir);
            float LdotN = std::max(0.f, dotProduct(lightDir, N));
            float LdotNN = std::max(0.f, dotProduct(-lightDir, NN));
            Vector3f shadowPointOrig = (dotProduct(wo, N) < 0) ?
                                       hitPoint + N * EPSILON :
                                       hitPoint - N * EPSILON;
//        Vector3f shadowPointOrig = hitPoint;
            Ray testRay = Ray(shadowPointOrig, lightDir);
            Intersection test = Scene::intersect(testRay);
            if (test.happened == true) {
                auto testlightDir = test.coords - hitPoint;
                float testlightDistance2 = dotProduct(testlightDir, testlightDir);
                if(testlightDistance2 >= (lightDistance2 - EPSILON * 5 * sqrt(lightDistance2))){
                    L_dir = emit * m->eval(wo, lightDir, N) * LdotN * LdotNN
                            / lightDistance2 / pdf_light;
                }
            }
            float weight = probability * pdf_light / (probability * pdf_light + (1-probability) * m->pdf(wo , lightDir , N));
            L_dir = L_dir * weight / probability;
            //L_dir = Vector3f(0);

        }
        else {
            auto wi = m->sample(wo,N).normalized();
            Vector3f PointOrig = (dotProduct(ray.direction, N) < 0) ?
                                 hitPoint + N * EPSILON :
                                 hitPoint - N * EPSILON;
            Ray testRayInd = Ray(PointOrig, wi);
            Intersection testInd = Scene::intersect(testRayInd);
            if(testInd.happened == 1 && testInd.distance >= 0 && testInd.m->hasEmission()){
                auto emit = testInd.m->m_emission;
                auto NN = testInd.normal;
                auto LDir = testInd.coords - hitPoint;
                float Distance2 = dotProduct(LDir, LDir);
                float wiDotN = std::max(0.f,dotProduct(wi , N));
                float LdotNN = std::max(0.f, dotProduct(-wi, NN));

                L_dir = emit * m->eval(wo, wi, N) * wiDotN * LdotNN
                        / m->pdf(wo , wi , N);
                float pdf_light = pdfLight(testInd);
                float weight = (1-probability) * m->pdf(wo , wi , N) / (probability * pdf_light + (1-probability) * m->pdf(wo , wi , N));
                L_dir = L_dir * weight / (1-probability);
                //L_dir = Vector3f(0);
            }
        }

        if(m->m_type == REFLECT){
            //L_dir = Vector3f(0.0f);
        }
        //sampleIndirect

        Vector3f L_indir(0,0,0);

        if(get_random_float() < RussianRoulette){
            auto wi = m->sample(wo,N).normalized();
            Vector3f PointOrig = (dotProduct(ray.direction, N) < 0) ?
                                 hitPoint + N * EPSILON :
                                 hitPoint - N * EPSILON;
//            Vector3f PointOrig = hitPoint;
            Ray testRayInd = Ray(PointOrig, wi);
            Intersection testInd = Scene::intersect(testRayInd);
            if(testInd.happened == 1 && testInd.distance >= 0 && !testInd.m->hasEmission()){
                float wiDotN = std::max(0.f,dotProduct(wi , N));
                depth++;
                if(m->m_type == REFLECT){
                //    depth = 0;
                }
                if (m->pdf(wo , wi , N) > EPSILON) {
                    L_indir = Scene::castRay(Ray(PointOrig, wi),depth) * m->eval(wo , wi , N) * wiDotN
                              / m->pdf(wo , wi , N) / RussianRoulette;
                }

            }

        }

        //return L_indir;
        return L_dir + L_indir;

    }

    return hitColor;
}