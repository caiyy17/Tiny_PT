//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"



enum MaterialType { DIFFUSE, REFLECT, Microfacet};

class Material{
private:
    //GGX microfacet
    float GGX_D(Vector3f wm, Vector3f normal, float alpha) // alpha为粗糙度
    {
        float cosTheta2 = dotProduct(wm,normal) * dotProduct(wm,normal);
        if (dotProduct(wm,normal) < EPSILON){
            return 0.0f;
        }
        float sinTheta2 = 1-cosTheta2;
        float tanTheta2 = sinTheta2 / cosTheta2;

        float root = alpha / (cosTheta2 * (alpha * alpha + tanTheta2));

        return 1/M_PI * (root * root);
    }

    float SmithG1(Vector3f v, Vector3f wm, Vector3f normal, float alpha)
    {
        float cosTheta = dotProduct(v,normal);
        float sinTheta = sqrt(1-cosTheta*cosTheta);
        float tanTheta = sinTheta / cosTheta;
        tanTheta = abs(tanTheta);

        if (tanTheta < EPSILON){
            return 1.0f;
        }
        if (dotProduct(v, wm) * cosTheta < EPSILON){
            return 0.0f;
        }



        float root = alpha * tanTheta;
        float result = 2.0f / (1.0f + sqrt(1.0f + root*root));
        return result;
    }

    float Smith_G(Vector3f wo, Vector3f wi, Vector3f wm, Vector3f normal, float alpha)
    {
        return SmithG1(wo, wm, normal, alpha) * SmithG1(wi, wm, normal, alpha);
    }


    // Compute reflection direction
    Vector3f reflect(const Vector3f &I, const Vector3f &N) const
    {
        return I - 2 * dotProduct(I, N) * N;
    }

    // Compute refraction direction using Snell's law
    //
    // We need to handle with care the two possible situations:
    //
    //    - When the ray is inside the object
    //
    //    - When the ray is outside.
    //
    // If the ray is outside, you need to make cosi positive cosi = -N.I
    //
    // If the ray is inside, you need to invert the refractive indices and negate the normal N
    Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        Vector3f n = N;
        if (cosi < 0) { cosi = -cosi; } else { std::swap(etai, etat); n= -N; }
        float eta = etai / etat;
        float k = 1 - eta * eta * (1 - cosi * cosi);
        return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
    }

    // Compute Fresnel equation
    //
    // \param I is the incident view direction
    //
    // \param N is the normal at the intersection point
    //
    // \param ior is the material refractive index
    //
    // \param[out] kr is the amount of light reflected
    void fresnel(const Vector3f &I, const Vector3f &N, const float &ior, float &kr) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        if (cosi < 0) {  std::swap(etai, etat); }
        // Compute sini using Snell's law
        float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
        // Total internal reflection
        if (sint >= 1) {
            kr = 1;
        }
        else {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
            float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
            kr = (Rs * Rs + Rp * Rp) / 2;
        }
        // As a consequence of the conservation of energy, transmittance is given by:
        // kt = 1 - kr;
    }

    void fresnel2(const Vector3f &I, const Vector3f &N, const float &ior, float &kr) const
    {
            float cosi = clamp(-1, 1, dotProduct(I, N));
            float etai = 1, etat = ior;
            if (cosi < 0) {
                cosi = -cosi;
                std::swap(etai, etat);
            }
            float n = etat/etai;
            //float F0 = ((n-1)*(n-1)) / ((n+1)*(n+1));
            float F0 = 0.8;
            float F = F0 + (1-F0) * pow((1 - cosi),5);
            kr = std::min(F,1.0f);
    }

    Vector3f toWorld(const Vector3f &a, const Vector3f &N){
        Vector3f B, C;
        if (std::fabs(N.x) > std::fabs(N.y)){
            float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
            C = Vector3f(N.z * invLen, 0.0f, -N.x *invLen);
        }
        else {
            float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
            C = Vector3f(0.0f, N.z * invLen, -N.y *invLen);
        }
        B = crossProduct(C, N);
        return a.x * B + a.y * C + a.z * N;
    }

public:
    MaterialType m_type;
    //Vector3f m_color;
    Vector3f m_emission;
    float ior = 100;
    Vector3f Kd, Ks;
    float roughness;
    //Texture tex;

    inline Material(MaterialType t=DIFFUSE, Vector3f e=Vector3f(0,0,0));
    inline MaterialType getType();
    //inline Vector3f getColor();
    inline Vector3f getColorAt(double u, double v);
    inline Vector3f getEmission();
    inline bool hasEmission();

    // sample a ray by Material properties
    inline Vector3f sample(const Vector3f &wi, const Vector3f &N);
    // given a ray, calculate the PdF of this ray
    inline float pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
    // given a ray, calculate the contribution of this ray
    inline Vector3f eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);

};

Material::Material(MaterialType t, Vector3f e){
    m_type = t;
    //m_color = c;
    m_emission = e;
}

MaterialType Material::getType(){return m_type;}
//Vector3f Material::getColor(){return m_color;}
Vector3f Material::getEmission() {return m_emission;}
bool Material::hasEmission() {
    if (m_emission.norm() > EPSILON) return true;
    else return false;
}

Vector3f Material::getColorAt(double u, double v) {
    return Vector3f();
}


Vector3f Material::sample(const Vector3f &wi, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // uniform sample on the hemisphere
            float x_1 = get_random_float(), x_2 = get_random_float();
            float z = std::fabs(1.0f - 2.0f * x_1);
            float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
            Vector3f localRay(r*std::cos(phi), r*std::sin(phi), z);
            return toWorld(localRay, N);
            
            break;
        }
        case REFLECT:
        {
            return reflect(wi,N);
            break;
        }
        case Microfacet:
        {
            /*
            // uniform sample on the hemisphere
            float x_1 = get_random_float(), x_2 = get_random_float();
            float z = std::fabs(1.0f - 2.0f * x_1);
            float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
            Vector3f localRay(r*std::cos(phi), r*std::sin(phi), z);
            return toWorld(localRay, N);

            break;
            */

            ///*
            // sample the half vector
            float x_1 = get_random_float(), x_2 = get_random_float();
            float phi = 2 * M_PI * x_1;
            float tempPhi = 1 / (roughness * roughness);
            float theta = acosf(sqrtf( (tempPhi*(1-x_2)) / ((1-tempPhi)*x_2 + tempPhi) ));

            Vector3f tempH(sinf(theta)*cosf(phi),sinf(theta)*sinf(phi),cosf(theta));
            auto half = toWorld(tempH, N);
            float IdotH = dotProduct(half,-wi);
            auto result = 2 * IdotH * half + wi;
            return result;

            break;
            //*/
        }
    }
}

float Material::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // uniform sample probability 1 / (2 * PI)
            if (dotProduct(wo, N) > 0.0f)
                return 0.5f / M_PI;
            else
                return 0.0f;
            break;
        }
        case REFLECT:
        {
            // uniform sample probability 1 / (2 * PI)
            Vector3f r = reflect(wi,N);
            if ((reflect(wi,N) - wo).norm() < 0.01f)
                return 1.0f;
            else
                return 0.0f;
            break;
        }
        case Microfacet:
        {
            /*
            // uniform sample probability 1 / (2 * PI)
            if (dotProduct(wo, N) > 0.0f)
                return 0.5f / M_PI;
            else
                return 0.0f;
            */

            // uniform sample probability 1 / (2 * PI)
            if (dotProduct(wo, N) > 0.0f){
                Vector3f wii = -wi, woo = wo;
                auto h = (wii.normalized() + woo.normalized()).normalized();
                float result = GGX_D(h, N, roughness) * dotProduct(h, N) / (4 * dotProduct(woo,h));
                return result;
            } else {
                return 0.0f;
            }
            break;
        }
    }
}

Vector3f Material::eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // calculate the contribution of diffuse   model
            float cosalpha = dotProduct(N, wo);
            if (cosalpha > 0.01f) {
                Vector3f diffuse = Kd / M_PI;
                return diffuse;
            }
            else
                return Vector3f(0.0f);
            break;
        }
        case REFLECT:
        {
            if ((reflect(wi,N) - wo).norm() < EPSILON)
                return Kd * Vector3f(1.0f) / dotProduct(N,wo);
            else
                return Vector3f(0.0f);
            break;
        }
        case Microfacet:
        {
            // calculate the contribution of diffuse   model
            float cosalpha = dotProduct(N, wo);
            if (cosalpha > EPSILON) {

                Vector3f wii = -wi, woo = wo;
                auto h = (wii.normalized() + woo.normalized()).normalized();
                float kr;
                Vector3f result;
                fresnel2(wii,h,ior,kr);
                //float kr2;
                //fresnel2(wii,h,ior,kr2);
                //float roughness = 0.1;
                auto G = Smith_G(woo,wii,h,N,roughness);
                //auto G = dotProduct(N, wii) * dotProduct(N, woo);
                auto D = GGX_D(h,N,roughness);
                //auto D = 10 * pow(dotProduct(N, h),10)/M_PI;
                result = Kd * kr * G * D / (4 * dotProduct(N, wii) * dotProduct(N, woo));
                return result;
            }
            else
                return Vector3f(0.0f);
            break;
        }
    }
}



#endif //RAYTRACING_MATERIAL_H
