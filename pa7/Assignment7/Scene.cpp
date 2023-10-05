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

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
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
    Intersection inter_to_obj = intersect(ray);
    Vector3f L_dir;
    Vector3f L_indir;

    //hit nothing, return directly
    if (!inter_to_obj.happened)
        return Vector3f(0,0,0);

    //hit the light source
    if (inter_to_obj.m->hasEmission())
        return inter_to_obj.m->getEmission();

    //hit an object
    Vector3f p = inter_to_obj.coords;
    Vector3f N = inter_to_obj.normal.normalized();
    Vector3f wo = ray.direction;

    //sample in light source
    float pdf_light;
    Intersection inter_to_light;
    sampleLight(inter_to_light,pdf_light);
    Vector3f x = inter_to_light.coords;
    Vector3f NN = inter_to_light.normal.normalized();
    Vector3f ws = (x-p).normalized();
    Vector3f emit = inter_to_light.emit;
    Ray obj_to_light(p,ws);
    float light_to_obj_distance = (x-p).norm();
    Intersection obj_to_light_inter = intersect(obj_to_light);
    //there are no objects between hitpoint and light
    if (obj_to_light_inter.distance - light_to_obj_distance > - 0.01) {

        //we take wo as wi, because we know wo and we want to calculate the direction of wi
        //in other word, we take output ray as input ray
        L_dir = emit * inter_to_obj.m->eval(wo, ws, N)
                * dotProduct(ws, N) //cos_theta
                * dotProduct(-ws, NN) //cos_theta_x
                / std::pow(light_to_obj_distance,2)
                / pdf_light;
    }

    float P_RR = get_random_float();
    if (P_RR < RussianRoulette) {
        //we take wo as wi, because we know wo and we want to calculate the direction of wi
        Vector3f wi = inter_to_obj.m->sample(wo,N).normalized();
        //shoot a ray from p , take wi as direction
        Ray r(p,wi);
        Intersection inter_to_q = intersect(r);
        //if r hit non-emitting obj at q
        if (inter_to_q.happened && !inter_to_q.m->hasEmission()) {
            L_indir = castRay(r,depth + 1)
                    * inter_to_obj.m->eval(wo,wi,N)
                    * dotProduct(wi,N)
                    / inter_to_obj.m->pdf(wo,wi,N)
                    / RussianRoulette;
        }
    }
    return L_dir + L_indir;
}