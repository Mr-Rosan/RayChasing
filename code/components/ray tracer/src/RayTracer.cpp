#include "server/Server.hpp"

#include "RayTracer.hpp"

#include <chrono>
#include "VertexTransformer.hpp"
#include "intersections/intersections.hpp"

#include "glm/gtc/matrix_transform.hpp"

#include <string>

#define THRESHOLD 0.0005f

namespace RayTracer {
bool PathTracerRenderer::xBounds(const Ray& ray, const Bounds& b)
{
    Vec3 dirVec = { 1.0f / ray.direction.x, 1.0f / ray.direction.y, 1.0f / ray.direction.z };
    Vec3 temp1 = (b.getMinPoint() - ray.origin) * dirVec;
    Vec3 temp2 = (b.getMaxPoint() - ray.origin) * dirVec;
    Vec3 Min,Max;

    if (ray.direction.x >= 0)
    {
        Min.x = temp1.x;
        Max.x = temp2.x;
    }
    else
    {
        Min.x = temp2.x;
        Max.x = temp1.x;
    }

    if (ray.direction.y >= 0)
    {
        Min.y = temp1.y;
        Max.y = temp2.y;
    }
    else
    {
        Min.y = temp2.y;
        Max.y = temp1.y;
    }

    if (ray.direction.z >= 0)
    {
        Min.z = temp1.z;
        Max.z = temp2.z;
    }
    else
    {
        Min.z = temp2.z;
        Max.z = temp1.z;
    }

    float start = max({ Min.x, Min.y, Min.z });
    float end = min({ Max.x, Max.y, Max.z });
    return start <= end && end > 0;
}

HitRecord PathTracerRenderer::BVHTree(const Ray& ray, BVHBuildNode* node)
{
    HitRecord closestHit = nullopt;

    if (!xBounds(ray, node->bound))
        return closestHit;

    if (!node->left && !node->right)
    {
        switch (node->object->type)
        {
            case Entity::EntityType::SPHERE:
            {
                closestHit = Intersection::xSphere(ray, *(Sphere*)(node->object), 0.000001, FLOAT_INF);
                break;
            }
            case Entity::EntityType::PLANE:
            {
                closestHit = Intersection::xPlane(ray, *(Plane*)(node->object), 0.000001, FLOAT_INF);
                break;
            }
            case Entity::EntityType::TRIANGLE:
            {
                closestHit = Intersection::xTriangle(ray, *(Triangle*)(node->object), 0.000001, FLOAT_INF);
                break;
            }
            case Entity::EntityType::MESH:
            {
                closestHit = xMesh(ray, *(Mesh*)(node->object), 0.000001, FLOAT_INF);
                break;
            }
        }
        return closestHit;
    }
    auto left = BVHTree(ray, node->left);
    auto right = BVHTree(ray, node->right);
    if (!left)
        return right;
    else if (!right)
        return left;
    else
    {
        if (left->t < right->t)
            return left;
        else
            return right;
    }
}

HitRecord PathTracerRenderer::theClosestHitObject(BVH* bvh, const Ray& ray) {
    HitRecord closestHitObj = nullopt;
    if (!bvh->root)
        return closestHitObj;
    closestHitObj = BVHTree(ray, bvh->root);
    return closestHitObj;
}

HitRecord PathTracerRenderer::defaultLightSample() const
{
    float lightSampleSum = 0;
    for (auto& obj : scene.areaLightBuffer)
        lightSampleSum += obj.getLightArea();

    auto samples = defaultSamplerInstance<UniformSampler>();
    float p = samples.sample1d() * lightSampleSum;

    float lightArea = 0;
    for (auto& obj : scene.areaLightBuffer)
    {
        auto lightArea = obj.getLightArea();
        lightArea += lightArea;
        if (p <= lightSampleSum)
        {
            Vec3 pos = obj.position + samples.sample1d() * obj.u + samples.sample1d() * obj.v;
            return getHitRecord(0, pos, obj.getNormal(), 1 / lightArea, obj.radiance, {});
        }
    }
}

HitRecord PathTracerRenderer::xMesh(const Ray& ray, const Mesh& p, float tMin, float tMax)
{
    HitRecord closestHitObj = nullopt;
    if (!p.bvh)
        return nullopt;
    closestHitObj = theClosestHitObject(p.bvh, ray);
    if (closestHitObj->t > tMax || closestHitObj->t < tMin)
        return nullopt;
    return closestHitObj;
}

RGB PathTracerRenderer::gamma(const RGB& rgb) {
    return glm::sqrt(rgb);
}

void PathTracerRenderer::renderTask(RGBA* pixels, int width, int height, int off,int step)
{
    for (int i = off; i < height; i += step)
    {
        for (int j = 0; j < width; j++)
        {
            Vec3 color{0, 0, 0};
            for (int k = 0; k < samples; k++)
            {
                auto r = defaultSamplerInstance<UniformInSquare>().sample2d();
                float rx = r.x;
                float ry = r.y;
                float x = (float(j) + rx) / float(width);
                float y = (float(i) + ry) / float(height);
                auto ray = camera.shoot(x, y);
                color += trace(ray, 0);
            }
            color /= samples;
            color = gamma(color);
            pixels[(height - i - 1) * width + j] = {color, 1};
        }
    }
}

auto PathTracerRenderer::render() -> RenderResult
{
    // shaders
    shaderPrograms.clear();
    ShaderCreator shaderCreator{};
    for (auto& m : scene.materials) {
        shaderPrograms.push_back(shaderCreator.create(m, scene.textures));
    }

    RGBA* pixels = new RGBA[width * height]{};

    // 局部坐标转换成世界坐标
    VertexTransformer vertexTransformer{};
    vertexTransformer.exec(spScene);
    scene.buildBVHTree();

    const auto taskNums = 8;
    thread t[taskNums];
    for (int i = 0; i < taskNums; i++) {
        t[i] = thread(&PathTracerRenderer::renderTask, this, pixels, width, height, i, taskNums);
    }
    for (int i = 0; i < taskNums; i++) {
        t[i].join();
    }
    getServer().logger.log("Done...");
    return {pixels, width, height};
}

void PathTracerRenderer::release(const RenderResult& r) {
    auto [p, w, h] = r;
    delete[] p;
}

HitRecord PathTracerRenderer::closestHitObjectWithoutBVH(const Ray& r)
{
    HitRecord closestHit = nullopt;
        float closest = FLOAT_INF;
        for (auto& s : scene.sphereBuffer)
        {
            auto hitRecord = Intersection::xSphere(r, s, 0.000001, closest);
            if (hitRecord && hitRecord->t < closest)
            {
                closest = hitRecord->t;
                closestHit = hitRecord;
            }
        }
        for (auto& t : scene.triangleBuffer)
        {
            auto hitRecord = Intersection::xTriangle(r, t, 0.000001, closest);
            if (hitRecord && hitRecord->t < closest)
            {
                closest = hitRecord->t;
                closestHit = hitRecord;
            }
        }
        for (auto& p : scene.planeBuffer)
        {
            auto hitRecord = Intersection::xPlane(r, p, 0.000001, closest);
            if (hitRecord && hitRecord->t < closest)
            {
                closest = hitRecord->t;
                closestHit = hitRecord;
            }
        }
        for (auto& m : scene.meshBuffer)
        {
            auto hitRecord = xMesh(r, m, 0.000001, closest);
            if (hitRecord && hitRecord->t < closest)
            {
                closest = hitRecord->t;
                closestHit = hitRecord;
            }
        }
    return closestHit;
}

HitRecord PathTracerRenderer::closestHitObject(const Ray& r) {
    HitRecord closestHit = nullopt;
    closestHit = theClosestHitObject(scene.bvh, r);
    return closestHit;
}

tuple<float, Vec3> PathTracerRenderer::closestHitLight(const Ray& r) {
    Vec3 v = {};
    HitRecord closest = getHitRecord(FLOAT_INF, {}, {}, {});
    for (auto& a : scene.areaLightBuffer) {
        auto hitRecord = Intersection::xAreaLight(r, a, 0.000001, closest->t);
        if (hitRecord && closest->t > hitRecord->t) {
            closest = hitRecord;
            v = a.radiance;
        }
    }
    return {closest->t, v};
}

bool xEmit(Vec3 p1, Vec3 p2, HitRecord hitp, float t2)
{
    return glm::length(p1 - p2) < THRESHOLD && (!hitp.has_value() || hitp->t > t2 || fabs(hitp->t - t2) < THRESHOLD);
}

bool xAtten(HitRecord hitp, float t3)
{
    return hitp&& hitp->t < t3 && !fabs(hitp->t - t3) < THRESHOLD;
}

RGB PathTracerRenderer::trace(const Ray& r, int currDepth) {
    if (currDepth == depth)
        return scene.ambient.constant;

    auto hitObject = closestHitObject(r);
    //auto hitObject = closestHitObjectWithoutBVH(r);
    auto [t, emitted] = closestHitLight(r);
    // hit object
    if (hitObject && hitObject->t < t)
    {
        auto lightSample = defaultLightSample().value();
        Vec3 lightPoint = lightSample.hitPoint;

        auto hitPoint = hitObject.value().hitPoint;
        Vec3 lightRay = glm::normalize(lightPoint - hitPoint);
        auto [t2, emitted2] = closestHitLight(Ray(hitPoint, lightRay));
        auto testPoint = hitPoint + t2 * lightRay;

        Vec3 emit = {0,0,0}, atten = { 0,0,0 };
        auto mtlHandle = hitObject->material;
        auto scattered = shaderPrograms[mtlHandle.index()]->shade(r, hitObject->hitPoint, hitObject->normal);
        auto scatteredRay = scattered.ray;
        if(xEmit(testPoint, lightPoint, closestHitObject(Ray(hitPoint, lightRay)), t2))
        {
            Vec3 couter = -r.direction;
            Vec3 objN = hitObject.value().normal;
            float dotN = glm::dot(-lightRay, lightSample.normal);
            emit = emitted2 * shaderPrograms[mtlHandle.index()]->eval(couter, lightRay, objN) * glm::dot(lightRay, objN) * fabs(dotN) / (float)pow(glm::length(lightPoint - hitPoint), 2) / lightSample.pdf;
        }

        auto srObj = closestHitObject(scatteredRay);
        auto [t3, emitted3] = closestHitLight(scatteredRay);
        if (xAtten(srObj, t3) && defaultSamplerInstance<UniformSampler>().sample1d() < scene.renderOption.russian)
        {
            auto attenuation = scattered.attenuation;
            auto emitted = scattered.emitted;
            float n_dot_in = glm::dot(hitObject->normal, scatteredRay.direction);
            float pdf = scattered.pdf;
            auto next = trace(Ray(scatteredRay), currDepth + 1);
            atten = clamp(next) * attenuation * n_dot_in / pdf / scene.renderOption.russian;
        }
        return atten + emit;
    }
    else if (t != FLOAT_INF)
        return emitted;
    else
        return Vec3{0};
}
}  // namespace PathTracer