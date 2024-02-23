#include "server/Server.hpp"

#include "Photon.hpp"

#include <chrono>
#include "VertexTransformer.hpp"
#include "intersections/intersections.hpp"

#include "glm/gtc/matrix_transform.hpp"

#include <string>

#define THRESHOLD 0.0005f

namespace PhotonMap {
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

HitRecord PathTracerRenderer::defaultLightSample() const {
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

HitRecord PathTracerRenderer::xMesh(const Ray& ray, const Mesh& p, float tMin, float tMax) {
    HitRecord closestHitObj = nullopt;
    if (!p.bvh)
        return nullopt;
    closestHitObj = theClosestHitObject(p.bvh, ray);
    if (closestHitObj->t > tMax || closestHitObj->t < tMin)
        return nullopt;
    return closestHitObj;
}

float calDis(Photon a, Vec3 b);
float vecGetAxis(Vec3 a, int axis);

RGB PathTracerRenderer::gamma(const RGB& rgb) {
    return glm::sqrt(rgb);
}

float random_float() {
    srand(time(NULL));
    return rand() % 100 / (float)100;
}

void PathTracerRenderer::generatePhoton(AreaLight a, Vec3& ori, float& PowerScale, Vec3& dir)
{
    Vec3 normal = glm::cross(a.u, a.v);
    uniform_real_distribution<float> uni;
    ori = Vec3(a.position[0] + random_float() * (a.u[0]), a.position[1], a.position[2] + random_float() * (a.v[2]));
    float mode = 0;
    do
    {
        dir = Vec3(2 * uni(eng), 2 * uni(eng), 2 * uni(eng)) - Vec3(1, 1, 1);
        mode = glm::dot(dir, dir);
    }
    while (mode >= 1.0 || mode == 0.0 || glm::dot(dir, normal) < 0);
    PowerScale = glm::dot(dir, normal);
}

void PathTracerRenderer::generatePhotonMap()
{
    Vec3 ori;
    Vec3 dir;
    Vec3 pow(1, 1, 1);
    float powerscale;
    while (phtmap->p_num < phtmap->p_max)
    {
        float PowScale;
        for (auto a : scene.areaLightBuffer)
        {
            generatePhoton(a, ori,PowScale, dir);
            //cout << dir<<endl;
            Photon tp(ori, dir, pow);
            tracePhoton(tp, 0);
        }
    }
    phtmap->balance();
}

void photonMap::add_p(Photon* p) {
    //cout << max << endl;
    if (p_num < p_max) {
        p_list[p_num].position = p->position;
        p_list[p_num].direction = p->direction;
        p_list[p_num].power = p->power;
        p_num++;
        box_min = Vec3(min(box_min.x, p->position.x), min(box_min .y, p->position.y), min(box_min.z, p->position.z));
        box_max = Vec3(max(box_max.x, p->position.x), max(box_max.y, p->position.y), max(box_max.z, p->position.z));
    }
    else
        cout << "Photon number overflow, not added.\n";
}

int photonMap::calMedian(int start, int end)
{
    int num = end - start + 1;
    return start + num / 2;


    int as = 1, b = 2;
    while (as < num)
    {
        as += b;
        b *= 2;
    }
    if (as == num) {
        return start + num / 2;
    }
    b /= 2;
    if (as - b / 2 < num) {
        return start + as / 2;
    }
    else
        return start + as / 2 - (as - b / 2 - num);
}

void photonMap::MedianSplit(Photon* tempPhoton, int start, int end, int med, int axis)
{
    int l = start, r = end;
    while (l < r)
    {
        double key = tempPhoton[r-1].position[axis];
        int i = l - 1, j = r;
        for (;;)
        {
            while (tempPhoton[++i-1].position[axis] < key);
            while (tempPhoton[--j-1].position[axis] > key && j > l);
            if (i >= j)
                break;
            swap(tempPhoton[i-1], tempPhoton[j-1]);
        }
        swap(tempPhoton[i-1], tempPhoton[r-1]);
        if(i >= med) r = i - 1;
        if(i <= med) l = i + 1;
    }
}

float photonMap::getPhotonPosAxis(int index, int axis)
{
    return p_list[index - 1].position[axis];
}

void photonMap :: balance()
{
    Photon* tempPhoton = new Photon[p_num];
    for(int i = 0; i < p_num; i++)
        tempPhoton[i] = p_list[i];
    balanceSegment(tempPhoton, 1, 1, p_num);
    delete[] tempPhoton;
}

void photonMap::balanceSegment(Photon* tempPhoton, int index, int start, int end)
{
    int med = calMedian(start, end);
    int axis = 2;
    if (box_max.x - box_min.x > box_max.y - box_min.y && box_max.x - box_min.x > box_max.z - box_min.z)
        axis = 0;
    else if (box_max.y - box_min.y > box_max.z - box_min.z)
        axis = 1;
    if (index > p_max)
    {
        p_list[p_max-1] = tempPhoton[med - 1];
        return;
    }
    if (start == end)
    {
        p_list[index - 1] = tempPhoton[start - 1];
        return;
    }
    MedianSplit(tempPhoton, start, end, med, axis);
    p_list[index-1] = tempPhoton[med-1];
    p_list[index-1].axis = axis;
    if (start <= med)
    {
        double tmp = box_max[axis];
        box_max[axis] = p_list[index-1].position[axis];
        balanceSegment(tempPhoton, index * 2, start, med - 1);
        box_max[axis] = tmp;
    }
    if (end > med)
    {
        double tmp = box_min[axis];
        box_min[axis] = p_list[index-1].position[axis];
        balanceSegment(tempPhoton, index * 2 + 1, med + 1, end);
        box_min[axis] = tmp;
    }
}

void PathTracerRenderer::tracePhoton(Photon& p, int depth) {
    //cout << this->depth << endl;
    if (depth > this->depth)
        return;
    HitRecord hitObject = closestHitObject(p);
    Ray p_ray(p.position, p.direction);
    auto [t, emitted] = closestHitLight(p_ray);
    if (hitObject && hitObject->t < t)
    {
        float cosine = glm::dot(p.direction, hitObject->normal) / sqrt(glm::dot(p.direction, p.direction)) / sqrt(glm::dot(hitObject->normal, hitObject->normal));
        //p.power = p.power * abs(glm::dot(p.direction, hitObject->normal));
        p.power = p.power * abs(cosine);
        auto mtlHandle = hitObject->material;
        auto scattered = shaderPrograms[mtlHandle.index()]->shade(p_ray, hitObject->hitPoint, hitObject->normal);
        auto scatteredRay = scattered.ray;
        Photon tphoton(hitObject->hitPoint, scatteredRay.direction, p.power);
        //cout << hitObject->hitPoint << endl;
        int rus;
        srand(time(NULL));
        float t = rand() % 100 / (float)100;
        if (t < russian)
            rus = diffusion;
        else
            rus = absorb;

        auto diffuseColor = shaderPrograms[mtlHandle.index()]->material.getProperty<Property::Wrapper::RGBType>("diffuseColor");
        Vec3 tcolor = (*diffuseColor).value;


        switch (rus)
        {
            case diffusion:
            {
                Vec3 temp = tphoton.power;
                tphoton.power = tphoton.power * tcolor;
                tphoton.direction = scattered.ray.direction;
                phtmap->add_p(&tphoton);
                tphoton.power = temp * scattered.attenuation;
                tracePhoton(tphoton, depth + 1);
                break;
            }
            default:
            {
                tphoton.power = tphoton.power * tcolor;
                phtmap->add_p(&tphoton);
                return;
            }
        }

    }
}

void PathTracerRenderer::renderTask(RGBA* pixels, int width, int height, int off, int step)
{
    for (int i = off; i < height; i += step)
    {
        for (int j = 0; j < width; j++) {
            Vec3 color{ 0, 0, 0 };
            Vec3 irra{ 0,0,0 };
            auto r = defaultSamplerInstance<UniformInSquare>().sample2d();
            float rx = r.x; 
            float ry = r.y;
            float x = (float(j) + rx) / float(width);
            float y = (float(i) + ry) / float(height);
            auto ray = camera.shoot(x, y);
            irra += getIrradiance(ray);
            //color += trace(ray, 0);

            for (int k = 0; k < samples; k++)
            {
                auto r = defaultSamplerInstance<UniformInSquare>().sample2d();
                float rx = r.x;
                float ry = r.y;
                float x = (float(j) + rx) / float(width);
                float y = (float(i) + ry) / float(height);
                auto ray = camera.shoot(x, y);
            }
           //color /= samples;
            //irra /= samples;
            color += irra;
            //color = clamp(color);
            color = gamma(color);

            pixels[(height - i - 1) * width + j] = { color, 1 };
        }
    }
}

//void PathTracerRenderer::renderTask(RGBA* pixels, int width, int height, int off,int step) {
//    for (int i = off; i < height; i += step)
//    {
//        for (int j = 0; j < width; j++)
//        {
//            Vec3 color{0, 0, 0};
//            for (int k = 0; k < samples; k++)
//            {
//                auto r = defaultSamplerInstance<UniformInSquare>().sample2d();
//                float rx = r.x;
//                float ry = r.y;
//                float x = (float(j) + rx) / float(width);
//                float y = (float(i) + ry) / float(height);
//                auto ray = camera.shoot(x, y);
//                color += trace(ray, 0);
//            }
//            color /= samples;
//            color = gamma(color);
//            pixels[(height - i - 1) * width + j] = {color, 1};
//        }
//    }
//}

auto PathTracerRenderer::render() -> RenderResult {
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
    this->generatePhotonMap();

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

HitRecord PathTracerRenderer::closestHitObject(const Ray& r) {
    HitRecord closestHit = nullopt;
    closestHit = theClosestHitObject(scene.bvh, r);
    return closestHit;
}

HitRecord PathTracerRenderer::closestHitObject(const Photon& p)
{
    Ray r;
    r.setDirection(p.direction);
    r.setOrigin(p.position);
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
        if (xAtten(srObj, t3) && defaultSamplerInstance<UniformSampler>().sample1d() < 
            
            scene.renderOption.russian)
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

RGB PathTracerRenderer::getIrradiance(Ray r) {
    Vec3 ir{ 0,0,0 };
    Vec3 norm = r.direction;

    auto hitObject = closestHitObject(r);
    auto [t, emitted] = closestHitLight(r);

    if (hitObject && hitObject->t < t)
    { 
        nearestNeighbor np(hitObject->hitPoint);

        np.getNearest(phtmap);
        //np.getNearestWithKD(phtmap, 1);

        int p_num = np.found < np.sample ? np.found : np.sample;
        //cout << np.sample << endl;
        for (int i = 0; i < p_num; i++)
        {
            //cout << "#############################      " << i << endl;
            ir += np.photonDict[np.photonDis[i]].power;
        }
        ir = float(150) * ir / (4 * PI * np.photonDis[np.found - 1]);
        //std::cout << ir<<endl;
        return ir;
    }
    else if (t != FLOAT_INF)
        return emitted;
    else
    {
        //cout << "fuxk!" << endl;
        return Vec3{ 0, 0, 0 };
    }
}

void nearestNeighbor::getNearest(photonMap* phtmap) {
    for (int i = 0; i < phtmap->p_max; i++)
    {
        if (this->found >= this->maxPhoton)
            break;
        Vec3 ppos = phtmap->p_list[i].position - this->pos;
        float dis = glm::dot(ppos, ppos);
        if (dis < this->l2dis)
        {
            this->photonDict[dis] = phtmap->p_list[i];
            this->photonDis[found] = dis;
            this->found++;
        }
    }
    sort(this->photonDis, this->photonDis + this->found);
}

void nearestNeighbor::getNearestWithKD(photonMap* phtmap, int index)
{
    if (index > phtmap->p_num)
        return;
    Photon* photon = &phtmap->p_list[index - 1];
    if (index * 2 <= phtmap->p_num)
    {
        //cout << photon->axis << endl;
        float dist = pos[photon-> axis] - photon-> position[photon->axis];
        if (dist < 0)
        {
            getNearestWithKD(phtmap, index * 2);
            if (dist * dist < this->l2dis)
                getNearestWithKD(phtmap, index * 2 + 1);
        }
        else
        {
            getNearestWithKD(phtmap, index * 2 + 1);
            if (dist * dist < this->l2dis)
                getNearestWithKD(phtmap, index * 2);
        }
    }
    float dist2 = glm::dot(photon->position - this->pos, photon->position - this->pos);
    if (dist2 > l2dis)
        return;
    if (found < maxPhoton)
    {
        found++;
        photonDis[found] = dist2;
        photonDict[dist2] = *photon;
    }
    else
    {
        if (!gotHeap)
        {
            for (int i = found >> 1; i >= 1; i--)
            {
                int par = i;
                Photon* tmp_photon = &photonDict[photonDis[i - 1]];
                if ((par << 1) <= found)
                {
                    int j = par << 1;
                    if (j + 1 <= found) j++;
                    photonDict[photonDis[par - 1]] = photonDict[photonDis[j - 1]];
                    par = j;
                }
                photonDict[photonDis[par - 1]] = *tmp_photon;
            }
            gotHeap = true;
        }
        int par = 1;
        if ((par << 1) <= found)
        {
            int j = par << 1;
            if (j + 1 <= found) j++;
            photonDict[photonDis[par - 1]] = photonDict[photonDis[j - 1]];
            par = j;
        }
        photonDict[photonDis[par - 1]] = *photon;
        l2dis = dist2;
    }
}

}  // namespace PathTracer