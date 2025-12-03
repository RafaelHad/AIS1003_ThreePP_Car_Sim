// main.cpp
#include <iostream>
#include <vector>
#include <memory>
#include <cmath>
#include <map>
#include <ctime>
#include <cstdlib>
#include <chrono>
#include <unordered_map>

#include <threepp/threepp.hpp>

#include <threepp/loaders/OBJLoader.hpp>
#include <threepp/loaders/MTLLoader.hpp>
#include <threepp/objects/InstancedMesh.hpp>
#include <threepp/scenes/Scene.hpp>
#include <threepp/math/MathUtils.hpp>
#include <threepp/lights/AmbientLight.hpp>
#include <threepp/lights/DirectionalLight.hpp>
#include <threepp/materials/MeshStandardMaterial.hpp>
#include <threepp/objects/Mesh.hpp>
#include <threepp/geometries/PlaneGeometry.hpp>
#include <threepp/objects/Group.hpp>
#include <threepp/geometries/BoxGeometry.hpp>
#include <threepp/materials/MeshBasicMaterial.hpp>
#include <threepp/geometries/SphereGeometry.hpp>
#include <threepp/geometries/CylinderGeometry.hpp>

using namespace threepp;

// map size variable
const float MAP_SIZE = 600.0f;

// spatial grid cell size for neighborhood queries
const float GRID_CELL_SIZE = 20.0f;

// SIMULATION SETTINGS
struct SimSettings {
    static const int GRASS_COUNT = 3000;
    static const int ROCK_COUNT = 600;
    static const int TREE_COUNT = 400;
    static const int CLOUD_COUNT = 100;
};

// obstacle
enum class ObstacleType { TREE_CYLINDER, ROCK_SPHERE };

struct Obstacle {
    ObstacleType type;
    Vector3 position; // world-space center (for cylinder center at ground + height/2)
    float radius;     // cylinder radius or sphere radius
    float halfHeight; // for cylinder: half height (else 0)
    // optional: instance index and pointer (for debug/visualization) - not used for collision math
    int instanceIndex;
};

// spatial grid key util
static inline int makeCellKey(int ix, int iz) {
    return (ix & 0xFFFF) << 16 | (iz & 0xFFFF);
}

// aux. functions
void enableShadowsForGroup(Group* g, bool cast = true, bool receive = false) {
    for (const auto &child : g->children) {
        if (child->is<Mesh>()) {
            auto m = child->as<Mesh>();
            m->castShadow = cast;
            m->receiveShadow = receive;
        } else if (child->is<Group>()) {
            enableShadowsForGroup(child->as<Group>(), cast, receive);
        }
    }
}

// we build obstacle lists when we make instanced meshes
// these vectors createXXX-functions and used by CollisionSystem
std::vector<Obstacle> g_obstacles; // alle obstacles
std::unordered_map<int, std::vector<int>> g_spatialGrid; // cellKey -> vector of indices i g_obstacles

static inline void addObstacleToGrid(int obsIndex, const Vector3 &pos) {
    int ix = static_cast<int>(std::floor((pos.x + MAP_SIZE*0.5f) / GRID_CELL_SIZE));
    int iz = static_cast<int>(std::floor((pos.z + MAP_SIZE*0.5f) / GRID_CELL_SIZE));
    int key = makeCellKey(ix, iz);
    g_spatialGrid[key].push_back(obsIndex);
}

static inline std::vector<int> queryNearbyObstacleIndices(const Vector3 &pos, int searchRadiusCells = 1) {
    std::vector<int> result;
    int cx = static_cast<int>(std::floor((pos.x + MAP_SIZE*0.5f) / GRID_CELL_SIZE));
    int cz = static_cast<int>(std::floor((pos.z + MAP_SIZE*0.5f) / GRID_CELL_SIZE));
    for (int dx = -searchRadiusCells; dx <= searchRadiusCells; ++dx) {
        for (int dz = -searchRadiusCells; dz <= searchRadiusCells; ++dz) {
            int key = makeCellKey(cx + dx, cz + dz);
            auto it = g_spatialGrid.find(key);
            if (it != g_spatialGrid.end()) {
                for (int idx : it->second) result.push_back(idx);
            }
        }
    }
    return result;
}

// grass generation, count is variable
// grass has no collision
void createGrassFieldInstanced(threepp::Scene& scene, int grassCount = 3000) {
    threepp::MTLLoader mtlLoader;
    auto materialCreator = mtlLoader.load(R"(..\Assets\Grass_2_D_Color1.mtl)");
    materialCreator->preload();

    threepp::OBJLoader loader;
    loader.setMaterials(materialCreator);
    auto grassModel = loader.load(R"(..\Assets\Grass_2_D_Color1.obj)");
    if (!grassModel) {
        std::cerr << "Could not load grass model!" << std::endl;
        return;
    }

    threepp::Mesh* grassMesh = nullptr;
    for (const auto& child : grassModel->children) {
        if (child->is<threepp::Mesh>()) {
            grassMesh = child->as<threepp::Mesh>();
            break;
        }
    }

    if (!grassMesh) {
        std::cerr << "No mesh found in grass model!" << std::endl;
        return;
    }

    auto grassGeometry = grassMesh->geometry();
    auto grassMaterial = grassMesh->material();
    auto instancedGrass = threepp::InstancedMesh::create(grassGeometry, grassMaterial, grassCount);
    instancedGrass->castShadow = false;
    instancedGrass->receiveShadow = true;
    instancedGrass->frustumCulled = false;

    for (int i = 0; i < grassCount; i++) {
        float x = (static_cast<float>(std::rand()) / RAND_MAX - 0.5f) * MAP_SIZE;
        float z = (static_cast<float>(std::rand()) / RAND_MAX - 0.5f) * MAP_SIZE;
        float y = 0.0f;
        float rotation = (static_cast<float>(std::rand()) / RAND_MAX) * 2.0f * threepp::math::PI;
        float scale = 0.8f + (static_cast<float>(std::rand()) / RAND_MAX) * 0.4f;

        threepp::Matrix4 matrix;
        threepp::Vector3 position(x, y, z);
        threepp::Quaternion quaternion;
        quaternion.setFromEuler(threepp::Euler(0, rotation, 0));
        threepp::Vector3 scaleVec(scale, scale, scale);

        matrix.compose(position, quaternion, scaleVec);
        instancedGrass->setMatrixAt(i, matrix);
    }

    instancedGrass->instanceMatrix()->needsUpdate();
    scene.add(instancedGrass);
}
// rock generation, count is variable
// has collision
void createRockFieldInstanced(threepp::Scene& scene, int rockCount = 600) {
    threepp::MTLLoader mtlLoader;
    auto materialCreator = mtlLoader.load(R"(..\Assets\Rock_3_R_Color1.mtl)");
    materialCreator->preload();

    threepp::OBJLoader loader;
    loader.setMaterials(materialCreator);
    auto rockModel = loader.load(R"(..\Assets\Rock_3_R_Color1.obj)");
    if (!rockModel) {
        std::cerr << "Could not load rock model!" << std::endl;
        return;
    }

    threepp::Mesh* rockMesh = nullptr;
    for (const auto& child : rockModel->children) {
        if (child->is<threepp::Mesh>()) {
            rockMesh = child->as<threepp::Mesh>();
            break;
        }
    }

    if (!rockMesh) {
        std::cerr << "No Mesh found in rock model!" << std::endl;
        return;
    }

    auto rockGeometry = rockMesh->geometry();
    auto rockMaterial = rockMesh->material();
    auto instancedRocks = threepp::InstancedMesh::create(rockGeometry, rockMaterial, rockCount);
    instancedRocks->castShadow = true;
    instancedRocks->receiveShadow = true;
    instancedRocks->frustumCulled = false;

    // approximation: use base radius
    const float baseRockRadius = 2.0f;

    for (int i = 0; i < rockCount; i++) {
        float x = (static_cast<float>(std::rand()) / RAND_MAX - 0.5f) * MAP_SIZE;
        float z = (static_cast<float>(std::rand()) / RAND_MAX - 0.5f) * MAP_SIZE;
        float y = -0.2f + (static_cast<float>(std::rand()) / RAND_MAX) * 0.4f;
        float rotationX = (static_cast<float>(std::rand()) / RAND_MAX) * 2.0f * threepp::math::PI;
        float rotationY = (static_cast<float>(std::rand()) / RAND_MAX) * 2.0f * threepp::math::PI;
        float rotationZ = (static_cast<float>(std::rand()) / RAND_MAX) * 2.0f * threepp::math::PI;

        float scale = 0.2f + (static_cast<float>(std::rand()) / RAND_MAX) * 1.0f;
        threepp::Vector3 position(x, y, z);
        threepp::Quaternion quaternion;
        quaternion.setFromEuler(threepp::Euler(rotationX, rotationY, rotationZ));
        threepp::Vector3 scaleVec(scale, scale, scale);

        threepp::Matrix4 matrix;
        matrix.compose(position, quaternion, scaleVec);

        instancedRocks->setMatrixAt(i, matrix);

        //add obstacle entry (sphere)
        Obstacle obs;
        obs.type = ObstacleType::ROCK_SPHERE;
        obs.position = position;
        obs.radius = baseRockRadius * scale; // scale * base radius
        obs.halfHeight = 0.0f;
        obs.instanceIndex = i;
        int idx = static_cast<int>(g_obstacles.size());
        g_obstacles.push_back(obs);
        addObstacleToGrid(idx, obs.position);
    }

    instancedRocks->instanceMatrix()->needsUpdate();
    scene.add(instancedRocks);
}
// tree generation, count is variable
void createTreeFieldInstanced(threepp::Scene& scene, int treeCount = 400) {
    threepp::MTLLoader mtlLoader;
    auto materialCreator = mtlLoader.load(R"(..\Assets\Tree_4_A_Color1.mtl)");
    materialCreator->preload();

    threepp::OBJLoader loader;
    loader.setMaterials(materialCreator);
    auto treeModel = loader.load(R"(..\Assets\Tree_4_A_Color1.obj)");
    if (!treeModel) {
        std::cerr << "Could not load tree model!" << std::endl;
        return;
    }

    threepp::Mesh* treeMesh = nullptr;
    for (const auto& child : treeModel->children) {
        if (child->is<threepp::Mesh>()) {
            treeMesh = child->as<threepp::Mesh>();
            break;
        }
    }

    if (!treeMesh) {
        std::cerr << "No mesh found in tree model!" << std::endl;
        return;
    }

    auto treeGeometry = treeMesh->geometry();
    auto treeMaterial = treeMesh->material();

    if (treeMaterial->is<threepp::MeshStandardMaterial>()) {
        auto standardMat = treeMaterial->as<threepp::MeshStandardMaterial>();
        standardMat->roughness = 1.0f;
        standardMat->metalness = 0.0f;
    }

    auto instancedTrees = threepp::InstancedMesh::create(treeGeometry, treeMaterial, treeCount);
    instancedTrees->castShadow = true;
    instancedTrees->receiveShadow = true;
    instancedTrees->frustumCulled = false;

    // tree base cylinder-dimension
    const float modelTreeRadius = 0.6f;
    const float modelTreeHeight = 6.0f;

    for (int i = 0; i < treeCount; i++) {
        float x = (static_cast<float>(std::rand()) / RAND_MAX - 0.5f) * MAP_SIZE;
        float z = (static_cast<float>(std::rand()) / RAND_MAX - 0.5f) * MAP_SIZE;
        float y = 0.0f;
        float rotation = (static_cast<float>(std::rand()) / RAND_MAX) * 2.0f * threepp::math::PI;
        float scale = 2.0f + (static_cast<float>(std::rand()) / RAND_MAX) * 0.8f;

        threepp::Vector3 position(x, y, z);
        threepp::Quaternion quaternion;
        quaternion.setFromEuler(threepp::Euler(0, rotation, 0));
        threepp::Vector3 scaleVec(scale, scale, scale);

        threepp::Matrix4 matrix;
        matrix.compose(position, quaternion, scaleVec);

        instancedTrees->setMatrixAt(i, matrix);

        // add obstacle entry for tree trunk (as cylinder)
        Obstacle obs;
        obs.type = ObstacleType::TREE_CYLINDER;
        //set center at halfway up (y + height/2)
        float height = modelTreeHeight * scale;
        obs.halfHeight = height * 0.5f;
        obs.position = position; // CENTER AT GROUND (y=0)

        obs.radius = modelTreeRadius * scale;
        obs.instanceIndex = i;
        int idx = static_cast<int>(g_obstacles.size());
        g_obstacles.push_back(obs);
        addObstacleToGrid(idx, obs.position);
    }

    instancedTrees->instanceMatrix()->needsUpdate();
    scene.add(instancedTrees);
}
// clouds generation, count is variable
// clouds re-use rock model
void createCloudFieldInstanced(threepp::Scene& scene, int cloudCount = 100) {
    threepp::OBJLoader loader;
    auto rockModel = loader.load(R"(..\Assets\Rock_3_R_Color1.obj)");
    if (!rockModel) {
        std::cerr << "Could not load rock model (for clouds)!" << std::endl;
        return;
    }

    threepp::Mesh* rockMesh = nullptr;
    for (const auto& child : rockModel->children) {
        if (child->is<threepp::Mesh>()) {
            rockMesh = child->as<threepp::Mesh>();
            break;
        }
    }

    if (!rockMesh) {
        std::cerr << "No mesh found in rock model (for clouds)!" << std::endl;
        return;
    }
    // cloud properties
    auto cloudGeometry = rockMesh->geometry();
    auto cloudMaterial = threepp::MeshStandardMaterial::create();
    cloudMaterial->color = threepp::Color(0xffffff);
    cloudMaterial->metalness = 0.0f;
    cloudMaterial->roughness = 1.0f;
    cloudMaterial->transparent = true;
    cloudMaterial->opacity = 0.85f;

    // cloud shadow behaviour
    auto instancedClouds = threepp::InstancedMesh::create(cloudGeometry, cloudMaterial, cloudCount);
    instancedClouds->castShadow = true;
    instancedClouds->receiveShadow = true;
    instancedClouds->frustumCulled = false;

    for (int i = 0; i < cloudCount; i++) {
        float x = (static_cast<float>(std::rand()) / RAND_MAX - 0.5f) * MAP_SIZE * 1.2f;
        float z = (static_cast<float>(std::rand()) / RAND_MAX - 0.5f) * MAP_SIZE * 1.2f;
        float y = 70.0f + (static_cast<float>(std::rand()) / RAND_MAX) * 25.0f;
        float rotationY = (static_cast<float>(std::rand()) / RAND_MAX) * 2.0f * threepp::math::PI;
        float scale = 8.0f + (static_cast<float>(std::rand()) / RAND_MAX) * 6.0f;

        threepp::Vector3 position(x, y, z);
        threepp::Quaternion quaternion;
        quaternion.setFromEuler(threepp::Euler(0, rotationY, 0));
        threepp::Vector3 scaleVec(scale, scale * 0.7f, scale);
        threepp::Matrix4 matrix;
        matrix.compose(position, quaternion, scaleVec);

        instancedClouds->setMatrixAt(i, matrix);
    }

    instancedClouds->instanceMatrix()->needsUpdate();
    scene.add(instancedClouds);
}
// terrain border generation, auto adjusts based on map size
void createTerrainBorder(threepp::Scene& scene) {
    const float MODEL_WIDTH = 25.0f;
    const int SEGMENTS_PER_SIDE = 6;
    const float DESIRED_SEGMENT_WIDTH = MAP_SIZE / SEGMENTS_PER_SIDE;
    float baseScale = DESIRED_SEGMENT_WIDTH / MODEL_WIDTH;
    float borderDistance = 65.0f;
    float groundEdge = MAP_SIZE / 2.0f;
    int totalSegments = SEGMENTS_PER_SIDE * 4;

    threepp::OBJLoader loader;
    auto model = loader.load(R"(..\Assets\Low Poly Terrain 2.obj)");
    if (!model) {
        std::cerr << "Could not load terrain model for border!" << std::endl;
        return;
    }

    threepp::Mesh* terrainMesh = nullptr;
    for (const auto& child : model->children) {
        if (child->is<threepp::Mesh>()) {
            terrainMesh = child->as<threepp::Mesh>();
            break;
        }
    }
    if (!terrainMesh) return;

    auto terrainGeometry = terrainMesh->geometry();
    auto terrainMaterial = threepp::MeshStandardMaterial::create();
    terrainMaterial->color = threepp::Color(0x555555);
    terrainMaterial->metalness = 0.0f;
    terrainMaterial->roughness = 0.8f;

    auto instancedBorder = threepp::InstancedMesh::create(terrainGeometry, terrainMaterial, totalSegments);
    instancedBorder->castShadow = true;
    instancedBorder->receiveShadow = true;
    instancedBorder->frustumCulled = false;

    float rotationY = 0.0f;
    int index = 0;
    float segmentSpacing = DESIRED_SEGMENT_WIDTH;

    for (int side = 0; side < 4; ++side) {
        float currentX = 0.0f;
        float currentZ = 0.0f;

        switch (side) {
            case 0:
                rotationY = 0.0f;
                currentZ = groundEdge + borderDistance;
                currentX = -groundEdge + segmentSpacing / 2.0f;
                break;
            case 1:
                rotationY = threepp::math::PI / 2.0f;
                currentX = groundEdge + borderDistance;
                currentZ = -groundEdge + segmentSpacing / 2.0f;
                break;
            case 2:
                rotationY = threepp::math::PI;
                currentZ = -groundEdge - borderDistance;
                currentX = groundEdge - segmentSpacing / 2.0f;
                break;
            case 3:
                rotationY = 3.0f * threepp::math::PI / 2.0f;
                currentX = -groundEdge - borderDistance;
                currentZ = groundEdge - segmentSpacing / 2.0f;
                break;
        }

        for (int i = 0; i < SEGMENTS_PER_SIDE; ++i) {
            threepp::Matrix4 matrix;
            threepp::Vector3 position;
            threepp::Quaternion quaternion;

            float randomX = (static_cast<float>(std::rand()) / RAND_MAX * 0.4f - 0.2f) * segmentSpacing;
            float randomY = (static_cast<float>(std::rand()) / RAND_MAX * 0.5f - 0.1f) * baseScale;
            float randomScaleY = baseScale + (static_cast<float>(std::rand()) / RAND_MAX * 0.5f - 0.25f) * baseScale;

            threepp::Vector3 scaleVec(baseScale, randomScaleY, baseScale);

            switch (side) {
                case 0:
                    position.set(currentX + i * segmentSpacing + randomX, 0.0f, currentZ);
                    break;
                case 1:
                    position.set(currentX, 0.0f, currentZ + i * segmentSpacing + randomX);
                    break;
                case 2:
                    position.set(currentX - i * segmentSpacing + randomX, 0.0f, currentZ);
                    break;
                case 3:
                    position.set(currentX, 0.0f, currentZ - i * segmentSpacing + randomX);
                    break;
            }

            position.y = randomY;

            quaternion.setFromEuler(threepp::Euler(0, rotationY, 0));
            matrix.compose(position, quaternion, scaleVec);

            instancedBorder->setMatrixAt(index++, matrix);
        }
    }

    instancedBorder->instanceMatrix()->needsUpdate();
    scene.add(instancedBorder);
}
// ground plane generation
void createGround(Scene& scene) {
    auto groundGeometry = PlaneGeometry::create(MAP_SIZE, MAP_SIZE);
    auto groundMaterial = MeshStandardMaterial::create();
    groundMaterial->color = threepp::Color(0x527368);
    groundMaterial->roughness = 1.0f;
    groundMaterial->metalness = 0.0f;

    auto ground = Mesh::create(groundGeometry, groundMaterial);
    ground->rotation.x = -math::PI / 2;
    ground->position.y = 0.0f;
    ground->receiveShadow = true;
    scene.add(ground);
}

// tank
class Tank {
public:
    std::shared_ptr<Group> model;

    // belt speed system
    float leftTrackSpeed = 0.0f;
    float rightTrackSpeed = 0.0f;

    // steering intertia system
    float targetSteeringInput = 0.0f; // desired steering
    float currentSteeringInput = 0.0f; // actual steering
    float steeringInertia = 0.08f; // steering response (low = slow)

    // acceleration inertia system
    float targetThrottle = 0.0f; // desired throttle
    float currentThrottle = 0.0f; // actual throttle
    float throttleInertia = 0.05f; // throttle response (low = slow)

    // physics parameters
    float maxSpeedForward = 1.00f;
    float maxSpeedReverse = 0.50f;
    float acceleration = 0.002f;
    float reverseAcceleration = 0.0015f;
    float friction = 0.992f;
    float brakingForce = 0.95f;

    // steering parameters
    float trackWidth = 3.6f;
    float pivotSpeedThreshold = 0.8f;
    float maxTurnRate = 0.025f;
    float pivotTurnRate = 0.015f;

    // regenerative steering factor
    float steeringPowerTransfer = 0.05f; // reduce for smooth transition

    // visual elements
    std::shared_ptr<Mesh> leftTrack;
    std::shared_ptr<Mesh> rightTrack;

    Tank() {
        model = Group::create();

        auto greenColor = Color::green;
        auto darkGreen = Color(0x2d5016);
        auto veryDarkGreen = Color(0x1a2e0a);

        // body
        auto bodyGeometry = BoxGeometry::create(3, 1.2f, 5);
        auto bodyMaterial = MeshStandardMaterial::create();
        bodyMaterial->color = greenColor;
        bodyMaterial->metalness = 0.2f;
        bodyMaterial->roughness = 0.8f;
        auto body = Mesh::create(bodyGeometry, bodyMaterial);
        body->position.y = 0.8f;
        model->add(body);

        // turret
        auto turretGeometry = BoxGeometry::create(2, 1, 2.5f);
        auto turretMaterial = MeshStandardMaterial::create();
        turretMaterial->color = darkGreen;
        turretMaterial->metalness = 0.3f;
        turretMaterial->roughness = 0.7f;
        auto turret = Mesh::create(turretGeometry, turretMaterial);
        turret->position.set(0, 1.9f, -0.5f);
        model->add(turret);

        // cannon
        auto cannonGeometry = BoxGeometry::create(0.4f, 0.4f, 3);
        auto cannonMaterial = MeshStandardMaterial::create();
        cannonMaterial->color = veryDarkGreen;
        cannonMaterial->metalness = 0.4f;
        cannonMaterial->roughness = 0.6f;
        auto cannon = Mesh::create(cannonGeometry, cannonMaterial);
        cannon->position.set(0, 1.9f, 1.5f);
        model->add(cannon);

        // tracks
        auto trackMaterial = MeshStandardMaterial::create();
        trackMaterial->color = veryDarkGreen;
        trackMaterial->metalness = 0.5f;
        trackMaterial->roughness = 0.9f;

        auto leftTrackGeometry = BoxGeometry::create(0.6f, 0.8f, 5.5f);
        leftTrack = Mesh::create(leftTrackGeometry, trackMaterial);
        leftTrack->position.set(-1.8f, 0.4f, 0);
        model->add(leftTrack);

        auto rightTrackGeometry = BoxGeometry::create(0.6f, 0.8f, 5.5f);
        rightTrack = Mesh::create(rightTrackGeometry, trackMaterial);
        rightTrack->position.set(1.8f, 0.4f, 0);
        model->add(rightTrack);

        // wheels
        auto wheelMaterial = MeshStandardMaterial::create();
        wheelMaterial->color = Color::black;
        wheelMaterial->metalness = 0.2f;
        wheelMaterial->roughness = 0.9f;

        for (int i = 0; i < 5; i++) {
            auto wheelGeometry = BoxGeometry::create(0.4f, 0.6f, 0.6f);
            auto leftWheel = Mesh::create(wheelGeometry, wheelMaterial);
            leftWheel->position.set(-1.8f, 0.4f, -2.0f + i * 1.0f);
            model->add(leftWheel);

            auto rightWheel = Mesh::create(wheelGeometry, wheelMaterial);
            rightWheel->position.set(1.8f, 0.4f, -2.0f + i * 1.0f);
            model->add(rightWheel);
        }
    }

    void update() {
    // gradually apply steering input (inertia)
    currentSteeringInput += (targetSteeringInput - currentSteeringInput) * steeringInertia;

    // gradually apply throttle input (inertia)
    currentThrottle += (targetThrottle - currentThrottle) * throttleInertia;

    // apply throttle to both tracks
    if (currentThrottle > 0.01f) {
        float throttleForce = currentThrottle * acceleration;
        leftTrackSpeed += throttleForce;
        rightTrackSpeed += throttleForce;
    } else if (currentThrottle < -0.01f) {
        float throttleForce = currentThrottle * reverseAcceleration;
        leftTrackSpeed += throttleForce;
        rightTrackSpeed += throttleForce;
    }

    // calculate average speed for steering mode selection
    float avgSpeed = (leftTrackSpeed + rightTrackSpeed) / 2.0f;
    float absAvgSpeed = std::abs(avgSpeed);

    // apply steering based on current (smoothed) input
        if (std::abs(currentSteeringInput) > 0.01f) {
            if (absAvgSpeed < pivotSpeedThreshold) {
                // pivot steering at low speed
                float steerForce = currentSteeringInput * pivotTurnRate * 0.5f;
                leftTrackSpeed -= steerForce;
                rightTrackSpeed += steerForce;

                // prevent net forward motion when only steering (no throttle)
                if (std::abs(currentThrottle) < 0.01f) {
                    float avgSpeedNow = (leftTrackSpeed + rightTrackSpeed) / 2.0f;
                    leftTrackSpeed -= avgSpeedNow;
                    rightTrackSpeed -= avgSpeedNow;
                }
            } else {
                // regenerative steering at high speed
                // pure power transfer - slow one track, speed up the other equally
                float speedFactor = std::min(1.0f, pivotSpeedThreshold / absAvgSpeed);
                float steerForce = currentSteeringInput * steeringPowerTransfer * speedFactor;

                // transfer speed from one track to the other (no net gain)
                leftTrackSpeed -= steerForce;
                rightTrackSpeed += steerForce;

                // preserve original average speed (steering shouldn't change overall speed)
                float avgSpeedNow = (leftTrackSpeed + rightTrackSpeed) / 2.0f;
                float correction = avgSpeed - avgSpeedNow;
                leftTrackSpeed += correction;
                rightTrackSpeed += correction;
            }
        }

    // apply friction
    leftTrackSpeed *= friction;
    rightTrackSpeed *= friction;

    // clamp speeds
    leftTrackSpeed = std::clamp(leftTrackSpeed, -maxSpeedReverse, maxSpeedForward);
    rightTrackSpeed = std::clamp(rightTrackSpeed, -maxSpeedReverse, maxSpeedForward);

    // calculate turn rate from track speed difference
    float turnRate = (rightTrackSpeed - leftTrackSpeed) / trackWidth * maxTurnRate;

    // update rotation
    model->rotation.y += turnRate;

    // recalculate avgSpeed after modifications
    avgSpeed = (leftTrackSpeed + rightTrackSpeed) / 2.0f;

    // calculate movement
    float moveX = std::sin(model->rotation. y) * avgSpeed;
    float moveZ = std::cos(model->rotation.y) * avgSpeed;

    model->position.x += moveX;
    model->position.z += moveZ;

    // visual belt animation
    if (std::abs(leftTrackSpeed) > 0.001f) {
        leftTrack->position.z = std::fmod(leftTrack->position.z - leftTrackSpeed * 0.5f, 1.0f);
    }
    if (std::abs(rightTrackSpeed) > 0.001f) {
        rightTrack->position.z = std::fmod(rightTrack->position.z - rightTrackSpeed * 0.5f, 1.0f);
    }

    // Reset targets for next frame
    targetSteeringInput = 0.0f;
    targetThrottle = 0.0f;
}

    // forward acceleration
    void accelerateLeft() {
        leftTrackSpeed += acceleration;
    }

    void accelerateRight() {
        rightTrackSpeed += acceleration;
    }

    // reverse acceleration
    void brakeLeft() {
        leftTrackSpeed -= reverseAcceleration;
    }

    void brakeRight() {
        rightTrackSpeed -= reverseAcceleration;
    }

    // active braking
    void activeBrakeLeft() {
        leftTrackSpeed *= brakingForce;
        if (std::abs(leftTrackSpeed) < 0.1f) leftTrackSpeed = 0.0f;
    }

    void activeBrakeRight() {
        rightTrackSpeed *= brakingForce;
        if (std::abs(rightTrackSpeed) < 0.1f) rightTrackSpeed = 0.0f;
    }

    // stop belt
    void stopLeft() { leftTrackSpeed = 0; }
    void stopRight() { rightTrackSpeed = 0; }

    // aux function, regenerative steering at high speed
    void applyRegenerativeSteering(bool turnLeft) {
        float avgSpeed = (leftTrackSpeed + rightTrackSpeed) / 2.0f;
        float absAvgSpeed = std::abs(avgSpeed);

        if (absAvgSpeed > pivotSpeedThreshold) {
            if (turnLeft) {
                // reduce left track speed gradually, increase right track speed a little
                float powerTransfer = acceleration * steeringPowerTransfer;
                leftTrackSpeed -= powerTransfer;
                rightTrackSpeed += powerTransfer * 0.3f; //less boost for smoother transition
            } else {
                // reduce right track speed gradually, increase left track speed a little
                float powerTransfer = acceleration * steeringPowerTransfer;
                rightTrackSpeed -= powerTransfer;
                leftTrackSpeed += powerTransfer * 0.3f;
            }
        }
    }
};
// forward declaration for getTankHalfExtents (defined later in file)
Vector3 getTankHalfExtents();

class CollisionVisualizer {
public:
    std::shared_ptr<Group> debugGroup;
    bool enabled = false;

    // store debug meshes for obstacles
    std::vector<std::shared_ptr<Mesh>> obstacleMeshes;
    std::shared_ptr<Mesh> tankMesh;

    CollisionVisualizer() {
        debugGroup = Group::create();
    }

    void initialize(Scene& scene, const std::vector<Obstacle>& obstacles, Tank& tank) {
        // create semi-transparent red material
        auto debugMaterial = MeshBasicMaterial::create();
        debugMaterial->color = Color(0xff0000);
        debugMaterial->transparent = true;
        debugMaterial->opacity = 0.3f;
        debugMaterial->wireframe = false;
        debugMaterial->depthWrite = false; // Prevent z-fighting

        // create wireframe material for edges
        auto wireframeMaterial = MeshBasicMaterial::create();
        wireframeMaterial->color = Color(0xff0000);
        wireframeMaterial->wireframe = true;
        wireframeMaterial->opacity = 0.8f;
        wireframeMaterial->transparent = true;

        // create debug meshes for obstacles
        for (const auto& obs : obstacles) {
            std::shared_ptr<Mesh> debugMesh;

            if (obs.type == ObstacleType::ROCK_SPHERE) {
                // sphere geometry for rocks
                auto sphereGeom = SphereGeometry::create(obs.radius, 16, 16);
                debugMesh = Mesh::create(sphereGeom, debugMaterial);
                debugMesh->position.copy(obs.position);

                // add wireframe overlay
                auto wireMesh = Mesh::create(sphereGeom, wireframeMaterial);
                wireMesh->position.copy(obs.position);
                debugGroup->add(wireMesh);

            } else if (obs.type == ObstacleType::TREE_CYLINDER) {
                // cylinder geometry for trees
                auto cylGeom = CylinderGeometry::create(
                    obs.radius,           // radiusTop
                    obs.radius,           // radiusBottom
                    obs.halfHeight * 2.0f, // height
                    16                     // radialSegments
                );
                debugMesh = Mesh::create(cylGeom, debugMaterial);
                debugMesh->position.copy(obs.position);

                // add wireframe overlay
                auto wireMesh = Mesh::create(cylGeom, wireframeMaterial);
                wireMesh->position.copy(obs.position);
                debugGroup->add(wireMesh);
            }

            if (debugMesh) {
                obstacleMeshes.push_back(debugMesh);
                debugGroup->add(debugMesh);
            }
        }

        // create debug mesh for tank (OBB)
        Vector3 tankHalfExtents = getTankHalfExtents();
        auto tankBoxGeom = BoxGeometry::create(
            tankHalfExtents.x * 2.0f,
            tankHalfExtents.y * 2.0f,
            tankHalfExtents.z * 2.0f
        );

        tankMesh = Mesh::create(tankBoxGeom, debugMaterial);

        // add wireframe for tank
        auto tankWireMesh = Mesh::create(tankBoxGeom, wireframeMaterial);
        debugGroup->add(tankWireMesh);
        debugGroup->add(tankMesh);

        // add debug group to scene but keep it hidden initially
        scene.add(debugGroup);
        debugGroup->visible = false;
    }

    void update(Tank& tank) {
        if (! enabled || !tankMesh) return;

        // update tank debug mesh to match tank position and rotation
        tankMesh->position. copy(tank.model->position);
        tankMesh->rotation.copy(tank. model->rotation);  // Copy full rotation, not just Y

        // update wireframe tank mesh (it's the second-to-last child)
        if (debugGroup->children.size() >= 2) {
            auto wireTank = debugGroup->children[debugGroup->children.size() - 2];
            if (wireTank->is<Mesh>()) {
                wireTank->position.copy(tank. model->position);
                wireTank->rotation.copy(tank.model->rotation);  // Copy full rotation here too
            }
        }
    }

    void toggle() {
        enabled = !enabled;
        debugGroup->visible = enabled;
        std::cout << "\nCollision Visualization: " << (enabled ? "ON" : "OFF") << std::endl;
    }

    bool isEnabled() const {
        return enabled;
    }
};

// aux: clamp
static inline float clampf(float v, float a, float b) {
    return std::max(a, std::min(b, v));
}

// transform a point to tank local space (uses model-transform inverse)
Vector3 transformPointToLocal(const Group& g, const Vector3& worldPoint) {

    // matrixWorld is a shared_ptr<Matrix4>
    Matrix4 inv = *g.matrixWorld;
    inv.invert();

    Vector3 p = worldPoint;     // applyMatrix4 requires non-const
    p.applyMatrix4(inv);
    return p;
}

// OBB (tank) approximate extents (half dimensions) in local space
// we use the constructed body/turret/wheels but simpler: we know the body box dimensions = (3,1.2,5) and tracks etc.
Vector3 getTankHalfExtents() {
    // based on body geometry from constructor (3 x 1.2 x 5)
    return Vector3(3.0f/2.0f, 1.2f/2.0f + 1.0f, 5.0f/2.0f);
}

// OBB vs sphere test: transform sphere center to OBB local, clamp, find distance
bool OBBvsSphere(const Group& tankModel, const Vector3 &sphereCenter, float sphereRadius, Vector3 &outClosestPointLocal) {
    Vector3 localCenter = transformPointToLocal(tankModel, sphereCenter);
    Vector3 half = getTankHalfExtents();

    outClosestPointLocal.x = clampf(localCenter.x, -half.x, half.x);
    outClosestPointLocal.y = clampf(localCenter.y, -half.y, half.y);
    outClosestPointLocal.z = clampf(localCenter.z, -half.z, half.z);

    // calculate squared distance in local space
    Vector3 diff = localCenter - outClosestPointLocal;
    float dist2 = diff.lengthSq();
    return dist2 <= sphereRadius * sphereRadius;
}

// OBB vs cylinder (axis aligned in world-space along Y)
bool OBBvsCylinder(const Group& tankModel, const Vector3 &cylCenter, float cylRadius, float cylHalfHeight, Vector3 &outClosestPointWorld) {
    Vector3 half = getTankHalfExtents();

    // fix: account for the tank body's local Y offset (0.8f) relative to the group root
    Vector3 tankCenter = tankModel.position;
    float tankBodyCenterY = tankCenter.y + 0.8f;

    // tank Y-extents in world
    float tankMinY = tankBodyCenterY - half.y;
    float tankMaxY = tankBodyCenterY + half.y;

    float cylMinY = cylCenter.y - cylHalfHeight;
    float cylMaxY = cylCenter.y + cylHalfHeight;

    // Y-overlap check:
    if (tankMaxY < cylMinY || tankMinY > cylMaxY) {
        return false;
    }

    // project cylinder center into tank-local space
    Vector3 localC = transformPointToLocal(tankModel, cylCenter);

    // half extents in local
    Vector3 halfLocal = half;

    float closestX = clampf(localC.x, -halfLocal.x, halfLocal.x);
    float closestZ = clampf(localC.z, -halfLocal.z, halfLocal.z);

    // compute squared distance in XZ-plane
    float dx = localC.x - closestX;
    float dz = localC.z - closestZ;
    float dist2 = dx*dx + dz*dz;
    bool overlap = dist2 <= cylRadius*cylRadius;

    if (!overlap) return false;

    Vector3 closestLocal(closestX, clampf(localC.y, -halfLocal.y, halfLocal.y), closestZ);

    // transform closestLocal to world
    Matrix4 mat = *tankModel.matrix;
    outClosestPointWorld = closestLocal.applyMatrix4(mat);

    return true;
}

// broad-phase: tank bounding sphere radius (approx)
float getTankBoundingSphereRadius() {
    // conservative bound: diagonal of OBB extents
    Vector3 half = getTankHalfExtents();
    return std::sqrt(half.x*half.x + half.y*half.y + half.z*half.z);
}

// separate tank out of penetration: push tank along direction from obstacle to tank by penetration depth
void separateTankFromPoint(Group &tankModel, const Vector3 &pushFromWorld, float pushDistance) {

    // horizontal tank to obstacle
    Vector3 dir = tankModel.position.clone().sub(pushFromWorld);

    // force collision response to XZ-plan
    dir.y = 0;

    if (dir.lengthSq() < 1e-6f) {
        dir.set(std::sin(tankModel.rotation.y), 0, std::cos(tankModel.rotation.y));
    } else {
        dir.normalize();
    }

    // retain original y position after collision
    float originalY = tankModel.position.y;

    tankModel.position.addScaledVector(dir, pushDistance + 0.01f);

    tankModel.position.y = originalY;
}

// collisionSystem: checks nearby obstacles and resolves collisions
void resolveCollisions(Tank &tank) {
    // broad-phase: tank bounding sphere center is tank.model->position
    Vector3 tankCenter = tank.model->position;
    float tankSphereRadius = getTankBoundingSphereRadius();

    // query nearby obstacles using spatial grid (1 cell radius). This reduces checks.
    std::vector<int> candidates = queryNearbyObstacleIndices(tankCenter, 1);

    // if grid empty, optionally fallback to check all (we choose to simply return)
    if (candidates.empty()) return;

    // for each candidate, first broad-phase precise sphere-sphere test (if obstacle is sphere) or sphere-approx test (use obstacle.radius)
    for (int idx : candidates) {
        const Obstacle &obs = g_obstacles[idx];

        // quick sphere-based broad-phase: distance squared between centers
        float dx = tankCenter.x - obs.position.x;
        float dz = tankCenter.z - obs.position.z;
        float dy = tankCenter.y - obs.position.y;
        float dist2 = dx*dx + dy*dy + dz*dz;
        float broadR = tankSphereRadius + obs.radius;
        if (dist2 > broadR*broadR) {
            continue; // not in range
        }

        // narrow-phase: OBB (tank) vs sphere/cylinder
        if (obs.type == ObstacleType::ROCK_SPHERE) {
            Vector3 closestLocal;
            bool hit = OBBvsSphere(*tank.model, obs.position, obs.radius, closestLocal);
            if (hit) {
                // compute the closest point in world
                Matrix4 mat = *tank.model->matrix;
                Vector3 closestWorld = closestLocal.applyMatrix4(mat);

                // compute penetration depth along vector from rock center to closestWorld
                Vector3 fromRockToClosest = closestWorld.clone().sub(obs.position);
                float d = fromRockToClosest.length();
                float penetration = (obs.radius) - d;
                if (penetration < 0) penetration = 0.0f;

                // response: move tank out and damp momentum
                if (d > 1e-6f) {
                    separateTankFromPoint(*tank.model, obs.position, penetration + 0.01f);
                } else {
                    // if coincided, push backwards
                    Vector3 pushDir(std::sin(tank.model->rotation.y), 0, std::cos(tank.model->rotation.y));
                    tank.model->position.addScaledVector(pushDir, -0.2f);
                }

                // reduce momentum strongly
                tank.leftTrackSpeed *= 0.2f;
                tank.rightTrackSpeed *= 0.2f;
            }
        } else if (obs.type == ObstacleType::TREE_CYLINDER) {
            Vector3 closestWorld;
            bool hit = OBBvsCylinder(*tank.model, obs.position, obs.radius, obs.halfHeight, closestWorld);
            if (hit) {
                // compute horizontal penetration in XZ plane for separation
                Vector3 dir = tank.model->position.clone().sub(obs.position);
                dir.y = 0;
                float d = dir.length();
                if (d < 1e-6f) {
                    // fallback push
                    dir.set(std::sin(tank.model->rotation.y), 0, std::cos(tank.model->rotation.y));
                    d = dir.length();
                }
                dir.normalize();
                float overlap = (tankSphereRadius + obs.radius) - d;
                if (overlap < 0) overlap = 0.0f;
                tank.model->position.addScaledVector(dir, overlap + 0.01f);

                // reduce momentum moderately
                tank.leftTrackSpeed *= 0.15f;
                tank.rightTrackSpeed *= 0.15f;
            }
        }
    }
}

// cameraSystem with chase and free look
struct CameraSystem {
    enum Mode { CHASE, FREE_LOOK };
    Mode currentMode = CHASE;

    float freeLookSpeed = 0.5f;
    float yaw = 0.0f;

    void toggleMode() {
        currentMode = (currentMode == CHASE) ? FREE_LOOK : CHASE;
        std::cout << "\nCamera Mode: " << (currentMode == CHASE ? "CHASE" : "FREE LOOK") << std::endl;
    }

    void updateChase(PerspectiveCamera& camera, const Vector3& tankPos, float tankYaw) {
        const float chaseDistance = 20.0f;
        const float chaseHeight = 8.0f;
        const float LerpFactor = 0.15f; //lerp

        Vector3 targetPos;
        targetPos.x = tankPos.x - std::sin(tankYaw) * chaseDistance;
        targetPos.y = tankPos.y + chaseHeight;
        targetPos.z = tankPos.z - std::cos(tankYaw) * chaseDistance;

        camera.position.x += (targetPos.x - camera.position.x) * LerpFactor;
        camera.position.y += (targetPos.y - camera.position.y) * LerpFactor;
        camera.position.z += (targetPos.z - camera.position.z) * LerpFactor;

        Vector3 lookAtPoint;
        lookAtPoint.x = tankPos.x + std::sin(tankYaw) * 5.0f;
        lookAtPoint.y = tankPos.y + 2.0f;
        lookAtPoint.z = tankPos.z + std::cos(tankYaw) * 5.0f;

        camera.lookAt(lookAtPoint);
    }

    void updateFreeLook(PerspectiveCamera& camera,
                        bool forward, bool backward,
                        bool left, bool right,
                        bool up, bool down,
                        bool rotLeft,
                        bool rotRight) {
        if (rotLeft)  yaw += 0.03f;
        if (rotRight) yaw -= 0.03f;

        Vector3 forwardVec(std::sin(yaw), 0, std::cos(yaw));
        Vector3 rightVec(std::cos(yaw), 0, -std::sin(yaw));

        if (forward)  camera.position.addScaledVector(forwardVec,  freeLookSpeed);
        if (backward) camera.position.addScaledVector(forwardVec, -freeLookSpeed);
        if (left)     camera.position.addScaledVector(rightVec,   -freeLookSpeed);
        if (right)    camera.position.addScaledVector(rightVec,    freeLookSpeed);
        if (up)       camera.position.y += freeLookSpeed;
        if (down)     camera.position.y -= freeLookSpeed;

        Vector3 lookTarget = camera.position + forwardVec;
        camera.lookAt(lookTarget);
    }
};

int main() {
    Canvas canvas("Tank Simulator");
    GLRenderer renderer(canvas. size());
    renderer.shadowMap(). enabled = true;

    Scene scene;
    scene.background = Color::skyblue;

    PerspectiveCamera camera(60, canvas.aspect(), 0.1f, 1000.f);
    camera.position. set(0, 15, 20);
    camera. lookAt({0, 0, 0});

    // initialize random seed
    std::srand(static_cast<unsigned>(std::time(nullptr)));

    // lighting system
    auto ambientLight = AmbientLight::create(Color(0xffffff), 0.3f);
    scene.add(ambientLight);

    auto sunLight = DirectionalLight::create(Color(0xfff4e6), 1.2f);
    sunLight->position.set(100, 200, 100);
    sunLight->castShadow = true;
    sunLight->shadow->radius = 5.0f;

    auto shadowCam = sunLight->shadow->camera->as<OrthographicCamera>();
    shadowCam->left = -MAP_SIZE / 2.0f;
    shadowCam->right = MAP_SIZE / 2.0f;
    shadowCam->top = MAP_SIZE / 2.0f;
    shadowCam->bottom = -MAP_SIZE / 2.0f;
    shadowCam->updateProjectionMatrix();

    scene.add(sunLight);

    auto fillLight = DirectionalLight::create(Color(0x8899ff), 0.2f);
    fillLight->position.set(-50, 50, -50);
    scene.add(fillLight);

    // create scene and populate with obstacles
    createGround(scene);
    createGrassFieldInstanced(scene,SimSettings::GRASS_COUNT);
    createRockFieldInstanced(scene,SimSettings::ROCK_COUNT);
    createTreeFieldInstanced(scene,SimSettings::TREE_COUNT);
    createCloudFieldInstanced(scene,SimSettings::CLOUD_COUNT);
    createTerrainBorder(scene);

    // Create tank BEFORE using it
    Tank tank;
    enableShadowsForGroup(tank.model.get(), true, true);
    scene.add(tank. model);

    // Create collision visualizer AFTER tank is created
    CollisionVisualizer collisionViz;

    // Initialize collision visualization AFTER both obstacles and tank are created
    collisionViz.initialize(scene, g_obstacles, tank);

    CameraSystem cameraSystem;

    std::map<Key, bool> keysPressed;
    KeyAdapter keyDownListener(KeyAdapter::Mode::KEY_PRESSED, [&](KeyEvent evt) {
        keysPressed[evt.key] = true;
        if (evt.key == Key::C) {
            cameraSystem.toggleMode();
        }
        if (evt.key == Key::O) {
        collisionViz.toggle();
        }
    });
    KeyAdapter keyUpListener(KeyAdapter::Mode::KEY_RELEASED, [&](KeyEvent evt) {
        keysPressed[evt.key] = false;
    });
    canvas.addKeyListener(keyDownListener);
    canvas.addKeyListener(keyUpListener);

    Clock clock;
    const float cloudSpeed = 0.01f;

    canvas.animate([&] {
    if (cameraSystem. currentMode == CameraSystem::CHASE) {
        // Set throttle target
        if (keysPressed[Key::W] || keysPressed[Key::UP]) {
            tank.targetThrottle = 1.0f;
        } else if (keysPressed[Key::S] || keysPressed[Key::DOWN]) {
            tank. targetThrottle = -1.0f;
        }

        // Set steering target
        if (keysPressed[Key::A] || keysPressed[Key::LEFT]) {
            tank.targetSteeringInput = 1.0f;
        } else if (keysPressed[Key::D] || keysPressed[Key::RIGHT]) {
            tank.targetSteeringInput = -1.0f;
        }

        // Active braking
        if (keysPressed[Key::SPACE]) {
            tank. leftTrackSpeed *= tank.brakingForce;
            tank.rightTrackSpeed *= tank.brakingForce;
        }
    }

    tank.update();

    // Collision resolution
    resolveCollisions(tank);

    // Update collision visualization
    collisionViz.update(tank);

    // Camera update
    if (cameraSystem. currentMode == CameraSystem::CHASE) {
        cameraSystem.updateChase(camera, tank. model->position, tank.model->rotation. y);
    } else {
        cameraSystem.updateFreeLook(camera,
            keysPressed[Key::W],
            keysPressed[Key::S],
            keysPressed[Key::D],
            keysPressed[Key::A],
            keysPressed[Key::X],
            keysPressed[Key::Z],
            keysPressed[Key::Q],
            keysPressed[Key::E]
        );
    }

    renderer.render(scene, camera);
});

    return 0;
}
