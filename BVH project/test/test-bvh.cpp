#include "common.hpp"       // Test utilities
#include "bvh.hpp"          // Bvh
#include "shapes.hpp"       // Dealing with shapes
#include "cs350_loader.hpp" // Loading scenes
#include "logging.hpp"      // Pretty printing
#include "stats.hpp"        // Keeping track of stats
#include "PRNG.h"           // Random number generator
#include "utils.hpp"

#include <gtest/gtest.h>
#include <limits>
#include <memory>
#include <optional>
#include <unordered_set>
#include <fstream>
#include <filesystem>
#include <algorithm>

namespace {
    struct Object;
    using Bvh     = CS350::Bvh<Object*>;
    using BvhNode = Bvh::Node;

    // Scene objects
    struct Object {
        unsigned    id{}; // Object identification
        CS350::Aabb bv{}; // Bounding volume of the object

        // Bvh information
        struct {
            Object*  next = nullptr; // Next object in the Bvh node
            Object*  prev = nullptr; // Previous object in the Bvh node
            BvhNode* node = nullptr; // The node it belongs to
        } bvhInfo;
    };

    /**
     * @brief
     *  Retrieve all the indices of a node (recursively)
     */
    std::vector<unsigned> BvhFlatMap(BvhNode const* n) {
        std::vector<unsigned> objectsIds;
        n->TraverseLevelOrderObjects([&](Object const* object) {
            objectsIds.push_back(object->id);
        });
        return objectsIds;
    }

    /**
     * @brief
     *  Shuffles items in a container naively
     */
    template <typename T>
    void shuffle(T& container) {
        for (size_t i = 0; i < container.size(); ++i) {
            auto lhs = CS170::Utils::rand() % container.size();
            auto rhs = CS170::Utils::rand() % container.size();
            std::swap(container.at(lhs), container.at(rhs));
        }
    }
}

namespace {
    constexpr float const       cTestEpsilon   = 1e-3f;
    constexpr char const*       cAssetPath     = "assets/cs350/gam400s20-mirlo/mirlo_{}.cs350_binary"; // To be used with fmt
    constexpr char const*       cSceneNormal   = "assets/cs350/gam400s20-mirlo/scene.txt";

    const CS350::BvhBuildConfig cTopDownConfig = {
        std::numeric_limits<unsigned>::max(), // max_depth
        20,                                   // min_objects
        250.0f,                               // min_volume
    };
    const CS350::BvhBuildConfig cBotUpConfig = {
        std::numeric_limits<unsigned>::max(), // max_depth
        0,                                    // min_objects
        250.0f,                               // min_volume
    };
    const CS350::BvhBuildConfig cInsertConfig = {
        100,          // max_depth
        1,            // min_objects
        10 * 10 * 10, // min_volume
    };

    // Test setup
    class BoundingVolumeHierarchy : public testing::Test {
      protected:
        static void SetUpTestSuite() { CS350::ChangeWorkdir(); }

        void SetUp() override {
            CS350::ChangeWorkdir();
            CS350::Stats::Instance().rayVsAabb     = 0;
            CS350::Stats::Instance().frustumVsAabb = 0;
        }

        std::vector<std::unique_ptr<Object> > storage_objects;

        template <typename C>
        std::vector<Object*> CreateObjects(C const& aabbs) {
            storage_objects.clear();
            std::vector<Object*> result;
            unsigned             idx = 0;
            for (CS350::Aabb const& bv : aabbs) {
                storage_objects.emplace_back(std::make_unique<Object>());
                auto* obj = storage_objects.back().get(); // Unsafe
                obj->bv   = bv;
                obj->id   = idx++;
                result.push_back(obj);
            }
            return result;
        }
    };

    /**
     * @brief
     *  Ensures nodes properties are satisfied
     */
    void AssertProperNodes(Bvh const& bvh) {
        // Ensures leaf/internal states
        bvh.TraverseLevelOrder([](BvhNode const* n) {
            if (n->IsLeaf()) {
                ASSERT_TRUE(n->ObjectCount() > 0) << "Leaf nodes should contain objects";
            } else {
                ASSERT_TRUE(n->ObjectCount() == 0) << "Internal nodes should NOT contain objects";
            }
        });

        // Ensures containment
        bvh.TraverseLevelOrder([](BvhNode const* n) {
            auto parentBv = n->bv;
            if (!n->IsLeaf()) {
                for (int i = 0; i < 2; ++i) {
                    auto*       child   = n->children[i];
                    auto const& childBv = child->bv;
                    ASSERT_LE(parentBv.min.x, childBv.min.x) << "Child node BV outside of parent BV";
                    ASSERT_LE(parentBv.min.y, childBv.min.y) << "Child node BV outside of parent BV";
                    ASSERT_LE(parentBv.min.z, childBv.min.z) << "Child node BV outside of parent BV";
                    ASSERT_GE(parentBv.max.x, childBv.max.x) << "Child node BV outside of parent BV";
                    ASSERT_GE(parentBv.max.y, childBv.max.y) << "Child node BV outside of parent BV";
                    ASSERT_GE(parentBv.max.z, childBv.max.z) << "Child node BV outside of parent BV";
                }
            }
        });
    }

    void AssertAllAccountedFor(Bvh const& bvh, std::vector<Object*> const& allObjects) {
        // Ensure all objects are tracked
        auto                         bvhIds = BvhFlatMap(bvh.root());
        std::unordered_set<unsigned> visibleBvhSet;
        for (auto id : bvhIds) {
            ASSERT_FALSE(visibleBvhSet.contains(id)) << "Object " << id << " has been found twice in the BVH";
            visibleBvhSet.insert(id);
        }
        for (auto* obj : allObjects) {
            ASSERT_TRUE(visibleBvhSet.contains(obj->id)) << "Object " << obj->id << " not found in the BVH";
        }
    }

    /**
     * @brief
     *  Given a Bounding volume hierarchy, print all useful information into a file
     */
    void PrintDebugInformation(Bvh const& bvh) {
#ifndef GRADING_SERVER
        // Debug information
        auto debugFile = std::ofstream(fmt::format(".{}.txt", TestName()));
        bvh.DumpInfo(debugFile);

        auto plotFile = std::ofstream(fmt::format(".{}.dot", TestName()));
        bvh.DumpGraph(plotFile);
#endif
    }

    /**
     * @brief
     *  Loads both the primitives and the scene
     */
    void LoadPrimitivesAndScene(std::vector<CS350::CS350PrimitiveData>& allPrimitives,
                                std::vector<CS350::CS350SceneObject>&   objects,
                                std::vector<CS350::Aabb>&               worldBvs,
                                char const*                             sceneFile) {
        // Load all models
        allPrimitives.clear();
        size_t lastIndex = 0;
        auto   assetPath = fmt::format(cAssetPath, lastIndex);
        while (std::filesystem::exists(assetPath)) {
            allPrimitives.push_back(CS350::LoadCS350Binary(assetPath));
            lastIndex++;
            assetPath = fmt::format(cAssetPath, lastIndex);
        }
        ASSERT_FALSE(allPrimitives.empty()) << "No assets were loaded";

        // Load the scene
        objects = CS350::LoadCS350Scene(sceneFile);

        // Process bvs
        worldBvs.clear();
        worldBvs.reserve(objects.size());
        for (auto& object : objects) {
            auto const& primitive = allPrimitives.at(static_cast<size_t>(object.primitiveIndex));
            // Retrieve objects model
            auto primitiveAabb = CS350::Aabb(primitive.bvMin, primitive.bvMax); // See it's aabb
            auto objectAabb    = primitiveAabb.transform(object.m2w);           // Transform it to world
            worldBvs.push_back(objectAabb);
        }
        ASSERT_GT(allPrimitives.size(), 1u) << "No primitives were loaded";
        ASSERT_GT(objects.size(), 1u) << "No objects were loaded";
        ASSERT_GT(worldBvs.size(), 1u) << "No bvs were created";
    }

    /**
     * @brief
     *  Places a virtual camera at random positions and directions. If it is visible in the brute force version,
     *   it must be visible in the BVH implementation.
     */
    void TestSceneAtRandomPositions(std::vector<Object*> const& objects, Bvh const& bvh, int positions = 100) {
        float averageTests = 0.0f;
        for (int i = 0; i < positions; ++i) {
            vec3 cameraPosition = vec3(CS170::Utils::Random(-100.0f, 100.0f), CS170::Utils::Random(-100.0f, 100.0f), CS170::Utils::Random(-100.0f, 100.0f));
            vec3 cameraTarget   = vec3(CS170::Utils::Random(-10.0f, 10.0f), CS170::Utils::Random(-10.0f, 10.0f), CS170::Utils::Random(-10.0f, 10.0f));
            vec3 cameraUp       = vec3(0, 1, 0);
            mat4 view           = glm::lookAt(cameraPosition, cameraTarget, cameraUp);
            mat4 proj           = glm::perspective(glm::radians(50.0f), 1920.0f / 1080.0f, 0.01f, 1000.0f);
            mat4 viewProj       = proj * view;

            CS350::Frustum frustum(viewProj);

            // Brute force query
            std::unordered_set<size_t> visibleBf;
            CS350::Stats::Instance().Reset();
            for (auto* object : objects) {
                if (frustum.classify(object->bv) != CS350::SideResult::eOUTSIDE) {
                    visibleBf.insert(object->id);
                }
            }
            ASSERT_EQ(CS350::Stats::Instance().frustumVsAabb, static_cast<unsigned>(objects.size())); // As many as Bvs

            // BVH approach
            CS350::Stats::Instance().Reset();
            auto                       visibleBvh = bvh.Query(frustum);
            std::unordered_set<size_t> visibleBvhSet(visibleBvh.begin(), visibleBvh.end());
            for (auto visibleInBv : visibleBf) {
                ASSERT_TRUE(visibleBvhSet.find(visibleInBv) != visibleBvhSet.end())
                    << "Object " << visibleInBv << " is visible on brute force approach but not on BVH approach."
                    << fmt::format(" cameraPosition: {}", cameraPosition).c_str()
                    << fmt::format(", cameraTarget: {}", cameraTarget).c_str();
            }
            ASSERT_GT(CS350::Stats::Instance().frustumVsAabb, 0); // At least we should test the root
            ASSERT_LT(CS350::Stats::Instance().frustumVsAabb,
                      objects.size()) // If we are making many queries, something is wrong
                << fmt::format(" cameraPosition: {}", cameraPosition).c_str()
                << fmt::format(", cameraTarget: {}", cameraTarget).c_str();
            averageTests += static_cast<float>(CS350::Stats::Instance().frustumVsAabb);
        }

        averageTests /= static_cast<float>(positions);
        ASSERT_LT(averageTests,
                  static_cast<float>(objects.size()) /
                      4.0f)
            << "Making way too many frustum/aabb calls, built BVH is too expensive";
        // If we are making many queries, something is wrong
    }

    void TestSceneRandomRays(std::vector<Object*> const& objects, Bvh const& bvh, int tries = 100, bool checkPerformance = true) {
        // Quickfix
        auto objectWithId = [&objects](unsigned id) -> Object* {
            for (auto& obj : objects) {
                if (obj->id == id) {
                    return obj;
                }
            }
            return nullptr;
        };

        float averageTests                  = 0.0f;
        float averageBvhRayQueries          = 0.0f;
        float averageBvhRayOptimizedQueries = 0.0f;
        for (int i = 0; i < tries; ++i) {
            vec3 rayStart = glm::normalize(vec3(CS170::Utils::Random(-100.0f, 100.0f),
                                                CS170::Utils::Random(-100.0f, 100.0f),
                                                CS170::Utils::Random(-100.0f, 100.0f))) *
                            2000.0f;
            vec3             rayTarget = vec3(CS170::Utils::Random(-100.0f, 100.0f), CS170::Utils::Random(-100.0f, 100.0f), CS170::Utils::Random(-100.0f, 100.0f));
            CS350::Ray const ray(rayStart, rayTarget - rayStart);

            // Brute force query
            float    smallestT     = std::numeric_limits<float>::max();
            unsigned closestObject = std::numeric_limits<unsigned>::max();
            CS350::Stats::Instance().Reset();
            std::unordered_set<size_t> hitsBf;
            for (auto* object : objects) {
                float t = ray.intersect(object->bv);
                if (t >= 0.0f) {
                    hitsBf.insert(object->id);
                    if (t < smallestT) {
                        smallestT     = t;
                        closestObject = object->id;
                    }
                }
            }

            // BVH approach (full)
            CS350::Stats::Instance().Reset();
            std::vector<unsigned>       allIntersectedObjects;
            std::vector<BvhNode const*> allIntersectedNodes;
            auto                        hitBvh       = bvh.QueryDebug(ray, false, allIntersectedObjects, allIntersectedNodes);
            float                       smallestTBvh = hitBvh.has_value() ? ray.intersect(objectWithId(hitBvh.value())->bv) : -1;

            // Ensure same closest intersection
            if (hitBvh || closestObject != std::numeric_limits<unsigned>::max()) {
                ASSERT_FLOAT_EQ(smallestT, smallestTBvh)
                    << "Closest object in brute force is not same as with BVH\n"
                    << fmt::format("\trayStart: {}\n", rayStart).c_str()
                    << fmt::format("\trayTarget: {}\n", rayTarget).c_str()
                    << fmt::format("\tsmallestT: {:.02f}\n", smallestT).c_str()
                    << fmt::format("\tsmallestTBvh: {:.02f}\n", smallestTBvh).c_str();
            }

            // Ensure same set of collisions
            std::unordered_set<size_t> hitsBvhSet(allIntersectedObjects.begin(), allIntersectedObjects.end());
            for (auto hitBf : hitsBf) {
                ASSERT_TRUE(hitsBvhSet.find(hitBf) != hitsBvhSet.end())
                    << "Object " << hitBf << " is hit on brute force approach but not on BVH approach.\b"
                    << fmt::format("\trayStart: {}\n", rayStart).c_str()
                    << fmt::format("\trayTarget: {}\n", rayTarget).c_str()
                    << fmt::format("\tsmallestT: {:.02f}\n", smallestT).c_str()
                    << fmt::format("\tsmallestTBvh: {:.02f}\n", smallestTBvh).c_str();
            }
            ASSERT_GT(CS350::Stats::Instance().rayVsAabb, 0); // At least we should test the root
            if (checkPerformance) {
                ASSERT_LT(CS350::Stats::Instance().rayVsAabb, objects.size());
                // If we are making many queries, something is wrong
            }
            averageTests += static_cast<float>(CS350::Stats::Instance().rayVsAabb);
            averageBvhRayQueries += static_cast<float>(CS350::Stats::Instance().rayVsAabb);
            {
                // BVH approach (single, optimized)
                CS350::Stats::Instance().Reset();
                auto hitSingleBvh = bvh.QueryDebug(ray, true, allIntersectedObjects, allIntersectedNodes);
                averageBvhRayOptimizedQueries += static_cast<float>(CS350::Stats::Instance().rayVsAabb);
                float smallestTBvhSingle = hitSingleBvh.has_value()
                                               ? ray.intersect(objectWithId(hitSingleBvh.value())->bv)
                                               : -1;
                ASSERT_FLOAT_EQ(smallestTBvhSingle,
                                smallestTBvh)
                    << "Different result between optimized and non-optimized"
                    << fmt::format("\trayStart: {}\n", rayStart).c_str()
                    << fmt::format("\trayTarget: {}\n", rayTarget).c_str()
                    << fmt::format(
                           "\tsmallestTBvhSingle: {:.02f}\n", smallestTBvhSingle)
                           .c_str()
                    << fmt::format(
                           "\tsmallestTBvh: {:.02f}\n", smallestTBvh)
                           .c_str();
            }
        }

        if (checkPerformance) {
            averageTests /= static_cast<float>(tries);
            ASSERT_LT(averageTests,
                      static_cast<float>(objects.size()) /
                          3.0f)
                << "Making way too many ray/aabb calls, built BVH is too expensive";
            // If we are making many queries, something is wrong

            averageBvhRayQueries /= static_cast<float>(tries);
            averageBvhRayOptimizedQueries /= static_cast<float>(tries);
            ASSERT_LT(averageBvhRayOptimizedQueries,
                      averageBvhRayQueries *
                          0.75f)
                << "Optimal version performs as many queries as non optimal queries (or not good enough)";
        }
    }
}

TEST_F(BoundingVolumeHierarchy, Unused) {
    ASSERT_NO_FATAL_FAILURE({ Bvh bvh; }); // Constructor + destructor of unused
}

TEST_F(BoundingVolumeHierarchy, TopDown_SingleAabb) {
    // Scene BVs
    CS350::Aabb const bvs[]      = { CS350::Aabb{ { 0, 0, 0 }, { 1, 1, 1 } } };
    auto              bvhObjects = CreateObjects(bvs);

    // BVH
    Bvh bvh;
    bvh.BuildTopDown(bvhObjects.begin(), bvhObjects.end(), cTopDownConfig);
    AssertProperNodes(bvh);
    AssertAllAccountedFor(bvh, bvhObjects);
    PrintDebugInformation(bvh);

    auto bvh_objects = BvhFlatMap(bvh.root());

    //
    ASSERT_NEAR(bvh.root()->bv, bvs[0], cTestEpsilon) << "Only a single node, should be tight";
    ASSERT_EQ(bvh.root()->Depth(), 0) << "Only a single node, should have depth 0";
    ASSERT_EQ(bvh.root()->Size(), 1);
    ASSERT_EQ(bvh_objects.size(), 1u);
    ASSERT_EQ(bvh_objects.front(), 0u);
}

TEST_F(BoundingVolumeHierarchy, TopDown_PairAabb) {
    // Scene BVs
    std::vector<CS350::Aabb> bvs = {
        CS350::Aabb{ { 0, 0, 0 }, { 1, 1, 1 } },
        CS350::Aabb{ { 1, 0, 0 }, { 2, 1, 1 } },
    };
    auto bvhObjects = CreateObjects(bvs);

    auto cfg       = cTopDownConfig;
    cfg.minObjects = 1;

    // BVH
    Bvh bvh;
    bvh.BuildTopDown(bvhObjects.begin(), bvhObjects.end(), cfg);
    PrintDebugInformation(bvh);
    AssertProperNodes(bvh);
    AssertAllAccountedFor(bvh, bvhObjects);
    auto bvh_objects = BvhFlatMap(bvh.root());

    //
    CS350::Aabb full = { { 0, 0, 0 }, { 2, 1, 1 } }; // Full scene Aabb
    ASSERT_NEAR(bvh.root()->bv, full, cTestEpsilon); // Should match
}

TEST_F(BoundingVolumeHierarchy, TopDown_ClearCheck) {
    // Scene BVs
    std::vector<CS350::Aabb> bvs        = { CS350::Aabb{ { 0, 0, 0 }, { 1, 1, 1 } } };
    auto                     bvhObjects = CreateObjects(bvs);

    // BVH
    Bvh bvh;
    bvh.BuildTopDown(bvhObjects.begin(), bvhObjects.end(), cTopDownConfig);
    PrintDebugInformation(bvh);
    AssertProperNodes(bvh);
    AssertAllAccountedFor(bvh, bvhObjects);
    bvh.Clear();

    //
    ASSERT_EQ(bvh.Depth(), -1);
    ASSERT_EQ(bvh.Size(), 0);
    ASSERT_EQ(bvh.root(), nullptr);
}

TEST_F(BoundingVolumeHierarchy, TopDown_CornerCase) {
    // Scene BVs
    std::vector<CS350::Aabb> bvs(500, CS350::Aabb{ { 0, 0, 0 }, { 1, 1, 1 } }); // All equal
    auto                     bvhObjects = CreateObjects(bvs);

    // BVH
    Bvh bvh;
    bvh.BuildTopDown(bvhObjects.begin(), bvhObjects.end(), cTopDownConfig);
    PrintDebugInformation(bvh);
    AssertProperNodes(bvh);
    AssertAllAccountedFor(bvh, bvhObjects);
    ASSERT_EQ(bvh.Depth(), 0u);
    ASSERT_EQ(bvh.Size(), 1u);
    bvh.Clear();

    //
    ASSERT_EQ(bvh.Depth(), -1);
    ASSERT_EQ(bvh.Size(), 0);
    ASSERT_EQ(bvh.root(), nullptr);
}

TEST_F(BoundingVolumeHierarchy, TopDown_Rebuild) {
    // Scene BVs
    std::vector<CS350::Aabb> bvs        = { CS350::Aabb{ { 0, 0, 0 }, { 1, 1, 1 } } };
    auto                     bvhObjects = CreateObjects(bvs);

    // BVH
    Bvh bvh;
    bvh.BuildTopDown(bvhObjects.begin(), bvhObjects.end(), cTopDownConfig);
    PrintDebugInformation(bvh);
    AssertProperNodes(bvh);
    AssertAllAccountedFor(bvh, bvhObjects);

    //
    ASSERT_NO_FATAL_FAILURE(bvh.BuildTopDown(bvhObjects.begin(), bvhObjects.end(), cTopDownConfig));
}

TEST_F(BoundingVolumeHierarchy, TopDown_MirloRandom) {
    CS170::Utils::srand(5, 5);
    std::vector<CS350::CS350PrimitiveData> allPrimitives;
    std::vector<CS350::CS350SceneObject>   objects;
    std::vector<CS350::Aabb>               worldBvs;
    LoadPrimitivesAndScene(allPrimitives, objects, worldBvs, cSceneNormal);
    auto bvhObjects = CreateObjects(worldBvs);

    Bvh bvh;
    bvh.BuildTopDown(bvhObjects.begin(), bvhObjects.end(), cTopDownConfig);
    PrintDebugInformation(bvh);
    AssertProperNodes(bvh);
    AssertAllAccountedFor(bvh, bvhObjects);
    TestSceneAtRandomPositions(bvhObjects, bvh);
}

TEST_F(BoundingVolumeHierarchy, TopDown_MirloOutside) {
    CS170::Utils::srand(5, 5);
    std::vector<CS350::CS350PrimitiveData> allPrimitives;
    std::vector<CS350::CS350SceneObject>   objects;
    std::vector<CS350::Aabb>               worldBvs;
    LoadPrimitivesAndScene(allPrimitives, objects, worldBvs, cSceneNormal);
    auto bvhObjects = CreateObjects(worldBvs);

    Bvh bvh;
    bvh.BuildTopDown(bvhObjects.begin(), bvhObjects.end(), cTopDownConfig);
    PrintDebugInformation(bvh);
    AssertProperNodes(bvh);
    AssertAllAccountedFor(bvh, bvhObjects);

    vec3 max = bvh.root()->bv.max;

    // Place a camera outside the root, looking in an opposite direction
    vec3 cameraPosition = max + vec3(1, 0, 0);
    vec3 cameraTarget   = cameraPosition + vec3(1, 1, 1);
    vec3 cameraUp       = vec3(0, 1, 0);
    mat4 view           = glm::lookAt(cameraPosition, cameraTarget, cameraUp);
    mat4 proj           = glm::perspective(glm::radians(50.0f), 1920.0f / 1080.0f, 0.01f, 1000.0f);
    mat4 viewProj       = proj * view;
    auto visible        = bvh.Query(CS350::Frustum(viewProj));
    ASSERT_TRUE(visible.empty()) << "Camera is outside, looking outside, nothing should be visible";
    ASSERT_EQ(CS350::Stats::Instance().frustumVsAabb, 1u) << "Only root should have been tested";
}

TEST_F(BoundingVolumeHierarchy, TopDown_MirloCompletelyInside) {
    CS170::Utils::srand(5, 5);
    std::vector<CS350::CS350PrimitiveData> allPrimitives;
    std::vector<CS350::CS350SceneObject>   objects;
    std::vector<CS350::Aabb>               worldBvs;
    LoadPrimitivesAndScene(allPrimitives, objects, worldBvs, cSceneNormal);
    auto bvhObjects = CreateObjects(worldBvs);

    Bvh bvh;
    bvh.BuildTopDown(bvhObjects.begin(), bvhObjects.end(), cTopDownConfig);
    PrintDebugInformation(bvh);
    AssertProperNodes(bvh);
    AssertAllAccountedFor(bvh, bvhObjects);

    vec3 max = bvh.root()->bv.max;
    vec3 min = bvh.root()->bv.min;

    // Place a camera outside the root, looking in an opposite direction
    vec3 cameraPosition = vec3(max.x + (max.x - min.x) * 2, 0, 0); // Make it big enough so that the scene is covered
    vec3 cameraTarget   = vec3(0, 0, 0);
    vec3 cameraUp       = vec3(0, 1, 0);
    mat4 view           = glm::lookAt(cameraPosition, cameraTarget, cameraUp);
    mat4 proj           = glm::perspective(glm::radians(50.0f), 1920.0f / 1080.0f, 0.01f, (max.x - min.x) * 5);
    // Make it big enough so that the scene is covered
    mat4 viewProj = proj * view;
    auto visible  = bvh.Query(CS350::Frustum(viewProj));
    ASSERT_EQ(visible.size(), objects.size()) << "All objects should be visible from this point";
    ASSERT_EQ(CS350::Stats::Instance().frustumVsAabb, 1u) << "Only root should have been tested";
}

TEST_F(BoundingVolumeHierarchy, TopDown_MirloRandomRays) {
    CS170::Utils::srand(5, 5);
    std::vector<CS350::CS350PrimitiveData> allPrimitives;
    std::vector<CS350::CS350SceneObject>   objects;
    std::vector<CS350::Aabb>               worldBvs;
    LoadPrimitivesAndScene(allPrimitives, objects, worldBvs, cSceneNormal);
    auto bvhObjects = CreateObjects(worldBvs);

    Bvh bvh;
    bvh.BuildTopDown(bvhObjects.begin(), bvhObjects.end(), cTopDownConfig);
    PrintDebugInformation(bvh);
    AssertProperNodes(bvh);
    AssertAllAccountedFor(bvh, bvhObjects);
    TestSceneRandomRays(bvhObjects, bvh, 100, true);
}

TEST_F(BoundingVolumeHierarchy, Insert_MirloRandom) {
    CS170::Utils::srand(5, 5);
    std::vector<CS350::CS350PrimitiveData> allPrimitives;
    std::vector<CS350::CS350SceneObject>   objects;
    std::vector<CS350::Aabb>               worldBvs;
    LoadPrimitivesAndScene(allPrimitives, objects, worldBvs, cSceneNormal);
    auto bvhObjects = CreateObjects(worldBvs);

    Bvh bvh;
    bvh.Insert(bvhObjects.begin(), bvhObjects.end(), cInsertConfig);
    PrintDebugInformation(bvh);
    AssertProperNodes(bvh);
    AssertAllAccountedFor(bvh, bvhObjects);
    TestSceneAtRandomPositions(bvhObjects, bvh);
}

TEST_F(BoundingVolumeHierarchy, Insert_MirloCompletelyInside) {
    CS170::Utils::srand(5, 5);
    std::vector<CS350::CS350PrimitiveData> allPrimitives;
    std::vector<CS350::CS350SceneObject>   objects;
    std::vector<CS350::Aabb>               worldBvs;
    LoadPrimitivesAndScene(allPrimitives, objects, worldBvs, cSceneNormal);
    auto bvhObjects = CreateObjects(worldBvs);
    shuffle(bvhObjects);

    Bvh bvh;
    bvh.Insert(bvhObjects.begin(), bvhObjects.end(), cInsertConfig);
    PrintDebugInformation(bvh);
    AssertProperNodes(bvh);
    AssertAllAccountedFor(bvh, bvhObjects);

    vec3 max = bvh.root()->bv.max;
    vec3 min = bvh.root()->bv.min;

    // Place a camera outside the root, looking in an opposite direction
    vec3 cameraPosition = vec3(max.x + (max.x - min.x) * 2, 0, 0);
    vec3 cameraTarget   = vec3(0, 0, 0);
    vec3 cameraUp       = vec3(0, 1, 0);
    mat4 view           = glm::lookAt(cameraPosition, cameraTarget, cameraUp);
    mat4 proj           = glm::perspective(glm::radians(50.0f), 1920.0f / 1080.0f, 0.01f, (max.x - min.x) * 5);
    mat4 viewProj       = proj * view;
    auto visible        = bvh.Query(CS350::Frustum(viewProj));
    ASSERT_EQ(visible.size(), objects.size()) << "All objects should be visible from this point";
    ASSERT_EQ(CS350::Stats::Instance().frustumVsAabb, 1u) << "Only root should have been tested";
}

TEST_F(BoundingVolumeHierarchy, Insert_SingleAabb) {
    // Scene BVs
    std::vector<CS350::Aabb> bvs        = { CS350::Aabb{ { 0, 0, 0 }, { 1, 1, 1 } } };
    auto                     bvhObjects = CreateObjects(bvs);

    // BVH
    Bvh bvh;
    for (unsigned i = 0; i < bvs.size(); ++i) {
        bvh.Insert(bvhObjects[i], cInsertConfig);
        AssertProperNodes(bvh);
        AssertAllAccountedFor(bvh, bvhObjects);
    }
    PrintDebugInformation(bvh);

    //
    ASSERT_NEAR(bvh.root()->bv, bvs[0], cTestEpsilon);
}

TEST_F(BoundingVolumeHierarchy, Insert_PairAabb) {
    // Scene BVs
    std::vector<CS350::Aabb> bvs = {
        CS350::Aabb{ { 0, 0, 0 }, { 1, 1, 1 } },
        CS350::Aabb{ { 1, 0, 0 }, { 2, 1, 1 } },
    };
    auto bvhObjects = CreateObjects(bvs);

    // BVH
    Bvh bvh;
    for (unsigned i = 0; i < bvhObjects.size(); ++i) {
        bvh.Insert(bvhObjects[i], cInsertConfig);
        AssertProperNodes(bvh);
    }
    AssertAllAccountedFor(bvh, bvhObjects);
    PrintDebugInformation(bvh);
    auto bvh_objects = BvhFlatMap(bvh.root());

    //
    CS350::Aabb full = { { 0, 0, 0 }, { 2, 1, 1 } }; // Full scene Aabb
    ASSERT_NEAR(bvh.root()->bv, full, cTestEpsilon); // Should match
}

TEST_F(BoundingVolumeHierarchy, Insert_ClearCheck) {
    // Scene BVs
    std::vector<CS350::Aabb> bvs        = { CS350::Aabb{ { 0, 0, 0 }, { 1, 1, 1 } } };
    auto                     bvhObjects = CreateObjects(bvs);

    // BVH
    Bvh bvh;
    for (unsigned i = 0; i < bvhObjects.size(); ++i) {
        bvh.Insert(bvhObjects[i], cInsertConfig);
        AssertProperNodes(bvh);
    }
    AssertAllAccountedFor(bvh, bvhObjects);
    PrintDebugInformation(bvh);
    bvh.Clear();

    //
    ASSERT_EQ(bvh.Depth(), -1);
    ASSERT_EQ(bvh.Size(), 0);
    ASSERT_EQ(bvh.root(), nullptr);
}

TEST_F(BoundingVolumeHierarchy, Insert_CornerCase) {
    // Scene BVs
    std::vector<CS350::Aabb> bvs(500, CS350::Aabb{ { 0, 0, 0 }, { 1, 1, 1 } });
    auto                     bvhObjects = CreateObjects(bvs);
    shuffle(bvhObjects);

    // BVH
    Bvh bvh;
    bvh.Insert(bvhObjects.begin(), bvhObjects.end(), cInsertConfig);
    PrintDebugInformation(bvh);
    AssertProperNodes(bvh);
    AssertAllAccountedFor(bvh, bvhObjects);
    ASSERT_EQ(bvh.Depth(), 0u);
    ASSERT_EQ(bvh.Size(), 1u);
    bvh.Clear();

    //
    ASSERT_EQ(bvh.Depth(), -1);
    ASSERT_EQ(bvh.Size(), 0);
    ASSERT_EQ(bvh.root(), nullptr);
}

TEST_F(BoundingVolumeHierarchy, Insert_Manual01) {
    // Scene BVs
    std::vector<CS350::Aabb> bvs = {
        CS350::Aabb{ { 1, 3, 0 }, { 3, 5, 1 } },  // 0
        CS350::Aabb{ { 4, 1, 0 }, { 6, 7, 1 } },  // 1
        CS350::Aabb{ { 6, 6, 0 }, { 7, 7, 1 } },  // 2
        CS350::Aabb{ { 6, 5, 0 }, { 7, 6, 1 } },  // 3
        CS350::Aabb{ { 6, 4, 0 }, { 7, 5, 1 } },  // 4
        CS350::Aabb{ { 6, 3, 0 }, { 7, 4, 1 } },  // 5
        CS350::Aabb{ { 6, 2, 0 }, { 7, 3, 1 } },  // 6
        CS350::Aabb{ { 6, 1, 0 }, { 7, 2, 1 } },  // 7
        CS350::Aabb{ { 8, 3, 0 }, { 9, 5, 1 } },  // 8
        CS350::Aabb{ { 9, 3, 0 }, { 10, 5, 1 } }, // 9
    };
    auto bvhObjects = CreateObjects(bvs);
    shuffle(bvhObjects);

    // Override config
    const CS350::BvhBuildConfig cInsertConfig = {
        100,       // max_depth
        1,         // min_objects
        1 * 1 * 1, // min_volume
    };

    // BVH
    Bvh bvh;
    bvh.Insert(bvhObjects.begin(), bvhObjects.end(), cInsertConfig);
    PrintDebugInformation(bvh);
    AssertProperNodes(bvh);
    AssertAllAccountedFor(bvh, bvhObjects);

    // Testing
    auto createRay = [](vec2 from, vec2 to) {
        vec3 st  = vec3(from.x, from.y, 0.5f); // 0.5f so that we are in the same plane as AABBs volumes
        vec3 end = vec3(to.x, to.y, 0.5f);
        vec3 dir = end - st;
        return CS350::Ray(st, dir);
    };

    // Some rays manually
    std::vector<unsigned>       allIntersectedObjects;
    std::vector<BvhNode const*> allIntersectedNodes;
    ASSERT_EQ(bvh.QueryDebug(createRay({ 0, 0 }, { 2, 4 }), true, allIntersectedObjects, allIntersectedNodes), 0u);
    ASSERT_EQ(bvh.QueryDebug(createRay({ 1, 1 }, { 2, 4 }), true, allIntersectedObjects, allIntersectedNodes), 0u);
    ASSERT_EQ(bvh.QueryDebug(createRay({ 5, 0 }, { 5, 1 }), true, allIntersectedObjects, allIntersectedNodes), 1u);
    ASSERT_EQ(bvh.QueryDebug(createRay({ 5, 20 }, { 5, 0 }), true, allIntersectedObjects, allIntersectedNodes), 1u);
    ASSERT_EQ(bvh.QueryDebug(createRay({ 3, 2 }, { 5, 3 }), true, allIntersectedObjects, allIntersectedNodes), 1u);
    ASSERT_EQ(bvh.QueryDebug(createRay({ 7.5f, 6.5f }, { 7, 6.5f }), true, allIntersectedObjects, allIntersectedNodes), 2u);
    ASSERT_EQ(bvh.QueryDebug(createRay({ 7.5f, 5.5f }, { 7, 5.5f }), true, allIntersectedObjects, allIntersectedNodes), 3u);
    ASSERT_EQ(bvh.QueryDebug(createRay({ 7.5f, 4.5f }, { 7, 4.5f }), true, allIntersectedObjects, allIntersectedNodes), 4u);
    ASSERT_EQ(bvh.QueryDebug(createRay({ 7.5f, 3.5f }, { 7, 3.5f }), true, allIntersectedObjects, allIntersectedNodes), 5u);
    ASSERT_EQ(bvh.QueryDebug(createRay({ 7.5f, 2.5f }, { 7, 2.5f }), true, allIntersectedObjects, allIntersectedNodes), 6u);
    ASSERT_EQ(bvh.QueryDebug(createRay({ 7.5f, 1.5f }, { 7, 1.5f }), true, allIntersectedObjects, allIntersectedNodes), 7u);
    ASSERT_EQ(bvh.QueryDebug(createRay({ 7.5f, 4.5f }, { 8, 4.5f }), true, allIntersectedObjects, allIntersectedNodes), 8u);
    ASSERT_EQ(bvh.QueryDebug(createRay({ 11, 4 }, { 8, 4 }), true, allIntersectedObjects, allIntersectedNodes), 9u);
    ASSERT_EQ(bvh.QueryDebug(createRay({ 0, 0 }, { 0, 1 }), true, allIntersectedObjects, allIntersectedNodes), std::nullopt);
    ASSERT_EQ(bvh.QueryDebug(createRay({ 1, 1 }, { 1, 0 }), true, allIntersectedObjects, allIntersectedNodes), std::nullopt);
    ASSERT_EQ(bvh.QueryDebug(createRay({ 3, 1 }, { 4, 10 }), true, allIntersectedObjects, allIntersectedNodes), std::nullopt);

    //
    bvh.Clear();
    ASSERT_EQ(bvh.Depth(), -1);
    ASSERT_EQ(bvh.Size(), 0);
    ASSERT_EQ(bvh.root(), nullptr);
}

TEST_F(BoundingVolumeHierarchy, Insert_Manual01_Cumulative) {
    // Scene BVs
    std::vector<CS350::Aabb> bvs = {
        CS350::Aabb{ { 1, 3, 0 }, { 3, 5, 1 } },  // 0
        CS350::Aabb{ { 4, 1, 0 }, { 6, 7, 1 } },  // 1
        CS350::Aabb{ { 6, 6, 0 }, { 7, 7, 1 } },  // 2
        CS350::Aabb{ { 6, 5, 0 }, { 7, 6, 1 } },  // 3
        CS350::Aabb{ { 6, 4, 0 }, { 7, 5, 1 } },  // 4
        CS350::Aabb{ { 6, 3, 0 }, { 7, 4, 1 } },  // 5
        CS350::Aabb{ { 6, 2, 0 }, { 7, 3, 1 } },  // 6
        CS350::Aabb{ { 6, 1, 0 }, { 7, 2, 1 } },  // 7
        CS350::Aabb{ { 8, 3, 0 }, { 9, 5, 1 } },  // 8
        CS350::Aabb{ { 9, 3, 0 }, { 10, 5, 1 } }, // 9
    };
    auto bvhObjects = CreateObjects(bvs);
    shuffle(bvhObjects);

    // Override config
    const CS350::BvhBuildConfig cInsertConfig = {
        100,       // max_depth
        1,         // min_objects
        1 * 1 * 1, // min_volume
    };

    // BVH
    Bvh bvh;
    bvh.Insert(bvhObjects.begin(), bvhObjects.end(), cInsertConfig);
    PrintDebugInformation(bvh);
    AssertProperNodes(bvh);
    AssertAllAccountedFor(bvh, bvhObjects);

    // Testing
    auto createRay = [](vec2 from, vec2 to) {
        vec3 st  = vec3(from.x, from.y, 0.5f); // 0.5f so that we are in the same plane as AABBs volumes
        vec3 end = vec3(to.x, to.y, 0.5f);
        vec3 dir = end - st;
        return CS350::Ray(st, dir);
    };

    // Some rays manually
    std::vector<unsigned>       others;
    std::vector<BvhNode const*> allIntersectedNodes;
    ASSERT_EQ(bvh.QueryDebug(createRay({ 0, 0 }, { 2, 4 }), false, others, allIntersectedNodes), 0u);
    ASSERT_EQ(bvh.QueryDebug(createRay({ 1, 1 }, { 2, 4 }), false, others, allIntersectedNodes), 0u);
    ASSERT_EQ(bvh.QueryDebug(createRay({ 5, 0 }, { 5, 1 }), false, others, allIntersectedNodes), 1u);
    ASSERT_EQ(bvh.QueryDebug(createRay({ 5, 20 }, { 5, 0 }), false, others, allIntersectedNodes), 1u);
    ASSERT_EQ(bvh.QueryDebug(createRay({ 3, 2 }, { 5, 3 }), false, others, allIntersectedNodes), 1u);
    ASSERT_EQ(bvh.QueryDebug(createRay({ 7.5f, 6.5f }, { 7, 6.5f }), false, others, allIntersectedNodes), 2u);
    ASSERT_EQ(bvh.QueryDebug(createRay({ 7.5f, 5.5f }, { 7, 5.5f }), false, others, allIntersectedNodes), 3u);
    ASSERT_EQ(bvh.QueryDebug(createRay({ 7.5f, 4.5f }, { 7, 4.5f }), false, others, allIntersectedNodes), 4u);
    ASSERT_EQ(bvh.QueryDebug(createRay({ 7.5f, 3.5f }, { 7, 3.5f }), false, others, allIntersectedNodes), 5u);
    ASSERT_EQ(bvh.QueryDebug(createRay({ 7.5f, 2.5f }, { 7, 2.5f }), false, others, allIntersectedNodes), 6u);
    ASSERT_EQ(bvh.QueryDebug(createRay({ 7.5f, 1.5f }, { 7, 1.5f }), false, others, allIntersectedNodes), 7u);
    ASSERT_EQ(bvh.QueryDebug(createRay({ 7.5f, 4.5f }, { 8, 4.5f }), false, others, allIntersectedNodes), 8u);
    ASSERT_EQ(bvh.QueryDebug(createRay({ 11, 4 }, { 8, 4 }), false, others, allIntersectedNodes), 9u);
    ASSERT_EQ(bvh.QueryDebug(createRay({ 0, 0 }, { 0, 1 }), false, others, allIntersectedNodes), std::nullopt);
    ASSERT_EQ(bvh.QueryDebug(createRay({ 1, 1 }, { 1, 0 }), false, others, allIntersectedNodes), std::nullopt);
    ASSERT_EQ(bvh.QueryDebug(createRay({ 3, 1 }, { 4, 10 }), false, others, allIntersectedNodes), std::nullopt);

    // Extra tested
    ASSERT_EQ(bvh.QueryDebug(createRay({ 0, 3.5 }, { 1, 3.5 }), false, others, allIntersectedNodes), 0);
    ASSERT_EQ(others.size(), 5) << "Did not find same intersections";
    ASSERT_TRUE(
        std::find(others.begin(), others.end(), 0) != others.end())
        << "Did not find a collision that should be there";
    ASSERT_TRUE(
        std::find(others.begin(), others.end(), 1) != others.end())
        << "Did not find a collision that should be there";
    ASSERT_TRUE(
        std::find(others.begin(), others.end(), 5) != others.end())
        << "Did not find a collision that should be there";
    ASSERT_TRUE(
        std::find(others.begin(), others.end(), 8) != others.end())
        << "Did not find a collision that should be there";
    ASSERT_TRUE(
        std::find(others.begin(), others.end(), 9) != others.end())
        << "Did not find a collision that should be there";

    ASSERT_EQ(bvh.QueryDebug(createRay({ 1.5, 3.5 }, { 1, 3.5 }), false, others, allIntersectedNodes), 0); // Inside 0
    ASSERT_EQ(others.size(), 1) << "Did not find same intersections";
    ASSERT_TRUE(
        std::find(others.begin(), others.end(), 0) != others.end())
        << "Did not find a collision that should be there";

    ASSERT_EQ(bvh.QueryDebug(createRay({ 50, 3.5 }, { 0, 3.5 }), false, others, allIntersectedNodes), 9);
    ASSERT_EQ(others.size(), 5) << "Did not find same intersections";
    ASSERT_TRUE(
        std::find(others.begin(), others.end(), 0) != others.end())
        << "Did not find a collision that should be there";
    ASSERT_TRUE(
        std::find(others.begin(), others.end(), 1) != others.end())
        << "Did not find a collision that should be there";
    ASSERT_TRUE(
        std::find(others.begin(), others.end(), 5) != others.end())
        << "Did not find a collision that should be there";
    ASSERT_TRUE(
        std::find(others.begin(), others.end(), 8) != others.end())
        << "Did not find a collision that should be there";
    ASSERT_TRUE(
        std::find(others.begin(), others.end(), 9) != others.end())
        << "Did not find a collision that should be there";

    ASSERT_EQ(bvh.QueryDebug(createRay({ 50, 3.5 }, { 0, 3.5 }), false, others, allIntersectedNodes), 9);
    ASSERT_EQ(others.size(), 5) << "Did not find same intersections";
    ASSERT_TRUE(
        std::find(others.begin(), others.end(), 0) != others.end())
        << "Did not find a collision that should be there";
    ASSERT_TRUE(
        std::find(others.begin(), others.end(), 1) != others.end())
        << "Did not find a collision that should be there";
    ASSERT_TRUE(
        std::find(others.begin(), others.end(), 5) != others.end())
        << "Did not find a collision that should be there";
    ASSERT_TRUE(
        std::find(others.begin(), others.end(), 8) != others.end())
        << "Did not find a collision that should be there";
    ASSERT_TRUE(
        std::find(others.begin(), others.end(), 9) != others.end())
        << "Did not find a collision that should be there";

    ASSERT_EQ(bvh.QueryDebug(createRay({ 6.5, -10 }, { 6.5, 1 }), false, others, allIntersectedNodes), 7);
    ASSERT_EQ(others.size(), 6) << "Did not find same intersections";
    ASSERT_TRUE(
        std::find(others.begin(), others.end(), 2) != others.end())
        << "Did not find a collision that should be there";
    ASSERT_TRUE(
        std::find(others.begin(), others.end(), 3) != others.end())
        << "Did not find a collision that should be there";
    ASSERT_TRUE(
        std::find(others.begin(), others.end(), 4) != others.end())
        << "Did not find a collision that should be there";
    ASSERT_TRUE(
        std::find(others.begin(), others.end(), 5) != others.end())
        << "Did not find a collision that should be there";
    ASSERT_TRUE(
        std::find(others.begin(), others.end(), 6) != others.end())
        << "Did not find a collision that should be there";
    ASSERT_TRUE(
        std::find(others.begin(), others.end(), 7) != others.end())
        << "Did not find a collision that should be there";

    ASSERT_EQ(bvh.QueryDebug(createRay({ 6.5, 50 }, { 6.5, 1 }), false, others, allIntersectedNodes), 2);
    ASSERT_EQ(others.size(), 6) << "Did not find same intersections";
    ASSERT_TRUE(
        std::find(others.begin(), others.end(), 2) != others.end())
        << "Did not find a collision that should be there";
    ASSERT_TRUE(
        std::find(others.begin(), others.end(), 3) != others.end())
        << "Did not find a collision that should be there";
    ASSERT_TRUE(
        std::find(others.begin(), others.end(), 4) != others.end())
        << "Did not find a collision that should be there";
    ASSERT_TRUE(
        std::find(others.begin(), others.end(), 5) != others.end())
        << "Did not find a collision that should be there";
    ASSERT_TRUE(
        std::find(others.begin(), others.end(), 6) != others.end())
        << "Did not find a collision that should be there";
    ASSERT_TRUE(
        std::find(others.begin(), others.end(), 7) != others.end())
        << "Did not find a collision that should be there";

    //
    bvh.Clear();
    ASSERT_EQ(bvh.Depth(), -1);
    ASSERT_EQ(bvh.Size(), 0);
    ASSERT_EQ(bvh.root(), nullptr);
}

TEST_F(BoundingVolumeHierarchy, Insert_Manual01_Random) {
    CS170::Utils::srand(932928255, 232551434);
    // Scene BVs
    std::vector<CS350::Aabb> bvs = {
        CS350::Aabb{ { 1, 3, 0 }, { 3, 5, 1 } },  // 0
        CS350::Aabb{ { 4, 1, 0 }, { 6, 7, 1 } },  // 1
        CS350::Aabb{ { 6, 6, 0 }, { 7, 7, 1 } },  // 2
        CS350::Aabb{ { 6, 5, 0 }, { 7, 6, 1 } },  // 3
        CS350::Aabb{ { 6, 4, 0 }, { 7, 5, 1 } },  // 4
        CS350::Aabb{ { 6, 3, 0 }, { 7, 4, 1 } },  // 5
        CS350::Aabb{ { 6, 2, 0 }, { 7, 3, 1 } },  // 6
        CS350::Aabb{ { 6, 1, 0 }, { 7, 2, 1 } },  // 7
        CS350::Aabb{ { 8, 3, 0 }, { 9, 5, 1 } },  // 8
        CS350::Aabb{ { 9, 3, 0 }, { 10, 5, 1 } }, // 9
    };
    auto bvhObjects = CreateObjects(bvs);
    shuffle(bvhObjects);

    // Override config
    const CS350::BvhBuildConfig cInsertConfig = {
        100,       // max_depth
        1,         // min_objects
        1 * 1 * 1, // min_volume
    };

    // BVH
    Bvh bvh;
    bvh.Insert(bvhObjects.begin(), bvhObjects.end(), cInsertConfig);
    PrintDebugInformation(bvh);
    AssertProperNodes(bvh);
    AssertAllAccountedFor(bvh, bvhObjects);
    TestSceneRandomRays(bvhObjects, bvh, 100000, false);

    //
    bvh.Clear();
    ASSERT_EQ(bvh.Depth(), -1);
    ASSERT_EQ(bvh.Size(), 0);
    ASSERT_EQ(bvh.root(), nullptr);
}

TEST_F(BoundingVolumeHierarchy, Insert_MirloRandomRays) {
    CS170::Utils::srand(5, 5);
    std::vector<CS350::CS350PrimitiveData> allPrimitives;
    std::vector<CS350::CS350SceneObject>   objects;
    std::vector<CS350::Aabb>               worldBvs;
    LoadPrimitivesAndScene(allPrimitives, objects, worldBvs, cSceneNormal);
    auto bvhObjects = CreateObjects(worldBvs);
    shuffle(bvhObjects);

    Bvh bvh;
    bvh.Insert(bvhObjects.begin(), bvhObjects.end(), cInsertConfig);
    PrintDebugInformation(bvh);
    AssertProperNodes(bvh);
    AssertAllAccountedFor(bvh, bvhObjects);
    TestSceneRandomRays(bvhObjects, bvh, 100, true);
}
