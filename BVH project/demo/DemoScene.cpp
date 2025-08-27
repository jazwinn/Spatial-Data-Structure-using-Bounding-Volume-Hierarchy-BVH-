/**
 * @file
 *  DemoScene.cpp
 * @author
 *  Eder Beldad (eder.b1@digipen.edu)
 * @co-author
 *  Jaz Winn Ng, 670001224, jazwinn.ng@digipen.edu
 * @date
 *  2025/06/25
 * @brief
 *  Definition for scene object that reads scene file , 
 *  loads the assets, draw debug shapes, and draw the scene.
 * @copyright
 *  Copyright (C) 2025 DigiPen Institute of Technology.
 */

#include "DemoScene.hpp"
#include "cs350_loader.hpp"
#include "shader.hpp"
#include "shapes.hpp"
#include "primitive.hpp"
#include "utils.hpp"
#include "stats.hpp"
#include "ImGui.hpp"

#include <filesystem>
#include <fmt/format.h>
#include <imgui.h>
#include <memory>
#include <vector>
#include <random>

namespace {
    constexpr int cUniformM2w   = 0;
    constexpr int cUniformV     = 1;
    constexpr int cUniformP     = 2;
    constexpr int cUniformColor = 3;

    std::shared_ptr<CS350::Primitive> PrimitiveFromCS350(CS350::CS350PrimitiveData const& data) {
        // Create primitive
        std::vector<float> vbo;
        vbo.reserve(data.positions.size() * 3);
        for (auto pos : data.positions) {
            vbo.push_back(pos.x);
            vbo.push_back(pos.y);
            vbo.push_back(pos.z);
        }
        return std::make_shared<CS350::Primitive>(vbo.data(), vbo.size() * sizeof(float));
    }
}

namespace {
    constexpr char const* cAssetPath     = "assets/cs350/gam400s20-mirlo/mirlo_{}.cs350_binary"; // To be used with fmt
    constexpr char const* cSceneNormal   = "assets/cs350/gam400s20-mirlo/scene.txt";
    constexpr char const* cSceneBottomUp = "assets/cs350/gam400s20-mirlo/scene-small.txt";

    const CS350::BvhBuildConfig cTopDownConfig = {
        std::numeric_limits<unsigned>::max(), // max_depth
        50,                                   // min_objects
        0.0f
    };

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

        // Load the scene
        objects = CS350::LoadCS350Scene(sceneFile);

        // Process bvs
        worldBvs.clear();
        worldBvs.reserve(objects.size());
        for (auto& object : objects) {
            auto const& primitive     = allPrimitives.at(static_cast<size_t>(object.primitiveIndex)); // Retrieve objects model
            auto        primitiveAabb = CS350::Aabb(primitive.bvMin, primitive.bvMax);                // See it's aabb
            auto        objectAabb    = primitiveAabb.transform(object.m2w);                          // Transform it to world
            worldBvs.push_back(objectAabb);
        }
    }
}
namespace CS350 {

    DemoScene::DemoScene() {
        mShader = std::make_shared<Shader>(LoadFile("assets/shaders/color.vert").c_str(),
                                           LoadFile("assets/shaders/color.frag").c_str());
        LoadScene();
        mMainCamera = &mCamera;
    }

    DemoScene::~DemoScene() {
        mBvh.~Bvh();
    }

    void DemoScene::Update() {
        mOptions.drawCalls              = 0;
        Stats::Instance().frustumVsAabb = 0;


		//Do this to prevent crash when applicaation is minimized
        if (mCamera.display_h > 0.f || mAuxCamera.display_h > 0.f) {

            if (mOptions.viewFustrumCamera) {
                mAuxCamera.compute_matrices();
            }
            else {
                mCamera.compute_matrices();
            }
           
            
        }
        
    }

    void DemoScene::LoadScene() {
        // Clear
        mObjects.clear();
        mPrimitives.clear();
        std::vector<Aabb> bvs;

        // Load
        std::vector<CS350PrimitiveData> primitives;
        std::vector<CS350SceneObject>   sceneObjects;
        LoadPrimitivesAndScene(primitives, sceneObjects, bvs, cSceneNormal);

        // Primitives
        for (auto const& primitiveData : primitives) {
            mPrimitives.push_back(::PrimitiveFromCS350(primitiveData));
            mModelBvs.push_back({ primitiveData.bvMin, primitiveData.bvMax });
        }

        // Objects
        std::vector<Object*> objPtrs;
        for (auto const& obj : sceneObjects) {
            auto gameObj       = std::make_shared<Object>();
            gameObj->bv        = mModelBvs.at(obj.primitiveIndex).transform(obj.m2w);
            gameObj->meshIndex = obj.primitiveIndex;
            gameObj->m2w       = obj.m2w;
            gameObj->id        = static_cast<int>(objPtrs.size());
            mObjects.push_back(gameObj);
            objPtrs.push_back(gameObj.get());
        }

        // BVH
        mBvh.Clear();
        mBvh.BuildTopDown(objPtrs.begin(), objPtrs.end(), mOptions.config);
    }

    void DemoScene::PassDebug() {
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);

        Frustum frustum(mAuxCamera.vp);
        std::vector<unsigned int> visible;

        { // Render shapes
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            mShader->use();
            set_uniform(cUniformV, mMainCamera->v);
            set_uniform(cUniformP, mMainCamera->p);

            if (!mOptions.frustumCulling) {
                // Draw all
                for (auto const& obj : mObjects) {
                    set_uniform(cUniformM2w, obj->m2w);
                    set_uniform(cUniformColor, vec4(1, 1, 1, 0.1f));
                    mPrimitives.at(obj->meshIndex)->draw(GL_TRIANGLES);
                    mOptions.drawCalls++;
                }
            } else {
                if (!mOptions.usingBvh) {
                    // Frustum vs all
                    for (auto const& obj : mObjects) {
                        if (frustum.classify(obj->bv) != eOUTSIDE) {
                            set_uniform(cUniformM2w, obj->m2w);
                            set_uniform(cUniformColor, vec4(1, 1, 1, 0.1f));
                            mPrimitives.at(obj->meshIndex)->draw(GL_TRIANGLES);
                            mOptions.drawCalls++;
                        }
                    }
                } else {
                    // Frustum vs Bvh
                    visible = mBvh.Query(frustum);
                    for (auto objIdx : visible) {
                        auto obj = mObjects.at(objIdx);
                        set_uniform(cUniformM2w, obj->m2w);
                        set_uniform(cUniformColor, vec4(1, 1, 1, 0.1f));
                        mPrimitives.at(obj->meshIndex)->draw(GL_TRIANGLES);
                        mOptions.drawCalls++;
                    }
                }
            }

            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        }

        // All
        mDebug.draw_begin();
        if (mOptions.debugDrawAllOutline) {
            for (auto& obj : mObjects) {
                mDebug.draw_aabb(mMainCamera->vp, obj->bv.get_center(), obj->bv.get_extents(), { 1, 1, 1, 0.1 });
            }
        }

         // Inside only
         if (mOptions.debugDrawInsideOutline) {
             for (auto index : visible) {
                 auto& obj = mObjects.at(index);
                 mDebug.draw_aabb(mMainCamera->vp, obj->bv.get_center(), obj->bv.get_extents(), { 1, 1, 1, 0.1 });
             }
         }


        if (mOptions.debugBvh) {
            // Debug nodes
            glDisable(GL_DEPTH_TEST);
            glDepthMask(GL_FALSE);
            glDisable(GL_CULL_FACE);

            if (mOptions.debugNode && mOptions.debugDrawNodeObjects) {
                DebugDrawObjects(mOptions.debugNode, { 1, 0, 0, 0.1 });
            }

            if (mOptions.hoverNode) {
                mDebug.draw_aabb(mMainCamera->vp, mOptions.hoverNode->bv.get_center(), mOptions.hoverNode->bv.get_extents(), { 0.8, 0.8, 0.8, 0.1 });
                DebugDrawObjects(mOptions.hoverNode, { 0.8, 0.8, 0.8, 0.1 });
            }
            if (mOptions.debugNode) {
                auto node = mOptions.debugNode;
                // mDebug.DrawAabb(mOptions.debugNode->bv, { 0.2, 0.8, 0.3, 0.1 });
                if (!node->IsLeaf()) {
                    if (mOptions.debugPreviewLeftNode)
                        mDebug.draw_aabb(mMainCamera->vp, node->children[0]->bv.get_center(), node->children[0]->bv.get_extents(), { 0.8, 0.2, 0.3, 0.1 });
                    if (mOptions.debugPreviewRightNode)
                        mDebug.draw_aabb(mMainCamera->vp, node->children[1]->bv.get_center(), node->children[1]->bv.get_extents(), { 0.2, 0.8, 0.3, 0.1 });
                }
            }
        }

        if (mOptions.ray_debug) { // Ray
            if (!ImGui::GetIO().WantCaptureMouse && ImGui::IsMouseDown(0)) {
                mOptions.ray = mMainCamera->cursor_ray();
            }
            perform_ray_query();
            mDebug.draw_segment(mMainCamera->vp, mOptions.ray.start, mOptions.ray.at(1000.0f), { 1, 0, 1, 1 });

            // Draw intersection
            if (mOptions.ray_closest_object) {
                mDebug.draw_aabb(mMainCamera->vp, mOptions.ray_closest_object->bv.get_center(), mOptions.ray_closest_object->bv.get_extents(), { 1, 0, 1, 0.5f });

                // Draw all intersected objects
                for (auto const& obj_index : mOptions.ray_all_intersected_objects) {
                    mDebug.draw_aabb(mMainCamera->vp, mObjects.at(obj_index)->bv.get_center(), mObjects.at(obj_index)->bv.get_extents(), { 0.5f, 0.6f, 0.6f, 0.2f });
                }
            }
        }

        if (mOptions.debugDrawFustrum && !mOptions.viewFustrumCamera) {
			// Draw frustum
            mDebug.draw_frustum(mMainCamera->vp, mAuxCamera.vp, { 1.f, 0.75f, 0.8f, 0.1f });
			
        }

        glDepthMask(GL_TRUE); // Restore
    }

    void DemoScene::ImguiOptions(float dt) {
        if (ImGui::CollapsingHeader("BVH", ImGuiTreeNodeFlags_::ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Text("FPS (dt): %.02f (%.04fms)", 1.0f / dt, dt);
            ImGui::Text("Frustum Vs. Aabb: %lu", CS350::Stats::Instance().frustumVsAabb);
            ImGui::Text("Ray Vs. Aabb: %lu", CS350::Stats::Instance().rayVsAabb);
            ImGui::Text("ray_intersected_nodes: %lu", mOptions.ray_intersected_nodes.size());
            ImGui::Text("ray_all_intersected_objects: %lu", mOptions.ray_all_intersected_objects.size());
            ImGui::Text("Draw calls: %u", mOptions.drawCalls);
            ImGui::Checkbox("frustumCulling", &mOptions.frustumCulling);
            ImGui::Checkbox("usingBvh", &mOptions.usingBvh);
            ImGui::Checkbox("debugDrawAllOutline", &mOptions.debugDrawAllOutline);
            if (mOptions.usingBvh) {
                ImGui::Checkbox("debugDrawInsideOutline", &mOptions.debugDrawInsideOutline);
            }
            ImGui::Checkbox("debugBvh", &mOptions.debugBvh);
            ImGui::Checkbox("debugDrawNodes", &mOptions.debugDrawNodes);
            ImGui::Checkbox("ray_debug", &mOptions.ray_debug);
            ImGui::Checkbox("ray_only_closest", &mOptions.ray_only_closest);

            ImGui::DragFloat("Min volume", &mOptions.config.minVolume);
            int maxDepth = mOptions.config.maxDepth;
            ImGui::DragInt("Max depth", &maxDepth);
            mOptions.config.maxDepth = maxDepth;

            int minObjects = mOptions.config.minObjects;
            ImGui::DragInt("Min objects", &minObjects);
            mOptions.config.minObjects = minObjects;

            if (ImGui::Button("Build TopDown")) {
                std::vector<Object*> objPtrs;
                for (auto const& obj : mObjects) {
                    objPtrs.push_back(obj.get());
                }
                mBvh.Clear();
                mBvh.BuildTopDown(objPtrs.begin(), objPtrs.end(), mOptions.config);
                mOptions.Clear();
            }
            if (ImGui::Button("Build insert")) {
                std::vector<Object*> objPtrs;
                for (auto const& obj : mObjects) {
                    objPtrs.push_back(obj.get());
                }
                std::random_device rd;
                std::mt19937       g(rd());
                std::shuffle(objPtrs.begin(), objPtrs.end(), g);
                mBvh.Clear();
                mBvh.Insert(objPtrs.begin(), objPtrs.end(), mOptions.config);
                mOptions.Clear();
            }
            ImGui::SameLine();
            if (ImGui::Button("One")) {
                if (mBvh.objectCount() < mObjects.size()) {
                    std::vector<Object*> objPtrs;
                    for (auto const& obj : mObjects) {
                        objPtrs.push_back(obj.get());
                    }
                    std::mt19937 g(5);
                    std::shuffle(objPtrs.begin(), objPtrs.end(), g);
                    mBvh.Insert(objPtrs[mBvh.objectCount()], mOptions.config);
                    mOptions.Clear();
                }
            }

            if (ImGui::Button("Clear")) {
                mBvh.Clear();
                mOptions.Clear();
            }

            if (mOptions.debugNode && mOptions.debugBvh) {
                ImguiBvhNode(mOptions.debugNode);
                mOptions.hoverNode = nullptr;
                auto& n            = *mOptions.debugNode;
                if (!n.IsLeaf()) {
                    if (ImGui::Button("Go L")) {
                        mOptions.nodeStack.push_back(mOptions.debugNode);
                        mOptions.debugNode = n.children[0];
                    }
                    if (ImGui::IsItemHovered())
                        mOptions.hoverNode = n.children[0];
                    ImGui::SameLine();
                    ImGui::PushID("L");
                    ImGui::Checkbox("Preview", &mOptions.debugPreviewLeftNode);
                    ImGui::SameLine();
                    if (ImGui::Button("Center"))
                        CenterCamera(n.children[0]);
                    ImGui::PopID();

                    if (ImGui::Button("Go R")) {
                        mOptions.nodeStack.push_back(mOptions.debugNode);
                        mOptions.debugNode = n.children[1];
                    }
                    if (ImGui::IsItemHovered())
                        mOptions.hoverNode = n.children[1];
                    ImGui::SameLine();
                    ImGui::PushID("R");
                    ImGui::Checkbox("Preview", &mOptions.debugPreviewRightNode);
                    ImGui::SameLine();
                    if (ImGui::Button("Center"))
                        CenterCamera(n.children[1]);
                    ImGui::PopID();
                }
                // Go to parent
                if (!mOptions.nodeStack.empty()) {
                    if (ImGui::Button("Go Parent")) {
                        mOptions.debugNode = mOptions.nodeStack.back();
                        mOptions.nodeStack.pop_back();
                    }
                    if (ImGui::IsItemHovered() && !mOptions.nodeStack.empty())
                        mOptions.hoverNode = mOptions.nodeStack.back();
                }
            } else {
                mOptions.debugNode = mBvh.root();
                mOptions.nodeStack.clear();
            }
        }

        if (ImGui::CollapsingHeader("Camera", ImGuiTreeNodeFlags_::ImGuiTreeNodeFlags_DefaultOpen)) {

            if (ImGui::Checkbox("Switch to Fustrum Camera", &mOptions.viewFustrumCamera)) {
                mMainCamera = (mMainCamera == &mCamera) ? &mAuxCamera : &mCamera;
            }
            ImGui::Checkbox("Draw Fustrum", &mOptions.debugDrawFustrum);
        }

    }

    void DemoScene::ImguiBvhNode(BvhNode const* node) {
        ImGui::PushID(node);
        ImGui::Text("%s", fmt::format("Node:  {}", (void const*)node).c_str());
        ImGui::Text("%s", fmt::format("Depth: {}", node->Depth()).c_str());
        ImGui::Text("%s", fmt::format("Size:  {}", node->Size()).c_str());
        ImGui::Text("%s", fmt::format("Min:   {}", node->bv.min).c_str());
        ImGui::Text("%s", fmt::format("Max:   {}", node->bv.max).c_str());
        ImGui::Checkbox("Draw node objects", &mOptions.debugDrawNodeObjects);
        ImGui::Checkbox("List objects", &mOptions.listObjects);

        if (mOptions.listObjects) {
            auto recordFn = [this](Object const* object) {
                // Debug draw boxes on top of the leaf objects
                auto& bv = object->bv;
                ImGui::Selectable(fmt::format("Object: {}\n\tmin: {}\n\tmax: {}\n", object->id, bv.min, bv.max).c_str());

                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                if (ImGui::IsItemHovered()) {
                    mDebug.draw_primitive(mCamera.vp, object->m2w, mPrimitives.at(object->meshIndex).get(), { 1, 0, 1, 1 });
                }
            };
            if (ImGui::BeginListBox("Objects")) {
                node->TraverseLevelOrderObjects(recordFn);
                ImGui::EndListBox();
            }
        }

        ImGui::PopID();
    }

    void DemoScene::CenterCamera(BvhNode const* n) {
        auto bv                 = n->bv;
        mCamera.camera_position = (bv.max + vec3(50, 50, 50));
        mCamera.camera_dir      = bv.get_center() - mCamera.camera_position;
    }

    void DemoScene::DebugDrawObjects(BvhNode const* node, vec4 color) {
        auto drawFn = [this, &color](Object const* obj) {
            // Debug draw boxes on top of the leaf objects
            mDebug.draw_aabb(mCamera.vp, obj->bv.get_center(), obj->bv.get_extents(), color);
        };
        node->TraverseLevelOrderObjects(drawFn);
    }

    void DemoScene::perform_ray_query() {
        Stats::Instance().rayVsAabb = 0;
        auto intersection           = mBvh.QueryDebug(mOptions.ray, mOptions.ray_only_closest, mOptions.ray_all_intersected_objects, mOptions.ray_intersected_nodes);
        mOptions.ray_closest_object = {};
        if (intersection) {
            mOptions.ray_closest_object = mObjects.at(*intersection).get();
        }
    }
}