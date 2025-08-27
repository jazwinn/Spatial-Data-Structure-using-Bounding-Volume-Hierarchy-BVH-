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
 *  Declaration for DemoScene class.
 * @copyright
 *  Copyright (C) 2025 DigiPen Institute of Technology.
 */

#ifndef DEMOSCENE_HPP
#define DEMOSCENE_HPP

#include "bvh.hpp"
#include "debug_renderer.hpp"
#include "camera.hpp"
#include <vector>
#include <memory>

namespace CS350 {

    class DemoScene {
      public:
      private:
        struct Object;
        using BvhObject = Bvh<Object*>;
        using BvhNode   = BvhObject::Node;

        // Scene objects
        struct Object {
            unsigned    id; // Object identification
            CS350::Aabb bv; // Bounding volume of the object
            int         meshIndex;
            mat4        m2w;

            // Bvh information
            struct
            {
                Object*  next = nullptr; // Next object in the Bvh node
                Object*  prev = nullptr; // Previous object in the Bvh node
                BvhNode* node = nullptr; // The node it belongs to
            } bvhInfo;
        };

        struct
        {
            bool                        frustumCulling         = true;
            bool                        usingBvh               = true;
            bool                        debugDrawAllOutline    = false;
            bool                        debugDrawInsideOutline = false;
            bool                        debugDrawNodes         = false;
            bool                        debugBvh               = true;
            bool                        debugDrawNodeObjects   = false;
            bool                        listObjects            = false;
            BvhNode const*              debugNode              = nullptr;
            bool                        debugPreviewLeftNode   = false;
            bool                        debugPreviewRightNode  = false;
            unsigned                    drawCalls              = 0;
            BvhNode const*              hoverNode              = nullptr;
            std::vector<BvhNode const*> nodeStack;

            bool                        ray_debug = false;
            std::vector<BvhNode const*> ray_intersected_nodes;
            std::vector<unsigned>       ray_all_intersected_objects;
            Object*                     ray_closest_object = {};
            CS350::Ray                  ray                = {};
            bool                        ray_only_closest   = false;

            bool                        viewFustrumCamera  = false;
            bool                        debugDrawFustrum   = true;

            CS350::BvhBuildConfig config;

            void Clear() {
                debugNode = nullptr;
                hoverNode = nullptr;
                nodeStack.clear();
                ray_intersected_nodes.clear();
                ray_all_intersected_objects.clear();
                ray_closest_object = {};
            }
        } mOptions;

        Camera        mCamera;
        Camera        mAuxCamera;

        Camera*       mMainCamera;

        DebugRenderer mDebug;
        BvhObject     mBvh;

        std::vector<std::shared_ptr<Primitive>> mPrimitives;
        std::vector<std::shared_ptr<Object>>    mObjects;
        std::vector<Aabb>                       mModelBvs;
        std::shared_ptr<Shader>                 mShader;

      public:

        /**
        * @brief
		*  Default Constructor of Demoscene class.
        *  Loads all assets and shader
        */
        DemoScene();

		/**
		 * @brief
		 *  Destructor of DemoScene class.
		 *  Cleans up the Bvh and other resources
		 */
        ~DemoScene();

		/**
		 * @brief
		 *  Updates the scene, computes camera matrices and reset 
		 *  stats for the next frame.
		 */
        void Update();

        /**
         * @brief
		 *  Renders the scene.
         */
        void PassDebug();
        

        Camera const& camera() const { return mCamera; }
        Camera&       camera() { return mCamera; }

        Camera const& AuxCamera() const { return mAuxCamera; }
        Camera& AuxCamera() { return mAuxCamera; }

        bool AuxCameraMain() { return mOptions.viewFustrumCamera; };
        // Debug

        auto& debug() { return mDebug; }

        /**
         * @brief
		 *  Draws Imgui options for the scene.
		 * @param dt
         *  delta time
         */
        void  ImguiOptions(float dt);

        /**
         * @brief
		 *  Shows Bvh node information in Imgui.
         * @param node
         *  Root of the Bvh
         */
        void  ImguiBvhNode(BvhNode const* node);

        /**
         * @brief
		 *  Centers the camera on a Bvh node.
         * @param n
         *  Node the cameara centers on
         */
        void  CenterCamera(BvhNode const* n);

        /**
         * @brief
		 *  Draw Aabb of all objects in the Bvh node.
         * @param node
		 *  Root of the Bvh node
		 * @param color
         *  Color of the Aabb box
         */
        void  DebugDrawObjects(BvhNode const* node, vec4 color);

        /**
         * @brief
         *  Test the ray against the Bvh
         */
        void  perform_ray_query();

      private:
        /**
        * @brief
        *  Loads assets from the asset folder into the scene
        */
        void LoadScene();
    };
}

#endif // DEMOSCENE_HPP
