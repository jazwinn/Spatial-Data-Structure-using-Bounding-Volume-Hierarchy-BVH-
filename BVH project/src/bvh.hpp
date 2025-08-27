/**
 * @file
 *  bvh.inl
 * @author
 *  Jaz Winn Ng, 670001224, jazwinn.ng@digipen.edu
 * @date
 *  2025/06/24
 * @brief
 *  Declaration of bounding volume heirachy (BVH) functions
 * @copyright
 *  Copyright (C) 2025 DigiPen Institute of Technology.
 */


#ifndef __BVH_HPP__
#define __BVH_HPP__

#include "shapes.hpp"
#include "logging.hpp" // fmt

#include <iomanip>       // Format manipulators
#include <unordered_map> //
#include <limits>
#include <optional>
#include <vector>
#include <queue>
#include <ostream>
#include <functional> // Debug


namespace CS350 {

    /**
     * @brief
     *  Some rules for Bvh construction. Not all rules apply to all methods.
     *  Feel free to add more rules.
     */
    struct BvhBuildConfig {
        unsigned maxDepth   = std::numeric_limits<unsigned>::max();
        unsigned minObjects = 10; // Nodes should have more than this amount of objects to be split
        float    minVolume  = 0; // Nodes with smaller volume than this will not be splitted
    };



    /**
     * @brief
     *  Bounding Volume Hierarchy for type T
     *  Requires the following members of T.
     *      Aabb T::bv
     *      unsigned T::id
     *      T T::bvhInfo.next
     *      T T::bvhInfo.prev
     *      Node* T::bvhInfo.node
     */
    template <typename T>
    class Bvh {
      public: // Public for testing reasons
        struct Node {
			Node(Aabb boundingVolume) :
				bv(boundingVolume),
				firstObject(nullptr),
				lastObject(nullptr) {
				children[0] = nullptr;
				children[1] = nullptr;
			}

            Aabb  bv;          // Node bounding volume
            Node* children[2]; // Both children
            T     firstObject; //
            T     lastObject;  //

            /**
             * @brief
			 *  Adds Object to the node
             * @param object
			 *  Object to be added to the node
             */
            void                        AddObject(T object);

            /**
             * @brief
             *  Measure node depth 
             * @return
			 *  Depth of the node
             */
            int                         Depth() const; // Branches to leaves, gives to depth position of the node

            /**
             * @brief
			 *  Counts number of nodes under this node
             * @return
             *  Number of nodes
             */
            int                         Size() const;  // Amount of nodes

			/**
			* @brief
			*  Checks if the node is a leaf node
			* @return
			*  True if the node is a leaf, false otherwise
			*/
            bool                        IsLeaf() const;

			/**
			 * @brief
			 *  Counts number of objects in this node
			 * @return
			 *  Number of objects in this node
			 */
            unsigned                    ObjectCount() const;  
            
            // Amount of objects in current node (not children)

            /**
             * @brief
			 *  Traverse each node and apply `func` to each node
             * @param func
			 *  function to be applied to each node
             */
            template <typename Fn> void TraverseLevelOrder(Fn func) const;        // Traverses a tree from an starting point. `func` is of type void(Node const* node);

            /**
             * @brief
             *  Traverse each node and apply `func` to each object
             * @param func
			 *  function to be applied to each object
             */
            template <typename Fn> void TraverseLevelOrderObjects(Fn func) const; // Traverses a tree from an starting point. `func` is of type void(T const& object);


        };

      private:
        Node*    mRoot;
        unsigned mObjectCount;

      public:
        /**
        * @brief
		*  Constructor of the Bvh
        */
        Bvh();

        /**
        * @brief
        *  Destructor of the Bvh
        */
        ~Bvh();
        Bvh(Bvh const&)            = delete;
        Bvh& operator=(Bvh const&) = delete;

        /**
         * @brief
		 *  Uses the given range to build a top-down Bvh tree
         * @param begin
		 *  The beginning of the range
         * @param end
         *  The end of the range
         * @param config
		 *  Configuration for the Bvh build
         * @param parentNode
		 *  Parent node to attach the new nodes to. If nullptr, the new nodes will be the root of the Bvh
         */
        template <typename IT> void BuildTopDown(IT begin, IT end, BvhBuildConfig const& config, Node* parentNode = nullptr);

        template <typename IT> void BuildBottomUp(IT begin, IT end, BvhBuildConfig const& config);

		/**
		 * @brief
		 *  Inserts a range of objects into the Bvh using the incremental approach
		 * @param begin
		 *  The beginning of the range
		 * @param end
		 *  The end of the range
		 * @param config
		 *  Configuration for the Bvh build
		 */
        template <typename IT> void Insert(IT begin, IT end, BvhBuildConfig const& config);

		/**
		 * @brief
		 *  Inserts an object into the Bvh using the incremental approach
		 * @param object
		 *  The object to be inserted
		 * @param config
		 *  Configuration for the Bvh build
		 */
        void                        Insert(T object, BvhBuildConfig const& config);

		/**
		 * @brief
		 *  Clears the Bvh tree and resets the object count
		 */
        void                        Clear();

		/**
		 * @brief
		 *  Checks if the Bvh is empty
		 * @return
		 *  True if the Bvh is empty, false otherwise
		 */
        bool                        Empty() const;

		/**
		 * @brief
		 *  Returns the depth of the Bvh tree
		 * @return
		 *  Depth of the Bvh tree
		 */
        int                         Depth() const; //gives the depth of the bvh tree

		/**
		 * @brief
		 *  Returns the size of the Bvh tree
		 * @return
		 *  Size of the Bvh tree
		 */
        int                         Size() const;

        /**
         * @brief
		 *  Access the root of the Bvh tree
         * @return
		 *  Root node of the Bvh tree
         */
        Node const*                 root() const;

		/**
		 * @brief
		 *  Peforms frustum vs Bvh 
		 * @return
		 *  Vector of unsigned integers representing the object ids tha are visible in the frustum
		 */
        std::vector<unsigned>       Query(Frustum const& frustum) const;

		/**
		 * @brief
		 *  Peforms ray vs Bvh query
		 * @param ray
		 *  Ray to be tested against the Bvh
		 * @param closest_only
		 *  If true, only the closest intersection will be returned
		 * @param allIntersectedObjects
		 *  Vector to store all intersected objects
		 * @param debug_tested_nodes
		 *  Vector to store all nodes that were tested during the query
		 * @return
		 *  Optional unsigned representing the id of the closest intersected object
		 */
        std::optional<unsigned>     QueryDebug(Ray const& ray, bool closest_only, std::vector<unsigned>& allIntersectedObjects, std::vector<Node const*>& debug_tested_nodes) const;

        // Debug functions
		/**
		 * @brief
		 *  Prints the Bvh file format to render a graph
		 * @param os
		 *  Output stream to print the information to
		 */
        void                        DumpGraph(std::ostream& os) const;               // Prints a graph script

		/**
		 * @brief
		 *  Prints the Bvh tree information in a readable way
		 * @param os
		 *  Output stream to print the information to
		 */
        void                        DumpInfo(std::ostream& os) const;                // Prints information of the tree

		/**
		 * @brief
		 *  Prints information of a particular node in a readable way
		 * @param os
		 *  Output stream to print the information to
		 * @param n
		 *  Node to print information about
		 */
        void                        DumpInfo(std::ostream& os, Node const* n) const; // Prints information of a particular node

		/**
		 * @brief
		 *  Traverses the Bvh tree in level order and applies `func` to each node
		 * @param func
		 *  Function to be applied to each node
		 */
        template <typename Fn> void TraverseLevelOrder(Fn func) const;               // Traverses the tree. `func` is of type void(Node const* node);

		/**
		 * @brief
		 *  Traverses the Bvh tree in level order and applies `func` to each object
		 * @param func
		 *  Function to be applied to each object
		 */
        template <typename Fn> void TraverseLevelOrderObjects(Fn func) const;        // Traverses the tree. `func` is of type void(T const& node);

		/**
		 * @brief
		 *  Returns the number of objects in the Bvh
		 * @return
		 *  Number of objects in the Bvh
		 */
        unsigned                    objectCount() const { return mObjectCount; }

      private:
        /**
         * @brief
		 *  Helper structure to store costs of a node
         *  Used in the insert function
         */
        struct NodeCosts {
            /**
             * @brief
			 *  Default Constructor of NodeCosts
             */
            NodeCosts() = default;

            /**
             * @brief
             *  Constructor of NodeCosts, calculate the cost of traversal and cost
			 *  to generate a new parent node
             * @param _node
			 *  node of the Bvh
             * @param object
			 *  object to be added to the node
             * @param costToNode
			 *  cost to expand the nodes from root to include the object
             * @param _level
			 *  level of the node from root
             */
            NodeCosts(Node* _node, T object, float costToNode, unsigned int _level);

            Node* node = nullptr;
            float rootToNewParentCost;
            float rootToNodeCost;
            unsigned int level;

            Aabb newAabb;
            float newGeometrics;
            float newGeometricsChange;
        };
    };

    /**
     * @brief
     *  Prints information about the BVH in a readable way
     */
    template <typename T>
    void Bvh<T>::DumpInfo(std::ostream& os) const {
        os << std::fixed;
        os << "GENERAL INFO: \n"
           << std::setw(20) << "Depth: " << Depth() << "\n"
           << std::setw(20) << "Size: " << Size() << "\n"
           << std::endl;
        TraverseLevelOrder([&](Bvh<T>::Node const* n) { DumpInfo(os, n); });
    }

    /**
     * @brief
     *  Shows information about the node in a readable way
     */
    template <typename T>
    void Bvh<T>::DumpInfo(std::ostream& os, Node const* n) const {
        if (!n) {
            return;
        }
            

        // Node info
        auto const& bv = n->bv;
        os << "NODE [" << n << "] \n"
           << std::setw(20) << "BV: " << bv << "\n"
           << std::setw(20) << "Volume: " << bv.volume() << "\n"
           << std::setw(20) << "Surface area: " << bv.surface_area() << "\n";

        // Subchildren
        if (n->IsLeaf()) {
            os << std::setw(20) << "Children: "
               << "NONE"
               << "\n"
               << std::setw(20) << "Objects count: " << n->ObjectCount() << "\n";

        } else {
            os << std::setw(20) << "Children: "
               << "\n"
               << std::setw(25) << "NODE [" << n->children[0] << "] \n"
               << std::setw(30) << "Depth: " << n->children[0]->Depth() << "\n"
               << std::setw(30) << "Size: " << n->children[0]->Size() << "\n"
               << std::setw(25) << "NODE [" << n->children[1] << "] \n"
               << std::setw(30) << "Depth: " << n->children[1]->Depth() << "\n"
               << std::setw(30) << "Size: " << n->children[1]->Size() << "\n";
        }
        os << std::endl;
    }

    template <typename T>
    void Bvh<T>::DumpGraph(std::ostream& os) const {
        os << "digraph bvh {\n";
        os << "\tnode[group=\"\", shape=none, style=\"rounded,filled\", fontcolor=\"#101010\"]\n";
        // Create all nodes
        int                                  lastNodeId = 0;
        std::unordered_map<Node const*, int> nodeIds;
        TraverseLevelOrder([&](Bvh<T>::Node const* node) {
            nodeIds[node]     = lastNodeId;
            std::string label = fmt::format("[{:.02f},{:.02f},{:.02f}]\\n[{:.02f},{:.02f},{:.02f}]\\nSA: {:.02f}\\nVOL: {:.02f}",
                                            node->bv.min.x,
                                            node->bv.min.y,
                                            node->bv.min.z,
                                            node->bv.max.x,
                                            node->bv.max.y,
                                            node->bv.max.z,
                                            node->bv.surface_area(),
                                            node->bv.volume());
            if (node->IsLeaf()) {
                label += fmt::format("\\n{} objects", node->ObjectCount());
            }
            os << fmt::format("\tNODE{}[label=\"{}\"];\n", lastNodeId, label);
            lastNodeId++;
        });

        // Create all links
        TraverseLevelOrder([&](Bvh<T>::Node const* node) {
            auto nodeId = nodeIds.at(node);
            if (!node->IsLeaf()) {
                auto nodeLeft = nodeIds.at(node->children[0]);
                os << fmt::format("\tNODE{} -> NODE{};\n", nodeId, nodeLeft);
                auto nodeRight = nodeIds.at(node->children[1]);
                os << fmt::format("\tNODE{} -> NODE{};\n", nodeId, nodeRight);
            }
        });
        os << "}";
    }

}

#include "bvh.inl"

#endif // __BVH_HPP__
