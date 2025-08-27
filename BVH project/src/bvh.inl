/**
 * @file
 *  bvh.inl
 * @author
 *  Jaz Winn Ng, 670001224, jazwinn.ng@digipen.edu
 * @date
 *  2025/06/24
 * @brief
 *  Definition of bounding volume heirachy (BVH) functions
 * @copyright
 *  Copyright (C) 2025 DigiPen Institute of Technology.
 */

#ifndef BVH_INL
#define BVH_INL

#include "bvh.hpp"
#include "shapes.hpp"
#include "logging.hpp"

#include <array>
#include <iomanip>
#include <cstring>
#include <limits>
#include <type_traits>
#include <unordered_map>
#include <algorithm>
#include <stack>



namespace CS350 {

    constexpr float cEpsilon3 = 1e-3f;

    template <typename T>
    void Bvh<T>::Node::AddObject(T object) {
        //check if node already inside 
        if (object->bvhInfo.node == this) {
            return;
        }

        // If object is already in another node, extract object out of that node
        if (object->bvhInfo.node != nullptr) {
            if (object->bvhInfo.prev != nullptr) {
                object->bvhInfo.prev->bvhInfo.next = object->bvhInfo.next;
                object->bvhInfo.prev = nullptr;
            }

            if (object->bvhInfo.next != nullptr) {
                object->bvhInfo.next->bvhInfo.prev = object->bvhInfo.prev;
                object->bvhInfo.next = nullptr;
            }
        }

        // if object is tge first object inserted into the node
        if (firstObject == nullptr) {
            firstObject = object;
        }
        
        //set new object prev to be last object
        object->bvhInfo.prev = lastObject;
        object->bvhInfo.next = nullptr;
        object->bvhInfo.node = this;
        
        //set last object next to be new object
        //If statement as lastObject is null at the start
        if (lastObject != nullptr) {
            lastObject->bvhInfo.next = object;
        }
        

        //set last object to be the new object
        lastObject = object;


        
    }

    template <typename T>
    int Bvh<T>::Node::Depth() const { 

		//recurse down the tree until we reach a leaf node
        if (IsLeaf()) {
            return 0;
        }

        int child0Depth = children[0]? children[0]->Depth() : 0;
        
        int child1Depth = children[1]?children[1]->Depth() : 0;


        return 1 + glm::max(child0Depth, child1Depth);
    }

    template <typename T>
    int Bvh<T>::Node::Size() const {

		//recurse down the tree until we reach a leaf node
        if ( IsLeaf()) {
            return 1;
        }
        //include current node
        int count = 1;

        count += children[0]->Size();
        count += children[1]->Size();

        return count;
    }
    
    template <typename T>
    bool Bvh<T>::Node::IsLeaf() const {
        //If children 0 is null, children 1 will also be null
		//Since children 0 is always created before children 1
        return children[0] ? false : true;
    }

    template <typename T>
    unsigned Bvh<T>::Node::ObjectCount() const {
        unsigned int count = 0;
        
        T object = firstObject;
        while (object != nullptr) {
            count++;
            object = object->bvhInfo.next;
        }
        return count;
    }

    template <typename T>
    template <typename Fn> 
    void Bvh<T>::Node::TraverseLevelOrder(Fn func) const {

        std::queue<const Node*> queue;
        queue.push(this);


        while (!queue.empty()) {
            const Node* node = queue.front();
            queue.pop();

            if (node->children[0] != nullptr) {
                queue.push(node->children[0]);
            }
            if (node->children[1] != nullptr) {
                queue.push(node->children[1]);
            }

            func(node);
        }


    }

    template <typename T>
    template <typename Fn>
    void Bvh<T>::Node::TraverseLevelOrderObjects(Fn func) const {


        std::queue<const Node*> queue;
        queue.push(this);
        

        while (!queue.empty()) {
            const Node* node = queue.front();
            queue.pop();

            if (node->children[0] != nullptr) {
                queue.push(node->children[0]);
            }
            if (node->children[1] != nullptr) {
                queue.push(node->children[1]);
            }

            if (!node->IsLeaf()) {
                continue;
            }

            T object = node->firstObject;
            while (object != nullptr) {

                //save next in case function modify object
                T next = object->bvhInfo.next;
                func(object);
                object = next;

            }
        }
    }


    template <typename T>
    Bvh<T>::Bvh() :
        mRoot{nullptr},
        mObjectCount{0}
    {}

    template <typename T>
    Bvh<T>::~Bvh() {
        if (mRoot != nullptr) {

            Clear();
        }

    }

    template <typename T>
    template <typename IT> 
    void Bvh<T>::BuildTopDown(IT begin, IT end, BvhBuildConfig const& config, Node* parentNode) {

        //check if iterator is valid
        if (begin == end || *begin == nullptr) {
            return;
        }

		std::vector<T> objects;

        

		//retreive the min and max to build the bounding volume
        unsigned int countObject{1};
        vec3 maxPoint{ (*begin)->bv.max };
        vec3 minPoint{ (*begin)->bv.min };
		objects.push_back(*begin);
        for (auto it = begin + 1; it != end; it++) {
            maxPoint = glm::max(maxPoint, (*it)->bv.max);
            minPoint = glm::min(minPoint, (*it)->bv.min);

            
			objects.push_back(*it);

            ++countObject;
        }


        Node* workingNode = new Node(Aabb(minPoint, maxPoint));

        //check if object is already inside node
        if (parentNode != nullptr) {

            // find children that has nullptr

            if (parentNode->children[0] == nullptr) {
                parentNode->children[0] = workingNode;
            }
            else if (parentNode->children[1] == nullptr) {
                parentNode->children[1] = workingNode;
            }
            else {
                delete workingNode;
                throw std::runtime_error("bvh.inl: node already have 2 children");
            }


        }
        else {
            mRoot = workingNode;
        }

        // if node is root, update object count
        if (workingNode == mRoot) {
            mObjectCount = countObject;
        }

        //stop if config condition met
		unsigned int currentDepth = static_cast<unsigned int>(mRoot->Depth());
        if ((countObject <= config.minObjects) ||
            (workingNode->bv.volume() <= config.minVolume) ||
            (currentDepth >= config.maxDepth)) {
            //clear first object and last object of previous node



            if ((*begin)->bvhInfo.node != nullptr) {
                (*begin)->bvhInfo.node->firstObject = nullptr;
                (*begin)->bvhInfo.node->lastObject = nullptr;
            }

            //add objects to node on
            for (auto it = begin; it != end; it++) {
                workingNode->AddObject(*it);

            }

            return;
        }



        //find the greatest axis
        int axis = workingNode->bv.longest_axis();

		std::sort(objects.begin(), objects.end(),[&](const T& lhs, const T& rhs) {
				return lhs->bv.get_center()[axis] < rhs->bv.get_center()[axis];
			});

		//split object in the mean
		int splitIndex = static_cast<int>(static_cast<float>(objects.size()) * 0.5f);

        std::vector<T> split1(objects.begin(), objects.begin() + splitIndex);
        std::vector<T> split2(objects.begin() + splitIndex, objects.end());
        //recurse
        BuildTopDown(split1.begin(), split1.end(), config, workingNode);
        BuildTopDown(split2.begin(), split2.end(), config, workingNode);


    }

    template <typename T>
    template <typename IT> 
    void Bvh<T>::BuildBottomUp(IT begin, IT end, BvhBuildConfig const& config) {
        
    }


    template <typename T>
    template <typename IT>
    void Bvh<T>::Insert(IT begin, IT end, BvhBuildConfig const& config) {
        for (auto it = begin; it != end; it++) {

            Insert(*it, config);

        }
        
    }


    template <typename T>
    void Bvh<T>::Insert(T object, BvhBuildConfig const& config) {

        ++mObjectCount;
        if (mRoot == nullptr) {
            mRoot = new Node(object->bv);
            mRoot->AddObject(object);

            return;
        }
        

        

        //node cost from root to leaf
        auto compareCheapest = [](const auto& lhs, const auto& rhs) {
            //highest level
            if (lhs.level != rhs.level) {
                return lhs.level < rhs.level;
            }
            //lowest geometric changed
            return lhs.newGeometricsChange > rhs.newGeometricsChange;

            };
        std::priority_queue<NodeCosts, std::vector<NodeCosts>, decltype(compareCheapest)> cheapestPathCost;

        cheapestPathCost.push(NodeCosts{ mRoot, object, 0.f, 0 });

        std::vector<NodeCosts> cheapestPath;

        //Store leafNode when leaf node is reached
        NodeCosts* leafNode = nullptr;

        //find the node with the cheapest rootToNewParentCost
       int smallestCostIndex = 0;

        int index = 0;
        while (!cheapestPathCost.empty()) {


            cheapestPath.push_back(cheapestPathCost.top());
            cheapestPathCost.pop();
            NodeCosts* nodeCost = &cheapestPath.back();

            //find new lowest cost
            if ((nodeCost->rootToNewParentCost <= (cheapestPath[smallestCostIndex].rootToNewParentCost + cEpsilon3))) {
                smallestCostIndex = index;
            }


            //if node is a leaf end loop
            if (nodeCost->node->IsLeaf()) {

                leafNode = nodeCost;
                break;
            }

            //push children
            cheapestPathCost.push(NodeCosts{nodeCost->node->children[0], object, nodeCost->rootToNodeCost, nodeCost->level + 1});
            cheapestPathCost.push(NodeCosts{nodeCost->node->children[1], object, nodeCost->rootToNodeCost, nodeCost->level + 1});


            ++index;
        }

        
        if (leafNode) {
           
            //add level (cost of traversal) to allow for a more balance tree
            if (leafNode->rootToNodeCost < cheapestPath[smallestCostIndex].rootToNewParentCost) {

                // Priority to check if leaf less than min object or leaf node hits max depth, insert object into leaf node
                if (leafNode->node->ObjectCount() < config.minObjects || leafNode->level >= config.maxDepth) {
                    for (auto& nodeCost : cheapestPath) {
                        nodeCost.node->bv = nodeCost.newAabb;
                    }

                    leafNode->node->AddObject(object);
                    return;
                }


                //leaf node is larger than min volume, and wants to continue to expand, create new node instead
                //Do this as some object may be larger than min volume
                if (leafNode->newAabb.volume() >= config.minVolume && leafNode->newGeometricsChange > 0.f) {
                    smallestCostIndex = static_cast<int>(cheapestPath.size()) - 1;
                }
                else{
                    for (auto& nodeCost : cheapestPath) {
                        nodeCost.node->bv = nodeCost.newAabb;
                    }

                    leafNode->node->AddObject(object);
                    return;
                }

                
            }

        }
        

        //create a leaf note

        // check if smallest cose node is root, aka index 0
         if (cheapestPath[smallestCostIndex].node == mRoot) {
            mRoot = new Node(cheapestPath[smallestCostIndex].newAabb);
            mRoot->children[0] = cheapestPath[smallestCostIndex].node;
            mRoot->children[1] = new Node(object->bv);
            mRoot->children[1]->AddObject(object);
            return;
        }

        //expand the size of all nodes except for smallesCost node
        for (int n{}; n < smallestCostIndex; n++) {
            cheapestPath[n].node->bv = cheapestPath[n].newAabb;
        }


        Node* parentNode = cheapestPath[smallestCostIndex - 1].node;

        //find which child the parent node is pointing to
        unsigned child = 0;
        if (parentNode->children[0] != cheapestPath[smallestCostIndex].node) { // if children address isnt child 0, is child 1
            child = 1;
        }

       
        parentNode->children[child] = new Node(cheapestPath[smallestCostIndex].newAabb);

        parentNode->children[child]->children[child] = cheapestPath[smallestCostIndex].node;
        parentNode->children[child]->children[child^1] = new Node(object->bv);
        parentNode->children[child]->children[child^1]->AddObject(object);

     
    }

    template <typename T>
    void Bvh<T>::Clear() {
        // clear all object prev, next, node
        if (mRoot == nullptr) {
            return;
        }


        auto lamdaClearObject = [](T object) {
            object->bvhInfo.next = nullptr;
            object->bvhInfo.prev = nullptr;
            object->bvhInfo.node = nullptr;
            };


        mRoot->TraverseLevelOrderObjects(lamdaClearObject);

        auto lamdaClearNode = [](const Node* node) {

            delete node;
        };

        mRoot->TraverseLevelOrder(lamdaClearNode);
        mRoot = nullptr;
        mObjectCount = 0;
    }

    template <typename T>
    bool Bvh<T>::Empty() const {

        return (mRoot == nullptr && mObjectCount == 0);
    }

    template <typename T>
    int Bvh<T>::Depth() const {
 
        if (mRoot == nullptr) {
            return -1;
        }

        return mRoot->Depth();
    }

    template <typename T>
    int Bvh<T>::Size() const {
        if (mRoot == nullptr) {
            return 0;
        }
        return mRoot->Size();
    }

    template <typename T>
    Bvh<T>::Node const* Bvh<T>::root() const {
        return this->mRoot;
    }


    template <typename T>
    std::vector<unsigned> Bvh<T>::Query(Frustum const& frustum) const {

        std::vector<unsigned> objectsIds;

		std::stack<Node*> stack;
		stack.push(mRoot);

        while (!stack.empty()) {

			Node* node = stack.top();
			stack.pop();

            if (node == nullptr) {
                break;
            }
            SideResult result = frustum.classify(node->bv);

            // if node is outside, skip
            if (result == SideResult::eOUTSIDE) {
                continue;
            }

            // if node is inside, skip query
            if (result == SideResult::eINSIDE) {
                auto retrieveIds = [&](T obj) {
                    objectsIds.push_back(obj->id);
                    };

                node->TraverseLevelOrderObjects(retrieveIds);

                continue;
            }

            //if node is intersecting, check children node
            if (result == SideResult::eINTERSECTING) {
                // if node is leaf
                if (node->IsLeaf()) {
                    T object = node->firstObject;
                    while (object != nullptr) {

                        //render objects intersecting/inside
                        if (frustum.classify(object->bv) != SideResult::eOUTSIDE) {
                            objectsIds.push_back(object->id);
                        }

                        object = object->bvhInfo.next;
                    }

                    continue;
                }

                stack.push(node->children[0]);
                stack.push(node->children[1]);
            }
        }

        return objectsIds;
    }

    template <typename T>
    std::optional<unsigned> Bvh<T>::QueryDebug(Ray const& ray, bool closest_only, std::vector<unsigned>& allIntersectedObjects, std::vector<Node const*>& debug_tested_nodes) const {

        //empty containers
        allIntersectedObjects.clear();
        debug_tested_nodes.clear();

        if (mRoot == nullptr) {
            return std::nullopt;
        }

        int closestIntersect = -1;
        float bvhShortestTime = std::numeric_limits<float>::max();

		//Recurse lamda to find the closest object intersected by the ray
        auto QueryNodesRay = [&](auto queryNodeRayFunc, const Node* node) {

            //if leaf check children, check all objects and return min time
            if (node->IsLeaf()) {

                float nodeShortestTime = std::numeric_limits<float>::max();
                T object = node->firstObject;

                while (object != nullptr) {
                    float time = ray.intersect(object->bv);

                    //object intersects
                    if (time >= 0) {
                        //if closest_only is false, insert object into allIntersectedObjects

                        if (!closest_only) {
                            allIntersectedObjects.push_back(object->id);
                        }


                        if (time < nodeShortestTime) {
                            nodeShortestTime = time;
                        }

                        if (time < bvhShortestTime) {
                            bvhShortestTime = time;
                            closestIntersect = static_cast<int>(object->id);
                        }
                    }
                    object = object->bvhInfo.next;
                }


                return nodeShortestTime;
            }



            // calculate T of children node
            float childFirstT = -1.f;
            float childSecondT = -1.f;

            if (node->children[0]) {
                debug_tested_nodes.push_back(node->children[0]);
                childFirstT = ray.intersect(node->children[0]->bv);
            }
            if (node->children[1]) {
                debug_tested_nodes.push_back(node->children[1]);
                childSecondT = ray.intersect(node->children[1]->bv);
            }

            //both child does not intersect
            if (childFirstT < 0 && childSecondT < 0) {
                return -1.f;
            }//both child intersects
            else if (childFirstT >= 0 && childSecondT >= 0) {

                //child[0] closer
                if (childFirstT < childSecondT) {
                    //check cloest child node first
                    float time = queryNodeRayFunc(queryNodeRayFunc,node->children[0]);

                    //if time is smaller than child second T, skip check
                    // if closest_only set to false, also check
                    if (!closest_only || time < 0 || time > childSecondT) {
                        time = glm::min(queryNodeRayFunc(queryNodeRayFunc,node->children[1]), time);
                    }

                    return time;
                }//child[1] closer
                else{
                    //check cloest child node first
                    float time = queryNodeRayFunc(queryNodeRayFunc, node->children[1]);

                    //if time is smaller than child second T, skip check
                    // if closest_only set to false, also check
                    if ( !closest_only  || time < 0|| time > childFirstT) {
                        time = glm::min(queryNodeRayFunc(queryNodeRayFunc, node->children[0]), time);
                    }

                    return time;
                }

            }//only child 0 intersects
            else if (childFirstT >= 0) {
                return queryNodeRayFunc(queryNodeRayFunc, node->children[0]);
            }//only child 1 intersects
            else if (childSecondT >= 0) {
                return queryNodeRayFunc(queryNodeRayFunc, node->children[1]);
            }


            return -1.f;
        };


        debug_tested_nodes.push_back(mRoot);

        if (ray.intersect(mRoot->bv) >= 0) {

            QueryNodesRay(QueryNodesRay, mRoot);
        }

        
        if(closestIntersect < 0){
            return std::nullopt;
        }


        //if closest_only, store only first object hit
        if (closest_only) {
            allIntersectedObjects.clear();
            allIntersectedObjects.push_back(static_cast<unsigned>(closestIntersect));
        }

        

        return closestIntersect;
    }

    template <typename T>
    template <typename Fn> 
    void Bvh<T>::TraverseLevelOrder(Fn func) const {
        if (mRoot == nullptr) {
            return;
        }

        mRoot->TraverseLevelOrder(func);
    }

    template <typename T>
    template <typename Fn> 
    void Bvh<T>::TraverseLevelOrderObjects(Fn func) const {
        if (mRoot == nullptr) {
            return;
        }

        mRoot->TraverseLevelOrderObjects(func);
    }

    template <typename T>
    Bvh<T>::NodeCosts::NodeCosts(Node* _node, T object, float costToNode, unsigned int _level) :
        node{ _node },
        level{ _level }
    {
        newAabb = Aabb(node->bv, object->bv);
        newGeometrics = newAabb.volume();
        newGeometricsChange = newGeometrics - node->bv.volume();

        rootToNewParentCost = newGeometrics + costToNode;


        rootToNodeCost = costToNode + newGeometricsChange;

    }
}



#endif // BVH_INL
