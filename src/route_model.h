#ifndef ROUTE_MODEL_H
#define ROUTE_MODEL_H

#include <limits>
#include <cmath>
#include <unordered_map>
#include "model.h"
#include <iostream>

class RouteModel : public Model {

  public:
    class Node : public Model::Node {
      public:
        Node * parent = nullptr;
        float h_value = std::numeric_limits<float>::max();
        float g_value = 0.0;
        bool visited = false;
        std::vector<Node *> neighbors;

        void FindNeighbors();
        float distance(Node second) const {
            float x = std::pow((this->x - second.x), 2);
            float y = std::pow((this->y - second.y), 2);
            return std::sqrt(x + y);
        }

        Node(){}
        Node(int idx, RouteModel * search_model, Model::Node node) : Model::Node(node), parent_model(search_model), index(idx) {}

      private:
        int index;
        // closest unvisited node from "this" using node indices
        RouteModel::Node * FindNeighbor(std::vector<int> node_indices);
        RouteModel * parent_model = nullptr;
    };

    RouteModel(const std::vector<std::byte> &xml);
    Node &FindClosestNode(float x, float y);
    std::vector<Node> &SNodes() { return m_Nodes; }
    std::vector<Node> path;
    auto &GetNodeToRoadMap() {return node_to_road; }
    
  private:
    std::vector<Node> m_Nodes;  // Store nodes from the OSM data
    std::unordered_map<int, std::vector<const Model::Road *>> node_to_road; // all roads from this node
    void CreateNodeToRoadHashmap();

};

#endif
