//
// Created by Jack Purvis
//

#include <iostream>
#include <mesh.hpp>

using namespace Eigen;

Mesh::Mesh(int numVertices, int numFaces) {
    this->numVertices = numVertices;
    this->numFaces = numFaces;
}

void Mesh::computeNeighbours() {
    for (int i = 0; i < numVertices; i++) {
        std::set<int> temp;
        neighbours.push_back(temp);
    }

    for (int i = 0; i < numFaces; i++) {
        triangle face = faces[i];

        neighbours[face.v[0]].insert(face.v[1]);
        neighbours[face.v[0]].insert(face.v[2]);

        neighbours[face.v[1]].insert(face.v[0]);
        neighbours[face.v[1]].insert(face.v[2]);

        neighbours[face.v[2]].insert(face.v[0]);
        neighbours[face.v[2]].insert(face.v[1]);
    }
}

void Mesh::computeAdjacencyMatrix(std::vector<int> handleSelection) {
    adjacency.resize(numVertices, numVertices);
    //adjacency.reserve(VectorXi::Constant(numVertices, 7));

    for (int i = 0; i < numVertices; i++) {
        for (int j = 0; j < neighbours[j].size(); j++) {
            float weight = 1.0f / ((float) neighbours.size());

            adjacency.coeffRef(i, i) += weight;

            if (handleSelection[j] == 1) {
                adjacency.coeffRef(i, j) -= weight;
            }
        }
    }

    adjacency.makeCompressed();
}