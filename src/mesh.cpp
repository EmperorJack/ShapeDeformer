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
    SparseMatrix<float> adjacency(numVertices, numVertices);
    adjacency.reserve(VectorXi::Constant(numVertices, 7));

    // Compute the edges by using connectivity information from the faces
    for (int i = 0; i < numFaces; i++) {
        triangle face = faces[i];

        // Edges are v0 to v1, v0 to v2, v1 to v3
        //if (handleSelection[face.v[0]] == 1 && handleSelection[face.v[1]] == 1)
        adjacency.coeffRef(face.v[0], face.v[1]) = -1; adjacency.coeffRef(face.v[1], face.v[0]) = -1;

        //if (handleSelection[face.v[0]] == 1 && handleSelection[face.v[2]] == 1)
        adjacency.coeffRef(face.v[0], face.v[2]) = -1; adjacency.coeffRef(face.v[2], face.v[0]) = -1;

        //if (handleSelection[face.v[1]] == 1 && handleSelection[face.v[2]] == 1)
        adjacency.coeffRef(face.v[1], face.v[2]) = -1; adjacency.coeffRef(face.v[2], face.v[1]) = -1;
    }

    // Compute the uniform weights for each vertex
    for (int i = 0; i < numVertices; i++) {
        adjacency.coeffRef(i, i) = neighbours[i].size();
    }
}