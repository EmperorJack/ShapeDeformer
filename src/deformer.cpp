//
// Created by Jack Purvis
//

#include <iostream>
#include <vector>
#include <Eigen>
#include <mesh.hpp>

using namespace Eigen;

MatrixXf computeAdjacencyMatrix(Mesh* mesh) {
    MatrixXf adjacency((int) mesh->vertices.size(), (int) mesh->vertices.size());
    adjacency.setZero();

    // Compute the edges by using connectivity information from the faces
    for (int i = 0; i < mesh->faces.size(); i++) {
        triangle face =  mesh->faces[i];

        // Edges are v0 to v1, v0 to v2, v1 to v3
        adjacency(face.v[0], face.v[1]) = -1; adjacency(face.v[1], face.v[0]) = -1;
        adjacency(face.v[0], face.v[2]) = -1; adjacency(face.v[2], face.v[0]) = -1;
        adjacency(face.v[1], face.v[2]) = -1; adjacency(face.v[2], face.v[1]) = -1;
    }

    // Compute the uniform weights for each vertex
    for (int i = 0; i < mesh->vertices.size(); i++) {

        // Sum the connected edges in the row
        int d = 0;
        for (int j = 0; j < mesh->vertices.size(); j++) {
            if (adjacency(i, j) == -1) d++;
        }

        adjacency(i, i) = d;
    }

    return adjacency;
}

void globalStep() {

}

void localStep() {
    
}

void performDeformation(Mesh* mesh, Eigen::Matrix4f handleDeformation, std::vector<int> handleSelection) {
    std::cout << "Deformation step" << std::endl;

    MatrixXf adjacency = computeAdjacencyMatrix(mesh);

    std::cout << adjacency << std::endl;
}
