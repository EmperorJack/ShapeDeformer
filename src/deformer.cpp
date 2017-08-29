//
// Created by Jack Purvis
//

#include <iostream>
#include <vector>
#include <Eigen>
#include <mesh.hpp>

using namespace Eigen;

MatrixXf computeAdjacencyMatrix(Mesh* mesh) {
    MatrixXf adjacency(mesh->numVertices, mesh->numVertices);
    //adjacency.reserve(VectorXi::Constant(mesh->numVertices, 10));
    adjacency.setZero();

    // Compute the edges by using connectivity information from the faces
    for (int i = 0; i < mesh->numFaces; i++) {
        triangle face =  mesh->faces[i];

        // Edges are v0 to v1, v0 to v2, v1 to v3
        adjacency.coeffRef(face.v[0], face.v[1]) = -1; adjacency.coeffRef(face.v[1], face.v[0]) = -1;
        adjacency.coeffRef(face.v[0], face.v[2]) = -1; adjacency.coeffRef(face.v[2], face.v[0]) = -1;
        adjacency.coeffRef(face.v[1], face.v[2]) = -1; adjacency.coeffRef(face.v[2], face.v[1]) = -1;
    }

    // Compute the uniform weights for each vertex
    for (int i = 0; i < mesh->numVertices; i++) {

        // Sum the connected edges in the row
        int d = 0;
        for (int j = 0; j < mesh->numVertices; j++) {
            d += -adjacency.coeffRef(i, j);
        }

        adjacency.coeffRef(i, i) = d;
    }

    return adjacency;
}

void globalStep() {

}

void localStep() {

}

void performDeformation(Mesh* mesh, Eigen::Affine3f handleDeformation, std::vector<int> handleSelection) {
    std::cout << "Deformation step" << std::endl;

    // Ax = b
    // A = adjacency matrix
    // x = current guess
    // b = energy

    // Compute A
    MatrixXf adjacency = computeAdjacencyMatrix(mesh);

    // Remove constrained rows from A
    for (int i = 0; i < mesh->numVertices; i++) {
        if (handleSelection[i] == 0) {
            adjacency.row(i).setZero();
            adjacency.col(i).setZero();
        } else if (handleSelection[i] == 2) {
            adjacency.row(i).setZero();
            adjacency.col(i).setZero();
            mesh->vertices[i] = handleDeformation * mesh->vertices[i];
        }
    }

//    std::cout << adjacency << std::endl;
}
