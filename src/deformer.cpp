//
// Created by Jack Purvis
//

#include <iostream>
#include <vector>
#include <Eigen>
#include <mesh.hpp>

using namespace Eigen;

SparseMatrix<float> computeAdjacencyMatrix(Mesh* mesh, std::vector<int> handleSelection) {
    SparseMatrix<float> adjacency(mesh->numVertices, mesh->numVertices);
    adjacency.reserve(VectorXi::Constant(mesh->numVertices, 7));

    //typedef Eigen::Triplet<double> T;
    //std::vector<T> tripletList;
    //tripletList.reserve(estimation_of_entries);
    //for(...)
    //{
        // ...
        //tripletList.push_back(T(i,j,v_ij));
    //}

    // Compute the edges by using connectivity information from the faces
    for (int i = 0; i < mesh->numFaces; i++) {
        triangle face = mesh->faces[i];

        // Edges are v0 to v1, v0 to v2, v1 to v3
        if (handleSelection[face.v[0]] == 1 && handleSelection[face.v[1]] == 1)
            adjacency.coeffRef(face.v[0], face.v[1]) = -1; adjacency.coeffRef(face.v[1], face.v[0]) = -1;

        if (handleSelection[face.v[0]] == 1 && handleSelection[face.v[2]] == 1)
            adjacency.coeffRef(face.v[0], face.v[2]) = -1; adjacency.coeffRef(face.v[2], face.v[0]) = -1;

        if (handleSelection[face.v[1]] == 1 && handleSelection[face.v[2]] == 1)
            adjacency.coeffRef(face.v[1], face.v[2]) = -1; adjacency.coeffRef(face.v[2], face.v[1]) = -1;
    }

    // Compute the uniform weights for each vertex
    for (int i = 0; i < mesh->numVertices; i++) {

        // Sum the connected edges in the row
        int d = 0;
        for (int j = 0; j < mesh->numVertices; j++) {
            if (adjacency.coeff(i, j) == -1) d++;
        }

        adjacency.coeffRef(i, i) = d;
    }

    //adjacency.setFromTriplets(tripletList.begin(), tripletList.end());

    return adjacency;
}

void performDeformation(Mesh* mesh, Eigen::Affine3f handleDeformation, std::vector<int> handleSelection) {
    // Ax = b
    // A = adjacency matrix
    // x = current guess
    // b = energy

    std::cout << "Computing adjacency matrix" << std::endl;

    SparseMatrix<float> adjacency = computeAdjacencyMatrix(mesh, handleSelection);

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    std::cout << "Apply constraints" << std::endl;

    // Apply transform to handle constraints from A
    for (int i = 0; i < mesh->numVertices; i++) {
        if (handleSelection[i] == 2) {
            mesh->vertices[i] = handleDeformation * mesh->vertices[i];
        }
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    std::cout << "Computing energies" << std::endl;

    // Compute the energies
    MatrixXf energies(mesh->numVertices, 3);

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    std::cout << "Solving system" << std::endl;

    // Perform a Cholesky factorization of adjacency matrix
    SimplicialCholesky<SparseMatrix<float>> solver(adjacency);

    if(solver.info() != Success) {
        std::cout << "Decomposition failed" << std::endl;
        return;
    }

    // Solve the factorisation for a new set of points
    MatrixXf guess = solver.solve(energies);

    if(solver.info() != Success) {
        std::cout << "Solving failed" << std::endl;
        return;
    }
}
