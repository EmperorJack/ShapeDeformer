//
// Created by Jack Purvis
//

#include <iostream>
#include <vector>
#include <Eigen>
#include <mesh.hpp>

using namespace Eigen;

void performDeformation(Mesh* mesh, Eigen::Affine3f handleDeformation, std::vector<int> handleSelection) {
    // Ax = b
    // A = adjacency matrix
    // x = current guess
    // b = energy

    std::cout << "Computing vertex neighbours" << std::endl;

    mesh->computeNeighbours();
//    for (int i = 0; i < numVertices; i++) {
//        std::set<int> cell = neighbours[i];
//        std::cout << i << ": { ";
//        for (int j : cell) {
//            std::cout << j << ", ";
//        }
//        std::cout << " }" << std::endl;
//    };

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    std::cout << "Computing adjacency matrix" << std::endl;

    mesh->computeAdjacencyMatrix(handleSelection);
    std::cout << mesh->adjacency << std::endl;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    std::cout << "Applying constraints to adjacency matrix" << std::endl;

    int nc = 0;

    // Apply transform to handle constraints from A
    for (int i = 0; i < mesh->numVertices; i++) {
        if (handleSelection[i] == 0) {
            mesh->adjacency.coeffRef(mesh->numVertices + nc, i) = 1;
            nc++;
        } else if (handleSelection[i] == 2) {
            mesh->adjacency.coeffRef(mesh->numVertices + nc, i) = 1;
            nc++;
        }
    }

    std::cout << mesh->adjacency << std::endl;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    std::cout << "Computing energies" << std::endl;

    // Compute the energies
    MatrixXf energies(mesh->numVertices, 3);

    for (int i = 0; i < mesh->numVertices; i++) {
        std::set<int> neighbours = mesh->neighbours[i];
        Matrix3f rotation = Matrix3f::Identity();

        float weight = neighbours.size();

        Vector3f energy;

        for (int j = 0; j < neighbours.size(); j++) {
            energy += (weight / 2.0f) * ((rotation) * (mesh->vertices[i] - mesh->vertices[j]));
        }

        energies.row(i) = energy;
    }

    std::cout << energies << std::endl;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    std::cout << "Applying constraints to energies" << std::endl;

    // Apply transform to handle constraints from A
    for (int i = 0; i < mesh->numVertices; i++) {
        if (handleSelection[i] == 0) {
            energies.row(i) = mesh->vertices[i];
        } else if (handleSelection[i] == 2) {
            energies.row(i) = handleDeformation * mesh->vertices[i];
            mesh->vertices[i] = handleDeformation * mesh->vertices[i];
        }
    }

    std::cout << energies << std::endl;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    std::cout << "Factorising system" << std::endl;

    // Perform a Cholesky factorization of adjacency matrix
    SimplicialLDLT<SparseMatrix<float>> solver;
    solver.compute(mesh->adjacency);

    if(solver.info() != Success) {
        std::cout << "Decomposition failed" << std::endl;
        return;
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    std::cout << "Solving system" << std::endl;

    // Solve the factorisation for a new set of points
    MatrixXf guess = solver.solve(energies);

    if(solver.info() != Success) {
        std::cout << "Solving failed" << std::endl;
        return;
    }

    std::cout << guess << std::endl;

    for (int i = 0; i < mesh->numVertices; i++) {
        if (handleSelection[i] == 1) {
            mesh->vertices[i] = guess.row(i);
        }
    }
}
