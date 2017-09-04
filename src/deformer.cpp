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

//    std::cout << mesh->adjacency << std::endl;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    std::cout << "Factorising system" << std::endl;

    // Perform a Cholesky factorization of adjacency matrix
    SparseLU<SparseMatrix<float>> solver;
    solver.compute(mesh->adjacency);

    if(solver.info() != Success) {
        std::cout << "Decomposition failed" << std::endl;
        return;
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    std::cout << "Initialise updated vertices" << std::endl;

    mesh->verticesUpdated.resize((size_t) mesh->numVertices, Vector3f::Zero());

    for (int i = 0; i < mesh->numVertices; i++) {
        if (handleSelection[i] == 2) {
            mesh->verticesUpdated[i] = handleDeformation * mesh->vertices[i];
        } else {
            mesh->verticesUpdated[i] = mesh->vertices[i];
        }
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    for (int iteration = 0; iteration < 1; iteration++) {

        std::cout << "Computing energies" << std::endl;

        // Compute the energies
        MatrixXf energies = MatrixXf::Zero(mesh->numVertices, 3);

        for (int i = 0; i < mesh->numVertices; i++) {
            std::set<int> neighbours = mesh->neighbours[i];

            Matrix3f rotation = Matrix3f::Identity();

            float weight = 1.0f / ((float) neighbours.size());

            for (int j : neighbours) {
                Vector3f vec = (weight / 2.0f) * (rotation) * (mesh->vertices[i] - mesh->vertices[j]);
                energies.row(i) += vec;

                if (handleSelection[j] != 1) {
                    energies.row(i) += weight * mesh->verticesUpdated[j];
                }
            }
        }

//        std::cout << energies << std::endl;

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        std::cout << "Solving system" << std::endl;

        // Solve the factorisation for a new set of points
        MatrixXf solution = solver.solve(energies);

        if (solver.info() != Success) {
            std::cout << "Solving failed" << std::endl;
            return;
        }

//        std::cout << solution << std::endl;

        for (int i = 0; i < mesh->numVertices; i++) {
            if (handleSelection[i] == 1) {
                mesh->verticesUpdated[i] = solution.row(i);
            }
        }
    }

    for (int i = 0; i < mesh->numVertices; i++) {
        mesh->vertices[i] = mesh->verticesUpdated[i];
    }
}
