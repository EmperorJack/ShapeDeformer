//
// Created by Jack Purvis
//

#include <iostream>
#include <solver.hpp>

using namespace Eigen;

Solver::Solver(vector<Vector3f> vertices, vector<triangle> faces, Affine3f handleDeformation, vector<int> handleSelection) :
        vertices(vertices), faces(faces), handleDeformation(handleDeformation), handleSelection(handleSelection) {
    this->numVertices = (int) vertices.size();
    this->numFaces = (int) faces.size();

    std::cout << "Initialise updated vertices" << std::endl;

    verticesUpdated.resize((size_t) numVertices, Vector3f::Zero());

    for (int i = 0; i < numVertices; i++) {
        if (handleSelection[i] == 2) {
            verticesUpdated[i] = handleDeformation * vertices[i];
        } else {
            verticesUpdated[i] = vertices[i];
        }
    }
}

void Solver::preProcess() {
    std::cout << "Computing vertex neighbours" << std::endl;
    computeNeighbours();

    std::cout << "Computing weights" << std::endl;
    computeWeights();

    std::cout << "Computing laplace beltrami matrix" << std::endl;
    computeLaplaceBeltrami(handleSelection);

    std::cout << "Factorising system" << std::endl;
    systemSolver.compute(laplaceBeltrami);

    if(systemSolver.info() != Success) {
        std::cout << "Decomposition failed" << std::endl;
        return;
    }
}

void Solver::computeNeighbours() {
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

void Solver::computeWeights() {
    weights.resize(numVertices);

    for (int i = 0; i < numVertices; i++) {
        weights.coeffRef(i) = 1.0f / ((float) neighbours[i].size());
    }
}

void Solver::computeLaplaceBeltrami(std::vector<int> handleSelection) {
    laplaceBeltrami.resize(numVertices, numVertices);
    laplaceBeltrami.reserve(VectorXi::Constant(numVertices, 7));

    for (int i = 0; i < numVertices; i++) {
        for (int j : neighbours[i]) {
            float weight = weights.coeff(i);

            laplaceBeltrami.coeffRef(i, i) += weight;

            if (handleSelection[j] == 1) {
                laplaceBeltrami.coeffRef(i, j) -= weight;
            }
        }
    }

    laplaceBeltrami.makeCompressed();
}

void Solver::solveIteration() {
    std::cout << "Computing energies" << std::endl;

    // Compute the energies
    MatrixXf energies = MatrixXf::Zero(numVertices, 3);

    for (int i = 0; i < numVertices; i++) {
        Matrix3f rotation = Matrix3f::Identity();

        float weight = weights.coeff(i);

        for (int j : neighbours[i]) {
            Vector3f vec = (weight / 2.0f) * (rotation) * (vertices[i] - vertices[j]);
            energies.row(i) += vec;

            if (handleSelection[j] != 1) {
                energies.row(i) += weight * verticesUpdated[j];
            }
        }
    }

    std::cout << "Solving system" << std::endl;

    // Solve the factorisation for a new set of points
    MatrixXf solution = systemSolver.solve(energies);

    if (systemSolver.info() != Success) {
        std::cout << "Solving failed" << std::endl;
        return;
    }

    for (int i = 0; i < numVertices; i++) {
        if (handleSelection[i] == 1) {
            verticesUpdated[i] = solution.row(i);
        }
    }
}

void Solver::postProcess() {

}
