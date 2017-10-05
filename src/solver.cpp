//
// Created by Jack Purvis
//

#include <iostream>
#include <omp.h>
#include <solver.hpp>

const bool COTAN_WEIGHTS = true;

using namespace Eigen;

Solver::Solver(vector<Vector3d> vertices, vector<Vector3d> initialGuess, vector<Triangle> faces, Affine3d* handleDeformation, vector<int> handleSelection) :
        vertices(vertices), faces(faces), handleDeformation(handleDeformation) {
    this->numVertices = (int) vertices.size();
    this->numFaces = (int) faces.size();

    // std::cout << "Computing free vs fixed vertices" << std::endl;

    for (int i = 0; i < numVertices; i++) {
        if (handleSelection[i] == 0) {
            fixedVertices.push_back(i);
            vertexTypes.push_back(Fixed);
        } else if (handleSelection[i] == 1) {
            freeVertices.push_back(i);
            vertexTypes.push_back(Free);
        } else if (handleSelection[i] == 2) {
            fixedVertices.push_back(i);
            vertexTypes.push_back(Handle);
        }
    }

    numFreeVertices = (int) freeVertices.size();
    numFixedVerties = (int) fixedVertices.size();

    // std::cout << "Initializing updated vertices" << std::endl;

    if (initialGuess.empty()) {
        verticesUpdated = vertices;
    } else {
        verticesUpdated = initialGuess;
    }
}

void Solver::preProcess() {
    // std::cout << "Computing vertex neighbours" << std::endl;
    computeNeighbours();

    // std::cout << "Computing weights" << std::endl;
    computeWeights();

    // std::cout << "Initializing rotations" << std::endl;
    rotations.resize((size_t) numVertices, Matrix3d::Identity());
    computeRotations();

    // std::cout << "Enforcing constraints on updated vertices" << std::endl;
    for (int i = 0; i < numVertices; i++) {
        if (vertexTypes[i] == Handle) {
            verticesUpdated[i] = *handleDeformation * vertices[i];
        }
    }

    // std::cout << "Computing laplace beltrami matrix" << std::endl;
    computeLaplaceBeltrami();

    // std::cout << "Factorising system" << std::endl;
    systemSolver.analyzePattern(laplaceBeltrami);
    systemSolver.factorize(laplaceBeltrami);

    if(systemSolver.info() != Success) {
        // std::cout << "Factorisation failed" << std::endl;
        exit(-1);
    }
}

void Solver::computeNeighbours() {
    vector<set<int>> neighbourSets;

    for (int i = 0; i < numVertices; i++) {
        set<int> temp;
        neighbourSets.push_back(temp);
    }

    for (int i = 0; i < numFaces; i++) {
        Triangle face = faces[i];

        neighbourSets[face.v[0]].insert(face.v[1]);
        neighbourSets[face.v[0]].insert(face.v[2]);

        neighbourSets[face.v[1]].insert(face.v[0]);
        neighbourSets[face.v[1]].insert(face.v[2]);

        neighbourSets[face.v[2]].insert(face.v[0]);
        neighbourSets[face.v[2]].insert(face.v[1]);
    }

    for (int i = 0; i < numVertices; i++) {
        vector<int> temp(neighbourSets[i].begin(), neighbourSets[i].end());
        neighbours.push_back(temp);
    }
}

void Solver::computeWeights() {
    weights.resize(numVertices, numVertices);
    weights.reserve(VectorXi::Constant(numVertices, 7));

    if (COTAN_WEIGHTS) {
        Vector3d a, b, c;
        Vector3d v1, v2;
        double cot;

        for (int i = 0; i < numFaces; i++) {
            Triangle face = faces[i];
            a = vertices[face.v[0]];
            b = vertices[face.v[1]];
            c = vertices[face.v[2]];

            v1 = a - c;
            v2 = b - c;
            cot = v1.dot(v2) / v1.cross(v2).norm();
            cot *= 0.5;
            weights.coeffRef(face.v[0], face.v[1]) += cot;
            weights.coeffRef(face.v[1], face.v[0]) += cot;

            v1 = a - b;
            v2 = c - b;
            cot = v1.dot(v2) / v1.cross(v2).norm();
            cot *= 0.5;
            weights.coeffRef(face.v[0], face.v[2]) += cot;
            weights.coeffRef(face.v[2], face.v[0]) += cot;

            v1 = b - a;
            v2 = c - a;
            cot = v1.dot(v2) / v1.cross(v2).norm();
            cot *= 0.5;
            weights.coeffRef(face.v[1], face.v[2]) += cot;
            weights.coeffRef(face.v[2], face.v[1]) += cot;
        }
    } else {
        for (int i = 0; i < numVertices; i++) {
            for (int j : neighbours[i]) {
                double weight = 1.0;

                weights.coeffRef(i, j) = weight;
            }
        }
    }
}

void Solver::computeLaplaceBeltrami() {
    laplaceBeltrami.resize(numVertices, numVertices);
    laplaceBeltrami.reserve(VectorXi::Constant(numVertices, 7));

    for (int i = 0; i < numVertices; i++) {
        if (vertexTypes[i] != Free) {
            laplaceBeltrami.coeffRef(i, i) = 1.0;
        } else {
            for (int j : neighbours[i]) {
                double weight = weights.coeff(i, j);

                laplaceBeltrami.coeffRef(i, i) += weight;

                if (vertexTypes[j] == Free) {
                    laplaceBeltrami.coeffRef(i, j) -= weight;
                }
            }
        }
    }
}

void Solver::computeRotations() {

    #pragma omp parallel for
    for (int i = 0; i < numVertices; i++) {
        Matrix3d covariance = Matrix3d::Zero();

        for (int j : neighbours[i]) {
            double weight = weights.coeff(i, j);

            Vector3d edge = vertices[i] - vertices[j];
            Vector3d edgeUpdated = verticesUpdated[i] - verticesUpdated[j];

            covariance += weight * edge * edgeUpdated.transpose();
        }

        JacobiSVD<MatrixXd> svd(covariance, ComputeFullU | ComputeFullV);
        Matrix3d u = svd.matrixU();
        Matrix3d v = svd.matrixV();
        Matrix3d rotation = v * u.transpose();

        if (rotation.determinant() <= 0) {
            Vector3d singularValues = svd.singularValues();

            int smallestSingularValueColumn = 0;
            double smallestSingularValue = 100000000000000000.0;

            for (int c = 0; c < 3; c++) {
                if (singularValues(c) < smallestSingularValue) {
                    smallestSingularValue = singularValues(c);
                    smallestSingularValueColumn = c;
                }
            }

            u.col(smallestSingularValueColumn) *= -1;
            rotation = v * u.transpose();
        }

        if (rotation.determinant() < 0) std::cout << "Determinant should not be negative" << std::endl;

        rotations[i] = rotation;
    }
}

void Solver::solveIteration() {

    computeRotations();

    // Compute the RHS
    MatrixX3d rhs = MatrixXd::Zero(numVertices, 3);

    #pragma omp parallel for
    for (int i = 0; i < numVertices; i++) {

        if (vertexTypes[i] != Free) {
            rhs.row(i) = verticesUpdated[i];
        } else {
            for (int j : neighbours[i]) {
                double weight = weights.coeff(i, j);

                Vector3d vec = (weight / 2.0) * (rotations[i] + rotations[j]) * (vertices[i] - vertices[j]);
                rhs.row(i) += vec;

                if (vertexTypes[j] != Free) {
                    rhs.row(i) += weight * verticesUpdated[j];
                }
            }
        }
    }

    // Solve the factorisation for a new set of vertices
    MatrixXd solution = systemSolver.solve(rhs);

    if (systemSolver.info() != Success) {
        // std::cout << "Solving failed" << std::endl;
        exit(-1);
    }

    for (int i = 0; i < numVertices; i++) {
        if (vertexTypes[i] == Free) {
            verticesUpdated[i] = solution.row(i);
        }
    }
}

double Solver::computeEnergy() {
    double energies[numVertices];

    #pragma omp parallel for
    for (int i = 0; i < numVertices; i++) {
        energies[i] = 0.0f;

        for (int j : neighbours[i]) {
            double weight = weights.coeff(i, j);

            Vector3d vec = (verticesUpdated[i] - verticesUpdated[j]) - rotations[i] * (vertices[i] - vertices[j]);

            energies[i] += weight * vec.squaredNorm();
        }
    }

    double energy = 0.0;
    for (int i = 0; i < numVertices; i++) {
        energy += energies[i];
    }

    return energy;
}
