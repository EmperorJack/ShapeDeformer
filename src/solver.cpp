//
// Created by Jack Purvis
//

#include <iostream>
#include <solver.hpp>

using namespace Eigen;

Solver::Solver(vector<Vector3f> vertices, vector<Triangle> faces, Affine3f handleDeformation, vector<int> handleSelection) :
        vertices(vertices), faces(faces) {
    this->numVertices = (int) vertices.size();
    this->numFaces = (int) faces.size();

    std::cout << "Computing free vs fixed vertices" << std::endl;

    for (int i = 0; i < numVertices; i++) {
        if (handleSelection[i] == 0) {
            vertexInformation.push_back(VertexInfo(Fixed, fixedVertices.size()));
            fixedVertices.push_back(i);
        } else if (handleSelection[i] == 1) {
            vertexInformation.push_back(VertexInfo(Free, freeVertices.size()));
            freeVertices.push_back(i);
        } else if (handleSelection[i] == 2) {
            vertexInformation.push_back(VertexInfo(Handle, fixedVertices.size()));
            fixedVertices.push_back(i);
        }
    }

    numFreeVertices = freeVertices.size();
    numFixedVerties = fixedVertices.size();

    std::cout << "Initializing updated vertices" << std::endl;

    for (int i = 0; i < numVertices; i++) {
        if (vertexInformation[i].type == Handle) {
            verticesUpdated.push_back(handleDeformation * vertices[i]);
        } else {
            verticesUpdated.push_back(vertices[i]);
        }
    }
}

void Solver::preProcess() {
    std::cout << "Computing vertex neighbours" << std::endl;
    computeNeighbours();

    std::cout << "Computing weights" << std::endl;
    computeWeights();

    std::cout << "Initializing rotations" << std::endl;

    rotations.resize((size_t) numVertices, Matrix3f::Zero());
    computeRotations();

    std::cout << "Computing laplace beltrami matrix" << std::endl;
    computeLaplaceBeltrami();

    std::cout << "Factorising system" << std::endl;
    systemSolver.compute(laplaceBeltrami);

    if(systemSolver.info() != Success) {
        std::cout << "Factorisation failed" << std::endl;
        exit(-1);
    }
}

void Solver::computeNeighbours() {
    for (int i = 0; i < numVertices; i++) {
        std::set<int> temp;
        neighbours.push_back(temp);
    }

    for (int i = 0; i < numFaces; i++) {
        Triangle face = faces[i];

        neighbours[face.v[0]].insert(face.v[1]);
        neighbours[face.v[0]].insert(face.v[2]);

        neighbours[face.v[1]].insert(face.v[0]);
        neighbours[face.v[1]].insert(face.v[2]);

        neighbours[face.v[2]].insert(face.v[0]);
        neighbours[face.v[2]].insert(face.v[1]);
    }
}

void Solver::computeWeights() {
    weights.resize((size_t) numVertices, 1.0f);
}

void Solver::computeLaplaceBeltrami() {
    laplaceBeltrami.resize(numFreeVertices, numFreeVertices);
    laplaceBeltrami.reserve(VectorXi::Constant(numFreeVertices, 7));

    for (int i = 0; i < numFreeVertices; i++) {
        int iPos = freeVertices[i];

        for (int jPos : neighbours[iPos]) {
            float weight = weights[iPos];

            laplaceBeltrami.coeffRef(i, i) += weight;

            if (vertexInformation[jPos].type == Free) {
                laplaceBeltrami.coeffRef(i, vertexInformation[jPos].pos) -= weight;
            }
        }
    }

    laplaceBeltrami.makeCompressed();
}

void Solver::computeRotations() {
    vector<Matrix3f> edgeProduct((size_t) numVertices, Matrix3f::Zero());

    for (int i = 0; i < numVertices; i++) {
        for (int j : neighbours[i]) {
            float weight = weights[i];

            Vector3f edge = vertices[i] - vertices[j];
            Vector3f edgeUpdated = verticesUpdated[i] - verticesUpdated[j];

            edgeProduct[i] += weight * edge * edgeUpdated.transpose();
        }
    }

    for (int i = 0; i < numVertices; i++) {
        JacobiSVD<MatrixXf> svd(edgeProduct[i], ComputeThinU | ComputeThinV);
        Matrix3f rotation = svd.matrixV() * svd.matrixU().transpose();
        rotations[i] = rotation;
    }
}

void Solver::solveIteration() {

    computeRotations();

    // Compute the RHS
    MatrixXf rhs = MatrixXf::Zero(numFreeVertices, 3);

    for (int i = 0; i < numFreeVertices; i++) {
        int iPos = freeVertices[i];

        for (int jPos : neighbours[iPos]) {
            float weight = weights[iPos];

            Vector3f vec = (weight / 2.0f) * (rotations[iPos] + rotations[jPos]) * (vertices[iPos] - vertices[jPos]);
            rhs.row(i) += vec;

            if (vertexInformation[jPos].type != Free) {
                rhs.row(i) += weight * verticesUpdated[jPos];
            }
        }
    }

    // Solve the factorisation for a new set of vertices
    MatrixXf solution = systemSolver.solve(rhs);

    if (systemSolver.info() != Success) {
        std::cout << "Solving failed" << std::endl;
        exit(-1);
    }

    for (int i = 0; i < numFreeVertices; i++) {
        int iPos = freeVertices[i];
        verticesUpdated[iPos] = solution.row(i);
    }
}

void Solver::postProcess() {

}

float Solver::computeEnergy() {
    float energy = 0.0f;

    for (int i = 0; i < numVertices; i++) {
        for (int j : neighbours[i]) {
            float weight = weights[i];

            Vector3f vec = (verticesUpdated[i] - verticesUpdated[j]) - rotations[i] * (vertices[i] - vertices[j]);

            energy += weight * vec.squaredNorm();
        }
    }

    return energy;
}
