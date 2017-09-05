//
// Created by Jack Purvis
//

#include <iostream>
#include <WunderSVD3x3.h>
#include <solver.hpp>

using namespace Eigen;

Solver::Solver(vector<Vector3f> vertices, vector<Triangle> faces, Affine3f handleDeformation, vector<int> handleSelection) :
        vertices(vertices), faces(faces) {
    this->numVertices = (int) vertices.size();
    this->numFaces = (int) faces.size();

    std::cout << "Computing free vs fixed vertices" << std::endl;

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

    std::cout << "Initializing updated vertices" << std::endl;

    for (int i = 0; i < numVertices; i++) {
        if (vertexTypes[i] == Handle) {
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

//    rotations.resize((size_t) numVertices, Matrix3f::Zero());
    computeRotations();

    std::cout << "Computing laplace beltrami matrix" << std::endl;
    computeLaplaceBeltrami();

    std::cout << "Factorising system" << std::endl;
    systemSolver.analyzePattern(laplaceBeltrami);
    systemSolver.factorize(laplaceBeltrami);

    if(systemSolver.info() != Success) {
        std::cout << "Factorisation failed" << std::endl;
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

bool useCotanWeights = false;

void Solver::computeWeights() {
    weights.resize(numVertices, numVertices);
    weights.reserve(VectorXi::Constant(numVertices, 7));

    if (!useCotanWeights) {
        for (int i = 0; i < numVertices; i++) {
            for (int j : neighbours[i]) {
                float weight = 1.0f;

                weights.coeffRef(i, j) = weight;
//                weights.coeffRef(j, i) = weight;
            }
        }
    } else {
        // 1/2(cot alpha_ij + cot beta_ij)

        int indexMap[3][2] = { {1, 2}, {2, 0}, {0, 1} };

        for (int f = 0; f < numFaces; f++) {
            Triangle face = faces[f];

            // Get the cotangent value with that face.
            Vector3f cotangent = computeCotangent(face);

            // Loop over the three vertices within the same triangle.
            // i = 0 => A.
            // i = 1 => B.
            // i = 2 => C.
            for (int i = 0; i < 3; i++) {

                // Indices of the two vertices in the edge corresponding to vertex i.
                int first = face.v[indexMap[i][0]];
                int second = face.v[indexMap[i][1]];

                float halfCotangent = cotangent(i) / 2.0f;

                weights.coeffRef(first, second) += halfCotangent;
                weights.coeffRef(second, first) += halfCotangent;

                // Note that weight_(i, i) is the sum of all the -weight_(i, j).

                weights.coeffRef(first, first) -= halfCotangent;
                weights.coeffRef(second, second) -= halfCotangent;
            }
        }
    }
}

Vector3f Solver::computeCotangent(Triangle face) {
    Eigen::Vector3f cotangent(0.0f, 0.0f, 0.0f);
    // The triangle is defined as follows:
    //            A
    //           /  -
    //        c /     - b
    //         /        -
    //        /    a      -
    //       B--------------C
    // where A, B, C corresponds to faces_(face_id, 0), faces_(face_id, 1) and
    // faces_(face_id, 2). The return value is (cotA, cotB, cotC).
    // Compute the triangle area first.
    Vector3f A = vertices[face.v[0]];
    Vector3f B = vertices[face.v[1]];
    Vector3f C = vertices[face.v[2]];
    float a_squared = (B - C).squaredNorm();
    float b_squared = (C - A).squaredNorm();
    float c_squared = (A - B).squaredNorm();
    // Compute the area of the triangle. area = 1/2bcsinA.
    float area = (B - A).cross(C - A).norm() / 2.0f;
    // Compute cotA = cosA / sinA.
    // b^2 + c^2 -2bccosA = a^2, or cosA = (b^2 + c^2 - a^2) / 2bc.
    // 1/2bcsinA = area, or sinA = 2area/bc.
    // cotA = (b^2 + c^2 -a^2) / 4area.
    float four_area = 4.0f * area;
    cotangent(0) = (b_squared + c_squared - a_squared) / four_area;
    cotangent(1) = (c_squared + a_squared - b_squared) / four_area;
    cotangent(2) = (a_squared + b_squared - c_squared) / four_area;
    return cotangent;
}

void Solver::computeLaplaceBeltrami() {
//    laplaceBeltrami.resize(numVertices, numVertices);
//    laplaceBeltrami.reserve(VectorXi::Constant(numVertices, 7));
//
//    for (int i = 0; i < numVertices; i++) {
//        if (vertexTypes[i] != Free) {
//            laplaceBeltrami.coeffRef(i, i) = 1.0f;
//        } else {
//            for (int j : neighbours[i]) {
//                float weight = weights.coeff(i, j);
//
//                laplaceBeltrami.coeffRef(i, i) += weight;
//
//                if (vertexTypes[j] == Free) {
//                    laplaceBeltrami.coeffRef(i, j) -= weight;
////                    laplaceBeltrami.coeffRef(j, i) -= weight;
//                }
//            }
//        }
//    }
//
//    laplaceBeltrami.makeCompressed();

//    std::cout << laplaceBeltrami << std::endl << "~~~" << std::endl;
//
    SparseMatrix<float> weightSum;
    weightSum.resize(numVertices, numVertices);
    weightSum.reserve(VectorXi::Constant(numVertices, 7));

    for (int i = 0; i < numVertices; i++) {

        float wi = 0.0f;

        for (int j : neighbours[i]) {

            if (vertexTypes[i] == Free) {
                wi += weights.coeff(i, j);
            } else {
                weights.coeffRef(i, j) = 0.0f;
//                weights.coeffRef(j, i) = 0.0f;

                wi = 1.0f;
            }
        }

        weightSum.coeffRef(i, i) = wi;
    }

    laplaceBeltrami = weightSum - weights;

//    std::cout << weights << std::endl << "~~~" << std::endl;
//    std::cout << weightSum << std::endl << "~~~" << std::endl;
//    std::cout << laplaceBeltrami << std::endl << "~~~" << std::endl;
}

void Solver::computeRotations() {
    rotations.clear();

    for (int i = 0; i < numVertices; i++) {
        Matrix3f covariance = Matrix3f::Zero();

        for (int j : neighbours[i]) {
            float weight = weights.coeff(i, j);

            Vector3f edge = vertices[i] - vertices[j];
            Vector3f edgeUpdated = verticesUpdated[i] - verticesUpdated[j];

            covariance += weight * edge * edgeUpdated.transpose();
        }

//        JacobiSVD<MatrixXf> svd(covariance, ComputeFullU | ComputeFullV);
//        Matrix3f u = svd.matrixU();
//        Matrix3f v = svd.matrixV();
//        Matrix3f rotation = v * u.transpose();
//
//        if (rotation.determinant() <= 0) {
//            Vector3f singularValues = svd.singularValues();
//
//            int smallestSingularValueColumn = 0;
//            float smallestSingularValue = 100000000000000000.0f;
//
//            for (int c = 0; c < 3; c++) {
//                if (singularValues(c) < smallestSingularValue) {
//                    smallestSingularValue = singularValues(c);
//                    smallestSingularValueColumn = c;
//                }
//            }
//
//            u.col(smallestSingularValueColumn) *= -1;
//            rotation = v * u.transpose();
//        }

//        std::cout << rotation << std::endl << "~ VS ~" << std::endl;

        Matrix3f Ui;
        Vector3f Wi;
        Matrix3f Vi;
        wunderSVD3x3(covariance, Ui, Wi, Vi);

        Matrix3f rotation = Vi * Ui.transpose();

//        std::cout << rotation << std::endl<< "~~~~~" << std::endl << std::endl;

//        Affine3f t;
//        t = covariance;
//        Matrix3f rotation = t.rotation();

        if (rotation.determinant() < 0) std::cout << "Determinant should not be negative" << std::endl;

        rotations.push_back(rotation.transpose());
    }
}

void Solver::solveIteration() {

    computeRotations();

    // Compute the RHS
    MatrixX3f rhs = MatrixXf::Zero(numVertices, 3);

//    for (int i = 0; i < numVertices; i++) {
//        for (int j : neighbours[i]) {
//            float weight = weights.coeff(i, j);
//
//            Vector3f vec = (weight / 2.0f) * (rotations[i] + rotations[j]) * (vertices[i] - vertices[j]);
//            rhs.row(i) += vec;
//
//            if (vertexTypes[j] != Free) {
//                rhs.row(i) += weight * verticesUpdated[j];
//            }
//        }
//    }

//    std::cout << rhs << std::endl << "~~~" << std::endl;
//
    rhs = MatrixXf::Zero(numVertices, 3);

    for (int i = 0; i < numVertices; i++) {

        if (vertexTypes[i] != Free) {
            rhs.row(i) = verticesUpdated[i];
        } else {
            for (int j : neighbours[i]) {
                float weight = weights.coeff(i, j);

                Vector3f vec = (weight / 2.0f) * (rotations[i] + rotations[j]) * (vertices[i] - vertices[j]);
                rhs.row(i) += vec;
            }
        }
    }
//
//    std::cout << rhs << std::endl;

    // Solve the factorisation for a new set of vertices
    MatrixXf solution = systemSolver.solve(rhs);

    if (systemSolver.info() != Success) {
        std::cout << "Solving failed" << std::endl;
        exit(-1);
    }

    for (int i = 0; i < numVertices; i++) {
        if (vertexTypes[i] == Free) {
            verticesUpdated[i] = solution.row(i);
        }
    }
}

void Solver::postProcess() {

}

float Solver::computeEnergy() {
    float energy = 0.0f;

    for (int i = 0; i < numVertices; i++) {
        for (int j : neighbours[i]) {
            float weight = weights.coeff(i, j);

            Vector3f vec = (verticesUpdated[i] - verticesUpdated[j]) - rotations[i] * (vertices[i] - vertices[j]);

            energy += weight * vec.squaredNorm();
        }
    }

    return energy;
}
