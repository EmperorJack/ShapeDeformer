//
// Created by Jack Purvis
//

#ifndef SHAPEDEFORMER_SOLVER_HPP
#define SHAPEDEFORMER_SOLVER_HPP

#include <set>
#include <vector>
#include <Eigen>

using std::set;
using std::vector;
using namespace Eigen;

struct triangle {
    int v[3];
};

class Solver {

public:
    Solver(vector<Vector3f> vertices, vector<triangle> faces, Affine3f handleDeformation, vector<int> handleSelection);

    void preProcess();
    void computeNeighbours();
    void computeWeights();
    void computeLaplaceBeltrami(vector<int> handleSelection);

    void computeRotations();
    void solveIteration();

    void postProcess();

    float computeEnergy();

    int numVertices;
    int numFaces;

    // Mesh information
    vector<Vector3f> vertices;
    vector<Vector3f> verticesUpdated;
    vector<triangle> faces;
    vector<set<int>> neighbours;

    // Handle information
    Affine3f handleDeformation;
    vector<int> handleSelection;

    // Algorithm information
    vector<float> weights;
    vector<MatrixXf> rotations;
    SparseMatrix<float> laplaceBeltrami;
    SparseLU<SparseMatrix<float>> systemSolver;

};

#endif //SHAPEDEFORMER_SOLVER_HPP
