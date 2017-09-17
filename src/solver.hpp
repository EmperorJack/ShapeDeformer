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

enum VertexType {
    Fixed,
    Free,
    Handle
};

struct Triangle {
    int v[3];
};

class Solver {

public:
    Solver(vector<Vector3d> vertices, vector<Vector3d> initialGuess, vector<Triangle> faces, Affine3d* handleDeformation, vector<int> handleSelection);

    void preProcess();
    void computeNeighbours();
    void computeWeights();
    void computeLaplaceBeltrami();

    void computeRotations();
    void solveIteration();

    double computeEnergy();

    int numVertices;
    int numFaces;

    int numFreeVertices;
    int numFixedVerties;

    Affine3d* handleDeformation;

    vector<Vector3d> vertices;
    vector<Vector3d> verticesUpdated;
    vector<Triangle> faces;
    vector<vector<int>> neighbours;

    vector<VertexType> vertexTypes;
    vector<int> freeVertices;
    vector<int> fixedVertices;

    vector<MatrixXd> rotations;
    SparseMatrix<double> weights;
    SparseMatrix<double> laplaceBeltrami;
    SparseLU<SparseMatrix<double>> systemSolver;

};

#endif //SHAPEDEFORMER_SOLVER_HPP
