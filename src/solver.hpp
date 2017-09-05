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
    Free,
    Fixed,
    Handle
};

struct Triangle {
    int v[3];
};

class Solver {

public:
    Solver(vector<Vector3f> vertices, vector<Triangle> faces, Affine3f handleDeformation, vector<int> handleSelection);

    void preProcess();
    void computeNeighbours();
    void computeWeights();
    void computeLaplaceBeltrami();

    void computeRotations();
    void solveIteration();

    void postProcess();

    float computeEnergy();

    int numVertices;
    int numFaces;

    int numFreeVertices;
    int numFixedVerties;

    vector<Vector3f> vertices;
    vector<Vector3f> verticesUpdated;
    vector<Triangle> faces;
    vector<set<int>> neighbours;

    vector<VertexType> vertexTypes;
    vector<int> freeVertices;
    vector<int> fixedVertices;

    vector<float> weights;
    vector<MatrixXf> rotations;
    SparseMatrix<float> laplaceBeltrami;
    SparseLU<SparseMatrix<float>> systemSolver;

};

#endif //SHAPEDEFORMER_SOLVER_HPP
