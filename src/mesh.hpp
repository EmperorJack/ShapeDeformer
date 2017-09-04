//
// Created by Jack Purvis
//

#ifndef SHAPEDEFORMER_MESH_HPP
#define SHAPEDEFORMER_MESH_HPP

#include <set>
#include <vector>
#include <Eigen>

using namespace Eigen;

struct triangle {
    int v[3];
};

class Mesh {

public:
    Mesh(int numVertices, int numFaces);
    void computeNeighbours();
    void computeWeights();
    void computeLaplaceBeltrami(std::vector<int> handleSelection);

    int numVertices;
    int numFaces;

    std::vector<Vector3f> vertices;
    std::vector<Vector3f> verticesUpdated;
    std::vector<triangle> faces;

    std::vector<std::set<int>> neighbours;

    VectorXf weights;
    SparseMatrix<float> laplaceBeltrami;

};

#endif //SHAPEDEFORMER_MESH_HPP
