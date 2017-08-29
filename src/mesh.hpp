//
// Created by Jack Purvis
//

#ifndef SHAPEDEFORMER_MESH_HPP
#define SHAPEDEFORMER_MESH_HPP

#include <vector>
#include <Eigen>

struct triangle {
    int v[3];
};

class Mesh {

public:
    std::vector<Eigen::Vector3f> vertices;
    std::vector<triangle> faces;

};

#endif //SHAPEDEFORMER_MESH_HPP
