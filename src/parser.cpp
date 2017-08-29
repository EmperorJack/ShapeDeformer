//
// Created by Jack Purvis
//

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <mesh.hpp>

using std::string;

Mesh* readOFF(string filename) {
    Mesh* mesh = new Mesh();

    std::ifstream fileStream(filename);
    string line;

    // Skip the first line
    std::getline(fileStream, line);

    // Read the vertex, face and edge counts
    int vertexCount, faceCount, edgeCount;

    std::getline(fileStream, line);
    std::istringstream lineTokens(line);
    lineTokens >> vertexCount >> faceCount >> edgeCount;

    // Read each vertex
    for (int i = 0; i < vertexCount; i++) {
        Eigen::Vector3f vertex;

        std::getline(fileStream, line);
        std::istringstream vertexTokens(line);
        vertexTokens >> vertex.x() >> vertex.y() >> vertex.z();

        mesh->vertices.push_back(vertex);
    }

    // Read each triangle
    for (int i = 0; i < faceCount; i++) {
        triangle face;
        int numVertices; // Should always be 3

        std::getline(fileStream, line);
        std::istringstream faceTokens(line);
        faceTokens >> numVertices >> face.v[0] >> face.v[1] >> face.v[2];

        mesh->faces.push_back(face);
    }

    return mesh;
}

Eigen::Matrix4f readDef(string filename) {
    Eigen::Matrix4f deformation;

    std::ifstream fileStream(filename);
    string line;

    // Skip the first line
    std::getline(fileStream, line);

    // For each row
    for (int i = 0; i < 4; i++) {
        std::getline(fileStream, line);
        std::istringstream lineTokens(line);

        // For each column
        for (int j = 0; j < 4; j++) {
            float value;
            lineTokens >> value;
            deformation(i, j) = value;
        }
    }

    return deformation;
}

std::vector<int> readSel(string filename, int vertexCount) {
    std::vector<int> selection;

    std::ifstream fileStream(filename);
    string line;

    // Skip the first two lines
    std::getline(fileStream, line);
    std::getline(fileStream, line);

    // For each vertex
    for (int i = 0; i < vertexCount; i++) {
        std::getline(fileStream, line);
        selection.push_back(std::stoi(line));
    }

    return selection;
}

void writeOFF(string filename, Mesh* mesh) {

}