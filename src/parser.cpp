//
// Created by Jack Purvis
//

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
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

void readDef(string filename) {

}

void readSel(string filename) {

}

void writeOFF(string filename, Mesh* mesh) {

}