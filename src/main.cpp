//
// Created by Jack Purvis
//

#include <iostream>
#include <mesh.hpp>
#include <Eigen>

using std::string;

// Parser declarations
Mesh* readOFF(string filename);
Eigen::Matrix4f readDef(string filename);
std::vector<int> readSel(string filename, int vertexCount);
void writeOFF(string filename, Mesh* mesh);

// Deformation declaration
void performDeformation(Mesh* mesh, Eigen::Matrix4f handleDeformation, std::vector<int> handleSelection);

int main(int argc, char *argv[]) {
    fprintf(stdout, "ARAP Shape Deformer\n");

    if (argc != 4) {
        fprintf(stderr, "Filenames for an input model, handle deformation matrix and handle selection indices should be provided!\n");
        //fprintf(stderr, "e.g: ./ShapeDeformer \"../res/\"\n");
        return -1;
    }

    // Parse input files
    string inputFilename = string(argv[1]);
    string handleDeformationFilename = string(argv[2]);
    string handleSelectionFilename = string(argv[3]);

    Mesh* mesh = readOFF(inputFilename);
    Eigen::Matrix4f handleDeformation = readDef(handleDeformationFilename);
    std::vector<int> handleSelection = readSel(handleSelectionFilename, mesh->vertices.size());

    // Perform the deformation
    performDeformation(mesh, handleDeformation, handleSelection);

    // Write output file
    string outputFilename = "deformed_" + inputFilename;
    writeOFF(outputFilename, mesh);

    delete mesh;

    return 0;
}
