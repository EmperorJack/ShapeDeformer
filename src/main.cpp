//
// Created by Jack Purvis
//

#include <iostream>
#include <mesh.hpp>
#include <Eigen>

using std::string;

// Parser declarations
Mesh* readOFF(string filename);
Eigen::Affine3f readDef(string filename);
std::vector<int> readSel(string filename, int vertexCount);
void writeOFF(string filename, Mesh* mesh);

// Deformation declaration
void performDeformation(Mesh* mesh, Eigen::Affine3f handleDeformation, std::vector<int> handleSelection);

int main(int argc, char *argv[]) {
    fprintf(stdout, "ARAP Shape Deformer\n");

    if (argc != 5) {
        fprintf(stderr, "Filenames for an input model, handle deformation matrix, handle selection indices and output model should be provided!\n");
        //fprintf(stderr, "e.g: ./ShapeDeformer \"../res/\"\n");
        return -1;
    }

    // Parse input arguments
    string inputFilename = string(argv[1]);
    string handleDeformationFilename = string(argv[2]);
    string handleSelectionFilename = string(argv[3]);
    string outputFilename = string(argv[4]);

    // Parse input files
    Mesh* mesh = readOFF(inputFilename);
    Eigen::Affine3f handleDeformation = readDef(handleDeformationFilename);
    std::vector<int> handleSelection = readSel(handleSelectionFilename, (int) mesh->vertices.size());

    // Perform the deformation
    performDeformation(mesh, handleDeformation, handleSelection);

    // Write output file
    writeOFF(outputFilename, mesh);

    delete mesh;

    return 0;
}
