//
// Created by Jack Purvis
//

#include <iostream>
#include <mesh.hpp>

using std::string;

// Parser declarations
Mesh* readOFF(string filename);
void readDef(string filename);
void readSel(string filename);
void writeOFF(string filename, Mesh* mesh);

// Deformation declaration
void performDeformation();

int main(int argc, char *argv[]) {
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
    readDef(handleDeformationFilename);
    readSel(handleSelectionFilename);

    // Perform the deformation
    performDeformation();

    // Write output file
    string outputFilename = "deformed_" + inputFilename;
    writeOFF(outputFilename, mesh);

    delete mesh;

    return 0;
}