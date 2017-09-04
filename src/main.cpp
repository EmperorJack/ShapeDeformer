//
// Created by Jack Purvis
//

#include <iostream>
#include <solver.hpp>
#include <Eigen>

using std::string;
using std::vector;

// Parser declarations
void readOFF(string filename, vector<Vector3f> &vertices, vector<triangle> &faces);
Eigen::Affine3f readDef(string filename);
std::vector<int> readSel(string filename, int vertexCount);
void writeOFF(string filename, vector<Vector3f> vertices, vector<triangle> faces);

// Deformation declaration
void performDeformation(Solver* solver, Eigen::Affine3f handleDeformation, std::vector<int> handleSelection);

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

    vector<Vector3f> vertices;
    vector<triangle> faces;

    // Parse input files
    readOFF(inputFilename, vertices, faces);
    Eigen::Affine3f handleDeformation = readDef(handleDeformationFilename);
    vector<int> handleSelection = readSel(handleSelectionFilename, (int) vertices.size());

    // Setup the solver
    Solver* solver = new Solver(vertices, faces, handleDeformation, handleSelection);

    // Perform the deformation algorithm
    solver->preProcess();
    for (int iteration = 0; iteration < 1; iteration++) {
        solver->solveIteration();
    }
    solver->postProcess();

    // Write output file
    writeOFF(outputFilename, solver->verticesUpdated, faces);

    delete solver;

    return 0;
}
