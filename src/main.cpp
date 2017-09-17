//
// Created by Jack Purvis
//

#include <iostream>
#include <solver.hpp>
#include <Eigen>

using std::string;
using std::vector;

const int NUM_ITERATIONS = 100;
const double CONVERGENCE_THRESHOLD = 0.0;

// Parser declarations
void readOFF(string filename, vector<Vector3d> &vertices, vector<Triangle> &faces);
Eigen::Affine3d readDef(string filename);
std::vector<int> readSel(string filename, int vertexCount);
void writeOFF(string filename, vector<Vector3d> vertices, vector<Triangle> faces);

int main(int argc, char *argv[]) {
    fprintf(stdout, "ARAP Shape Deformer\n");

    if (argc < 5) {
        fprintf(stderr, "Filenames for an input model, handle deformation matrix, handle selection indices and output model should be provided!\nInitial guess input model is optional\n");
        fprintf(stderr, "Usage: ./ShapeDeformer inputModelPath handleDeformationPath selectionPath outputModelPath (initialGuessModelPath)\n");
        fprintf(stderr, "e.g: ./ShapeDeformer \"../res/model.off\" \"../res/model.def\" \"../res/model.sel\" \"model-deformed.off\" \"../res/model-start.off\"\n");
        return -1;
    }

    // Parse input arguments
    string inputFilename = string(argv[1]);
    string handleDeformationFilename = string(argv[2]);
    string handleSelectionFilename = string(argv[3]);
    string outputFilename = string(argv[4]);
    string initialGuessFilename;
    if (argc == 6) {
        initialGuessFilename = string(argv[5]);
    }

    vector<Vector3d> vertices;
    vector<Vector3d> initialGuess;
    vector<Triangle> faces;

    // Parse input files
    readOFF(inputFilename, vertices, faces);
    Eigen::Affine3d handleDeformation = readDef(handleDeformationFilename);
    vector<int> handleSelection = readSel(handleSelectionFilename, (int) vertices.size());

    if (argc == 6) {
        readOFF(initialGuessFilename, initialGuess, faces);
    }

    // Setup the solver
    Solver* solver = new Solver(vertices, initialGuess, faces, &handleDeformation, handleSelection);

    // Perform the deformation algorithm
    solver->preProcess();

    double energy = solver->computeEnergy();
    std::cout << "Initial energy: " << energy << std::endl;

    for (int iteration = 0; iteration < NUM_ITERATIONS && energy > CONVERGENCE_THRESHOLD; iteration++) {
        solver->solveIteration();

        energy = solver->computeEnergy();
        std::cout << "Iteration " << iteration << " energy: " << energy << std::endl;
    }

    std::cout << "Final energy: " << energy << std::endl;

    // Write output file
    writeOFF(outputFilename, solver->verticesUpdated, faces);

    delete solver;

    return 0;
}
