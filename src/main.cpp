//
// Created by Jack Purvis
//

#include <iostream>
#include <chrono>
#include <Eigen>
#include <solver.hpp>

using namespace std;
using namespace chrono;

const int MAX_ITERATIONS = 1500;
const double ENERGY_CHANGE_THRESHOLD = 0.9999;

// Parser declarations
void readOFF(string filename, vector<Vector3d> &vertices, vector<Triangle> &faces);
Eigen::Affine3d readDef(string filename);
vector<int> readSel(string filename, int vertexCount);
void writeOFF(string filename, vector<Vector3d> vertices, vector<Triangle> faces);

int main(int argc, char *argv[]) {
    cout << "ARAP Shape Deformer" << endl;

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

    // Begin algorithm pre-processing
    cout << "\n~~~ Pre-processing ~~~" << endl;
    auto startTime = high_resolution_clock::now();

    // Setup the solver
    Solver* solver = new Solver(vertices, initialGuess, faces, &handleDeformation, handleSelection);
    solver->preProcess();

    auto endTime = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(endTime - startTime);
    cout << "Took " << duration.count() << " ms" << "\n~~~~~" << endl;

    // Compute the initial energy and the threshold
    double energy = solver->computeEnergy();
    double lastEnergy = energy * (1 / ENERGY_CHANGE_THRESHOLD) + 1;
    cout << "\nInitial energy: " << energy << endl;

    // Begin algorithm body
    cout << "\n~~~ Begin Algorithm ~~~" << endl;
    double averageIterationTime = 0;
    int iterationsPerformed = 0;

    for (int iteration = 0; iteration < MAX_ITERATIONS && (energy / lastEnergy) <= ENERGY_CHANGE_THRESHOLD; iteration++) {
        startTime = high_resolution_clock::now();

        solver->solveIteration();

        endTime = high_resolution_clock::now();
        duration = duration_cast<milliseconds>(endTime - startTime);
        averageIterationTime += duration.count();

        lastEnergy = energy;
        energy = solver->computeEnergy();
        cout << "Iteration " << iteration + 1 << ", energy: " << energy << ", time taken: " << duration.count() << " ms" << endl;

        iterationsPerformed++;
    }

    double totalTimeTaken = averageIterationTime;

    averageIterationTime /= (double) iterationsPerformed;

    cout << "Total iterations: " << iterationsPerformed << ", final energy: " << energy << ", average iteration time: " << averageIterationTime << " ms" << ", total time taken: " << totalTimeTaken << " ms" << "\n~~~~~" << endl;

    // Write output file
    writeOFF(outputFilename, solver->verticesUpdated, faces);

    delete solver;

    return 0;
}
