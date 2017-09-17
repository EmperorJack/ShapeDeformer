
# ARAP Shape Deformer

Mesh deformation program based on the paper by [Sorkine and Alexa 2007]. Currently only supports uniform weights and `.off` files. A test dataset is provided in `/res`.

Cmake files provided. Tested with MinGW, Unix and Visual Studio 15 2017.

Paper:
Olga Sorkine and Marc Alexa. As-rigid-as-possible surface modeling. In SGP, pages
109–116, 2007.

Dataset Source:
http://igl.ethz.ch/projects/deformation-survey/

## Usage

**Parameters**

`./ShapeDeformer inputModelPath handleDeformationPath selectionPath outputModelPath (initialGuessModelPath)`

**Example**

`./ShapeDeformer "../res/model.off" "../res/model.def" "../res/model.sel" "model-deformed.off" "../res/model-start.off"`

## Included Examples

**Knubbel Plane**

`./ShapeDeformer "../res/knubbel.off" "../res/knubbel.def" "../res/knubbel.sel" "knubbel-deformed.off"`

`./ShapeDeformer "../res/knubbel.off" "../res/knubbel.def" "../res/knubbel.sel" "knubbel-deformed.off" "../res/knubbel-laplacian_editing.off"`

**Cylinder**

`./ShapeDeformer "../res/cylinder.off" "../res/cylinder.def" "../res/cylinder.sel" "cylinder-deformed.off"`

`./ShapeDeformer "../res/cylinder.off" "../res/cylinder.def" "../res/cylinder.sel" "cylinder-deformed.off" "../res/cylinder-laplacian_editing.off"`

**Bar**

`./ShapeDeformer "../res/bar.off" "../res/bar.def" "../res/bar.sel" "bar-deformed.off"`

`./ShapeDeformer "../res/bar.off" "../res/bar.def" "../res/bar.sel" "bar-deformed.off" "../res/bar-laplacian_editing.off"`

**Cactus**

`./ShapeDeformer "../res/cactus.off" "../res/cactus.def" "../res/cactus.sel" "cactus-deformed.off"`

`./ShapeDeformer "../res/cactus.off" "../res/cactus.def" "../res/cactus.sel" "cactus-deformed.off" "../res/cactus-laplacian_editing.off"`

**Simple**

`./ShapeDeformer "../res/simple.off" "../res/simple.def" "../res/simple.sel" "../res/simple-deformed.off"`


