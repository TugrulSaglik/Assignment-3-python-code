# 2D Object-Oriented Structural Analysis Program

This repository contains a modular, object-oriented 2D structural analysis software developed in Python. It is capable of analyzing linear elastic structures composed of Frame, Truss, and Spring elements. The program is built entirely from scratch, utilizing a custom sparse matrix library without dependencies on external mathematical packages like `numpy`.

## Features
* **Custom Matrix Library:** Includes memory-efficient `SparseMatrix` storage (Dictionary of Keys) and a custom Gaussian elimination solver.
* **Element Types:** Supports standard Frame elements, pure axial Truss elements, and Spring elements.
* **Moment Releases:** Supports internal hinges via statical condensation of the local stiffness matrices.
* **Advanced Error Checking:** Validates kinematic stability, checks for disconnected substructures (floating members), catches overlapping coordinates, and prevents singular matrix crashes (collapse mechanisms).
* **Automated Output:** Generates a structured `.txt` report detailing equation numbering, nodal displacements, and local member end forces.

## Project Structure
The software is divided into three highly modular files:
1. `matrix_library.py`: The core mathematical backend housing the `Matrix` class hierarchy and linear solvers.
2. `frame_solver.py`: The physical modeling backend containing the `StructuralModel` and object-oriented structural hierarchy (Nodes, Materials, Elements, Supports, Loads).
3. `main.py`: The execution pipeline that parses the input text file, triggers the assembly and solution methods, and writes the output report.

## How to Run the Program
1. Ensure you have Python installed on your system (Python 3.6+ is recommended). No external libraries are required.
2. Clone or download this repository so that `matrix_library.py`, `frame_solver.py`, and `main.py` are in the same folder.
3. Create a text file named `input.txt` in the same directory (see formatting rules below).
4. Run the program via terminal/command prompt:
   ```bash
   python main.py
   ```
5. The results will be automatically saved in the same directory as output_report.txt.

The input file uses bracketed headers to define structural properties. Decimals should use a dot (.).

[Materials]
 Material ID / Area / Inertia / Elastic Modulus
 
1 / 0.02 / 0.08 / 200000.0

[Nodes]
 Node ID / X - Coor / Y - Coor
 
1 / 0.0 / 0.0
2 / 0.0 / 4.0

[Members]
 Mem ID / Start Node / End Node / Mat ID / Type (Frame/Truss) / Release Start (0/1) / Release End (0/1)
 Note: 1 = Moment Released (Hinge), 0 = Rigid Connection
 
1 / 1 / 2 / 1 / Frame / 0 / 0

[Supports]
 Node ID / RX / RY / RZ 
 Note: 1 = Restrained, 0 = Free
 
1 / 1 / 1 / 1

[Loads]
 Load ID / Node ID / Fx / Fy / Mz
 
1 / 2 / 10.0 / 0.0 / 0.0
