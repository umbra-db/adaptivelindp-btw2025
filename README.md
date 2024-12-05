
# Implementation of Adaptive LinDP

Implemented optimization algorithms:

* IKKBZ (with and without linearization transfer)
* MCM DP
* Adaptive DP
* Extended LinDP
* Adaptive LinDP

The code has been written for research purposes and has not been battle-tested in a production setting.
This repository mainly serves as a reference for the implementation of the algorithms from the paper
"Optimizing Linearized Join Enumeration by Adapting to the Query Structure" to be presented at BTW 2025.

## Compilation

We have compiled the code on Ubuntu 24.10, CMake 3.30.2, and GCC 14.2.0.

## Data

In addition to randomly generating data, we have logic for reading graphs in the g6 format or trees in the list of edges
format.

Dataset of all connected graphs of various sizes can be found here:
https://users.cecs.anu.edu.au/~bdm/data/graphs.html

Dataset of all trees of various sizes can be found here:
https://users.cecs.anu.edu.au/~bdm/data/trees.html

Description of the graph6 format can be found here:
https://users.cecs.anu.edu.au/~bdm/data/formats.html

You can download and place the graphs and trees in the data folder.

## License

The repository is licensed with the AGPL-3.0 license found in `LICENSE.txt` unless otherwise specified.

