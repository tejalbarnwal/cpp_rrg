# Rapidly Explording Random Graphs with cpp

### Method

###### Problem
* Understanding of the Problem: The problem is related to a setting where given we have info about obstalces of the environment, we wish to explore the entrie environemnt. This problem can be particularly useful when in a big 2D space we wish to look for artifacts in all the space.
* A good exploration algorithm will visit almost all places while ensuring that the time(or distance) required to complete the exploration is minimized.

##### Solution
* Rapidly Explording Random Graphs(RRG), an algorithm which derives RRT*
* The method doesnt require us to store a 2D map of any resolution.
* RRG pseudo code
```
let the initial pose be x_init
V: list of vertices
E: list of edges
i = 0
neighbour_radius

while i < N:
    G(V, E)
    x_rand = RandomPosition()
    i = i + 1 

    x_nearest = Nearest(G, x_rand)
    x_new = Steer(x_nearest, x_rand)
    
    if ObstacleFree(x_nearest, x_new):
        V add x_new
        E add (x_nearest, x_new) and (x_new, x_nearest)
        x_neigbours = findNeighbors(G, x_new, neighbour_radius)

        for x in x_neigbours:
            if ObstacleFree(x_new, x):
                E add (x_new, x) and (x, x_new)
```
* A variation of TSP problem is solved on graph formed with RRG to ensure that all the vertices in the graph are visited *atleast once* while ensuring that the travel ditance is minimized
* The TSP is solved using heuristic based algorithm to get a near optimal solution so that the solving takes less time 

### File Arragement
* The repositories has directories for header files in `include`, source files inside `src` and compilation in `build` directory.
* `include` :
	* `custom_graph.h` : declares structures like vertex, edge, graph
	* `geometry.h` : declares functions to calculate euclidean and manhattan distances, collision checking between line segment and circular obstacle, etc
	* `matplotlibcpp.h` : avaiable open source wrapper for python-matplotlib
	* `tspnn.h` : declares functions for solving TSP, Djkistra, adjacency matrix
	* `rrg.h` : declares helper functions for RRG
* `src` :
	* `test_graph.cpp` : testing file to ensure working of data structures inside `custom_graph`
	* `test_geom.cpp` : testing file to ensure all function of `geometry` are working
	* `test_tsp.cpp` : testing file to ensure all function inside `tspnn` are working
	* `test_rrg.cpp` : contains actual code for the algorithm, and hence the assigment


### Test
Move inside build directly to see compiled source files:
`test_geom`, `test_graph`, `test_tsp`, `tesp_rrg` 

To run the RRG algorithm, do `./test_rrg`

To generate new compiled files for the sources files, do:
1. Move inside build directly
2. run `cmake ..`
3. run `make`


### References
* https://arxiv.org/pdf/1005.0416.pdf
* https://www.baeldung.com/cs/circle-line-segment-collision-detection
* https://www.geeksforgeeks.org/printing-paths-dijkstras-shortest-path-algorithm/
* https://blog.devgenius.io/traveling-salesman-problem-nearest-neighbor-algorithm-solution-e78399d0ab0c

