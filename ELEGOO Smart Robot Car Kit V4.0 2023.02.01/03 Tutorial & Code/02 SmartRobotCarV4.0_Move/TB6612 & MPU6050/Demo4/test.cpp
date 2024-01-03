// C++ program for Dijkstra's single source shortest path
// algorithm. The program is for adjacency matrix
// representation of the graph
using namespace std;
#include <limits.h>;
#include "DeviceDriverSet_xxx0.h"
#include "MPU6050_getdata.h"

// Number of vertices in the graph
#define V 16

static int pathArray[V][V];

// A utility function to find the vertex with minimum
// distance value, from the set of vertices not yet included
// in shortest path tree
static int minDistance(int dist[], bool sptSet[])
{

	// Initialize min value
	int min = INT_MAX, min_index;

	for (int v = 0; v < V; v++)
		if (sptSet[v] == false && dist[v] <= min)
			min = dist[v], min_index = v;

	return min_index;
}

// A utility function to print the constructed distance
// array
static void printSolution(int dist[])
{
	Serial.println("Vertex \t Distance from Source");
	for (int i = 0; i < V; i++)
    Serial.println(dist[i]);
}

// Function that implements Dijkstra's single source
// shortest path algorithm for a graph represented using
// adjacency matrix representation
static void dijkstra(int graph[V][V], int src)
{
	int dist[V]; // The output array. dist[i] will hold the
				// shortest
	// distance from src to i

	bool sptSet[V]; // sptSet[i] will be true if vertex i is
					// included in shortest
	// path tree or shortest distance from src to i is
	// finalized

  for (int i = 0; i < V; i++) 
    for (int j = 0; j < V; j++)
      pathArray[i][j] = -1;

	// Initialize all distances as INFINITE and stpSet[] as
	// false
	for (int i = 0; i < V; i++)
		dist[i] = INT_MAX, sptSet[i] = false;

	// Distance of source vertex from itself is always 0
	dist[src] = 0;
  pathArray[src][0] = src;

	// Find shortest path for all vertices
	for (int count = 0; count < V - 1; count++) {
		// Pick the minimum distance vertex from the set of
		// vertices not yet processed. u is always equal to
		// src in the first iteration.
		int u = minDistance(dist, sptSet);

		// Mark the picked vertex as processed
		sptSet[u] = true;

		// Update dist value of the adjacent vertices of the
		// picked vertex.
		for (int v = 0; v < V; v++)

			// Update dist[v] only if is not in sptSet,
			// there is an edge from u to v, and total
			// weight of path from src to v through u is
			// smaller than current value of dist[v]
			if (!sptSet[v] && graph[u][v]
				&& dist[u] != INT_MAX
				&& dist[u] + graph[u][v] < dist[v]) {
          int lastIndex = -1;
          for (int j = 0; j < V; j++) {
            if (pathArray[u][j] != -1) {
              pathArray[v][j] = pathArray[u][j];
            } else {
              lastIndex = j;
              break;
            }
          }
          pathArray[v][lastIndex] = v;

				  dist[v] = dist[u] + graph[u][v];
      }
	}
}

// This code is contributed by shivanisinghss2110
