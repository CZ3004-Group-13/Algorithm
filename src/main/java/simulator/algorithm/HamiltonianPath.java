package simulator.algorithm;

import javax.swing.*;
import java.awt.*;
import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.Locale;
import java.util.Set;
import java.util.logging.Level;
import java.util.logging.Logger;

public class HamiltonianPath extends JComponent {

    private final Logger logger;

    private Polygon polygon = new Polygon();

    public HamiltonianPath() {
        logger = Logger.getLogger(HamiltonianPath.class.getName());
    }

    // method to be called by Simulator to generate the shortest path
    // this method doesn't actually get the shortest path but will
    // call other methods in this class to do so
    // this class does handle calling other methods
    public Point[] getShortestPath(Point src, Point[] inputs) {
        this.polygon.reset();

        int n = inputs.length;

        // adds origin and all input points into single ArrayList
        ArrayList<Point> pointList = new ArrayList<>(1 + n);
        Collections.addAll(pointList, src);
        Collections.addAll(pointList, inputs);

        Point[] pointsArray = new Point[n + 1];
        pointsArray = pointList.toArray(pointsArray);

        // get adjacency matrix to be used for shortest path algo
        double[][] adjacencyMatrix = getAdjacencyMatrix(pointsArray);
        printAdjacencyMatrix(adjacencyMatrix);

        // use whichever algo you want here
        // eg, return [1, 4, 2, 3] means go to
        // 1st -> 4th -> 2nd -> 3rd node of pointsArray
        // pointsArray[0] -> pointsArray[3] -> pointsArray[1] -> pointsArray[2]
        // int[] shortestPathPointIndex = getShortestPathGreedy(adjacencyMatrix);
        int[] shortestPathPointIndex = getShortestPathEfficiently(adjacencyMatrix);

        ////

        // create new Point[] to be returned
        Point[] shortestPath = new Point[n + 1];
        for (int i = 0; i < n + 1; i++) {
            shortestPath[i] = pointsArray[shortestPathPointIndex[i]];
        }

        for (Point p : shortestPath) {
            this.polygon.addPoint((int) p.getX(), (int) p.getY());
        }
        //this.polygon.addPoint(src.getX(), src.getY());

        this.repaint();

        return shortestPath;
    }

    /**
     * Get the shortest path in the shortest hamiltonian path in O(2^N * N^2) and return it.
     *
     * @param adjacencyMatrix Adjacency Matrix of the graph.
     * @return Shortest path.
     */
    int[] getShortestPathEfficiently(double[][] adjacencyMatrix) {
        int numberOfNodes = adjacencyMatrix.length;

        double[][] dynamicProgramming = new double[1 << numberOfNodes][numberOfNodes];
        for (double[] row : dynamicProgramming) {
            Arrays.fill(row, Double.MAX_VALUE / 2);
        }
        for (int i = 0; i < numberOfNodes; i++) {
            dynamicProgramming[1 << i][i] = 0;
        }
        for (int mask = 0; mask < 1 << numberOfNodes; mask++) {
            for (int i = 0; i < numberOfNodes; i++) {
                if ((mask & 1 << i) != 0) {
                    for (int j = 0; j < numberOfNodes; j++) {
                        if ((mask & 1 << j) != 0) {
                            dynamicProgramming[mask][i] = Math.min(dynamicProgramming[mask][i], dynamicProgramming[mask ^ (1 << i)][j] + adjacencyMatrix[j][i]);
                        }
                    }
                }
            }
        }

        double distance = Double.MAX_VALUE;
        for (int i = 0; i < numberOfNodes; i++) {
            distance = Math.min(distance, dynamicProgramming[(1 << numberOfNodes) - 1][i]);
        }

        int curr = (1 << numberOfNodes) - 1;
        int[] path = new int[numberOfNodes];
        int last = -1;
        for (int i = numberOfNodes - 1; i >= 0; i--) {
            int k = -1;
            for (int j = 0; j < numberOfNodes; j++) {
                if ((curr & 1 << j) != 0 && (k == -1 || dynamicProgramming[curr][k] + (last == -1 ? 0 : adjacencyMatrix[k][last]) > dynamicProgramming[curr][j] + (last == -1 ? 0 : adjacencyMatrix[j][last]))) {
                    k = j;
                }
            }
            path[i] = k;
            curr ^= 1 << k;
            last = k;
        }

        logger.log(Level.FINER, "Total Distance: " + distance);
        logger.log(Level.FINER, Arrays.toString(path));

        return path;
    }

    // one implementation to get (approximately) the shortest path
    int[] getShortestPathGreedy(double[][] adjacencyMatrix) {
        // final path to take
        ArrayList<Integer> path = new ArrayList<>(adjacencyMatrix.length);

        // Set of nodes to traverse
        Set<Integer> nodes = new HashSet<>();
        for (int i = 0; i < adjacencyMatrix.length; i++) {
            nodes.add(i);
        }

        path.add(0); // add origin
        nodes.remove(0); // remove origin

        int currentNode = 0; // to track current processing node
        double min; // to find min distance from current node
        int minIdx; // to track index of node with min distance from current node

        double totalDistance = 0;

        // traverse all nodes
        while (!nodes.isEmpty()) {
            min = Integer.MAX_VALUE;
            minIdx = 0;

            // for current node, find the closest node
            for (int ii = 0; ii < adjacencyMatrix.length; ii++) {
                if (currentNode != ii) {
                    if (!path.contains(ii)) {
                        logger.log(Level.FINEST, "Calculating for " + currentNode + " " + ii);
                        if (adjacencyMatrix[currentNode][ii] < min) {
                            logger.log(Level.FINEST, currentNode + " " + ii);
                            minIdx = ii;
                            min = adjacencyMatrix[currentNode][ii];
                        }
                    }
                }
            }
            // add index of the closest node to final path
            totalDistance += min;
            path.add(minIdx);

            // prepare to traverse the next node in path
            currentNode = minIdx;
            // remove next node from list of nodes to traverse
            nodes.remove(minIdx);
        }

        logger.log(Level.FINER, "Path index: " + path);
        logger.log(Level.FINER, "Total Distance: " + totalDistance);

        return path.stream().mapToInt(i -> i).toArray();
    }

    // calculate adjacency matrix based on given points
    double[][] getAdjacencyMatrix(Point[] points) {
        double[][] adjacencyMatrix = new double[points.length][points.length];
        for (int i = 0; i < points.length; i++) {
            for (int j = 0; j < points.length; j++) {
                adjacencyMatrix[i][j] = getEuclideanDistance(points[i], points[j]);
            }
        }
        return adjacencyMatrix;
    }

    // helper function to calculate distance between two points
    double getEuclideanDistance(Point a, Point b) {
        return Math.sqrt(Math.pow(Math.abs(a.getX() - b.getX()), 2) + Math.pow(Math.abs(a.getY() - b.getY()), 2));
    }

    // to print out the adjacency matrix
    void printAdjacencyMatrix(double[][] adjacencyMatrix) {
        System.out.println("Adjacency Matrix:");
        for (double[] row : adjacencyMatrix) {
            for (double cell : row) {
                System.out.print(new DecimalFormat("000.000", DecimalFormatSymbols.getInstance(Locale.ENGLISH))
                        .format(cell));
                System.out.print("   ");
            }
            System.out.println();
        }
    }

    public void paintComponent(Graphics g) {
        super.paintComponent(g);

        Graphics2D g2 = (Graphics2D) g;
        g2.setColor(Color.green);
        g2.drawPolygon(this.polygon);
    }

}

// input of Points
// need to generate the shortest path based on Points
// need to generated weighted graph
//