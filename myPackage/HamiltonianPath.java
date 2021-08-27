package myPackage;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.Polygon;
import javax.swing.*;

import java.util.*;

import java.lang.Math;
import java.text.*;

public class HamiltonianPath extends JComponent {

  private Polygon polygon = new Polygon();

  public HamiltonianPath() {
  }

  // method to be called by Simulator to generate shortest path
  // this method doesn't actually get the shortest path but will
  // call other methods in this class to do so
  // this class does handles calling other methods 
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
    int[] shortestPathPointIndex = getShortestPathGreedy(adjacencyMatrix);

    ////

    // create new Point[] to be returned
    Point[] shortestPath = new Point[n + 1];
    for (int i = 0; i < n + 1; i++) {
      shortestPath[i] = pointsArray[shortestPathPointIndex[i]];
    }

    for (Point p : shortestPath) {
      this.polygon.addPoint((int) p.getX(), (int) p.getY());
    }
    this.polygon.addPoint(0, 0);

    this.repaint();

    return shortestPath;
  }

  // one implementation to get shortest path
  int[] getShortestPathGreedy(double[][] adjacencyMatrix) {
    // final path to take
    ArrayList<Integer> path = new ArrayList<Integer>(adjacencyMatrix.length);

    // list of nodes to traverse
    ArrayList<Integer> nodes = new ArrayList<Integer>(adjacencyMatrix.length);
    for (int i = 0; i < adjacencyMatrix.length; i++) {
      nodes.add(i);
    }

    path.add(0); // add origin
    nodes.remove(0); // remove origin

    int currentNode = 0; // to track current processing node
    double min = 99999; // to find min distance from current node
    int minIdx = 0; // to track index of node with min distance from current node

    // traverse all nodes
    while (!nodes.isEmpty()) {
      min = 99999;
      minIdx = 0;

      // for current node, find the closest node
      for (int ii = 0; ii < adjacencyMatrix.length; ii++) {
        if (currentNode != ii) {
          if (!path.contains(ii)) {
            System.out.println("Calculating for " + currentNode + " " + ii);
            if (adjacencyMatrix[currentNode][ii] < min) {
              System.out.println(currentNode + " " + ii);
              minIdx = ii;
              min = adjacencyMatrix[currentNode][ii];
            }
          }
        }
      }
      // add index of closest ndoe to final path
      path.add(minIdx);

      // prepare to traverse the next node in path
      currentNode = minIdx;
      // remove next node from list of nodes to traverse
      nodes.remove((Integer) minIdx);
    }

    System.out.println("Path index: " + path.toString());

    // tranforming arrayList<Integer> to int[]
    int[] result = new int[adjacencyMatrix.length];
    for (int ii = 0; ii < adjacencyMatrix.length; ii++) {
      result[ii] = path.get(ii);
    }

    return result;
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
    for (int i = 0; i < adjacencyMatrix.length; i++) {
      for (int j = 0; j < adjacencyMatrix[i].length; j++) {
        System.out.print(new DecimalFormat("000.000", DecimalFormatSymbols.getInstance(Locale.ENGLISH))
            .format(adjacencyMatrix[i][j]));
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
// need to generate shortest path based on Points
// need to generated weighted graph
//