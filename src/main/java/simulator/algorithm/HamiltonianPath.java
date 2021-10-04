package simulator.algorithm;

import simulator.entity.Direction;
import simulator.entity.Grid;
import simulator.entity.MyPoint;
import simulator.entity.RelativeDirection;
import simulator.entity.Robot;

import javax.swing.*;
import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.NoninvertibleTransformException;
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

    // Line is currently drawn using polygon
    private final Polygon polygon = new Polygon();
    private final Polygon plannedPPolygon = new Polygon();
    private MyPoint[] shortestPath;

    private ArrayList<MyPoint> plannedPath = new ArrayList<>();

    public HamiltonianPath() {
        logger = Logger.getLogger(HamiltonianPath.class.getName());
    }

    public MyPoint[] getShortestPath(MyPoint src, MyPoint[] inputs) {
        return getShortestPath(src, inputs, false, null, null);
    }

    // method to be called by Simulator to generate the shortest path
    // this method doesn't actually get the shortest path but will
    // call other methods in this class to do so
    // this class does handle calling other methods
    public MyPoint[] getShortestPath(MyPoint src, MyPoint[] inputs, boolean flipObstacleDirections, Grid grid,
            Robot robot) {
        this.reset();
        this.polygon.reset();

        int n = inputs.length;

        if (flipObstacleDirections) {
            // instead of giving the directions of the obstacle face,
            // set direction to the direction that the robot should be facing at that point
            // to be used for the planned path
            for (MyPoint p : inputs) {
                switch (p.getDirection()) {
                    case NORTH:
                        p.setDirection(Direction.SOUTH);
                        break;
                    case SOUTH:
                        p.setDirection(Direction.NORTH);
                        break;
                    case EAST:
                        p.setDirection(Direction.WEST);
                        break;
                    case WEST:
                        p.setDirection(Direction.EAST);
                        break;
                    case NONE:
                        break;
                    default:
                        break;

                }
            }
        }

        // adds origin and all input points into single ArrayList
        ArrayList<MyPoint> pointList = new ArrayList<>(1 + n);
        Collections.addAll(pointList, src);
        Collections.addAll(pointList, inputs);

        MyPoint[] pointsArray = pointList.toArray(new MyPoint[0]);

        // get adjacency matrix to be used for shortest path algo
        double[][] adjacencyMatrix = getAdjacencyMatrix(pointsArray);
        printAdjacencyMatrix(adjacencyMatrix);

        // use whichever algo you want here
        // eg, return [1, 4, 2, 3] means go to
        // 1st -> 4th -> 2nd -> 3rd node of pointsArray
        // pointsArray[0] -> pointsArray[3] -> pointsArray[1] -> pointsArray[2]
        int[] shortestPathPointIndex = getShortestPathWithChecking(adjacencyMatrix, pointsArray, grid, robot);

        ////

        // create new Point[] to be returned
        MyPoint[] shortestPath = new MyPoint[n + 1];
        for (int i = 0; i < n + 1; i++) {
            shortestPath[i] = pointsArray[shortestPathPointIndex[i]];
        }

        /*
         * for (Point p : shortestPath) { this.polygon.addPoint((int) p.getX(), (int)
         * p.getY()); }
         *
         * Not necessary this.polygon.addPoint((int) src.getX(), (int) src.getY());
         */

        this.repaint();

        this.shortestPath = shortestPath;
        return shortestPath;
    }

    /**
     * Get the shortest path in the shortest hamiltonian path in O(2^N * N^2) and
     * return it.
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
                            dynamicProgramming[mask][i] = Math.min(dynamicProgramming[mask][i],
                                    dynamicProgramming[mask ^ (1 << i)][j] + adjacencyMatrix[j][i]);
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
                if ((curr & 1 << j) != 0 && (k == -1 || dynamicProgramming[curr][k]
                        + (last == -1 ? 0 : adjacencyMatrix[k][last]) > dynamicProgramming[curr][j]
                                + (last == -1 ? 0 : adjacencyMatrix[j][last]))) {
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

    // get shortest path based on the path finding algorithm below
    // checks every permutation of node travel
    // returns the permutation that results in the shortest valid path
    public int[] getShortestPathWithChecking(double[][] adjacencyMatrix, MyPoint[] pointsArray, Grid grid,
            Robot robot) {
        ArrayList<Integer> pathIndex = new ArrayList<Integer>(adjacencyMatrix.length);

        ArrayList<ArrayList<Integer>> pathspermu = new ArrayList<>();
        ArrayList<ArrayList<Integer>> paths = new ArrayList<>();
        ArrayList<Double> pathLengths = new ArrayList<>();

        for (int i = 1; i < adjacencyMatrix.length; i++) {
            pathIndex.add(i);
        }
        pathspermu = generatePerm(pathIndex);

        for (int ii = 0; ii < pathspermu.size(); ii++) {
            pathspermu.get(ii).add(0, 0);
            // for (int jj = 0 ; jj < pathspermu.get(ii).size(); jj++) {
            // System.out.print(pathspermu.get(ii).get(jj) + " ");
            // }
            // System.out.println("");
            MyPoint[] shortestPath = new MyPoint[adjacencyMatrix.length];
            for (int i = 0; i < adjacencyMatrix.length; i++) {
                shortestPath[i] = pointsArray[pathspermu.get(ii).get(i)];
            }
            this.shortestPath = shortestPath;

            boolean valid = this.generatePlannedPath(grid, robot);
            if (valid) {
                paths.add(pathspermu.get(ii));
            }
            double distance = this.getPathEstTotalDistance(robot);
            pathLengths.add(distance);
        }

        double smallestDistance = Double.MAX_VALUE;
        int idx = -1;
        for (int ii = 0; ii < paths.size(); ii++) {
            if (pathLengths.get(ii) < smallestDistance) {
                smallestDistance = pathLengths.get(ii);
                idx = ii;
            }
        }

        System.out.println("Index: " + idx);

        return paths.get(idx).stream().mapToInt(i -> i).toArray();
    }

    <E> ArrayList<ArrayList<E>> generatePerm(ArrayList<E> original) {
        if (original.isEmpty()) {
            ArrayList<ArrayList<E>> result = new ArrayList<>();
            result.add(new ArrayList<>());
            return result;
        }
        E firstElement = original.remove(0);
        ArrayList<ArrayList<E>> returnValue = new ArrayList<>();
        ArrayList<ArrayList<E>> permutations = generatePerm(original);
        for (ArrayList<E> smallerPermutated : permutations) {
            for (int index = 0; index <= smallerPermutated.size(); index++) {
                ArrayList<E> temp = new ArrayList<>(smallerPermutated);
                temp.add(index, firstElement);
                returnValue.add(temp);
            }
        }
        return returnValue;
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
                System.out.print(
                        new DecimalFormat("000.000", DecimalFormatSymbols.getInstance(Locale.ENGLISH)).format(cell));
                System.out.print("   ");
            }
            System.out.println();
        }
    }

    public void paintComponent(Graphics g) {
        super.paintComponent(g);

        Graphics2D g2 = (Graphics2D) g;

        /*
         * g2.setColor(Color.green); g2.drawPolygon(this.polygon);
         */

        g2.setColor(Color.blue);
        g2.drawPolygon(this.plannedPPolygon);
    }

    public boolean generatePlannedPath(Grid grid, Robot robot) {
        this.reset();
        boolean isValid = true;
        ArrayList<MyPoint> workingPath = new ArrayList<MyPoint>(Arrays.asList(this.shortestPath));

        this.plannedPath.add(robot.getCurrentLocation());
        MyPoint sp;
        int caStep = 5;
        int turnRadius2, turnRadius1;
        turnRadius2 = (int) robot.MAX_TURNING_RADIUS * 2;
        turnRadius1 = (int) robot.MAX_TURNING_RADIUS;

        System.out.println("-----GENERATING PATH-----");
        int i = 1;
        while (i < workingPath.size()) {
            // sp = source point (absolute point)
            sp = (MyPoint) this.plannedPath.get(this.plannedPath.size() - 1).clone();
            if (sp.getDirection() == Direction.NONE) {
                sp = (MyPoint) this.plannedPath.get(this.plannedPath.size() - 2).clone();
            }
            if (sp.getDirection() == Direction.NONE) {
                sp = (MyPoint) this.plannedPath.get(this.plannedPath.size() - 3).clone();
            }
            if (sp.getDirection() == Direction.NONE) {
                sp = (MyPoint) this.plannedPath.get(this.plannedPath.size() - 4).clone();
            }
            if (sp.getDirection() == Direction.NONE) {
                sp = (MyPoint) this.plannedPath.get(this.plannedPath.size() - 5).clone();
            }
            if (sp.getDirection() == Direction.NONE) {
                sp = (MyPoint) this.plannedPath.get(this.plannedPath.size() - 6).clone();
            }
            if (sp.getDirection() == Direction.NONE) {
                sp = (MyPoint) this.plannedPath.get(this.plannedPath.size() - 7).clone();
            }
            if (sp.getDirection() == Direction.NONE) {
                sp = (MyPoint) this.plannedPath.get(this.plannedPath.size() - 8).clone();
            }

            MyPoint point = null;
            point = workingPath.get(i);
            // take the current working point as destination)

            RelativeDirection rDirection = robot.getRelativeDirection(sp, point);
            // get destination point's relative direction from source point
            Direction rOrientation = robot.getRelativeOrientation(sp, point);
            // get destination point's relative orientation from source point

            MyPoint dp = (MyPoint) point.clone();
            // dp = destination point (here is absolute point)

            // System.out.println(sp.getX() + " " + sp.getY());
            // System.out.println(dp.getX() + " " + dp.getY());
            // preparing AffineTransform to use relative points to start point
            AffineTransform af = new AffineTransform();
            af.setToIdentity();
            af.translate(sp.getX(), sp.getY());
            switch (sp.getDirection()) {
                case NORTH:
                    af.rotate(Math.toRadians(0));
                    break;
                case SOUTH:
                    af.rotate(Math.toRadians(180));
                    break;
                case EAST:
                    af.rotate(Math.toRadians(90));
                    break;
                case WEST:
                    af.rotate(Math.toRadians(-90));
                    break;
                case NONE:
                    break;
                default:
                    break;

            }

            // dp becomes a relative point to origin
            try {
                af.inverseTransform(dp, dp);
            } catch (NoninvertibleTransformException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
                logger.log(Level.SEVERE, "Cannot transform");
            }

            System.out.println(rDirection + " " + rOrientation);
            switch (rDirection) {
                // ignore collision detection for the rest for now

                case FRONT: // DONE (no c a)
                    // need to account for destination direction
                    switch (rOrientation) { // DONE (no collision avoidance)
                        case NORTH: { // DONE (with some collision avoidance)
                            // idea: move forward only
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { sp, fp };

                            dp.move(0, (int) dp.getY());
                            af.transform(dp, fp);

                            if (!grid.checkIfPathCollides(tpArray)) {
                                this.plannedPath.add(fp);
                                break;
                            }

                            if (grid.checkIfNeedReverse(sp, turnRadius1)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, turnRadius1);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }

                            // turn right
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray2 = { sp, tp1, tp2 };

                            tp1.move(0, -turnRadius1);
                            tp2.move(turnRadius1, -turnRadius1);
                            tp1.rotateRight90();
                            tp2.rotateRight90();
                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            if (!grid.checkIfPathCollides(tpArray2)) {
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                i--;
                                break;
                            }

                            // turn left
                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());

                            tp1.move(0, -turnRadius1);
                            tp2.move(-turnRadius1, -turnRadius1);
                            tp1.rotateRight90();
                            tp2.rotateRight90();
                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            if (!grid.checkIfPathCollides(tpArray2)) {
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                i--;
                                break;
                            }

                            System.out.println("Collision detected at FRONT + NORTH");
                            isValid = false;
                            break;
                        }
                        case SOUTH: { // DONE (no collision avoidance)
                            // idea: 4 turns (right, left, left, left)
                            if (grid.checkIfNeedReverse(sp, turnRadius1)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, turnRadius1);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp4 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { sp, tp1, tp2, tp3, tp4, fp };

                            // try for first turn to be right turn
                            tp1.rotateRight90(); // face right
                            // tp2 already facing straight
                            tp3.rotateLeft90(); // face left
                            tp4.rotate180(); // face backwards

                            tp1.move(0, -turnRadius1);
                            tp2.move(dp.x + turnRadius1, -turnRadius1);
                            tp3.move(dp.x + turnRadius1, (int) dp.getY() - turnRadius1);
                            tp4.move(dp.x, (int) dp.getY() - turnRadius1);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(tp3, tp3);
                            af.transform(tp4, tp4);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                this.plannedPath.add(tp4);
                                this.plannedPath.add(fp);
                                break;
                            }
                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            tp3 = new MyPoint(0, 0, sp.getDirection());
                            tp4 = new MyPoint(0, 0, sp.getDirection());
                            fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray2 = { sp, tp1, tp2, tp3, tp4, fp };
                            // try turn left first instead
                            tp1.rotateLeft90(); // face right
                            // tp2 already facing straight
                            tp3.rotateRight90(); // face left
                            tp4.rotate180(); // face backwards

                            tp1.move(0, -turnRadius1);
                            tp2.move(dp.x - turnRadius1, -turnRadius1);
                            tp3.move(dp.x - turnRadius1, (int) dp.getY() - turnRadius1);
                            tp4.move(dp.x, (int) dp.getY() - turnRadius1);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(tp3, tp3);
                            af.transform(tp4, tp4);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray2)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                this.plannedPath.add(tp4);
                                this.plannedPath.add(fp);
                                break;
                            }
                            ;
                            // this is where to do collision detection for other cases

                            System.out.println("Collision detected at FRONT + SOUTH");
                            isValid = false;
                            break;
                        }
                        case EAST: { // DONE (no collision avoidance)
                            // idea: 3 turns
                            if (grid.checkIfNeedReverse(sp, turnRadius1)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, turnRadius1);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }
                            if (dp.y < -turnRadius1) {
                                MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                                MyPoint[] tpArray = { sp, tp1, tp2, tp3, fp };

                                // first turn = left
                                tp1.rotateLeft90();
                                tp3.rotateRight90();

                                tp1.move(0, -turnRadius1);
                                tp2.move(-turnRadius1, -turnRadius1);
                                tp3.move(-turnRadius1, (int) dp.getY());

                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                af.transform(tp3, tp3);
                                af.transform(dp, fp);
                                if (!grid.checkIfPathCollides(tpArray)) {
                                    // this path works, go next
                                    this.plannedPath.add(tp1);
                                    this.plannedPath.add(tp2);
                                    this.plannedPath.add(tp3);
                                    this.plannedPath.add(fp);
                                    break;
                                }
                            } else {

                            }

                            // do collision detection
                            System.out.println("Collision detected at FRONT + EAST");
                            isValid = false;
                            break;
                        }
                        case WEST: { // DONE (no collision avoidance)
                            // idea: 3 turns
                            if (grid.checkIfNeedReverse(sp, turnRadius1)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, turnRadius1);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { sp, tp1, tp2, tp3, fp };

                            // first turn = right
                            tp1.rotateRight90();
                            if (-turnRadius1 < dp.getY()) {
                                tp2.rotate180();
                            }
                            tp3.rotateLeft90();
                            //
                            tp1.move(0, -turnRadius1);
                            tp2.move(turnRadius1, -turnRadius1);
                            tp3.move(turnRadius1, (int) dp.getY());

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(tp3, tp3);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                this.plannedPath.add(fp);
                                break;
                            }
                            // do collision detection
                            System.out.println("Collision detected at FRONT + WEST");
                            isValid = false;
                            break;
                        }
                        case NONE:
                            break;
                        default:
                            break;

                    }
                    break;
                case FRONT_RIGHT: // DONE (no c a)
                    switch (rOrientation) {
                        case NORTH: { // DONE (no collision avoidance)
                            // idea: 2 turns
                            if (grid.checkIfNeedReverse(sp, turnRadius1)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, turnRadius1);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }

                            // first turn = right
                            boolean breakFlag = false;
                            for (int yyy = turnRadius1; yyy <= Math.abs(dp.y) - turnRadius1; yyy += caStep) {
                                MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                                MyPoint[] tpArray = { sp, tp1, tp2, fp };
                                tp1.rotateRight90();
                                tp1.move(0, -yyy);
                                tp2.move(dp.x, -yyy);
                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                af.transform(dp, fp);
                                if (!grid.checkIfPathCollides(tpArray)) {
                                    // this path works, go next
                                    this.plannedPath.add(tp1);
                                    this.plannedPath.add(tp2);
                                    this.plannedPath.add(fp);
                                    breakFlag = true;
                                    break;
                                }
                            }
                            if (breakFlag)
                                break;

                            // do collision detection
                            System.out.println("Collision detected at FRONT RIGHT + NORTH");
                            isValid = false;
                            break;
                        }
                        case SOUTH: { // DONE (no c a)
                            // idea: 2 turns
                            // if can't, turn left
                            // if can't, turn right
                            if (grid.checkIfNeedReverse(sp, (int) robot.getTwoTurnsDistance() / 2)) {
                                // reverse, then will be handled by another case
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, (int) robot.getTwoTurnsDistance() / 2);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { sp, tp1, tp2, fp };

                            // first turn = right
                            tp1.rotateRight90();
                            tp2.rotate180();
                            //
                            tp1.move(0, dp.y - turnRadius1);
                            tp2.move(dp.x, dp.y - turnRadius1);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(fp);
                                break;
                            }

                            // left turn
                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray2 = { sp, tp1, tp2 };
                            tp1.rotateLeft90();
                            tp2.rotateLeft90();
                            tp1.move(0, -turnRadius1);
                            tp2.move(-turnRadius1, -turnRadius1);
                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            if (!grid.checkIfPathCollides(tpArray2)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                i--;
                                break;
                            }

                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray3 = { sp, tp1, tp2 };
                            // right turn
                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            tp1.rotateRight90();
                            tp2.rotateRight90();
                            tp1.move(0, -turnRadius1);
                            tp2.move(turnRadius1, -turnRadius1);
                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            if (!grid.checkIfPathCollides(tpArray3)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                i--;
                                break;
                            }

                            // do collision detection
                            System.out.println("Collision detected at FRONT RIGHT + SOUTH");
                            isValid = false;
                            break;
                        }
                        case EAST: { // DONE (no c a)
                            // idea: 1 turn
                            if (grid.checkIfNeedReverse(sp, (int) robot.getTwoTurnsDistance() / 2)) {
                                // reverse, then will be handled by another case
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, (int) robot.getTwoTurnsDistance() / 2);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }

                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { sp, tp1, fp };

                            // first turn = right
                            tp1.rotateRight90();
                            tp1.move(0, dp.y);

                            af.transform(tp1, tp1);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(fp);
                                break;
                            }

                            // right turn
                            if (dp.x >= turnRadius2) {
                                tp1 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint[] tpArray2 = { sp, tp1, tp2 };
                                tp1.rotateRight90();
                                tp2.rotateRight90();
                                tp1.move(0, -turnRadius1);
                                tp2.move(turnRadius1, -turnRadius1);
                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                if (!grid.checkIfPathCollides(tpArray2)) {
                                    // this path works, go next
                                    this.plannedPath.add(tp1);
                                    this.plannedPath.add(tp2);
                                    i--;
                                    break;
                                }
                            }

                            // left turn
                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray2 = { sp, tp1, tp2 };
                            tp1.rotateLeft90();
                            tp2.rotateLeft90();
                            tp1.move(0, -turnRadius1);
                            tp2.move(-turnRadius1, -turnRadius1);
                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            if (!grid.checkIfPathCollides(tpArray2)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                i--;
                                break;
                            }

                            // right turn
                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray3 = { sp, tp1, tp2 };
                            tp1.rotateRight90();
                            tp2.rotateRight90();
                            tp1.move(0, -turnRadius1);
                            tp2.move(turnRadius1, -turnRadius1);
                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            if (!grid.checkIfPathCollides(tpArray3)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                i--;
                                break;
                            }

                            // do collision detection
                            System.out.println("Collision detected at FRONT RIGHT + EAST");
                            isValid = false;
                            break;
                        }
                        case WEST: { // DONE (no c a)
                            // idea: 3 turns
                            if (grid.checkIfNeedReverse(sp, turnRadius1)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, turnRadius1);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }

                            if (dp.y <= -turnRadius2) {
                                MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                                MyPoint[] tpArray = { sp, tp1, tp2, tp3, fp };

                                // first turn = right
                                tp1.rotateRight90();
                                tp3.rotateLeft90();
                                //
                                tp1.move(0, -turnRadius1);
                                tp2.move(dp.x + turnRadius1, -turnRadius1);
                                tp3.move(dp.x + turnRadius1, (int) dp.getY());

                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                af.transform(tp3, tp3);
                                af.transform(dp, fp);
                                if (!grid.checkIfPathCollides(tpArray)) {
                                    // this path works, go next
                                    this.plannedPath.add(tp1);
                                    this.plannedPath.add(tp2);
                                    this.plannedPath.add(tp3);
                                    this.plannedPath.add(fp);
                                    break;
                                }
                            }
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { sp, tp1, tp2, tp3, fp };

                            // first turn = right
                            tp1.rotateRight90();
                            tp2.rotate180();
                            tp3.rotateLeft90();
                            //
                            tp1.move(0, dp.y - turnRadius1);
                            tp2.move(dp.x + turnRadius1, dp.y - turnRadius1);
                            tp3.move(dp.x + turnRadius1, (int) dp.getY());

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(tp3, tp3);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                this.plannedPath.add(fp);
                                break;
                            }

                            // do collision detection
                            System.out.println("Collision detected at FRONT RIGHT + WEST");
                            isValid = false;
                            break;
                        }
                        case NONE:
                            break;
                        default:
                            break;
                    }
                    break;
                case FRONT_LEFT: // DONE (no c a)
                    switch (rOrientation) {
                        case NORTH: { // DONE (collision avoidance)
                            // idea: 2 turns
                            if (grid.checkIfNeedReverse(sp, turnRadius1)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, turnRadius1);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }

                            // first turn = left
                            boolean breakFlag = false;
                            for (int yyy = turnRadius1; yyy <= Math.abs(dp.y) - turnRadius1; yyy += caStep) {
                                MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                                MyPoint[] tpArray = { sp, tp1, tp2, fp };
                                tp1.rotateLeft90();
                                tp1.move(0, -yyy);
                                tp2.move(dp.x, -yyy);
                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                af.transform(dp, fp);
                                if (!grid.checkIfPathCollides(tpArray)) {
                                    // this path works, go next
                                    this.plannedPath.add(tp1);
                                    this.plannedPath.add(tp2);
                                    this.plannedPath.add(fp);
                                    breakFlag = true;
                                    break;
                                }
                            }
                            if (breakFlag)
                                break;

                            // do collision detection
                            System.out.println("Collision detected at FRONT LEFT + NORTH");
                            isValid = false;
                            break;
                        }
                        case SOUTH: { // DONE (no collision avoidance)
                            // idea: 2 turns
                            // if can't, turn right
                            // if can't, turn left
                            if (grid.checkIfNeedReverse(sp, (int) robot.getTwoTurnsDistance() / 2)) {
                                // reverse, then will be handled by another case
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, (int) robot.getTwoTurnsDistance() / 2);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { sp, tp1, tp2, fp };

                            // first turn = left
                            tp1.rotateLeft90();
                            tp2.rotate180();

                            tp1.move(0, dp.y - turnRadius1);
                            tp2.move(dp.x, dp.y - turnRadius1);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(fp);
                                break;
                            }

                            // right turn
                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray2 = { sp, tp1, tp2 };
                            tp1.rotateRight90();
                            tp2.rotateRight90();
                            tp1.move(0, -turnRadius1);
                            tp2.move(turnRadius1, -turnRadius1);
                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            if (!grid.checkIfPathCollides(tpArray2)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                i--;
                                break;
                            }

                            // left turn
                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray3 = { sp, tp1, tp2 };
                            tp1.rotateLeft90();
                            tp2.rotateLeft90();
                            tp1.move(0, -turnRadius1);
                            tp2.move(-turnRadius1, -turnRadius1);
                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            if (!grid.checkIfPathCollides(tpArray3)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                i--;
                                break;
                            }
                            // do collision detection
                            System.out.println("Collision detected at FRONT LEFT + SOUTH");
                            isValid = false;
                            break;
                        }
                        case EAST: { // DONE (no collision avoidance)
                            // idea: 3 turns
                            if (grid.checkIfNeedReverse(sp, turnRadius1)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, turnRadius1);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }

                            if (dp.y <= -turnRadius2) {
                                MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                                MyPoint[] tpArray = { sp, tp1, tp2, tp3, fp };

                                // first turn = left
                                tp1.rotateLeft90();
                                tp3.rotateRight90();
                                //
                                tp1.move(0, -turnRadius1);
                                tp2.move((int) dp.getX() - turnRadius1, -turnRadius1);
                                tp3.move((int) dp.getX() - turnRadius1, (int) dp.getY());

                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                af.transform(tp3, tp3);
                                af.transform(dp, fp);
                                if (!grid.checkIfPathCollides(tpArray)) {
                                    // this path works, go next
                                    this.plannedPath.add(tp1);
                                    this.plannedPath.add(tp2);
                                    this.plannedPath.add(tp3);
                                    this.plannedPath.add(fp);
                                    break;
                                }
                            }

                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { sp, tp1, tp2, tp3, fp };

                            // first turn = left
                            tp1.rotateLeft90();
                            tp2.rotate180();
                            tp3.rotateRight90();
                            //
                            tp1.move(0, dp.y - turnRadius1);
                            tp2.move((int) dp.getX() - turnRadius1, dp.y - turnRadius1);
                            tp3.move((int) dp.getX() - turnRadius1, (int) dp.getY());

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(tp3, tp3);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                this.plannedPath.add(fp);
                                break;
                            }

                            // do collision detection
                            System.out.println("Collision detected at FRONT LEFT + EAST");
                            isValid = false;
                            break;
                        }
                        case WEST: { // DONE (no c a)
                            // idea: 1 turn
                            if (grid.checkIfNeedReverse(sp, (int) robot.getTwoTurnsDistance() / 2)) {
                                // reverse, then will be handled by another case
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, (int) robot.getTwoTurnsDistance() / 2);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { sp, tp1, fp };

                            // first turn = left
                            tp1.rotateLeft90();
                            tp1.move(0, dp.y);

                            af.transform(tp1, tp1);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(fp);
                                break;
                            }

                            // left turn
                            if (dp.x <= -turnRadius2) {
                                tp1 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint[] tpArray2 = { sp, tp1, tp2 };
                                tp1.rotateLeft90();
                                tp2.rotateLeft90();
                                tp1.move(0, -turnRadius1);
                                tp2.move(-turnRadius1, -turnRadius1);
                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                if (!grid.checkIfPathCollides(tpArray2)) {
                                    // this path works, go next
                                    this.plannedPath.add(tp1);
                                    this.plannedPath.add(tp2);
                                    i--;
                                    break;
                                }
                            }

                            // right turn
                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray2 = { sp, tp1, tp2 };
                            tp1.rotateRight90();
                            tp2.rotate180();
                            tp1.move(0, -turnRadius1);
                            tp2.move(turnRadius1, -turnRadius1);
                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            if (!grid.checkIfPathCollides(tpArray2)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                i--;
                                break;
                            }

                            // left turn
                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray3 = { sp, tp1, tp2 };
                            tp1.rotateLeft90();
                            tp2.rotateLeft90();
                            tp1.move(0, -turnRadius1);
                            tp2.move(-turnRadius1, -turnRadius1);
                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            if (!grid.checkIfPathCollides(tpArray3)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                i--;
                                break;
                            }
                            // do collision detection
                            System.out.println("Collision detected at FRONT LEFT + WEST");
                            isValid = false;
                            break;
                        }
                        case NONE:
                            break;
                        default:
                            break;
                    }
                    break;

                case FRONT_SLIGHT_LEFT: // DONE for now
                    switch (rOrientation) {
                        case WEST:
                            if (dp.x <= (int) robot.getTwoTurnsDistance() / 2) {
                                MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                                MyPoint[] tpArray = { sp, tp1, fp };

                                // first turn = left
                                tp1.rotateLeft90();
                                tp1.move(0, dp.y);

                                af.transform(tp1, tp1);
                                af.transform(dp, fp);
                                if (!grid.checkIfPathCollides(tpArray)) {
                                    // this path works, go next
                                    this.plannedPath.add(tp1);
                                    this.plannedPath.add(fp);
                                    break;
                                }
                            }
                        case NORTH:
                        case SOUTH: {
                            // turn left to go to other cases
                            // if not, turn right
                            if (grid.checkIfNeedReverse(sp, turnRadius1)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, turnRadius1);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray = { sp, tp1, tp2 };

                            // first turn = left
                            tp1.rotateLeft90();
                            tp2.rotateLeft90();

                            tp1.move(0, -turnRadius1);
                            tp2.move(-turnRadius1, -turnRadius1);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                i--;
                                break;
                            }

                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray2 = { sp, tp1, tp2 };

                            // first turn = right
                            tp1.rotateRight90();
                            tp2.rotateRight90();
                            tp1.move(0, -turnRadius1);
                            tp2.move(turnRadius1, -turnRadius1);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            if (!grid.checkIfPathCollides(tpArray2)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                i--;
                                break;
                            }
                            // do collision detection
                            System.out.println("Collision detected at FRONT SLIGHT LEFT");
                            isValid = false;
                            break;
                        }
                        case EAST: {
                            // idea: 3 turns (front_left east case)
                            // turn left to go to other cases
                            // if not, turn right
                            if (grid.checkIfNeedReverse(sp, turnRadius1)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, turnRadius1);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }

                            if (dp.y <= -turnRadius2) {
                                MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                                MyPoint[] tpArray = { sp, tp1, tp2, tp3, fp };

                                // first turn = left
                                tp1.rotateLeft90();
                                tp3.rotateRight90();
                                //
                                tp1.move(0, -turnRadius1);
                                tp2.move((int) dp.getX() - turnRadius1, -turnRadius1);
                                tp3.move((int) dp.getX() - turnRadius1, (int) dp.getY());

                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                af.transform(tp3, tp3);
                                af.transform(dp, fp);
                                if (!grid.checkIfPathCollides(tpArray)) {
                                    // this path works, go next
                                    this.plannedPath.add(tp1);
                                    this.plannedPath.add(tp2);
                                    this.plannedPath.add(tp3);
                                    this.plannedPath.add(fp);
                                    break;
                                }
                            }

                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { sp, tp1, tp2, tp3, fp };

                            // first turn = left
                            tp1.rotateLeft90();
                            tp2.rotate180();
                            tp3.rotateRight90();
                            //
                            tp1.move(0, dp.y - turnRadius1);
                            tp2.move((int) dp.getX() - turnRadius1, dp.y - turnRadius1);
                            tp3.move((int) dp.getX() - turnRadius1, (int) dp.getY());

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(tp3, tp3);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                this.plannedPath.add(fp);
                                break;
                            }

                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray2 = { sp, tp1, tp2 };

                            // first turn = left
                            tp1.rotateLeft90();
                            tp2.rotateLeft90();

                            tp1.move(0, -turnRadius1);
                            tp2.move(-turnRadius1, -turnRadius1);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            if (!grid.checkIfPathCollides(tpArray2)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                i--;
                                break;
                            }

                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray3 = { sp, tp1, tp2 };

                            // first turn = right
                            tp1.rotateRight90();
                            tp2.rotateRight90();
                            tp1.move(0, -turnRadius1);
                            tp2.move(turnRadius1, -turnRadius1);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            if (!grid.checkIfPathCollides(tpArray3)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                i--;
                                break;
                            }
                            // do collision detection
                            System.out.println("Collision detected at FRONT SLIGHT LEFT + EAST");
                            isValid = false;
                            break;
                        }
                        case NONE:
                            break;
                        default:
                            break;

                    }
                    break;
                case FRONT_SLIGHT_RIGHT: // DONE for now
                    switch (rOrientation) {
                        case EAST:
                            if (dp.x >= (int) robot.getTwoTurnsDistance() / 2) {
                                MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                                MyPoint[] tpArray = { sp, tp1, fp };

                                // first turn = right
                                tp1.rotateRight90();
                                tp1.move(0, dp.y);

                                af.transform(tp1, tp1);
                                af.transform(dp, fp);
                                if (!grid.checkIfPathCollides(tpArray)) {
                                    // this path works, go next
                                    this.plannedPath.add(tp1);
                                    this.plannedPath.add(fp);
                                    break;
                                }
                            }
                        case NORTH:
                        case SOUTH: {
                            // turn right to go other case
                            // else turn left
                            if (grid.checkIfNeedReverse(sp, turnRadius1)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, turnRadius1);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray = { sp, tp1, tp2 };

                            // first turn = right
                            tp1.rotateRight90();
                            tp2.rotateRight90();
                            tp1.move(0, -turnRadius1);
                            tp2.move(turnRadius1, -turnRadius1);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                i--;
                                break;
                            }

                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray2 = { sp, tp1, tp2 };

                            // first turn = left
                            tp1.rotateLeft90();
                            tp2.rotateLeft90();
                            //

                            tp1.move(0, -turnRadius1);
                            tp2.move(-turnRadius1, -turnRadius1);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            if (!grid.checkIfPathCollides(tpArray2)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                i--;
                                break;
                            }
                            System.out.println("Collision detected at FRONT SLIGHT RIGHT");
                            isValid = false;
                            break;
                        }
                        case WEST: {
                            // idea: 3 turns (similar to front right)
                            // else turn right to go other case
                            // else turn left
                            if (grid.checkIfNeedReverse(sp, turnRadius1)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, turnRadius1);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }
                            if (dp.y <= -turnRadius2) {
                                MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                                MyPoint[] tpArray = { sp, tp1, tp2, tp3, fp };

                                // first turn = right
                                tp1.rotateRight90();
                                tp3.rotateLeft90();
                                //
                                tp1.move(0, -turnRadius1);
                                tp2.move(dp.x + turnRadius1, -turnRadius1);
                                tp3.move(dp.x + turnRadius1, (int) dp.getY());

                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                af.transform(tp3, tp3);
                                af.transform(dp, fp);
                                if (!grid.checkIfPathCollides(tpArray)) {
                                    // this path works, go next
                                    this.plannedPath.add(tp1);
                                    this.plannedPath.add(tp2);
                                    this.plannedPath.add(tp3);
                                    this.plannedPath.add(fp);
                                    break;
                                }
                            }
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { sp, tp1, tp2, tp3, fp };

                            // first turn = right
                            tp1.rotateRight90();
                            tp2.rotate180();
                            tp3.rotateLeft90();
                            //
                            tp1.move(0, dp.y - turnRadius1);
                            tp2.move(dp.x + turnRadius1, dp.y - turnRadius1);
                            tp3.move(dp.x + turnRadius1, (int) dp.getY());

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(tp3, tp3);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                this.plannedPath.add(fp);
                                break;
                            }

                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray2 = { sp, tp1, tp2 };

                            // first turn = right
                            tp1.rotateRight90();
                            tp2.rotateRight90();
                            tp1.move(0, -turnRadius1);
                            tp2.move(turnRadius1, -turnRadius1);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            if (!grid.checkIfPathCollides(tpArray2)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                i--;
                                break;
                            }

                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray3 = { sp, tp1, tp2 };

                            // first turn = left
                            tp1.rotateLeft90();
                            tp2.rotateLeft90();
                            //

                            tp1.move(0, -turnRadius1);
                            tp2.move(-turnRadius1, -turnRadius1);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            if (!grid.checkIfPathCollides(tpArray3)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                i--;
                                break;
                            }
                            // do collision detection
                            System.out.println("Collision detected at FRONT SLIGHT RIGHT + WEST");
                            isValid = false;
                            break;
                        }
                        case NONE:
                            break;
                        default:
                            break;

                    }
                    break;
                case BACK:
                    switch (rOrientation) {
                        case NORTH: { // DONE (no collision avoidance)
                            // we try 3 ways to see if it works
                            // 1. 4 turns, R R R R
                            // 2. 4 turns, L L L L
                            // 3. make 180 turn, then let other case handle
                            if (grid.checkIfNeedReverse(sp, turnRadius1)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, turnRadius1);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }

                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp4 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { sp, tp1, tp2, tp3, tp4, fp };

                            // first turn = right
                            tp1.rotateRight90();
                            tp2.rotate180();
                            tp3.rotateLeft90();

                            tp1.move(0, -turnRadius1);
                            tp2.move(turnRadius2, -turnRadius1);
                            tp3.move(turnRadius2, dp.y + turnRadius1);
                            tp4.move(0, dp.y + turnRadius1);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(tp3, tp3);
                            af.transform(tp4, tp4);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                this.plannedPath.add(tp4);
                                this.plannedPath.add(fp);
                                break;
                            }

                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            tp3 = new MyPoint(0, 0, sp.getDirection());
                            tp4 = new MyPoint(0, 0, sp.getDirection());
                            fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray2 = { sp, tp1, tp2, tp3, tp4, fp };

                            // first turn = left
                            tp1.rotateLeft90();
                            tp2.rotate180();
                            tp3.rotateRight90();
                            //
                            tp1.move(0, -turnRadius1);
                            tp2.move(-turnRadius2, -turnRadius1);
                            tp3.move(-turnRadius2, dp.y + turnRadius1);
                            tp4.move(0, dp.y + turnRadius1);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(tp3, tp3);
                            af.transform(tp4, tp4);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray2)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                this.plannedPath.add(tp4);
                                this.plannedPath.add(fp);
                                break;
                            }

                            // make 180 turn (Reverse, Right, Reverse, Right)
                            // Reverse already done above
                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                            tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray3 = { sp, tp1, tp2, rp, tp3 };

                            tp1.move(0, -turnRadius1);
                            tp1.rotateRight90();
                            tp2.move(turnRadius1, -turnRadius1);
                            tp2.rotateRight90();
                            rp.move(-turnRadius1, -turnRadius1);
                            rp.rotateRight90();
                            tp3.move(0, -turnRadius1);
                            tp3.rotate180();
                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(rp, rp);
                            af.transform(tp3, tp3);
                            if (!grid.checkIfPathCollides(tpArray3)) {
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(rp);
                                this.plannedPath.add(tp3);
                                i--;
                                break;
                            }
                            System.out.println("Collision detected at BACK + NORTH");
                            isValid = false;
                            break;
                        }
                        case SOUTH: { // DONE (no collision avoidance)
                            // we try 3 ways to see if it works
                            // 1. 4 turns, R R R L
                            // 2. 4 turns, L L L R
                            // 3. make 180 turn, then let other case handle
                            if (grid.checkIfNeedReverse(sp, turnRadius1)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, turnRadius1);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }

                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2_3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2__3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp4 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { sp, tp1, tp2, tp2_3, tp2__3, tp3, tp4, fp };

                            // first turn = right
                            tp1.rotateRight90();
                            tp2.rotate180();
                            tp2_3.rotate180();
                            tp2__3.rotate180();
                            tp3.rotateLeft90();
                            tp4.rotate180();

                            tp1.move(0, -turnRadius1);
                            tp2.move(dp.x + turnRadius1, -turnRadius1);
                            tp2_3.move(dp.x + turnRadius1, 0);
                            tp2__3.move(dp.x + turnRadius1, dp.y - turnRadius2);
                            tp3.move(dp.x + turnRadius1, dp.y - turnRadius1);
                            tp4.move(dp.x, dp.y - turnRadius1);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(tp2_3, tp2_3);
                            af.transform(tp2__3, tp2__3);
                            af.transform(tp3, tp3);
                            af.transform(tp4, tp4);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp2_3);
                                this.plannedPath.add(tp2__3);
                                this.plannedPath.add(tp3);
                                this.plannedPath.add(tp4);
                                this.plannedPath.add(fp);
                                break;
                            }
                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            tp2_3 = new MyPoint(0, 0, sp.getDirection());
                            tp2__3 = new MyPoint(0, 0, sp.getDirection());
                            tp3 = new MyPoint(0, 0, sp.getDirection());
                            tp4 = new MyPoint(0, 0, sp.getDirection());
                            fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray2 = { sp, tp1, tp2, tp2_3, tp2__3, tp3, tp4, fp };

                            // first turn = left
                            tp1.rotateLeft90();
                            tp2.rotate180();
                            tp2_3.rotate180();
                            tp2__3.rotate180();
                            tp3.rotateRight90();
                            tp4.rotate180();
                            //
                            tp1.move(0, -turnRadius1);
                            tp2.move(dp.x - turnRadius1, -turnRadius1);
                            tp2_3.move(dp.x - turnRadius1, 0);
                            tp2__3.move(dp.x - turnRadius1, dp.y - turnRadius2);
                            tp3.move(dp.x - turnRadius1, dp.y - turnRadius1);
                            tp4.move(dp.x, dp.y - turnRadius1);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(tp2_3, tp2_3);
                            af.transform(tp2__3, tp2__3);
                            af.transform(tp3, tp3);
                            af.transform(tp4, tp4);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray2)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp2_3);
                                this.plannedPath.add(tp2__3);
                                this.plannedPath.add(tp3);
                                this.plannedPath.add(tp4);
                                this.plannedPath.add(fp);
                                break;
                            }

                            // make 180 turn (Reverse, Right, Reverse, Right)
                            // Reverse already done above
                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                            tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray3 = { sp, tp1, tp2, rp, tp3 };

                            tp1.move(0, -turnRadius1);
                            tp1.rotateRight90();
                            tp2.move(turnRadius1, -turnRadius1);
                            tp2.rotateRight90();
                            rp.move(-turnRadius1, -turnRadius1);
                            rp.rotateRight90();
                            tp3.move(0, -turnRadius1);
                            tp3.rotate180();
                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(rp, rp);
                            af.transform(tp3, tp3);
                            if (!grid.checkIfPathCollides(tpArray3)) {
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(rp);
                                this.plannedPath.add(tp3);
                                i--;
                                break;
                            }
                            System.out.println("Collision detected at BACK + SOUTH");
                            isValid = false;
                            break;
                        }
                        case EAST: { // DONE (no collision avoidance)
                            // we try 2 ways to see if it works
                            // 1. 3 turns, L L L
                            // 2. make 180 turn, then let other case handle
                            if (grid.checkIfNeedReverse(sp, turnRadius1)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, turnRadius1);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }

                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { sp, tp1, tp2, tp3, fp };

                            // first turn = left
                            tp1.rotateLeft90();
                            tp2.rotate180();
                            tp3.rotateRight90();

                            tp1.move(0, -turnRadius1);
                            tp2.move(-turnRadius2, -turnRadius1);
                            tp3.move(-turnRadius2, dp.y);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(tp3, tp3);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                this.plannedPath.add(fp);
                                break;
                            }

                            // make 180 turn (Reverse, Right, Reverse, Right)
                            // Reverse already done above
                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                            tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray2 = { sp, tp1, tp2, rp, tp3 };

                            tp1.move(0, -turnRadius1);
                            tp1.rotateRight90();
                            tp2.move(turnRadius1, -turnRadius1);
                            tp2.rotateRight90();
                            rp.move(-turnRadius1, -turnRadius1);
                            rp.rotateRight90();
                            tp3.move(0, -turnRadius1);
                            tp3.rotate180();
                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(rp, rp);
                            af.transform(tp3, tp3);
                            if (!grid.checkIfPathCollides(tpArray2)) {
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(rp);
                                this.plannedPath.add(tp3);
                                i--;
                                break;
                            }
                            System.out.println("Collision detected at BACK + EAST");
                            isValid = false;
                            break;
                        }
                        case WEST: { // DONE (no collision avoidance)
                            // we try 2 ways to see if it works
                            // 1. 3 turns, R R R
                            // 2. make 180 turn, then let other case handle
                            if (grid.checkIfNeedReverse(sp, turnRadius1)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, turnRadius1);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }

                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { sp, tp1, tp2, tp3, fp };

                            // first turn = right
                            tp1.rotateRight90();
                            tp2.rotate180();
                            tp3.rotateLeft90();

                            tp1.move(0, -turnRadius1);
                            tp2.move(turnRadius2, -turnRadius1);
                            tp3.move(turnRadius2, dp.y);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(tp3, tp3);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                this.plannedPath.add(fp);
                                break;
                            }

                            // make 180 turn (Reverse, Right, Reverse, Right)
                            // Reverse already done above
                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                            tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray2 = { sp, tp1, tp2, rp, tp3 };

                            tp1.move(0, -turnRadius1);
                            tp1.rotateRight90();
                            tp2.move(turnRadius1, -turnRadius1);
                            tp2.rotateRight90();
                            rp.move(-turnRadius1, -turnRadius1);
                            rp.rotateRight90();
                            tp3.move(0, -turnRadius1);
                            tp3.rotate180();
                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(rp, rp);
                            af.transform(tp3, tp3);
                            if (!grid.checkIfPathCollides(tpArray2)) {
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(rp);
                                this.plannedPath.add(tp3);
                                i--;
                                break;
                            }
                            System.out.println("Collision detected at BACK + WEST");
                            isValid = false;
                            break;
                        }
                        case NONE:
                            break;
                        default:
                            break;

                    }
                    break;
                case BACK_RIGHT:
                    switch (rOrientation) {
                        case NORTH: { // DONE (no c a)
                            // we try 3 ways to see if it works (similar to BACK NORTH)
                            // 1. 4 turns, R R R R
                            // 2. 4 turns, L L L L
                            // 3. make 180 turn, then let other case handle
                            if (grid.checkIfNeedReverse(sp, turnRadius1)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, turnRadius1);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }

                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp4 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { sp, tp1, tp2, tp3, tp4, fp };

                            // first turn = right
                            tp1.rotateRight90();
                            tp2.rotate180();
                            tp3.rotateLeft90();

                            tp1.move(0, -turnRadius1);
                            tp2.move(dp.x + turnRadius2, -turnRadius1);
                            tp3.move(dp.x + turnRadius2, dp.y + turnRadius1);
                            tp4.move(dp.x, dp.y + turnRadius1);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(tp3, tp3);
                            af.transform(tp4, tp4);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                this.plannedPath.add(tp4);
                                this.plannedPath.add(fp);
                                break;
                            }

                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            tp3 = new MyPoint(0, 0, sp.getDirection());
                            tp4 = new MyPoint(0, 0, sp.getDirection());
                            fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray2 = { sp, tp1, tp2, tp3, tp4, fp };

                            // first turn = left
                            tp1.rotateLeft90();
                            tp2.rotate180();
                            tp3.rotateRight90();
                            //
                            tp1.move(0, -turnRadius1);
                            tp2.move(-turnRadius2, -turnRadius1);
                            tp3.move(-turnRadius2, dp.y + turnRadius1);
                            tp4.move(dp.x, dp.y + turnRadius1);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(tp3, tp3);
                            af.transform(tp4, tp4);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray2)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                this.plannedPath.add(tp4);
                                this.plannedPath.add(fp);
                                break;
                            }

                            // make 180 turn (Reverse, Right, Reverse, Right)
                            // Reverse already done above
                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                            tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray3 = { sp, tp1, tp2, rp, tp3 };

                            tp1.move(0, -turnRadius1);
                            tp1.rotateRight90();
                            tp2.move(turnRadius1, -turnRadius1);
                            tp2.rotateRight90();
                            rp.move(-turnRadius1, -turnRadius1);
                            rp.rotateRight90();
                            tp3.move(0, -turnRadius1);
                            tp3.rotate180();
                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(rp, rp);
                            af.transform(tp3, tp3);
                            if (!grid.checkIfPathCollides(tpArray3)) {
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(rp);
                                this.plannedPath.add(tp3);
                                i--;
                                break;
                            }
                            System.out.println("Collision detected at BACK RIGHT + NORTH");
                            isValid = false;
                            break;
                        }
                        case SOUTH: {
                            // idea: 2 turns R R
                            if (grid.checkIfNeedReverse(sp, turnRadius1)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, turnRadius1);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }

                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { sp, tp1, tp2, fp };

                            // first turn = right
                            tp1.rotateRight90();
                            tp2.rotate180();

                            tp1.move(0, -turnRadius1);
                            tp2.move(dp.x, -turnRadius1);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(fp);
                                break;
                            }

                            System.out.println("Collision detected at BACK RIGHT + SOUTH");
                            isValid = false;
                            break;
                        }
                        case EAST: {
                            // idea: 3 turns L L L
                            // or R then pass
                            if (grid.checkIfNeedReverse(sp, turnRadius1)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, turnRadius1);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }

                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { sp, tp1, tp2, tp3, fp };

                            // first turn = left
                            tp1.rotateLeft90();
                            tp2.rotate180();
                            tp3.rotateRight90();

                            tp1.move(0, -turnRadius1);
                            tp2.move(-turnRadius2, -turnRadius1);
                            tp3.move(-turnRadius2, dp.y);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(tp3, tp3);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                this.plannedPath.add(fp);
                                break;
                            }

                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray2 = { sp, tp1, tp2, tp3 };
                            tp1.rotateRight90();
                            tp2.rotateRight90();
                            tp3.rotateRight90();
                            tp1.move(0, -turnRadius1);
                            tp2.move(turnRadius1, -turnRadius1);
                            tp3.move(0, -turnRadius1);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(tp3, tp3);
                            if (!grid.checkIfPathCollides(tpArray2)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                i--;
                                break;
                            }
                            System.out.println("Collision detected at BACK RIGHT + EAST");
                            isValid = false;
                            break;
                        }
                        case WEST: {
                            // idea: 3 turns R R R
                            if (grid.checkIfNeedReverse(sp, turnRadius1)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, turnRadius1);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }

                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { sp, tp1, tp2, tp3, fp };

                            // first turn = right
                            tp1.rotateRight90();
                            tp2.rotate180();
                            tp3.rotateLeft90();

                            tp1.move(0, -turnRadius1);
                            tp2.move(dp.x + turnRadius1, -turnRadius1);
                            tp3.move(dp.x + turnRadius1, dp.y);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(tp3, tp3);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                this.plannedPath.add(fp);
                                break;
                            }

                            System.out.println("Collision detected at BACK RIGHT + WEST");
                            isValid = false;
                            break;
                        }
                        case NONE:
                            break;
                        default:
                            break;

                    }
                    break;
                case BACK_LEFT:
                    switch (rOrientation) {
                        case NORTH: { // DONE (no c a)
                            // we try 3 ways to see if it works (similar to BACK NORTH)
                            // 1. 4 turns, R R R R
                            // 2. 4 turns, L L L L
                            // 3. make 180 turn, then let other case handle
                            if (grid.checkIfNeedReverse(sp, turnRadius1)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, turnRadius1);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }

                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp4 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { sp, tp1, tp2, tp3, tp4, fp };

                            // first turn = left
                            tp1.rotateLeft90();
                            tp2.rotate180();
                            tp3.rotateRight90();

                            tp1.move(0, -turnRadius1);
                            tp2.move(dp.x - turnRadius2, -turnRadius1);
                            tp3.move(dp.x - turnRadius2, dp.y + turnRadius1);
                            tp4.move(dp.x, dp.y + turnRadius1);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(tp3, tp3);
                            af.transform(tp4, tp4);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                this.plannedPath.add(tp4);
                                this.plannedPath.add(fp);
                                break;
                            }

                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            tp3 = new MyPoint(0, 0, sp.getDirection());
                            tp4 = new MyPoint(0, 0, sp.getDirection());
                            fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray2 = { sp, tp1, tp2, tp3, tp4, fp };

                            // first turn = right
                            tp1.rotateRight90();
                            tp2.rotate180();
                            tp3.rotateLeft90();
                            //
                            tp1.move(0, -turnRadius1);
                            tp2.move(turnRadius2, -turnRadius1);
                            tp3.move(turnRadius2, dp.y + turnRadius1);
                            tp4.move(dp.x, dp.y + turnRadius1);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(tp3, tp3);
                            af.transform(tp4, tp4);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray2)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                this.plannedPath.add(tp4);
                                this.plannedPath.add(fp);
                                break;
                            }

                            // make 180 turn (Reverse, Right, Reverse, Right)
                            // Reverse already done above
                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                            tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray3 = { sp, tp1, tp2, rp, tp3 };

                            tp1.move(0, -turnRadius1);
                            tp1.rotateRight90();
                            tp2.move(turnRadius1, -turnRadius1);
                            tp2.rotateRight90();
                            rp.move(-turnRadius1, -turnRadius1);
                            rp.rotateRight90();
                            tp3.move(0, -turnRadius1);
                            tp3.rotate180();
                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(rp, rp);
                            af.transform(tp3, tp3);

                            if (!grid.checkIfPathCollides(tpArray3)) {
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(rp);
                                this.plannedPath.add(tp3);
                                i--;
                                break;
                            }
                            System.out.println("Collision detected at BACK LEFT + NORTH");
                            isValid = false;
                            break;
                        }
                        case SOUTH: {
                            // idea: 2 turns L L
                            if (grid.checkIfNeedReverse(sp, turnRadius1)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, turnRadius1);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }

                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { sp, tp1, tp2, fp };

                            // first turn = right
                            tp1.rotateLeft90();
                            tp2.rotate180();

                            tp1.move(0, -turnRadius1);
                            tp2.move(dp.x, -turnRadius1);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(fp);
                                break;
                            }

                            System.out.println("Collision detected at BACK LEFT + SOUTH");
                            isValid = false;
                            break;
                        }
                        case EAST: {
                            // idea: 3 turns L L L
                            if (grid.checkIfNeedReverse(sp, turnRadius1)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, turnRadius1);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }

                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { sp, tp1, tp2, tp3, fp };

                            // first turn = left
                            tp1.rotateLeft90();
                            tp2.rotate180();
                            tp3.rotateRight90();

                            tp1.move(0, -turnRadius1);
                            tp2.move(dp.x - turnRadius1, -turnRadius1);
                            tp3.move(dp.x - turnRadius1, dp.y);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(tp3, tp3);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                this.plannedPath.add(fp);
                                break;
                            }

                            System.out.println("Collision detected at BACK LEFT + EAST");
                            isValid = false;
                            break;
                        }
                        case WEST: {
                            // idea: 3 turns R R R
                            // or L then pass
                            if (grid.checkIfNeedReverse(sp, turnRadius1)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, turnRadius1);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }

                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { sp, tp1, tp2, tp3, fp };

                            // first turn = right
                            tp1.rotateRight90();
                            tp2.rotate180();
                            tp3.rotateLeft90();

                            tp1.move(0, -turnRadius1);
                            tp2.move(turnRadius2, -turnRadius1);
                            tp3.move(turnRadius2, dp.y);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(tp3, tp3);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                this.plannedPath.add(fp);
                                break;
                            }

                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray2 = { sp, tp1, tp2, tp3 };
                            tp1.rotateLeft90();
                            tp2.rotateLeft90();
                            tp3.rotateLeft90();
                            tp1.move(0, -turnRadius1);
                            tp2.move(-turnRadius1, -turnRadius1);
                            tp3.move(0, -turnRadius1);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(tp3, tp3);
                            if (!grid.checkIfPathCollides(tpArray2)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                i--;
                                break;
                            }

                            System.out.println("Collision detected at BACK LEFT + WEST");
                            isValid = false;
                            break;
                        }
                        case NONE:
                            break;
                        default:
                            break;

                    }
                    break;
                case BACK_SLIGHT_LEFT: // DONE for now
                    switch (rOrientation) {
                        // turn left to go to other cases
                        case NORTH:
                        case SOUTH:
                        case EAST:
                        case WEST:
                            if (grid.checkIfNeedReverse(sp, turnRadius1)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, turnRadius1);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray = { sp, tp1, tp2 };

                            // first turn = left
                            tp1.rotateLeft90();
                            tp2.rotateLeft90();

                            tp1.move(0, -turnRadius1);
                            tp2.move(-turnRadius1, -turnRadius1);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                i--;
                                break;
                            }
                            // do collision detection
                            System.out.println("Collision detected at FRONT SLIGHT LEFT");
                            isValid = false;
                            break;
                        case NONE:
                            break;
                        default:
                            break;

                    }
                    break;
                case BACK_SLIGHT_RIGHT: // DONE for now
                    switch (rOrientation) {
                        case NORTH:
                        case SOUTH:
                        case EAST:
                        case WEST:
                            if (grid.checkIfNeedReverse(sp, turnRadius1)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, turnRadius1);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray = { sp, tp1, tp2 };

                            // first turn = right
                            tp1.rotateRight90();
                            tp2.rotateRight90();

                            tp1.move(0, -turnRadius1);
                            tp2.move(turnRadius1, -turnRadius1);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                i--;
                                break;
                            }
                            // do collision detection
                            System.out.println("Collision detected at BACK SLIGHT RIGHT");
                            isValid = false;
                            break;
                        case NONE:
                            break;
                        default:
                            break;

                    }
                    break;
                case CENTER_RIGHT:
                    switch (rOrientation) {
                        case NORTH: {
                            if (grid.checkIfNeedReverse(sp, (int) robot.getTwoTurnsDistance() / 2)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, (int) robot.getTwoTurnsDistance() / 2);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp4 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray0 = { sp, tp1, tp2, tp3, tp4, fp };

                            // first turn = right
                            tp1.rotateRight90();
                            tp2.rotate180();
                            tp3.rotateLeft90();

                            tp1.move(0, -turnRadius1);
                            tp2.move(dp.x + turnRadius2, -turnRadius1);
                            tp3.move(dp.x + turnRadius2, dp.y + turnRadius1);
                            tp4.move(dp.x, dp.y + turnRadius1);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(tp3, tp3);
                            af.transform(tp4, tp4);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray0)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                this.plannedPath.add(tp4);
                                this.plannedPath.add(fp);
                                break;
                            }

                            // try to reverse till it is in top left
                            MyPoint tp = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray = { sp, tp };

                            tp.move(0, turnRadius2 + dp.y);

                            af.transform(tp, tp);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp);
                                i--;
                                break;
                            }
                            // if cannot, move forward till in bottom left
                            tp = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray2 = { sp, tp };

                            tp.move(0, -turnRadius2 + dp.y);

                            af.transform(tp, tp);
                            if (!grid.checkIfPathCollides(tpArray2)) {
                                // this path works, go next
                                this.plannedPath.add(tp);
                                i--;
                                break;
                            }
                            System.out.println("Collision detected at CENTER RIGHT + NORTH");
                            isValid = false;
                            break;
                        }
                        case SOUTH: {
                            // move forward and turn right till in top right
                            // else turn right
                            // else turn left
                            if (grid.checkIfNeedReverse(sp, turnRadius1)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, turnRadius1);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray = { sp, tp1, tp2, tp3 };

                            tp1.move(0, dp.y);
                            tp2.move(0, dp.y - turnRadius1);
                            tp3.move(turnRadius1, dp.y - turnRadius1);
                            tp2.rotateRight90();
                            tp3.rotateRight90();

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(tp3, tp3);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                i--;
                                break;
                            }
                            // turn right
                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray2 = { sp, tp1, tp2 };

                            tp1.move(0, -turnRadius1);
                            tp2.move(turnRadius1, -turnRadius1);
                            tp1.rotateRight90();
                            tp2.rotateRight90();

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            if (!grid.checkIfPathCollides(tpArray2)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                i--;
                                break;
                            }
                            // turn left
                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray3 = { sp, tp1, tp2 };

                            tp1.move(0, -turnRadius1);
                            tp2.move(-turnRadius1, -turnRadius1);
                            tp1.rotateLeft90();
                            tp2.rotateLeft90();

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            if (!grid.checkIfPathCollides(tpArray3)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                i--;
                                break;
                            }
                            System.out.println("Collision detected at CENTER RIGHT + SOUTH");
                            isValid = false;
                            break;
                        }
                        case EAST: {
                            // idea: if far enough left, reverse enough and turn right to pass to FORWARD
                            // if cannot, turn right where will be front right
                            // or just turn right
                            // or do 180 turn (try right)
                            // or do 3 R 1 L???
                            // else if too close, turn left
                            // else, reverse and pass to front slight right
                            if (dp.x >= turnRadius1) {
                                if (grid.checkIfNeedReverse(sp, (int) robot.getTwoTurnsDistance() / 2)) {
                                    MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                    rp.move(0, (int) robot.getTwoTurnsDistance() / 2);
                                    af.transform(rp, rp);
                                    this.plannedPath.add(rp);
                                    i--; // minus so that it does not go to the next workingPath point
                                    break;
                                }
                                MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint[] tpArray = { sp, tp1, tp2, tp3 };

                                tp1.move(0, (dp.y + turnRadius1));
                                tp2.move(0, dp.y);
                                tp3.move(turnRadius1, dp.y);
                                tp2.rotateRight90();
                                tp3.rotateRight90();

                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                af.transform(tp3, tp3);
                                if (!grid.checkIfPathCollides(tpArray)) {
                                    // this path works, go next
                                    this.plannedPath.add(tp1);
                                    this.plannedPath.add(tp2);
                                    this.plannedPath.add(tp3);
                                    i--;
                                    break;
                                }
                                // right turn in front enough to be front right
                                MyPoint tp0 = new MyPoint(0, 0, sp.getDirection());
                                tp1 = new MyPoint(0, 0, sp.getDirection());
                                tp2 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint[] temp = { sp, tp1, tp2 };

                                tp0.move(0, dp.y - turnRadius2 - 1 + turnRadius1);
                                tp1.move(0, dp.y - turnRadius2 - 1);
                                tp2.move(turnRadius1, dp.y - turnRadius2 - 1);
                                tp1.rotateRight90();
                                tp2.rotateRight90();

                                af.transform(tp0, tp0);
                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                if (!grid.checkIfPathCollides(temp)) {
                                    // this path works, go next
                                    this.plannedPath.add(tp0);
                                    this.plannedPath.add(tp1);
                                    this.plannedPath.add(tp2);
                                    i--;
                                    break;
                                }
                                // just turn right
                                tp1 = new MyPoint(0, 0, sp.getDirection());
                                tp2 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint[] temp2 = { sp, tp1, tp2 };

                                tp1.move(0, -turnRadius1);
                                tp2.move(turnRadius1, -turnRadius1);
                                tp1.rotateRight90();
                                tp2.rotateRight90();

                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                if (!grid.checkIfPathCollides(temp2)) {
                                    // this path works, go next
                                    this.plannedPath.add(tp1);
                                    this.plannedPath.add(tp2);
                                    i--;
                                    break;
                                }
                                // 180 turn R
                                tp1 = new MyPoint(0, 0, sp.getDirection());
                                tp2 = new MyPoint(0, 0, sp.getDirection());
                                tp3 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp4 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint[] tpArray2 = { sp, tp1, tp2, tp3, tp4 };

                                tp1.move(0, dp.y - turnRadius2);
                                tp1.rotateRight90();
                                tp2.move(turnRadius1, dp.y - turnRadius2);
                                tp2.rotateRight90();
                                tp3.move(-turnRadius1, dp.y - turnRadius2);
                                tp3.rotateRight90();
                                tp4.move(0, dp.y - turnRadius2);
                                tp4.rotate180();

                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                af.transform(tp3, tp3);
                                af.transform(tp4, tp4);
                                if (!grid.checkIfPathCollides(tpArray2)) {
                                    // this path works, go next
                                    this.plannedPath.add(tp1);
                                    this.plannedPath.add(tp2);
                                    this.plannedPath.add(tp3);
                                    this.plannedPath.add(tp4);
                                    i--;
                                    break;
                                }

                                // this is the RRRL 180 turn move X_X
                                tp1 = new MyPoint(0, 0, sp.getDirection());
                                tp2 = new MyPoint(0, 0, sp.getDirection());
                                tp3 = new MyPoint(0, 0, sp.getDirection());
                                tp4 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp1f = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp2f = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp3f = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp4f = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp1r = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp2r = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp3r = new MyPoint(0, 0, sp.getDirection());
                                MyPoint[] tpArray3 = { sp, tp1, tp1f, tp1r, tp2, tp2f, tp2r, tp3, tp3f, tp3r, tp4,
                                        tp4f };

                                tp1.move(0, dp.y - turnRadius2);
                                tp1.rotateRight90();
                                tp1f.move(turnRadius1, dp.y - turnRadius2);
                                tp1f.rotateRight90();
                                tp1r.move(0, dp.y - turnRadius2);
                                tp1r.rotateRight90();
                                tp2.move(turnRadius1, dp.y - turnRadius2);
                                tp2.rotate180();
                                tp2f.move(turnRadius1, dp.y - turnRadius2 + turnRadius1);
                                tp2f.rotate180();
                                tp2r.move(turnRadius1, dp.y - turnRadius2);
                                tp2r.rotate180();
                                tp3.move(turnRadius1, dp.y - turnRadius2 + turnRadius1);
                                tp3.rotateLeft90();
                                tp3f.move(0, dp.y - turnRadius2 + turnRadius1);
                                tp3f.rotateLeft90();
                                tp3r.move(turnRadius1, dp.y - turnRadius2 + turnRadius1);
                                tp3r.rotateLeft90();
                                tp4.move(0, dp.y - turnRadius2 + turnRadius1);
                                tp4.rotate180();
                                tp4f.move(0, dp.y);
                                tp4f.rotate180();

                                af.transform(tp1, tp1);
                                af.transform(tp1f, tp1f);
                                af.transform(tp1r, tp1r);
                                af.transform(tp2, tp2);
                                af.transform(tp2f, tp2f);
                                af.transform(tp2r, tp2r);
                                af.transform(tp3, tp3);
                                af.transform(tp3f, tp3f);
                                af.transform(tp3r, tp3r);
                                af.transform(tp4, tp4);
                                af.transform(tp4f, tp4f);
                                if (!grid.checkIfPathCollides(tpArray3)) {
                                    // this path works, go next
                                    this.plannedPath.add(tp1);
                                    this.plannedPath.add(tp1f);
                                    this.plannedPath.add(tp1r);
                                    this.plannedPath.add(tp2);
                                    this.plannedPath.add(tp2f);
                                    this.plannedPath.add(tp2r);
                                    this.plannedPath.add(tp3);
                                    this.plannedPath.add(tp3f);
                                    this.plannedPath.add(tp3r);
                                    this.plannedPath.add(tp4);
                                    this.plannedPath.add(tp4f);
                                    i--;
                                    break;
                                }

                                System.out.println("Collision detected at CENTER RIGHT + EAST 1");
                            } else if (dp.x < turnRadius1) {
                                // turn left (reverse if need)
                                if (grid.checkIfNeedReverse(sp, (int) robot.getTwoTurnsDistance() / 2)) {
                                    MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                    rp.move(0, (int) robot.getTwoTurnsDistance() / 2);
                                    af.transform(rp, rp);
                                    this.plannedPath.add(rp);
                                    i--; // minus so that it does not go to the next workingPath point
                                    break;
                                }
                                MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint[] tpArray = { sp, tp1, tp2 };

                                tp1.move(0, -turnRadius1);
                                tp2.move(-turnRadius1, -turnRadius1);
                                tp1.rotateLeft90();
                                tp2.rotateLeft90();

                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                if (!grid.checkIfPathCollides(tpArray)) {
                                    // this path works, go next
                                    this.plannedPath.add(tp1);
                                    this.plannedPath.add(tp2);
                                    i--;
                                    break;
                                }

                                tp1 = new MyPoint(0, 0, sp.getDirection());
                                tp2 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint[] tpArray2 = { sp, tp1, tp2 };

                                tp1.move(0, -turnRadius1);
                                tp2.move(turnRadius1, -turnRadius1);
                                tp1.rotateRight90();
                                tp2.rotateRight90();

                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                if (!grid.checkIfPathCollides(tpArray2)) {
                                    // this path works, go next
                                    this.plannedPath.add(tp1);
                                    this.plannedPath.add(tp2);
                                    i--;
                                    break;
                                }
                                System.out.println("Collision detected at CENTER RIGHT + EAST 2");
                            } else {
                                MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());

                                tp1.move(0, (dp.y + turnRadius2));

                                af.transform(tp1, tp1);
                                if (!grid.checkIfPointCollides(tp1)) {
                                    // this path works, go next
                                    this.plannedPath.add(tp1);
                                    i--;
                                    break;
                                }
                                System.out.println("Collision detected at CENTER RIGHT + EAST 3");
                            }
                            isValid = false;
                            break;
                        }
                        case WEST: {
                            if (grid.checkIfNeedReverse(sp, (int) robot.getTwoTurnsDistance() / 2)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, (int) robot.getTwoTurnsDistance() / 2);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }
                            // idea: 3 turns to reach dest
                            // else reverse until other case
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { sp, tp1, tp2, tp3, fp };

                            // first turn = right
                            tp1.rotateRight90();
                            tp2.rotate180();
                            tp3.rotateLeft90();
                            //
                            tp1.move(0, dp.y - turnRadius1);
                            tp2.move(dp.x + turnRadius1, dp.y - turnRadius1);
                            tp3.move(dp.x + turnRadius1, (int) dp.getY());

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(tp3, tp3);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                this.plannedPath.add(fp);
                                break;
                            }

                            tp1 = new MyPoint(0, 0, sp.getDirection());

                            tp1.move(0, (dp.y + turnRadius2));

                            af.transform(tp1, tp1);
                            if (!grid.checkIfPointCollides(tp1)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                i--;
                                break;
                            }
                            System.out.println("Collision detected at CENTER RIGHT + WEST");
                            isValid = false;
                            break;
                        }
                        case NONE:
                            break;
                        default:
                            break;

                    }
                    break;
                case CENTER_LEFT:
                    switch (rOrientation) {
                        case NORTH: {
                            if (grid.checkIfNeedReverse(sp, (int) robot.getTwoTurnsDistance() / 2)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, (int) robot.getTwoTurnsDistance() / 2);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }

                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp4 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray0 = { sp, tp1, tp2, tp3, tp4, fp };

                            // first turn = left
                            tp1.rotateLeft90();
                            tp2.rotate180();
                            tp3.rotateRight90();

                            tp1.move(0, -turnRadius1);
                            tp2.move(dp.x - turnRadius2, -turnRadius1);
                            tp3.move(dp.x - turnRadius2, dp.y + turnRadius1);
                            tp4.move(dp.x, dp.y + turnRadius1);

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(tp3, tp3);
                            af.transform(tp4, tp4);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray0)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                this.plannedPath.add(tp4);
                                this.plannedPath.add(fp);
                                break;
                            }
                            // try to reverse till it is in top left
                            MyPoint tp = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray = { sp, tp };

                            tp.move(0, turnRadius2 + dp.y);

                            af.transform(tp, tp);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp);
                                i--;
                                break;
                            }
                            // if cannot, move forward till in bottom left
                            tp = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray2 = { sp, tp };

                            tp.move(0, -turnRadius2 + dp.y);

                            af.transform(tp, tp);
                            if (!grid.checkIfPathCollides(tpArray2)) {
                                // this path works, go next
                                this.plannedPath.add(tp);
                                i--;
                                break;
                            }
                            System.out.println("Collision detected at CENTER LEFT + NORTH");
                            isValid = false;
                            break;
                        }
                        case SOUTH: {
                            // move forward and turn left till in top left
                            // else turn left
                            // else turn right
                            if (grid.checkIfNeedReverse(sp, turnRadius1)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, turnRadius1);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray = { sp, tp1, tp2, tp3 };

                            tp1.move(0, dp.y - turnRadius1);
                            tp2.move(0, dp.y - turnRadius2);
                            tp3.move(-turnRadius1, dp.y - turnRadius2);
                            tp2.rotateLeft90();
                            tp3.rotateLeft90();

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(tp3, tp3);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                i--;
                                break;
                            }
                            // turn left
                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray2 = { sp, tp1, tp2 };

                            tp1.move(0, -turnRadius1);
                            tp2.move(-turnRadius1, -turnRadius1);
                            tp1.rotateLeft90();
                            tp2.rotateLeft90();

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            if (!grid.checkIfPathCollides(tpArray2)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                i--;
                                break;
                            }
                            // turn right
                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray3 = { sp, tp1, tp2 };

                            tp1.move(0, -turnRadius1);
                            tp2.move(turnRadius1, -turnRadius1);
                            tp1.rotateRight90();
                            tp2.rotateRight90();

                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            if (!grid.checkIfPathCollides(tpArray3)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                i--;
                                break;
                            }
                            System.out.println("Collision detected at CENTER LEFT + SOUTH");
                            isValid = false;
                            break;
                        }
                        case EAST: {
                            if (grid.checkIfNeedReverse(sp, (int) robot.getTwoTurnsDistance() / 2)) {
                                // double amount = grid.getAmountToReverse(sp, (int) robot.getTwoTurnsDistance()
                                // / 2);
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, turnRadius1);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }
                            // idea: 3 turns to reach dest
                            // else, reverse until other case
                            MyPoint tp0 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { sp, tp0, tp1, tp2, tp3, fp };

                            // first turn = left
                            tp1.rotateLeft90();
                            tp2.rotate180();
                            tp3.rotateRight90();
                            //
                            tp0.move(0, dp.y);
                            tp1.move(0, dp.y - turnRadius1);
                            tp2.move((int) dp.getX() - turnRadius1, dp.y - turnRadius1);
                            tp3.move((int) dp.getX() - turnRadius1, (int) dp.getY());

                            af.transform(tp0, tp0);
                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(tp3, tp3);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp0);
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                this.plannedPath.add(fp);
                                break;
                            }

                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray2 = { sp, tp1 };

                            tp1.move(0, (dp.y + turnRadius2));

                            af.transform(tp1, tp1);
                            if (!grid.checkIfPathCollides(tpArray2)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                i--;
                                break;
                            }
                            System.out.println("Collision detected at CENTER LEFT + EAST");
                            isValid = false;
                            break;
                        }
                        case WEST: {
                            // idea: if far enough left, reverse enough and turn left to pass to FORWARD
                            // if cannot, turn left where will be front left
                            // or just turn left
                            // or do 180 turn (try left)
                            // or do 3 L 1 R???
                            // if too close, turn right
                            // else, reverse and pass to front slight left
                            if (dp.x <= -turnRadius1) {
                                if (grid.checkIfNeedReverse(sp, (int) robot.getTwoTurnsDistance() / 2)) {
                                    MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                    rp.move(0, (int) robot.getTwoTurnsDistance() / 2);
                                    af.transform(rp, rp);
                                    this.plannedPath.add(rp);
                                    i--; // minus so that it does not go to the next workingPath point
                                    break;
                                }
                                MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint[] tpArray = { sp, tp1, tp2, tp3 };

                                tp1.move(0, (dp.y + turnRadius1));
                                tp2.move(0, dp.y);
                                tp3.move(-turnRadius1, dp.y);
                                tp2.rotateLeft90();
                                tp3.rotateLeft90();

                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                af.transform(tp3, tp3);
                                if (!grid.checkIfPathCollides(tpArray)) {
                                    // this path works, go next
                                    this.plannedPath.add(tp1);
                                    this.plannedPath.add(tp2);
                                    this.plannedPath.add(tp3);
                                    i--;
                                    break;
                                }
                                // left turn in front enough to be front left
                                MyPoint tp0 = new MyPoint(0, 0, sp.getDirection());
                                tp1 = new MyPoint(0, 0, sp.getDirection());
                                tp2 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint[] temp = { sp, tp0, tp1, tp2 };

                                tp0.move(0, dp.y - turnRadius2 - 1 + turnRadius1);
                                tp1.move(0, dp.y - turnRadius2 - 1);
                                tp2.move(-turnRadius1, dp.y - turnRadius2 - 1);
                                tp1.rotateLeft90();
                                tp2.rotateLeft90();

                                af.transform(tp0, tp0);
                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                if (!grid.checkIfPathCollides(temp)) {
                                    // this path works, go next
                                    this.plannedPath.add(tp0);
                                    this.plannedPath.add(tp1);
                                    this.plannedPath.add(tp2);
                                    i--;
                                    break;
                                }
                                // just turn left
                                tp1 = new MyPoint(0, 0, sp.getDirection());
                                tp2 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint[] temp2 = { sp, tp1, tp2 };

                                tp1.move(0, -turnRadius1);
                                tp2.move(-turnRadius1, -turnRadius1);
                                tp1.rotateLeft90();
                                tp2.rotateLeft90();

                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                if (!grid.checkIfPathCollides(temp2)) {
                                    // this path works, go next
                                    this.plannedPath.add(tp1);
                                    this.plannedPath.add(tp2);
                                    i--;
                                    break;
                                }

                                // 180 turn L
                                tp1 = new MyPoint(0, 0, sp.getDirection());
                                tp2 = new MyPoint(0, 0, sp.getDirection());
                                tp3 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp4 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint[] tpArray2 = { sp, tp1, tp2, tp3, tp4 };

                                tp1.move(0, dp.y - turnRadius2);
                                tp1.rotateLeft90();
                                tp2.move(-turnRadius1, dp.y - turnRadius2);
                                tp2.rotateLeft90();
                                tp3.move(turnRadius1, dp.y - turnRadius2);
                                tp3.rotateLeft90();
                                tp4.move(0, dp.y - turnRadius2);
                                tp4.rotate180();

                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                af.transform(tp3, tp3);
                                af.transform(tp4, tp4);
                                if (!grid.checkIfPathCollides(tpArray2)) {
                                    // this path works, go next
                                    this.plannedPath.add(tp1);
                                    this.plannedPath.add(tp2);
                                    this.plannedPath.add(tp3);
                                    this.plannedPath.add(tp4);
                                    i--;
                                    break;
                                }

                                // this is the LLLR 180 turn move X_X
                                tp1 = new MyPoint(0, 0, sp.getDirection());
                                tp2 = new MyPoint(0, 0, sp.getDirection());
                                tp3 = new MyPoint(0, 0, sp.getDirection());
                                tp4 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp1f = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp2f = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp3f = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp4f = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp1r = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp2r = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp3r = new MyPoint(0, 0, sp.getDirection());
                                MyPoint[] tpArray3 = { sp, tp1, tp1f, tp1r, tp2, tp2f, tp2r, tp3, tp3f, tp3r, tp4,
                                        tp4f };

                                tp1.move(0, dp.y - turnRadius2);
                                tp1.rotateLeft90();
                                tp1f.move(-turnRadius1, dp.y - turnRadius2);
                                tp1f.rotateLeft90();
                                tp1r.move(0, dp.y - turnRadius2);
                                tp1r.rotateLeft90();
                                tp2.move(-turnRadius1, dp.y - turnRadius2);
                                tp2.rotate180();
                                tp2f.move(-turnRadius1, dp.y - turnRadius2 + turnRadius1);
                                tp2f.rotate180();
                                tp2r.move(-turnRadius1, dp.y - turnRadius2);
                                tp2r.rotate180();
                                tp3.move(-turnRadius1, dp.y - turnRadius2 + turnRadius1);
                                tp3.rotateRight90();
                                tp3f.move(0, dp.y - turnRadius2 + turnRadius1);
                                tp3f.rotateRight90();
                                tp3r.move(-turnRadius1, dp.y - turnRadius2 + turnRadius1);
                                tp3r.rotateRight90();
                                tp4.move(0, dp.y - turnRadius2 + turnRadius1);
                                tp4.rotate180();
                                tp4f.move(0, dp.y);
                                tp4f.rotate180();

                                af.transform(tp1, tp1);
                                af.transform(tp1f, tp1f);
                                af.transform(tp1r, tp1r);
                                af.transform(tp2, tp2);
                                af.transform(tp2f, tp2f);
                                af.transform(tp2r, tp2r);
                                af.transform(tp3, tp3);
                                af.transform(tp3f, tp3f);
                                af.transform(tp3r, tp3r);
                                af.transform(tp4, tp4);
                                af.transform(tp4f, tp4f);
                                if (!grid.checkIfPathCollides(tpArray3)) {
                                    // this path works, go next
                                    this.plannedPath.add(tp1);
                                    this.plannedPath.add(tp1f);
                                    this.plannedPath.add(tp1r);
                                    this.plannedPath.add(tp2);
                                    this.plannedPath.add(tp2f);
                                    this.plannedPath.add(tp2r);
                                    this.plannedPath.add(tp3);
                                    this.plannedPath.add(tp3f);
                                    this.plannedPath.add(tp3r);
                                    this.plannedPath.add(tp4);
                                    this.plannedPath.add(tp4f);
                                    i--;
                                    break;
                                }
                                System.out.println("Collision detected at CENTER LEFT + WEST 1");
                                isValid = false;
                            } else if (dp.x > -turnRadius1) {
                                if (grid.checkIfNeedReverse(sp, (int) robot.getTwoTurnsDistance() / 2)) {
                                    MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                    rp.move(0, (int) robot.getTwoTurnsDistance() / 2);
                                    af.transform(rp, rp);
                                    this.plannedPath.add(rp);
                                    i--; // minus so that it does not go to the next workingPath point
                                    break;
                                }
                                MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint[] tpArray = { sp, tp1, tp2 };

                                tp1.move(0, -turnRadius1);
                                tp2.move(turnRadius1, -turnRadius1);
                                tp1.rotateRight90();
                                tp2.rotateRight90();

                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                if (!grid.checkIfPathCollides(tpArray)) {
                                    // this path works, go next
                                    this.plannedPath.add(tp1);
                                    this.plannedPath.add(tp2);
                                    i--;
                                    break;
                                }
                                tp1 = new MyPoint(0, 0, sp.getDirection());
                                tp2 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint[] tpArray2 = { sp, tp1, tp2 };

                                tp1.move(0, -turnRadius1);
                                tp2.move(-turnRadius1, -turnRadius1);
                                tp1.rotateLeft90();
                                tp2.rotateLeft90();

                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                if (!grid.checkIfPathCollides(tpArray2)) {
                                    // this path works, go next
                                    this.plannedPath.add(tp1);
                                    this.plannedPath.add(tp2);
                                    i--;
                                    break;
                                }
                                System.out.println("Collision detected at CENTER LEFT + WEST 2");
                                isValid = false;
                            } else {
                                MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());

                                tp1.move(0, (dp.y + turnRadius2));

                                af.transform(tp1, tp1);
                                if (!grid.checkIfPointCollides(tp1)) {
                                    // this path works, go next
                                    this.plannedPath.add(tp1);
                                    i--;
                                    break;
                                }
                                System.out.println("Collision detected at CENTER LEFT + WEST 3");
                                isValid = false;
                            }
                            break;
                        }
                        case NONE:
                            break;
                        default:
                            break;

                    }
                    break;
                case NONE:
                    break;
                default:
                    break;

            }

            i++;
            // System.out.println("i: " + i + " " + workingPath.size());
            if (i == workingPath.size() || point != workingPath.get(i)) {
                this.plannedPath.add(new MyPoint(-999, -999, Direction.NONE));
            }
        }
        System.out.println("-----GENERATED PATH-----");
        this.determinePolygonPath();
        return isValid;
    }

    public void addToPlannedPath(MyPoint p) {
        this.plannedPath.add(p);
    }

    public void printPlannedPath() {
        System.out.println("-------------");
        System.out.println("Planned path:");
        for (MyPoint p : plannedPath) {
            System.out.println(p.x + " " + p.y + " " + p.getDirection());
        }
        System.out.println("-------------");
    }

    public ArrayList<MyPoint> getPlannedPath() {
        return this.plannedPath;
    }

    private void determinePolygonPath() {
        this.plannedPPolygon.reset();
        for (MyPoint p : this.plannedPath) {
            if (p.x == -999 || p.y == -999) {
                continue;
            }
            this.plannedPPolygon.addPoint(p.x, p.y);
        }
    }

    private double getPathEstTotalDistance(Robot robot) {
        boolean justTurned = false;
        double totalDist = 0;
        double dist = 0;
        for (int i = 1; i < plannedPath.size(); i++) {
            MyPoint src = plannedPath.get(i - 1);
            MyPoint dest = plannedPath.get(i);
            if (dest.getDirection() == Direction.NONE) {
                continue;
            }
            if (src.getDirection() == Direction.NONE) {
                src = plannedPath.get(i - 2);
            }

            switch (robot.getRelativeOrientation(src, dest)) {
                case NORTH:
                    dist = this.getEuclideanDistance(src, dest);
                    if (justTurned) {
                        dist -= robot.MAX_TURNING_RADIUS;
                        justTurned = false;
                    }
                    switch (robot.getRelativeDirection(src, dest)) {
                        case BACK:
                            if (dist > 0) {
                                totalDist += dist;
                            } else {
                                totalDist += -dist;
                            }
                            justTurned = false;
                            break;
                        case FRONT:
                            if (dist > 0) {
                                totalDist += dist;
                            } else {
                                totalDist -= dist;
                            }
                            justTurned = false;
                            break;
                        case NONE:
                            break;
                        default:
                            break;
                    }
                    break;
                case SOUTH:
                    break;
                case EAST:
                    dist = this.getEuclideanDistance(src, dest);
                    dist -= robot.MAX_TURNING_RADIUS;
                    if (justTurned) {
                        dist -= robot.MAX_TURNING_RADIUS;
                        justTurned = false;
                    }
                    if (dist > 0) {
                        totalDist += dist;
                    } else {
                        totalDist += -dist;
                    }
                    totalDist += robot.MAX_TURNING_RADIUS * 2;
                    justTurned = true;
                    break;
                case WEST:
                    dist = this.getEuclideanDistance(src, dest);
                    dist -= robot.MAX_TURNING_RADIUS;
                    if (justTurned) {
                        dist -= robot.MAX_TURNING_RADIUS;
                        justTurned = false;
                    }
                    if (dist > 0) {
                        totalDist += dist;
                    } else {
                        totalDist += -dist;
                    }
                    totalDist += robot.MAX_TURNING_RADIUS * 2;
                    justTurned = true;
                    break;
                case NONE:
                    break;
                default:
                    break;

            }

        }
        return totalDist;
    }

    public void reset() {
        this.plannedPath.clear();
        this.plannedPPolygon.reset();
    }

}
