package simulator.algorithm;

import simulator.entity.MyPoint;
import simulator.entity.RelativeDirection;
import simulator.entity.Grid;
import simulator.entity.Robot;
import simulator.entity.Direction;

import javax.swing.*;
import java.awt.*;
import java.awt.geom.Path2D;
import java.awt.geom.Rectangle2D;
import java.lang.reflect.Array;
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
        return getShortestPath(src, inputs, false);
    }

    // method to be called by Simulator to generate the shortest path
    // this method doesn't actually get the shortest path but will
    // call other methods in this class to do so
    // this class does handle calling other methods
    public MyPoint[] getShortestPath(MyPoint src, MyPoint[] inputs, boolean flipObstacleDirections) {
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
        int[] shortestPathPointIndex = getShortestPathGreedy(adjacencyMatrix);

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

    public void generatePlannedPath(Grid grid, Robot robot) {
        this.reset();
        ArrayList<MyPoint> workingPath = new ArrayList<MyPoint>(Arrays.asList(this.shortestPath));

        this.plannedPath.add(robot.getCurrentLocation());
        MyPoint sp;

        System.out.println("-----GENERATING PATH-----");
        int i = 1;
        while (i < workingPath.size()) {
            // sp = source point (absolute point)
            sp = this.plannedPath.get(this.plannedPath.size() - 1);

            MyPoint point = workingPath.get(i);
            // take the current working point as destination

            RelativeDirection rDirection = robot.getRelativeDirection(sp, point);
            // get destination point's relative direction from source point
            Direction rOrientation = robot.getRelativeOrientation(sp, point);
            // get destination point's relative orientation from source point

            MyPoint dp = (MyPoint) point.clone();
            // dp = destination point (here is absolute point)

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
                            boolean flag = false; // to remember if collision will happen

                            double y = dp.getY();
                            dp.move(0, (int) y);
                            // set destination point to be point straight ahead of
                            // source point
                            // this is basically asking it to move ONLY forward, keeping x=0

                            MyPoint fp = new MyPoint(0, 0, dp.getDirection()); // give fp the same direction as
                                                                               // destination
                            af.transform(dp, fp);
                            // fp = final point (transformed back to absolute point)

                            // collision detection
                            while (grid.checkIfLineCollides(sp, fp)) {
                                flag = true;
                                // move the dp downwards to and check until no collision
                                y += 10;
                                dp.move(0, (int) y);

                                fp = new MyPoint(0, 0, dp.getDirection());
                                af.transform(dp, fp);
                            }
                            if (flag) {
                                // // move the destination point down by X amount that allows it to turn 90 deg
                                // dp.translate(0, (int) robot.getTwoTurnsDistance()/2); // NOTE: potential edge
                                // case here

                                fp = new MyPoint(0, 0, dp.getDirection());
                                af.transform(dp, fp); //
                                fp.rotateRight90(); // turn right to avoid collision

                                i--; // minus so that it does not go to the next workingPath point
                            }
                            this.plannedPath.add(fp); // add point to planned path
                            break;
                        }
                        case SOUTH: { // DONE (no collision avoidance)
                            // idea: 4 turns (right, left, left, left)
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
                            MyPoint[] tpArray = { tp1, tp2, tp3, tp4, fp };
                            int x, y;

                            // try for first turn to be right turn
                            tp1.rotateRight90(); // face right
                            // tp2 already facing straight
                            tp3.rotateLeft90(); // face left
                            tp4.rotate180(); // face backwards
                            //
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;

                            tp1.move(0, -y);
                            tp2.move(x, -y);
                            tp3.move(x, (int) dp.getY() - y);
                            tp4.move(0, (int) dp.getY() - y);

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

                            // try turn left first instead
                            tp1.rotateLeft90(); // face right
                            // tp2 already facing straight
                            tp3.rotateRight90(); // face left
                            tp4.rotate180(); // face backwards
                            //
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, -y);
                            tp2.move(-x, -y);
                            tp3.move(-x, (int) dp.getY() - y);
                            tp4.move(0, (int) dp.getY() - y);

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
                            ;
                            // this is where to do collision detection for other cases

                            break;
                        }
                        case EAST: { // DONE (no collision avoidance)
                            // idea: 3 turns
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
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { tp1, tp2, tp3, fp };
                            int x, y;

                            // first turn = left
                            tp1.rotateLeft90();
                            tp3.rotateRight90();
                            //
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, 0);
                            tp2.move(-x, 0);
                            tp3.move(-x, (int) dp.getY());

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
                            System.out.println("Collision detected at FRONT + EAST");
                            break;
                        }
                        case WEST: { // DONE (no collision avoidance)
                            // idea: 3 turns
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
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { tp1, tp2, tp3, fp };
                            int x, y;

                            // first turn = right
                            tp1.rotateRight90();
                            tp3.rotateLeft90();
                            //
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, -y);
                            tp2.move(x, -y);
                            tp3.move(x, (int) dp.getY());

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
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { tp1, tp2, fp };
                            int x, y;

                            // first turn = right
                            tp1.rotateRight90();
                            //
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, -y);
                            tp2.move(dp.x, -y);

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
                            // do collision detection
                            System.out.println("Collision detected at FRONT RIGHT + NORTH");
                            break;
                        }
                        case SOUTH: { // DONE (no c a)
                            // idea: 2 turns
                            if (grid.checkIfNeedReverse(sp, (int) robot.getTwoTurnsDistance() / 2)) {
                                // reverse, then turn right, then will be handled by another case
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                                tp1.rotateRight90();
                                tp2.rotateRight90();
                                rp.move(0, (int) robot.getTwoTurnsDistance() / 2);
                                tp2.move((int) robot.getTwoTurnsDistance() / 2, 0);
                                af.transform(rp, rp);
                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                this.plannedPath.add(rp);
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { tp1, tp2, fp };
                            int x, y;

                            // first turn = right
                            tp1.rotateRight90();
                            tp2.rotate180();
                            //
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, dp.y - y);
                            tp2.move(dp.x, dp.y - y);

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
                            // do collision detection
                            System.out.println("Collision detected at FRONT RIGHT + SOUTH");
                            break;
                        }
                        case EAST: { // DONE (no c a)
                            // idea: 1 turn
                            if (grid.checkIfNeedReverse(sp, (int) robot.getTwoTurnsDistance() / 2)) {
                                // reverse, then turn left, then will be handled by another case
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                                tp1.rotateLeft90();
                                tp2.rotateLeft90();
                                rp.move(0, (int) robot.getTwoTurnsDistance() / 2);
                                tp2.move(-(int) robot.getTwoTurnsDistance() / 2, 0);
                                af.transform(rp, rp);
                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                this.plannedPath.add(rp);
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { tp1, fp };
                            int x, y;

                            // first turn = right
                            tp1.rotateRight90();
                            //
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, dp.y);

                            af.transform(tp1, tp1);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(fp);
                                break;
                            }
                            // do collision detection
                            System.out.println("Collision detected at FRONT RIGHT + EAST");
                            break;
                        }
                        case WEST: { // DONE (no c a)
                            // idea: 3 turns
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
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { tp1, tp2, tp3, fp };
                            int x, y;

                            // first turn = right
                            tp1.rotateRight90();
                            tp3.rotateLeft90();
                            //
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, -y);
                            tp2.move(dp.x + y, -y);
                            tp3.move(dp.x + y, (int) dp.getY());

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
                        case NORTH: { // DONE (no collision avoidance)
                            // idea: 2 turns
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
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { tp1, tp2, fp };
                            int x, y;

                            // first turn = left
                            tp1.rotateLeft90();
                            //
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, -y);
                            tp2.move(dp.x, -y);

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
                            // do collision detection
                            System.out.println("Collision detected at FRONT LEFT + NORTH");
                            break;
                        }
                        case SOUTH: { // DONE (no collision avoidance)
                            // idea: 2 turns
                            if (grid.checkIfNeedReverse(sp, (int) robot.getTwoTurnsDistance() / 2)) {
                                // reverse, then turn left, then will be handled by another case
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                                tp1.rotateLeft90();
                                tp2.rotateLeft90();
                                rp.move(0, (int) robot.getTwoTurnsDistance() / 2);
                                tp2.move(-(int) robot.getTwoTurnsDistance() / 2, 0);
                                af.transform(rp, rp);
                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                this.plannedPath.add(rp);
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { tp1, tp2, fp };
                            int x, y;

                            // first turn = left
                            tp1.rotateLeft90();
                            tp2.rotate180();
                            //
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, dp.y - y);
                            tp2.move(dp.x, dp.y - y);

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
                            // do collision detection
                            System.out.println("Collision detected at FRONT LEFT + SOUTH");
                            break;
                        }
                        case EAST: { // DONE (no collision avoidance)
                            // idea: 3 turns
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
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { tp1, tp2, tp3, fp };
                            int x, y;

                            // first turn = left
                            tp1.rotateLeft90();
                            tp3.rotateRight90();
                            //
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, -y);
                            tp2.move((int) dp.getX() - y, -y);
                            tp3.move((int) dp.getX() - y, (int) dp.getY());

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
                            break;
                        }
                        case WEST: { // DONE (no c a)
                            // idea: 1 turn
                            if (grid.checkIfNeedReverse(sp, (int) robot.getTwoTurnsDistance() / 2)) {
                                // reverse, then turn left, then will be handled by another case
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                                tp1.rotateLeft90();
                                tp2.rotateLeft90();
                                rp.move(0, (int) robot.getTwoTurnsDistance() / 2);
                                tp2.move(-(int) robot.getTwoTurnsDistance() / 2, 0);
                                af.transform(rp, rp);
                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                this.plannedPath.add(rp);
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { tp1, fp };
                            int x, y;

                            // first turn = left
                            tp1.rotateLeft90();
                            //
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, dp.y);

                            af.transform(tp1, tp1);
                            af.transform(dp, fp);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(fp);
                                break;
                            }
                            // do collision detection
                            System.out.println("Collision detected at FRONT LEFT + WEST");
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
                                MyPoint[] tpArray = { tp1, fp };

                                // first turn = left
                                tp1.rotateLeft90();
                                //
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
                        case SOUTH:
                        case EAST:
                            // turn left to go to other cases
                            // if not, turn right
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
                            MyPoint[] tpArray = { tp1, tp2 };

                            // first turn = left
                            tp1.rotateLeft90();
                            tp2.rotateLeft90();
                            //
                            int y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, -y);
                            tp2.move(-y, -y);

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

                            // first turn = right
                            tp1.rotateRight90();
                            tp2.rotateRight90();
                            //
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, -y);
                            tp2.move(y, -y);

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
                            break;
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
                                MyPoint[] tpArray = { tp1, fp };

                                // first turn = right
                                tp1.rotateRight90();
                                //
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
                        case SOUTH:
                        case WEST:
                            // turn right to go other case
                            // else turn left
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
                            MyPoint[] tpArray = { tp1, tp2 };

                            // first turn = right
                            tp1.rotateRight90();
                            tp2.rotateRight90();
                            //
                            int y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, -y);
                            tp2.move(y, -y);

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

                            // first turn = left
                            tp1.rotateLeft90();
                            tp2.rotateLeft90();
                            //
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, -y);
                            tp2.move(-y, -y);

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
                            System.out.println("Collision detected at FRONT SLIGHT RIGHT");
                            break;
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
                            MyPoint[] tpArray = { tp1, tp2, tp3, tp4, fp };
                            int x, y;

                            // first turn = right
                            tp1.rotateRight90();
                            tp2.rotate180();
                            tp3.rotateLeft90();
                            //
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, -y);
                            tp2.move(x, -y);
                            tp3.move(x, dp.y + y);
                            tp4.move(0, dp.y + y);

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

                            // first turn = left
                            tp1.rotateLeft90();
                            tp2.rotate180();
                            tp3.rotateRight90();
                            //
                            tp1.move(0, -y);
                            tp2.move(-x, -y);
                            tp3.move(-x, dp.y + y);
                            tp4.move(0, dp.y + y);

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

                            // make 180 turn (Reverse, Right, Reverse, Right)
                            // Reverse already done above
                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                            tp3 = new MyPoint(0, 0, sp.getDirection());
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, -y);
                            tp1.rotateRight90();
                            tp2.move(y, -y);
                            tp2.rotateRight90();
                            rp.move(-y, -y);
                            rp.rotateRight90();
                            tp3.move(0, -y);
                            tp3.rotate180();
                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(rp, rp);
                            af.transform(tp3, tp3);
                            this.plannedPath.add(tp1);
                            this.plannedPath.add(tp2);
                            this.plannedPath.add(rp);
                            this.plannedPath.add(tp3);
                            i--;
                            break;
                        }
                        case SOUTH: { // DONE (no collision avoidance)
                            // we try 3 ways to see if it works
                            // 1. 4 turns, R R R L
                            // 2. 4 turns, L L L R
                            // 3. make 180 turn, then let other case handle
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
                            MyPoint[] tpArray = { tp1, tp2, tp3, tp4, fp };
                            int x, y;

                            // first turn = right
                            tp1.rotateRight90();
                            tp2.rotate180();
                            tp3.rotateLeft90();
                            tp4.rotate180();
                            //
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, -y);
                            tp2.move(x, -y);
                            tp3.move(x, dp.y - y);
                            tp4.move(0, dp.y - y);

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

                            // first turn = left
                            tp1.rotateLeft90();
                            tp2.rotate180();
                            tp3.rotate180();
                            //
                            tp1.move(0, -y);
                            tp2.move(-x, -y);
                            tp3.move(-x, dp.y - y);
                            tp4.move(0, dp.y - y);

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

                            // make 180 turn (Reverse, Right, Reverse, Right)
                            // Reverse already done above
                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                            tp3 = new MyPoint(0, 0, sp.getDirection());
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, -y);
                            tp1.rotateRight90();
                            tp2.move(y, -y);
                            tp2.rotateRight90();
                            rp.move(-y, -y);
                            rp.rotateRight90();
                            tp3.move(0, -y);
                            tp3.rotate180();
                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(rp, rp);
                            af.transform(tp3, tp3);
                            this.plannedPath.add(tp1);
                            this.plannedPath.add(tp2);
                            this.plannedPath.add(rp);
                            this.plannedPath.add(tp3);
                            i--;
                            break;
                        }
                        case EAST: { // DONE (no collision avoidance)
                            // we try 2 ways to see if it works
                            // 1. 3 turns, L L L
                            // 2. make 180 turn, then let other case handle
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
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { tp1, tp2, tp3, fp };
                            int x, y;

                            // first turn = right
                            tp1.rotateLeft90();
                            tp2.rotate180();
                            tp3.rotateRight90();
                            //
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, -y);
                            tp2.move(-x, -y);
                            tp3.move(-x, dp.y);

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
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, -y);
                            tp1.rotateRight90();
                            tp2.move(y, -y);
                            tp2.rotateRight90();
                            rp.move(-y, -y);
                            rp.rotateRight90();
                            tp3.move(0, -y);
                            tp3.rotate180();
                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(rp, rp);
                            af.transform(tp3, tp3);
                            this.plannedPath.add(tp1);
                            this.plannedPath.add(tp2);
                            this.plannedPath.add(rp);
                            this.plannedPath.add(tp3);
                            i--;
                            break;
                        }
                        case WEST: { // DONE (no collision avoidance)
                            // we try 2 ways to see if it works
                            // 1. 3 turns, R R R
                            // 2. make 180 turn, then let other case handle
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
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { tp1, tp2, tp3, fp };
                            int x, y;

                            // first turn = right
                            tp1.rotateRight90();
                            tp2.rotate180();
                            tp3.rotateLeft90();
                            //
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, -y);
                            tp2.move(x, -y);
                            tp3.move(x, dp.y);

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
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, -y);
                            tp1.rotateRight90();
                            tp2.move(y, -y);
                            tp2.rotateRight90();
                            rp.move(-y, -y);
                            rp.rotateRight90();
                            tp3.move(0, -y);
                            tp3.rotate180();
                            af.transform(tp1, tp1);
                            af.transform(tp2, tp2);
                            af.transform(rp, rp);
                            af.transform(tp3, tp3);
                            this.plannedPath.add(tp1);
                            this.plannedPath.add(tp2);
                            this.plannedPath.add(rp);
                            this.plannedPath.add(tp3);
                            i--;
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
                            MyPoint[] tpArray = { tp1, tp2, tp3, tp4, fp };
                            int x, y;

                            // first turn = right
                            tp1.rotateRight90();
                            tp2.rotate180();
                            tp3.rotateLeft90();
                            //
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, -y);
                            tp2.move(dp.x + x, -y);
                            tp3.move(dp.x + x, dp.y + y);
                            tp4.move(dp.x, dp.y + y);

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

                            // first turn = left
                            tp1.rotateLeft90();
                            tp2.rotate180();
                            tp3.rotateRight90();
                            //
                            tp1.move(0, -y);
                            tp2.move(-x, -y);
                            tp3.move(-x, dp.y + y);
                            tp4.move(dp.x, dp.y + y);

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

                            // make 180 turn (Reverse, Right, Reverse, Right)
                            // Reverse already done above
                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                            tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray2 = { tp1, tp2, rp, tp3 };

                            tp1.move(0, -y);
                            tp1.rotateRight90();
                            tp2.move(y, -y);
                            tp2.rotateRight90();
                            rp.move(-y, -y);
                            rp.rotateRight90();
                            tp3.move(0, -y);
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
                            break;
                        }
                        case SOUTH: {
                            // idea: 2 turns R R
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
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { tp1, tp2, fp };
                            int x, y;

                            // first turn = right
                            tp1.rotateRight90();
                            tp2.rotate180();
                            //
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, -y);
                            tp2.move(dp.x, -y);

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
                            break;
                        }
                        case EAST: {
                            // idea: 3 turns L L L
                            // or R then pass
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
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { tp1, tp2, tp3, fp };
                            int x, y;

                            // first turn = left
                            tp1.rotateLeft90();
                            tp2.rotate180();
                            tp3.rotateRight90();
                            //
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, -y);
                            tp2.move(-x, -y);
                            tp3.move(-x, dp.y);

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
                            MyPoint[] tpArray2 = { tp1, tp2, tp3 };
                            tp1.rotateRight90();
                            tp2.rotateRight90();
                            tp3.rotateRight90();
                            tp1.move(0, -y);
                            tp2.move(y, -y);
                            tp3.move(0, -y);

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
                            break;
                        }
                        case WEST: {
                            // idea: 3 turns R R R
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
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { tp1, tp2, tp3, fp };
                            int x, y;

                            // first turn = right
                            tp1.rotateRight90();
                            tp2.rotate180();
                            tp3.rotateLeft90();
                            //
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, -y);
                            tp2.move(dp.x + y, -y);
                            tp3.move(dp.x + y, dp.y);

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
                            MyPoint[] tpArray = { tp1, tp2, tp3, tp4, fp };
                            int x, y;

                            // first turn = left
                            tp1.rotateLeft90();
                            tp2.rotate180();
                            tp3.rotateRight90();
                            //
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, -y);
                            tp2.move(dp.x - x, -y);
                            tp3.move(dp.x - x, dp.y + y);
                            tp4.move(dp.x, dp.y + y);

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

                            // first turn = right
                            tp1.rotateRight90();
                            tp2.rotate180();
                            tp3.rotateLeft90();
                            //
                            tp1.move(0, -y);
                            tp2.move(x, -y);
                            tp3.move(x, dp.y + y);
                            tp4.move(dp.x, dp.y + y);

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

                            // make 180 turn (Reverse, Right, Reverse, Right)
                            // Reverse already done above
                            tp1 = new MyPoint(0, 0, sp.getDirection());
                            tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                            tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray2 = { tp1, tp2, rp, tp3 };

                            tp1.move(0, -y);
                            tp1.rotateRight90();
                            tp2.move(y, -y);
                            tp2.rotateRight90();
                            rp.move(-y, -y);
                            rp.rotateRight90();
                            tp3.move(0, -y);
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
                            break;
                        }
                        case SOUTH: {
                            // idea: 2 turns L L
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
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { tp1, tp2, fp };
                            int x, y;

                            // first turn = right
                            tp1.rotateLeft90();
                            tp2.rotate180();
                            //
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, -y);
                            tp2.move(dp.x, -y);

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
                            break;
                        }
                        case EAST: {
                            // idea: 3 turns L L L
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
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { tp1, tp2, tp3, fp };
                            int x, y;

                            // first turn = left
                            tp1.rotateLeft90();
                            tp2.rotate180();
                            tp3.rotateRight90();
                            //
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, -y);
                            tp2.move(dp.x - y, -y);
                            tp3.move(dp.x - y, dp.y);

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
                            break;
                        }
                        case WEST: {
                            // idea: 3 turns R R R
                            // or L then pass
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
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { tp1, tp2, tp3, fp };
                            int x, y;

                            // first turn = right
                            tp1.rotateRight90();
                            tp2.rotate180();
                            tp3.rotateLeft90();
                            //
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, -y);
                            tp2.move(x, -y);
                            tp3.move(x, dp.y);

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
                            MyPoint[] tpArray2 = { tp1, tp2, tp3 };
                            tp1.rotateLeft90();
                            tp2.rotateLeft90();
                            tp3.rotateLeft90();
                            tp1.move(0, -y);
                            tp2.move(-y, -y);
                            tp3.move(0, -y);

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
                        // turn right to go to other cases
                        case NORTH:
                        case SOUTH:
                        case EAST:
                        case WEST:
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
                            MyPoint[] tpArray = { tp1, tp2 };

                            // first turn = left
                            tp1.rotateLeft90();
                            tp2.rotateLeft90();
                            //
                            int y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, -y);
                            tp2.move(-y, -y);

                            af.transform(tp1, tp1);
                            if (!grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                i--;
                                break;
                            }
                            // do collision detection
                            System.out.println("Collision detected at FRONT SLIGHT LEFT");
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
                            MyPoint[] tpArray = { tp1, tp2 };

                            // first turn = right
                            tp1.rotateRight90();
                            tp2.rotateRight90();
                            //
                            int y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, -y);
                            tp2.move(y, -y);

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
                            // try to reverse till it is in top left
                            int x;
                            MyPoint rp = new MyPoint(0, 0, sp.getDirection());

                            x = (int) robot.getTwoTurnsDistance();
                            rp.move(0, x - dp.y);

                            af.transform(rp, rp);
                            if (!grid.checkIfPointCollides(rp)) {
                                // this path works, go next
                                this.plannedPath.add(rp);
                                i--;
                                break;
                            }
                            // if cannot, move forward till in bottom left
                            MyPoint tp = new MyPoint(0, 0, sp.getDirection());

                            x = (int) robot.getTwoTurnsDistance();
                            tp.move(0, -(x - dp.y));

                            af.transform(tp, tp);
                            if (!grid.checkIfPointCollides(tp)) {
                                // this path works, go next
                                this.plannedPath.add(tp);
                                i--;
                                break;
                            }
                            System.out.println("Collision detected at CENTER RIGHT + NORTH");
                            break;
                        }
                        case SOUTH: {
                            int x, y;
                            // move forward and turn right till in top right
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray = { tp1, tp2, tp3 };

                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, dp.y);
                            tp2.move(0, dp.y - y);
                            tp3.move(y, dp.y - y);
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
                            System.out.println("Collision detected at CENTER RIGHT + SOUTH");
                            break;
                        }
                        case EAST: {
                            // idea: if far enough left, reverse enough and turn right to pass to FORWARD
                            // if cannot, just turn right
                            // or do 180 turn (try right)
                            // or do 3 R 1 L???
                            // else if too close, turn left
                            // else, reverse and pass to front slight right
                            int x, y;
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            if (dp.x >= y) {
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
                                MyPoint[] tpArray = { tp1, tp2, tp3 };

                                tp1.move(0, (dp.y + y));
                                tp2.move(0, dp.y);
                                tp3.move(y, dp.y);
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
                                // just right turn
                                tp1= new MyPoint(0, 0, sp.getDirection());
                                tp2 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint[] temp = {tp1,tp2};

                                tp1.move(0, -y);
                                tp2.move(y, -y);
                                tp1.rotateRight90();
                                tp2.rotateRight90();

                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                if (!grid.checkIfPathCollides(temp)) {
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
                                MyPoint[] tpArray2 = { tp1, tp2, tp3, tp4 };

                                tp1.move(0, dp.y - x);
                                tp1.rotateRight90();
                                tp2.move(y, dp.y - x);
                                tp2.rotateRight90();
                                tp3.move(-y, dp.y - x);
                                tp3.rotateRight90();
                                tp4.move(0, dp.y - x);
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
                                MyPoint[] tpArray3 = { tp1, tp1f, tp1r, tp2, tp2f, tp2r, tp3, tp3f, tp3r, tp4, tp4f };

                                tp1.move(0, dp.y - x);
                                tp1.rotateRight90();
                                tp1f.move(y, dp.y - x);
                                tp1f.rotateRight90();
                                tp1r.move(0, dp.y - x);
                                tp1r.rotateRight90();
                                tp2.move(y, dp.y - x);
                                tp2.rotate180();
                                tp2f.move(y, dp.y - x + y);
                                tp2f.rotate180();
                                tp2r.move(y, dp.y - x);
                                tp2r.rotate180();
                                tp3.move(y, dp.y - x + y);
                                tp3.rotateLeft90();
                                tp3f.move(0, dp.y - x + y);
                                tp3f.rotateLeft90();
                                tp3r.move(y, dp.y - x + y);
                                tp3r.rotateLeft90();
                                tp4.move(0, dp.y - x + y);
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
                            } else if (dp.x < y) {
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
                                MyPoint[] tpArray = { tp1, tp2 };

                                tp1.move(0, -y);
                                tp2.move(-y, -y);
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
                                System.out.println("Collision detected at CENTER RIGHT + EAST 2");
                            } else {
                                MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());

                                tp1.move(0, (dp.y + x));

                                af.transform(tp1, tp1);
                                if (!grid.checkIfPointCollides(tp1)) {
                                    // this path works, go next
                                    this.plannedPath.add(tp1);
                                    i--;
                                    break;
                                }
                                System.out.println("Collision detected at CENTER RIGHT + EAST 3");
                            }
                            break;
                        }
                        case WEST: {
                            // idea: reverse untill other case
                            int x, y;
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());

                            tp1.move(0, (dp.y + x));

                            af.transform(tp1, tp1);
                            if (!grid.checkIfPointCollides(tp1)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                i--;
                                break;
                            }
                            System.out.println("Collision detected at CENTER RIGHT + WEST");
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
                            // try to reverse till it is in top left
                            int x;
                            MyPoint tp = new MyPoint(0, 0, sp.getDirection());

                            x = (int) robot.getTwoTurnsDistance();
                            tp.move(0, x - dp.y);

                            af.transform(tp, tp);
                            if (!grid.checkIfPointCollides(tp)) {
                                // this path works, go next
                                this.plannedPath.add(tp);
                                i--;
                                break;
                            }
                            // if cannot, move forward till in bottom left
                            tp = new MyPoint(0, 0, sp.getDirection());

                            x = (int) robot.getTwoTurnsDistance();
                            tp.move(0, -(x - dp.y));

                            af.transform(tp, tp);
                            if (!grid.checkIfPointCollides(tp)) {
                                // this path works, go next
                                this.plannedPath.add(tp);
                                i--;
                                break;
                            }
                            System.out.println("Collision detected at CENTER LEFT + NORTH");
                            break;
                        }
                        case SOUTH: {
                            int x, y;
                            // move forward and turn left till in top left
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint[] tpArray = { tp1, tp2, tp3 };

                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, dp.y - y);
                            tp2.move(0, dp.y - x);
                            tp3.move(-y, dp.y - x);
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
                            System.out.println("Collision detected at CENTER LEFT + SOUTH");
                            break;
                        }
                        case EAST: {
                            // idea: reverse untill other case
                            int x, y;
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());

                            tp1.move(0, (dp.y + x));

                            af.transform(tp1, tp1);
                            if (!grid.checkIfPointCollides(tp1)) {
                                // this path works, go next
                                this.plannedPath.add(tp1);
                                i--;
                                break;
                            }
                            System.out.println("Collision detected at CENTER LEFT + EAST");
                            break;
                        }
                        case WEST: {
                            // idea: if far enough left, reverse enough and turn left to pass to FORWARD
                            // if cannot, just turn left
                            // or do 180 turn (try left)
                            // or do 3 L 1 R???
                            // if too close, turn right
                            // else, reverse and pass to front slight left
                            int x, y;
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            if (dp.x <= -y) {
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
                                MyPoint[] tpArray = { tp1, tp2, tp3 };

                                tp1.move(0, (dp.y + y));
                                tp2.move(0, dp.y);
                                tp3.move(-y, dp.y);
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
                                // just left turn
                                tp1= new MyPoint(0, 0, sp.getDirection());
                                tp2 = new MyPoint(0, 0, sp.getDirection());
                                MyPoint[] temp = {tp1,tp2};

                                tp1.move(0, -y);
                                tp2.move(-y, -y);
                                tp1.rotateLeft90();
                                tp2.rotateLeft90();

                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                if (!grid.checkIfPathCollides(temp)) {
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
                                MyPoint[] tpArray2 = { tp1, tp2, tp3, tp4 };

                                tp1.move(0, dp.y - x);
                                tp1.rotateLeft90();
                                tp2.move(-y, dp.y - x);
                                tp2.rotateLeft90();
                                tp3.move(y, dp.y - x);
                                tp3.rotateLeft90();
                                tp4.move(0, dp.y - x);
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
                                MyPoint[] tpArray3 = { tp1, tp1f, tp1r, tp2, tp2f, tp2r, tp3, tp3f, tp3r, tp4, tp4f };

                                tp1.move(0, dp.y - x);
                                tp1.rotateLeft90();
                                tp1f.move(-y, dp.y - x);
                                tp1f.rotateLeft90();
                                tp1r.move(0, dp.y - x);
                                tp1r.rotateLeft90();
                                tp2.move(-y, dp.y - x);
                                tp2.rotate180();
                                tp2f.move(-y, dp.y - x + y);
                                tp2f.rotate180();
                                tp2r.move(-y, dp.y - x);
                                tp2r.rotate180();
                                tp3.move(-y, dp.y - x + y);
                                tp3.rotateRight90();
                                tp3f.move(0, dp.y - x + y);
                                tp3f.rotateRight90();
                                tp3r.move(-y, dp.y - x + y);
                                tp3r.rotateRight90();
                                tp4.move(0, dp.y - x + y);
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
                            } else if (dp.x > -y) {
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
                                MyPoint[] tpArray = { tp1, tp2 };

                                tp1.move(0, -y);
                                tp2.move(y, -y);
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
                                System.out.println("Collision detected at CENTER LEFT + WEST 2");
                            } else {
                                MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());

                                tp1.move(0, (dp.y + x));

                                af.transform(tp1, tp1);
                                if (!grid.checkIfPointCollides(tp1)) {
                                    // this path works, go next
                                    this.plannedPath.add(tp1);
                                    i--;
                                    break;
                                }
                                System.out.println("Collision detected at CENTER LEFT + WEST 3");
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
        }
        System.out.println("-----GENERATED PATH-----");
        this.determinePolygonPath();
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
            this.plannedPPolygon.addPoint(p.x, p.y);
        }
    }

    public void reset() {
        this.plannedPath.clear();
        this.plannedPPolygon.reset();
    }

}

// input of Points
// need to generate the shortest path based on Points
// need to generated weighted graph
//