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

        for (Point p : shortestPath) {
            this.polygon.addPoint((int) p.getX(), (int) p.getY());
        }

        // Not necessary
        // this.polygon.addPoint((int) src.getX(), (int) src.getY());

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
        g2.setColor(Color.green);
        g2.drawPolygon(this.polygon);

        g2.setColor(Color.blue);
        g2.drawPolygon(this.plannedPPolygon);
    }

    public void generatePlannedPath(Grid grid, Robot robot) {
        this.reset();

        // this.shortestPath
        // this.plannedPath
        this.plannedPath.add(robot.getCurrentLocation());
        this.plannedPPolygon.addPoint((int) robot.getCurrentLocation().getX(), (int) robot.getCurrentLocation().getY());

        ArrayList<MyPoint> workingPath = new ArrayList<MyPoint>(Arrays.asList(this.shortestPath));

        // System.out.println("WORKING PATH");
        // for (MyPoint ppp : workingPath) {
        // System.out.println(ppp.x + " " + ppp.y + " " + ppp.getDirection());
        // }
        // System.out.println("----------------");
        // Direction robotDirection = robot.getGeneralDirection();
        MyPoint sp;

        int i = 1;
        while (i < workingPath.size()) {
            // sp = source point (absolute point)
            sp = this.plannedPath.get(this.plannedPath.size() - 1);
            // if start, take first point of workingPath AKA robot starting point
            // else, that the last planned step as source
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
                            this.plannedPPolygon.addPoint((int) fp.getX(), (int) fp.getY());
                            break;
                        }
                        case SOUTH: { // DONE (no collision avoidance)
                            // idea: 4 turns (right, left, left, left)
                            if (grid.checkIfNeedReverse(sp, (int) robot.getTwoTurnsDistance() / 2)) {
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                rp.move(0, (int) robot.getTwoTurnsDistance() / 2);
                                af.transform(rp, rp);
                                this.plannedPath.add(rp);
                                this.plannedPPolygon.addPoint(rp.x, rp.y);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp4 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { tp1, tp2, tp3, tp4, dp };
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

                            if (grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                af.transform(tp3, tp3);
                                af.transform(tp4, tp4);
                                af.transform(dp, fp);
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                this.plannedPath.add(tp4);
                                this.plannedPath.add(fp);
                                this.plannedPPolygon.addPoint(tp1.x, tp1.y);
                                this.plannedPPolygon.addPoint(tp2.x, tp2.y);
                                this.plannedPPolygon.addPoint(tp3.x, tp3.y);
                                this.plannedPPolygon.addPoint(tp4.x, tp4.y);
                                this.plannedPPolygon.addPoint(fp.x, fp.y);
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

                            if (grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                af.transform(tp3, tp3);
                                af.transform(tp4, tp4);
                                af.transform(dp, fp);
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                this.plannedPath.add(tp4);
                                this.plannedPath.add(fp);
                                this.plannedPPolygon.addPoint(tp1.x, tp1.y);
                                this.plannedPPolygon.addPoint(tp2.x, tp2.y);
                                this.plannedPPolygon.addPoint(tp3.x, tp3.y);
                                this.plannedPPolygon.addPoint(tp4.x, tp4.y);
                                this.plannedPPolygon.addPoint(fp.x, fp.y);
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
                                this.plannedPPolygon.addPoint(rp.x, rp.y);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { tp1, tp2, tp3, dp };
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

                            if (grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                af.transform(tp3, tp3);
                                af.transform(dp, fp);
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                this.plannedPath.add(fp);
                                this.plannedPPolygon.addPoint(tp1.x, tp1.y);
                                this.plannedPPolygon.addPoint(tp2.x, tp2.y);
                                this.plannedPPolygon.addPoint(tp3.x, tp3.y);
                                this.plannedPPolygon.addPoint(fp.x, fp.y);
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
                                this.plannedPPolygon.addPoint(rp.x, rp.y);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { tp1, tp2, tp3, dp };
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

                            if (grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                af.transform(tp3, tp3);
                                af.transform(dp, fp);
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                this.plannedPath.add(fp);
                                this.plannedPPolygon.addPoint(tp1.x, tp1.y);
                                this.plannedPPolygon.addPoint(tp2.x, tp2.y);
                                this.plannedPPolygon.addPoint(tp3.x, tp3.y);
                                this.plannedPPolygon.addPoint(fp.x, fp.y);
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
                                this.plannedPPolygon.addPoint(rp.x, rp.y);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { tp1, tp2, dp };
                            int x, y;

                            // first turn = right
                            tp1.rotateRight90();
                            //
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, -y);
                            tp2.move(dp.x, -y);

                            if (grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                af.transform(dp, fp);
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(fp);
                                this.plannedPPolygon.addPoint(tp1.x, tp1.y);
                                this.plannedPPolygon.addPoint(tp2.x, tp2.y);
                                this.plannedPPolygon.addPoint(fp.x, fp.y);
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
                                tp1.rotateRight90();
                                rp.move(0, (int) robot.getTwoTurnsDistance() / 2);
                                af.transform(rp, rp);
                                af.transform(tp1, tp1);
                                this.plannedPath.add(rp);
                                this.plannedPath.add(tp1);
                                this.plannedPPolygon.addPoint(rp.x, rp.y);
                                this.plannedPPolygon.addPoint(tp1.x, tp1.y);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { tp1, tp2, dp };
                            int x, y;

                            // first turn = left
                            tp1.rotateLeft90();
                            tp2.rotate180();
                            //
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, dp.y - y);
                            tp2.move(dp.x, dp.y - y);

                            if (grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                af.transform(dp, fp);
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(fp);
                                this.plannedPPolygon.addPoint(tp1.x, tp1.y);
                                this.plannedPPolygon.addPoint(tp2.x, tp2.y);
                                this.plannedPPolygon.addPoint(fp.x, fp.y);
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
                                tp1.rotateLeft90();
                                rp.move(0, (int) robot.getTwoTurnsDistance() / 2);
                                af.transform(rp, rp);
                                af.transform(tp1, tp1);
                                this.plannedPath.add(rp);
                                this.plannedPath.add(tp1);
                                this.plannedPPolygon.addPoint(rp.x, rp.y);
                                this.plannedPPolygon.addPoint(tp1.x, tp1.y);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { tp1, dp };
                            int x, y;

                            // first turn = right
                            tp1.rotateRight90();
                            //
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, dp.y);

                            if (grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                af.transform(tp1, tp1);
                                af.transform(dp, fp);
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(fp);
                                this.plannedPPolygon.addPoint(tp1.x, tp1.y);
                                this.plannedPPolygon.addPoint(fp.x, fp.y);
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
                                this.plannedPPolygon.addPoint(rp.x, rp.y);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { tp1, tp2, tp3, dp };
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

                            if (grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                af.transform(tp3, tp3);
                                af.transform(dp, fp);
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                this.plannedPath.add(fp);
                                this.plannedPPolygon.addPoint(tp1.x, tp1.y);
                                this.plannedPPolygon.addPoint(tp2.x, tp2.y);
                                this.plannedPPolygon.addPoint(tp3.x, tp3.y);
                                this.plannedPPolygon.addPoint(fp.x, fp.y);
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
                                this.plannedPPolygon.addPoint(rp.x, rp.y);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { tp1, tp2, dp };
                            int x, y;

                            // first turn = left
                            tp1.rotateLeft90();
                            //
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, -y);
                            tp2.move(dp.x, -y);

                            if (grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                af.transform(dp, fp);
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(fp);
                                this.plannedPPolygon.addPoint(tp1.x, tp1.y);
                                this.plannedPPolygon.addPoint(tp2.x, tp2.y);
                                this.plannedPPolygon.addPoint(fp.x, fp.y);
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
                                tp1.rotateLeft90();
                                rp.move(0, (int) robot.getTwoTurnsDistance() / 2);
                                af.transform(rp, rp);
                                af.transform(tp1, tp1);
                                this.plannedPath.add(rp);
                                this.plannedPath.add(tp1);
                                this.plannedPPolygon.addPoint(rp.x, rp.y);
                                this.plannedPPolygon.addPoint(tp1.x, tp1.y);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { tp1, tp2, dp };
                            int x, y;

                            // first turn = right
                            tp1.rotateRight90();
                            tp2.rotate180();
                            //
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, dp.y - y);
                            tp2.move(dp.x, dp.y - y);

                            if (grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                af.transform(dp, fp);
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(fp);
                                this.plannedPPolygon.addPoint(tp1.x, tp1.y);
                                this.plannedPPolygon.addPoint(tp2.x, tp2.y);
                                this.plannedPPolygon.addPoint(fp.x, fp.y);
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
                                this.plannedPPolygon.addPoint(rp.x, rp.y);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp2 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint tp3 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { tp1, tp2, tp3, dp };
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

                            if (grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                af.transform(tp1, tp1);
                                af.transform(tp2, tp2);
                                af.transform(tp3, tp3);
                                af.transform(dp, fp);
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(tp2);
                                this.plannedPath.add(tp3);
                                this.plannedPath.add(fp);
                                this.plannedPPolygon.addPoint(tp1.x, tp1.y);
                                this.plannedPPolygon.addPoint(tp2.x, tp2.y);
                                this.plannedPPolygon.addPoint(tp3.x, tp3.y);
                                this.plannedPPolygon.addPoint(fp.x, fp.y);
                                break;
                            }
                            // do collision detection
                            System.out.println("Collision detected at FRONT LEFT + EAST");
                            break;
                        }
                        case WEST: { // DONE (no c a)
                            // idea: 1 turn
                            if (grid.checkIfNeedReverse(sp, (int) robot.getTwoTurnsDistance() / 2)) {
                                // reverse, then turn right, then will be handled by another case
                                MyPoint rp = new MyPoint(0, 0, sp.getDirection());
                                MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                                tp1.rotateRight90();
                                rp.move(0, (int) robot.getTwoTurnsDistance() / 2);
                                af.transform(rp, rp);
                                af.transform(tp1, tp1);
                                this.plannedPath.add(rp);
                                this.plannedPath.add(tp1);
                                this.plannedPPolygon.addPoint(rp.x, rp.y);
                                this.plannedPPolygon.addPoint(tp1.x, tp1.y);
                                i--; // minus so that it does not go to the next workingPath point
                                break;
                            }
                            MyPoint tp1 = new MyPoint(0, 0, sp.getDirection());
                            MyPoint fp = new MyPoint(0, 0, dp.getDirection());
                            MyPoint[] tpArray = { tp1, dp };
                            int x, y;

                            // first turn = left
                            tp1.rotateLeft90();
                            //
                            x = (int) robot.getTwoTurnsDistance();
                            y = (int) robot.getTwoTurnsDistance() / 2;
                            tp1.move(0, dp.y);

                            if (grid.checkIfPathCollides(tpArray)) {
                                // this path works, go next
                                af.transform(tp1, tp1);
                                af.transform(dp, fp);
                                this.plannedPath.add(tp1);
                                this.plannedPath.add(fp);
                                this.plannedPPolygon.addPoint(tp1.x, tp1.y);
                                this.plannedPPolygon.addPoint(fp.x, fp.y);
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

                case FRONT_SLIGHT_LEFT:
                    switch (rOrientation) {
                        case NORTH: {
                            break;
                        }
                        case SOUTH: {
                            break;
                        }
                        case EAST: {
                            break;
                        }
                        case WEST: {
                            break;
                        }
                        case NONE:
                            break;
                        default:
                            break;

                    }
                    break;
                case FRONT_SLIGHT_RIGHT:
                    switch (rOrientation) {
                        case NORTH: {
                            break;
                        }
                        case SOUTH: {
                            break;
                        }
                        case EAST: {
                            break;
                        }
                        case WEST: {
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
                        case NORTH: {
                            break;
                        }
                        case SOUTH: {
                            break;
                        }
                        case EAST: {
                            break;
                        }
                        case WEST: {
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
                        case NORTH: {
                            break;
                        }
                        case SOUTH: {
                            break;
                        }
                        case EAST: {
                            break;
                        }
                        case WEST: {
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
                        case NORTH: {
                            break;
                        }
                        case SOUTH: {
                            break;
                        }
                        case EAST: {
                            break;
                        }
                        case WEST: {
                            break;
                        }
                        case NONE:
                            break;
                        default:
                            break;

                    }
                    break;
                case BACK_SLIGHT_LEFT:
                    switch (rOrientation) {
                        case NORTH: {
                            break;
                        }
                        case SOUTH: {
                            break;
                        }
                        case EAST: {
                            break;
                        }
                        case WEST: {
                            break;
                        }
                        case NONE:
                            break;
                        default:
                            break;

                    }
                    break;
                case BACK_SLIGHT_RIGHT:
                    switch (rOrientation) {
                        case NORTH: {
                            break;
                        }
                        case SOUTH: {
                            break;
                        }
                        case EAST: {
                            break;
                        }
                        case WEST: {
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
                            break;
                        }
                        case SOUTH: {
                            break;
                        }
                        case EAST: {
                            break;
                        }
                        case WEST: {
                            break;
                        }
                        case NONE:
                            break;
                        default:
                            break;

                    }
                    break;
                case CENTER_RIGHT:
                    switch (rOrientation) {
                        case NORTH: {
                            break;
                        }
                        case SOUTH: {
                            break;
                        }
                        case EAST: {
                            break;
                        }
                        case WEST: {
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
    
    public void reset() {
        this.plannedPath.clear();
        this.plannedPPolygon.reset();
    }

}

// input of Points
// need to generate the shortest path based on Points
// need to generated weighted graph
//