package simulator;

import simulator.algorithm.HamiltonianPath;
import simulator.connection.Connection;
import simulator.entity.ComplexInstruction;
import simulator.entity.Direction;
import simulator.entity.Grid;
import simulator.entity.MyPoint;
import simulator.entity.Robot;

import javax.swing.*;
import java.awt.*;
import java.awt.geom.Rectangle2D;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;
import java.util.logging.Level;
import java.util.logging.Logger;

class Simulator {

    private static final int ENVIRONMENT_SCALING_FACTOR = 3;
    private static final double ROBOT_SIZE = 60;
    private static final int DISTANCE_MARGIN_OF_ERROR = 40;
    private static Connection connection;
    public static String host = "192.168.13.13";
    public static int port = 3053;
    private final Logger logger = Logger.getLogger(Simulator.class.getName());
    private final Dimension environmentActualSize = new Dimension(200, 200);
    private final Dimension robotActualSize = new Dimension(20, 20);
    private final double robotActualDistanceBetweenFrontBackWheels = 14.5;
    private final Point robotActualStartingPoint = new Point(20, 180);
    private JFrame jFrame;
    private Grid grid;
    private Robot robot;
    private HamiltonianPath hPath;
    private MyPoint[] shortestPath = new MyPoint[0];
    private Thread gameLoop;
    private boolean isRunning = false;
    private boolean newPath = true;
    private Queue<ComplexInstruction> instructions = new LinkedList<>();
    private Thread movementsLoop;
    private boolean isRunning2 = false;
    private long startTime;
    private JLabel timerLabel = new JLabel();

    public static void main(String[] args) {

        boolean rpiConnect = false; //set to true to test connection

        if (rpiConnect) {
            connection = Connection.getConnection();
            connection.openConnection(host, port);
            connection.sendMsg("C", "type"); //C take pic
        }

        // need to use this utility to call the initial method that draws GUI
        SwingUtilities.invokeLater(() -> new Simulator().createAndShowGUI());
    }

    public void createAndShowGUI() {
        jFrame = new JFrame("Simulator");
        jFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        Dimension environmentModelSize = new Dimension(environmentActualSize.width * ENVIRONMENT_SCALING_FACTOR,
                environmentActualSize.height * ENVIRONMENT_SCALING_FACTOR);

        Dimension robotModelSize = new Dimension(robotActualSize.width * ENVIRONMENT_SCALING_FACTOR,
                robotActualSize.height * ENVIRONMENT_SCALING_FACTOR);

        Point robotModelStartingPoint = new Point(robotActualStartingPoint.x * ENVIRONMENT_SCALING_FACTOR,
                robotActualStartingPoint.y * ENVIRONMENT_SCALING_FACTOR);

        double robotModelDistanceBetweenFrontBackWheels = robotActualDistanceBetweenFrontBackWheels
                * ENVIRONMENT_SCALING_FACTOR;

        JLayeredPane layeredPane = new JLayeredPane();
        layeredPane.setPreferredSize(new Dimension(600, 600)); // hardcoded for now
        // use layered pane
        // grid in one layer
        // robot as one layer
        // path as one layer

        grid = new Grid(environmentModelSize);
        grid.setSize(environmentModelSize);
        // grid.setPreferredSize(new Dimension(600, 600));
        layeredPane.add(grid, 0, 0);
        // (x, y, z) y is layer, z is depth
        // bigger y is higher up
        // smaller z is higher up

        robot = new Robot(robotModelSize, robotModelStartingPoint, robotModelDistanceBetweenFrontBackWheels, ENVIRONMENT_SCALING_FACTOR);
        robot.setSize(environmentModelSize);
        layeredPane.add(robot, 1, 0);

        hPath = new HamiltonianPath();
        hPath.setSize(environmentModelSize);
        layeredPane.add(hPath, 2, 0);

        // add surrounding boxes as margins
        jFrame.add(Box.createRigidArea(new Dimension(20, 20)), BorderLayout.PAGE_START);
        jFrame.add(Box.createRigidArea(new Dimension(20, 20)), BorderLayout.LINE_START);
        jFrame.add(Box.createRigidArea(new Dimension(20, 20)), BorderLayout.LINE_END);
        jFrame.add(Box.createRigidArea(new Dimension(20, 20)), BorderLayout.PAGE_END);

        // add the center layered pane to the window
        jFrame.add(layeredPane, BorderLayout.CENTER);

        // make panel for right side of window
        JPanel rightPanel = new JPanel();
        rightPanel.setLayout(new BoxLayout(rightPanel, BoxLayout.PAGE_AXIS));
        jFrame.add(rightPanel, BorderLayout.LINE_END);

        // add buttons
        /*JButton drawButton = new JButton("Draw");
        drawButton.addActionListener(e -> {
            logger.log(Level.FINE, "Start Button Clicked");

            shortestPath = hPath.getShortestPath(robot.getCurrentLocation(), grid.getObstacleFronts());
        });*/

        JButton forwardButton = new JButton("Forward");
        forwardButton.addActionListener(e -> robot.moveForward());

        JButton backwardButton = new JButton("Backward");
        backwardButton.addActionListener(e -> robot.moveBackward());

        JButton leftButton = new JButton("Turn left");
        leftButton.addActionListener(e -> robot.turnLeft());

        JButton rightButton = new JButton("Turn right");
        rightButton.addActionListener(e -> robot.turnRight());

        /*JButton startButton = new JButton("Start");
        startButton.addActionListener(e -> {
            setupGameLoop();
            isRunning = true;
            gameLoop.start();
            logger.log(Level.FINE, "Start");
        });*/

        /*JButton stopButton = new JButton("Stop");
        stopButton.addActionListener(e -> isRunning = false);*/

        JButton resetButton = new JButton("Reset");
        resetButton.addActionListener(e -> {
            robot.reset();
            isRunning = false;
            isRunning2 = false;
            instructions = new LinkedList<>();
            hPath.reset();
        });

        JButton drawButton2 = new JButton("Draw Path");
        drawButton2.addActionListener(e -> {
            logger.log(Level.FINE, "Start Button Clicked");

            //     shortestPath = hPath.getShortestPath(robot.getCurrentLocation(), grid.getObstacleFronts());
            hPath.getShortestPath(robot.getCurrentLocation(), grid.getObstacleFronts(), true);
            hPath.generatePlannedPath(grid, robot);
            hPath.printPlannedPath();
            robot.generateMovements(hPath.getPlannedPath());
            robot.printGeneratedMovements();
            grid.repaint();
        });

        JButton startMovementsButton = new JButton("Start Movements");
        startMovementsButton.addActionListener(e -> {
            startTime = System.nanoTime();
            startMovements();
            isRunning2 = true;
            movementsLoop.start();
            logger.log(Level.FINE, "Start");
        });

        JButton connectButton = new JButton("Connect to RPI");
        connectButton.addActionListener(e -> {
            System.out.println("Connecting");
            connection = Connection.getConnection();
            connection.openConnection(host, port);
            if (connection.isConnected()) {
                System.out.println("Connection opened");
                connection.sendMsg("R", "type");
            }
        });

        JButton disconnectButton = new JButton("Disconnect from RPI");
        connectButton.addActionListener(e -> {
            System.out.println("Disconnecting");
            connection.closeConnection();
        });

        JButton sendMovements = new JButton("Send movements");
        sendMovements.addActionListener(e -> {
            ArrayList<String> commands = robot.getCommandsToSend();
            System.out.println("----------Sending movements...");
            for (String s : commands) {
                if (s.compareTo("Reached") == 0 ) {
                    // stop sending commannds for subsequent obstacle
                    break;
                }
                System.out.println(s);
                if (connection!=null && connection.isConnected()) {
                    connection.sendMsg(s, "command");
                }
            }
        });

        //rightPanel.add(drawButton);
        rightPanel.add(forwardButton);
        rightPanel.add(backwardButton);
        rightPanel.add(leftButton);
        rightPanel.add(rightButton);
        /*rightPanel.add(startButton);
        rightPanel.add(stopButton);*/
        rightPanel.add(resetButton);
        rightPanel.add(drawButton2);
        rightPanel.add(startMovementsButton);
        rightPanel.add(connectButton);
        rightPanel.add(disconnectButton);
        rightPanel.add(sendMovements);
        rightPanel.add(timerLabel);

        jFrame.pack();
        jFrame.setLocationRelativeTo(null);
        jFrame.setVisible(true); // now frame will be visible, by default not visible
    }

    public void setupGameLoop() {
        gameLoop = new Thread(() -> {
            logger.log(Level.FINEST, "Thread");
            /*
             * while (isRunning) { logger.log(Level.FINEST, "Loop"); robot.moveForward();
             *
             * robot.repaint(); try { Thread.sleep(15); } catch (InterruptedException ex) {
             * ex.printStackTrace(); } }
             */

            int index = 1;
            boolean reached = false;

            Rectangle2D[] obstacleBoundaries = grid.getObstacleBoundaries();

            // First iteration
            moveByPath(shortestPath[index]);
            ComplexInstruction nextInstruction = instructions.peek();

            while (isRunning && index < shortestPath.length) {
                logger.log(Level.FINEST, "Loop");

                if (reached) {
                    // nextInstruction = instructions.poll();
                    instructions.poll();
                    nextInstruction = instructions.peek();
                    reached = false;

                    if (instructions.isEmpty()) {
                        index++;
                        if (index >= shortestPath.length) {
                            System.out.println("Reached point!");
                            break;
                        }
                        moveByPath(shortestPath[index]);
                        nextInstruction = instructions.peek();
                    }
                }
                switch (nextInstruction.getInstruction()) {
                case FORWARD:
                    reached = robot.moveForwardWithChecking(shortestPath[index], DISTANCE_MARGIN_OF_ERROR,
                            nextInstruction.getFinalDirection(), nextInstruction, obstacleBoundaries);
                    break;
                case REVERSE:
                    reached = robot.moveBackwardWithChecking(shortestPath[index], DISTANCE_MARGIN_OF_ERROR,
                            nextInstruction.getFinalDirection(), nextInstruction);
                    break;
                case FORWARD_LEFT:
                    reached = robot.moveForwardLeftWithChecking(nextInstruction.getFinalDirection());
                    break;
                case FORWARD_RIGHT:
                    reached = robot.moveForwardRightWithChecking(nextInstruction.getFinalDirection());
                    break;
                }

                // System.out.println("Robot's Instruction: " +
                // nextInstruction.getInstruction().name());

                robot.repaint();
                try {
                    Thread.sleep(15);
                } catch (InterruptedException ex) {
                    ex.printStackTrace();
                }
            }
        });
    }

    /**
     * Choose correct path and follow that path.
     *
     * @param currPoint Point to move to.
     */
    private void moveByPath(MyPoint currPoint) {

        Direction robotDirection = robot.getGeneralDirection();
        Direction pointDirection = currPoint.getDirection();

        MyPoint robotLocation = robot.getCurrentLocation();

        // make sure there's direction for both robot and point
        assert robotDirection.ordinal() != 0 && pointDirection.ordinal() != 0;

        if (Math.abs(pointDirection.ordinal() - robotDirection.ordinal()) == 2) {
            // 1) Robot and image facing opposite directions
            logger.log(Level.INFO, "OPPOSITE");

            switch (robotDirection) {
            case NORTH:
                if (robotLocation.getX() - ROBOT_SIZE / 2 <= currPoint.getX()
                        && robotLocation.getX() + ROBOT_SIZE / 2 >= currPoint.getX()
                        && currPoint.getY() <= robotLocation.getY()) {
                    // (a)
                    System.out.println("(a)");
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                } else if (robotLocation.getX() + ROBOT_SIZE / 2 <= currPoint.getX()
                        && currPoint.getY() + robot.getTwoTurnsDistance() <= robotLocation.getY()) {
                    // (b)
                    System.out.println("(b)");
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, Direction.EAST));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.EAST));
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, robotDirection));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                } else if (robotLocation.getX() - ROBOT_SIZE / 2 >= currPoint.getX()
                        && currPoint.getY() + robot.getTwoTurnsDistance() <= robotLocation.getY()) {
                    // (c)
                    System.out.println("(c)");
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, Direction.WEST));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.WEST));
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, robotDirection));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                } else if (robotLocation.getX() + ROBOT_SIZE / 2 <= currPoint.getX()
                        && currPoint.getY() + robot.getTwoTurnsDistance() >= robotLocation.getY()) {
                    // (d)
                    System.out.println("(d)");
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.REVERSE, robotDirection,
                            currPoint.getY() - robotLocation.getY() + robot.getTwoTurnsDistance()));
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, Direction.EAST));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.EAST));
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, robotDirection));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                } else if (robotLocation.getX() - ROBOT_SIZE / 2 >= currPoint.getX()
                        && currPoint.getY() + robot.getTwoTurnsDistance() >= robotLocation.getY()) {
                    // (e)
                    System.out.println("(e)");
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.REVERSE, robotDirection,
                            currPoint.getY() - robotLocation.getY() + robot.getTwoTurnsDistance()));
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, Direction.WEST));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.WEST));
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, robotDirection));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                }
                break;
            case SOUTH:
                if (robotLocation.getX() - ROBOT_SIZE / 2 <= currPoint.getX()
                        && robotLocation.getX() + ROBOT_SIZE / 2 >= currPoint.getX()
                        && currPoint.getY() >= robotLocation.getY()) {
                    // (a)
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, robotDirection));
                } else if (robotLocation.getX() + ROBOT_SIZE / 2 <= currPoint.getX()
                        && currPoint.getY() - robot.getTwoTurnsDistance() >= robotLocation.getY()) {
                    // (b)
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, Direction.WEST));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.WEST));
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, robotDirection));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, robotDirection));
                } else if (robotLocation.getX() - ROBOT_SIZE / 2 >= currPoint.getX()
                        && currPoint.getY() - robot.getTwoTurnsDistance() >= robotLocation.getY()) {
                    // (c)
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, Direction.EAST));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.EAST));
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, robotDirection));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, robotDirection));
                } else if (robotLocation.getX() + ROBOT_SIZE / 2 <= currPoint.getX()
                        && currPoint.getY() - robot.getTwoTurnsDistance() <= robotLocation.getY()) {
                    // (d)
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.REVERSE, robotDirection,
                            robotLocation.getY() - currPoint.getY() + robot.getTwoTurnsDistance()));
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, Direction.WEST));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.WEST));
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, robotDirection));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, robotDirection));
                } else if (robotLocation.getX() - ROBOT_SIZE / 2 >= currPoint.getX()
                        && currPoint.getY() - robot.getTwoTurnsDistance() <= robotLocation.getY()) {
                    // (e)
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.REVERSE, robotDirection,
                            robotLocation.getY() - currPoint.getY() + robot.getTwoTurnsDistance()));
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, Direction.EAST));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.EAST));
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, robotDirection));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, robotDirection));
                }
                break;
            case EAST:
                if (robotLocation.getY() - ROBOT_SIZE / 2 <= currPoint.getY()
                        && robotLocation.getY() + ROBOT_SIZE / 2 >= currPoint.getY()
                        && currPoint.getX() <= robotLocation.getX()) {
                    // (a)
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, robotDirection));
                } else if (robotLocation.getY() + ROBOT_SIZE / 2 <= currPoint.getY()
                        && robotLocation.getX() + robot.getTwoTurnsDistance() <= robotLocation.getX()) {
                    // (b)
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, Direction.SOUTH));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.SOUTH));
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, robotDirection));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, robotDirection));
                } else if (robotLocation.getY() - ROBOT_SIZE / 2 >= currPoint.getY()
                        && robotLocation.getX() + robot.getTwoTurnsDistance() <= robotLocation.getX()) {
                    // (c)
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, Direction.NORTH));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NORTH));
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, robotDirection));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, robotDirection));
                } else if (robotLocation.getY() + ROBOT_SIZE / 2 <= currPoint.getY()
                        && robotLocation.getX() + robot.getTwoTurnsDistance() >= robotLocation.getX()) {
                    // (d)
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.REVERSE, robotDirection,
                            robotLocation.getX() - currPoint.getX() + robot.getTwoTurnsDistance()));
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, Direction.SOUTH));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.SOUTH));
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, robotDirection));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, robotDirection));
                } else if (robotLocation.getY() - ROBOT_SIZE / 2 >= currPoint.getY()
                        && robotLocation.getX() + robot.getTwoTurnsDistance() >= robotLocation.getX()) {
                    // (e)
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.REVERSE, robotDirection,
                            robotLocation.getX() - currPoint.getX() + robot.getTwoTurnsDistance()));
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, Direction.NORTH));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NORTH));
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, robotDirection));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, robotDirection));
                }
                break;
            case WEST:
                if (robotLocation.getY() - ROBOT_SIZE / 2 <= currPoint.getY()
                        && robotLocation.getY() + ROBOT_SIZE / 2 >= currPoint.getY()
                        && currPoint.getX() >= robotLocation.getX()) {
                    // (a)
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, robotDirection));
                } else if (robotLocation.getY() + ROBOT_SIZE / 2 <= currPoint.getY()
                        && robotLocation.getX() - robot.getTwoTurnsDistance() >= robotLocation.getX()) {
                    // (b)
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, Direction.NORTH));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NORTH));
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, robotDirection));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, robotDirection));
                } else if (robotLocation.getY() - ROBOT_SIZE / 2 >= currPoint.getY()
                        && robotLocation.getX() - robot.getTwoTurnsDistance() >= robotLocation.getX()) {
                    // (c)
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, Direction.SOUTH));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.SOUTH));
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, robotDirection));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, robotDirection));
                } else if (robotLocation.getY() + ROBOT_SIZE / 2 <= currPoint.getY()
                        && robotLocation.getX() + robot.getTwoTurnsDistance() <= robotLocation.getX()) {
                    // (d)
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.REVERSE, robotDirection,
                            currPoint.getX() - robotLocation.getX() + robot.getTwoTurnsDistance()));
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, Direction.NORTH));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NORTH));
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, robotDirection));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, robotDirection));
                } else if (robotLocation.getY() - ROBOT_SIZE / 2 >= currPoint.getY()
                        && robotLocation.getX() + robot.getTwoTurnsDistance() <= robotLocation.getX()) {
                    // (e)
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.REVERSE, robotDirection,
                            currPoint.getX() - robotLocation.getX() + robot.getTwoTurnsDistance()));
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, Direction.SOUTH));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.SOUTH));
                    instructions.add(
                            new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, robotDirection));
                    instructions
                            .add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, robotDirection));
                }
                break;
            }

        } else if (Math.abs(pointDirection.ordinal() - robotDirection.ordinal()) == 1
                || Math.abs(pointDirection.ordinal() - robotDirection.ordinal()) == 3) {
            // 2) Robot and image facing has a difference of pi/2 or -pi/2
            logger.log(Level.INFO, "pi/2 & -pi/2");

            switch (robotDirection) {
            case NORTH:
                if (robotLocation.getX() - ROBOT_SIZE / 2 >= currPoint.getX()
                        && currPoint.getY() <= robotLocation.getY()) {
                    // (a)
                    System.out.println("(a)");
                    switch (pointDirection) {
                    case WEST:
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.WEST));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.WEST,
                                        robotLocation.getX() - currPoint.getX() + robot.getTwoTurnsDistance()));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.NORTH));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NORTH,
                                        currPoint.getY() - robotLocation.getY() - robot.getTwoTurnsDistance()));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.EAST));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    case EAST:
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD,
                                Direction.NORTH));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.WEST));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    }

                } else if (robotLocation.getX() + ROBOT_SIZE / 2 <= currPoint.getX()
                        && currPoint.getY() <= robotLocation.getY()) {
                    // (b)
                    System.out.println("(b)");
                    switch (pointDirection) {
                    case WEST:
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD,
                                Direction.NORTH));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.EAST));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    case EAST:
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.EAST));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD,
                                Direction.EAST, currPoint.getX() - robotLocation.getX()));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.NORTH));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD,
                                Direction.NORTH));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.WEST));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    }
                } else if (robotLocation.getX() - ROBOT_SIZE / 2 >= currPoint.getX()
                        && currPoint.getY() >= robotLocation.getY()) {
                    // (c)
                    System.out.println("(c)");
                    switch (pointDirection) {
                    case WEST:
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.WEST));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD,
                                Direction.WEST, robotLocation.getX() - currPoint.getX()));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.SOUTH));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD,
                                Direction.SOUTH));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.EAST));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    case EAST:
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.REVERSE, Direction.NORTH,
                                        currPoint.getY() - robotLocation.getY() + robot.getTwoTurnsDistance()));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD,
                                Direction.NORTH));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.WEST));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    }
                } else if (robotLocation.getX() + ROBOT_SIZE / 2 <= currPoint.getX()
                        && currPoint.getY() >= robotLocation.getY()) {
                    // (c)
                    System.out.println("(c)");
                    switch (pointDirection) {
                    case WEST:
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.REVERSE, Direction.NORTH,
                                        currPoint.getY() - robotLocation.getY() + robot.getTwoTurnsDistance()));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD,
                                Direction.NORTH));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.EAST));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    case EAST:
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.EAST));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD,
                                Direction.EAST, currPoint.getX() - robotLocation.getX()));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.SOUTH));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD,
                                Direction.SOUTH));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.WEST));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    }
                }
                break;
            case SOUTH:
                if (robotLocation.getX() + ROBOT_SIZE / 2 <= currPoint.getX()
                        && currPoint.getY() >= robotLocation.getY()) {
                    // (a)
                    System.out.println("(a)");
                    switch (pointDirection) {
                    case EAST:
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.EAST));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.EAST,
                                        currPoint.getX() - robotLocation.getX() + robot.getTwoTurnsDistance()));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.SOUTH));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.SOUTH,
                                        robotLocation.getY() - currPoint.getY() - robot.getTwoTurnsDistance()));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.WEST));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    case WEST:
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD,
                                Direction.SOUTH));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.EAST));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    }

                } else if (robotLocation.getX() - ROBOT_SIZE / 2 >= currPoint.getX()
                        && currPoint.getY() >= robotLocation.getY()) {
                    // (b)
                    System.out.println("(b)");
                    switch (pointDirection) {
                    case EAST:
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD,
                                Direction.SOUTH));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.WEST));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    case WEST:
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.WEST));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD,
                                Direction.WEST, robotLocation.getX() - currPoint.getX()));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.SOUTH));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD,
                                Direction.SOUTH));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.EAST));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    }
                } else if (robotLocation.getX() + ROBOT_SIZE / 2 <= currPoint.getX()
                        && currPoint.getY() <= robotLocation.getY()) {
                    // (c)
                    System.out.println("(c)");
                    switch (pointDirection) {
                    case EAST:
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.EAST));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD,
                                Direction.EAST, currPoint.getX() - robotLocation.getX()));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.NORTH));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD,
                                Direction.NORTH));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.WEST));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    case WEST:
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.REVERSE, Direction.SOUTH,
                                        robotLocation.getY() - currPoint.getY() + robot.getTwoTurnsDistance()));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD,
                                Direction.SOUTH));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.EAST));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    }
                } else if (robotLocation.getX() - ROBOT_SIZE / 2 >= currPoint.getX()
                        && currPoint.getY() <= robotLocation.getY()) {
                    // (c)
                    System.out.println("(c)");
                    switch (pointDirection) {
                    case EAST:
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.REVERSE, Direction.SOUTH,
                                        robotLocation.getY() - currPoint.getY() + robot.getTwoTurnsDistance()));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD,
                                Direction.SOUTH));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.WEST));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    case WEST:
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.WEST));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD,
                                Direction.WEST, robotLocation.getX() - currPoint.getX()));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.NORTH));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD,
                                Direction.NORTH));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.EAST));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    }
                }
                break;
            case EAST:
                if (robotLocation.getY() - ROBOT_SIZE / 2 >= currPoint.getY()
                        && currPoint.getX() >= robotLocation.getX()) {
                    // (a)
                    System.out.println("(a)");
                    switch (pointDirection) {
                    case NORTH:
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.NORTH));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NORTH,
                                        robotLocation.getY() - currPoint.getY() + robot.getTwoTurnsDistance()));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.EAST));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.EAST,
                                        currPoint.getX() - robotLocation.getX() - robot.getTwoTurnsDistance()));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.SOUTH));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    case SOUTH:
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.EAST));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.NORTH));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    }

                } else if (robotLocation.getY() + ROBOT_SIZE / 2 <= currPoint.getY()
                        && currPoint.getX() >= robotLocation.getX()) {
                    // (b)
                    System.out.println("(b)");
                    switch (pointDirection) {
                    case NORTH:
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.EAST));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.SOUTH));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    case SOUTH:
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.SOUTH));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD,
                                Direction.SOUTH, currPoint.getY() - robotLocation.getY()));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.EAST));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.EAST));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.NORTH));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    }
                } else if (robotLocation.getX() - ROBOT_SIZE / 2 >= currPoint.getX()
                        && currPoint.getY() <= robotLocation.getY()) {
                    // (c)
                    System.out.println("(c)");
                    switch (pointDirection) {
                    case NORTH:
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.NORTH));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD,
                                Direction.NORTH, robotLocation.getY() - currPoint.getY()));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.WEST));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.WEST));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.SOUTH));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    case SOUTH:
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.REVERSE, Direction.EAST,
                                        robotLocation.getX() - currPoint.getX() + robot.getTwoTurnsDistance()));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.EAST));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.NORTH));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    }
                } else if (robotLocation.getX() - ROBOT_SIZE / 2 >= currPoint.getX()
                        && currPoint.getY() >= robotLocation.getY()) {
                    // (c)
                    System.out.println("(c)");
                    switch (pointDirection) {
                    case NORTH:
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.REVERSE, Direction.EAST,
                                        robotLocation.getX() - currPoint.getX() + robot.getTwoTurnsDistance()));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.EAST));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.SOUTH));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    case SOUTH:
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.SOUTH));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD,
                                Direction.SOUTH, currPoint.getY() - robotLocation.getY()));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.WEST));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.WEST));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.NORTH));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    }
                }
                break;
            case WEST:
                if (robotLocation.getY() + ROBOT_SIZE / 2 <= currPoint.getY()
                        && currPoint.getX() <= robotLocation.getX()) {
                    // (a)
                    System.out.println("(a)");
                    switch (pointDirection) {
                    case NORTH:
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.WEST));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.SOUTH));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    case SOUTH:
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.SOUTH));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.SOUTH,
                                        currPoint.getY() - robotLocation.getY() + robot.getTwoTurnsDistance()));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.WEST));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.WEST,
                                        robotLocation.getX() - currPoint.getX() - robot.getTwoTurnsDistance()));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.NORTH));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    }

                } else if (robotLocation.getY() - ROBOT_SIZE / 2 >= currPoint.getY()
                        && currPoint.getX() <= robotLocation.getX()) {
                    // (b)
                    System.out.println("(b)"); // RESUME!!!!
                    switch (pointDirection) {
                    case NORTH:
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.NORTH));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD,
                                Direction.SOUTH, robotLocation.getY() - currPoint.getY()));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.WEST));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.WEST));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.SOUTH));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    case SOUTH:
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.WEST));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.NORTH));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    }
                } else if (robotLocation.getX() + ROBOT_SIZE / 2 <= currPoint.getX()
                        && currPoint.getY() >= robotLocation.getY()) {
                    // (c)
                    System.out.println("(c)");
                    switch (pointDirection) {
                    case NORTH:
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.REVERSE, Direction.WEST,
                                        currPoint.getX() - robotLocation.getX() + robot.getTwoTurnsDistance()));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.WEST));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.SOUTH));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    case SOUTH:
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.SOUTH));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD,
                                Direction.SOUTH, currPoint.getY() - robotLocation.getY()));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.EAST));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.EAST));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT,
                                Direction.NORTH));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    }
                } else if (robotLocation.getX() + ROBOT_SIZE / 2 <= currPoint.getX()
                        && currPoint.getY() <= robotLocation.getY()) {
                    // (c)
                    System.out.println("(c)");
                    switch (pointDirection) {
                    case NORTH:
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.NORTH));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD,
                                Direction.NORTH, robotLocation.getY() - currPoint.getY()));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.EAST));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.EAST));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.SOUTH));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    case SOUTH:
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.REVERSE, Direction.WEST,
                                        currPoint.getX() - robotLocation.getX() + robot.getTwoTurnsDistance()));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.WEST));
                        instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT,
                                Direction.NORTH));
                        instructions.add(
                                new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                        break;
                    }
                }
                break;
            }

        } else if (pointDirection.ordinal() == robotDirection.ordinal()) {
            // 3) Robot and image facing the same direction
            logger.log(Level.INFO, "SAME");
            switch (robotDirection) {
            case NORTH:
                instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NORTH,
                        robotLocation.getY() - currPoint.getY()));
                instructions.add(robot.getX() > currPoint.getX()
                        ? new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, Direction.WEST)
                        : new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, Direction.EAST));
                instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD,
                        robot.getX() > currPoint.getX() ? Direction.WEST : Direction.EAST));
                instructions.add(robot.getX() > currPoint.getX()
                        ? new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, Direction.SOUTH)
                        : new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, Direction.SOUTH));
                instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.SOUTH));
                break;
            case SOUTH:
                instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.SOUTH,
                        currPoint.getY() - robotLocation.getY()));
                instructions.add(robot.getX() > currPoint.getX()
                        ? new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, Direction.WEST)
                        : new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, Direction.EAST));
                instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD,
                        robot.getX() > currPoint.getX() ? Direction.WEST : Direction.EAST));
                instructions.add(robot.getX() > currPoint.getX()
                        ? new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, Direction.NORTH)
                        : new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, Direction.NORTH));
                instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NORTH));
                break;
            }
        }

        // return instructions;
    }

    public void startMovements() {
        movementsLoop = new Thread(() -> {
            logger.log(Level.FINEST, "Thread");

            long lastLoopTime = System.nanoTime();
            final int TARGET_FPS = 120;
            final long OPTIMAL_TIME = 1000000000 / TARGET_FPS;
            long lastFpsTime = 0;

            //     double timeFor90DegTurn = robot.getTimeFor90DegTurn();
            //     System.out.println(timeFor90DegTurn);
            //     // add instructions
            //     robot.addToQueue("Forward", 10.0);
            //     robot.addToQueue("Right", 1);
            //     robot.addToQueue("Forward", timeFor90DegTurn);

            while (isRunning2) {
                logger.log(Level.FINEST, "Start Movements");

                long now = System.nanoTime();
                long updateLength = now - lastLoopTime;
                lastLoopTime = now;
                double delta = updateLength / ((double) OPTIMAL_TIME);
                // delta is the number of frames that has passed

                lastFpsTime += updateLength;
                if (lastFpsTime >= 1000000000) {
                    lastFpsTime = 0;
                }

                if (robot.letsGo(delta)) {
                    break;
                }

                robot.repaint();
                timerLabel.setFont(new Font("Serif", Font.PLAIN, 20));
                DecimalFormat df = new DecimalFormat("#.##");
                timerLabel.setText("Time:" + df.format((double) (System.nanoTime() - startTime) / 1000000000) + "s");

                try {
                    Thread.sleep((lastLoopTime - System.nanoTime() + OPTIMAL_TIME) / 1000000);
                } catch (Exception e) {
                    e.printStackTrace();
                }

            }
        });
    }
}
