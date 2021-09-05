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
import java.util.LinkedList;
import java.util.Queue;
import java.util.logging.Level;
import java.util.logging.Logger;

class Simulator {

    public static String host = "192.168.13.13";
    public static int port = 3053;

    private final Logger logger = Logger.getLogger(Simulator.class.getName());

    private JFrame jFrame;
    private Grid grid;
    private Robot robot;
    private HamiltonianPath hPath;
    private final Dimension environmentActualSize = new Dimension(200, 200);
    private static final int ENVIRONMENT_SCALING_FACTOR = 3;

    private final Dimension robotActualSize = new Dimension(30, 30);
    private final Point robotActualStartingPoint = new Point(20, 180);

    private MyPoint[] shortestPath = new MyPoint[0];
    private Thread gameLoop;
    private boolean isRunning = false;
    private boolean newPath = true;

    private final Queue<ComplexInstruction> instructions = new LinkedList<>();

    private static final double ROBOT_SIZE = 60;
    private static final int DISTANCE_MARGIN_OF_ERROR = 40;

    public void createAndShowGUI() {
        jFrame = new JFrame("Simulator");
        jFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        Dimension environmentModelSize = new Dimension(environmentActualSize.width * ENVIRONMENT_SCALING_FACTOR,
                environmentActualSize.height * ENVIRONMENT_SCALING_FACTOR);

        Dimension robotModelSize = new Dimension(robotActualSize.width * ENVIRONMENT_SCALING_FACTOR,
                robotActualSize.height * ENVIRONMENT_SCALING_FACTOR);

        Point robotModelStartingPoint = new Point(robotActualStartingPoint.x * ENVIRONMENT_SCALING_FACTOR,
                robotActualStartingPoint.y * ENVIRONMENT_SCALING_FACTOR);

        JLayeredPane layeredPane = new JLayeredPane();
        layeredPane.setPreferredSize(new Dimension(800, 800)); // hardcoded for now
        // use layered pane
        // grid in one layer
        // robot as one layer
        // path as one layer

        grid = new Grid();
        grid.setSize(environmentModelSize);
        // grid.setPreferredSize(new Dimension(600, 600));
        layeredPane.add(grid, 0, 0);
        // (x, y, z) y is layer, z is depth
        // bigger y is higher up
        // smaller z is higher up

        robot = new Robot(robotModelSize, robotModelStartingPoint);
        robot.setSize(environmentModelSize);
        layeredPane.add(robot, 1, 0);

        hPath = new HamiltonianPath();
        hPath.setSize(environmentModelSize);
        layeredPane.add(hPath, 2, 0);

        // add surrounding boxes as margins
        jFrame.add(Box.createRigidArea(new Dimension(50, 50)), BorderLayout.PAGE_START);
        jFrame.add(Box.createRigidArea(new Dimension(50, 50)), BorderLayout.LINE_START);
        jFrame.add(Box.createRigidArea(new Dimension(50, 50)), BorderLayout.LINE_END);
        jFrame.add(Box.createRigidArea(new Dimension(50, 50)), BorderLayout.PAGE_END);

        // add the center layered pane to the window
        jFrame.add(layeredPane, BorderLayout.CENTER);

        // make panel for right side of window
        JPanel rightPanel = new JPanel();
        rightPanel.setLayout(new BoxLayout(rightPanel, BoxLayout.PAGE_AXIS));
        jFrame.add(rightPanel, BorderLayout.LINE_END);

        // add buttons
        JButton drawButton = new JButton("Draw");
        drawButton.addActionListener(e -> {
            logger.log(Level.FINE, "Start Button Clicked");
            // Point origin = new Point(0, 0);

            // TODO: pass obstacle cells as well, to determine direction.
            //hPath.getShortestPath(robotModelStartingPoint, grid.getObstacleCenters());
            shortestPath = hPath.getShortestPath(robot.getCurrentLocation(), grid.getObstacleFronts());
        });

        JButton forwardButton = new JButton("Forward");
        forwardButton.addActionListener(e -> robot.moveForward());

        JButton backwardButton = new JButton("Backward");
        backwardButton.addActionListener(e -> robot.moveBackward());

        JButton leftButton = new JButton("Turn left");
        leftButton.addActionListener(e -> robot.turnLeft());

        JButton rightButton = new JButton("Turn right");
        rightButton.addActionListener(e -> robot.turnRight());

        JButton startButton = new JButton("Start");
        startButton.addActionListener(e -> {
            setupGameLoop();
            isRunning = true;
            gameLoop.start();
            logger.log(Level.FINE, "Start");
        });

        JButton stopButton = new JButton("Stop");
        stopButton.addActionListener(e -> isRunning = false);

        rightPanel.add(drawButton);
        rightPanel.add(forwardButton);
        rightPanel.add(backwardButton);
        rightPanel.add(leftButton);
        rightPanel.add(rightButton);
        rightPanel.add(startButton);
        rightPanel.add(stopButton);

        jFrame.pack();
        jFrame.setVisible(true); // now frame will be visible, by default not visible
    }

    public static void main(String[] args) {

        boolean rpiConnect = false;

        if (rpiConnect) {
            Connection connection = Connection.getConnection();
            connection.openConnection(host, port);
        }

        // need to use this utility to call the initial method that draws GUI
        SwingUtilities.invokeLater(() -> new Simulator().createAndShowGUI());
    }

    public void setupGameLoop() {
        gameLoop = new Thread(() -> {
            logger.log(Level.FINEST, "Thread");
            /*while (isRunning) {
                logger.log(Level.FINEST, "Loop");
                robot.moveForward();

                robot.repaint();
                try {
                    Thread.sleep(15);
                } catch (InterruptedException ex) {
                    ex.printStackTrace();
                }
            }*/

            int index = 0;
            boolean reached = false;


            // First iteration
            moveByPath(shortestPath[index]);
            ComplexInstruction nextInstruction = instructions.peek();

            while (isRunning && index < shortestPath.length - 1) {
                logger.log(Level.FINEST, "Loop");


                if (reached) {
                    //nextInstruction = instructions.poll();
                    instructions.poll();
                    nextInstruction = instructions.peek();
                    reached = false;


                    if (instructions.isEmpty()) {
                        index++;
                        if (index >= shortestPath.length - 1) {
                            System.out.println("Reached point!");
                            break;
                        }
                        moveByPath(shortestPath[index]);
                    }
                }
                switch (nextInstruction.getInstruction()) {
                case FORWARD:
                    reached = robot.moveForwardWithChecking(shortestPath[index], DISTANCE_MARGIN_OF_ERROR, nextInstruction.getFinalDirection());
                    break;
                case REVERSE:
                    reached = robot.moveBackwardWithChecking(shortestPath[index], DISTANCE_MARGIN_OF_ERROR, nextInstruction.getFinalDirection(), nextInstruction);
                    break;
                case FORWARD_LEFT:
                    reached = robot.moveForwardLeftWithChecking(nextInstruction.getFinalDirection());
                    break;
                case FORWARD_RIGHT:
                    reached = robot.moveForwardRightWithChecking(nextInstruction.getFinalDirection());
                    break;
                }


                //System.out.println("Robot's Instruction: " + nextInstruction.getInstruction().name());

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

        //ArrayList<Instruction> instructions = new ArrayList<>();

        Direction robotDirection = robot.getGeneralDirection();
        Direction pointDirection = currPoint.getDirection();

        MyPoint robotLocation = robot.getCurrentLocation();

        assert robotDirection.ordinal() != 0 && pointDirection.ordinal() != 0;

        if (Math.abs(pointDirection.ordinal() - robotDirection.ordinal()) == 2) {
            // 1) Robot and image facing opposite directions
            logger.log(Level.INFO, "OPPOSITE");

            switch (robotDirection) {
            case NORTH:
                if (robotLocation.getX() - ROBOT_SIZE / 2 <= currPoint.getX() && robotLocation.getX() + ROBOT_SIZE / 2 >= currPoint.getX() && currPoint.getY() <= robotLocation.getY()) {
                    // (a)
                    System.out.println("(a)");
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                } else if (robotLocation.getX() + ROBOT_SIZE / 2 <= currPoint.getX() && currPoint.getY() + robot.getTwoTurnsDistance() <= robotLocation.getY()) {
                    // (b)
                    System.out.println("(b)");
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, Direction.EAST));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.EAST));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, robotDirection));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                } else if (robotLocation.getX() - ROBOT_SIZE / 2 >= currPoint.getX() && currPoint.getY() + robot.getTwoTurnsDistance() <= robotLocation.getY()) {
                    // (c)
                    System.out.println("(c)");
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, Direction.WEST));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.WEST));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, robotDirection));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                } else if (robotLocation.getX() + ROBOT_SIZE / 2 <= currPoint.getX() && currPoint.getY() + robot.getTwoTurnsDistance() >= robotLocation.getY()) {
                    // (d)
                    System.out.println("(d)");
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.REVERSE, robotDirection, currPoint.getY() - robotLocation.getY() + robot.getTwoTurnsDistance()));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, Direction.EAST));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.EAST));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, robotDirection));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                } else if (robotLocation.getX() - ROBOT_SIZE / 2 >= currPoint.getX() && currPoint.getY() + robot.getTwoTurnsDistance() >= robotLocation.getY()) {
                    // (e)
                    System.out.println("(e)");
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.REVERSE, robotDirection, currPoint.getY() - robotLocation.getY() + robot.getTwoTurnsDistance()));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, Direction.WEST));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.WEST));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, robotDirection));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NONE));
                }
                break;
            case SOUTH:
                if (robotLocation.getX() - ROBOT_SIZE / 2 <= currPoint.getX() && robotLocation.getX() + ROBOT_SIZE / 2 >= currPoint.getX() && currPoint.getY() >= robotLocation.getY()) {
                    // (a)
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, robotDirection));
                } else if (robotLocation.getX() + ROBOT_SIZE / 2 <= currPoint.getX() && currPoint.getY() - robot.getTwoTurnsDistance() >= robotLocation.getY()) {
                    // (b)
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, Direction.WEST));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.WEST));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, robotDirection));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, robotDirection));
                } else if (robotLocation.getX() - ROBOT_SIZE / 2 >= currPoint.getX() && currPoint.getY() - robot.getTwoTurnsDistance() >= robotLocation.getY()) {
                    // (c)
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, Direction.EAST));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.EAST));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, robotDirection));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, robotDirection));
                } else if (robotLocation.getX() + ROBOT_SIZE / 2 <= currPoint.getX() && currPoint.getY() - robot.getTwoTurnsDistance() <= robotLocation.getY()) {
                    // (d)
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.REVERSE, robotDirection, robotLocation.getY() - currPoint.getY() + robot.getTwoTurnsDistance()));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, Direction.WEST));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.WEST));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, robotDirection));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, robotDirection));
                } else if (robotLocation.getX() - ROBOT_SIZE / 2 >= currPoint.getX() && currPoint.getY() - robot.getTwoTurnsDistance() <= robotLocation.getY()) {
                    // (e)
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.REVERSE, robotDirection, robotLocation.getY() - currPoint.getY() + robot.getTwoTurnsDistance()));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, Direction.EAST));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.EAST));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, robotDirection));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, robotDirection));
                }
                break;
            case EAST:
                if (robotLocation.getY() - ROBOT_SIZE / 2 <= currPoint.getY() && robotLocation.getY() + ROBOT_SIZE / 2 >= currPoint.getY() && currPoint.getX() <= robotLocation.getX()) {
                    // (a)
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, robotDirection));
                } else if (robotLocation.getY() + ROBOT_SIZE / 2 <= currPoint.getY() && robotLocation.getX() + robot.getTwoTurnsDistance() <= robotLocation.getX()) {
                    // (b)
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, Direction.SOUTH));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.SOUTH));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, robotDirection));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, robotDirection));
                } else if (robotLocation.getY() - ROBOT_SIZE / 2 >= currPoint.getY() && robotLocation.getX() + robot.getTwoTurnsDistance() <= robotLocation.getX()) {
                    // (c)
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, Direction.NORTH));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NORTH));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, robotDirection));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, robotDirection));
                } else if (robotLocation.getY() + ROBOT_SIZE / 2 <= currPoint.getY() && robotLocation.getX() + robot.getTwoTurnsDistance() >= robotLocation.getX()) {
                    // (d)
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.REVERSE, robotDirection, robotLocation.getX() - currPoint.getX() + robot.getTwoTurnsDistance()));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, Direction.SOUTH));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.SOUTH));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, robotDirection));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, robotDirection));
                } else if (robotLocation.getY() - ROBOT_SIZE / 2 >= currPoint.getY() && robotLocation.getX() + robot.getTwoTurnsDistance() >= robotLocation.getX()) {
                    // (e)
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.REVERSE, robotDirection, robotLocation.getX() - currPoint.getX() + robot.getTwoTurnsDistance()));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, Direction.NORTH));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NORTH));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, robotDirection));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, robotDirection));
                }
                break;
            case WEST:
                if (robotLocation.getY() - ROBOT_SIZE / 2 <= currPoint.getY() && robotLocation.getY() + ROBOT_SIZE / 2 >= currPoint.getY() && currPoint.getX() >= robotLocation.getX()) {
                    // (a)
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, robotDirection));
                } else if (robotLocation.getY() + ROBOT_SIZE / 2 <= currPoint.getY() && robotLocation.getX() - robot.getTwoTurnsDistance() >= robotLocation.getX()) {
                    // (b)
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, Direction.NORTH));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NORTH));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, robotDirection));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, robotDirection));
                } else if (robotLocation.getY() - ROBOT_SIZE / 2 >= currPoint.getY() && robotLocation.getX() - robot.getTwoTurnsDistance() >= robotLocation.getX()) {
                    // (c)
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, Direction.SOUTH));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.SOUTH));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, robotDirection));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, robotDirection));
                } else if (robotLocation.getY() + ROBOT_SIZE / 2 <= currPoint.getY() && robotLocation.getX() + robot.getTwoTurnsDistance() <= robotLocation.getX()) {
                    // (d)
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.REVERSE, robotDirection, currPoint.getX() - robotLocation.getX() + robot.getTwoTurnsDistance()));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, Direction.NORTH));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.NORTH));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, robotDirection));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, robotDirection));
                } else if (robotLocation.getY() - ROBOT_SIZE / 2 >= currPoint.getY() && robotLocation.getX() + robot.getTwoTurnsDistance() <= robotLocation.getX()) {
                    // (e)
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.REVERSE, robotDirection, currPoint.getX() - robotLocation.getX() + robot.getTwoTurnsDistance()));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_LEFT, Direction.SOUTH));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, Direction.SOUTH));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD_RIGHT, robotDirection));
                    instructions.add(new ComplexInstruction(ComplexInstruction.Instruction.FORWARD, robotDirection));
                }
                break;
            }


        } else if (Math.abs(pointDirection.ordinal() - robotDirection.ordinal()) == 1 || Math.abs(pointDirection.ordinal() - robotDirection.ordinal()) == 3) {
            // 2) Robot and image facing has a difference of pi/2 or -pi/2
            logger.log(Level.INFO, "pi/2 & -pi/2");

        } else if (pointDirection.ordinal() == robotDirection.ordinal()) {
            // 3) Robot and image facing the same direction
            logger.log(Level.INFO, "SAME");

        }

        // return instructions;
    }
}