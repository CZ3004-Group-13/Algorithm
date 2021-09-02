package simulator;

import simulator.algorithm.HamiltonianPath;
import simulator.connection.Connection;
import simulator.entity.Grid;
import simulator.entity.Robot;

import javax.swing.*;
import java.awt.*;
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

    private Point[] shortestPath = new Point[0];
    private Thread gameLoop;
    private boolean isRunning = false;

    private static final int DISTANCE_MARGIN_OF_ERROR = 50;
    private static final double DIRECTION_MARGIN_OF_ERROR = 5;

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
            shortestPath = hPath.getShortestPath(robot.getCurrentLocation(), grid.getObstacleCenters());
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
            int index = 0;
            while (isRunning/* && index < shortestPath.length - 1*/) {
                logger.log(Level.FINEST, "Loop");
                robot.moveForward();
                /*if (Math.abs(shortestPath[index].getX() - robot.getBodyAffineTransform().getTranslateX()) > DISTANCE_MARGIN_OF_ERROR || Math.abs(shortestPath[index].getY() - robot.getBodyAffineTransform().getTranslateY()) > DISTANCE_MARGIN_OF_ERROR) {
                    moveSmartly(shortestPath[index]);
                } else {
                    index++;
                }*/

                robot.repaint();
                try {
                    Thread.sleep(15);
                } catch (InterruptedException ex) {
                    ex.printStackTrace();
                }
            }
        });
    }

    public void moveSmartly(Point point) {
        if (point == null) {
            return;
        }

        if (point.getX() - robot.getBodyAffineTransform().getTranslateX() > DISTANCE_MARGIN_OF_ERROR) {
            if (point.getY() - robot.getBodyAffineTransform().getTranslateY() > DISTANCE_MARGIN_OF_ERROR) {
                // move south-east
                logger.log(Level.FINE, "South East");
                if (robot.getDirectionInRadians() > Math.toRadians(-45) && robot.getDirectionInRadians() < 135) {
                    for (int i = 0; i < 4; i++) {
                        robot.turnRight();
                    }
                } else {
                    for (int i = 0; i < 4; i++) {
                        robot.turnLeft();
                    }
                }
                robot.moveForward();

            } else if (robot.getBodyAffineTransform().getTranslateY() - point.getY() > DISTANCE_MARGIN_OF_ERROR) {
                // move north-east
                logger.log(Level.FINE, "North East");
                if (robot.getDirectionInRadians() > Math.toRadians(-135) && robot.getDirectionInRadians() < 45) {
                    for (int i = 0; i < 4; i++) {
                        robot.turnRight();
                    }
                } else {
                    for (int i = 0; i < 4; i++) {
                        robot.turnLeft();
                    }
                }
                robot.moveForward();
            } else {
                // move east
                logger.log(Level.FINE, "East");
                if (robot.getDirectionInRadians() > Math.toRadians(-90) && robot.getDirectionInRadians() < 90) {
                    for (int i = 0; i < 4; i++) {
                        robot.turnRight();
                    }
                } else {
                    for (int i = 0; i < 4; i++) {
                        robot.turnLeft();
                    }
                }
                robot.moveForward();
            }
        } else if (robot.getBodyAffineTransform().getTranslateX() - point.getX() > DISTANCE_MARGIN_OF_ERROR) {
            if (point.getY() - robot.getBodyAffineTransform().getTranslateY() > DISTANCE_MARGIN_OF_ERROR) {
                // move south-west
                logger.log(Level.FINE, "South West");
                if (robot.getDirectionInRadians() > Math.toRadians(-135) && robot.getDirectionInRadians() < 45) {
                    for (int i = 0; i < 4; i++) {
                        robot.turnLeft();
                    }
                } else {
                    for (int i = 0; i < 4; i++) {
                        robot.turnRight();
                    }
                }
                robot.moveForward();
            } else if (robot.getBodyAffineTransform().getTranslateY() - point.getY() > DISTANCE_MARGIN_OF_ERROR) {
                // move north-west
                logger.log(Level.FINE, "North West");
                if (robot.getDirectionInRadians() > Math.toRadians(-45) && robot.getDirectionInRadians() < 135) {
                    for (int i = 0; i < 4; i++) {
                        robot.turnLeft();
                    }
                } else {
                    for (int i = 0; i < 4; i++) {
                        robot.turnRight();
                    }
                }
                robot.moveForward();
            } else {
                // move west
                logger.log(Level.FINE, "West");
                if (robot.getDirectionInRadians() > Math.toRadians(-90) && robot.getDirectionInRadians() < 90) {
                    for (int i = 0; i < 4; i++) {
                        robot.turnLeft();
                    }
                } else {
                    for (int i = 0; i < 4; i++) {
                        robot.turnRight();
                    }
                }
                robot.moveForward();
            }
        } else {
            if (point.getY() - robot.getBodyAffineTransform().getTranslateY() > DISTANCE_MARGIN_OF_ERROR) {
                // move south
                logger.log(Level.FINE, "South");
                if (robot.getDirectionInRadians() > Math.toRadians(-180) && robot.getDirectionInRadians() < 180) {
                    for (int i = 0; i < 4; i++) {
                        robot.turnLeft();
                    }
                } else {
                    for (int i = 0; i < 4; i++) {
                        robot.turnRight();
                    }
                }
                robot.moveForward();
            } else if (robot.getBodyAffineTransform().getTranslateY() - point.getY() > DISTANCE_MARGIN_OF_ERROR) {
                // move north
                logger.log(Level.FINE, "North");
                if (robot.getDirectionInRadians() > Math.toRadians(-180) && robot.getDirectionInRadians() < 180) {
                    for (int i = 0; i < 4; i++) {
                        robot.turnRight();
                    }
                } else {
                    for (int i = 0; i < 4; i++) {
                        robot.turnLeft();
                    }
                }
                robot.moveForward();
            }
        }
    }
}