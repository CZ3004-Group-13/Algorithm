package simulator;

import simulator.algorithm.HamiltonianPath;
import simulator.connection.Connection;
import simulator.entity.Grid;
import simulator.entity.Robot;

import javax.swing.*;
import java.awt.*;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;

class Simulator {

    private static final int ENVIRONMENT_SCALING_FACTOR = 3;
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
    private Thread movementsLoop;
    private boolean isRunning2 = false;
    private long startTime;
    private JLabel timerLabel = new JLabel();

    public static void main(String[] args) {

        boolean rpiConnect = false; // set to true to test connection
        connection = Connection.getConnection();

        if (rpiConnect) {
            connection.openConnection(host, port);
            // connection.sendMsg("C", "type"); //C take pic
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

        robot = new Robot(robotModelSize, robotModelStartingPoint, robotModelDistanceBetweenFrontBackWheels,
                ENVIRONMENT_SCALING_FACTOR);
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

        JButton connectButton = new JButton("Connect to RPI");
        connectButton.addActionListener(e -> {
            System.out.println("Connecting");
            if (!connection.isConnected()) {
                connection = Connection.getConnection();
                connection.openConnection(host, port);
            }
            if (connection.isConnected()) {
                System.out.println("Connection opened");
                // connection.sendMsg("R", "type");
            }
        });

        JButton disconnectButton = new JButton("Disconnect from RPI");
        disconnectButton.addActionListener(e -> {
            System.out.println("Disconnecting");
            connection.closeConnection();
        });

        JButton forwardButton = new JButton("Forward");
        forwardButton.addActionListener(e -> robot.moveForward());

        JButton backwardButton = new JButton("Backward");
        backwardButton.addActionListener(e -> robot.moveBackward());

        JButton leftButton = new JButton("Turn left");
        leftButton.addActionListener(e -> robot.turnLeft());

        JButton rightButton = new JButton("Turn right");
        rightButton.addActionListener(e -> robot.turnRight());

        JButton resetButton = new JButton("Reset");
        resetButton.addActionListener(e -> {
            robot.reset();
            isRunning2 = false;
            hPath.reset();
        });

        JButton drawButton = new JButton("Draw Path");
        drawButton.addActionListener(e -> {
            logger.log(Level.FINE, "Start Button Clicked");

            hPath.getShortestPath(robot.getCurrentLocation(), grid.getObstacleFronts(), true, grid, robot);
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

        JButton pauseButton = new JButton("Pause");
        pauseButton.addActionListener(e -> {
            isRunning2 = false;
        });
        JButton continueButton = new JButton("Continue");
        continueButton.addActionListener(e -> {
            isRunning2 = true;
        });

        JButton sendMovements = new JButton("Send Movements");
        sendMovements.addActionListener(e -> {
            ArrayList<String> commands = robot.getCommandsToSend();
            System.out.println("----------Sending movements...");
            String concatCommand = "";
            for (String s : commands) {
                if (s.compareTo("Reached") == 0) {
                    // stop sending commannds for subsequent obstacle
                    break;
                }
                concatCommand += s + '|';
            }
            System.out.println(concatCommand);
            if (connection != null && connection.isConnected()) {
                connection.sendMsg(concatCommand, "command");
            }
        });

        // rightPanel.add(forwardButton);
        // rightPanel.add(backwardButton);
        // rightPanel.add(leftButton);
        // rightPanel.add(rightButton);

        rightPanel.add(timerLabel);
        rightPanel.add(connectButton);
        rightPanel.add(disconnectButton);
        rightPanel.add(resetButton);
        rightPanel.add(drawButton);
        rightPanel.add(startMovementsButton);
        rightPanel.add(pauseButton);
        rightPanel.add(continueButton);
        rightPanel.add(sendMovements);

        jFrame.pack();
        jFrame.setLocationRelativeTo(null);
        jFrame.setVisible(true); // now frame will be visible, by default not visible
    }

    public void startMovements() {
        movementsLoop = new Thread(() -> {
            logger.log(Level.FINEST, "Thread");

            long lastLoopTime = System.nanoTime();
            final int TARGET_FPS = 120;
            final long OPTIMAL_TIME = 1000000000 / TARGET_FPS;
            long lastFpsTime = 0;

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
                    // e.printStackTrace();
                }

            }
        });
    }
}
