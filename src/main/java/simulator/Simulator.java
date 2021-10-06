package simulator;

import simulator.algorithm.HamiltonianPath;
import simulator.connection.Connection;
import simulator.connection.Messages;
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
    private Messages messages;
    public static String host = "192.168.13.13";
    public static int port = 3053;
    private final Logger logger = Logger.getLogger(Simulator.class.getName());
    private final Dimension environmentActualSize = new Dimension(200, 200);
    private final Dimension robotActualSize = new Dimension(20, 20);
    private final double robotActualDistanceBetweenFrontBackWheels = 14.5;
    private final Point robotActualStartingPoint = new Point(10, 190);
    private JFrame jFrame;
    private Grid grid;
    private Robot robot;
    private HamiltonianPath hPath;
    private Thread movementsLoop;
    private boolean isRunning = false;
    private boolean isPaused = false;
    private boolean waitingImageRec = false;
    private JLabel timerLabel = new JLabel();

    private JTextField robotX;
    private JTextField robotY;
    private JTextField commandTextField;

    public static void main(String[] args) {

        boolean rpiConnect = false; // set to true to test connection
        connection = Connection.getConnection();

        if (rpiConnect) {
            connection.openConnection(host, port);
            // connection.sendMsg("C", "type"); //C take pic
        }

        Simulator sim = new Simulator();

        // need to use this utility to call the initial method that draws GUI
        SwingUtilities.invokeLater(() -> sim.createAndShowGUI(sim));
    }

    public void createAndShowGUI(Simulator sim) {
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

        // textfields

        robotX = new JTextField();
        robotX.setMaximumSize(new Dimension(100, robotX.getPreferredSize().height));

        robotY = new JTextField();
        robotY.setMaximumSize(new Dimension(100, robotY.getPreferredSize().height));

        commandTextField = new JTextField();
        commandTextField.setMaximumSize(new Dimension(100, commandTextField.getPreferredSize().height));

        // add buttons
        JButton disconnectButton = new JButton("Disconnect from RPI");
        disconnectButton.addActionListener(e -> {
            System.out.println("Disconnecting");
            System.out.println(connection.isConnected());
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
            isRunning = false;
            isPaused = false;
            waitingImageRec = false;
            hPath.reset();
            grid.reset();
            timerLabel.setText("Time: 0.00s");
            // this is so that robot gets painted first (so that the affine transform gets
            // processed first)
            new java.util.Timer().schedule(new java.util.TimerTask() {
                @Override
                public void run() {
                    // TODO Auto-generated method stub
                    updateRobotCoordinates();
                }

            }, 100);
        });

        JButton drawButton = new JButton("Draw Path");
        drawButton.addActionListener(e -> {
            logger.log(Level.FINE, "Start Button Clicked");

            hPath.getShortestPath(this.grid, this.robot, true);
            hPath.generatePlannedPath(grid, robot);
            hPath.printPlannedPath();
            robot.generateMovements(hPath.getPlannedPath(), hPath.getOrderedObstacleIds());
            robot.printGeneratedMovements();
            grid.repaint();
        });

        JButton startMovementsButton = new JButton("Start Movements");
        startMovementsButton.addActionListener(e -> {
            startMovements();
            isRunning = true;
            movementsLoop.start();
            logger.log(Level.FINE, "Start");
        });

        JButton pauseButton = new JButton("Pause");
        pauseButton.addActionListener(e -> {
            isPaused = true;
        });
        JButton continueButton = new JButton("Continue");
        continueButton.addActionListener(e -> {
            isPaused = false;
        });

        JButton sendMovements = new JButton("Send Movements");
        sendMovements.addActionListener(e -> {
            ArrayList<String> commands = robot.getCommandsToSend();
            System.out.println("----------Sending movements...");
            String concatCommand = "";
            for (String s : commands) {
                concatCommand += s + '|';
                if (s.charAt(0) == 'R' || s.charAt(0) == 'S') {
                    // stop sending commannds for subsequent obstacle
                    break;
                }
            }
            System.out.println(concatCommand);
            if (connection != null && connection.isConnected()) {
                connection.sendMsg(concatCommand, "command");
            }
        });

        JButton setRobotLocButton = new JButton("Set Robot Location");
        setRobotLocButton.addActionListener(e -> {
            moveRobotTo(Double.parseDouble(robotX.getText()), Double.parseDouble(robotY.getText()));
        });

        JButton task1Button = new JButton("Task 1 BEGIN");
        task1Button.addActionListener(e -> {
            startMovements(true);
            isRunning = true;
            movementsLoop.start();
        });

        JButton receiveButton = new JButton("Receive message");
        receiveButton.addActionListener(e -> {
        });

        JButton sendOneCommand = new JButton("Send one command");
        sendOneCommand.addActionListener(e -> {
            sendOne();
        });

        JButton imageRecButton = new JButton("*image rec done*");
        imageRecButton.addActionListener(e -> {
            waitingImageRec = false;
        });

        JButton connectButton = new JButton("Connect to RPI");
        connectButton.addActionListener(e -> {
            System.out.println("Connecting");
            if (!connection.isConnected()) {
                connection = Connection.getConnection();
                connection.openConnection(host, port);
            }
            if (connection.isConnected()) {
                System.out.println("Connection opened");
                this.messages = new Messages(connection, this.grid, this.robot, this.hPath, resetButton, task1Button,
                        imageRecButton);
                // connection.sendMsg("R", "type");
            }
        });

        // rightPanel.add(forwardButton);
        // rightPanel.add(backwardButton);
        // rightPanel.add(leftButton);
        // rightPanel.add(rightButton);
        robotX.setAlignmentX(Component.CENTER_ALIGNMENT);
        robotY.setAlignmentX(Component.CENTER_ALIGNMENT);
        setRobotLocButton.setAlignmentX(Component.CENTER_ALIGNMENT);
        connectButton.setAlignmentX(Component.CENTER_ALIGNMENT);
        disconnectButton.setAlignmentX(Component.CENTER_ALIGNMENT);
        receiveButton.setAlignmentX(Component.CENTER_ALIGNMENT);
        resetButton.setAlignmentX(Component.CENTER_ALIGNMENT);
        drawButton.setAlignmentX(Component.CENTER_ALIGNMENT);
        startMovementsButton.setAlignmentX(Component.CENTER_ALIGNMENT);
        pauseButton.setAlignmentX(Component.CENTER_ALIGNMENT);
        continueButton.setAlignmentX(Component.CENTER_ALIGNMENT);
        sendMovements.setAlignmentX(Component.CENTER_ALIGNMENT);
        commandTextField.setAlignmentX(Component.CENTER_ALIGNMENT);
        sendOneCommand.setAlignmentX(Component.CENTER_ALIGNMENT);
        timerLabel.setAlignmentX(Component.CENTER_ALIGNMENT);
        task1Button.setAlignmentX(Component.CENTER_ALIGNMENT);
        imageRecButton.setAlignmentX(Component.CENTER_ALIGNMENT);

        rightPanel.add(robotX);
        rightPanel.add(robotY);
        rightPanel.add(setRobotLocButton);
        rightPanel.add(Box.createRigidArea(new Dimension(10, 20)));
        rightPanel.add(connectButton);
        rightPanel.add(disconnectButton);
        rightPanel.add(receiveButton);
        rightPanel.add(Box.createRigidArea(new Dimension(10, 20)));
        rightPanel.add(drawButton);
        rightPanel.add(resetButton);
        rightPanel.add(Box.createRigidArea(new Dimension(10, 20)));
        rightPanel.add(startMovementsButton);
        rightPanel.add(pauseButton);
        rightPanel.add(continueButton);
        rightPanel.add(sendMovements);
        rightPanel.add(commandTextField);
        rightPanel.add(sendOneCommand);

        rightPanel.add(timerLabel);
        rightPanel.add(Box.createRigidArea(new Dimension(10, 20)));
        rightPanel.add(task1Button);
        rightPanel.add(imageRecButton);

        timerLabel.setFont(new Font("Serif", Font.PLAIN, 20));
        timerLabel.setText("Time: 0.00s");

        jFrame.pack();
        jFrame.setLocationRelativeTo(null);
        jFrame.setVisible(true); // now frame will be visible, by default not visible

        // this is so that robot gets painted first (so that the affine transform gets
        // processed first)
        new java.util.Timer().schedule(new java.util.TimerTask() {
            @Override
            public void run() {
                // TODO Auto-generated method stub
                updateRobotCoordinates();
            }
        }, 100);
    }

    public void startMovements() {
        startMovements(false);
    }

    public void startMovements(boolean pauseUponObstacle) {
        movementsLoop = new Thread(() -> {
            logger.log(Level.FINEST, "Thread");

            long lastLoopTime = System.nanoTime();
            final int TARGET_FPS = 120;
            final long OPTIMAL_TIME = 1000000000 / TARGET_FPS;
            long lastFpsTime = 0;

            long duration = 0;

            while (isRunning) {
                logger.log(Level.FINEST, "Start Movements");

                if (isPaused || waitingImageRec) {
                    lastLoopTime = System.nanoTime();
                    continue;
                }
                long now = System.nanoTime();
                long updateLength = now - lastLoopTime;
                lastLoopTime = now;
                duration += updateLength;
                double delta = updateLength / ((double) OPTIMAL_TIME);
                // delta is the number of frames that has passed

                lastFpsTime += updateLength;
                if (lastFpsTime >= 1000000000) {
                    lastFpsTime = 0;
                }

                if (robot.letsGo(delta)) {
                    if (pauseUponObstacle) {
                        waitingImageRec = true;
                    }
                }
                if (robot.finishedMovements()) {
                    break;
                }
                updateRobotCoordinates();

                robot.repaint();
                DecimalFormat df = new DecimalFormat("#.##");
                timerLabel.setText("Time: " + df.format((double) (duration) / 1000000000) + "s");

                try {
                    Thread.sleep((lastLoopTime - System.nanoTime() + OPTIMAL_TIME) / 1000000);
                } catch (Exception e) {
                    // e.printStackTrace();
                }

            }
        });
    }

    public void updateRobotCoordinates() {
        double x = robot.getCurrentLocation().getX();
        double y = robot.getCurrentLocation().getY();
        x = x / ENVIRONMENT_SCALING_FACTOR;
        y = y / ENVIRONMENT_SCALING_FACTOR;
        y = -y + 200;
        robotX.setText(String.format("%.2f", x));
        robotY.setText(String.format("%.2f", y));
    }

    public void moveRobotTo(double x, double y) {
        x = x * ENVIRONMENT_SCALING_FACTOR;
        y = -y + 200;
        y = y * ENVIRONMENT_SCALING_FACTOR;
        robot.moveTo(x, y);
    }

    public void sendOne() {
        String cmd = commandTextField.getText();

        connection.sendMsg(cmd + '|', "command");
        // robot.addCommand(cmd);
    }
}
