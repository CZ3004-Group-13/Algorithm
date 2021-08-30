package simulator;

import simulator.algorithm.HamiltonianPath;
import simulator.entity.Grid;
import simulator.entity.Robot;

import javax.swing.*;
import java.awt.*;

class Simulator {

    private static JFrame jFrame;
    private static Grid grid;
    private static Robot robot;
    private static HamiltonianPath hPath;
    private static Dimension environmentActualSize = new Dimension(200, 200);
    private static int environmentScalingFactor = 3;

    private static Dimension robotActualSize = new Dimension(30, 30);
    private static Point robotActualStartingPoint = new Point(20, 180);

    private static Thread gameLoop;
    private static boolean isRunning = false;

    public static void createAndShowGUI() {
        jFrame = new JFrame("Test Window");
        jFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        Dimension environmentModelSize = new Dimension(environmentActualSize.width * environmentScalingFactor,
                environmentActualSize.height * environmentScalingFactor);

        Dimension robotModelSize = new Dimension(robotActualSize.width * environmentScalingFactor,
                robotActualSize.height * environmentScalingFactor);

        Point robotModelStartingPoint = new Point(robotActualStartingPoint.x * environmentScalingFactor,
                robotActualStartingPoint.y * environmentScalingFactor);

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
            System.out.println("Start button clicked");
            Point origin = new Point(0, 0);
            hPath.getShortestPath(origin, grid.getObstacleCenters());
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
            System.out.println("Start");
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
        // need to use this utility to call the initial method that draws GUI
        SwingUtilities.invokeLater(Simulator::createAndShowGUI);
    }

    public static void setupGameLoop() {
        gameLoop = new Thread(() -> {
            System.out.println("Thread");
            while (isRunning) {
                System.out.println("Loop");
                robot.moveForward();
                robot.repaint();
                try {
                    Thread.sleep(15);
                } catch (InterruptedException ex) {
                }
            }
        });
    }

}