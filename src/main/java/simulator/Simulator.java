package simulator;

import simulator.algorithm.HamiltonianPath;
import simulator.entity.Grid;
import simulator.entity.Robot;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

class Simulator {

    private static JFrame jFrame;
    private static Grid grid;
    private static Robot robotLayer;
    private static HamiltonianPath hPath;
    private static Dimension enviromentActualSize = new Dimension(200, 200);
    private static int enviromentScalingFactor = 3;

    private static Dimension robotActualSize = new Dimension(30, 30);
    private static Point robotActualStartingPoint = new Point(20, 180);

    public static void createAndShowGUI() {
        jFrame = new JFrame("Test Window");
        jFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        Dimension environmentModelSize = new Dimension(enviromentActualSize.width * enviromentScalingFactor,
                enviromentActualSize.height * enviromentScalingFactor);

        Dimension robotModelSize = new Dimension(robotActualSize.width * enviromentScalingFactor,
                robotActualSize.height * enviromentScalingFactor);

        Point robotModelStartingPoint = new Point(robotActualStartingPoint.x * enviromentScalingFactor,
                robotActualStartingPoint.y * enviromentScalingFactor);

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

        robotLayer = new Robot(robotModelSize, robotModelStartingPoint);
        robotLayer.setSize(environmentModelSize);
        layeredPane.add(robotLayer, 1, 0);

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
        drawButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                System.out.println("Start button clicked");
                Point origin = new Point(0, 0);
                hPath.getShortestPath(origin, grid.getObstacleCenters());
            }
        });

        JButton downButton = new JButton("Down");
        downButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
            }
        });

        rightPanel.add(drawButton);
        rightPanel.add(downButton);

        jFrame.pack();
        jFrame.setVisible(true); // now frame will be visible, by default not visible
    }

    public static void main(String args[]) {
        // need to use this ultility to call the initial method that draws GUI
        SwingUtilities.invokeLater(new Runnable() {
            public void run() {
                createAndShowGUI();
            }
        });

    }

}
