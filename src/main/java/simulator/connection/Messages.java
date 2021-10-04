package simulator.connection;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import javax.swing.JButton;

import simulator.algorithm.HamiltonianPath;
import simulator.entity.Grid;
import simulator.entity.Robot;

public class Messages {
    private Connection conn;
    private Grid grid;
    private Robot robot;
    private HamiltonianPath hPath;
    private ArrayList<String> messages;

    private JButton startButton;
    private JButton imageRecCompleteButton;

    private ArrayList<String> obstacles;

    public Messages(Connection connection, Grid grid, Robot robot, HamiltonianPath hPath, JButton startButton,
            JButton imageRecCompleteButton) {

        this.conn = connection;
        this.grid = grid;
        this.robot = robot;
        this.hPath = hPath;
        this.startButton = startButton;
        this.imageRecCompleteButton = imageRecCompleteButton;

        Thread t1 = new Thread(() -> {
            // this.messages.add(this.conn.recvMsg());
            while (true) {
                processMessage(this.conn.recvMsg());
            }
        });

        t1.start();
    }

    private void processMessage(String msg) {
        char[] charArray = msg.toCharArray();

        if (msg.substring(0, 3).compareTo("OBS") == 0) {
            // Fill in obstacles
            String arr = msg.substring(5, msg.length() - 1);
            List<Integer> lst = Arrays.asList((arr.split(","))).parallelStream().map(s -> Integer.parseInt(s)).toList();
            this.grid.addObstacle(lst.get(0), lst.get(1), lst.get(2), lst.get(3));
        } else if (msg.length() >= 9 && msg.substring(0, 9).compareTo("DRAW_PATH") == 0) {
            // Draw path on simulator
            this.hPath.getShortestPath(grid, robot, true);
            this.hPath.generatePlannedPath(grid, robot);
            // this.hPath.printPlannedPath();
            this.robot.generateMovements(hPath.getPlannedPath(), hPath.getOrderedObstacleIds());
            // this.robot.printGeneratedMovements();
            this.grid.repaint();
        } else if (msg.length() >= 7 && msg.substring(0, 7).compareTo("BANANAS") == 0) {
            // START TASK 1
            ArrayList<String> commands = robot.getCommandsToSend();
            System.out.println("BANANAS");
            String concatCommand = "";
            for (String s : commands) {
                if (s.charAt(0) == 'R' || s.charAt(0) == 'S') {
                    // stop sending commannds for subsequent obstacle
                    break;
                }
                concatCommand += s + '|';
            }
            System.out.println(concatCommand);
            if (conn != null && conn.isConnected()) {
                conn.sendMsg(concatCommand, "command");
            }

            // start the simulation
            this.startButton.doClick();
        } else if (msg.length() >= 6 && msg.substring(0, 6).compareTo("something") == 0) {
            // process image rec results
        }
    }

    public boolean messagesWaiting() {
        return !messages.isEmpty();
    }

}
