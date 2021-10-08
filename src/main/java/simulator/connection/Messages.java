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

    private JButton resetButton;
    private JButton startButton;
    private JButton imageRecCompleteButton;

    private boolean isReset = false;

    private ArrayList<String> obstacles;
    private ArrayList<String> commands;
    private int commandsIdx = 0;
    private int howManyReverse = 0;
    private String lastCommand;

    public Messages(Connection connection, Grid grid, Robot robot, HamiltonianPath hPath, JButton resetButton,
            JButton startButton, JButton imageRecCompleteButton) {

        this.conn = connection;
        this.grid = grid;
        this.robot = robot;
        this.hPath = hPath;
        this.resetButton = resetButton;
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
        System.out.println("Receive message: " + msg);

        if (msg.length() >= 5 && msg.substring(0, 5).compareTo("RESET") == 0) {
            this.isReset = false;
            this.resetButton.doClick();
            this.isReset = true;
        } else if (msg.substring(0, 3).compareTo("OBS") == 0) {
            // Fill in obstacles
            System.out.println("OBS");
            System.out.println(msg);
            while (!this.isReset) {
                // wait for reset to be done
            }
            String arr = msg.substring(5, msg.length() - 1);
            List<Integer> lst = Arrays.asList((arr.split(","))).parallelStream().map(s -> Integer.parseInt(s)).toList();
            this.grid.addObstacle(lst.get(0), lst.get(1), lst.get(2), lst.get(3));
        } else if (msg.length() >= 9 && msg.substring(0, 9).compareTo("DRAW_PATH") == 0) {
            // Draw path on simulator
            try {
                this.drawPath();
            } catch (Exception e) {
                System.out.println("Cannot draw path");
            }
        } else if (msg.length() >= 7 && msg.substring(0, 7).compareTo("BANANAS") == 0) {
            // START TASK 1
            System.out.println("BANANAS");
            this.sendForTask1();
            this.goNextStage();
            // start the simulation
            this.startButton.doClick();
        } else if (msg.length() >= 4 && msg.substring(0, 4).compareTo("None") == 0) {
            System.out.println("Image Rec (None)");
            System.out.println(msg);

            if (this.howManyReverse == 4) {
                String cmd = "w" + Integer.toString(5 * howManyReverse);
                this.conn.sendMsg(cmd + '|', "command");
                this.howManyReverse = 0;
                this.goNextStage();
                this.imageRecCompleteButton.doClick();
                if (this.lastCommand.charAt(0) == 'S') {
                    System.out.println("Sending DONE");
                    this.conn.sendMsg("DONE", "rpi");
                }
            } else {
                String cmd = "s" + Integer.toString(5);
                this.conn.sendMsg(cmd + '|', "command");
                this.howManyReverse += 1;
                this.conn.sendMsg(lastCommand + '|', "command");
            }

        } else if (msg.length() >= 3 && msg.substring(0, 1).compareTo("D") == 0) {
            // process image rec results
            System.out.println("Image Rec (D)");
            System.out.println(msg);
            System.out.println(msg.substring(1, msg.length()));
            String arr[] = msg.substring(1, msg.length()).split(",");
            System.out.println(arr[0]);
            System.out.println(arr[1]);
            System.out.println(arr[2]);
            int idx = Integer.parseInt(arr[0]);
            int distance = Integer.parseInt(arr[1]);
            System.out.println(distance);
            // this.grid.setObstacleVisited(idx);

            // this.drawPath();
            // this.sendForTask1();
            // move forwards the amount it moved back
            if (this.howManyReverse > 0) {
                String cmd = "w" + Integer.toString(5 * howManyReverse);
                this.conn.sendMsg(cmd + '|', "command");
                this.howManyReverse = 0;
            }

            if (this.lastCommand.charAt(0) == 'S') {
                System.out.println("Sending DONE");
                this.conn.sendMsg("DONE", "rpi");
            }
            this.goNextStage();
            // image rec complete -> continue movements
            this.imageRecCompleteButton.doClick();
        }
    }

    private void drawPath() {
        this.hPath.getShortestPath(grid, robot, true);
        this.hPath.generatePlannedPath(grid, robot);
        // this.hPath.printPlannedPath();
        this.robot.generateMovements(hPath.getPlannedPath(), hPath.getOrderedObstacleIds());
        // this.robot.printGeneratedMovements();
        this.grid.repaint();
    }

    private void sendForTask1() {
        this.commandsIdx = 0;
        this.commands = robot.getCommandsToSend();

    }

    private void goNextStage() {
        String concatCommand = "";
        int counter = 0;
        for (String s : this.commands) {
            if (counter == this.commandsIdx) {
                concatCommand += s + '|';

            }
            if (s.charAt(0) == 'R' || s.charAt(0) == 'S') {
                // stop sending commannds for subsequent obstacle
                this.lastCommand = s;
                counter += 1;
            }
            if (counter > this.commandsIdx) {
                this.commandsIdx += 1;
                break;
            }
        }
        System.out.println(concatCommand);
        if (this.conn != null && this.conn.isConnected()) {
            this.conn.sendMsg(concatCommand, "command");
        }
    }

    public boolean messagesWaiting() {
        return !messages.isEmpty();
    }

}
