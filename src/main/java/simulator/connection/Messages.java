package simulator.connection;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import simulator.algorithm.HamiltonianPath;
import simulator.entity.Grid;
import simulator.entity.Robot;

public class Messages {
    private Connection conn;
    private Grid grid;
    private Robot robot;
    private HamiltonianPath hpath;
    private ArrayList<String> messages;

    private ArrayList<String> obstacles;

    public Messages(Connection connection, Grid grid, Robot robot, HamiltonianPath hpath) {

        this.conn = connection;
        this.grid = grid;
        this.robot = robot;
        this.hpath = hpath;

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
            String arr = msg.substring(5, msg.length() - 1);
            List<Integer> lst = Arrays.asList((arr.split(","))).parallelStream().map(s -> Integer.parseInt(s)).toList();
            this.grid.addObstacle(lst.get(1), lst.get(2), lst.get(3));
        }
    }

    public boolean messagesWaiting() {
        return !messages.isEmpty();
    }

}
