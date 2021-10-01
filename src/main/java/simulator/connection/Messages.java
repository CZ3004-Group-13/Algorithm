package simulator.connection;

import java.util.ArrayList;
import java.util.Arrays;

public class Messages {
    private Connection conn;
    private ArrayList<String> messages;

    private ArrayList<String> obstacles;

    public Messages(Connection connection) {

        this.conn = connection;
        Thread t1 = new Thread(() -> {
            this.messages.add(this.conn.recvMsg());
        });

        t1.start();
    }

    private void processMessage(String msg) {
        char[] charArray = msg.toCharArray();

        if (msg.substring(0, 3).compareTo("OBS") == 0) {
            String arr = msg.substring(5, msg.length() - 1);
            ArrayList<Integer> lst = (ArrayList<Integer>) Arrays.asList((arr.split(","))).parallelStream()
                    .map(s -> Integer.parseInt(s)).toList();
        }
    }

    public void getObstacles() {
        
    }

    public boolean messagesWaiting() {
        return !messages.isEmpty();
    }

}
