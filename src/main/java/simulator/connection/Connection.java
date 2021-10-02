package simulator.connection;

import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.SocketAddress;
import java.net.UnknownHostException;
import java.util.logging.Level;
import java.util.logging.Logger;

public class Connection {

    private static Connection connection = null;
    private static Socket socket = null;

    private final Logger logger = Logger.getLogger(Connection.class.getName());
    public BufferedReader reader;
    private BufferedWriter writer;

    private Connection() {
    }

    public static Connection getConnection() {
        if (connection == null) {
            connection = new Connection();
        }
        return connection;
    }

    public boolean openConnection(String host, int port) {
        logger.log(Level.FINE, "Opening connection...");

        try {
            // String HOST = "192.168.13.13";
            // int PORT = 3053;
            socket = new Socket();
            SocketAddress socketAddress = new InetSocketAddress(host, port);
            socket.connect(socketAddress, 2000); // 10s connection timeout

            writer = new BufferedWriter(new OutputStreamWriter(new BufferedOutputStream(socket.getOutputStream())));
            InputStreamReader instream = new InputStreamReader(socket.getInputStream());
            reader = new BufferedReader(instream);

            logger.log(Level.FINE, "openConnection() --> " + "Connection established successfully!");
            return true;
        } catch (UnknownHostException e) {
            logger.log(Level.FINE, "openConnection() --> UnknownHostException");
        } catch (IOException e) {
            logger.log(Level.FINE, "openConnection() --> IOException");
        } catch (Exception e) {
            logger.log(Level.FINE, "openConnection() --> Exception");
            logger.log(Level.FINE, e.toString());
        }

        logger.log(Level.FINE, "Failed to establish connection!");
        return false;
    }

    public void closeConnection() {
        logger.log(Level.FINE, "Closing connection...");

        try {
            reader.close();
            if (socket != null) {
                socket.close();
                socket = null;
            }
            logger.log(Level.FINE, "Connection closed!");
        } catch (IOException e) {
            logger.log(Level.FINE, "closeConnection() --> IOException");
        } catch (NullPointerException e) {
            logger.log(Level.FINE, "closeConnection() --> NullPointerException");
        } catch (Exception e) {
            logger.log(Level.FINE, "closeConnection() --> Exception");
            logger.log(Level.FINE, e.toString());
        }
    }

    public void sendMsg(String msg, String msgType) {
        // logger.log(Level.FINE, "Sending a message...");

        try {
            // TODO: Define string tokens for sending messages

            // logger.log(Level.FINE, "Sending out message: " + outputMsg);
            writer.write(msg);
            writer.flush();
        } catch (IOException e) {
            logger.log(Level.FINE, "sendMsg() --> IOException");
        } catch (Exception e) {
            logger.log(Level.FINE, "sendMsg() --> Exception");
            logger.log(Level.FINE, e.toString());
        }
    }

    public String recvMsg() {
        try {
            StringBuilder sb = new StringBuilder();
            String input = reader.readLine();
            if (input != null && input.length() > 0) {
                sb.append(input);
                // logger.log(Level.FINE, "message received: " + sb.toString());
                return sb.toString();
            }
        } catch (IOException e) {
            logger.log(Level.FINE, "recvMsg() --> IOException");
        } catch (Exception e) {
            logger.log(Level.FINE, "recvMsg() --> Exception");
            logger.log(Level.FINE, e.toString());
        }

        return null;
    }

    public boolean isConnected() {
        return socket != null && socket.isConnected();
    }
}
