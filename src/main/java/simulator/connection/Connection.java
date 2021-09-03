package simulator.connection;

import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.net.Socket;
import java.net.UnknownHostException;

public class Connection {

	private static Connection connection = null;
	private static Socket socket = null;

	private BufferedWriter writer;
	public BufferedReader reader;

	private Connection() { }

	public static Connection getConnection() {
		if (connection == null) {
			connection = new Connection();
		}
		return connection;
	}

	public void openConnection(String host, int port) {
		System.out.println("Opening connection...");

		try {
			String HOST = host;
			int PORT = port;
			// String HOST = "192.168.13.13";
			// int PORT = 3053;
			socket = new Socket(HOST, PORT);

			writer = new BufferedWriter(new OutputStreamWriter(new BufferedOutputStream(socket.getOutputStream())));
			InputStreamReader instream = new InputStreamReader(socket.getInputStream());
			reader = new BufferedReader(instream);
			
			System.out.println("openConnection() --> " + "Connection established successfully!");

			return;
		} catch (UnknownHostException e) {
			System.out.println("openConnection() --> UnknownHostException");
		} catch (IOException e) {
			System.out.println("openConnection() --> IOException");
		} catch (Exception e) {
			System.out.println("openConnection() --> Exception");
			System.out.println(e.toString());
		}

		System.out.println("Failed to establish connection!");
	}

	public void closeConnection() {
		System.out.println("Closing connection...");

		try {
			reader.close();
			if (socket != null) {
				socket.close();
				socket = null;
			}
			System.out.println("Connection closed!");
		} catch (IOException e) {
			System.out.println("closeConnection() --> IOException");
		} catch (NullPointerException e) {
			System.out.println("closeConnection() --> NullPointerException");
		} catch (Exception e) {
			System.out.println("closeConnection() --> Exception");
			System.out.println(e.toString());
		}
	}

	public void sendMsg(String msg, String msgType) {
		// System.out.println("Sending a message...");

		try {
			String outputMsg = "";
			// TODO: Define string tokens for sending messages

			// System.out.println("Sending out message: " + outputMsg);
			writer.write(outputMsg);
			writer.flush();
		} catch (IOException e) {
			System.out.println("sendMsg() --> IOException");
		} catch (Exception e) {
			System.out.println("sendMsg() --> Exception");
			System.out.println(e.toString());
		}
	}

	public String recvMsg() {
		try {
			StringBuilder sb = new StringBuilder();
			String input = reader.readLine();
			if (input != null && input.length() > 0) {
				sb.append(input);
				// System.out.println("message received: " + sb.toString());
				return sb.toString();
			}
		} catch (IOException e) {
			System.out.println("recvMsg() --> IOException");
		} catch (Exception e) {
			System.out.println("recvMsg() --> Exception");
			System.out.println(e.toString());
		}

		return null;
	}

	public boolean isConnected() {
		return socket.isConnected();
	}
}
