import simulator.connection.Connection;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;

public class ImageDetectionProcessor {
    private static final String COMMAND = "C:\\darknet\\darknet-master\\build\\darknet\\x64\\darknet.exe detector demo data/yolov4.data cfg/yolov4_custom_test.cfg backup/yolov4_custom_train_final.weights -c 1 -thresh 0.9 -ext_output";
    private static final String DIRECTORY = "C:\\darknet\\darknet-master\\build\\darknet\\x64";
    private static final String OBJECTS = "Objects";
    private static final String FPS = "FPS";
    private static final String SPACE = " ";
    private static final String EMPTY_STRING = "";
    private static final Connection conn = Connection.getConnection();

    public static void main(String[] args) {
        new ImageDetectionProcessor().RunImageRecognition();
    }

    /**
     * Runs image recognition and directly processes input from command line.
     */
    private void RunImageRecognition() {
        new Thread(() -> {
            try {
                long timeNow = System.currentTimeMillis();
                ProcessBuilder builder = new ProcessBuilder(COMMAND.split(SPACE));
                builder.directory(new File(DIRECTORY));
                builder.redirectErrorStream(true);
                BufferedReader in = new BufferedReader(new InputStreamReader(builder.start().getInputStream()));
                String line;
                boolean isNextLines = false;
                boolean isThereItems = false;

                if (!conn.isConnected()) {
                    System.out.println("opening connection");
                    conn.openConnection("192.168.13.13", 3053);
                    if (conn.isConnected()) {
                        System.out.println("connection opened");
                    }
                }

                while ((line = in.readLine()) != null) {
                    if (line.startsWith(FPS)) {

                        // Print new line to separate between detections
                        if (isThereItems) {
                            System.out.println();
                            isThereItems = false;
                        }
                        isNextLines = false;
                    }

                    // Format detection
                    if (isNextLines && !line.equals(EMPTY_STRING)) {
                        isThereItems = true;
                        String[] items = line.split("[:)]");
                        String item = items[0];
                        int left = Integer.parseInt(items[2].trim().split(SPACE, 2)[0]);
                        int top = Integer.parseInt(items[3].trim().split(SPACE, 2)[0]);
                        int width = Integer.parseInt(items[4].trim().split(SPACE, 2)[0]);
                        int height = Integer.parseInt(items[5].trim().split("\\)", 2)[0]);

                        // This is the command that is output
                        String command = processInput(item, left, width, height);

                        // send command to rpi
                        if (command != null) {
                            conn.sendMsg(command, "type");
                        }
                        System.out.println(
                                item + " Left: " + left + " Top: " + top + " Width: " + width + " Height: " + height);

                        if (command.equals("s")) {
                            // conn.closeConnection();
                            return;
                        }
                    }

                    // Only print detections if it is past 1 second since previous print
                    if (line.startsWith(OBJECTS) && System.currentTimeMillis() - timeNow > 1000) {
                        timeNow = System.currentTimeMillis();
                        isNextLines = true;
                    }
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
        }).start();
    }

    private String processInput(String item, int left, int width, int height) {
        if (left < 170) {
            System.out.println("Image on the left!");
        } else if (left < 346) {
            System.out.println("Image on the centre!");
        } else {
            System.out.println("Image on the right!");
        }

        if (height > 450) {
            System.out.println("Image is about less than 15 cm away!");
        } else if (height > 340) {
            System.out.println("Image is about 20 cm away!");
        } else if (height > 260) {
            System.out.println("Image is about 25 cm away!");
        } else if (height > 210) {
            System.out.println("Image is about 30 cm away!");
        } else if (height > 190) {
            System.out.println("Image is about 35 cm away!");
        } else if (height > 160) {
            System.out.println("Image is about 40 cm away!");
        } else if (height > 140) {
            System.out.println("Image is about 45 cm away!");
        } else {
            System.out.println("Image is more than 50 cm away!");
        }
        System.out.println(height);

        double slantness = (double) height / width;

        if (slantness < 1.25) {
            System.out.println("Not slanted");
        } else {
            System.out.println("Very slanted");
        }

        // return item.equals("stop") ? "s" : "g";
        switch (item) {
            case "stop":
                return "s";
            case "bullseye":
                return "g";
            default:
                return "?";
        }
        // return item.equals("bullseye") ? "g" : "s";
    }
}