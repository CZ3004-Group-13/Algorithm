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
                        int left = Integer.parseInt(items[2].trim().split(SPACE, 2)[0]);
                        int top = Integer.parseInt(items[3].trim().split(SPACE, 2)[0]);
                        int width = Integer.parseInt(items[4].trim().split(SPACE, 2)[0]);
                        int height = Integer.parseInt(items[5].trim().split("\\)", 2)[0]);
                        processInput(left, width, height);
                        System.out.println(items[0] + " Left: " + left + " Top: " + top + " Width: " + width + " Height: " + height);
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

    private void processInput(int left, int width, int height) {
        if (left < 170) {
            System.out.println("Image on the left!");
        } else if (left < 346) {
            System.out.println("Image on the centre!");
        } else {
            System.out.println("Image on the right!");
        }

        int area = width * height;

        if (area > 100000) {
            System.out.println("Image is about less than 15 cm away!");
        } else if (area > 70000) {
            System.out.println("Image is about 20 cm away!");
        } else if (area > 45000) {
            System.out.println("Image is about 25 cm away!");
        } else if (area > 35000) {
            System.out.println("Image is about 30 cm away!");
        } else if (area > 28000) {
            System.out.println("Image is about 35 cm away!");
        } else if (area > 21000) {
            System.out.println("Image is about 40 cm away!");
        } else if (area > 17000) {
            System.out.println("Image is about 45 cm away!");
        } else {
            System.out.println("Image is more than 50 cm away!");
        }

        double slantness = (double) height / width;

        // System.out.println(slantness);

        if (slantness < 1.25) {
            System.out.println("Not slanted");
        } else {
            System.out.println("Very slanted");
        }
    }
}