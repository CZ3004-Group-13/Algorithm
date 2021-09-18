import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;

public class Reader {
    private static final String COMMAND = "C:\\darknet\\darknet-master\\build\\darknet\\x64\\darknet.exe detector demo data/yolov4.data cfg/yolov4_custom_test.cfg backup/yolov4_custom_train_final.weights -c 1 -thresh 0.9 -ext_output";
    private static final String DIRECTORY = "C:\\darknet\\darknet-master\\build\\darknet\\x64";
    private static final String OBJECTS = "Objects";
    private static final String FPS = "FPS";
    private static final String SPACE = " ";
    private static final String EMPTY_STRING = "";

    public static void main(String[] args) {
        new Reader().processOutputDirectly();
    }

    /**
     * Directly processes input from command line.
     */
    private void processOutputDirectly() {
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
}