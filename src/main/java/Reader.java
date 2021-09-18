import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
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
        // readTextFile();
        processOutputDirectly();

    }

    private static void processOutputDirectly() {
         new Thread(() -> {
            try {
                ProcessBuilder builder = new ProcessBuilder(COMMAND.split(SPACE));
                builder.directory(new File(DIRECTORY));
                builder.redirectErrorStream(true);
                final Process process = builder.start();
                BufferedReader in = new BufferedReader(new InputStreamReader(process.getInputStream()));
                String line;
                boolean isNextLines = false;
                while ((line = in.readLine()) != null) {
                    if (line.startsWith(FPS)) {
                        isNextLines = false;
                    }
                    if (isNextLines && !line.equals(EMPTY_STRING)) {
                        String[] items = line.split("[:)]");
                        int left = Integer.parseInt(items[2].trim().split(SPACE, 2)[0]);
                        int top = Integer.parseInt(items[3].trim().split(SPACE, 2)[0]);
                        int width = Integer.parseInt(items[4].trim().split(SPACE, 2)[0]);
                        int height = Integer.parseInt(items[5].trim().split("\\)", 2)[0]);
                        System.out.println(left + SPACE + top + SPACE + width + SPACE + height);
                    }
                    if (line.startsWith(OBJECTS)) {
                        isNextLines = true;
                    }
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
        }).start();
    }

    private static void readTextFile() throws IOException, InterruptedException {
        String currString = "?";
        StringBuilder temp = new StringBuilder();

        StringBuilder bufferTime = new StringBuilder();

        while (!currString.equals(temp.toString())) {
            currString = temp.toString();
            temp = new StringBuilder();
            BufferedReader objReader = new BufferedReader(new FileReader(DIRECTORY + "\\detections.txt"));
            String strCurrentLine;
            while ((strCurrentLine = objReader.readLine()) != null) {
                temp.append(strCurrentLine).append(System.lineSeparator());
                if (strCurrentLine.startsWith(OBJECTS)) {
                    bufferTime = new StringBuilder();
                } else {
                    if (!strCurrentLine.equals(EMPTY_STRING)) {
                        bufferTime.append(strCurrentLine).append(System.lineSeparator());
                    }
                }
            }
            System.out.println(bufferTime);

            String[] charactersString = bufferTime.toString().split(System.lineSeparator());

            for (String string: charactersString) {
                String[] items = string.split("[:)]");
                int left = Integer.parseInt(items[2].trim().split(SPACE, 2)[0]);
                int top = Integer.parseInt(items[3].trim().split(SPACE, 2)[0]);
                int width = Integer.parseInt(items[4].trim().split(SPACE, 2)[0]);
                int height = Integer.parseInt(items[5].trim().split("\\)", 2)[0]);
                System.out.println(left + SPACE + top + SPACE + width + SPACE + height);
            }
            Thread.sleep(1000);
        }
    }
}