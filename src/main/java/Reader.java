import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

public class Reader {

    static Map<String, int[]> characters = new HashMap<>();

    public static void main(String[] args) throws IOException, InterruptedException {

        String currString = "?";
        StringBuilder temp = new StringBuilder();

        StringBuilder bufferTime = new StringBuilder();

        while (!currString.equals(temp.toString())) {
            currString = temp.toString();
            temp = new StringBuilder();
            BufferedReader objReader = new BufferedReader(new FileReader("C:\\darknet\\darknet-master\\build\\darknet\\x64\\detections.txt"));
            String strCurrentLine;
            while ((strCurrentLine = objReader.readLine()) != null) {
                temp.append(strCurrentLine).append(System.lineSeparator());
                if (strCurrentLine.startsWith("Objects")) {
                    bufferTime = new StringBuilder();
                } else {
                    if (!strCurrentLine.equals("")) {
                        bufferTime.append(strCurrentLine).append(System.lineSeparator());
                    }
                }
            }
            System.out.println(bufferTime);

            String[] charactersString = bufferTime.toString().split(System.lineSeparator());

            for (String string: charactersString) {
                String[] items = string.split("[:)]");
                int left = Integer.parseInt(items[2].trim().split(" ", 2)[0]);
                int top = Integer.parseInt(items[3].trim().split(" ", 2)[0]);
                int width = Integer.parseInt(items[4].trim().split(" ", 2)[0]);
                int height = Integer.parseInt(items[5].trim().split("\\)", 2)[0]);
                //characters.put(items[i], new int[] {left, top, width, height});
                System.out.println(left + " " + top + " " + width + " " + height);
            }
            Thread.sleep(1000);
        }
    }
}