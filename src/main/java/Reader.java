import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

public class Reader {

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
                    if (!strCurrentLine.equals("\n")) {
                        bufferTime.append(strCurrentLine + System.lineSeparator());
                    }
                }
            }
            System.out.println(bufferTime);
            Thread.sleep(1000);
        }
    }
}