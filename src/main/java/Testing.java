import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.highgui.HighGui;
import org.opencv.imgcodecs.Imgcodecs;

public class Testing {


    public static void main (String[] args) {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        Imgcodecs imgcodecs = new Imgcodecs();
        Mat matrix = Imgcodecs.imread("C:\\darknet\\darknet-master\\build\\darknet\\x64\\examples\\test_image_1.jpg");
        HighGui.imshow("Image", matrix);
        HighGui.waitKey();
    }
}
