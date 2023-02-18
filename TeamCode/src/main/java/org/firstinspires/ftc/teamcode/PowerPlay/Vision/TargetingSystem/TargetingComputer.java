package org.firstinspires.ftc.teamcode.PowerPlay.Vision.TargetingSystem;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import java.util.concurrent.TimeUnit;

public class TargetingComputer {

	public void run(String[] args) {
		// TODO Auto-generated method stub
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        
		boolean DROP_CONE;
		boolean CONE_READY = true;
		
		int delayTimer = 0;
		// Turn ON Camera
        VideoCapture capture = new  VideoCapture(0);  // Change VideoCapture argument to camera number (0 to n for n cameras)

		while(CONE_READY) {
			
			try {
				TimeUnit.SECONDS.sleep(1);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}

			delayTimer++;
			
			// capture image
			Mat matrix = new Mat(); 
			capture.read(matrix);
			
			// grayscale and blur image
	        Mat gray = new Mat();
	        Imgproc.cvtColor(matrix, gray, Imgproc.COLOR_BGR2GRAY);
	        Imgproc.medianBlur(gray, gray, 5);
	        
	        // find circles
	        Mat circles = new Mat();
	        Imgproc.HoughCircles(gray, circles, Imgproc.HOUGH_GRADIENT, 1.0,
	                (double)gray.rows()/16, // change this value to detect circles with different distances to each other
	                100.0, 30.0, 30, 50); // change the last two parameters
	                // (min_radius & max_radius) to detect larger circles


			boolean circle = true;
			if(circles.cols() > 0) {
				DROP_CONE = true;
				CONE_READY = false;
			}
			else {
				DROP_CONE = false;
			}
			
			System.out.println("Timer=" + delayTimer + ", circle=" + circle + ", DROP_CONE= " + DROP_CONE);
			System.out.println(circles);
			}
		}
		// take picture once per second
		
		// Decision
		// Safe to Drop?
		// if Yes, set DROP_CONE to true
		// if no, set DROP_CONE to false
}
