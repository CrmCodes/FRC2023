// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.opencv.core.*;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorDetectCVSubsystem extends SubsystemBase {

  Thread m_colorThread;
  Thread m_colorMask;
  Thread m_inRange;
  public double mc_Aim = 0;
  public double mc_Distance = 0;

  public double centerY;
  public double centerX;
  /** Creates a new ColorDetectCV. */
  public ColorDetectCVSubsystem() {

    // var LowYellow = new Scalar(15,83,128);
    // var HighYellow = new Scalar(52,178,237);

    m_colorThread =
    new Thread(
        () -> {
          var camera = CameraServer.startAutomaticCapture(0);
          var cameraWidth = 640;
          var cameraHeight = 480;

          camera.setResolution(cameraWidth, cameraHeight);

          var cvSink = CameraServer.getVideo();
          var outputStream = CameraServer.putVideo("ColorDetect", cameraWidth, cameraHeight);
          // var outputStream2 = CameraServer.putVideo("ColorMask", cameraWidth, cameraHeight);
          
          // Create a new Mat object to store the current frame
        var frame = new Mat();
        var mask = new Mat();
        var hsv = new Mat();

        while (!Thread.interrupted()) {
          if (cvSink.grabFrame(frame) == 0) {
            outputStream.notifyError(cvSink.getError());
            continue;
          }

          // var LowYellow = new Scalar(51,91,115);
          // var HighYellow = new Scalar(146,246,255);

          // // var LowYellow = new Scalar(15,83,83);
          // // var HighYellow = new Scalar(52,178,237);

          //   // Perform color thresholding on the frame
          //   Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_BGR2HSV);

          //   Core.inRange(hsv, LowYellow, HighYellow, mask);

          //   List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

          //   Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

          //   double maxArea = 0;
          //   int biggestContour = 0;

            // for(int i = 0; i < contours.size(); i++) {

            //   double area = Imgproc.contourArea(contours.get(i));
            //   if (area > maxArea) {
            //        maxArea = area;
            //        biggestContour = i;
            //    }

            //    Imgproc.drawContours(frame, Arrays.asList(contours.get(biggestContour)), -1, new Scalar(0, 255, 0), -1);

            //    // Find the middle coordinate of the biggest contour
            //     Moments moments = Imgproc.moments(contours.get(biggestContour));
            //     Point center = new Point(moments.m10 / moments.m00, moments.m01 / moments.m00);
            //     Imgproc.circle(frame, center, 5, new Scalar(0, 0, 255), -2);

            //     SmartDashboard.putNumber("centerY : ", centerY);
            //     SmartDashboard.putNumber("centerX : ", centerX);

            //     centerY = center.y / 600;
            //     centerX = center.x / 600;
                

            //   // /* METHOD 1 */
            //   // Imgproc.rectangle(frame, rect.tl(), rect.br(), new Scalar(0, 0, 255), 2);

            //   //   SQUARE COORDINATE FORMULA   //
            //   // var topleftX = rect.tl().x;
            //   // var topLeftY = rect.tl().y;

            //   // var topRightX = topleftX + rect.width;
            //   // var topRightY = topLeftY;

            //   // var bottomLeftX = topleftX;
            //   // var bottomLeftY = topLeftY + rect.height;

            //   // var bottomRightX = topRightX;
            //   // var bottomRightY = bottomLeftY;

            //   // var midY = topLeftY + bottomLeftY / 2 / 500;
            //   // var midX = topleftX + topRightX / 2 / 500;

            //   // System.out.println("middle X : " + midX + ", middle Y : " + midY);

            //   // System.out.println("Edge points: (" + rect.x + "," + rect.y + "), (" + (rect.x + rect.width) + "," + rect.y + "), (" + rect.x + "," + (rect.y + rect.height) + "), (" + (rect.x + rect.width) + "," + (rect.y + rect.height) + ")");

            //   }
              
              
              
              
              // Imgproc.drawContours(frame, contours, i, new Scalar(0, 0, 255));
            
            // for (int i = 0; i < contours.size(); i++) {
            // Imgproc.drawContours(frame, contours, i, new Scalar(0, 0, 255), 2);
            // }

            // Output the processed frame
            outputStream.putFrame(frame);
      
            // outputStream2.putFrame(mask);

        }

        });
  m_colorThread.setDaemon(true);
  m_colorThread.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
