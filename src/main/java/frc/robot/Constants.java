// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.resources.customapriltagfield;
import swervelib.math.Matter;
import edu.wpi.first.math.Matrix;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (83) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(30); //was 14.5
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }
public static final class DriveConstants {

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
  }
  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
  public static class Vision4920 {
    public static final String kGreyFeederCam = "GreyFeederCam";
    public static final String kGreyReefCam = "GreyReefCam";
    public static final String kRedReefCam = "RedReefCam";
    public static final String kRedGeneralCam = "RedGeneralCam";
    public static final String kBlueGeneralCam = "BlueGeneralCam";
    public static final String kBlueFrontCam = "BlueFrontCam";
    public static final String kCenterCam = "CenterCam";
    public static final String kRightCam = "RightCam";
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.


    // positive x to the left, positive y up
    public static final Transform3d kRobotToCenterCam =
            new Transform3d(new Translation3d(Units.inchesToMeters(14), Units.inchesToMeters(-1.375), Units.inchesToMeters(19)), 
            new Rotation3d(Units.degreesToRadians(359), Units.degreesToRadians(8), Units.degreesToRadians(2))); //

    public static final Transform3d kRobotToRightCam =
            new Transform3d(new Translation3d(Units.inchesToMeters(15), Units.inchesToMeters(-9.5), Units.inchesToMeters(19)), 
            new Rotation3d(Units.degreesToRadians(1), Units.degreesToRadians(7), Units.degreesToRadians(4))); //

    public static final Transform3d kRobotToGreyFeederCam =
            new Transform3d(new Translation3d(Units.inchesToMeters(-3.25), Units.inchesToMeters(-10.75), Units.inchesToMeters(38.25)), 
            new Rotation3d(0, Units.degreesToRadians(309), Units.degreesToRadians(180))); //
  public static final Transform3d kRobotToGreyReefCam =
            new Transform3d(new Translation3d(Units.inchesToMeters(14.75), Units.inchesToMeters(-5.5), Units.inchesToMeters(7.5)), 
            new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(-13), Units.degreesToRadians(0))); // 0.48
  public static final Transform3d kRobotToRedReefCam =
            new Transform3d(new Translation3d(Units.inchesToMeters(14.75), Units.inchesToMeters(5.5), Units.inchesToMeters(7.5)), 
            new Rotation3d(0, Units.degreesToRadians(-13.0), 0)); // 0.48
  public static final Transform3d kRobotToRedGeneralCam =
            new Transform3d(new Translation3d(Units.inchesToMeters(-3.25), Units.inchesToMeters(10.75), Units.inchesToMeters(37.75)), 
            new Rotation3d(0, Units.degreesToRadians(12), Units.degreesToRadians(180))); // 0.48
  
  public static final Transform3d kRobotToBlueGeneralCam =
            new Transform3d(new Translation3d(Units.inchesToMeters(0.75), Units.inchesToMeters(-14.25), Units.inchesToMeters(40.25)), 
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-48), Units.degreesToRadians(270))); // 0.48
   
            public static final Transform3d kRobotToBlueFrontCam =
            new Transform3d(new Translation3d(Units.inchesToMeters(12.5), Units.inchesToMeters(-13), Units.inchesToMeters(34.5)), 
            new Rotation3d(0, Units.degreesToRadians(346), 0)); // 0.48
  
  public static final Transform3d ROBOT_TO_CAMERA_Front = kRobotToGreyFeederCam.inverse();
  public static final Transform3d ROBOT_TO_CAMERA_Rear = kRobotToGreyReefCam.inverse();
  public static final Transform3d ROBOT_TO_CAMERA_Right = kRobotToRedReefCam .inverse();
  public static final Transform3d ROBOT_TO_CAMERA_Left = kRobotToRedGeneralCam.inverse();
  public static final Transform3d ROBOT_TO_CAMERA_Center = kRobotToCenterCam.inverse();
public static final customapriltagfield atag = new customapriltagfield();
    
    public static final AprilTagFieldLayout kTagLayout = new AprilTagFieldLayout(atag.getTags(), 17.548, 8.052);  
        // public static final AprilTagFieldLayout kTagLayout =
        //         AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField();
   

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    //public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(.75, .75, 1.2);
    
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(.5, .5, 1);

// from Hemlock5712
    /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
    public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
    public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
    public static final double NOISY_DISTANCE_METERS = 2.5;
    public static final double DISTANCE_WEIGHT = 7;
    public static final int TAG_PRESENCE_WEIGHT = 10;

    /**
     * Standard deviations of model states. Increase these numbers to trust your
     * model's state estimates less. This
     * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
     * meters.
     *
     * Note: These values are used as base multipliers in confidenceCalculator().
     * Tuned based on observed vision measurement errors:
     * - x, y: 0.5m std dev for close measurements, increases with distance
     * - theta: 0.2 rad std dev, relatively stable
     */
    public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS = VecBuilder
        .fill(
            // if these numbers are less than one, multiplying will do bad things
            0.5, // x - reduced from 1.0 for better trust in vision measurements
            0.5, // y - reduced from 1.0 for better trust in vision measurements
            0.2 * Math.PI // theta - reduced from 1.0*PI for better rotation estimates
        );

    /**
     * Standard deviations of the vision measurements. Increase these numbers to
     * trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
     * radians.
     */
   



    public static final Matrix<N3, N1> STATE_STANDARD_DEVIATIONS = VecBuilder
        .fill(
            // if these numbers are less than one, multiplying will do bad things
            .1, // x
            .1, // y
            .1);

}
}
