// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveModule;
import dev.doglog.DogLog;

/**
 * Pre-match calibration routine for swerve drive system.
 * Performs gyro calibration, encoder verification, and module diagnostics.
 * Designed for pit testing - only uses in-place rotation, no linear movement.
 *
 * Inspired by Team 254's pre-match calibration practices.
 */
public class PreMatchCalibrationCommand extends Command {

  private final SwerveSubsystem swerveSubsystem;

  // Calibration state machine
  private enum CalibrationState {
    INIT,
    GYRO_CALIBRATION,
    GYRO_WAIT,
    MODULE_ANGLE_CHECK,
    MODULE_CENTER,
    ROTATION_TEST_START,
    ROTATION_TEST_CW,
    ROTATION_TEST_CCW,
    ROTATION_TEST_RETURN,
    MODULE_RESPONSE_TEST,
    ODOMETRY_CONSISTENCY_TEST,
    MODULE_CURRENT_TEST,
    COMPLETE,
    FAILED
  }

  private CalibrationState currentState;
  private Timer stateTimer;
  private int moduleIndex;

  // Calibration results
  private boolean gyroCalibrationSuccess;
  private boolean[] moduleAngleValid;
  private boolean rotationTestSuccess;
  private boolean[] modulePIDResponse;
  private boolean odometryConsistencySuccess;
  private boolean[] moduleCurrentNormal;

  // Test parameters
  private static final double GYRO_CALIBRATION_TIME = 3.0; // seconds
  private static final double ROTATION_TEST_SPEED = 0.5; // rad/s (full rotation for accurate wheel diameter measurement)
  private static final double ROTATION_TEST_DURATION = 5.0; // seconds (longer rotation = more encoder distance = better accuracy)
  private static final double GYRO_DRIFT_TOLERANCE = 1.0; // degrees
  private static final double ODOMETRY_POSITION_TOLERANCE = 0.05; // meters (5cm)
  private static final double MAX_NORMAL_ANGLE_CURRENT = 15.0; // amps (SparkMax NEO550)
  private static final double MAX_NORMAL_DRIVE_CURRENT = 20.0; // amps (Kraken stationary)

  // Wheel diameter measurement constants
  private static final double CONFIGURED_WHEEL_DIAMETER_INCHES = 2.833; // From physicalproperties.json
  private static final double WHEEL_WEAR_WARNING_THRESHOLD = 0.05; // 5% wear (0.15 inches on 3" wheel)

  // Data recording
  private Rotation2d initialGyroAngle;
  private Rotation2d[] initialModuleAngles;
  private Rotation2d testStartGyroAngle;
  private double expectedGyroChange;
  private double actualGyroChange;

  // Advanced diagnostics
  private double[] moduleDistances; // Distance each module traveled
  private double[] peakAngleCurrents; // Peak current for angle motors
  private double[] peakDriveCurrents; // Peak current for drive motors
  private Translation2d odometryDrift; // How far robot thinks it moved during rotation test
  private double[] measuredWheelDiameters; // Measured effective wheel diameter in inches
  private double[] wheelWearPercentage; // Percentage of tread wear per module

  // Per-module rotation tracking for diagnosis
  private double[] moduleStartAngles; // Module steering angles at rotation start
  private double[] moduleCWEndAngles; // Module angles after CW rotation
  private double[] moduleCCWEndAngles; // Module angles after CCW rotation
  private double cwGyroChange; // Gyro change during CW rotation
  private double ccwGyroChange; // Gyro change during CCW rotation
  private double[] ccwEncoderStartPos; // Encoder positions at start of CCW rotation

  public PreMatchCalibrationCommand(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);

    stateTimer = new Timer();
    moduleAngleValid = new boolean[4];
    modulePIDResponse = new boolean[4];
    moduleCurrentNormal = new boolean[4];
    initialModuleAngles = new Rotation2d[4];
    moduleDistances = new double[4];
    peakAngleCurrents = new double[4];
    peakDriveCurrents = new double[4];
    moduleStartAngles = new double[4];
    moduleCWEndAngles = new double[4];
    moduleCCWEndAngles = new double[4];
    measuredWheelDiameters = new double[4];
    wheelWearPercentage = new double[4];
    ccwEncoderStartPos = new double[4];
  }

  @Override
  public void initialize() {
    currentState = CalibrationState.INIT;
    stateTimer.reset();
    gyroCalibrationSuccess = false;
    rotationTestSuccess = false;
    moduleIndex = 0;

    // Initialize arrays
    for (int i = 0; i < 4; i++) {
      moduleAngleValid[i] = false;
      modulePIDResponse[i] = false;
    }

    DogLog.log("Calibration/Status", "Pre-Match Calibration Started");
    SmartDashboard.putString("Calibration Status", "Starting...");
    SmartDashboard.putBoolean("Calibration Complete", false);
  }

  @Override
  public void execute() {
    switch (currentState) {
      case INIT:
        handleInit();
        break;
      case GYRO_CALIBRATION:
        handleGyroCalibration();
        break;
      case GYRO_WAIT:
        handleGyroWait();
        break;
      case MODULE_ANGLE_CHECK:
        handleModuleAngleCheck();
        break;
      case MODULE_CENTER:
        handleModuleCenter();
        break;
      case ROTATION_TEST_START:
        handleRotationTestStart();
        break;
      case ROTATION_TEST_CW:
        handleRotationTestCW();
        break;
      case ROTATION_TEST_CCW:
        handleRotationTestCCW();
        break;
      case ROTATION_TEST_RETURN:
        handleRotationTestReturn();
        break;
      case MODULE_RESPONSE_TEST:
        handleModuleResponseTest();
        break;
      case ODOMETRY_CONSISTENCY_TEST:
        handleOdometryConsistencyTest();
        break;
      case MODULE_CURRENT_TEST:
        handleModuleCurrentTest();
        break;
      case COMPLETE:
        // Do nothing, waiting for command to end
        break;
      case FAILED:
        // Do nothing, waiting for command to end
        break;
    }

    // Update dashboard
    updateDashboard();
  }

  private void handleInit() {
    DogLog.log("Calibration/State", "INIT");
    SmartDashboard.putString("Calibration Status", "Initializing...");

    // Stop the robot
    swerveSubsystem.drive(new ChassisSpeeds(0, 0, 0));

    // Move to gyro calibration
    currentState = CalibrationState.GYRO_CALIBRATION;
    stateTimer.restart();
  }

  private void handleGyroCalibration() {
    DogLog.log("Calibration/State", "GYRO_CALIBRATION");
    SmartDashboard.putString("Calibration Status", "Calibrating Gyro - Keep Robot Still!");

    if (stateTimer.get() < 0.5) {
      // Wait a bit before starting calibration
      return;
    }

    if (stateTimer.get() < 0.6) {
      // Zero the gyro (YAGSL will handle calibration internally)
      swerveSubsystem.zeroGyro();
      initialGyroAngle = swerveSubsystem.getHeading();
      DogLog.log("Calibration/GyroInitialAngle", initialGyroAngle.getDegrees());
    }

    if (stateTimer.get() >= GYRO_CALIBRATION_TIME) {
      currentState = CalibrationState.GYRO_WAIT;
      stateTimer.restart();
    }
  }

  private void handleGyroWait() {
    DogLog.log("Calibration/State", "GYRO_WAIT");

    if (stateTimer.get() >= 1.0) {
      // Check if gyro has drifted significantly
      Rotation2d currentGyro = swerveSubsystem.getHeading();
      double drift = Math.abs(currentGyro.getDegrees() - initialGyroAngle.getDegrees());

      gyroCalibrationSuccess = drift < GYRO_DRIFT_TOLERANCE;

      DogLog.log("Calibration/GyroDrift", drift);
      DogLog.log("Calibration/GyroCalibrationSuccess", gyroCalibrationSuccess);

      if (gyroCalibrationSuccess) {
        currentState = CalibrationState.MODULE_ANGLE_CHECK;
        stateTimer.restart();
      } else {
        DogLog.log("Calibration/Error", "Gyro drift too high: " + drift + " degrees");
        currentState = CalibrationState.FAILED;
      }
    }
  }

  private void handleModuleAngleCheck() {
    DogLog.log("Calibration/State", "MODULE_ANGLE_CHECK");
    SmartDashboard.putString("Calibration Status", "Checking Module Encoders...");

    // Read all module angles
    SwerveModule[] modules = swerveSubsystem.getSwerveDrive().getModules();

    String[] moduleNames = {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};

    for (int i = 0; i < modules.length; i++) {
      // Get raw absolute encoder position
      double rawPosition = modules[i].getAbsolutePosition();
      Rotation2d moduleAngle = Rotation2d.fromDegrees(rawPosition);
      initialModuleAngles[i] = moduleAngle;

      // Check if encoder is responding (not NaN, not stuck at exactly 0.0)
      // The actual angle value doesn't matter - we just need to know the encoder works
      boolean isConnected = !Double.isNaN(rawPosition) && !Double.isInfinite(rawPosition);

      // Additional check: see if reading is in valid range
      // Some encoders return values outside -180 to 180 before wrapping
      boolean inReasonableRange = Math.abs(rawPosition) < 360;

      moduleAngleValid[i] = isConnected && inReasonableRange;

      // Detailed logging for diagnostics
      DogLog.log("Calibration/" + moduleNames[i] + "/RawPosition", rawPosition);
      DogLog.log("Calibration/" + moduleNames[i] + "/IsConnected", isConnected);
      DogLog.log("Calibration/" + moduleNames[i] + "/InRange", inReasonableRange);
      DogLog.log("Calibration/" + moduleNames[i] + "/Valid", moduleAngleValid[i]);

      // Put on SmartDashboard for easy viewing
      SmartDashboard.putNumber("Cal/" + moduleNames[i] + " Raw Angle", rawPosition);
      SmartDashboard.putBoolean("Cal/" + moduleNames[i] + " OK", moduleAngleValid[i]);

      // If encoder offset is needed, calculate it
      // Offset should be: current_raw_position (when module points forward)
      SmartDashboard.putNumber("Cal/" + moduleNames[i] + " Suggested Offset", -rawPosition);
    }

    // Check if all modules are valid
    boolean allValid = true;
    StringBuilder failedModules = new StringBuilder("Failed modules: ");
    for (int i = 0; i < moduleAngleValid.length; i++) {
      if (!moduleAngleValid[i]) {
        allValid = false;
        failedModules.append(moduleNames[i]).append(" ");
      }
    }

    if (allValid) {
      currentState = CalibrationState.MODULE_CENTER;
      stateTimer.restart();
    } else {
      DogLog.log("Calibration/Error", failedModules.toString());
      SmartDashboard.putString("Calibration Error", failedModules.toString());
      currentState = CalibrationState.FAILED;
    }
  }

  private void handleModuleCenter() {
    DogLog.log("Calibration/State", "MODULE_CENTER");
    SmartDashboard.putString("Calibration Status", "Centering Modules...");

    SwerveModule[] modules = swerveSubsystem.getSwerveDrive().getModules();
    String[] moduleNames = {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};

    if (stateTimer.get() < 1.5) {
      // Command all modules to point forward
      for (SwerveModule module : modules) {
        module.setAngle(0.0);
      }
    } else {
      // Verify modules are centered
      // Use getRelativePosition() which accounts for the encoder offset
      // This is the angle the module thinks it's at, relative to forward
      boolean allCentered = true;

      for (int i = 0; i < modules.length; i++) {
        // Get the relative position (offset-corrected) from the state
        double relativeAngle = modules[i].getState().angle.getDegrees();
        double absoluteAngle = modules[i].getAbsolutePosition();

        // Error from target (0 degrees = forward)
        double error = Math.abs(relativeAngle);

        // Handle wraparound (e.g., -179 or 179 should be considered close to 0)
        if (error > 180) {
          error = 360 - error;
        }

        // Use a more relaxed tolerance since offsets may not be calibrated yet
        // 4 degrees allows some margin for uncalibrated modules
        boolean centered = error < 4.0;

        DogLog.log("Calibration/" + moduleNames[i] + "/RelativeAngle", relativeAngle);
        DogLog.log("Calibration/" + moduleNames[i] + "/AbsoluteAngle", absoluteAngle);
        DogLog.log("Calibration/" + moduleNames[i] + "/CenterError", error);
        DogLog.log("Calibration/" + moduleNames[i] + "/Centered", centered);

        SmartDashboard.putNumber("Cal/" + moduleNames[i] + " Center Error", error);
        SmartDashboard.putBoolean("Cal/" + moduleNames[i] + " Centered", centered);

        if (!centered) {
          allCentered = false;
          DogLog.log("Calibration/Warning", moduleNames[i] +
              " failed to center. Error: " + String.format("%.2f", error) + " degrees");
        }
      }

      if (allCentered) {
        currentState = CalibrationState.ROTATION_TEST_START;
        stateTimer.restart();
      } else {
        DogLog.log("Calibration/Error", "Modules failed to center properly");
        SmartDashboard.putString("Calibration Error", "Module centering failed - check PID or mechanical binding");
        currentState = CalibrationState.FAILED;
      }
    }
  }

  private void handleRotationTestStart() {
    DogLog.log("Calibration/State", "ROTATION_TEST_START");
    SmartDashboard.putString("Calibration Status", "Starting Rotation Test...");

    testStartGyroAngle = swerveSubsystem.getHeading();
    expectedGyroChange = 0.0;

    // Record starting odometry position to check for drift
    Pose2d startPose = swerveSubsystem.getPose();
    odometryDrift = startPose.getTranslation();

    // Reset module distance tracking and record starting steering angles
    SwerveModule[] modules = swerveSubsystem.getSwerveDrive().getModules();
    for (int i = 0; i < modules.length; i++) {
      moduleDistances[i] = modules[i].getDriveMotor().getPosition();
      moduleStartAngles[i] = modules[i].getState().angle.getDegrees();
    }

    currentState = CalibrationState.ROTATION_TEST_CW;
    stateTimer.restart();
  }

  private void handleRotationTestCW() {
    DogLog.log("Calibration/State", "ROTATION_TEST_CW");
    SmartDashboard.putString("Calibration Status", "Rotation Test - Clockwise");

    // Rotate clockwise (positive rotation)
    swerveSubsystem.drive(new ChassisSpeeds(0, 0, ROTATION_TEST_SPEED));

    expectedGyroChange += ROTATION_TEST_SPEED * 0.02 * (180.0 / Math.PI); // Convert to degrees

    // Debug: Log live encoder position and velocity during rotation
    SwerveModule[] modules = swerveSubsystem.getSwerveDrive().getModules();
    SmartDashboard.putNumber("Cal/Live FR Pos", modules[1].getDriveMotor().getPosition());
    SmartDashboard.putNumber("Cal/Live FR Vel", modules[1].getDriveMotor().getVelocity());
    SmartDashboard.putNumber("Cal/Live FR Start", moduleDistances[1]);

    if (stateTimer.get() >= ROTATION_TEST_DURATION) {
      swerveSubsystem.drive(new ChassisSpeeds(0, 0, 0));

      // Record gyro change and module angles at end of CW rotation
      cwGyroChange = swerveSubsystem.getHeading().getDegrees() - testStartGyroAngle.getDegrees();
      for (int i = 0; i < modules.length; i++) {
        moduleCWEndAngles[i] = modules[i].getState().angle.getDegrees();

        // Calculate distance traveled during CW rotation and store it
        // Save current position for CCW phase calculation
        double currentPos = modules[i].getDriveMotor().getPosition();
        double cwDistance = Math.abs(currentPos - moduleDistances[i]);
        moduleDistances[i] = cwDistance; // Store CW distance
        ccwEncoderStartPos[i] = currentPos; // Save CCW start position
      }

      currentState = CalibrationState.ROTATION_TEST_CCW;
      stateTimer.restart();
    }
  }

  private void handleRotationTestCCW() {
    DogLog.log("Calibration/State", "ROTATION_TEST_CCW");
    SmartDashboard.putString("Calibration Status", "Rotation Test - Counter-Clockwise");

    // Rotate counter-clockwise (negative rotation)
    swerveSubsystem.drive(new ChassisSpeeds(0, 0, -ROTATION_TEST_SPEED));

    expectedGyroChange -= ROTATION_TEST_SPEED * 0.02 * (180.0 / Math.PI); // Convert to degrees

    if (stateTimer.get() >= ROTATION_TEST_DURATION) {
      swerveSubsystem.drive(new ChassisSpeeds(0, 0, 0));

      // Record gyro change and module angles at end of CCW rotation
      ccwGyroChange = swerveSubsystem.getHeading().getDegrees() - testStartGyroAngle.getDegrees() - cwGyroChange;
      SwerveModule[] modules = swerveSubsystem.getSwerveDrive().getModules();
      for (int i = 0; i < modules.length; i++) {
        moduleCCWEndAngles[i] = modules[i].getState().angle.getDegrees();
      }

      currentState = CalibrationState.ROTATION_TEST_RETURN;
      stateTimer.restart();
    }
  }

  private void handleRotationTestReturn() {
    DogLog.log("Calibration/State", "ROTATION_TEST_RETURN");
    SmartDashboard.putString("Calibration Status", "Rotation Test - Analyzing...");

    if (stateTimer.get() >= 0.5) {
      Rotation2d currentGyro = swerveSubsystem.getHeading();
      actualGyroChange = currentGyro.getDegrees() - testStartGyroAngle.getDegrees();

      // Check if gyro tracked the rotation correctly
      // Since we rotated CW then CCW, we should be back near where we started
      double returnError = Math.abs(actualGyroChange);

      // Analyze rotation asymmetry - if CW and CCW are not equal, something is wrong
      double cwExpected = ROTATION_TEST_SPEED * ROTATION_TEST_DURATION * (180.0 / Math.PI);
      double ccwExpected = -cwExpected;
      double cwError = Math.abs(Math.abs(cwGyroChange) - cwExpected);
      double ccwError = Math.abs(Math.abs(ccwGyroChange) - cwExpected);
      double asymmetry = Math.abs(Math.abs(cwGyroChange) - Math.abs(ccwGyroChange));

      DogLog.log("Calibration/RotationTest/CWExpected", cwExpected);
      DogLog.log("Calibration/RotationTest/CWActual", cwGyroChange);
      DogLog.log("Calibration/RotationTest/CWError", cwError);
      DogLog.log("Calibration/RotationTest/CCWActual", ccwGyroChange);
      DogLog.log("Calibration/RotationTest/CCWError", ccwError);
      DogLog.log("Calibration/RotationTest/Asymmetry", asymmetry);

      SmartDashboard.putNumber("Cal/CW Rotation (deg)", cwGyroChange);
      SmartDashboard.putNumber("Cal/CCW Rotation (deg)", ccwGyroChange);
      SmartDashboard.putNumber("Cal/Rotation Asymmetry", asymmetry);

      // Per-module analysis to identify problematic modules
      String[] moduleNames = {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};
      SwerveModule[] modules = swerveSubsystem.getSwerveDrive().getModules();

      String suspectedModule = null;
      double worstModuleDeviation = 0;

      for (int i = 0; i < 4; i++) {
        // Check module steering angle consistency during rotation
        // During rotation, all modules should be pointing tangent to the rotation circle
        // Deviation from expected angle indicates module alignment or control issues
        double startAngle = moduleStartAngles[i];
        double cwAngle = moduleCWEndAngles[i];
        double ccwAngle = moduleCCWEndAngles[i];
        double finalAngle = modules[i].getState().angle.getDegrees();

        // Module should return close to starting steering angle
        double steeringReturn = Math.abs(finalAngle - startAngle);
        if (steeringReturn > 180) steeringReturn = 360 - steeringReturn;

        DogLog.log("Calibration/" + moduleNames[i] + "/SteeringStart", startAngle);
        DogLog.log("Calibration/" + moduleNames[i] + "/SteeringCWEnd", cwAngle);
        DogLog.log("Calibration/" + moduleNames[i] + "/SteeringCCWEnd", ccwAngle);
        DogLog.log("Calibration/" + moduleNames[i] + "/SteeringFinal", finalAngle);
        DogLog.log("Calibration/" + moduleNames[i] + "/SteeringReturnError", steeringReturn);

        if (steeringReturn > worstModuleDeviation) {
          worstModuleDeviation = steeringReturn;
          if (steeringReturn > 5.0) {
            suspectedModule = moduleNames[i];
          }
        }

        SmartDashboard.putNumber("Cal/" + moduleNames[i] + " Steering Return", steeringReturn);
      }

      // Determine rotation quality and provide actionable diagnostics
      String rotationQuality;
      String recommendation = "";

      if (returnError < 3.0) {
        rotationQuality = "EXCELLENT";
        rotationTestSuccess = true;
        recommendation = "Ready for competition autonomous.";
      } else if (returnError < 5.0) {
        rotationQuality = "GOOD";
        rotationTestSuccess = true;
        recommendation = "Acceptable for autonomous. Minor tuning may improve precision.";
      } else {
        // Rotation error >= 5° - identify the cause and recommend action
        rotationQuality = "NEEDS_CALIBRATION";
        rotationTestSuccess = false;

        // Determine likely cause based on diagnostics
        if (asymmetry > 5.0) {
          // Rotation is asymmetric - one direction is worse than the other
          if (cwError > ccwError) {
            recommendation = "CW rotation tracking worse than CCW. Check for:\n" +
                "  - Debris on one side of the robot\n" +
                "  - Asymmetric wheel wear\n" +
                "  - Module alignment on left side";
            DogLog.log("Calibration/Diagnosis", "Asymmetric rotation - CW worse");
          } else {
            recommendation = "CCW rotation tracking worse than CW. Check for:\n" +
                "  - Debris on one side of the robot\n" +
                "  - Asymmetric wheel wear\n" +
                "  - Module alignment on right side";
            DogLog.log("Calibration/Diagnosis", "Asymmetric rotation - CCW worse");
          }
        } else if (suspectedModule != null) {
          // One module has notably worse steering return
          recommendation = suspectedModule + " steering angle inconsistent. Check:\n" +
              "  - " + suspectedModule + " encoder offset calibration\n" +
              "  - " + suspectedModule + " steering PID tuning\n" +
              "  - Mechanical binding on " + suspectedModule;
          DogLog.log("Calibration/Diagnosis", "Module issue: " + suspectedModule);
        } else {
          // General slippage or floor traction issue
          recommendation = "Consistent rotation error suggests:\n" +
              "  - Floor surface too slippery (try different location)\n" +
              "  - All wheels worn evenly (measure wheel diameters)\n" +
              "  - Drive encoder calibration may be off";
          DogLog.log("Calibration/Diagnosis", "General slippage or traction issue");
        }
      }

      DogLog.log("Calibration/RotationTest/ExpectedChange", expectedGyroChange);
      DogLog.log("Calibration/RotationTest/ActualChange", actualGyroChange);
      DogLog.log("Calibration/RotationTest/ReturnError", returnError);
      DogLog.log("Calibration/RotationTest/Quality", rotationQuality);
      DogLog.log("Calibration/RotationTest/Success", rotationTestSuccess);
      DogLog.log("Calibration/RotationTest/Recommendation", recommendation);

      SmartDashboard.putNumber("Cal/Rotation Return Error", returnError);
      SmartDashboard.putString("Cal/Rotation Quality", rotationQuality);
      SmartDashboard.putString("Cal/Rotation Recommendation", recommendation);

      // Always continue to remaining tests to collect full diagnostic data
      // Even if rotation fails, we want to see PID response and current draw
      if (!rotationTestSuccess) {
        DogLog.log("Calibration/Error", "Rotation error: " + String.format("%.1f", returnError) +
            "°. " + recommendation.replace("\n", " "));
        SmartDashboard.putString("Calibration Error",
            "Rotation error " + String.format("%.1f", returnError) + "° - see Cal/Rotation Recommendation");

        // Print detailed diagnostic to console for pit crew
        System.out.println("\n=== ROTATION TEST FAILED ===");
        System.out.printf("Return Error: %.2f degrees (limit: 5.0 degrees)%n", returnError);
        System.out.printf("CW Rotation: %.2f degrees (expected: %.2f)%n", cwGyroChange, cwExpected);
        System.out.printf("CCW Rotation: %.2f degrees (expected: %.2f)%n", ccwGyroChange, ccwExpected);
        System.out.printf("Asymmetry: %.2f degrees%n", asymmetry);
        System.out.println("\nRECOMMENDED ACTION:");
        System.out.println(recommendation);
        System.out.println("Continuing with remaining tests to collect diagnostic data...");
        System.out.println("=========================\n");
      }

      // Continue to module response test regardless of rotation result
      currentState = CalibrationState.MODULE_RESPONSE_TEST;
      moduleIndex = 0;
      stateTimer.restart();
    }
  }

  private void handleModuleResponseTest() {
    DogLog.log("Calibration/State", "MODULE_RESPONSE_TEST");

    SwerveModule[] modules = swerveSubsystem.getSwerveDrive().getModules();
    String[] moduleNames = {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};

    if (moduleIndex < modules.length) {
      SmartDashboard.putString("Calibration Status",
          "Testing Module " + moduleNames[moduleIndex] + " Response...");

      SwerveModule module = modules[moduleIndex];

      if (stateTimer.get() < 0.4) {
        // Command module to 45 degrees
        module.setAngle(45.0);
      } else if (stateTimer.get() >= 0.7) {
        // Check if module reached 45 degrees (after 0.7s to allow settling)
        double currentAngle = module.getState().angle.getDegrees();
        double targetAngle = 45.0;
        double error = Math.abs(currentAngle - targetAngle);

        // Handle wraparound (e.g., if module is at -315 instead of 45)
        if (error > 180) {
          error = 360 - error;
        }

        // Use 15 degree tolerance - this is a sanity check, not precision test
        // Module should be roughly pointing the right direction
        // Tighter tolerances can be used once PID is well-tuned
        modulePIDResponse[moduleIndex] = error < 15.0;

        // Log warning if error is high but not failing
        if (error >= 8.0 && error < 15.0) {
          DogLog.log("Calibration/Warning", moduleNames[moduleIndex] +
              " PID response slow - " + String.format("%.1f", error) + "° error. Consider tuning PID.");
        }
        DogLog.log("Calibration/" + moduleNames[moduleIndex] + "/ResponseTarget", targetAngle);
        DogLog.log("Calibration/" + moduleNames[moduleIndex] + "/ResponseActual", currentAngle);
        DogLog.log("Calibration/" + moduleNames[moduleIndex] + "/ResponseError", error);
        DogLog.log("Calibration/" + moduleNames[moduleIndex] + "/ResponseSuccess",
            modulePIDResponse[moduleIndex]);

        SmartDashboard.putNumber("Cal/" + moduleNames[moduleIndex] + " Response Error", error);
        SmartDashboard.putBoolean("Cal/" + moduleNames[moduleIndex] + " PID OK", modulePIDResponse[moduleIndex]);

        // Move to next module
        moduleIndex++;
        stateTimer.restart();
      }
    } else {
      // All modules tested
      boolean allModulesRespond = true;
      StringBuilder failedModules = new StringBuilder();
      for (int i = 0; i < modulePIDResponse.length; i++) {
        if (!modulePIDResponse[i]) {
          allModulesRespond = false;
          failedModules.append(moduleNames[i]).append(" ");
        }
      }

      if (allModulesRespond) {
        currentState = CalibrationState.ODOMETRY_CONSISTENCY_TEST;
        stateTimer.restart();
      } else {
        DogLog.log("Calibration/Error", "Module PID response failed: " + failedModules.toString());
        SmartDashboard.putString("Calibration Error", "PID failed: " + failedModules.toString());
        currentState = CalibrationState.FAILED;
      }
    }
  }

  private void handleOdometryConsistencyTest() {
    DogLog.log("Calibration/State", "ODOMETRY_CONSISTENCY_TEST");
    SmartDashboard.putString("Calibration Status", "Checking Odometry Consistency...");

    if (stateTimer.get() < 0.5) {
      // Wait for pose estimator to settle
      return;
    }

    // Check how far odometry thinks robot moved during rotation test
    Pose2d currentPose = swerveSubsystem.getPose();
    Translation2d finalPosition = currentPose.getTranslation();
    double positionDrift = finalPosition.getDistance(odometryDrift);

    // During pure rotation, the robot shouldn't translate
    // Some drift is expected due to wheel slip, but excessive drift indicates:
    // - Wheel wear (different wheel diameters)
    // - Encoder calibration issues
    // - Module alignment problems
    odometryConsistencySuccess = positionDrift < ODOMETRY_POSITION_TOLERANCE;

    DogLog.log("Calibration/Odometry/StartPosition", odometryDrift);
    DogLog.log("Calibration/Odometry/FinalPosition", finalPosition);
    DogLog.log("Calibration/Odometry/Drift", positionDrift);
    DogLog.log("Calibration/Odometry/Success", odometryConsistencySuccess);

    SmartDashboard.putNumber("Cal/Odometry Drift (m)", positionDrift);
    SmartDashboard.putBoolean("Cal/Odometry OK", odometryConsistencySuccess);

    // Analyze individual module travel distances
    // moduleDistances[i] already contains CW distance from handleRotationTestCW()
    // Now add CCW distance to get total
    SwerveModule[] modules = swerveSubsystem.getSwerveDrive().getModules();
    String[] moduleNames = {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};

    double totalDistance = 0;
    for (int i = 0; i < modules.length; i++) {
      double currentPos = modules[i].getDriveMotor().getPosition();
      double ccwDistance = Math.abs(currentPos - ccwEncoderStartPos[i]);
      double totalModuleDistance = moduleDistances[i] + ccwDistance; // CW + CCW
      moduleDistances[i] = totalModuleDistance;
      totalDistance += totalModuleDistance;

      DogLog.log("Calibration/" + moduleNames[i] + "/DistanceTraveled", totalModuleDistance);
      SmartDashboard.putNumber("Cal/" + moduleNames[i] + " Distance", totalModuleDistance);
    }

    // Check for wheel wear by comparing module distances
    // All modules should travel similar distances during rotation
    double avgDistance = totalDistance / 4.0;
    double maxDeviation = 0.0;
    int worstModule = -1;

    for (int i = 0; i < 4; i++) {
      double deviation = Math.abs(moduleDistances[i] - avgDistance);
      if (deviation > maxDeviation) {
        maxDeviation = deviation;
        worstModule = i;
      }
      DogLog.log("Calibration/" + moduleNames[i] + "/DistanceDeviation", deviation);
    }

    // If one module traveled significantly different distance, may indicate:
    // - Wheel wear on that module
    // - Drive encoder issue
    // - Mechanical binding
    if (maxDeviation > 0.05) { // 5cm deviation
      DogLog.log("Calibration/Warning", moduleNames[worstModule] +
          " shows unusual distance: " + String.format("%.3f", maxDeviation) + "m deviation");
      SmartDashboard.putString("Cal/Wheel Wear Warning", moduleNames[worstModule]);
    }

    if (!odometryConsistencySuccess) {
      DogLog.log("Calibration/Warning",
          "Odometry drift high (" + String.format("%.3f", positionDrift) + "m). " +
          "Possible wheel wear or encoder issues.");
    }

    // Calculate wheel diameter correction factor based on rotation test
    // Use gyro as ground truth - the gyro measures rotation accurately
    // If encoders report different distance than expected, the wheel diameter constant is wrong

    // Module positions from Constants.java: wheelBase = wheelTrack = 26.5 inches = 0.673m
    double wheelBaseMeters = 0.673; // 26.5 inches
    double moduleRadiusFromCenter = (wheelBaseMeters / 2.0) * Math.sqrt(2.0); // diagonal distance

    // Total rotation in radians (use gyro as ground truth)
    double totalRotationDegrees = Math.abs(cwGyroChange) + Math.abs(ccwGyroChange);
    double totalRotationRadians = Math.toRadians(totalRotationDegrees);

    // Expected arc length each wheel ACTUALLY traveled (based on gyro)
    double actualArcLength = moduleRadiusFromCenter * totalRotationRadians;

    // Calculate average encoder-reported distance and correction factor
    double totalEncoderDistance = 0;
    int validModules = 0;

    for (int i = 0; i < 4; i++) {
      if (moduleDistances[i] > 0.001) {
        totalEncoderDistance += moduleDistances[i];
        validModules++;
      }
    }

    if (validModules > 0 && actualArcLength > 0.001) {
      double avgEncoderDistance = totalEncoderDistance / validModules;

      // Correction factor: what to multiply the wheel diameter by
      // If encoder reports MORE distance than actual, wheel diameter is too LARGE
      // If encoder reports LESS distance than actual, wheel diameter is too SMALL
      double correctionFactor = actualArcLength / avgEncoderDistance;
      double correctedDiameter = CONFIGURED_WHEEL_DIAMETER_INCHES * correctionFactor;
      double errorPercent = ((correctedDiameter - CONFIGURED_WHEEL_DIAMETER_INCHES)
          / CONFIGURED_WHEEL_DIAMETER_INCHES) * 100.0;

      // Store for each module (relative to average)
      for (int i = 0; i < 4; i++) {
        if (moduleDistances[i] > 0.001) {
          // Individual module correction (to detect uneven wear)
          double moduleCorrectionFactor = actualArcLength / moduleDistances[i];
          measuredWheelDiameters[i] = CONFIGURED_WHEEL_DIAMETER_INCHES * moduleCorrectionFactor;
          wheelWearPercentage[i] = ((measuredWheelDiameters[i] - correctedDiameter)
              / correctedDiameter) * 100.0;

          DogLog.log("Calibration/" + moduleNames[i] + "/CorrectedDiameter", measuredWheelDiameters[i]);
          DogLog.log("Calibration/" + moduleNames[i] + "/WearFromAverage", wheelWearPercentage[i]);

          SmartDashboard.putNumber("Cal/" + moduleNames[i] + " Corrected Diam", measuredWheelDiameters[i]);
          SmartDashboard.putNumber("Cal/" + moduleNames[i] + " Wear From Avg %", wheelWearPercentage[i]);
        } else {
          measuredWheelDiameters[i] = 0.0;
          wheelWearPercentage[i] = 0.0;
        }
      }

      // Log the main correction
      DogLog.log("Calibration/WheelDiameter/ConfiguredDiameter", CONFIGURED_WHEEL_DIAMETER_INCHES);
      DogLog.log("Calibration/WheelDiameter/CorrectedDiameter", correctedDiameter);
      DogLog.log("Calibration/WheelDiameter/CorrectionFactor", correctionFactor);
      DogLog.log("Calibration/WheelDiameter/ErrorPercent", errorPercent);
      DogLog.log("Calibration/WheelDiameter/ActualArcLength", actualArcLength);
      DogLog.log("Calibration/WheelDiameter/AvgEncoderDistance", avgEncoderDistance);

      SmartDashboard.putNumber("Cal/Corrected Wheel Diameter", correctedDiameter);
      SmartDashboard.putNumber("Cal/Diameter Error %", errorPercent);
      SmartDashboard.putNumber("Cal/Diameter Correction Factor", correctionFactor);

      // Provide actionable guidance
      if (Math.abs(errorPercent) > 1.0) {
        String direction = errorPercent > 0 ? "INCREASE" : "DECREASE";
        DogLog.log("Calibration/Warning", String.format(
            "Wheel diameter needs adjustment: %s from %.3f to %.3f inches (%.1f%% %s)",
            direction, CONFIGURED_WHEEL_DIAMETER_INCHES, correctedDiameter,
            Math.abs(errorPercent), direction));
        SmartDashboard.putString("Cal/Diameter Action", String.format(
            "%s to %.3f inches", direction, correctedDiameter));
      } else {
        SmartDashboard.putString("Cal/Diameter Action", "No adjustment needed");
      }
    }

    DogLog.log("Calibration/WheelDiameter/ModuleRadius", moduleRadiusFromCenter);
    DogLog.log("Calibration/WheelDiameter/TotalRotationDeg", totalRotationDegrees);

    // Move to current draw test
    currentState = CalibrationState.MODULE_CURRENT_TEST;
    moduleIndex = 0;
    stateTimer.restart();
  }

  private void handleModuleCurrentTest() {
    DogLog.log("Calibration/State", "MODULE_CURRENT_TEST");
    SmartDashboard.putString("Calibration Status",
        "Testing Module " + (moduleIndex + 1) + " Current Draw...");

    SwerveModule[] modules = swerveSubsystem.getSwerveDrive().getModules();
    String[] moduleNames = {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};

    if (moduleIndex < modules.length) {
      SwerveModule module = modules[moduleIndex];

      if (stateTimer.get() < 0.2) {
        // Reset peak currents
        peakAngleCurrents[moduleIndex] = 0;
        peakDriveCurrents[moduleIndex] = 0;
      } else if (stateTimer.get() < 1.0) {
        // Rotate module back and forth to measure current draw
        double testAngle = 90.0 * Math.sin(stateTimer.get() * 2 * Math.PI);
        module.setAngle(testAngle);

        // Track peak currents - need to get underlying motor objects
        try {
          double angleCurrent = 0;
          double driveCurrent = 0;

          // Get angle motor current (SparkMax)
          Object angleMotorObj = module.getAngleMotor().getMotor();
          if (angleMotorObj instanceof com.revrobotics.spark.SparkMax) {
            angleCurrent = ((com.revrobotics.spark.SparkMax) angleMotorObj).getOutputCurrent();
          }

          // Get drive motor current (TalonFX/Kraken)
          Object driveMotorObj = module.getDriveMotor().getMotor();
          if (driveMotorObj instanceof com.ctre.phoenix6.hardware.TalonFX) {
            driveCurrent = ((com.ctre.phoenix6.hardware.TalonFX) driveMotorObj)
                .getSupplyCurrent().getValueAsDouble();
          }

          peakAngleCurrents[moduleIndex] = Math.max(peakAngleCurrents[moduleIndex], angleCurrent);
          peakDriveCurrents[moduleIndex] = Math.max(peakDriveCurrents[moduleIndex], driveCurrent);

          DogLog.log("Calibration/" + moduleNames[moduleIndex] + "/AngleCurrent", angleCurrent);
          DogLog.log("Calibration/" + moduleNames[moduleIndex] + "/DriveCurrent", driveCurrent);
        } catch (Exception e) {
          // Current monitoring not available on this hardware
          DogLog.log("Calibration/Warning", "Current monitoring not available: " + e.getMessage());
          moduleCurrentNormal[moduleIndex] = true; // Assume OK if we can't measure
        }
      } else {
        // Analyze results for this module
        boolean angleCurrentNormal = peakAngleCurrents[moduleIndex] < MAX_NORMAL_ANGLE_CURRENT;
        boolean driveCurrentNormal = peakDriveCurrents[moduleIndex] < MAX_NORMAL_DRIVE_CURRENT;
        moduleCurrentNormal[moduleIndex] = angleCurrentNormal && driveCurrentNormal;

        DogLog.log("Calibration/" + moduleNames[moduleIndex] + "/PeakAngleCurrent",
            peakAngleCurrents[moduleIndex]);
        DogLog.log("Calibration/" + moduleNames[moduleIndex] + "/PeakDriveCurrent",
            peakDriveCurrents[moduleIndex]);
        DogLog.log("Calibration/" + moduleNames[moduleIndex] + "/CurrentNormal",
            moduleCurrentNormal[moduleIndex]);

        SmartDashboard.putNumber("Cal/" + moduleNames[moduleIndex] + " Peak Angle I",
            peakAngleCurrents[moduleIndex]);
        SmartDashboard.putNumber("Cal/" + moduleNames[moduleIndex] + " Peak Drive I",
            peakDriveCurrents[moduleIndex]);

        if (!angleCurrentNormal) {
          DogLog.log("Calibration/Warning", moduleNames[moduleIndex] +
              " angle motor high current (" + String.format("%.1f", peakAngleCurrents[moduleIndex]) +
              "A). Check for binding or friction.");
        }

        if (!driveCurrentNormal) {
          DogLog.log("Calibration/Warning", moduleNames[moduleIndex] +
              " drive motor high current (" + String.format("%.1f", peakDriveCurrents[moduleIndex]) +
              "A). Check for binding or friction.");
        }

        // Move to next module
        moduleIndex++;
        stateTimer.restart();
      }
    } else {
      // All modules tested - calibration complete
      currentState = CalibrationState.COMPLETE;
    }
  }

  private void updateDashboard() {
    SmartDashboard.putBoolean("Calibration/Gyro OK", gyroCalibrationSuccess);
    SmartDashboard.putBoolean("Calibration/Rotation Test OK", rotationTestSuccess);
    SmartDashboard.putBoolean("Calibration/Odometry OK", odometryConsistencySuccess);

    for (int i = 0; i < 4; i++) {
      SmartDashboard.putBoolean("Calibration/Module " + i + " Angle OK", moduleAngleValid[i]);
      SmartDashboard.putBoolean("Calibration/Module " + i + " PID OK", modulePIDResponse[i]);
      SmartDashboard.putBoolean("Calibration/Module " + i + " Current OK", moduleCurrentNormal[i]);
    }

    // Overall status - warnings don't fail calibration
    boolean criticalTestsPassed = gyroCalibrationSuccess && rotationTestSuccess;
    for (int i = 0; i < 4; i++) {
      criticalTestsPassed = criticalTestsPassed && moduleAngleValid[i] && modulePIDResponse[i];
    }

    // Additional health warnings
    boolean hasWarnings = !odometryConsistencySuccess;
    for (int i = 0; i < 4; i++) {
      if (!moduleCurrentNormal[i]) {
        hasWarnings = true;
      }
    }

    SmartDashboard.putBoolean("Calibration/All Critical Tests Passed", criticalTestsPassed);
    SmartDashboard.putBoolean("Calibration/Has Warnings", hasWarnings);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot
    swerveSubsystem.drive(new ChassisSpeeds(0, 0, 0));

    // Check if all critical tests passed
    boolean allCriticalPassed = gyroCalibrationSuccess && rotationTestSuccess;
    for (int i = 0; i < 4; i++) {
      allCriticalPassed = allCriticalPassed && moduleAngleValid[i] && modulePIDResponse[i];
    }

    if (interrupted) {
      SmartDashboard.putString("Calibration Status", "Interrupted!");
      DogLog.log("Calibration/Status", "Calibration Interrupted");
    } else if (currentState == CalibrationState.COMPLETE && allCriticalPassed) {
      SmartDashboard.putString("Calibration Status", "Complete - All Systems OK!");
      SmartDashboard.putBoolean("Calibration Complete", true);
      DogLog.log("Calibration/Status", "Calibration Complete - Success");

      // Center modules for storage
      SwerveModule[] modules = swerveSubsystem.getSwerveDrive().getModules();
      for (SwerveModule module : modules) {
        module.setAngle(0.0);
      }
    } else if (currentState == CalibrationState.COMPLETE && !allCriticalPassed) {
      // We completed all tests but some critical tests failed
      SmartDashboard.putString("Calibration Status", "FAILED - Check Diagnostics!");
      SmartDashboard.putBoolean("Calibration Complete", false);
      DogLog.log("Calibration/Status", "Calibration Failed - See diagnostic data");

      // Provide detailed failure information
      printDiagnosticSummary();

      // Center modules for storage
      SwerveModule[] modules = swerveSubsystem.getSwerveDrive().getModules();
      for (SwerveModule module : modules) {
        module.setAngle(0.0);
      }
    } else if (currentState == CalibrationState.FAILED) {
      SmartDashboard.putString("Calibration Status", "FAILED - Check Logs!");
      DogLog.log("Calibration/Status", "Calibration Failed");

      // Provide detailed failure information
      printDiagnosticSummary();
    }
  }

  /**
   * Prints a diagnostic summary when calibration fails to help with troubleshooting
   */
  private void printDiagnosticSummary() {
    System.out.println("=== CALIBRATION FAILURE DIAGNOSTIC ===");

    // Module encoder diagnostics
    String[] moduleNames = {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};
    SwerveModule[] modules = swerveSubsystem.getSwerveDrive().getModules();

    System.out.println("\nModule Encoder Status:");
    for (int i = 0; i < modules.length; i++) {
      double rawPos = modules[i].getAbsolutePosition();
      System.out.printf("  %s (Module %d):%n", moduleNames[i], i);
      System.out.printf("    Raw Position: %.2f degrees%n", rawPos);
      System.out.printf("    Encoder Valid: %s%n", moduleAngleValid[i] ? "YES" : "NO");
      System.out.printf("    Suggested Offset: %.2f%n", -rawPos);

      if (!moduleAngleValid[i]) {
        System.out.println("    >>> ISSUE DETECTED <<<");
        if (Double.isNaN(rawPos)) {
          System.out.println("    Possible cause: Encoder not connected or bad wiring");
        } else if (Math.abs(rawPos) > 360) {
          System.out.println("    Possible cause: Encoder offset incorrect or sensor malfunction");
        }
      }
    }

    // Gyro diagnostics
    if (!gyroCalibrationSuccess) {
      System.out.println("\nGyro Status: FAILED");
      System.out.println("  Possible causes:");
      System.out.println("    - Robot was not stationary during calibration");
      System.out.println("    - Gyro hardware issue");
      System.out.println("    - Excessive vibration or movement");
    }

    // Rotation test diagnostics
    if (!rotationTestSuccess && currentState != CalibrationState.MODULE_ANGLE_CHECK) {
      System.out.println("\nRotation Test: FAILED");
      System.out.printf("  Expected return error: < 5.0 degrees%n");
      System.out.printf("  Actual return error: %.2f degrees%n", Math.abs(actualGyroChange));
      System.out.println("  Possible causes:");
      System.out.println("    - Wheel slippage during rotation");
      System.out.println("    - Gyro drift or calibration issue");
      System.out.println("    - Mechanical binding or uneven wheel wear");
    }

    // Odometry consistency diagnostics
    System.out.println("\nOdometry Consistency:");
    if (odometryDrift != null) {
      Pose2d currentPose = swerveSubsystem.getPose();
      double drift = currentPose.getTranslation().getDistance(odometryDrift);
      System.out.printf("  Position Drift: %.3f meters%n", drift);
      System.out.printf("  Status: %s%n", odometryConsistencySuccess ? "PASS" : "WARNING");

      if (!odometryConsistencySuccess) {
        System.out.println("  >>> High odometry drift detected!");
        System.out.println("  Possible causes:");
        System.out.println("    - Uneven wheel wear (check wheel diameters)");
        System.out.println("    - Drive encoder calibration issues");
        System.out.println("    - Module alignment problems");
      }
    }

    // Module distance analysis
    // String[] moduleNames = {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};
    System.out.println("\nModule Distance Analysis (Wheel Wear Check):");
    double totalDist = 0;
    for (int i = 0; i < 4; i++) {
      totalDist += moduleDistances[i];
    }
    double avgDist = totalDist / 4.0;

    for (int i = 0; i < 4; i++) {
      double deviation = moduleDistances[i] - avgDist;
      double percentDev = (deviation / avgDist) * 100.0;
      System.out.printf("  %s: %.3fm (%.1f%% deviation)%n",
          moduleNames[i], moduleDistances[i], percentDev);

      if (Math.abs(percentDev) > 10.0) {
        System.out.println("    >>> SIGNIFICANT DEVIATION! Check wheel wear or encoder.");
      }
    }

    // Wheel diameter measurement
    System.out.println("\nWheel Diameter Measurement:");
    System.out.printf("  Configured: %.3f inches%n", CONFIGURED_WHEEL_DIAMETER_INCHES);
    boolean anyWheelWear = false;
    for (int i = 0; i < 4; i++) {
      if (measuredWheelDiameters[i] > 0.001) {
        System.out.printf("  %s: %.3f inches (%.1f%% wear)%n",
            moduleNames[i], measuredWheelDiameters[i], wheelWearPercentage[i]);

        if (Math.abs(wheelWearPercentage[i]) > WHEEL_WEAR_WARNING_THRESHOLD * 100.0) {
          String status = wheelWearPercentage[i] > 0 ? "WORN" : "OVERSIZED";
          System.out.printf("    >>> %s! Consider replacing wheel%n", status);
          anyWheelWear = true;
        }
      } else {
        System.out.printf("  %s: No data%n", moduleNames[i]);
      }
    }

    if (anyWheelWear) {
      System.out.println("\n  WHEEL WEAR DETECTED:");
      System.out.println("  - Replace worn wheels before competition");
      System.out.println("  - Update wheel diameter constant in physicalproperties.json if needed");
      System.out.println("  - Ensure all wheels are same diameter for best odometry accuracy");
    }

    // Current draw diagnostics
    System.out.println("\nMotor Current Draw Analysis:");
    boolean anyCurrentIssues = false;
    for (int i = 0; i < 4; i++) {
      System.out.printf("  %s:%n", moduleNames[i]);
      System.out.printf("    Angle Motor: %.1fA (max %.1fA)%n",
          peakAngleCurrents[i], MAX_NORMAL_ANGLE_CURRENT);
      System.out.printf("    Drive Motor: %.1fA (max %.1fA)%n",
          peakDriveCurrents[i], MAX_NORMAL_DRIVE_CURRENT);

      if (peakAngleCurrents[i] > MAX_NORMAL_ANGLE_CURRENT) {
        System.out.println("    >>> HIGH ANGLE CURRENT! Check for binding/friction");
        anyCurrentIssues = true;
      }
      if (peakDriveCurrents[i] > MAX_NORMAL_DRIVE_CURRENT) {
        System.out.println("    >>> HIGH DRIVE CURRENT! Check for binding/friction");
        anyCurrentIssues = true;
      }
    }

    if (anyCurrentIssues) {
      System.out.println("\n  MECHANICAL ISSUES DETECTED:");
      System.out.println("  - Inspect modules for binding or debris");
      System.out.println("  - Check belt tension");
      System.out.println("  - Verify module alignment");
      System.out.println("  - Lubricate bearings if necessary");
    }

    System.out.println("\n=== END DIAGNOSTIC ===");
  }

  @Override
  public boolean isFinished() {
    return currentState == CalibrationState.COMPLETE || currentState == CalibrationState.FAILED;
  }
}
