// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import java.util.Optional;

import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Vision4920;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToTargetV0_1 extends Command {
  /** Creates a new DriveToTargetV0_1. */
  SwerveSubsystem swerve;
  AprilTagFieldLayout apriltags;
  Pose3d target;
  public DriveToTargetV0_1(SwerveSubsystem SwerveDrive) {
    swerve = SwerveDrive;
    apriltags = Vision4920.kTagLayout;
    addRequirements(swerve);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Optional<Pose3d> targetAT =  apriltags.getTagPose(1);
    target = targetAT.get().transformBy(new Transform3d(2.0, -0.5, 0, new Rotation3d(0, 0, Units.degreesToRadians(180))));//210
    DogLog.log("TargetPoseD2TV0_1/targetPose",target);
    DogLog.log("TargetPoseD2TV0_1/targetATPose",targetAT.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        
    swerve.driveToPose(target.toPose2d()).schedule();    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new ChassisSpeeds(0,0,0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return Math.abs(swerve.getPose().getX() - target.getX()) <= 0.01 && Math.abs(swerve.getPose().getY() - target.getY()) <= 0.01 && Math.abs(swerve.getPose().getRotation().minus(target.toPose2d().getRotation()).getRadians()) <= 3*Math.PI/180;
  }
}
