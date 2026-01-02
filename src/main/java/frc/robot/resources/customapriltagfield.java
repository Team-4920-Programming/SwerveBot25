// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.resources;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class customapriltagfield {
    ArrayList<AprilTag> atag = new ArrayList<AprilTag>();
    
    
    public customapriltagfield(){

    atag.add(new AprilTag(18, new Pose3d(16.46-Units.inchesToMeters(3.25), 8.052 - Units.inchesToMeters(5.25), Units.inchesToMeters(30.0), new Rotation3d(0, 0, Units.degreesToRadians(-90)))));
    atag.add(new AprilTag(21, new Pose3d(16.46, 8.052-Units.inchesToMeters(116.5), Units.inchesToMeters(19.5), new Rotation3d(0, 0, Units.degreesToRadians(-180)))));
    atag.add(new AprilTag(22, new Pose3d(16.46-Units.inchesToMeters(98 + 49), 8.052, Units.inchesToMeters(32), new Rotation3d(0, 0, 0))));
    atag.add(new AprilTag(12, new Pose3d(16.46-Units.inchesToMeters(113.25), 8.052 - Units.inchesToMeters(79.5 + 116.5), Units.inchesToMeters(32), new Rotation3d(0, 0, Units.degreesToRadians(90)))));
    atag.add(new AprilTag(1, new Pose3d(16.46, 8.052 - Units.inchesToMeters(116.5 - 36.25), Units.inchesToMeters(11), new Rotation3d(0, 0, Units.degreesToRadians(-180)))));  
}

    public ArrayList<AprilTag> getTags(){
        return atag;
    }
}
