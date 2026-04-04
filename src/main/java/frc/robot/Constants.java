// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation3d;

//import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
//import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.units.measure.*;
//import frc.robot.commons.TagUtils;
import frc.robot.generated.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {


  public static class IOConstants {
    public static final int kDriverControllerPortUSB = 0;
    public static final int kManipulatorControllerPortUSB = 1;
    public static final int kDriverControllerPortBT = 2;
    public static final int kManipulatorControllerPortBT = 3;
  }


  public static class DriveConstants {
    public static final LinearVelocity kMaxSpeed = TunerConstants.kSpeedAt12Volts;
    public static final AngularVelocity kMaxAngularRate = RotationsPerSecond.of(1);
    public static final LinearAcceleration kMaxAcceleration = kMaxSpeed.times(2.0).per(Second);
    public static final AngularAcceleration kMaxAngularAcceleration = kMaxAngularRate.times(8.0).per(Second);

    public static final double kDriveDeadband = 0.05;
    public static final double kRotationDeadband = 0.05;

    public static final double kLoopPeriodSeconds = 0.02;

    public static final double kDriveSlowModifier = 0.25;
    public static final double kDriveFastModifier = 1.0;
    public static final double kDriveNormalModifier = 0.5;
    public static final double kTurnSlowModifier = 0.5;
    public static final double kTurnFastModifier = 1.0;
    public static final double kTurnNormalModifier = 0.5;
  }

  public final class FieldObjects {
    public static final String ROBOT = "RobotPose";
    public static final String LIMELIGHT = "LimelightPose";
    public static final String QUEST = "QuestPose";
  }
  
//   public static class FieldConstants {
//     public static final Distance FIELD_LENGTH = Inches.of(650.12);
//     public static final Distance FIELD_WIDTH = Inches.of(316.64);
//     public static final Distance ALLIANCE_ZONE = Inches.of(156.06);
//     public static final Translation3d HUB_BLUE = new Translation3d(Inches.of(181.56), FIELD_WIDTH.div(2), Inches.of(56.4));
//     public static final Translation3d HUB_RED =new Translation3d(FIELD_LENGTH.minus(Inches.of(181.56)), FIELD_WIDTH.div(2), Inches.of(56.4));
//     public static final Distance FUNNEL_RADIUS = Inches.of(24);
//     public static final Distance FUNNEL_HEIGHT = Inches.of(72 - 56.4);
// }
  
  // Set to true to enable tunable numbers in NetworkTables under /Tuning/
  public static boolean tuningMode = true;
  // When true, avoid using hardware-dependent WPILib/robot APIs (useful for unit tests)
  public static boolean disableHAL = false;

}