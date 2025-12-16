// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class CANIDS {
    // Drivetrain
    public static final int LEFT_FRONT_TURN = 1;
    public static final int LEFT_FRONT_DRIVE = 2;
    public static final int RIGHT_FRONT_TURN = 3;
    public static final int RIGHT_FRONT_DRIVE = 4;
    public static final int RIGHT_REAR_TURN = 5;
    public static final int RIGHT_REAR_DRIVE = 6;
    public static final int LEFT_REAR_TURN = 7;
    public static final int LEFT_REAR_DRIVE = 8;

    // Crane
    public static final int INTAKE = 15;
    public static final int WRIST = 10;
    public static final int BOOM = 11;
    public static final int EXTENDER = 12;

    // Climber
    public static final int WINCH = 21;
    public static final int PIVOT = 22; 
    public static final int ANCHOR = 25;
  }

  public static final class CraneConstants{
    // Encoder distance per pulse
    // Convert from rotatations of the motor shaft to the desired output unit (Radians, Degrees, Inches for example)
    // Typically in the format Rotations  * U / Gear Ratio
    // where U = 360 degrees, 2pi radians, or Pulley Circumference inches)
    public static final double kWristRotatorEncoderDistancePerPulse = 360.0 / 64.0;
    public static final double kPulleyCircumferenceInches = 2.25 * Math.PI; //The pulley on the extender is 2.25 inches wide -> approx 7 inches per rotation
    public static final double kExtenderEncoderDistancePerPulse = kPulleyCircumferenceInches / 36.0;
    public static final double kBoomRotatorEncoderDistancePerPulse = 360.0 / 125.0;
    
    // Extender limits
    public static final double kExtenderMin = 1;
    public static final double kExtenderMax = 26.5;

    // Boom Rotator limits
    public static final double kBoomRotatorMin = 3.0;
    public static final double kBoomRotatorMax = 143.0;

    // Wrist Rotator limits
    public static final double kWristRotatorMin = 0.0;
    public static final double kWristRotatorMax = 310.0;

    //Margins
    public static final double rotationMargin = 8;
    public static final double extendMargin = 2;
  }

  public static final class ClimberConstants {
    public static final double kWinchEncoderDistancePerPulse = 360.0 / 135.0; // degrees
    public static final double kAnchorEncoderDistancePerPulse = 1.0 / 32.0; // inches

    // Climber limits
    public static final double kAnchorMin = 0.25;
    public static final double kAnchorMax = 6.25; // inches

    // Winch limits
    public static final double kWinchMax = 780; // Bringing the climber out limit in degrees
    public static final double kWinchMin = 0; // Absolute retraction limit
    public static final double kWinchHoldLimit = 300; // Retraction limit to prevent tipping
  }

  public static final class IOConstants {
    //Controller Assignments
    public static final int DRIVER_CONTROLLER_0 = 0;
    public static final int DRIVER_CONTROLLER_1 = 1;
  
    // Thresholds
    public static final double XY_DEADBAND = 0.05;
    public static final double TRIGGER_DEADBAND = 0.2;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    public static final double kTopSpeed = 1.0; // 0 to 1
    public static final double kTopAngularSpeed = 1.0; // 0 to 1

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(20);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(20);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;
    public static final boolean kGyroReversed = false;

    public static final double standardSpeed = 0.5;
    public static final double fineSpeed = 0.3;
    public static final double microSpeed = 0.05;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 12;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}