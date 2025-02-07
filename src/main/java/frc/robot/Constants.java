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
    // SPARK MAX CAN IDs
    public static final int LEFT_FRONT_TURN = 1;
    public static final int LEFT_FRONT_DRIVE = 2;
    public static final int RIGHT_FRONT_TURN = 3;
    public static final int RIGHT_FRONT_DRIVE = 4;
    public static final int RIGHT_REAR_TURN = 5;
    public static final int RIGHT_REAR_DRIVE = 6;
    public static final int LEFT_REAR_TURN = 7;
    public static final int LEFT_REAR_DRIVE = 8;

    // Claw: consists of gripper (grabby grabby) and wrist
    public static final int GRIPPER = 9;
    public static final int WRIST = 10;
    public static final int ELBOW = 11;
    public static final int EXTENDER = 12;
  }

  public static final class CraneConstants{
    // Encoder distance per pulse (gear ratio * unit of revolution, 360 deg or 2pi rad)
    // TODO: Ratio needs to be changed
    public static final double kGripperEncoderDistancePerPulse = 360.0 * 1/5 * 1/4 * 1/3; 
    public static final double kWristEncoderDistancePerPulse = 360.0 * 1/4 * 1/4;
    public static final double kExtenderEncoderDistancePerPulse = 360.0 * 1/4 * 1/3 * 1/3; // Tested as of 1/25/2025
    public static final double kElbowEncoderDistancePerPulse = 360.0 * 1/5 * 1/5 * 1/4; // Tested as of 1/25/2025
    
    // Gripper limits
    public static final double kGripperHardDeck = 0;
    public static final double kGripperCeiling = 50;

    // Wrist limits
    public static final double kWristHardDeck = -45;
    public static final double kWristOrigin = 0;
    public static final double kWristOrigin180 = 180;
    public static final double kWristVertical = 90;
    public static final double kWristCeiling = 225;
    
    // Extender limits
    // Extender setpoints are measured with 2 inch soft limit included
    public static final double kExtenderHardDeck = 0;
    public static final double kExtenderStation = 10;
    public static final double kExtenderHigh = 21;
    public static final double kExtenderMid = 0;
    public static final double kExtenderLow = 0;
    public static final double kExtenderShelf = 0;
    public static final double kExtenderPickup = 12;
    public static final double kExtenderStart = 17; // starts retracted by 4 inches from maximum
    public static final double kExtenderCeiling = kExtenderStart + 4; // starting + tail end offset - 2 inch margin
    public static final double kExtensionCap = 17; // 17 inches
    public static final double kExtenderClawOffset = 15; // measured from edge of Claw to the soft limit of the extender
    public static final double kPullyCircumferenceInches = 2.25 * Math.PI; // 2.25 inches diameter
    // TODO: Verify that measurement from butt of extender to extention limit is 17 inches, if not add offset
    /** Keep extender butt-side within extension cap. */
    public static final double kExtenderLimit1 = kExtenderCeiling - kExtensionCap + 2; // 2 inches from soft limit offset
    /** Keep extender claw-side within extension cap. */
    public static final double kExtenderLimit2 = kExtensionCap - kExtenderClawOffset - 2; // 2 inches from soft limit offset

    // Elbow limits
    public static final double kElbowHardDeck = 10;
    public static final double kElbowStation = 45;
    public static final double kElbowHigh = 145;
    public static final double kElbowPause = 160;
    public static final double kElbowMid = 167;
    /** This is for low reef, not ground/low pickup. */
    public static final double kElbowLow = 200;
    public static final double kElbowShelf = 247.5;
    public static final double kElbowCeiling = 290;
    
    // pull from the .bak
    // public static final double kGripperMaxspeedMPS = 4.8;
    // public static final double kGripperAngularSpeed  =  2 * Math.PI;
    // public static final double kGripperTopSpeed = 1.0;
    // public static final double kGripperTopAngularSpeed = 1.0;

    // public static final double kArmExtensionMaxspeedMPS = 4.8;
    // public static final double kArmRotationAngularSpeed = 2 * Math.PI;
    // public static final double kArmExtensionTopSpeed = 1.0;
    // public static final double kArmRotationTopSpeed = 1.0;
    
    // 2023 constants.java
    // public static final int kRotateCurrentLimit = 30;
    // public static final double kRotateEncoderDistancePerPulse = 1.0/125 * 12.0/15 * 360;
    // public static final double kMotorKP = 0.007;
    // public static final double kMotorSingleSub = 40;
    // public static final double kMotorGroundClear = 15.0;
    // public static final double kMotorGround = -5.0;
    // public static final double kMotorHi = 160.0;
    // public static final double kMotorMid = 165.0;
    // public static final double kMotorDoubleSub = 85.0;
    // public static final double kMotorDoubleCubeOffset = -30;
    // public static final double kMotorSingleCubeOffset = -20;
    // public static final double kMotorVertical = 115.0;
    // public static final double kMotorClosed = 0.0;
    // public static final double MotorHorizontalOffset = 60;
  }

  public static final class ArmConstants {
  }

  public static final class IOConstants {
    //Controller Assignments
    public static final int DRIVER_CONTROLLER_0 = 0;
    public static final int DRIVER_CONTROLLER_1 = 1;
    
    //Control Axes
    public static final int LEFT_STICK_X = 0;
    public static final int LEFT_STICK_Y = 1;
    public static final int RIGHT_STICK_X = 2;
    public static final int RIGHT_STICK_Y = 3;

    //Control D-Pad
    public static final int DPAD_X = 2;
    public static final int DPAD_Y = 3;
    public static final int DPAD_UP = 0;
    public static final int DPAD_UP_RIGHT = 45;
    public static final int DPAD_RIGHT = 90;
    public static final int DPAD_DOWN_RIGHT = 135;
    public static final int DPAD_DOWN = 180;
    public static final int DPAD_DOWN_LEFT = 225;
    public static final int DPAD_LEFT = 270;
    public static final int DPAD_UP_LEFT = 315;

    //Control Buttons
    public static final int BUTTON_A = 2;
    public static final int BUTTON_B = 3;
    public static final int BUTTON_X = 1;
    public static final int BUTTON_Y = 4;
    public static final int LEFT_BUMPER = 5;
    public static final int RIGHT_BUMPER = 6;

    public static final int LEFT_TRIGGER = 7;
    public static final int RIGHT_TRIGGER = 8;

    public static final int BUTTON_BACK = 9;
    public static final int BUTTON_START = 10;
    public static final int LEFT_JOYSTICK_BUTTON = 11;
    public static final int RIGHT_JOYSTICK_BUTTON = 12;

    // Thresholds
    public static final double XY_DEADBAND = 0.1;
    public static final double TRIGGER_DEADBAND = 0.2;
  }

  public static final class DriveConstants {
    // Snap constants for reef
    // Angles are relative to the DPAD (CW +), will be inverted in snap method to drive robot (CCW +)
    public static final double SNAP_UP_LEFT = -30;
    public static final double SNAP_UP_RIGHT = 30;
    public static final double SNAP_DOWN_LEFT = -150;
    public static final double SNAP_DOWN_RIGHT = 150;

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

//   public static final class OIConstants {
//     public static final int kDriverControllerPort = 0;
//     public static final int kOperatorControllerPort = 1;
//     public static final double kDriveDeadband = 0.05;
//   }

//   public static final class AutoConstants {
//     public static final double kMaxSpeedMetersPerSecond = 3;
//     public static final double kMaxAccelerationMetersPerSecondSquared = 3;
//     public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
//     public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

//     public static final double kPXController = 1;
//     public static final double kPYController = 1;
//     public static final double kPThetaController = 1;

//     // Constraint for the motion profiled robot angle controller
//     public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
//         kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
//   }

//   public static final class UnderrollerConstants {
//     public static final int kFrontCanId = 21;
//     public static final int kRearCanId = 22;
//     public static final double kUnderrollerIntakeSpeed = 0.8;
//   }

//   public static final class HendersonConstants {
//     public static final int kLeftLauncherMotorCanId = 1;
//     public static final int kRightLauncherMotorCanId = 2;
//     public static final int kLeftFeederMotorCanId = 3;
//     public static final int kRightFeederMotorCanId = 4;

//     public static final double kFeederGearRatio = 47/11;
//     public static final double kIntakeLauncherSpeed = 0.1;
//     public static final double kIntakeFeederSpeed = 0.3;
//     public static double kLauncherIdleRpm = 1000;
//     public static double kMaxLauncherRpm = 4000;

//     public static double kFeederBackSpeed = 1; // Feeder --> EXIT
//     public static double kFeederFrontSpeed = -0.55; // Feeder --> Launcher
//     public static double kLauncherFrontSpeed = 0.75; // Launcher --> EXIT
//     public static double kLauncherBackSpeed = -0.75; // Launcher --> Feeder
    
//     public static double kLauncherIntakeNoteSpeed = -0.75; // Launcher --> Feeder
//     public static double kLauncherShootNoteBackSpeed = -0.75; // Launcher --> Feeder
//     public static double kLauncherCenteringSpeed = 0.1; // Launcher --> EXIT
//   }

//   public static final class ArmConstants {
//     public static final int kLeftArmMotorCanId = 31;
//     public static final int kRightArmMotorCanId = 32;

//     public static final double kSportGearRatio = 36.0;
//     public static final double kSportPinionTeeth = 10;
//     public static final double kArmSprocketTeeth = 36;
//     public static final double kArmGearRatio = 129.6;

//     // SysID values (in radians and radians/sec)
//     public static final double kSVolts = 1.8; //0.11356;
//     public static final double kGVolts = 0.5; //0.29175;
//     public static final double kVVoltSecondPerRad = 7; //3.65; //1.5928;
//     public static final double kAVoltSecondSquaredPerRad = 0.02;//0.030171;

//     // Set the arm speed and acceleration
//     public static final double kMaxVelocityRadPerSecond = 1.5;
//     public static final double kMaxAccelerationRadPerSecSquared = 2;

//     public static final double kArmOffsetRads = Units.degreesToRadians(-45); //Starting angle
//     public static final double kArmMaxRads = Units.degreesToRadians(90); //Ending angle

//     // public static final double kArmEncoderPositionPIDMinInput = kArmOffsetRads; // radians
//     // public static final double kArmEncoderPositionPIDMaxInput = kArmMaxRads; 


//     public static final double kMaxArmSpeedRpm = 
//       NeoMotorConstants.kFreeSpeedRpm / kArmGearRatio ;
//     public static final double kMaxArmRadiansPerSecond =
//       Units.rotationsPerMinuteToRadiansPerSecond(kMaxArmSpeedRpm);

//     public static final double kMaxArmSpeed = 0.8;
//     //public static final double kArmSlewRate = 2;
//     public static final double kArmDeadband = 0.1;

//     public static final double kArmTestOffsetRads = Units.degreesToRadians(15);
//     public static final double kArmShootingAngleRads = Units.degreesToRadians(57.5); //Amp shooting agngle
//     public static final double kArmFarShootingAngleRads = Units.degreesToRadians(47.5); //Connect to chain
//     public static final double kArmPickupAngleRads = Units.degreesToRadians(-39); //37.5 Normal Pickup & Auto Shoot
//     //public static final double kArmStraightUpAngleRads = Units.degreesToRadians(90 );
//     public static final double kArmDownRads = Units.degreesToRadians(-41); //-40 Jiggle & Trap 
//     public static final double kArmShootingStepsRads = (kArmShootingAngleRads - kArmFarShootingAngleRads) / 3; //20
//     public static final double kArmPickupStepsRads = Units.degreesToRadians(1); //20
//     public static final double kArmFeederAngle = Units.degreesToRadians(-20); //20
//   }

}