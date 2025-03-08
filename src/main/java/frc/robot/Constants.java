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
    public static final int SUCKER = 15;
    public static final int WRIST = 10;
    public static final int ELBOW = 11;
    public static final int EXTENDER = 12;

    // Climber: consists of rotator and extender
    public static final int ANCHOR1 = 13;
    public static final int ANCHOR2 = 14;
  }

  // Crane States
  public enum CraneState {
    CLIMB,
    STOW,
    STATION,
    SHELF,
    LOW_REEF,
    MID_REEF,
    HIGH_REEF
  }

  public static final class CraneConstants{
    // Arm Feedforward constants
    // public static double kSVolts = 0.0; // Volts
    public static final double kGVolts = 1.74; // Volts
    public static final double kVVoltSecsPerDeg = 2.44 * Math.PI / 180.0; // V*s/rad to V*s/deg
    public static final double kAVoltSecsSquaredPerDeg = 0.16 * Math.PI / 180.0; // V*s^2/rad to V*s^2/deg
    public static final double kDefaultErrorMargin = 1.0; // degrees

    // Encoder distance per pulse (gear ratio * unit of revolution, 360 deg or 2pi rad)
    public static final double kGripperEncoderDistancePerPulse = 360.0 * 1/5 * 1/4 * 1/3; // tested 2/15/2025
    public static final double kWristEncoderDistancePerPulse = 360.0 * 1/64 * 1/3; // tested 2/15/2025
    public static final double kExtenderEncoderDistancePerPulse = 360.0 * 1/4 * 1/3 * 1/3; // tested 2/15/2025
    public static final double kElbowEncoderDistancePerPulse = 360.0 * 1/5 * 1/5 * 1/5; // tested 2/15/2025
    public static final double kSuckerEncoderDistancePerPulse = 360.0 * 1/3; // tested 2/15/2025
    
    // Gripper limits
    public static final double kGripperHardDeck = 0;
    public static final double kGripperCeiling = 40;
    
    // Extender limits
    // Extender setpoints are measured with 2 inch soft limit included
    public static final double kExtenderOffset = 3.5;
    public static final double kExtenderHardDeck = 1;
    public static final double kExtenderStation = 5.16;
    public static final double kExtenderHigh = 26.044;
    public static final double kExtenderMid = 9.44;
    public static final double kExtenderLow = 3.46;
    // public static final double kExtenderPickup = 12;
    public static final double kExtenderStart = 21; // starts retracted by 4 inches from maximum
    public static final double kExtenderShelf = 5.16;
    public static final double kExtenderCeiling = 26.5; // starting + tail end offset - 2 inch margin
    public static final double kExtensionCap = 17; // 17 inches
    // public static final double kExtenderClawOffset = ; // measured from edge of Claw to the soft limit of the extender
    public static final double kPullyCircumferenceInches = 2.25 * Math.PI; // 2.25 inches diameter
    /** Keep extender butt-side within extension cap. */
    public static final double kExtenderLimit1 = kExtenderLow; // 2 inches from soft limit offset
    // /** Keep extender claw-side within extension cap. */
    // public static final double kExtenderLimit2 = kExtensionCap - kExtenderClawOffset - 2; // 2 inches from soft limit offset

    // Elbow limits
    public static final double kElbowPlatformOffset = -0; // degrees
    public static final double kElbowHorizonOffset = -51 - kElbowPlatformOffset; // measured from horizontal position to resting angle
    public static final double kElbowHardDeck = 0;
    public static final double kElbowShelf = 32;
    public static final double kElbowStation = 32 + kElbowPlatformOffset;
    public static final double kElbowHigh = 136.104 + kElbowPlatformOffset;
    public static final double kElbowClimb = 131 + kElbowPlatformOffset;
    public static final double kElbowMid = 105.033 + kElbowPlatformOffset;
    /** This is for low reef, not ground/low pickup. */
    public static final double kElbowLow = 73.571 + kElbowPlatformOffset;
    // public static final double kElbowCeiling = 290;
    public static final double kElbowCeiling = kElbowClimb + 10;

    // Wrist limits
    public static final double kWristDiffFromOldSlack = 14; // Old slack is 20 degrees, now it is 5 degrees, only affects front values
    public static final double kWristHigh = -113.64; // negative number because referencing the angle matching shows hard deck is vertical, so we need this value to bypass that angle matching
    public static final double kWristHardDeck = 0;
    public static final double kWristStation = 16;
    public static final double kWristShelf = 90;
    public static final double kWristLow = 106;
    public static final double kWristMid = 97.817;
    public static final double kWristCeiling = kWristMid + kElbowMid + 10; // This is absolute max adding the angle match from elbow at mid and the reference for mid from wrist.

    // Sucker limits
    public static final double kSuckerIntake = 6;
    public static final double kSuckerEject = -12;
    public static final double kSuckerHold = 0;
  }

  public static final class ClimberConstants {
    public static final double kRotatorEncoderDistancePerPulse = 360.0 * 1/5 * 1/3 * 1/3 * 1/3;
    // public static final double kAnchorEncoderDistancePerPulse = 1.0/4.0/8.0;
    public static final double kAnchorEncoderDistancePerPulse = 1.0/5.0 * (3.0/4.0)/8.0;

    // Rotator limits
    public static final double kRotatorHardDeck = -10;
    public static final double kRotatorCeiling = 135;

    // Anchor limits
    public static final double kAnchorHardDeck = 0.25;

    public static final double kAnchorCeiling = 8; // inches
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
    public static final double XY_DEADBAND = 0.05;
    public static final double TRIGGER_DEADBAND = 0.2;
  }

  public static final class DriveConstants {
    // Snap constants for reef
    // Angles are relative to the DPAD (CW +), will be inverted in snap method to drive robot (CCW +)
    public static final double SNAP_UP_RIGHT = 60;
    public static final double SNAP_DOWN_RIGHT = 120;
    public static final double SNAP_DOWN_LEFT = -120;
    public static final double SNAP_UP_LEFT = -60;

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
}