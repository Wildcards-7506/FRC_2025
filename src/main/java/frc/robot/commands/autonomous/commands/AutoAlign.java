// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
  /** Creates a new AutoAlign Command. */
  /*This is your linear, translation, and rotational tuning constants. They currently works for a roughly 55 pound robot with a lower CoG and wider base than you guys.
   *Tune accordingly so you hit your target reliably and don't overshoot.
   *Note the negative sign on the rotational kP. If your robot is rotating the wrong way, flip that negative sign.
  */
  double kP = 5;
  double rotationalKP = -0.3;

  //If true, the robot auto aligns to the left pole on the face of the reef.
  boolean left = true;
  int framesDropped = 0;
  Translation3d translate;
  double angle = 0;
  int ID = 0;

  Drivetrain drive;

  public AutoAlign(Drivetrain drive, boolean left) {
    this.left = left;
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("isGoingLeft?", left);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if we drop a frame, do nothing for this periodic, unless we've dropped 6 or more frames, in
    // which case we end the command
    if (LimelightHelpers.getTV("")) {
      framesDropped = 0;
    } else {
      framesDropped++;
      return;
    }

    //Get the target AprilTag ID, convert into a heading angle.
    this.ID = (int) LimelightHelpers.getFiducialID("");
    switch (ID) {
        case 6:
            angle = 120;
            break;
        case 7:
            angle = 180;
            break;
        case 8:
            angle = -120;
            break;
        case 9:
            angle = -60;
            break;
        case 10:
            angle = 0;
            break;
        case 11:
            angle = 60;
            break;
        case 17:
            angle = 60;
            break;
        case 18:
            angle = 0;
            break;
        case 19:
            angle = -60;
            break;
        case 20:
            angle = -120;
            break;
        case 21:
            angle = 180;
            break;
        case 22:
            angle = 120;
            break;
    }

    //Get the location of the target relative to the center of the robot
    double[] tagPoseRobot;
    tagPoseRobot = LimelightHelpers.getTargetPose_RobotSpace("");

    // converts the double array into Pose3d so we can use the values
    Pose3d pose =
        new Pose3d(
            new Translation3d(tagPoseRobot[0], tagPoseRobot[1], tagPoseRobot[2]),
            new Rotation3d(
                Math.toRadians(tagPoseRobot[3]),
                Math.toRadians(tagPoseRobot[4]),
                Math.toRadians(tagPoseRobot[5])));

    // Get the location we want to go to relative to the april tag
    // Rotate and translate that pose so it is relative to the robot, an x, y, theta vector for the robot to move to.

    //set this offset so the robot lines up centered on the pole to the right or left. Same with the -0.5 in line 76.
    double offset = left ? -0.2 : 0.2;
    translate = new Translation3d(offset, 0, -0.5);

    translate = translate.rotateBy(pose.getRotation());
    translate = translate.plus(pose.getTranslation());

    double maxVelocity = 0.5; // This will move very slowly to start. Maximum value here is around 6 meters/second
    double xDriveSpeed = Math.max(-maxVelocity, Math.min(maxVelocity, kP * translate.getZ()));
    SmartDashboard.putNumber("xDriveSpeed", xDriveSpeed);
    double yDriveSpeed = Math.max(-maxVelocity, Math.min(maxVelocity, kP * -translate.getX()));
    SmartDashboard.putNumber("yDriveSpeed", yDriveSpeed);

    drive.drive(xDriveSpeed, yDriveSpeed, (drive.getHeading() - angle) * rotationalKP, false);
    SmartDashboard.putNumber("Rotational error", drive.getHeading() - angle);;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((drive.getHeading() - angle) < 3
            && Math.sqrt(Math.pow(translate.getZ(), 2) + Math.pow(translate.getX(), 2)) < 0.1)
        || (framesDropped > 5);
  }
}