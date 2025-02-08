package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CraneConstants;
import frc.robot.Robot;
import frc.robot.players.PlayerConfigs;

public class CraneTeleopCommand extends Command {
    private boolean prevStationPickupState = false,
                    prevLowPickupState = false,
                    prevShelfReefState = false,
                    prevlowReefState = false,
                    prevMidReefState = false,
                    prevHighReefState = false;
    
    /** Degree of angleMargin so that the crane can progress to the next position. */
    private double angleMargin = 6;
    private double extensionMargin = 0.5;

    public CraneTeleopCommand() {
        addRequirements(Robot.crane);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time (~20 ms) the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(PlayerConfigs.gripperOpen)
            Robot.crane.setGripperPosition(90);
        else
            Robot.crane.setGripperPosition(0);

        updateCraneState();
        
        if(PlayerConfigs.fineControlEnable) { // fine control
            Robot.crane.setWristPosition(Robot.crane.wristSetpoint + PlayerConfigs.fineControlWrist * 0.1);
            Robot.crane.setElbowPosition(Robot.crane.elbowSetpoint + PlayerConfigs.fineControlElbow * 0.1);
            Robot.crane.setExtenderPosition(Robot.crane.extenderSetpoint + PlayerConfigs.fineControlExtender * 0.05);
            // Robot.crane.setGripperPosition(Robot.crane.gripperSetpoint + PlayerConfigs.fineControlExtender * 0.1);
        } else {
            if(Robot.crane.craneState == 1) { // station
                if(downToElbowPosition(CraneConstants.kElbowPause, CraneConstants.kExtenderLimit2)) {
                    if(downToElbowPosition(CraneConstants.kElbowStation, CraneConstants.kExtenderLimit1)
                       && upToElbowPosition(CraneConstants.kElbowStation, CraneConstants.kExtenderLimit1)) {
                        Robot.crane.setWristPosition(CraneConstants.kWristVertical);
                        Robot.crane.setElbowPosition(CraneConstants.kElbowStation);
                        Robot.crane.setExtenderPosition(CraneConstants.kExtenderStation);
                    }
                }
            }
            if(Robot.crane.craneState == 2) { // low pickup
                if(upToElbowPosition(CraneConstants.kElbowPause, CraneConstants.kExtenderLimit1)) {
                    Robot.crane.setWristPosition(CraneConstants.kWristOrigin);
                    if(upToElbowPosition(CraneConstants.kElbowCeiling, CraneConstants.kExtenderLimit2)) {
                        // If elbow is at ceiling, then we extend the extender to pick up the coral
                        Robot.crane.setWristPosition(CraneConstants.kWristOrigin); // may need to rotate 180 degrees
                        Robot.crane.setElbowPosition(CraneConstants.kElbowCeiling);
                        Robot.crane.setExtenderPosition(CraneConstants.kExtenderPickup);
                    }
                }
            }
            if(Robot.crane.craneState == 3) { // shelf reef
                if(downToElbowPosition(CraneConstants.kElbowPause, CraneConstants.kExtenderLimit2)) {
                    Robot.crane.setWristPosition(CraneConstants.kWristOrigin);
                    if(upToElbowPosition(CraneConstants.kElbowShelf, CraneConstants.kExtenderLimit1)
                       && downToElbowPosition(CraneConstants.kElbowShelf, CraneConstants.kExtenderLimit1)) {
                        Robot.crane.setWristPosition(CraneConstants.kWristOrigin);
                        Robot.crane.setElbowPosition(CraneConstants.kElbowShelf);
                        Robot.crane.setExtenderPosition(CraneConstants.kExtenderShelf);
                    }
                }
            }
            if(Robot.crane.craneState == 4) { // low reef
                if(downToElbowPosition(CraneConstants.kElbowPause, CraneConstants.kExtenderLimit2)) {
                    if(upToElbowPosition(CraneConstants.kElbowLow, CraneConstants.kExtenderLimit1)
                       && downToElbowPosition(CraneConstants.kElbowLow, CraneConstants.kExtenderLimit1)) {
                        Robot.crane.setWristPosition(CraneConstants.kWristVertical);
                        Robot.crane.setElbowPosition(CraneConstants.kElbowLow);
                        Robot.crane.setExtenderPosition(CraneConstants.kExtenderLow);
                    }
                }
            }
            if(Robot.crane.craneState == 5) { // mid reef
                if(downToElbowPosition(CraneConstants.kElbowPause, CraneConstants.kExtenderLimit2)) {
                    if(upToElbowPosition(CraneConstants.kElbowMid, CraneConstants.kExtenderLimit1)
                       && downToElbowPosition(CraneConstants.kElbowMid, CraneConstants.kExtenderLimit1)) {
                        Robot.crane.setWristPosition(CraneConstants.kWristVertical);
                        Robot.crane.setElbowPosition(CraneConstants.kElbowMid);
                        Robot.crane.setExtenderPosition(CraneConstants.kExtenderMid);
                    }
                }
            }
            if(Robot.crane.craneState == 6) { // high reef
                if(upToElbowPosition(CraneConstants.kElbowHigh, CraneConstants.kExtenderLimit1)) {
                    if(downToElbowPosition(CraneConstants.kElbowPause, CraneConstants.kExtenderLimit2)) {
                        if(downToElbowPosition(CraneConstants.kElbowHigh, CraneConstants.kExtenderLimit2 + 10)) {
                            Robot.crane.setWristPosition(CraneConstants.kWristVertical);
                            Robot.crane.setElbowPosition(CraneConstants.kElbowHigh);
                            Robot.crane.setExtenderPosition(CraneConstants.kExtenderHigh);
                        }
                    }
                }
            }
            if(Robot.crane.craneState == 0) { // stow
                // If we want to go to elbow pause, we must retract extender, then we rotate elbow to pause position
                if(downToElbowPosition(CraneConstants.kElbowPause, CraneConstants.kExtenderLimit2)) {
                    Robot.crane.setWristPosition(CraneConstants.kWristOrigin);
                    if(downToElbowPosition(CraneConstants.kElbowHardDeck, CraneConstants.kExtenderLimit1)) {
                        // If elbow is at hard deck, then we slightly retract the extender for stability
                        Robot.crane.setWristPosition(CraneConstants.kWristOrigin);
                        Robot.crane.setElbowPosition(CraneConstants.kElbowHardDeck);
                        Robot.crane.setExtenderPosition(CraneConstants.kExtenderStart);
                    }
                }
            }
        }

        SmartDashboard.putNumber("Crane State", Robot.crane.craneState);
        SmartDashboard.putBoolean("Fine Control", PlayerConfigs.fineControlEnable);
        SmartDashboard.putNumber("FC Elbow", PlayerConfigs.fineControlElbow);
        SmartDashboard.putNumber("FC Wrist", PlayerConfigs.fineControlWrist);
    }

    /**
     * Rotate elbow upwards (CEILING direction) with extender retracted to a limit.
     * 
     * @param elbowPosition The target elbow position in degrees.
     * @param extenderLimit The target extender position in inches.
     * @return True if the elbow is at the desired position.
     */
    private boolean upToElbowPosition(double elbowPosition, double extenderLimit) {
        // If we want to go to elbow position, we must retract extender, then we rotate elbow
        if(Robot.crane.getElbowPosition() + angleMargin < elbowPosition) {
            if(Robot.crane.getExtenderPosition() - extensionMargin > extenderLimit
               || Robot.crane.getExtenderPosition() + extensionMargin < extenderLimit) {
                Robot.crane.setExtenderPosition(extenderLimit);
            } else {
                Robot.crane.setElbowPosition(elbowPosition);
            }
            return false;
        }
        return true;
    }

    /**
     * Rotate elbow upwards (HARD DECK direction) with extender retracted to a limit.
     * 
     * @param elbowPosition The target elbow position in degrees.
     * @param extenderLimit The target extender position in inches.
     * @return True if the elbow is at the desired position.
     */
    private boolean downToElbowPosition(double elbowPosition, double extenderLimit) {
        // If we want to go to elbow position, we must retract extender, then we rotate elbow
        if(Robot.crane.getElbowPosition() - angleMargin > elbowPosition) {
            if(Robot.crane.getExtenderPosition() - extensionMargin > extenderLimit
               || Robot.crane.getExtenderPosition() + extensionMargin < extenderLimit) {
                Robot.crane.setExtenderPosition(extenderLimit);
            } else {
                Robot.crane.setElbowPosition(elbowPosition);
            }
            return false;
        }
        return true;
    }
    
    /**
     * Updates the crane state based on the button pressed. Holds the state of
     * the crane until more buttons are pressed.
     * <ul>
     * <li>Button A is pressed, state = 1, A pressed again, state = 0.</li>
     * <li>Button A is pressed, B is pressed, state = 2.</li>
     * <li>Button A is pressed, B is pressed, then B again, state = 0.</li>
     * <li>Button A is pressed, B is pressed, A is pressed, state = 1.</li>
     * </ul>
     */
    private void updateCraneState() {
        prevStationPickupState = updateButtonState(PlayerConfigs.stationPickup, prevStationPickupState, 1);
        prevLowPickupState = updateButtonState(PlayerConfigs.lowPickup, prevLowPickupState, 2);
        prevShelfReefState = updateButtonState(PlayerConfigs.shelfReef, prevShelfReefState, 3);
        prevlowReefState = updateButtonState(PlayerConfigs.lowReef, prevlowReefState, 4);
        prevMidReefState = updateButtonState(PlayerConfigs.midReef, prevMidReefState, 5);
        prevHighReefState = updateButtonState(PlayerConfigs.highReef, prevHighReefState, 6);
    }

    /**
     * Updates the button state based on the button pressed.
     * @param buttonPressed The button pressed.
     * @param prevState The previous state of the button.
     * @param craneState The desired state of the crane.
     * @return The updated state of the button.
     */
    private boolean updateButtonState(boolean buttonPressed, boolean prevState, int craneState) {
        if(buttonPressed && !prevState) {
            if(Robot.crane.craneState != craneState) {
                Robot.crane.craneState = craneState;
            } else {
                Robot.crane.craneState = 0;
            }
        }
        return buttonPressed;
    }
        
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}