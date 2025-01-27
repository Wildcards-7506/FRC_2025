package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CraneConstants;
import frc.robot.Robot;
import frc.robot.players.PlayerConfigs;

public class CraneTeleopCommand extends Command {
    private boolean prevLowPickupState = false,
                    prevShelfReefState = false,
                    prevStationOrLowReefState = false,
                    prevMidReefState = false,
                    prevHighReefState = false;
    
    /** Degree of offset so that the crane can progress to the next position. */
    private double offset = 2.5;

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
            // Robot.crane.setWristPosition(Robot.crane.wristSetpoint + PlayerConfigs.fineControlWrist * 0.1);
            Robot.crane.setElbowPosition(Robot.crane.elbowSetpoint + PlayerConfigs.fineControlElbow * 0.1);

            // TODO: Remove or comment out after we get positional data
            // Robot.crane.setExtenderPosition(Robot.crane.extenderSetpoint + PlayerConfigs.fineControlElbow * 0.5);
        } else {
            if(Robot.crane.craneState == 1) { // low pickup
                // TODO: CHECK BOUNDS: bumper, claw dimensions, have MARGIN OF ERROR
                // If we want to go to elbow pause, we must retract extender, then we rotate elbow to pause position
                Robot.crane.setWristPosition(0);
                if(Robot.crane.getElbowPosition() + offset < CraneConstants.kElbowPause) {
                    if(Robot.crane.getExtenderPosition() - offset > CraneConstants.kExtenderLimit1
                       || Robot.crane.getExtenderPosition() + offset < CraneConstants.kExtenderLimit1) {
                        Robot.crane.setExtenderPosition(CraneConstants.kExtenderLimit1);
                    } else {
                        Robot.crane.setElbowPosition(CraneConstants.kElbowPause);
                    }
                // If we want to go to elbow ceiling (low pickup), we must retract even more, then we rotate to elbow ceiling
                } else if(Robot.crane.getElbowPosition() + offset < CraneConstants.kElbowCeiling) {
                    if(Robot.crane.getExtenderPosition() - offset > CraneConstants.kExtenderLimit2
                       || Robot.crane.getExtenderPosition() + offset < CraneConstants.kExtenderLimit2) {
                        Robot.crane.setExtenderPosition(CraneConstants.kExtenderLimit2);
                    } else {
                        Robot.crane.setElbowPosition(CraneConstants.kElbowCeiling);
                    }
                } else { // If elbow is at ceiling, then we extend the extender to pick up the coral
                    Robot.crane.setElbowPosition(CraneConstants.kElbowCeiling);
                    Robot.crane.setExtenderPosition(CraneConstants.kExtenderCeiling);
                }
            }
            if(Robot.crane.craneState == 2) { // shelf reef
                // TODO: implement shelf reef logic
                Robot.crane.setWristPosition(0);
            }
            if(Robot.crane.craneState == 3) { // station or low reef
                // TODO: implement station or low reef logic
                Robot.crane.setWristPosition(0);
            }
            if(Robot.crane.craneState == 4) { // mid reef
                // TODO: implement mid reef logic
                Robot.crane.setWristPosition(0);
            }
            if(Robot.crane.craneState == 5) { // high reef
                // TODO: implement high reef logic
                Robot.crane.setWristPosition(0);
            }
            if(Robot.crane.craneState == 0) { // stow
                // TODO: implement stow logic, put an offset to not damage itself
                Robot.crane.setWristPosition(0);
                // If we want to go to elbow pause, we must retract extender, then we rotate elbow to pause position
                if(Robot.crane.getElbowPosition() - offset > CraneConstants.kElbowPause) {
                    if(Robot.crane.getExtenderPosition() - offset > CraneConstants.kExtenderLimit2
                       || Robot.crane.getExtenderPosition() + offset < CraneConstants.kExtenderLimit2) {
                        Robot.crane.setExtenderPosition(CraneConstants.kExtenderLimit2);
                    } else {
                        Robot.crane.setElbowPosition(CraneConstants.kElbowPause);
                    }
                // If we want to go to elbow ceiling (low pickup), we must retract even more, then we rotate to elbow ceiling
                } else if(Robot.crane.getElbowPosition() - offset > CraneConstants.kElbowHardDeck) {
                    if(Robot.crane.getExtenderPosition() - offset > CraneConstants.kExtenderLimit1
                       || Robot.crane.getExtenderPosition() + offset < CraneConstants.kExtenderLimit1) {
                        Robot.crane.setExtenderPosition(CraneConstants.kExtenderLimit1);
                    } else {
                        Robot.crane.setElbowPosition(CraneConstants.kElbowHardDeck);
                    }
                } else { // If elbow is at ceiling, then we extend the extender to pick up the coral
                    Robot.crane.setElbowPosition(CraneConstants.kElbowHardDeck);
                    Robot.crane.setExtenderPosition(CraneConstants.kExtenderCeiling - 360);
                }
            }
            // */
        }

        SmartDashboard.putNumber("Crane State", Robot.crane.craneState);
        SmartDashboard.putBoolean("Fine Control", PlayerConfigs.fineControlEnable);
        SmartDashboard.putNumber("FC Elbow", PlayerConfigs.fineControlElbow);
        SmartDashboard.putNumber("FC Wrist", PlayerConfigs.fineControlWrist);
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
        prevLowPickupState = updateButtonState(PlayerConfigs.lowPickup, prevLowPickupState, 1);
        prevShelfReefState = updateButtonState(PlayerConfigs.shelfReef, prevShelfReefState, 2);
        prevStationOrLowReefState = updateButtonState(PlayerConfigs.stationOrLowReef, prevStationOrLowReefState, 3);
        prevMidReefState = updateButtonState(PlayerConfigs.midReef, prevMidReefState, 4);
        prevHighReefState = updateButtonState(PlayerConfigs.highReef, prevHighReefState, 5);
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