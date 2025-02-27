package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CraneConstants;
import frc.robot.Constants.CraneState;
import frc.robot.Robot;
import frc.robot.players.PlayerConfigs;

public class CraneTeleopCommand extends Command {
    private boolean prevStationPickupState = false,
                    // prevLowPickupState = false,
                    prevShelfReefState = false,
                    prevlowReefState = false,
                    prevMidReefState = false,
                    prevHighReefState = false;
    
    /** Degree of angleMargin so that the crane can progress to the next position. */
    private double angleMargin = 8;
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
        // EXECUTE CRANE TELEOP ONLY WHEN CLIMBER CONTROL IS OFF
        if(Robot.climber.onClimberControl) {
            putCraneToClimbState();
            return;
        }

        // if(PlayerConfigs.gripperOpen)
        //     Robot.crane.setGripperPosition(90);
        // else
        //     Robot.crane.setGripperPosition(0);

        updateCraneState();

        /** Sucker */
        if (PlayerConfigs.suckerIntake) {
            Robot.crane.spinSucker(-4); // volts
        } else if (PlayerConfigs.suckerEject) {
            Robot.crane.spinSucker(4); // volts
        } else {
            Robot.crane.holdSucker();
        }
        
        if(PlayerConfigs.fineControlCraneEnable) { // fine control
            Robot.crane.setWristPosition(Robot.crane.wristSetpoint + PlayerConfigs.fineControlWrist * 0.1);
            Robot.crane.setElbowPosition(Robot.crane.elbowSetpoint + PlayerConfigs.fineControlElbow * 0.1);
            Robot.crane.setExtenderPosition(Robot.crane.extenderSetpoint + PlayerConfigs.fineControlExtender * 0.05);
        } else {
            // If crane state is updated to positive val, then crane won't go to climb state
            if(Robot.crane.craneState == CraneState.CLIMB) { // climb
                putCraneToClimbState();
            }
            if(Robot.crane.craneState == CraneState.STATION) { // station
                // Robot.crane.setWristPosition(CraneConstants.kWristHardDeck);
                Robot.crane.setWristPosition(CraneConstants.kWristStation);
                if(downToElbowPosition(CraneConstants.kElbowStation, CraneConstants.kExtenderLimit1)
                    && upToElbowPosition(CraneConstants.kElbowStation, CraneConstants.kExtenderLimit1)) {
                    Robot.crane.setWristPosition(CraneConstants.kWristStation);
                    Robot.crane.setElbowPosition(CraneConstants.kElbowStation);
                    Robot.crane.setExtenderPosition(CraneConstants.kExtenderStation);
                }
            }
            if(Robot.crane.craneState == CraneState.SHELF) { // shelf reef
                // Robot.crane.setWristPosition(CraneConstants.kWristHardDeck);
                Robot.crane.setWristPosition(CraneConstants.kWristShelf);
                if(downToElbowPosition(CraneConstants.kElbowShelf, CraneConstants.kExtenderLimit1)
                    && upToElbowPosition(CraneConstants.kElbowStation, CraneConstants.kExtenderLimit1)) {
                    Robot.crane.setWristPosition(CraneConstants.kWristShelf);
                    Robot.crane.setElbowPosition(CraneConstants.kElbowStation);
                    Robot.crane.setExtenderPosition(CraneConstants.kExtenderStation);
                }
            }
            if(Robot.crane.craneState == CraneState.LOW_REEF) { // low reef
                // Robot.crane.setWristPosition(CraneConstants.kWristHardDeck);
                Robot.crane.setWristPosition(CraneConstants.kWristLow);
                if(upToElbowPosition(CraneConstants.kElbowLow, CraneConstants.kExtenderLimit1)
                    && downToElbowPosition(CraneConstants.kElbowLow, CraneConstants.kExtenderLimit1)) {
                    Robot.crane.setWristPosition(CraneConstants.kWristLow);
                    Robot.crane.setElbowPosition(CraneConstants.kElbowLow);
                    Robot.crane.setExtenderPosition(CraneConstants.kExtenderLow);
                }
            }
            if(Robot.crane.craneState == CraneState.MID_REEF) { // mid 
                // Robot.crane.setWristPosition(CraneConstants.kWristHardDeck);
                Robot.crane.setWristPosition(CraneConstants.kWristMid);
                if(upToElbowPosition(CraneConstants.kElbowMid, CraneConstants.kExtenderLimit1)
                    && downToElbowPosition(CraneConstants.kElbowMid, CraneConstants.kExtenderLimit1)) {
                    Robot.crane.setWristPosition(CraneConstants.kWristMid);
                    Robot.crane.setElbowPosition(CraneConstants.kElbowMid);
                    Robot.crane.setExtenderPosition(CraneConstants.kExtenderMid);
                }
            }
            if(Robot.crane.craneState == CraneState.HIGH_REEF) { // high reef
                Robot.crane.setWristPosition(CraneConstants.kWristHardDeck);
                if(upToElbowPosition(CraneConstants.kElbowHigh, CraneConstants.kExtenderLimit1)
                    && downToElbowPosition(CraneConstants.kElbowHigh, CraneConstants.kExtenderLimit1)) {
                    Robot.crane.setWristPosition(CraneConstants.kWristHigh);
                    Robot.crane.setElbowPosition(CraneConstants.kElbowHigh);
                    Robot.crane.setExtenderPosition(CraneConstants.kExtenderHigh);
                }
            }
            // if(Robot.crane.craneState == CraneState.STOW) { // stow
            //     // If we want to go to elbow pause, we must retract extender
            //     Robot.crane.setWristPosition(CraneConstants.kWristHardDeck);
            //     if(downToElbowPosition(CraneConstants.kElbowHardDeck, CraneConstants.kExtenderLimit1)) {
            //         // If elbow is at hard deck, then we slightly retract the extender for stability
            //         Robot.crane.setWristPosition(CraneConstants.kWristHardDeck);
            //         Robot.crane.setElbowPosition(CraneConstants.kElbowHardDeck);
            //         Robot.crane.setExtenderPosition(CraneConstants.kExtenderStart);
            //     }
            // }
        }

        SmartDashboard.putString("Crane State", Robot.crane.craneState.toString());
        SmartDashboard.putBoolean("Crane FC", PlayerConfigs.fineControlCraneEnable);
        SmartDashboard.putNumber("FC Elbow", PlayerConfigs.fineControlElbow);
        SmartDashboard.putNumber("FC Wrist", PlayerConfigs.fineControlWrist);
        SmartDashboard.putNumber("FC Extender", PlayerConfigs.fineControlExtender);
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
     * Puts the crane to climb state and sets the craneState to CLIMB to keep hold.
     */
    private void putCraneToClimbState() {
        // This sets the crane to the climb state only if this method is called
        Robot.crane.craneState = CraneState.CLIMB; // climb state
        Robot.crane.setWristPosition(CraneConstants.kWristHardDeck);
        if(upToElbowPosition(CraneConstants.kElbowHigh, CraneConstants.kExtenderLimit1)
           && downToElbowPosition(CraneConstants.kElbowHigh, CraneConstants.kExtenderLimit1)) {
            Robot.crane.setElbowPosition(CraneConstants.kElbowHigh);
            Robot.crane.setExtenderPosition(CraneConstants.kExtenderHardDeck);
        }
        
        SmartDashboard.putString("Crane State", Robot.crane.craneState.toString());
        SmartDashboard.putBoolean("Crane FC", PlayerConfigs.fineControlCraneEnable);
        SmartDashboard.putNumber("FC Elbow", PlayerConfigs.fineControlElbow);
        SmartDashboard.putNumber("FC Wrist", PlayerConfigs.fineControlWrist);
        SmartDashboard.putNumber("FC Extender", PlayerConfigs.fineControlExtender);
    }
    
    /**
     * Updates the crane state based on the button pressed. Holds the state of
     * the crane until more buttons are pressed.
     */
    private void updateCraneState() {
        prevStationPickupState = updateButtonState(PlayerConfigs.stationPickup, prevStationPickupState, CraneState.STATION);
        // prevLowPickupState = updateButtonState(PlayerConfigs.lowPickup, prevLowPickupState, );
        prevShelfReefState = updateButtonState(PlayerConfigs.shelfReef, prevShelfReefState, CraneState.SHELF);
        prevlowReefState = updateButtonState(PlayerConfigs.lowReef, prevlowReefState, CraneState.LOW_REEF);
        prevMidReefState = updateButtonState(PlayerConfigs.midReef, prevMidReefState, CraneState.MID_REEF);
        prevHighReefState = updateButtonState(PlayerConfigs.highReef, prevHighReefState, CraneState.HIGH_REEF);
    }

    /**
     * Updates the button state based on the button pressed.
     * @param buttonPressed The button pressed.
     * @param prevState The previous state of the button.
     * @param craneState The desired state of the crane.
     * @return The updated state of the button.
     */
    private boolean updateButtonState(boolean buttonPressed, boolean prevState, CraneState craneState) {
        if(buttonPressed && !prevState) {
            if(Robot.crane.craneState != craneState) {
                Robot.crane.craneState = craneState;
            } else {
                // Robot.crane.craneState = CraneState.STOW;
                Robot.crane.craneState = CraneState.STATION;
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
