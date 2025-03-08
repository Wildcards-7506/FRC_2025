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
            Robot.crane.putCraneToClimbState();
            return;
        }

        // if(PlayerConfigs.gripperOpen)
        //     Robot.crane.setGripperPosition(90);
        // else
        //     Robot.crane.setGripperPosition(0);

        /** Sucker */
        if (PlayerConfigs.suckerIntake) {
            Robot.crane.spinSucker(CraneConstants.kSuckerIntake); // volts
            if(Robot.crane.getSuckerCurrent() > 20){
                System.out.println("Current High");
                Robot.led.solid(60,255,255);
            } else {
                System.out.println("Current Low");
                Robot.led.solidBlink(30,255,255);
            }
        } else if (PlayerConfigs.suckerEject) {
            Robot.crane.spinSucker(CraneConstants.kSuckerEject); // volts
            Robot.led.solidBlink(0,255,255);
        } else {
            Robot.crane.holdSucker();
        }

        updateCraneState();
        
        if(PlayerConfigs.fineControlCraneEnable) { // fine control
            Robot.crane.setWristPosition(Robot.crane.wristSetpoint + PlayerConfigs.fineControlWrist * 0.3);
            Robot.crane.setElbowPosition(Robot.crane.elbowSetpoint + PlayerConfigs.fineControlElbow * 0.3);
            // Robot.crane.setExtenderPosition(Robot.crane.extenderSetpoint + PlayerConfigs.fineControlExtender * 0.05);
            Robot.led.solidBlink(150,255,255);
        } else {
            Robot.crane.goToCraneState(Robot.crane.craneState);
        }

        SmartDashboard.putString("Crane State", Robot.crane.craneState.toString());
        SmartDashboard.putBoolean("Crane FC", PlayerConfigs.fineControlCraneEnable);
        SmartDashboard.putNumber("FC Elbow", PlayerConfigs.fineControlElbow);
        SmartDashboard.putNumber("FC Wrist", PlayerConfigs.fineControlWrist);
        // SmartDashboard.putNumber("FC Extender", PlayerConfigs.fineControlExtender);
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
                Robot.crane.craneState = CraneState.STOW;
                // Robot.crane.craneState = CraneState.STATION;
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
