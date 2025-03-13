package frc.robot.commands.crane;

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
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time (~20 ms) the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // EXECUTE CRANE TELEOP ONLY WHEN CLIMBER CONTROL IS OFF
        if(Robot.climber.onClimberControl) {
            if(!Robot.crane.getClimbMode()){
                Robot.climbPresetCommand.schedule();
            }
            Robot.led.rainbow();
            return;
        }

        /** Sucker */
        if (PlayerConfigs.suckerIntake) {
            Robot.crane.spinSucker(CraneConstants.kSuckerIntake); // volts
            if(Robot.crane.getSuckerCurrent() > 20){
                Robot.led.solid(60,255,255);
            } else {
                Robot.led.solidBlink(30,255,255);
            }
        } else if (PlayerConfigs.suckerEject) {
            Robot.crane.spinSucker(CraneConstants.kSuckerEject); // volts
            Robot.led.solidBlink(0,255,255);
        } else {
            Robot.crane.spinSucker(0);;
        }

        updateCraneState();

        if(Robot.crane.runSetpoint){
            if(Robot.crane.craneState == CraneState.STATION){
                Robot.stationCommand.schedule();
            } else if(Robot.crane.craneState == CraneState.SHELF){
                Robot.shelfCommand.schedule();
            } else if(Robot.crane.craneState == CraneState.LOW_REEF){
                Robot.lowCommand.schedule();
            } else if(Robot.crane.craneState == CraneState.MID_REEF){
                Robot.midCommand.schedule();
            } else if(Robot.crane.craneState == CraneState.HIGH_REEF){
                Robot.highCommand.schedule();
            } else {
                Robot.stowCommand.schedule();
            }
        }

        if(PlayerConfigs.fineControlCraneEnable) {
            Robot.fineControlCrane.schedule();
        }
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
            } Robot.crane.runSetpoint = true;
        }
        return buttonPressed;
    }
}
