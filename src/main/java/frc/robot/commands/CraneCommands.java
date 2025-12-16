package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Constants.CraneConstants;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.Crane.CraneState;

public class CraneCommands{
    /* This class uses the methods created in Crane.java to construct sequences of commands that
     * can execute complex movements. In the case of the crane, this file creates movement sequences
     * for each level of the reef. This allows the controller to be bound to individual movements that can be
     * commanded at any time, rather than relying on getting to a specific point in a large if statement.
    */

    private Crane crane;
    private Timer boomRotatorTimer = new Timer();
    private Timer wristRotatorTimer = new Timer();
    private Timer extendTimer = new Timer();

    public CraneCommands(Crane crane) {
        this.crane = crane;
    }

    public Command stowCommand = new SequentialCommandGroup(
        craneMovementCommand(CraneState.STOW_PREP),
        craneMovementCommand(CraneState.STOW)
    );
    public Command stationCommand = craneMovementCommand(CraneState.STATION);
    public Command shelfCommand = craneMovementCommand(CraneState.SHELF);
    public Command lowCommand = craneMovementCommand(CraneState.LOW);
    public Command midCommand = craneMovementCommand(CraneState.MID);
    public Command highCommand = new SequentialCommandGroup(
        craneMovementCommand(CraneState.HIGH_PREP),
        craneMovementCommand(CraneState.HIGH)
    );
    public Command algaeHighCommand = craneMovementCommand(CraneState.ALGAE_HIGH);
    public Command algaeLowCommand = craneMovementCommand(CraneState.ALGAE_LOW);

    public Command climbPrepCommand = new SequentialCommandGroup(
        craneMovementCommand(CraneState.CLIMB),
        Commands.runOnce(() -> crane.neutralExtend())
    );

    public Command craneMovementCommand(CraneState state){
        return new ParallelCommandGroup(
            setWristRotatorCommand(state.wristAngle),
            setExtenderCommand(state.extension),
            setBoomRotatorCommand(state.wristAngle)
        ).andThen(Commands.runOnce(() -> Robot.robotContainer.led.solid((int)state.boomAngle,255,255)));
    }

    private Command setBoomRotatorCommand(double setPoint){
        return Commands.runOnce(() -> {
            boomRotatorTimer.reset();
            boomRotatorTimer.start();
        })
        .andThen(Commands.runOnce(() -> crane.setBoomRotatorPosition(setPoint)))
        .until(() -> Math.abs(crane.getBoomPosition() - setPoint) < CraneConstants.rotationMargin || 
            boomRotatorTimer.get() > 0.5 && Math.abs(crane.getBoomVelocity()) < 0.01
        );
    }

    private Command setExtenderCommand(double setPoint) {
        return Commands.runOnce(() -> {
            extendTimer.reset();
            extendTimer.start();
        })
        .andThen(Commands.runOnce(() -> crane.setExtenderPosition(setPoint)))
        .until(() -> 
            (Math.abs(crane.getExtensionPosition() - setPoint) < CraneConstants.extendMargin  || 
            extendTimer.get() > 0.5 && Math.abs(crane.getExtensionVelocity()) < 0.01)
        );
    }

    private Command setWristRotatorCommand(double setPoint) {
        return Commands.runOnce(() -> {
            wristRotatorTimer.reset();
            wristRotatorTimer.start();
        })
        .andThen(Commands.runOnce(() -> crane.setWristRotatorPosition(setPoint)))
        .until(() -> 
            (Math.abs(crane.getWristPosition() - setPoint) < CraneConstants.rotationMargin  || 
            wristRotatorTimer.get() > 0.5 && Math.abs(crane.getWristVelocity()) < 0.01)
        );
    }


}
