package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CraneConstants;
import frc.robot.subsystems.Crane;

public class CraneCommands{
    /* This class uses the methods created in Crane.java to construct sequences of commands that
     * can execute complex movements. In the case of the crane, this file creates movement sequences
     * for each level of the reef. This allows the controller to be bound to individual movements that can be
     * commanded at any time, rather than relying on getting to a specific point in a large if statement.
    */

    private Crane crane;
    private Timer elbowTimer = new Timer();
    private Timer wristTimer = new Timer();
    private Timer extendTimer = new Timer();

    public CraneCommands(Crane crane) {
        this.crane = crane;
    }

    public Command stowCommand = new SequentialCommandGroup(
        new ParallelCommandGroup(
            setWristCommand(CraneConstants.kWristMin),
            setExtenderCommand(CraneConstants.kExtenderLimit1),
            setElbowCommand(CraneConstants.kElbowMin + 10)),
        setExtenderCommand(CraneConstants.kExtenderStow),
        setElbowCommand(CraneConstants.kElbowMin)
    );

    public Command stationCommand = new ParallelCommandGroup(
      setElbowCommand(CraneConstants.kElbowStation),
      setExtenderCommand(CraneConstants.kExtenderStation),
      setWristCommand(CraneConstants.kWristStation));

    public Command shelfCommand = new ParallelCommandGroup(
      setElbowCommand(CraneConstants.kElbowShelf),
      setExtenderCommand(CraneConstants.kExtenderShelf),
      setWristCommand(CraneConstants.kWristShelf));

    public Command lowCommand = new ParallelCommandGroup(
        setElbowCommand(CraneConstants.kElbowLow),
        setExtenderCommand(CraneConstants.kExtenderLow),
        setWristCommand(CraneConstants.kWristLow));

    public Command midCommand = new ParallelCommandGroup(
    setElbowCommand(CraneConstants.kElbowMid),
    setExtenderCommand(CraneConstants.kExtenderMid),
    setWristCommand(CraneConstants.kWristMid));

    public Command highCommand = new ParallelCommandGroup(
    setElbowCommand(CraneConstants.kElbowHigh),
    setExtenderCommand(CraneConstants.kExtenderHigh),
    setWristCommand(CraneConstants.kWristHigh));

    public Command algaeHighCommand = new ParallelCommandGroup(
    setElbowCommand(CraneConstants.kElbowAlgaeHigh),
    setExtenderCommand(CraneConstants.kExtenderAlgaeHigh),
    setWristCommand(CraneConstants.kWristAlgaeHigh));

    public Command algaeLowCommand = new ParallelCommandGroup(
    setElbowCommand(CraneConstants.kElbowAlgaeLow),
    setExtenderCommand(CraneConstants.kExtenderAlgaeLow),
    setWristCommand(CraneConstants.kWristAlgaeLow));

    public Command climbPrepCommand = new ParallelCommandGroup(
        setWristCommand(CraneConstants.kWristHigh),
        setExtenderCommand(CraneConstants.kExtenderMin - 0.25),
        setElbowCommand(CraneConstants.kElbowClimb),
        Commands.runOnce(() -> crane.neutralExtend())
    );

    public Command setElbowCommand(double setPoint){
        return Commands.runOnce(() -> {
            elbowTimer.reset();
            elbowTimer.start();
        })
        .andThen(Commands.runOnce(() -> crane.setElbowPosition(setPoint)))
        .until(() -> Math.abs(crane.getElbowPosition() - setPoint) < CraneConstants.rotationMargin || 
            elbowTimer.get() > 0.5 && Math.abs(crane.getElbowVelocity()) < 0.01
        );
    }

    public Command setExtenderCommand(double setPoint) {
        return Commands.runOnce(() -> {
            extendTimer.reset();
            extendTimer.start();
        })
        .andThen(Commands.runOnce(() -> crane.setExtenderPosition(setPoint)))
        .until(() -> 
            (Math.abs(crane.getExtenderPosition() - setPoint) < CraneConstants.extendMargin  || 
            extendTimer.get() > 0.5 && Math.abs(crane.getElbowVelocity()) < 0.01)
        );
    }

    public Command setWristCommand(double setPoint) {
        return Commands.runOnce(() -> {
            wristTimer.reset();
            wristTimer.start();
        })
        .andThen(Commands.runOnce(() -> crane.setWristPosition(setPoint)))
        .until(() -> 
            (Math.abs(crane.getWristPosition() - setPoint) < CraneConstants.rotationMargin  || 
            wristTimer.get() > 0.5 && Math.abs(crane.getElbowVelocity()) < 0.01)
        );
    }


}
