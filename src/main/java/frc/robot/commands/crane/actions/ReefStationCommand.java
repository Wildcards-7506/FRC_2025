package frc.robot.commands.crane.actions;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.crane.SetElbowCommand;
import frc.robot.commands.crane.SetExtenderCommand;
import frc.robot.commands.crane.SetWristCommand;

public class ReefStationCommand extends SequentialCommandGroup {
    public ReefStationCommand(double elbowSetpoint, double extenderSetpoint, double wristSetpoint, int hue) {
        addRequirements(Robot.crane);
        addCommands(
            //Simultaneously move elbow, extender, and wrist to the appropriate setpoints
            new ParallelCommandGroup(
                new SetElbowCommand(elbowSetpoint),
                new SetExtenderCommand(extenderSetpoint),
                new SetWristCommand(wristSetpoint)
            ),
            //prevent the commands from being scheduled more than once
            Commands.runOnce(() -> Robot.crane.runSetpoint = false)
        );
    }
}