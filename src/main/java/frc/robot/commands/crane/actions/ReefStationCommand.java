package frc.robot.commands.crane.actions;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CraneConstants;
import frc.robot.Robot;
import frc.robot.commands.crane.SetElbowCommand;
import frc.robot.commands.crane.SetExtenderCommand;
import frc.robot.commands.crane.SetWristCommand;

public class ReefStationCommand extends SequentialCommandGroup {
    public ReefStationCommand(double elbowSetpoint, double extenderSetpoint, double wristSetpoint, int hue) {
        addRequirements(Robot.crane);
        addCommands(
            new ParallelCommandGroup(
                new SetExtenderCommand(CraneConstants.kExtenderLimit1),
                new SetElbowCommand(elbowSetpoint)
            ),
            new ParallelCommandGroup(
                new SetExtenderCommand(extenderSetpoint),
                new SetWristCommand(wristSetpoint)
            ),
            Commands.runOnce(() -> Robot.crane.runSetpoint = false)
        );
    }
}