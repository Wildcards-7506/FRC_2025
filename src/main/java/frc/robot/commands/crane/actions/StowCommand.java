package frc.robot.commands.crane.actions;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CraneConstants;
import frc.robot.Robot;
import frc.robot.commands.crane.SetElbowCommand;
import frc.robot.commands.crane.SetExtenderCommand;
import frc.robot.commands.crane.SetWristCommand;

public class StowCommand extends SequentialCommandGroup {
    public StowCommand() {
        addRequirements(Robot.crane);
        addCommands(
            new ParallelCommandGroup(
                new SetWristCommand(CraneConstants.kWristHardDeck),
                new SetExtenderCommand(CraneConstants.kExtenderLimit1),
                new SetElbowCommand(CraneConstants.kElbowHardDeck)),
            new SetExtenderCommand(CraneConstants.kExtenderStow),
            Commands.runOnce(() -> Robot.crane.runSetpoint = false)
        );
    }
}