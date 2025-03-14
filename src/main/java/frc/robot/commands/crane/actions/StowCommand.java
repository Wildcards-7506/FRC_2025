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
            Commands.runOnce(() -> Robot.led.enableStreamer = false),
            //Simultaneously move elbow, extender, and wrist to the appropriate setpoints
            new ParallelCommandGroup(
                new SetWristCommand(CraneConstants.kWristHardDeck),
                new SetExtenderCommand(CraneConstants.kExtenderLimit1),
                new SetElbowCommand(CraneConstants.kElbowHardDeck)),
            new SetExtenderCommand(CraneConstants.kExtenderStow),
            //prevent the commands from being scheduled more than once
            Commands.runOnce(() -> Robot.crane.runSetpoint = false),
            Commands.runOnce(() -> Robot.led.streamerBrightness = 0),
            Commands.runOnce(() -> Robot.led.enableStreamer = true)
        );
    }
}