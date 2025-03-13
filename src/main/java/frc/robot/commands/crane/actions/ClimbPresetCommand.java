package frc.robot.commands.crane.actions;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CraneConstants;
import frc.robot.Robot;
import frc.robot.commands.crane.SetElbowCommand;
import frc.robot.commands.crane.SetExtenderCommand;
import frc.robot.commands.crane.SetWristCommand;

public class ClimbPresetCommand extends SequentialCommandGroup {
    public ClimbPresetCommand() {
        addRequirements(Robot.crane);
        addCommands(
            //Set climber mode to true, locks out crane movement
            Commands.runOnce(() -> Robot.crane.engageClimbMode()),
            //Simultaneously move elbow, extender, and wrist to appropriate setpoints for climb
            new ParallelCommandGroup(
                new SetWristCommand(CraneConstants.kWristHigh-30),
                new SetExtenderCommand(CraneConstants.kExtenderHardDeck - 0.25),
                new SetElbowCommand(CraneConstants.kElbowClimb)
            )
        );
    }
}