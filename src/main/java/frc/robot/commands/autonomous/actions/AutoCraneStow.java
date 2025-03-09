package frc.robot.commands.autonomous.actions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CraneState;
import frc.robot.commands.autonomous.commands.GoToCraneState;

public class AutoCraneStow extends SequentialCommandGroup {
    public AutoCraneStow() {
        addCommands(
            new GoToCraneState(CraneState.STOW)
        );
    }
}