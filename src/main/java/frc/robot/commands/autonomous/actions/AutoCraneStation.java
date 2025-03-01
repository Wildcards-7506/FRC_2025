package frc.robot.commands.autonomous.actions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CraneState;
import frc.robot.commands.autonomous.subsystem.GoToCraneState;

public class AutoCraneStation extends SequentialCommandGroup {
    public AutoCraneStation() {
        addCommands(
            new GoToCraneState(CraneState.STATION)
        );
    }
}