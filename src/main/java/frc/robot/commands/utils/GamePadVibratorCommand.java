package frc.robot.commands.utils;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class GamePadVibratorCommand extends SequentialCommandGroup {
    public GamePadVibratorCommand(CommandXboxController controller, double intensity, double durationSecs) {
        addCommands(
                new InstantCommand(() -> controller.getHID().setRumble(RumbleType.kBothRumble, intensity)),
                new WaitCommand(durationSecs), // Correct duration usage
                new InstantCommand(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0))
        );
    }
}
