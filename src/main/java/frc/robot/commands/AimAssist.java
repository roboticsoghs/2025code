package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoAlign.AdjustElevatorCommand;
import frc.robot.commands.AutoAlign.AlignXCommand;
import frc.robot.commands.AutoAlign.AlignYawCommand;

public class AimAssist {
    /**
     * Runs "Toaster AimAssist 9000"
     * with total runtime of 3.1 seconds
     * 
     * @param xOffset
     * @return
     */
    public static Command run(double xOffset) {
        return Commands.sequence(
            new AlignYawCommand(),
            Commands.waitSeconds(1),
            new AlignXCommand(xOffset),
            Commands.waitSeconds(1),
            new AdjustElevatorCommand()
        );
    }
}
