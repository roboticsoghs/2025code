package frc.robot.commands.AutoAlign;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AdjustElevatorCommand extends Command {
    private final Timer timer = new Timer();

    public AdjustElevatorCommand() {
        addRequirements(RobotContainer.elevator, RobotContainer.visionSystem);
    }

    @Override
    public void initialize() {
        double zVal = RobotContainer.visionSystem.getZ();
        final double idealDist = 0.25; // 25 cm
        final double maxDistError = 0.15; // max error in cm
        final double maxElevatorOffset = 4.0; // max amount of rotations

        double error = zVal - idealDist;
        error = Math.max(-maxDistError, Math.min(maxDistError, error)); // clamp error to prevent over correction

        double normalizedError = error / maxDistError; // prevents really big error offsets
        double rotations = (normalizedError * maxElevatorOffset);

        RobotContainer.elevator.moveRotation(rotations);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.7);
    }
}
