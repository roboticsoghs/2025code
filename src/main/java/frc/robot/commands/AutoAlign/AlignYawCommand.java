package frc.robot.commands.AutoAlign;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AlignYawCommand extends Command {
    private final Timer timer = new Timer();

    public AlignYawCommand() {
        addRequirements(RobotContainer.drivetrain, RobotContainer.visionSystem);
    }

    @Override
    public void initialize() {
        double yaw = RobotContainer.visionSystem.getYaw();
        RobotContainer.visionSystem.makeParallel(yaw, false);
        timer.reset();
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.7); // max rotation time of 700ms
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.drivetrain.stopMotor();
    }
}
