package frc.robot.commands.AutoAlign;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AlignXCommand extends Command {
    private final Timer timer = new Timer();
    private final double targetOffset;

    public AlignXCommand(double targetOffsetInches) {
        this.targetOffset = targetOffsetInches;
        addRequirements(RobotContainer.drivetrain, RobotContainer.visionSystem);
    }

    @Override
    public void initialize() {
        double currentXOffset = RobotContainer.visionSystem.getX() * 39.37; // convert limelight meters to inches
        double distToMove = targetOffset - currentXOffset; // get error offset
        double rotations = RobotContainer.drivetrain.lineartoRotations(distToMove); // convert linear distance to motor rotations

        if (Math.abs(rotations) < 5) { 
            // min of 5 rotations (pos/neg)
            rotations = Math.copySign(5, rotations);
        }

        RobotContainer.drivetrain.resetAllEncoders();
        RobotContainer.drivetrain.setAllMotorsPosition(rotations, rotations);

        timer.reset();
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(1.5); // align x for a max of 1.5 seconds
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.drivetrain.stopMotor();
    }
}
