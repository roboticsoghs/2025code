package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoCommand extends Command {
    private boolean finished = false;
    private boolean detectedTag = false;
    private boolean linedUp = false;
    private boolean movedBack = false;
    private double lineUpWithAprilTag = 0;
    double finalPosition = 0;
    private double position = 0;

    public AutoCommand() {
        addRequirements(RobotContainer.drivetrain, RobotContainer.visionSystem);
        RobotContainer.drivetrain.resetAllEncoders();
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("auto pos: ", position);
        SmartDashboard.putBoolean("auto finished", finished);

        if (RobotContainer.visionSystem.isApriltag()) {
            SmartDashboard.putBoolean("AprilTag Seen", true);
            detectedTag = true;

            // Get offsets
            double offSetX = RobotContainer.visionSystem.getX();
            double offSetY = RobotContainer.visionSystem.getY();

            // Stop the robot when the tag is detected
            RobotContainer.drivetrain.stopMotor();

            // Calculate horizontal distance to tag
            double horizontalDistance = Constants.offSetHeight * Math.tan(Math.toRadians(offSetY - 8));
            lineUpWithAprilTag = position + RobotContainer.drivetrain.lineartoRotations(horizontalDistance / Math.tan(Math.toRadians(offSetX)));

            // Move to line up with the AprilTag
            if (!linedUp) {
                SmartDashboard.putNumber("Aligning Distance", lineUpWithAprilTag);
                position = lineUpWithAprilTag;
                finalPosition = position;
                RobotContainer.drivetrain.setAllMotorsPosition(
                    // lineUpWithAprilTag,
                    // lineUpWithAprilTag
                    4,4
                );
                linedUp = true;
                // initialEncoderPosition = RobotContainer.drivetrain.getleftFrontEncoder(); // Record encoder position
            }

        } else {
            SmartDashboard.putBoolean("AprilTag Seen", false);

            if (!detectedTag) {
                // Keep driving forward until it sees a tag
                RobotContainer.drivetrain.setAllMotorsPosition(position - 1, position - 1);
                position--;
            }
        }

        // Move back 2 feet after aligning
        if (linedUp && !movedBack) {
            double targetDistance = finalPosition + RobotContainer.drivetrain.lineartoRotations(24); // 24 inches (2 feet)
            SmartDashboard.putNumber("Target Dist: ", targetDistance);
            double currentPosition = Math.abs(RobotContainer.drivetrain.getleftFrontEncoder());

            // if (currentPosition <= targetDistance) {
            //     RobotContainer.drivetrain.stopMotor();
            //     movedBack = true;
            //     finished = true;
            // } else {
                // RobotContainer.drivetrain.setAllMotorsPosition(targetDistance, targetDistance);
            // }
        }
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.drivetrain.setAllMotorsSpeed(0, 0); // Stop motors
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
