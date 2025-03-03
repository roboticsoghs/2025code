// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Vision extends SubsystemBase {
    private double x;
    private double y;
    private double z;
    private double[] camera = new double[6];

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry cameraPose = table.getEntry("targetpose_cameraspace");

  /** Creates a new ExampleSubsystem. */
  public Vision() {
    System.out.println("We are cooked!");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //read values periodically
    // camera = cameraPose.getDoubleArray({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    double[] defaultValue = {0,0,0,0,0,0};
    camera = cameraPose.getDoubleArray(defaultValue);
    x = camera[0];
    // Ignore y and area
    y = camera[1];
    z = camera[2];

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightZ", z);
  }

  public boolean isApriltag() {
    return x != 0 && z != 0;
  }

  public double getX() {
    return x;
  }

  public double getZ() {
    return z;
  }

  public void alignLeftSide() {
    double distToMove = Constants.leftAlignReef - (x * 39.37);
    double rotations = RobotContainer.drivetrain.lineartoRotations(distToMove);

    RobotContainer.drivetrain.setAllMotorsPosition(RobotContainer.drivetrain.getEncoderVal() + rotations, RobotContainer.drivetrain.getEncoderVal() + rotations);
  }

  public void alignRightSide() {
    double distToMove = Constants.rightAlignReef - (x * 39.37);
    double rotations = RobotContainer.drivetrain.lineartoRotations(distToMove);

    RobotContainer.drivetrain.setAllMotorsPosition(RobotContainer.drivetrain.getEncoderVal() + rotations, RobotContainer.drivetrain.getEncoderVal() + rotations);
  }

  public void alignCenterSide() {
    double distToMove = Constants.reefCenter + (x * 39.37);
    double rotations = RobotContainer.drivetrain.lineartoRotations(distToMove);

    RobotContainer.drivetrain.setAllMotorsPosition(RobotContainer.drivetrain.getEncoderVal() + rotations, RobotContainer.drivetrain.getEncoderVal() + rotations);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
