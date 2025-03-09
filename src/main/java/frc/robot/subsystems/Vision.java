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
    double[] defaultValue = {0,0,0,0,0,0}; // default value require by getDoubleArray
    camera = cameraPose.getDoubleArray(defaultValue);

    // limelight 3D offsets relative to camera
    x = camera[0];
    y = camera[1];
    z = camera[2];

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightZ", z);

    // output if the limelight sees an AprilTag
    SmartDashboard.putBoolean("AprilTag Seen", isApriltag());
  }

  /**
   * 
   * @return Whether the Limelight detects an AprilTag
   */
  public boolean isApriltag() {
    return x != 0 && z != 0;
  }

  /**
   * 
   * @return The x offset of the AprilTag relative to the Limelight
   */
  public double getX() {
    return x;
  }

  /**
   * 
   * @return The z offset of the AprilTag relative to the Limelight
   */
  public double getZ() {
    return z;
  }

  /**
   * Auto aligns the robot to the left reef pole
   */
  public void alignLeftSide() {
    if(isApriltag()) {
      double distToMove = Constants.leftAlignReef - (getX() / 39.37);
      double rotations = Math.round(RobotContainer.drivetrain.lineartoRotations(distToMove) * 100.0) / 100.0;
      RobotContainer.drivetrain.resetAllEncoders();
      RobotContainer.drivetrain.setAllMotorsPosition(RobotContainer.drivetrain.getEncoderVal() + rotations, RobotContainer.drivetrain.getEncoderVal() + rotations);
    }
  }

  /**
   * Auto aligns the robot to the right reef pole
   */
  public void alignRightSide() {
    if(isApriltag()) {
      double distToMove = Constants.rightAlignReef - (getX() / 39.37);
      double rotations = Math.round(RobotContainer.drivetrain.lineartoRotations(distToMove) * 100.0) / 100.0;
      RobotContainer.drivetrain.resetAllEncoders();
      SmartDashboard.putNumber("rotations to move", rotations);
      RobotContainer.drivetrain.setAllMotorsPosition(rotations, rotations);
    }
  }

  /**
   * Auto aligns the robot between the reef poles
   */
  public void alignCenterSide() {
    if(isApriltag()) {
      double distToMove = Constants.reefCenter - (getX() / 39.37);
      double rotations = Math.round(RobotContainer.drivetrain.lineartoRotations(distToMove) * 100.0) / 100.0;
      RobotContainer.drivetrain.resetAllEncoders();
      RobotContainer.drivetrain.setAllMotorsPosition(RobotContainer.drivetrain.getEncoderVal() + rotations, RobotContainer.drivetrain.getEncoderVal() + rotations);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
