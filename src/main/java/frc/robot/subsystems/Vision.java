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
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Vision extends SubsystemBase {
  private double x;
  private double y;
  private double z;
  private double pitch;
  private double yaw;
  private double roll;
  private long aprilTagId;
  private double[] camera = new double[6];

  private int alignState;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry cameraPose = table.getEntry("targetpose_cameraspace");

  public Vision() {
    System.out.println("We are cooked!");
    aprilTagId = 0;
    alignState = -1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //read values periodically
    double[] defaultValue = {0,0,0,0,0,0}; // default value required by getDoubleArray
    camera = cameraPose.getDoubleArray(defaultValue);
    long defaultValueID = 0;
    aprilTagId = table.getEntry("tid").getInteger(defaultValueID);

    // limelight 3D offsets relative to camera
    x = camera[0]; // in meters
    y = camera[1]; // in meters
    z = camera[2]; // distance in meters
    pitch = camera[3];
    yaw = camera[4];
    roll = camera[5];

    // angleX = ca

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightZ", z);

    SmartDashboard.putNumber("Limelight Pitch", pitch);
    SmartDashboard.putNumber("Limelight Yaw", yaw);
    SmartDashboard.putNumber("Limelight Roll", roll);

    SmartDashboard.putNumber("AprilTag ID", aprilTagId);
    // output if the limelight sees an AprilTag
    SmartDashboard.putBoolean("AprilTag Seen", isApriltag());

    SmartDashboard.putNumber("align state", alignState);
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
   * @return The x offset of the AprilTag relative to the Limelight (in meters)
   */
  public double getX() {
    return x;
  }

  /**
   *
   * @return The z offset of the AprilTag relative to the Limelight (in meters)
   */
  public double getZ() {
    return z;
  }

  public double getPitch() {
    return pitch;
  }

  public double getYaw() {
    return yaw;
  }

  public double getRoll() {
    return roll;
  }

  public double getId() {
    return aprilTagId;
  }

  // align robot to reef (angle is in degrees)
  public void makeParallel(double angleDegrees, boolean isAuto) {
    if (!isAuto) isAuto = false;
    // if (Math.abs(angleDegrees) < 1) return;

    final double degreesToRotations = 0.16; // magic constant

    // convert degrees to motor rotations
    double wheelRotations = angleDegrees * degreesToRotations;

    // clamp rotations between -6 and 6
    wheelRotations = Math.max(-6.0, Math.min(6.0, wheelRotations));

    if (Math.abs(wheelRotations) < 4 && !isAuto) {
      // minimum of 4 rotations (except during auton)
      wheelRotations = Math.copySign(4, wheelRotations);
    }

    SmartDashboard.putNumber("parallel rotations", wheelRotations);

    RobotContainer.drivetrain.resetAllEncoders();
    RobotContainer.drivetrain.setRightSideMotorsPosition(-wheelRotations);
    RobotContainer.drivetrain.setLeftSideMotorsPosition(wheelRotations);

    alignState = 1;
  }

  /**
   * Auto aligns the robot to the left reef pole
   */
  public void alignToReef(double offset) {
    if (!isApriltag()) return;

    alignState = 0;
    if (alignState == 0) {
      double yawVal = getYaw();
      makeParallel(yawVal, false);
    }
    
    if (alignState == 1) {
      double x = getX();
      double currentXOffset = x * 39.37;
      double distToMove = offset - currentXOffset;

      double rotations = RobotContainer.drivetrain.lineartoRotations(distToMove);

      if (Math.abs(rotations) < 5) {
        rotations = Math.copySign(5, rotations);
      }

      SmartDashboard.putNumber("align rotations", rotations);

      RobotContainer.drivetrain.resetAllEncoders();
      RobotContainer.drivetrain.setAllMotorsPosition(rotations, rotations);

      alignState = 2;
    }

    if (alignState == 2) {
      double zVal = getZ();
      adjustElevatorHeight(zVal);

      alignState = 0;
    }
  }

  /**
   * Adjust elevator height based on distance from the reef.
   * zVal is the distance to the detected AprilTag in meters
   */
  private void adjustElevatorHeight(double zVal) {
    // in meters
    final double idealDist = 0.30;
    final double maxDistError = 0.20;
    final double maxElevatorOffset = 4.0;

    double error = zVal - idealDist;
    error = Math.max(-maxDistError, Math.min(maxDistError, error));

    double normalizedError = error / maxDistError;
    double rotations = (normalizedError * maxElevatorOffset);

    RobotContainer.elevator.moveRotation(rotations);

    SmartDashboard.putNumber("elevator rotations", rotations);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
