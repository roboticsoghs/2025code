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
  private double pitch;
  private double yaw;
  private double roll;
  private long aprilTagId;
  private double[] camera = new double[6];

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry cameraPose = table.getEntry("targetpose_cameraspace");

  public Vision() {
    System.out.println("We are cooked!");
    aprilTagId = 0;
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
  public void makeParallel(double angleDegrees) {
    final double degreesToRotations = 0.15; // magic constant

    double wheelRotations = angleDegrees * degreesToRotations;

    wheelRotations = Math.max(-5.0, Math.min(5.0, wheelRotations));

    if (Math.abs(wheelRotations) < 0.3) {
      wheelRotations = Math.copySign(0.3, wheelRotations);
    }

    SmartDashboard.putNumber("parallel rotations", wheelRotations);

    RobotContainer.drivetrain.resetAllEncoders();
    RobotContainer.drivetrain.setRightSideMotorsPosition(-wheelRotations);
    RobotContainer.drivetrain.setLeftSideMotorsPosition(wheelRotations);

    try {
      Thread.sleep(600);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }

  /**
   * Auto aligns the robot to the left reef pole
   */
  public void alignToReef(double offset) {
    if (!isApriltag()) return;

    double yawVal = getYaw();

    makeParallel(yawVal);
    
    double currentXInches = getX() * 39.3701; // convert from meters to inches

    double errorX = offset - currentXInches;

    double rotations = RobotContainer.drivetrain.lineartoRotations(errorX);
    // rotations = Math.round(rotations * 100.0) / 100.0;

    SmartDashboard.putNumber("x offset error (inches)", errorX);
    SmartDashboard.putNumber("align rotations", rotations);

    RobotContainer.drivetrain.resetAllEncoders();
    RobotContainer.drivetrain.setRightSideMotorsPosition(50);
    RobotContainer.drivetrain.setLeftSideMotorsPosition(50);
    SmartDashboard.putNumber("left side pos:", RobotContainer.drivetrain.getEncoderVal());

    // double zVal = getZ();
    // adjustElevatorHeight(zVal);

    // OLD CODE

    // if(isApriltag()) {
    //   // first make the robot parallel to the reef
    //   double yawVal = getYaw();
    //   if(Math.abs(yawVal) > 3) {
    //     makeParallel(yawVal);
    //   }
    //   // calculate distance needed to line up with reef
    //   double distToMove = offset - (getX() / 39.37);
    //   double zVal = getZ();
    //   double rotations = Math.round(RobotContainer.drivetrain.lineartoRotations(distToMove) * 100.0) / 100.0;
    //   RobotContainer.drivetrain.resetAllEncoders();
    //   SmartDashboard.putNumber("rotations to move", rotations);
    //   RobotContainer.drivetrain.resetAllEncoders();
    //   RobotContainer.drivetrain.setAllMotorsPosition(rotations, rotations);
    //   adjustElevatorHeight(zVal);
    // }
  }

  /**
   * Adjust elevator height based on distance from the reef.
   * zVal is the distance to the detected AprilTag in meters
   */
  private void adjustElevatorHeight(double zVal) {
    // in meters
    final double idealDist = 0.10;
    final double maxDistError = 0.20;
    final double maxElevatorOffset = 4.0;

    double error = zVal - idealDist;
    error = Math.max(-maxDistError, Math.min(maxDistError, error));

    double normalizedError = error / maxDistError;
    double rotations = normalizedError * maxElevatorOffset;

    RobotContainer.elevator.moveRotation(rotations);

    SmartDashboard.putNumber("elevator offset error (m)", error);
    SmartDashboard.putNumber("elevator adjust rotations", rotations);

    // OLD CODE

    // if (zVal < 0.15) {
    //   // move down 1 rotation when within 15cm of the reef
    //   RobotContainer.elevator.moveRotation(-1);
    // } else if (zVal > 0.20) {
    //   // move up 1 rotation when further than 20cm from the reef
    //   RobotContainer.elevator.moveRotation(2);
    // } else if (zVal > 0.30) {
    //   // move up 4 rotation when further than 30cm from the reef
    //   RobotContainer.elevator.moveRotation(4);
    // }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
