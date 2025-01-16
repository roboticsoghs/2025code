// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem.POSITION;


public class ElevatorSubsystem extends SubsystemBase {
  private CANSparkMax elevatorMotor;
	private CANSparkMax elevatorSlaveMotor;
  private SparkPIDController ElevatorPIDController;
  private RelativeEncoder elevatorEncoder;

  public enum ElevatorPosition {
    BOTTOM(0),
    MID(Math.PI / 2),       // Half rotation
    TOP(Math.PI);           // Full rotation

    private final double radians;

    ElevatorPosition(double radians) {
        this.radians = radians;
    }

    public double getRadians() {
        return radians;
    }
  }

  /** Creates a new ExampleSubsystem. */
  public void Elevator()
	{
		elevatorMotor = new CANSparkMax(Constants.elevatorMainMotor, MotorType.kBrushless);
		elevatorSlaveMotor = new CANSparkMax(Constants.elevatorSlaveMotor, MotorType.kBrushless);

    elevatorSlaveMotor.follow(elevatorMotor);

    elevatorEncoder = elevatorMotor.getEncoder();
    ElevatorPIDController = elevatorMotor.getPIDController();

    defaultPID();
	}

  public void defaultPID() {
    double P = 0.0;
    double I = 0.0;
    double D = 0.0;
    double FF = 0.0;

    ElevatorPIDController.setP(P);
    ElevatorPIDController.setI(I);
    ElevatorPIDController.setD(D);
    ElevatorPIDController.setFF(FF);
    ElevatorPIDController.setOutputRange(-1.0, 1.0);
  }

  public double getEncoderPosition() {
    return elevatorEncoder.getPosition();
  }

  public void resetEncoder() {
    elevatorEncoder.setPosition(0);
  }

  public void setPosition(ElevatorPosition pos) {
    // TODO: implement
    switch (pos) {
      case BOTTOM:
        break;
      case MID:
        break;
      case TOP:
        break;
    }
  }
}
