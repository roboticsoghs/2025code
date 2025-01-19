// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class Elevator extends SubsystemBase {
    public static enum ElevatorPosition { //Different states that determines what stage the arm is in.
        RESTING_POSITION(0),
        LEVEL_0(0), // TODO:
        LEVEL_1(0), // TODO:
        LEVEL_2(0), // TODO:
        LEVEL_3(0); // TODO:

        private final int pos;
        ElevatorPosition(int pos)  {
            this.pos = pos;
        }
        public int getValue() { return pos; }
    }

    public static enum ElevatorStates { //Different states that determines what stage the arm is in.
        NOT_INITIALIZED,
        INITIALIZING,
        INITIALIZED
    }

    DigitalInput limitSwitch = new DigitalInput(Constants.limitPort);

    private final SparkMax leftMotor;
    private final SparkMax rightMotor;

    // motor configs
    private final SparkMaxConfig leftConfig;
    private final SparkMaxConfig rightConfig;

    // motor PID controllers
    private final SparkClosedLoopController leftPID;
    private final SparkClosedLoopController rightPID;

    // motor encoders
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    // SmartVelocity PID
    private final double SmartVelocityP = 0.0002;
    private final double SmartVelocityI = 0; // 0.00003
    private final double SmartVelocityD = 0;
    private final double SmartVelocityFF = 0;

    // max motor speed
    private final double MaxOutput = 1;
    // min motor speed
    private final double MinOutput = -1;

    // max motor acceleration
    private final double maxAccel = 1000000000;
    private final int SmartMotionID = 0;
    private int MaxMotionID = 1;
    private final int maxVel = 4075;

    // elevator
    private ElevatorPosition position;
    private ElevatorStates isCalibrated;
    private double restingLeftPosition;
    private double restingRightPosition;

    public final double allowedError = 0.05;

    public Elevator() {
        // init elevator
        isCalibrated = ElevatorStates.INITIALIZING;
        position = ElevatorPosition.RESTING_POSITION;
        restingLeftPosition = (double) 0;
        restingRightPosition = (double) 0;

        // init motors
        leftMotor = new SparkMax(Constants.elevatorLeftMotorPort, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.elevatorRightMotorPort, MotorType.kBrushless);

        // init config
        leftConfig = new SparkMaxConfig();
        rightConfig = new SparkMaxConfig();

        // Configuring the rear motors to follow the front motors
        leftConfig.follow(rightMotor);

        // init PID controllers (closed loop)
        leftPID = leftMotor.getClosedLoopController();
        rightPID = rightMotor.getClosedLoopController();

        // init encoders
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        // set idle mode for motors
        leftConfig.idleMode(IdleMode.kBrake);
        rightConfig.idleMode(IdleMode.kBrake);

        leftConfig.inverted(false);
        rightConfig.inverted(true);

        // set voltage compensation
        leftConfig.voltageCompensation(12.0);
        rightConfig.voltageCompensation(12.0);

        // Set current limits
        leftConfig.smartCurrentLimit(40);
        rightConfig.smartCurrentLimit(40);

        configurePIDControllers();

        // restore factory defaults and set new configs for each motor
        leftConfig.signals.primaryEncoderPositionPeriodMs(5);
        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightConfig.signals.primaryEncoderPositionPeriodMs(5);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configurePIDControllers() {
        // Apply PID constants to the PID controllers
        // left motor
        leftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        leftConfig.closedLoop.pid(SmartVelocityP, SmartVelocityI, SmartVelocityD);
        leftConfig.closedLoop.maxMotion.maxAcceleration(maxAccel);
        leftConfig.closedLoop.maxMotion.maxVelocity(maxVel);
        leftConfig.closedLoop.velocityFF(SmartVelocityFF);
        leftConfig.closedLoop.maxMotion.allowedClosedLoopError(allowedError);

        // right motor
        rightConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        rightConfig.closedLoop.pid(SmartVelocityP, SmartVelocityI, SmartVelocityD);
        rightConfig.closedLoop.maxMotion.maxAcceleration(maxAccel);
        rightConfig.closedLoop.maxMotion.maxVelocity(maxVel);
        rightConfig.closedLoop.velocityFF(SmartVelocityFF);
        rightConfig.closedLoop.maxMotion.allowedClosedLoopError(allowedError);
    }

    /**
     * @return position in ticks
     */
    public double getLeftEncoder() {
        return leftEncoder.getPosition() * 42;
    }

    /**
     * @return position in ticks
     */
    public double getRightEncoder() {
        return rightEncoder.getPosition() * 42;
    }

    public void setPosition(ElevatorPosition pos) {
        switch (pos) {
            case RESTING_POSITION -> {
                // should trip limit switch
                // resets and calibrates elevator
                if (restingLeftPosition == getLeftEncoder() && restingRightPosition == getRightEncoder()) break; // elevator already is in resting position

                // leftPID.setReference(-0.1, ControlType.kMAXMotionVelocityControl);
                position = ElevatorPosition.RESTING_POSITION;
            }

            case LEVEL_0 -> {
                // find height in inches
                position = ElevatorPosition.LEVEL_0;
            }

            case LEVEL_1 -> {
                // find height in inches
                position = ElevatorPosition.LEVEL_1;
            }

            case LEVEL_2 -> {
                // find height in inches
                position = ElevatorPosition.LEVEL_2;
            }
            case LEVEL_3 -> {
                // find height in inches
                position = ElevatorPosition.LEVEL_3;
            }
        }
    }

    public double getHeight() {
        // TODO: find scaling factor
        double scalingFactor = 1; // converts ticks to inches

        return (getLeftEncoder() - restingLeftPosition) * scalingFactor;
    }

    public boolean getLimitSwitch() {
        return limitSwitch.get();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (isCalibrated == ElevatorStates.INITIALIZING && limitSwitch.get()) {
            // Stop the motor when limit switch is triggered
            leftMotor.stopMotor();

            // Reset encoder or position tracking
            resetEncoders();
            System.out.println("Encoder resting values have been set");
            System.out.println("Right Encoder Value: " + restingRightPosition + " Left Encoder Value: " + restingLeftPosition);
            // Mark as calibrated
            isCalibrated = ElevatorStates.INITIALIZED;
        } else if (isCalibrated == ElevatorStates.INITIALIZING) {
            leftPID.setReference(-0.1, ControlType.kDutyCycle); // Move elevator slowly downward
            rightPID.setReference(-0.1, ControlType.kDutyCycle);
        }
    }

    public void resetEncoders() {
        restingLeftPosition = getLeftEncoder();
        restingRightPosition = getRightEncoder();
    }
}