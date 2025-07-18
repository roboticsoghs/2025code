// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class Elevator extends SubsystemBase {
    public static enum ElevatorPosition { //Different states that determines what stage the arm is in.
        // position value is in revolutions relative to RESTING_POSITION
        // TODO: figure out revolutions of the motor needed for each level
        RESTING_POSITION(1),
        INTAKE_POSITION(1), 
        LEVEL_0(10.8),
        LEVEL_1(13.9),
        LEVEL_2(46);

        private final double pos;
        ElevatorPosition(double pos)  {
            this.pos = pos;
        }
        public double getValue() { return pos; }
    }

    public static enum ElevatorStates { // Different states that determines what stage the arm is in.
        NOT_INITIALIZED("NOT_INITIALIZED"),
        INITIALIZING("INITIALIZING"),
        INITIALIZED("INITIALIZED");

        private final String state;
        ElevatorStates(String state) {
            this.state = state;
        }
        public String getStatusMessage(){ return state; }
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
    private final double SmartVelocityP = 0.55; // 2.2
    private final double SmartVelocityI = 0;
    private final double SmartVelocityD = 0;
    private final double SmartVelocityFF = 0;

    // max motor speed
    private final double MaxOutput = 1;
    // min motor speed
    private final double MinOutput = -1;
    private final double arbFeedForward = 0.125;

    // max motor acceleration
    private final double maxAccel = 2700;
    private final int SmartMotionID = 0;
    private int MaxMotionID = 1;
    private final int maxVel = 3500;

    // elevator
    private ElevatorPosition position;
    private ElevatorStates isCalibrated = ElevatorStates.INITIALIZING;

    // To Do: up feedForward code
    private static double kS = 0.1;
    private static double kG = 0.7;
    private static double kV = 0; // prev 0.05
    private static double kA = 0;
    // private final ElevatorFeedforward feedForward = new ElevatorFeedforward(kS, kG, kV, kA);
    public final double allowedError = 0.05;

    public double encoderValue;

    public Elevator() {
        // init elevator
        isCalibrated = ElevatorStates.INITIALIZING;
        position = ElevatorPosition.RESTING_POSITION;
        encoderValue = 0;
        // init motors
        leftMotor = new SparkMax(Constants.elevatorLeftMotorPort, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.elevatorRightMotorPort, MotorType.kBrushless);

        // init config
        leftConfig = new SparkMaxConfig();
        rightConfig = new SparkMaxConfig();

        // Configuring the rear motors to follow the front motors
        rightConfig.follow(leftMotor, true);

        // init PID controllers (closed loop)
        leftPID = leftMotor.getClosedLoopController();
        rightPID = rightMotor.getClosedLoopController();

        // init encoders
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        // set idle mode for motors
        leftConfig.idleMode(IdleMode.kBrake);
        rightConfig.idleMode(IdleMode.kBrake);

        leftConfig.inverted(true);
        rightConfig.inverted(false);

        // set voltage compensation
        leftConfig.voltageCompensation(12.0);
        // rightConfig.voltageCompensation(12.0);

        // Set current limits
        leftConfig.smartCurrentLimit(40);
        // rightConfig.smartCurrentLimit(40);

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
        leftConfig.closedLoop.pid(SmartVelocityP, SmartVelocityI, SmartVelocityD, ClosedLoopSlot.kSlot3);
        leftConfig.closedLoop.maxMotion.maxAcceleration(maxAccel, ClosedLoopSlot.kSlot3);
        leftConfig.closedLoop.maxMotion.maxVelocity(maxVel, ClosedLoopSlot.kSlot3);
        leftConfig.closedLoop.velocityFF(SmartVelocityFF, ClosedLoopSlot.kSlot3);
        leftConfig.closedLoop.maxMotion.allowedClosedLoopError(allowedError, ClosedLoopSlot.kSlot3);

        // right motor
        // rightConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        // rightConfig.closedLoop.pid(SmartVelocityP, SmartVelocityI, SmartVelocityD);
        // rightConfig.closedLoop.maxMotion.maxAcceleration(maxAccel);
        // rightConfig.closedLoop.maxMotion.maxVelocity(maxVel);
        // rightConfig.closedLoop.velocityFF(SmartVelocityFF);
        // rightConfig.closedLoop.maxMotion.allowedClosedLoopError(allowedError);
    }

    /**
     * @return position in ticks
     */
    public double getLeftEncoder() {
        return leftEncoder.getPosition(); // 42 ticks per rotation
    }

    /**
     * @return position in ticks
     */
    public double getRightEncoder() {
        return rightEncoder.getPosition();
    }

    public void setPosition(ElevatorPosition pos) {
        if(isCalibrated.getStatusMessage() != "INITIALIZED") return;
        if(limitCheck(pos.getValue())) {
            switch (pos) {
                case INTAKE_POSITION -> {
                    position = ElevatorPosition.INTAKE_POSITION;
                    leftPID.setReference(position.getValue(), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot3, arbFeedForward); // calculate necessary rotations
                    // rightPID.setReference(-position.getValue(), ControlType.kMAXMotionPositionControl); // calculate necessary rotations
                    // rightPID.setReference(position.getValue(), ControlType.kMAXMotionPositionControl);
                    SmartDashboard.putNumber("elevator revs: ", position.getValue());
                    position = ElevatorPosition.INTAKE_POSITION;
                }

                case RESTING_POSITION -> {
                    position = ElevatorPosition.RESTING_POSITION;
                    leftPID.setReference(position.getValue(), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot3, arbFeedForward); // calculate necessary rotations
                    // rightPID.setReference(-position.getValue(), ControlType.kMAXMotionPositionControl); // calculate necessary rotations
                    // rightPID.setReference(position.getValue(), ControlType.kMAXMotionPositionControl);
                    SmartDashboard.putNumber("elevator revs: ", position.getValue());
                    position = ElevatorPosition.RESTING_POSITION;
                }

                case LEVEL_0 -> {
                    position = ElevatorPosition.LEVEL_0;
                    leftPID.setReference(position.getValue(), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot3, arbFeedForward); // calculate necessary rotations
                    // rightPID.setReference(-position.getValue(), ControlType.kMAXMotionPositionControl);
                    // rightPID.setReference(position.getValue(), ControlType.kMAXMotionPositionControl);
                    SmartDashboard.putNumber("elevator revs: ", position.getValue());
                }

                case LEVEL_1 -> {
                    // find height in inches
                    position = ElevatorPosition.LEVEL_1;
                    // leftPID.setReference(-position.getValue(), ControlType.kMAXMotionPositionControl); // calculate necessary rotations
                    // rightPID.setReference(position.getValue(), ControlType.kMAXMotionPositionControl);
                    
                    // double inch = rotationsToLinear(0.125);
                    // double rotations = 0.125 * Constants.gearRatio;
                    leftPID.setReference(position.getValue(), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot3, arbFeedForward); // calculate necessary rotations
                    // rightPID.setReference(-rotations, ControlType.kMAXMotionPositionControl);
                    SmartDashboard.putNumber("elevator revs: ", position.getValue());
                }

                case LEVEL_2 -> {
                    // find height in inches
                    position = ElevatorPosition.LEVEL_2;
                    leftPID.setReference(position.getValue(), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot3, arbFeedForward); // calculate necessary rotations
                    // rightPID.setReference(-position.getValue(), ControlType.kMAXMotionPositionControl);
                    // rightPID.setReference(position.getValue(), ControlType.kMAXMotionPositionControl);
                    SmartDashboard.putNumber("elevator revs: ", position.getValue());
                }
            }
        }
    }

    public boolean limitCheck(double rotations) {
        return rotations > Constants.UPPER_LOWER_LIMIT && rotations < Constants.UPPER_HARD_LIMIT;
    }

    public void moveRotation(double rotations) {
        if(limitCheck(encoderValue + rotations)) {
            // double feedValue = feedForward.calculateWithVelocities(maxVel, maxAccel);
            // double feedForwardValue = feedForward.calculate(maxVel / 2);
            leftPID.setReference(encoderValue + rotations, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot3, arbFeedForward);
            
            SmartDashboard.putNumber("target pos: ", encoderValue + rotations);
            // leftMotor.setVoltage(feedForwardValue);
            // SmartDashboard.putNumber("feedforward voltage: ", feedForwardValue);
        }
    }

    // public void move(double rotations) {
    //     if(limitCheck(rotations)) {
    //         // double feedValue = feedForward.calculateWithVelocities(maxVel, maxAccel);
    //         // double feedForwardValue = feedForward.calculate(maxVel / 2);
    //         leftPID.setReference(rotations, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot3, arbFeedForward);
    //         // leftMotor.setVoltage(feedForwardValue);
    //         SmartDashboard.putNumber("target pos: ", rotations);
    //         // SmartDashboard.putNumber("feedforward voltage: ", feedForwardValue);
    //     }
    // }
    /**
     * @return height in inches
     */
    public double getHeight() {
        // TODO: find scaling factor
        double scalingFactor = 1; // converts ticks to inches ( x ticks = 1 inch )

        return (getLeftEncoder()) * scalingFactor;
    }

    /**
     * @return true if something is detected, false if nothing is detected
     */
    public boolean getLimitSwitch() {
        return limitSwitch.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("limit switch state: ", !limitSwitch.get());
        SmartDashboard.putString("elevator state: ", isCalibrated.getStatusMessage());
        SmartDashboard.putNumber("left encoder: ", getLeftEncoder());
        SmartDashboard.putNumber("right encoder: ", getRightEncoder());
        encoderValue = getLeftEncoder();

        // apply feedforward voltage to motor
        // double ffVoltage = feedForward.calculate(0);
        // leftMotor.setVoltage(ffVoltage);
        // SmartDashboard.putNumber("ff Voltage", ffVoltage);

        // This method will be called once per scheduler run
        if (isCalibrated == ElevatorStates.INITIALIZING && !limitSwitch.get()) {
            // Stop the motor when limit switch is triggered
            // leftMotor.stopMotor();
            // rightMotor.stopMotor();

            while(!limitSwitch.get()) {
                SmartDashboard.putBoolean("elevator state: ", false);
                SmartDashboard.putNumber("left encoder: ", getLeftEncoder());
                SmartDashboard.putNumber("right encoder: ", getRightEncoder());
                leftPID.setReference(0.1, ControlType.kDutyCycle, ClosedLoopSlot.kSlot3, arbFeedForward); 
            }
    
            leftPID.setReference(0, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot3, arbFeedForward);
            // leftMotor.stopMotor();
            try {
                Thread.sleep(500);
            } catch(InterruptedException e) {
                e.printStackTrace();
            }
            // Reset encoder or position tracking
            resetEncoders();
            SmartDashboard.putNumber("left encoder: ", getLeftEncoder());
            SmartDashboard.putNumber("right encoder: ", getRightEncoder());
            SmartDashboard.putBoolean("state: ", false);
            System.out.println("Encoder resting values have been set");
            System.out.println("Right Encoder Value: " + " Left Encoder Value: ");
            
            // Mark as calibrated
            isCalibrated = ElevatorStates.INITIALIZED;
        } else if (isCalibrated.getStatusMessage() == "INITIALIZING") {
            SmartDashboard.putBoolean("state: ", true);
            // leftPID.setReference(20, ControlType.kMAXMotionPositionControl); // Move elevator slowly downward
            // rightPID.setReference(20, ControlType.kMAXMotionPositionControl);
        }
    }

    /**
     * sets current encoder values for left and right motors as the resting positions
     */
    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }
    public double rotationsToLinear(double rotations) {
        double actualRotations = rotations / Constants.ElevatorgearRatio;
        return (2 * Math.PI * Constants.ElevatorwheelRadius) * actualRotations;
    }

    public double lineartoRotations(double inches) {
        double bigRotations = inches / (2 * Math.PI * Constants.ElevatorwheelRadius);
        return bigRotations * Constants.ElevatorgearRatio;
    }

    public double linearErrorRate(double error) {
        return error;
    }
}