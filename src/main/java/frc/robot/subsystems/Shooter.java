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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
    private final SparkMax motor;
    DigitalInput limitSwitch = new DigitalInput(Constants.limitCoralPort);
    // motor configs
    private final SparkMaxConfig config;

    // motor PID controllers
    private final SparkClosedLoopController PID;

    // private Counter counter = new Counter(Constants.LimitCoralPort);
    // motor encoders
    private final RelativeEncoder Encoder;
    // SmartVelocity PID
    private final double SmartVelocityP = 0.002;
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

    private boolean isLoaded;
    private boolean firstTime = false;
    private boolean intakingLoad;
    // elevator
    private double restingPosition;

    public final double allowedError = 0.05;

    public Shooter() {
        // counter.setSemiPeriodMode(true);
        // counter.reset();

        isLoaded = false;
        intakingLoad = false;
        // init motors
        motor = new SparkMax(Constants.shooterPort, MotorType.kBrushless);

        // init config
        config = new SparkMaxConfig();

        // init PID controllers (closed loop)
        PID = motor.getClosedLoopController();

        Encoder = motor.getEncoder();

        // set idle mode for motors
        config.idleMode(IdleMode.kBrake);

        // set voltage compensation
        config.voltageCompensation(12.0);

        // Set current limits
        config.smartCurrentLimit(40);

        configurePIDControllers();

        // restore factory defaults and set new configs for each motor
        config.signals.primaryEncoderPositionPeriodMs(5);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configurePIDControllers() {
        // Apply PID constants to the PID controllers
        // left motor
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.pid(SmartVelocityP, SmartVelocityI, SmartVelocityD);
        config.closedLoop.maxMotion.maxAcceleration(maxAccel);
        config.closedLoop.maxMotion.maxVelocity(maxVel);
        config.closedLoop.velocityFF(SmartVelocityFF);
        config.closedLoop.maxMotion.allowedClosedLoopError(allowedError);
    }

    private boolean getLimitSwitch() {
        return !limitSwitch.get();
    }

    /**
     * @return position in ticks
     */
    public double getEncoder() {
        return Encoder.getPosition() * 42;
    }

    public void shoot_that_fucker(double speed) { 
        // if(isLoaded) {
            
        //     double x_dist = getreefDistance();
        //     // Shoot coral at 20% speed
        //     double radianConstant = Math.toRadians(35);
        //     double t = (-1) * x_dist / Math.sin(radianConstant);
        //     double y_diff = Math.tan(radianConstant) * x_dist - 4.9 * Math.pow(t, 2);
        //     double yInch = y_diff * 39.37;

        //     PID.setReference(speed, ControlType.kDutyCycle);
        //     isLoaded = false;
        // }
        PID.setReference(speed, ControlType.kDutyCycle);
    }

    @Override
    public void periodic() {
        if(getLimitSwitch()) {
            isLoaded = true;
            if (firstTime) {    
                PID.setReference(0.1, ControlType.kDutyCycle);
                try {
                    Thread.sleep(100);
                } catch(InterruptedException e) {
                    e.printStackTrace();
                }
                PID.setReference(0, ControlType.kDutyCycle);
            }
            firstTime = false;
        } else {
            isLoaded = false;
        }
        if(intakingLoad && isLoaded == false) {
            // RobotContainer.elevator.move(-0.6);
            // RobotContainer.elevator.move(0.5);
            PID.setReference(0.1, ControlType.kDutyCycle);
            firstTime = true;
        } else {
            intakingLoad = false;
        }
        SmartDashboard.putBoolean("coral outtake: ", isLoaded);
        SmartDashboard.putNumber("distance sensor: ", getreefDistance());
    }

    private double getreefDistance() {
        // in cm
        // return ((counter.getPeriod() - 0.001) *1E+5 * 3.61) + 2;
        return 0;
    }

    public void move(double speed) {
        PID.setReference(speed, ControlType.kDutyCycle);
    }

    public void intakeStart() {
        intakingLoad = true;
    }
    public void intakeStop() {
        intakingLoad = false;
    }
}