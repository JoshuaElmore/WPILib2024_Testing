// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

/** Add your docs here. */
public class DriveIOSim implements DriveIO {

    private Encoder leftEncoder;
    private Encoder rightEncoder;

    private EncoderSim leftEncoderSim;
    private EncoderSim rightEncoderSim;

    private AnalogGyro gyro;
    private AnalogGyroSim gyroSim;

    private DifferentialDrivetrainSim driveSim;

    private final int NEO_TPR = 42;

    private PWMSparkMax leftMotorFront;
    private PWMSparkMax leftMotorBack;
    private PWMSparkMax rightMotorFront;
    private PWMSparkMax rightMotorBack;

    public DriveIOSim(double driveRadio, double wheelRadius) {
        leftEncoder = new Encoder(0, 1);
        rightEncoder = new Encoder(2, 3);

        leftEncoderSim = new EncoderSim(rightEncoder);
        rightEncoderSim = new EncoderSim(leftEncoder);

        leftEncoder.setDistancePerPulse(2 * Math.PI * wheelRadius / NEO_TPR);
        rightEncoder.setDistancePerPulse(2 * Math.PI * wheelRadius / NEO_TPR);

        gyro = new AnalogGyro(1);
        gyroSim = new AnalogGyroSim(gyro);

        driveSim = DifferentialDrivetrainSim.createKitbotSim(
                KitbotMotor.kDoubleNEOPerSide, // 2 CIMs per side.
                KitbotGearing.k10p71,          // 10.71:1
                KitbotWheelSize.kSixInch,      // 6" diameter wheels.
                null        // No measurement noise.
);

        leftMotorFront = new PWMSparkMax(0);
        leftMotorBack = new PWMSparkMax(1);
        rightMotorFront = new PWMSparkMax(2);
        rightMotorBack = new PWMSparkMax(3);
    }

    public void updateInputs(DriveIOInputs inputs) {
        driveSim.setInputs(-leftMotorFront.get() * RobotController.getInputVoltage(),
                -rightMotorFront.get() * RobotController.getInputVoltage());

        driveSim.update(0.02);

        // Update all of our sensors.
        leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
        leftEncoderSim.setRate(driveSim.getLeftVelocityMetersPerSecond());

        rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
        rightEncoderSim.setRate(driveSim.getRightVelocityMetersPerSecond());

        gyroSim.setAngle(-driveSim.getHeading().getDegrees());

        inputs.isBrake = false;
        inputs.leftCurent = 0;
        inputs.rightCurent = 0;
        inputs.leftPos = -leftEncoder.getDistance();
        inputs.rightPos = -rightEncoder.getDistance();
        inputs.leftVel = -leftEncoder.getRate();
        inputs.rightVel = -rightEncoder.getRate();
        inputs.leftPower = leftMotorFront.get();
        inputs.rightPower = rightMotorFront.get();
        inputs.heading = Rotation2d.fromDegrees(-gyro.getAngle());

    }

    public void drive(double leftPower, double rightPower) {
        leftMotorFront.set(leftPower);
        leftMotorBack.set(leftPower);
        rightMotorFront.set(rightPower);
        rightMotorBack.set(rightPower);
    }
}
