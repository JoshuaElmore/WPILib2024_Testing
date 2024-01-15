// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

/** Add your docs here. */
public class DriveIOSim implements DriveIO{

    private Encoder leftEncoder;
    private Encoder rightEncoder;

    private EncoderSim leftEncoderSim;
    private EncoderSim rightEncoderSim;

    private AnalogGyro gyro;
    private AnalogGyroSim gyroSim;

    private DifferentialDrivetrainSim driveSim;

    private final int NEO_TPR = 42;

    private SimMotorControler leftMotorFront;
    private SimMotorControler leftMotorBack;
    private SimMotorControler rightMotorFront;
    private SimMotorControler rightMotorBack;

    public DriveIOSim(double driveRadio, double wheelRadius) {
        leftEncoder = new Encoder(0, 1);
        rightEncoder = new Encoder(2, 3);

        leftEncoderSim = new EncoderSim(rightEncoder);
        rightEncoderSim = new EncoderSim(leftEncoder);

        leftEncoder.setDistancePerPulse(2 * Math.PI * wheelRadius / NEO_TPR);
        rightEncoder.setDistancePerPulse(2 * Math.PI * wheelRadius / NEO_TPR);

        gyro = new AnalogGyro(1);
        gyroSim = new AnalogGyroSim(gyro);

        driveSim = new DifferentialDrivetrainSim(
                DCMotor.getNEO(2), // 2 NEO motors on each side of the drivetrain.
                driveRadio, // gearing reduction.
                7.5, // MOI.
                Units.lbsToKilograms(105), // The mass of the robot.
                wheelRadius, // The robot wheel radius.
                Units.inchesToMeters(30), // The track width is meters.

                // The standard deviations for measurement noise:
                // x and y: 0.001 m
                // heading: 0.001 rad
                // l and r velocity: 0.1 m/s
                // l and r position: 0.005 m
                //VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
                null
                );

        leftMotorFront = new SimMotorControler(12.0);
        leftMotorBack = new SimMotorControler(12.0);
        rightMotorFront = new SimMotorControler(12.0);
        rightMotorBack = new SimMotorControler(12.0);

        leftMotorFront.setInverted(true);
        leftMotorBack.setInverted(true);

        leftMotorFront.setBrakeMode(true);
        leftMotorBack.setBrakeMode(true);
        rightMotorFront.setBrakeMode(true);
        rightMotorBack.setBrakeMode(true);
    }

    public void updateInputs(DriveIOInputs inputs) {
        driveSim.setInputs(leftMotorFront.getOutputVoltage(),rightMotorFront.getOutputVoltage());

        driveSim.update(0.02);

        // Update all of our sensors.
        leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
        leftEncoderSim.setRate(driveSim.getLeftVelocityMetersPerSecond());

        rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
        rightEncoderSim.setRate(driveSim.getRightVelocityMetersPerSecond());

        gyroSim.setAngle(-driveSim.getHeading().getDegrees());

        inputs.isBrake = leftMotorFront.getBrakeMode();
        inputs.leftCurent = 0;
        inputs.rightCurent = 0;
        inputs.leftPos = leftEncoder.getDistance();
        inputs.rightPos = rightEncoder.getDistance();
        inputs.leftVel = leftEncoder.getRate();
        inputs.rightVel = rightEncoder.getRate();
        inputs.leftPower = leftMotorFront.getAppliedOutput();
        inputs.rightPower = rightMotorFront.getAppliedOutput();
        inputs.heading = Rotation2d.fromDegrees(Units.degreesToRadians(gyro.getAngle()));

    }

    public void drive(double leftPower, double rightPower) {
        leftMotorFront.set(leftPower);
        leftMotorBack.set(leftPower);
        rightMotorFront.set(rightPower);
        rightMotorBack.set(rightPower);
    }
}
