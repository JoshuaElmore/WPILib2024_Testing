// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {

  private DriveIO io;

  private DifferentialDriveOdometry odometry;

  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

  private DifferentialDriveKinematics kinematics;

  private Pose2d pose;

  private double maxSpeed;
  /** Creates a new Drive. */
  public Drive(DriveIO io, double trackWidth, double maxSpeed) {
    this.io = io;
    this.maxSpeed = maxSpeed;

    io.updateInputs(inputs);

    odometry = new DifferentialDriveOdometry(inputs.heading,inputs.leftPos, inputs.rightPos,new Pose2d(0.0, 0.0, new Rotation2d()));

    kinematics = new DifferentialDriveKinematics(trackWidth);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("DriveTrain", inputs);

    if (this.getCurrentCommand() != null) {

      Logger.recordOutput("DriveTrain/CurentCommand", this.getCurrentCommand().getName());
    } else {
      Logger.recordOutput("DriveTrain/CurentCommand", "none");
    }
    
    //Update Odometry
    pose = odometry.update(inputs.heading, inputs.leftPos, inputs.rightPos);

    Logger.recordOutput("DriveTrain/Pos2d", pose);

    Logger.recordOutput("DriveTrain/Speeds", this.getChassisSpeed());


  }
  public void setPowers(double leftSpeed, double rightSpeed) {
    io.drive(leftSpeed, rightSpeed);
  }

  public ChassisSpeeds getChassisSpeed() {
    DifferentialDriveWheelSpeeds wheelSpeed = new DifferentialDriveWheelSpeeds (inputs.leftVel, inputs.rightVel);

    return kinematics.toChassisSpeeds(wheelSpeed);
  }

  public void setChassisSpeed(ChassisSpeeds chassisSpeed) {
    DifferentialDriveWheelSpeeds wheelSpeed = kinematics.toWheelSpeeds(chassisSpeed);

    io.drive(wheelSpeed.leftMetersPerSecond/maxSpeed, wheelSpeed.rightMetersPerSecond/maxSpeed);
  }

  public Pose2d getPose() {
    return pose;
  }
  public void resetPose() {
    odometry.resetPosition(inputs.heading, inputs.leftPos,inputs.rightPos,new Pose2d(0.0, 0.0, new Rotation2d()));
  }
  public void resetGyro() {
    odometry.resetPosition(inputs.heading, inputs.leftPos,inputs.rightPos,new Pose2d(pose.getX(), pose.getY(), new Rotation2d()));
  }
}
