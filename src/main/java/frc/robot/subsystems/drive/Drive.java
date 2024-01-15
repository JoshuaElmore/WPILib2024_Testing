// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {

  private DriveIO io;

  private DifferentialDriveOdometry odometry;

  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
  /** Creates a new Drive. */
  public Drive(DriveIO io, double trackWidth) {
    this.io = io;

    io.updateInputs(inputs);

    odometry = new DifferentialDriveOdometry(inputs.heading,inputs.leftPos, inputs.rightPos,new Pose2d(0.0, 0.0, new Rotation2d()));

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

    Logger.recordOutput("DriveTrain/Pos2d", this.getPose());




  }
  public void setSpeed(double leftSpeed, double rightSpeed) {
    io.drive(leftSpeed, rightSpeed);
  }

  public Pose2d getPose() {
    return odometry.update(inputs.heading, inputs.leftPos, inputs.rightPos);
  }
}
