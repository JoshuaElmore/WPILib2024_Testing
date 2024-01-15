package frc.robot.utility;

import static frc.robot.Constants.*;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOSim;

public final class SubsystemFactory {

    public static Drive createDriveTrain(RobotIdentity identity) {
        switch (identity) {
            case SIMULATION:
                return new Drive(new DriveIOSim(DRIVE_GEAR_RADIO, DRIVE_WHEEL_RADIUS), TRACK_WIDTH, DRIVE_MAX_SPEED);
            default:
                return null;
        }
    }
}
