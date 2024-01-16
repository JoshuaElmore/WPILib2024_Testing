
package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drive.Drive;

public final class AutoCommandConfig {

    public static void init(Drive drivetrain) {

        AutoBuilder.configureRamsete(
                () -> drivetrain.getPose(),
                (pose) -> drivetrain.setPose(pose),
                () -> drivetrain.getChassisSpeed(),
                (chassisSpeed) -> drivetrain.setChassisSpeed(chassisSpeed),
                2.0,
                0.7,
                new ReplanningConfig(),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                drivetrain);
    }

    // Default constructor that just throws an exception if you attempt to create an
    // instace of this class.
    private AutoCommandConfig() {
        throw new UnsupportedOperationException("This is a static class, you cannot instantiate it.");
    }
}
