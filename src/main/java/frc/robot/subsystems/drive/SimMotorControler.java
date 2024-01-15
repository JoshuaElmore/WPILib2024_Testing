package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;

class SimMotorControler {

    private double inputVoltage;
    private double appliedOutput;
    private boolean inverted;
    private boolean brakeMode = false;

    public SimMotorControler(double inputVoltage) {
        this.inputVoltage = inputVoltage;
        appliedOutput = 0.0;
        inverted = false;
    }

    public void setBrakeMode(boolean brakeMode) {
        this.brakeMode = brakeMode;
    }

    public boolean getBrakeMode() {
        return brakeMode;
    }

    public void set(double power) {
        power = MathUtil.clamp(power, -1.0, 1.0);

        if (inverted) {
            power = -power;
        }
        this.appliedOutput = power;
    }

    public double getInputVoltage() {
        return inputVoltage;
    }

    public double getAppliedOutput() {
        return appliedOutput;
    }

    public double getOutputVoltage() {
        return inputVoltage * appliedOutput;
    }

    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }
}