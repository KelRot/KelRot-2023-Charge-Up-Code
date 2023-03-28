package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDDebugger {
    public PIDDebugger() {
        
    }

    public PIDController getPIDControllerFromDashboard(String prefix) {
        double kp = SmartDashboard.getNumber(prefix + "P", 0);
        double ki = SmartDashboard.getNumber(prefix + "I", 0);
        double kd = SmartDashboard.getNumber(prefix + "D", 0);

        return new PIDController(kp, ki, kd);
    }

    public void setPIDControllerToDashboard(String prefix, double kp, double ki, double kd) {
        SmartDashboard.putNumber(prefix + "P", kp);
        SmartDashboard.putNumber(prefix + "I", ki);
        SmartDashboard.putNumber(prefix + "D", kd);
    }
}
