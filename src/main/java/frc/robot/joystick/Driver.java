package frc.robot.joystick;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class Driver extends XboxController {
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    private final SlewRateLimiter turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);

    public Driver() {
        super(0);
    }

    public double getXLimiter() {
        double speed = MathUtil.applyDeadband(this.getLeftY(),0.5);
        return this.xLimiter.calculate(-speed * DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    }

    public double getYLimiter() {
        double speed = MathUtil.applyDeadband(this.getLeftX(),0.5);
        return this.yLimiter.calculate(-speed *  DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    }

    public double getTurningLimiter() {
        double speed = MathUtil.applyDeadband(this.getRightX(),0.5);
        return this.turningLimiter.calculate(-speed * DriveConstants.MAX_ANGULAR_SPEED);
    }

    public boolean isIntake() {
        return this.getLeftTriggerAxis() > 0.5;
    }
    public boolean isIntake2() {
        return this.getRightTriggerAxis() > 0.5;
    }

    public boolean isOuttake() {
        return this.getLeftBumperButton();
    }
}

