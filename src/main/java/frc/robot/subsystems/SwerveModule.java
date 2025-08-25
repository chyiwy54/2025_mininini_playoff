package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder; 
import edu.wpi.first.math.controller.PIDController; 
import edu.wpi.first.math.geometry.Rotation2d; 
import edu.wpi.first.math.kinematics.SwerveModulePosition; 
import edu.wpi.first.math.kinematics.SwerveModuleState; 
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants; 
import frc.robot.Constants.ModuleConstants;
import com.ctre.phoenix6.hardware.CANcoder;

public class SwerveModule { 

    private final SparkFlex driveMotor; 
    private final SparkFlex turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder; 

    private final PIDController turningPidController; 

    private final CANcoder absoluteEncoder; 
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad; 
    private final SparkFlexConfig driveConfig = new SparkFlexConfig();
    private final SparkFlexConfig turningConfig = new SparkFlexConfig();

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderReversed = absoluteEncoderReversed; 
        this.absoluteEncoder = new CANcoder(absoluteEncoderId);

        this.absoluteEncoderOffsetRad = this.absoluteEncoder.getPosition().getValueAsDouble();

        driveMotor = new SparkFlex(driveMotorId, MotorType.kBrushless);
        turningMotor = new SparkFlex(turningMotorId, MotorType.kBrushless); 

        driveConfig.inverted(driveMotorReversed);
        driveConfig.idleMode(IdleMode.kBrake);
        driveConfig.encoder.positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveConfig.encoder.velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        turningConfig.inverted(turningMotorReversed);
        turningConfig.encoder.positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningConfig.encoder.velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
        turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        driveEncoder = driveMotor.getEncoder(); 
        turningEncoder = turningMotor.getEncoder();

        turningPidController = new PIDController(0.5, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI); // 設置輸入範圍為 -π 到 π

        resetEncoders();
    }

    public double getDrivePosition() { 
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() { 
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() { 
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double revolutions = Units.rotationsToRadians(absoluteEncoder.getAbsolutePosition().getValueAsDouble());

        revolutions *= (absoluteEncoderReversed ? -1.0 : 1.0);
        return Math.IEEEremainder(revolutions, 2 * Math.PI); 
    }

    public void resetEncoders() { 
        driveEncoder.setPosition(0); 
        turningEncoder.setPosition(getAbsoluteEncoderRad()); 
    }

    public SwerveModuleState getState() { 
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    // @SuppressWarnings("deprecation")//抑制 Java 編譯器對使用已被標記為「過時」的功能時產生的警告。
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) { 
            stop();
            return;
        }
        state.optimize(getState().angle);
        
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond); 
        turningMotor.set(turningPidController.calculate(getAbsoluteEncoderRad(), state.angle.getRadians()));
    }

    public SwerveModulePosition getPosition() { 
        return new SwerveModulePosition(
                getDrivePosition(),
                new Rotation2d(getTurningPosition())
        );
    }

    public void stop() { 
        driveMotor.set(0);
        turningMotor.set(0);
    }

}
