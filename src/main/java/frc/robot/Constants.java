package frc.robot; 

import edu.wpi.first.math.geometry.Translation2d; 
import edu.wpi.first.math.kinematics.SwerveDriveKinematics; 
import edu.wpi.first.math.util.Units; 

public final class Constants { 

    public static final class ModuleConstants { 
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4); 
        public static final double kDriveMotorGearRatio =  49.0/300.0;
        public static final double kTurningMotorGearRatio = 1.0 / 21.43; 
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0; 
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0; 
    }

    public static final class DriveConstants {
        public static final double kTrackWidth = 0.508;
        public static final double kWheelBase = 0.508;
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics( 
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); 

        public static final int kFrontLeftDriveMotorPort = 11; 
        public static final int kBackLeftDriveMotorPort = 44; 
        public static final int kFrontRightDriveMotorPort = 22;
        public static final int kBackRightDriveMotorPort = 33; 

        public static final int kFrontLeftTurningMotorPort = 1;
        public static final int kBackLeftTurningMotorPort = 4; 
        public static final int kFrontRightTurningMotorPort = 2;
        public static final int kBackRightTurningMotorPort = 3; 

        public static final boolean kFrontLeftTurningEncoderReversed = true; 
        public static final boolean kBackLeftTurningEncoderReversed = true; 
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true; 
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 3;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
        public static final int kBackRightDriveAbsoluteEncoderPort = 2;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = (0.00903) * Math.PI * 2;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = (0.000732) * Math.PI * 2;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = (0.008301) * Math.PI * 2;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = (0.005371) * Math.PI * 2;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 3.8; // Max speed
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI; // 最大角速度

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond; // 遙控模式最大速度
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond; // 遙控模式最大角速度
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 5.0; // 遙控模式最大加速度
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 5.0; // 遙控模式最大角加速度
        public static final double MAX_ANGULAR_SPEED = 3.5;
    }
    public static final class ControllerConstants {
        public static final double CORALINTAKE_SPEED = 0.16; // Intake speed
        public static final double CORALINTAKE2_SPEED = 0.225; // Intake2 speed
        public static final double CORALOUTTAKE_AUTO_SPEED = 0.5; // Outtake speed
        public static final double CORALOUTTAKE_SPEED = 0.17; // Manual outtake speed
        public static final double CORALINTAKE_AUTO_SPEED = 0.5; // Auto intake speed
    }
    
}
