package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import static edu.wpi.first.units.Units.Radians;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final Pigeon2 gyro = new Pigeon2(6);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics,
            new Rotation2d(0),
            new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
            });
    private final PIDController pidController = new PIDController(0.2, 0.0, 0.0);
    StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
    StructPublisher<Pose2d> realSwerve = NetworkTableInstance.getDefault()
            .getStructTopic("realSwervepose", Pose2d.struct).publish();
    StructPublisher<Pose2d> simPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("simSwerveposed", Pose2d.struct).publish();

    private final Field2d m_field = new Field2d();

    private Pose2d simOdometry;
    private double posedX = 0;
    private double posedY = 0;
    private double posedRot = 0;
    private double timeFromLastUpdate = 0;
    private double lastSimTime = 0;

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }
        this.simOdometry = this.odometer.getPoseMeters();
        AutoBuilder.configure(
                this::getPose,
                this::resetOdometry,
                this::getChassisSpeeds,
                this::setChassisSpeeds,
                new PPHolonomicDriveController(
                        new PIDConstants(0.5, 0.0, 0.0),
                        new PIDConstants(0.5, 0.0, 0.0)),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);
        SmartDashboard.putData("SwerveDrive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("FrontLeft Posisiton", () -> frontLeft.getPosition().angle.getRadians(),
                        null);
                builder.addDoubleProperty("FrontLeft Velocity", () -> frontLeft.getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("FrontRight Posisiton", () -> frontRight.getPosition().angle.getRadians(),
                        null);
                builder.addDoubleProperty("FrontRight Velocity", () -> frontRight.getState().speedMetersPerSecond,
                        null);

                builder.addDoubleProperty("BackLeft Posisiton", () -> backLeft.getPosition().angle.getRadians(), null);
                builder.addDoubleProperty("BackLeft Velocity", () -> backLeft.getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("BackRight Posisiton", () -> backRight.getPosition().angle.getRadians(),
                        null);
                builder.addDoubleProperty("BackRight Velocity", () -> backRight.getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Robot Heading", () -> getRotation2d().getRadians(), null);
            }
        });
    }
    /*
     * public SwerveModulePosition[] getModulePositions() {
     * return new SwerveModulePosition[] {
     * frontLeft.getPosition(),
     * frontRight.getPosition(),
     * backLeft.getPosition(),
     * backRight.getPosition()
     * };
     * }
     */

    public ChassisSpeeds getChassisSpeeds() {

        return DriveConstants.kDriveKinematics.toChassisSpeeds(
                new SwerveModuleState[] {
                        frontLeft.getState(),
                        frontRight.getState(),
                        backLeft.getState(),
                        backRight.getState()
                });
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        if (Robot.isSimulation()) {
            this.simDrive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
            SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
            this.setModuleStates(states);
            // SmartDashboard.putString("badddddd", speeds.toString());
            // SmartDashboard.putNumber("test", speeds.vxMetersPerSecond);
        } else {
            SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
            setModuleStates(states);
        }
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        StatusSignal<Angle> yawSignal = gyro.getYaw();
        double yawValue = yawSignal.getValueAsDouble();
        return Math.IEEEremainder(yawValue, 360);
    }

    public Rotation2d getRotation2d() {
        var yaw = gyro.getYaw();
        yaw.refresh();
        double rad = yaw.getValue().in(Radians);
        // rad = -rad; // 視情況取負號
        return new Rotation2d(rad);
    }

    /*
     * public Rotation2d getRotation2d() {
     * return Rotation2d.fromDegrees(getHeading());
     * }
     */

    public Pose2d getPose() {
        if (Robot.isSimulation())
            return this.simOdometry;
        else
            return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        if (Robot.isSimulation()) {
            this.simOdometry = pose;
            this.posedX = pose.getX();
            this.posedY = pose.getY();
            this.posedRot = pose.getRotation().getRadians();
        } else {
            odometer.resetPosition(
                    getRotation2d(),
                    new SwerveModulePosition[] {
                            frontLeft.getPosition(),
                            frontRight.getPosition(),
                            backLeft.getPosition(),
                            backRight.getPosition()
                    },
                    pose);
        }
    }

    @Override
    public void periodic() {
        this.updateTime();
        odometer.update(
                getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });
        SmartDashboard.putNumber("Robot Heading", getHeading());
        // SmartDashboard.putString("Robot Location",
        // getPose().getTranslation().toString());
        SmartDashboard.putString("Robot Location", getPose().toString());
        this.realSwerve.set(this.getPose());
        this.simPublisher.set(this.simOdometry);
        this.publisher.set(this.getModuleStates());

        SmartDashboard.putData("Field", m_field);
        m_field.setRobotPose(getPose());
    }

    // 停止所有輪組的運動
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void driveSwerve(double xSpeed, double ySpeed, double rotSpeed) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, getRotation2d());
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        this.setModuleStates(moduleStates);
    }

    public void autoTurning(double xSpeed, double ySpeed, double angle) {
        double speed = (angle == 0) ? 0 : this.pidController.calculate(angle, 0);
        this.driveSwerve(xSpeed, ySpeed, speed);
    }

    // 設定每個輪組的目標狀態
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        frontLeft.setDesiredState(desiredStates[0]);
        // 前左輪

        frontRight.setDesiredState(desiredStates[1]);
        // 前右輪

        backLeft.setDesiredState(desiredStates[2]);
        // 後左輪

        backRight.setDesiredState(desiredStates[3]);
        // 後右輪
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                this.frontLeft.getState(),
                this.frontRight.getState(),
                this.backLeft.getState(),
                this.backRight.getState()
        };
    }

    public void updateTime() {
        this.timeFromLastUpdate = Timer.getFPGATimestamp() - this.lastSimTime;
        this.lastSimTime = Timer.getFPGATimestamp();
    }

    public void simDrive(double xspeed, double ySpeed, double rotSpeed) {
        ChassisSpeeds speeds = new ChassisSpeeds(xspeed, ySpeed, rotSpeed);
        SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        this.setModuleStates(states);
        // SmartDashboard.putNumber("test/x", speeds.vxMetersPerSecond);
        // SmartDashboard.putNumber("test/y", speeds.vyMetersPerSecond);
        // SmartDashboard.putNumber("test/rot", speeds.omegaRadiansPerSecond);
        this.publisher.set(states);
        if (speeds.vxMetersPerSecond != 0) {
            this.posedX += speeds.vxMetersPerSecond * timeFromLastUpdate;
        }
        if (speeds.vyMetersPerSecond != 0) {
            this.posedY += speeds.vyMetersPerSecond * timeFromLastUpdate;
        }
        if (speeds.omegaRadiansPerSecond != 0) {
            this.posedRot += speeds.omegaRadiansPerSecond * timeFromLastUpdate;
        }
        this.simOdometry = new Pose2d(this.posedX, this.posedY, new Rotation2d(this.posedRot));
    }

}
