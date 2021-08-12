import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.UtilEverything;
import org.junit.Test;

import java.util.Arrays;

import static frc.robot.Constants.SwerveConstants.*;


public class TestClass extends SubsystemBase {

    Translation2d frSkew = new Translation2d(SKEW_X_FR_METERS, SKEW_Y_FR_METERS);
    Translation2d flSkew = new Translation2d(SKEW_X_FL_METERS, SKEW_Y_FL_METERS);
    Translation2d rrSkew = new Translation2d(SKEW_X_RR_METERS, SKEW_Y_RR_METERS);
    Translation2d rlSkew = new Translation2d(SKEW_X_RL_METERS, SKEW_Y_RL_METERS);
    Translation2d[] wheels = {frSkew, flSkew, rrSkew, rlSkew};
    SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(wheels);

    double robotAngle = 0;
    double[] initAngles = {0, 0, 0, 0};

    public void setChassisSpeeds(ChassisSpeeds desiredChassisSpeeds) {
        desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                desiredChassisSpeeds.vxMetersPerSecond,
                desiredChassisSpeeds.vyMetersPerSecond,
                desiredChassisSpeeds.omegaRadiansPerSecond,
                new Rotation2d(Math.toRadians(robotAngle)));

        SwerveModuleState[] swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
        Arrays.stream(swerveModuleStates).forEach(e -> e.angle = new Rotation2d(Math.toRadians(UtilEverything.toPositive(e.angle.getDegrees()))));

        System.out.println("Pre setModuleStates states:");
        Arrays.stream(swerveModuleStates).forEach(e -> System.out.println(e));
        System.out.println();

        setModuleStates(swerveModuleStates);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        for (int i = 0; i < 4; i++) {
            desiredStates[i] = SwerveModuleState.optimize(desiredStates[i], new Rotation2d(Math.toRadians((initAngles[i]))));
            desiredStates[i].angle = new Rotation2d(Math.toRadians(UtilEverything.toPositive(desiredStates[i].angle.getDegrees())));
            System.out.println(desiredStates[i]);
            setAngle(desiredStates[i].angle.getDegrees());
        }
    }

    private void setAngle(double angle) {
        PIDController errorCalculator = new PIDController(1, 0, 0);
        errorCalculator.enableContinuousInput(-180, 180);

        double error = errorCalculator.calculate(initAngles[0], angle);
        double percentOutput = UtilEverything.map(error, -90, 90, -1, 1);

        PIDController pidController = new PIDController(1, 0, 0);
        percentOutput = pidController.calculate(0, percentOutput);

        System.out.println("percentOutput: " + percentOutput);
    }

    @Test
    public void testFunction() {
        setChassisSpeeds(new ChassisSpeeds(0, -1, 0));
    }

    @Test
    public void testFunction2() {
        PIDController errorCalculator = new PIDController(1, 0, 0);
        double robotAngle = 0;
//        double angle = Math.toDegrees(Math.atan2(rightY, rightX));
        double angle = 181;
        errorCalculator.enableContinuousInput(-180, 180);
        double error = errorCalculator.calculate(robotAngle, angle);
        double mappedError = UtilEverything.map(error, 0, 360, -1, 1);
        System.out.println(error);
    }

    @Test
    @Override
    public void periodic() {
        XboxController xboxController = new XboxController(1);
        while (true) {
            xboxController.setRumble(GenericHID.RumbleType.kRightRumble, 1);
            System.out.println(xboxController.getButtonCount());
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}


