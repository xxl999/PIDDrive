package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;


public class PIDDrive {
    private ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
    
    private final PWMSparkMax m_leftMotor = new PWMSparkMax();
    private final PWMSparkMax m_rightMotor = new PWMSparkMax();
    private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    private final Encoder m_leftEncoder = new Encoder();
    private final Encoder m_rightEncoder = new Encoder();
    

    private final PIDController distancePID = new PIDController(0.0, 0.0, 0.0);
    private final PIDController turnPID = new PIDController(0.0, 0.0, 0.0);

    double wheelDiameterMeters =  ;
    int encoderCPR =  ;
    double distancePerPulse = Math.PI * wheelDiameterMeters / encoderCPR;
    


public PIDDrive() {
    m_gyro.calibrate()
    m_leftEncoder.setDistancePerPulse(distancePerPulse);
    m_rightEncoder.setDistancePerPulse(distancePerPulse);

    
    distancePID.setSetpoint(2.0);
    distancePID.setTolerance(0.05);
    turnPID.setSetpoint(90.0);
    turnPID.setTolerance(2.0);
}
    public void arcadeDrive(double forward, double turn) {
    m_robotDrive.arcadeDrive(forward, turn);
}

    public void update()  {
        double averageDistance = (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
        distancePID.setSetpoint(2.0);
        switch (currentState) {
        case MOVE_FORWARD_1:
                double forwardSpeed = distancePID.calculate(averageDistance);
                forwardSpeed = MathUtil.clamp(forwardSpeed,high);
                arcadeDrive(forwardSpeed, turn);

                if (distancePID.atSetpoint()) {
                    distancePID.reset();
                    currentState = State.TURN_RIGHT;
                }
                break;

            case MOVE_FORWARD_2:
                double forwardSpeed2 = distancePID.calculate(averageDistance);
                forwardSpeed2 = MathUtil.clamp(forwardSpeed2, high);
                arcadeDrive(forwardSpeed2, turn);

                if (distancePID.atSetpoint()) {
                    arcadeDrive(forward,turn);
                    currentState = State.STOP;
                }
                break;

            case STOP:
                arcadeDrive(0, 0);
                break;
    }
}
}