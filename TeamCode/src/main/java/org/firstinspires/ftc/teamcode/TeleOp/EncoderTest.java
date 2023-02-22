package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FusionFramework.GyroSensor;
import org.firstinspires.ftc.teamcode.HardwareSoftware;

@Disabled
@TeleOp(name="Encoder Motor Test")
public class EncoderTest extends OpMode {

    HardwareSoftware robot = new HardwareSoftware();
    GyroSensor gyro;
    private   BNO055IMU.Parameters    parameters;
    boolean useCompassMode = false;
    boolean inCompassMode;

    @Override
    public void init() {
        robot.init(hardwareMap);
        gyro = new GyroSensor(robot, false);

        // Initialize the parameters for the IMU so it operates the way we want
        parameters = new BNO055IMU.Parameters();
        if (useCompassMode) {
            parameters.mode = BNO055IMU.SensorMode.COMPASS; // IMU;
            inCompassMode = true;
        } else {
            parameters.mode = BNO055IMU.SensorMode.IMU;
            inCompassMode = false;
        }
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        // TODO fix later




    }

    @Override
    public void loop() {


        robot.getImu().initialize(parameters);

        telemetry.addData("IMU Error: ", robot.getImu().getSystemStatus());

        telemetry.addData("Front Right: ", robot.frontRight().getCurrentPosition());
        telemetry.addData("Front Left: ", robot.frontLeft().getCurrentPosition());
        telemetry.addData("Back Right: ", robot.backRight().getCurrentPosition());
        telemetry.addData("Back Left: ", robot.backLeft().getCurrentPosition());
//        telemetry.addData("Gyro heading: ", gyro.getHeading());

        telemetry.update();

    }
}