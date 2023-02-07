package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareSoftware;


@TeleOp(name="Field Centric Driver Test")
public class FieldCentricDT extends OpMode {
    HardwareSoftware robot = new HardwareSoftware();
    RobotCommands commands = new RobotCommands();


    /**
     * you can change the variable names to make more sense
     */
    double driveTurn;
    //double driveVertical;
    //double driveHorizontal;

    double gamepadXCoordinate;
    double gamepadYCoordinate;
    double gamepadHypot;
    double gamepadDegree;
    double robotDegree;
    double movementDegree;
    double gamepadXControl;
    double gamepadYControl;


    @Override
    public void init() {
        robot.init(hardwareMap);

        robot.gyro().initialize();
        robot.gyro().calibrate();

        while(robot.gyro().isCalibrating()){

            telemetry.addLine("Gyro is calibrating");
            telemetry.update();

        }

        telemetry.clear();

        telemetry.addLine("Gyro Calibrated");
        telemetry.update();


    }

    @Override
    public void loop() {
        driveTurn = -gamepad1.right_stick_x;
        //driveVertical = -gamepad1.right_stick_y;
        //driveHorizontal = gamepad1.right_stick_x;

        gamepadXCoordinate = gamepad1.left_stick_x; //this simply gives our x value relative to the driver
        gamepadYCoordinate = -gamepad1.left_stick_y; //this simply gives our y vaue relative to the driver



        //the inverse tangent of opposite/adjacent gives us our gamepad degree
        robotDegree = Math.toRadians(robot.gyro().getHeading());


        double rotX = gamepadXCoordinate * Math.cos(robotDegree) - gamepadYCoordinate * Math.sin(robotDegree);
        double rotY = gamepadXCoordinate * Math.sin(robotDegree) + gamepadYCoordinate * Math.cos(robotDegree);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(driveTurn), 1);
        double frontLeftPower = (rotY + rotX - driveTurn) / denominator;
        double backLeftPower = (-rotY + rotX + driveTurn) / denominator;
        double frontRightPower = (rotY + rotX + driveTurn) / denominator;
        double backRightPower = (-rotY + rotX - driveTurn) / denominator;

        robot.frontRight().setPower(frontRightPower);
        robot.frontLeft().setPower(frontLeftPower);
        robot.backRight().setPower(backRightPower);
        robot.backLeft().setPower(backLeftPower);


    }
}
