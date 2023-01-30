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
        driveTurn = -gamepad1.left_stick_x;
        //driveVertical = -gamepad1.right_stick_y;
        //driveHorizontal = gamepad1.right_stick_x;

        gamepadXCoordinate = gamepad1.right_stick_x; //this simply gives our x value relative to the driver
        gamepadYCoordinate = -gamepad1.right_stick_y; //this simply gives our y vaue relative to the driver

        gamepadHypot = Range.clip(Math.hypot(gamepadXCoordinate, gamepadYCoordinate), 0, 1);


        //finds just how much power to give the robot based on how much x and y given by gamepad
        //range.clip helps us keep our power within positive 1
        // also helps set maximum possible value of 1/sqrt(2) for x and y controls if at a 45 degree angle (which yields greatest possible value for y+x)
        gamepadDegree = Math.atan2(gamepadYCoordinate, gamepadXCoordinate);
        //the inverse tangent of opposite/adjacent gives us our gamepad degree
        robotDegree = robot.gyro().getHeading();
        //gives us the angle our robot is at
        movementDegree = gamepadDegree - robotDegree;
        //adjust the angle we need to move at by finding needed movement degree based on gamepad and robot angles
        gamepadXControl = Math.cos(Math.toRadians(movementDegree)) * gamepadHypot;
        //by finding the adjacent side, we can get our needed x value to power our motors
        gamepadYControl = Math.sin(Math.toRadians(movementDegree)) * gamepadHypot;
        //by finding the opposite side, we can get our needed y value to power our motors

        /**
         * again, make sure you've changed the motor names and variables to fit your team
         */


        telemetry.addData("Gyro Heading: ", robotDegree);
        telemetry.addData("Gamepad Heading: ", gamepadDegree);
        telemetry.addData("Gamepad Hypot: ", gamepadHypot);

        //by mulitplying the gamepadYControl and gamepadXControl by their respective absolute values, we can guarantee that our motor powers will not exceed 1 without any driveTurn
        //since we've maxed out our hypot at 1, the greatest possible value of x+y is (1/sqrt(2)) + (1/sqrt(2)) = sqrt(2)
        //since (1/sqrt(2))^2 = 1/2 = .5, we know that we will not exceed a power of 1 (with no turn), giving us more precision for our driving
        robot.frontRight().setPower(gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) + driveTurn);
        robot.backRight().setPower(gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) + driveTurn);
        robot.frontLeft().setPower(gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) - driveTurn);
        robot.backLeft().setPower(gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) - driveTurn);

    }
}
