package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.HardwareSoftware;

@Autonomous(name="Odometry Turn Test Code")

public class OdometryTurnTest extends LinearOpMode {

    HardwareSoftware robot = new HardwareSoftware();


    public void odoTurn(int turnDegrees) {


        //tuning variables
        int tolerance = 300; //how close the robot needs to be to the target

        int[] distanceCurve = {50000, 25000, 15000, 10000, 500}; //these 2 arrays should be same length with each value corresponding to the value in the same place at the other array.

        double[] powerCurve = {0.5, 0.3, 0.2, 0.15, 0.1}; // You put in a distance from target for the distance array and a motor power for the power array.

        int radius = 0; //set this to the distance between the encoder wheel and the center of rotation on the robot


        double circumferenceCm = 2*radius*Math.PI; //circumference of an imaginary circle from the center of the robot to the encoder

        double distanceCm = (circumferenceCm / 360)*turnDegrees; //taking total circumference and finding the length per degree of the circle

        int targetEncoderValue = (int)(distanceCm*522); //522 is the number of ticks the encoder turns per centimeter of distance it moves

        int distanceToTarget = targetEncoderValue - robot.turnEncoder().getCurrentPosition(); //taking the target position and finding how far it is from it

        int distanceMagnitude = Math.abs(distanceToTarget); //distance regardless of direction

        double motorPower = 0;


        while (-tolerance > distanceToTarget || distanceToTarget > tolerance){ //same while loop twice so that it can wait a moment then double check that it is within the target range

            while (-tolerance > distanceToTarget || distanceToTarget > tolerance) {

                distanceMagnitude = Math.abs(distanceToTarget);

                motorPower = 0.8;

                for (int i = 0; i <= (distanceCurve.length-1); i++) {
                    if (distanceMagnitude < distanceCurve[i]){
                        motorPower = powerCurve[i];
                    }
                }

                if (distanceToTarget < 0) {
                    motorPower = -motorPower;
                }

                runDriveTrain(motorPower, -motorPower);

                distanceToTarget = targetEncoderValue - robot.turnEncoder().getCurrentPosition();

                int degreesTurned = (int)((robot.turnEncoder().getCurrentPosition() * 522) / (circumferenceCm / 360));

                telemetry.addData("Power:", motorPower);
                telemetry.addData("Distance:", distanceToTarget);
                telemetry.addData ("Degrees Turned:", degreesTurned);
                telemetry.update();

            }

            runDriveTrain(0, 0);

            sleep(500);

            distanceToTarget = targetEncoderValue - robot.turnEncoder().getCurrentPosition();

            //sending data to driver hub for debugging and tuning
            telemetry.addData("Power:", motorPower);
            telemetry.addData("Distance:", distanceToTarget);
            telemetry.update();

        }

        odoReset(); //for future uses of command

    }

    public void runDriveTrain(double leftPower, double rightPower){
        //sets power for all drivetrain motors
        //left motors negative because they are reversed on drivetrain
        robot.frontLeft().setPower(-leftPower);
        robot.backLeft().setPower(-leftPower);
        robot.frontRight().setPower(rightPower);
        robot.backRight().setPower(rightPower);
    }

    public void odoReset() {
        //resets the encoder to zero
        robot.turnEncoder().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turnEncoder().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        odoReset();


        waitForStart();


        odoTurn(90);
    }
}