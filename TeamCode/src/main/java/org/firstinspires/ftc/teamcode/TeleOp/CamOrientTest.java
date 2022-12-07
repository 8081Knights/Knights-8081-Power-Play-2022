package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ConeOrientate;
import org.firstinspires.ftc.teamcode.FusionFramework.DriveTrainIntf;
import org.firstinspires.ftc.teamcode.HardwareSoftware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "Camera Orientation Test")

public class CamOrientTest extends OpMode {

    HardwareSoftware robot = new HardwareSoftware();
    RobotCommands commands;
    ConeOrientate detector = new ConeOrientate();
    OpenCvCamera camera;


    @Override
    public void init() {
        robot.init(hardwareMap);
        camera = robot.getFrontWebCam();


        camera.setPipeline(detector);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

    }

    @Override
    public void loop() {


        telemetry.addData("Silly cone position: ", detector.conePos());
        telemetry.update();

    }
}
