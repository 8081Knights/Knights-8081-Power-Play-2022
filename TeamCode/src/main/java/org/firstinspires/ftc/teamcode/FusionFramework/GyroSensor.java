package org.firstinspires.ftc.teamcode.FusionFramework;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.HardwareSoftware;

public class GyroSensor {

    private   HardwareSoftware              robot;
    private   BNO055IMU //LynxEmbeddedIMU
                                      imu; // the gyro in the Control Hub
    private   BNO055IMU.Parameters    parameters;
    private   boolean                 inCompassMode = false;
    private float zeroPosition = 0;
    private float X_Angle = 0;
    private float Y_Angle = 0;
    private float Z_Angle = 0;

    public GyroSensor(HardwareSoftware rs, boolean useCompassMode) {
        robot = rs; // remember the object ot get HW information from

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
        imu = rs.getImu();

        imu.initialize(parameters);

    }

    public boolean isIMUready() {
        // Only check for calibration if using the Gryo/IMU mode
        if (inCompassMode)
            return true;
        return imu.isGyroCalibrated();
    }

    public void initializeZeroPosition() {
        zeroPosition = 0;
        getHeading();
        zeroPosition = adjustAngle( Y_Angle, Z_Angle);
    }

    public float getZeroPosition() {
        return zeroPosition;
    }

    // Adjust the quaternion values to orientation based on Y-Axis and Z-Axis
    // returns a degree heading in compass degrees 0 to 360
    protected float adjustAngle(float angle, float Z) {
        // Adjust position to 360 degrees
        if ( Z < 0 ) {
            if (angle < 0) {
                angle *= -1; // since angle is negative, just change sign to get 0 to 90
            } else {
                angle = 360-angle;  // since angle is positive, subtract to give 0 to 270
                if (angle == 360) {angle = 0;} // make 360 == 0
            }
        } else { // Z_Angle is positive
            if (angle > 0) {
                angle += 180;  // since angle is positive gives value 180 to 270
            } else {
                angle += 180;  // since angle is negative gives value 90 to 180
            }
        }
        // angle angle to be zeroed by initial position
        angle -= zeroPosition;
        if ( angle < 0) angle += 360;

        // reverse the symmetry of the degrees to reflect "CLOCKWISE" compass directions
        if ( angle > 180) {
            angle -= 180;
        } else {
            angle += 180;
            if (angle == 360) angle = 0;
        }
        return angle;
    }

    public int getHeading() {
        float angle = 0;

        // get current heading in range -180 < angle <= 180
        Orientation angleSet = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        X_Angle = angleSet.firstAngle;
        Y_Angle = angleSet.secondAngle;
        Z_Angle = angleSet.thirdAngle;

        angle = adjustAngle(Y_Angle, Z_Angle);

        // adjust angle to appropriate range (-180, 180) and compass 'standard' direction
        if ( angle > 180) {
            angle = 360-angle;
        } else {
            angle *= -1;
        }
        return (int)angle;
    }

    /*
    This method returns the angle to turn in order to get the robot to be on the request heading
    The returned angle is between -180 and 180 degrees, in order to give the shortest turn arc to position to requested heading
    If the angle is negative, then the robot should turn COUNTER-CLOCKWISE
    If the angle is positive, then the robot should turn CLOCKWISE
     */
    public int getDegreesToHeading(int heading) {
        // adjust heading to ensure we are using the -180 to 180 range
        if (heading > 180) { heading = 360 - heading; }

        // Read the gyro's heading and determine the angle we must turn to get to requested heading
        int angle = heading - getHeading();

        // Adjust the angle to be within parameters
        // Example: If requested heading is -50 and current angle is 20, then -50 - 20 = -70
        // Example: If requested heading is -170 and current angle is 170, then -170 - 170 = (-340 +360)*-1 = 20
        // Example: If requested heading is 170 and current angle is -160, then 170 - -160 = (330 -360)*-1 = 30
        if (angle < -180 ) {
            angle = (angle - 360)*-1; // turn in the opposite direction to get the smaller arc
        } else if (angle > 180) {
            angle = (angle -360)*-1;
        }
        // If angle returned is positive, then must turn clockwise
        // If angle returned is negative, then must turn counter-clockwise
        return heading - angle;
    }

    public int getXangle() {
        return (int) X_Angle;
    }
    public int getYangle() {
        return (int) Y_Angle;
    }
    public int getZangle() {
        return (int) Z_Angle;
    }

}
