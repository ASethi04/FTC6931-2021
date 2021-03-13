package org.firstinspires.ftc.teamcode.Odometry;

import android.os.Debug;
import android.util.Log;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.motors.REVHDHEXHUB_1291;
import org.opencv.core.Point;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.io.Console;
import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Odometry.MathFunctions.AngleWrap;
import static org.firstinspires.ftc.teamcode.Odometry.MathFunctions.lineCircleIntersection;
import com.qualcomm.hardware.bosch.BNO055IMU;


public class RobotMovement {
    public Integer cpr = 28; //counts per rotation
    public Integer gearratio = 40;
    public Double diameter = 4.125;
    private static final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(REVHDHEXHUB_1291.class);



    public static void followCurve(double COUNTS_PER_INCH, Telemetry telemetry, ArrayList<CurvePoint> allPoints, double followAngle, OdometryGlobalCoordinatePosition position, DcMotor right_front, DcMotor right_back, DcMotor left_front, DcMotor left_back, BNO055IMU imu)
    {
        for(int i = 0; i < allPoints.size() - 1; i++)
        {
            //ComputerDebugging.sendLine(new FloatPoint(allPoints.get(i).x, allPoints.get(i).y), new FloatPoint(allPoints.get(i+1).x,allPoints.get(i+1).y));
        }

        CurvePoint followMe = getFollowPointPath(allPoints, new Point(position.returnXCoordinate(), position.returnYCoordinate()),  allPoints.get(0).followDistance, position);
        //This follow distance is not always the same; write function to get follow distance; use perpendicular line
       // ComputerDebugging.sendKeyPoint(new FloatPoint(followMe.x, followMe.y));

        goToPosition(COUNTS_PER_INCH, telemetry, right_front, right_back, left_front, left_back, imu, position, followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed);

    }


    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius, OdometryGlobalCoordinatePosition position) {
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for (int i = 0; i < pathPoints.size() - 1; i++) {
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);

            ArrayList<Point> intersections = lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint());

            double closestAngle = 1000000;

            for(Point thisIntersection: intersections)
            {
                Log.d("intersectionX", String.valueOf(thisIntersection.x));
                Log.d("intersectionY", String.valueOf(thisIntersection.y));
                double angle = Math.atan2(thisIntersection.y - position.returnYCoordinate(), thisIntersection.x-position.returnXCoordinate());
                double deltaAngle = Math.abs(MathFunctions.AngleWrap(angle - position.returnOrientation()));

                if(deltaAngle < closestAngle)
                {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }
        Log.d("POINTX", String.valueOf(followMe.x));
        Log.d("POINTY", String.valueOf(followMe.y));

        return followMe;


    }


        /**
         *
         * @param x
         * @param y
         * @param movementSpeed
         */
        public static void goToPosition(double COUNTS_PER_INCH, Telemetry tele, DcMotor right_front, DcMotor right_back, DcMotor left_front, DcMotor left_back, BNO055IMU imu, OdometryGlobalCoordinatePosition position, double x, double y, double movementSpeed, double preferredAngle, double turnSpeed)
        {
            double distanceToTarget = Math.hypot(x - position.returnXCoordinate() / COUNTS_PER_INCH, y - position.returnYCoordinate() / COUNTS_PER_INCH);
            tele.addData("position x", position.returnXCoordinate());
            tele.addData("position y", position.returnYCoordinate());
            tele.addData("distance", distanceToTarget);
            double absoluteAngleToTarget = Math.atan2(y - position.returnYCoordinate() / COUNTS_PER_INCH, x - position.returnXCoordinate() / COUNTS_PER_INCH);
            tele.addData("angle to target", absoluteAngleToTarget);
            double relativeAngleToPoint = MathFunctions.AngleWrap(absoluteAngleToTarget - (Math.toRadians(position.returnOrientation()) - Math.toRadians(90)));
            tele.addData("rel angle to target", relativeAngleToPoint);

            double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
            tele.addData("rel x to point", relativeXToPoint);

            double relativeYToPont = Math.sin(relativeAngleToPoint) * distanceToTarget;
            tele.addData("rel y to point", relativeYToPont);

            double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPont));
            double movementYPower = relativeYToPont / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPont));
            tele.addData("movement x power is ", movementXPower);
            tele.addData("movement y power is ", movementYPower);
            tele.update();
            left_back.setPower(movementXPower * movementSpeed);
            right_front.setPower(movementXPower * movementSpeed);
            left_front.setPower(movementYPower * movementSpeed);
            right_back.setPower(movementYPower * movementSpeed);


            double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
            //movement turn
            //turnWithGyro(left_front, left_back, right_front, right_back, imu, Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed, 0.5);

            //if(distanceToTarget < 10){
            //turnWithGyro(left_front, left_back, right_front, right_back, imu, 0, 0.5);
        }




    //

    }