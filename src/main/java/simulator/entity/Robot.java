
package simulator.entity;

import javax.swing.*;
import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.Rectangle2D;
import java.util.logging.Level;
import java.util.logging.Logger;

public class Robot extends JComponent {
    private final Rectangle2D body;
    private final Rectangle2D leftWheel;
    private final Rectangle2D rightWheel;
    private final Dimension size;
    private final double distanceBetweenFrontBackWheels;
    private Point currentLocation;
    private Point currentLocationCenter;
    private final AffineTransform bodyAffineTransform = new AffineTransform();
    private final AffineTransform leftWheelAffineTransform = new AffineTransform();
    private final AffineTransform rightWheelAffineTransform = new AffineTransform();

    private double speed;
    private double directionInDegrees = -90;
    private final double distancePerTick; // the distance the robot moves per tick
    private final double angleChangePerClick;
    // need to incorporate concept of speed later
    private double turningRadius = 0;
    private double turningAngleDegrees = 0;
    private static final double THETA_CAR = 270; // this is facing up
    private double thetaWheelsDegree = 0; // with reference to the front of car
    // since the coordinate system is positive x is right, positive y is down,
    // angles work by starting from positive x and going clockwise

    private final Logger logger;

    private boolean firstTime = true;

    public Robot(Dimension size, Point startingPoint) {
        logger = Logger.getLogger(Robot.class.getName());

        this.size = size;
        this.setCurrentLocationCenter(startingPoint);
        body = new Rectangle2D.Double(0, 0, size.getWidth(), size.getHeight());
        leftWheel = new Rectangle2D.Double(0, 0, size.getWidth() / 5, size.getHeight() / 3);
        rightWheel = new Rectangle2D.Double(0, 0, size.getWidth() / 5, size.getHeight() / 3);

        distanceBetweenFrontBackWheels = size.getHeight() * 2 / 3;
        distancePerTick = 1;
        angleChangePerClick = 5;

        setOpaque(false);
    }

    // can combine with turnRight and accept one parameter for the angle to turn
    public void turnLeft() {
        if (this.thetaWheelsDegree > -20) {
            this.thetaWheelsDegree -= angleChangePerClick;
            this.leftWheelAffineTransform.rotate(Math.toRadians(-angleChangePerClick));
            this.rightWheelAffineTransform.rotate(Math.toRadians(-angleChangePerClick));
            doWheelsTurned();
            this.repaint();
        }
    }

    public void turnRight() {
        if (this.thetaWheelsDegree < 20) {
            this.thetaWheelsDegree += angleChangePerClick;
            this.leftWheelAffineTransform.rotate(Math.toRadians(angleChangePerClick));
            this.rightWheelAffineTransform.rotate(Math.toRadians(angleChangePerClick));
            doWheelsTurned();
            this.repaint();
        }
    }

    public void moveForward() {
        double angleDegrees = THETA_CAR + turningAngleDegrees;

        double dX = this.distancePerTick * Math.cos(Math.toRadians(angleDegrees));
        double dY = this.distancePerTick * Math.sin(Math.toRadians(angleDegrees));

        this.bodyAffineTransform.translate(dX, dY);

        this.bodyAffineTransform.rotate(Math.toRadians(this.turningAngleDegrees));
        directionInDegrees += this.turningAngleDegrees;
        if (directionInDegrees > 180) {
            directionInDegrees -= 360;
        } else if (directionInDegrees < -180) {
            directionInDegrees += 360;
        }

        logger.log(Level.FINE, "X = " + this.bodyAffineTransform.getTranslateX() + ", Y = "
                + this.bodyAffineTransform.getTranslateY());

        this.repaint();
    }

    public void moveBackward() {
        double angleDegrees = THETA_CAR - 180 - this.turningAngleDegrees;

        double dX = this.distancePerTick * Math.cos(Math.toRadians(angleDegrees));
        double dY = this.distancePerTick * Math.sin(Math.toRadians(angleDegrees));

        this.bodyAffineTransform.translate(dX, dY);

        this.bodyAffineTransform.rotate(-Math.toRadians(this.turningAngleDegrees));
        directionInDegrees -= this.turningAngleDegrees;
        if (directionInDegrees > 180) {
            directionInDegrees -= 360;
        } else if (directionInDegrees < -180) {
            directionInDegrees += 360;
        }

        logger.log(Level.FINE, "X = " + this.bodyAffineTransform.getTranslateX() + ", Y = "
                + this.bodyAffineTransform.getTranslateY());

        this.repaint();
    }

    // calculate turningRadius and turningAngleDegrees
    // based on distance between front and back wheel axis
    private void doWheelsTurned() {
        this.turningRadius = Math.abs(
                this.distanceBetweenFrontBackWheels * Math.tan(Math.PI / 2 - Math.toRadians(this.thetaWheelsDegree)));

        this.turningAngleDegrees = this.turningRadius == 0 ? 0 : Math.toDegrees(this.distancePerTick / this.turningRadius);
        this.turningAngleDegrees = this.thetaWheelsDegree > 0 ? turningAngleDegrees : -turningAngleDegrees;
    }

    // using currentLocation, calculate the center point
    private Point calculateCenter() {
        return new Point((int) (currentLocation.getX() + size.getWidth() / 2),
                (int) (currentLocation.getY() + size.getHeight() / 2));
    }

    // using currentLocationCenter, calculate the top left point
    private Point calculateTopLeftPoint() {
        return new Point((int) (currentLocationCenter.getX() - size.getWidth() / 2),
                (int) (currentLocationCenter.getY() - size.getHeight() / 2));
    }

    public void paintComponent(Graphics g) {
        Graphics2D g2 = (Graphics2D) g;

        // initial transformation to starting point
        if (firstTime) {
            this.bodyAffineTransform.setToIdentity();
            this.bodyAffineTransform.translate(this.size.getWidth() / 2, this.size.getHeight() / 2);
            this.bodyAffineTransform.translate(this.currentLocation.x, this.currentLocation.y);

            this.leftWheelAffineTransform.setToIdentity();
            // for size of wheel
            this.leftWheelAffineTransform.translate(this.size.getWidth() / 10, this.size.getHeight() / 6);
            // position wheel relative to center of car
            this.leftWheelAffineTransform.translate(this.size.getWidth() / 5, this.size.getHeight() / 5);

            this.rightWheelAffineTransform.setToIdentity();
            this.rightWheelAffineTransform.translate(this.size.getWidth() / 10, this.size.getHeight() / 6);
            this.rightWheelAffineTransform.translate(3 * size.getWidth() / 5, size.getHeight() / 5);

            firstTime = false;
        }

        AffineTransform toCenterAt;

        // transform body of to the starting point
        toCenterAt = new AffineTransform();
        toCenterAt.concatenate(this.bodyAffineTransform);
        toCenterAt.translate(-this.size.getWidth() / 2, -this.size.getHeight() / 2);
        g2.transform(toCenterAt);

        // save this transformation to apply to the other wheels
        // so that overall car transformation will apply to them too
        AffineTransform saveXform = g2.getTransform();

        // draw car body
        g2.setColor(Color.lightGray);
        g2.fill(body);

        // draw left wheel
        //
        g2.setTransform(saveXform);
        toCenterAt = new AffineTransform();
        toCenterAt.concatenate(this.leftWheelAffineTransform);
        toCenterAt.translate(-this.size.getWidth() / 10, -this.size.getHeight() / 6);
        g2.transform(toCenterAt);

        g2.setColor(Color.darkGray);
        g2.fill(leftWheel);

        g2.setTransform(saveXform);
        toCenterAt = new AffineTransform();
        toCenterAt.concatenate(this.rightWheelAffineTransform);
        toCenterAt.translate(-this.size.getWidth() / 10, -this.size.getHeight() / 6);
        g2.transform(toCenterAt);

        g2.fill(rightWheel);
    }

    /**
     * Get the centre point of the robot.
     *
     * @return Point object of the centre of the robot.
     */
    public MyPoint getCurrentLocation() {
        return new MyPoint((int) this.bodyAffineTransform.getTranslateX(), (int) this.bodyAffineTransform.getTranslateY(), getGeneralDirection());
    }

    // when setting currentLocation, calculate and set currentLocationCenter
    public void setCurrentLocation(Point loc) {
        this.currentLocation = loc;
        this.currentLocationCenter = this.calculateCenter();
    }

    // when setting currentLocationCenter, calculate and set currentLocation
    public void setCurrentLocationCenter(Point loc) {
        this.currentLocationCenter = loc;
        this.currentLocation = this.calculateTopLeftPoint();
    }

    public AffineTransform getBodyAffineTransform() {
        return bodyAffineTransform;
    }

    public double getDirectionInDegrees() {
        return directionInDegrees;
    }

    public Direction getGeneralDirection() {
        if (directionInDegrees >= -45 && directionInDegrees <= 45) {
            return Direction.EAST;
        } else if (directionInDegrees >= -135 && directionInDegrees <= -45) {
            return Direction.NORTH;
        } else if (directionInDegrees >= 45 && directionInDegrees <= 135) {
            return Direction.SOUTH;
        } else if ((directionInDegrees >= 135 && directionInDegrees <= 180) || (directionInDegrees >= -180 && directionInDegrees <= -135)) {
            return Direction.WEST;
        }

        return Direction.NONE;
    }
}