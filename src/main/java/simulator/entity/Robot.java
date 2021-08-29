
package simulator.entity;

import javax.swing.*;
import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.Rectangle2D;
import java.util.logging.Level;
import java.util.logging.Logger;

public class Robot extends JComponent {
    private Rectangle2D body;
    private Rectangle2D leftWheel;
    private Rectangle2D rightWheel;
    private Dimension size;
    private final double distanceBetweenFrontBackWheels;
    private Point currentLocation;
    private Point currentLocationCenter;
    private AffineTransform bodyAffineTransform = new AffineTransform();
    private AffineTransform leftWheelAffineTransform = new AffineTransform();
    private AffineTransform rightWheelAffineTransform = new AffineTransform();

    private final double distancePerTick; // the distance the robot moves per tick
    private final double angleChangePerClick;
    // need to incorporate concept of speed later
    private double turningRadius = 0;
    private static final double THETA_CAR = 270; // this is facing up
    private double thetaWheels = 0; // with reference to the front of car
    // since the coordinate system is positive x is right, positive y is down,
    // angles work by starting from positive x and going clockwise

    private Logger logger;

    private boolean firstTime = true;

    public Robot(Dimension size, Point startingPoint) {
        logger = Logger.getLogger(Robot.class.getName());

        this.size = size;
        this.setCurrentLocationCenter(startingPoint);
        body = new Rectangle2D.Double(0, 0, size.getWidth(), size.getHeight());
        leftWheel = new Rectangle2D.Double(0, 0, size.getWidth() / 5, size.getHeight() / 3);
        rightWheel = new Rectangle2D.Double(0, 0, size.getWidth() / 5, size.getHeight() / 3);

        distanceBetweenFrontBackWheels = size.getHeight() * 2 / 3;
        distancePerTick = size.getHeight() / 5;
        angleChangePerClick = 5;

        setOpaque(false);
    }

    public void turnLeft() {
        if (this.thetaWheels > -20) {
            this.thetaWheels -= angleChangePerClick;
            this.leftWheelAffineTransform.rotate(Math.toRadians(-angleChangePerClick));
            this.rightWheelAffineTransform.rotate(Math.toRadians(-angleChangePerClick));
            this.repaint();
        }
    }

    public void turnRight() {
        if (this.thetaWheels < 20) {
            this.thetaWheels += angleChangePerClick;
            this.leftWheelAffineTransform.rotate(Math.toRadians(angleChangePerClick));
            this.rightWheelAffineTransform.rotate(Math.toRadians(angleChangePerClick));
            this.repaint();
        }
    }

    public void moveForward() {
        calculateTurningRadius();
        double turningAngle = this.turningRadius == 0 ? 0 : this.distancePerTick / this.turningRadius;

        double angle2 = this.thetaWheels > 0 ? THETA_CAR + turningAngle : THETA_CAR - turningAngle;

        double dX = this.distancePerTick * Math.cos(Math.toRadians(angle2));
        double dY = this.distancePerTick * Math.sin(Math.toRadians(angle2));

        this.bodyAffineTransform.translate(dX, dY);
        this.bodyAffineTransform.rotate(Math.toRadians(this.thetaWheels));

        logger.log(Level.INFO, "X = " + this.bodyAffineTransform.getTranslateX() + ", Y = " + this.bodyAffineTransform.getTranslateY());

        this.repaint();
    }

    public void moveBackward() {
        calculateTurningRadius();
        double turningAngle = this.turningRadius == 0 ? 0 : this.distancePerTick / this.turningRadius;

        double angle2 = this.thetaWheels > 0 ? THETA_CAR - 180 - turningAngle : THETA_CAR - 180 + turningAngle;

        double dX = this.distancePerTick * Math.cos(Math.toRadians(angle2));
        double dY = this.distancePerTick * Math.sin(Math.toRadians(angle2));

        this.bodyAffineTransform.translate(dX, dY);
        this.bodyAffineTransform.rotate(-Math.toRadians(this.thetaWheels));

        logger.log(Level.INFO, "X = " + this.bodyAffineTransform.getTranslateX() + ", Y = " + this.bodyAffineTransform.getTranslateY());

        this.repaint();
    }

    // calculate turning radius based on distance between front and back wheel axis
    // and angle of wheels
    private void calculateTurningRadius() {
        this.turningRadius = Math.abs(this.distanceBetweenFrontBackWheels * Math.tan(Math.PI/2 - this.thetaWheels));
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


}