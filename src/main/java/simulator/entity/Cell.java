package simulator.entity;

import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.geom.Rectangle2D;
import java.util.logging.Level;
import java.util.logging.Logger;

public class Cell extends JComponent implements MouseListener, MouseMotionListener {

    private final Logger logger;

    private int id = -1;
    private boolean obstaclePresent = false;
    private boolean obstacleVisited = false;
    private Direction imageDirection = Direction.NONE;
    private Point initialClickPoint;
    private Polygon imageDirectionPolygon;

    public Cell() {
        logger = Logger.getLogger(Cell.class.getName());

        setBorder(BorderFactory.createLineBorder(Color.lightGray));
        addMouseListener(this);
        addMouseMotionListener(this);
    }

    // if obstacle present
    // paints its wholeself black and the imageDirectionPolygon
    public void paintComponent(Graphics g) {
        super.paintComponent(g);

        if (obstaclePresent) {
            Dimension size = this.getSize();
            Graphics2D g2 = (Graphics2D) g;
            Rectangle2D rect = new Rectangle2D.Double(0, 0, size.getWidth(), size.getHeight());

            g2.setColor(Color.BLACK);
            g2.fill(rect);

            g2.setColor(Color.RED);
            g2.fill(imageDirectionPolygon);
        }
    }

    public void createImageDirectionPolygon() {
        imageDirectionPolygon = new Polygon();
        Dimension size = this.getSize();
        int halfWidth = (int) (0.5 * size.getWidth());
        int halfHeight = (int) (0.5 * size.getHeight());

        Point northCenter = new Point(0, 0);
        Point eastCenter = new Point(0, 0);
        Point southCenter = new Point(0, 0);
        Point westCenter = new Point(0, 0);

        northCenter.translate(halfWidth, 0);
        eastCenter.translate((int) size.getWidth(), halfHeight);
        southCenter.translate(halfWidth, (int) size.getHeight());
        westCenter.translate(0, halfHeight);

        switch (this.getImageDirection()) {
        case NORTH -> {
            imageDirectionPolygon.addPoint(northCenter.x, northCenter.y);
            imageDirectionPolygon.addPoint(eastCenter.x, eastCenter.y);
            imageDirectionPolygon.addPoint(westCenter.x, westCenter.y);
        }
        case EAST -> {
            imageDirectionPolygon.addPoint(southCenter.x, southCenter.y);
            imageDirectionPolygon.addPoint(eastCenter.x, eastCenter.y);
            imageDirectionPolygon.addPoint(northCenter.x, northCenter.y);
        }
        case SOUTH -> {
            imageDirectionPolygon.addPoint(southCenter.x, southCenter.y);
            imageDirectionPolygon.addPoint(eastCenter.x, eastCenter.y);
            imageDirectionPolygon.addPoint(westCenter.x, westCenter.y);
        }
        case WEST -> {
            imageDirectionPolygon.addPoint(southCenter.x, southCenter.y);
            imageDirectionPolygon.addPoint(westCenter.x, westCenter.y);
            imageDirectionPolygon.addPoint(northCenter.x, northCenter.y);
        }
        default -> {
        }
        }

    }

    @Override
    public void mouseClicked(MouseEvent e) {
        logger.log(Level.FINEST, "Mouse clicked");
    }

    @Override
    public void mousePressed(MouseEvent e) {
        logger.log(Level.FINEST, "Mouse pressed");

        initialClickPoint = e.getPoint();
        this.setObstaclePresent(!this.isObstaclePresent());
        if (this.isObstaclePresent()) {
            this.setId(Math.round((float)Math.random()*10)); // randomly assign int id
            this.setImageDirection(Direction.NORTH);
        }
        this.repaint();
    }

    @Override
    public void mouseReleased(MouseEvent e) {
        logger.log(Level.FINEST, "Mouse released");
        initialClickPoint = null;
    }

    @Override
    public void mouseEntered(MouseEvent e) {
        logger.log(Level.FINEST, "Mouse entered");
    }

    @Override
    public void mouseExited(MouseEvent e) {
        logger.log(Level.FINEST, "Mouse left");
    }

    @Override
    public void mouseDragged(MouseEvent e) {
        logger.log(Level.FINEST, "Mouse dragged");

        // only trigger if there is an obstacle present and
        // mouse click held down
        if (this.isObstaclePresent() && this.initialClickPoint != null) {
            Point currentPoint = e.getPoint();

            if (currentPoint.getX() > initialClickPoint.getX()) {
                if (currentPoint.getY() < (initialClickPoint.getY() - (currentPoint.getX() - initialClickPoint.getX()))) {
                    this.setImageDirection(Direction.NORTH);
                } else if (currentPoint
                        .getY() > (initialClickPoint.getY() + (currentPoint.getX() - initialClickPoint.getX()))) {
                    this.setImageDirection(Direction.SOUTH);
                } else {
                    this.setImageDirection(Direction.EAST);
                }
            } else {
                if (currentPoint.getY() < (initialClickPoint.getY() - (initialClickPoint.getX() - currentPoint.getX()))) {
                    this.setImageDirection(Direction.NORTH);
                } else if (currentPoint
                        .getY() > (initialClickPoint.getY() + (initialClickPoint.getX() - currentPoint.getX()))) {
                    this.setImageDirection(Direction.SOUTH);
                } else {
                    this.setImageDirection(Direction.WEST);
                }
            }
        }
        this.repaint();
    }

    @Override
    public void mouseMoved(MouseEvent e) {
        logger.log(Level.FINEST, "Mouse moved");
    }

    public Direction getImageDirection() {
        return imageDirection;
    }

    // when setting image direction, recreate image direction polygon
    public void setImageDirection(Direction imageDirection) {
        this.imageDirection = imageDirection;
        this.createImageDirectionPolygon();
    }

    public int getId() {
        return id;
    }

    public void setId(int id) {
        this.id = id;
    }

    public boolean isObstaclePresent() {
        return obstaclePresent;
    }

    public void setObstaclePresent(boolean obstaclePresent) {
        this.obstaclePresent = obstaclePresent;
        if (!this.obstaclePresent) {
            this.setId(-1);
            this.setImageDirection(Direction.NONE);
            this.setObstacleVisited(false);
        }
    }

    public boolean isObstacleVisited() {
        return obstacleVisited;
    }

    public void setObstacleVisited(boolean obstacleVisited) {
        this.obstacleVisited = obstacleVisited;
    }

}
