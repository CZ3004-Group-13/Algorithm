package simulator.entity;

import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.geom.Rectangle2D;

public class Cell extends JComponent implements MouseListener, MouseMotionListener {

    private int id;
    private boolean obstaclePresent = false;
    private int imageDirection = 0;
    // 0 is no image, 1 North, 2 East, 3 South, 4 West
    private Point initialClickPoint;
    private Polygon imageDirectionPolygon;

    public Cell() {
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
        case 1:
            imageDirectionPolygon.addPoint(northCenter.x, northCenter.y);
            imageDirectionPolygon.addPoint(eastCenter.x, eastCenter.y);
            imageDirectionPolygon.addPoint(westCenter.x, westCenter.y);
            break;
        case 2:
            imageDirectionPolygon.addPoint(southCenter.x, southCenter.y);
            imageDirectionPolygon.addPoint(eastCenter.x, eastCenter.y);
            imageDirectionPolygon.addPoint(northCenter.x, northCenter.y);
            break;
        case 3:
            imageDirectionPolygon.addPoint(southCenter.x, southCenter.y);
            imageDirectionPolygon.addPoint(eastCenter.x, eastCenter.y);
            imageDirectionPolygon.addPoint(westCenter.x, westCenter.y);
            break;
        case 4:
            imageDirectionPolygon.addPoint(southCenter.x, southCenter.y);
            imageDirectionPolygon.addPoint(westCenter.x, westCenter.y);
            imageDirectionPolygon.addPoint(northCenter.x, northCenter.y);
            break;
        case 0:
            break;
        default:
            break;
        }

    }

    @Override
    public void mouseClicked(MouseEvent e) {
        // System.out.println("Mouse clicked");
    }

    @Override
    public void mousePressed(MouseEvent e) {
        // System.out.println("Mouse pressed");

        initialClickPoint = e.getPoint();
        this.setObstaclePresent(!this.isObstaclePresent());
        if (this.isObstaclePresent()) {
            this.setImageDirection(1);
        }
        this.repaint();
    }

    @Override
    public void mouseReleased(MouseEvent e) {
        // System.out.println("Mouse released");
        initialClickPoint = null;
    }

    @Override
    public void mouseEntered(MouseEvent e) {
        // System.out.println("Mouse entered");
    }

    @Override
    public void mouseExited(MouseEvent e) {
        // System.out.println("Mouse left");
    }

    @Override
    public void mouseDragged(MouseEvent e) {
        // System.out.println("Mouse dragged");

        // only trigger if there is an obstacle present and
        // mouse click held down
        if (this.isObstaclePresent() && this.initialClickPoint != null) {
            Point currentPoint = e.getPoint();

            if (currentPoint.getX() > initialClickPoint.getX()) {
                if (currentPoint.getY() < (initialClickPoint.getY() - (currentPoint.getX() - initialClickPoint.getX()))) {
                    this.setImageDirection(1);
                } else if (currentPoint
                        .getY() > (initialClickPoint.getY() + (currentPoint.getX() - initialClickPoint.getX()))) {
                    this.setImageDirection(3);
                } else {
                    this.setImageDirection(2);
                }
            } else {
                if (currentPoint.getY() < (initialClickPoint.getY() - (initialClickPoint.getX() - currentPoint.getX()))) {
                    this.setImageDirection(1);
                } else if (currentPoint
                        .getY() > (initialClickPoint.getY() + (initialClickPoint.getX() - currentPoint.getX()))) {
                    this.setImageDirection(3);
                } else {
                    this.setImageDirection(4);
                }
            }
        }
        this.repaint();
    }

    @Override
    public void mouseMoved(MouseEvent e) {
        // System.out.println("Mouse moved");
    }

    public int getImageDirection() {
        return imageDirection;
    }

    // when setting image direction, recreate image direction polygon
    public void setImageDirection(int imageDirection) {
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
    }

}
