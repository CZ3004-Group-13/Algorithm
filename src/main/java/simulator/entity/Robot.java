package simulator.entity;

import java.awt.*;
import javax.swing.*;
import java.awt.geom.*;

public class Robot extends JComponent {
  private Rectangle2D rect;
  private Dimension size;
  private Point currentLocation;
  private Point currentLocationCenter;

  public Robot(Dimension size, Point startingPoint) {
    this.size = size;
    this.setCurrentLocationCenter(startingPoint);
    setOpaque(false);
  }

  public void rotate() {
  }

  // using currentLocation, calculate the center point
  private Point calculateCenter() {
    Point center = new Point((int) (currentLocation.getX() + size.getWidth() / 2),
        (int) (currentLocation.getY() + size.getHeight() / 2));

    return center;
  }

  // using currentLocationCenter, calculate the top left point
  private Point calculateTopLeftPoint() {
    Point topLeft = new Point((int) (currentLocationCenter.getX() - size.getWidth() / 2),
        (int) (currentLocationCenter.getY() - size.getHeight() / 2));

    return topLeft;
  }

  public void paintComponent(Graphics g) {
    Graphics2D g2 = (Graphics2D) g;
    rect = new Rectangle2D.Double(currentLocation.getX(), currentLocation.getY(), size.getWidth(), size.getHeight());
    
    g2.draw(rect);
    g2.setColor(Color.RED);
    g2.fill(rect);

    g2.rotate(25);
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