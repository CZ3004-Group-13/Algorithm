package myPackage;

import java.awt.*;
import javax.swing.*;
import java.awt.geom.*;


public class Robot extends JComponent {
  private Rectangle2D rect;

  public Robot() {
    rect = new Rectangle2D.Double(0, 0, 50, 50);
    // setBounds(20, 20, 50, 50);
    setOpaque(true);
  }

  public void rotate() {
  }

  public void paintComponent(Graphics g) {
    Graphics2D g2 = (Graphics2D) g;
    g2.draw(rect);
    g2.setColor(Color.RED);
    g2.fill(rect);

    // g2.rotate(25);
  }
}