package simulator.entity;

import java.awt.Point;

public class MyPoint extends Point {
  private Direction direction = Direction.NONE;

  public MyPoint(int i, int j, Direction d) {
    super(i, j);
    this.setDirection(d);
  }

  public Direction getDirection() {
    return direction;
  }

  public void setDirection(Direction direction) {
    this.direction = direction;
  }

}
