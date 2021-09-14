package simulator.entity;

import java.awt.*;

public class MyPoint extends Point {
    private Direction direction = Direction.NONE;

    public MyPoint(int i, int j, Direction d) {
        super(i, j);
        this.setDirection(d);
    }

    public MyPoint(double i, double j, Direction d) {
        super((int) i, (int) j);
        this.setDirection(d);
    }

    public Direction getDirection() {
        return direction;
    }

    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    public void rotateRight90() {
        switch (this.direction) {
        case NONE:
            break;
        case NORTH:
            this.direction = Direction.EAST;
            break;
        case SOUTH:
            this.direction = Direction.WEST;
            break;
        case EAST:
            this.direction = Direction.SOUTH;
            break;
        case WEST:
            this.direction = Direction.NORTH;
            break;
        default:
            break;
        }
    }

    public void rotateLeft90() {
        switch (this.direction) {
        case NONE:
            break;
        case NORTH:
            this.direction = Direction.WEST;
            break;
        case SOUTH:
            this.direction = Direction.EAST;
            break;
        case EAST:
            this.direction = Direction.NORTH;
            break;
        case WEST:
            this.direction = Direction.SOUTH;
            break;
        default:
            break;
        }
    }

    public void rotate180() {
        switch (this.direction) {
        case NONE:
            break;
        case NORTH:
            this.direction = Direction.SOUTH;
            break;
        case SOUTH:
            this.direction = Direction.NORTH;
            break;
        case EAST:
            this.direction = Direction.WEST;
            break;
        case WEST:
            this.direction = Direction.EAST;
            break;
        default:
            break;
        }
    }

}
