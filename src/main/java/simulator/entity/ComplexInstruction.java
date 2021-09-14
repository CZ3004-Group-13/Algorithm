package simulator.entity;


public class ComplexInstruction {

    private final Direction finalDirection;
    private final Instruction instruction;
    private double distance;

    public ComplexInstruction(Instruction instruction, Direction finalDirection) {
        this.instruction = instruction;
        this.finalDirection = finalDirection;
        this.distance = Double.MIN_VALUE;
    }

    public ComplexInstruction(Instruction instruction, Direction finalDirection, double distance) {
        this.instruction = instruction;
        this.finalDirection = finalDirection;
        this.distance = distance;
    }

    public Instruction getInstruction() {
        return instruction;
    }

    public Direction getFinalDirection() {
        return finalDirection;
    }

    public double getDistance() {
        return distance;
    }

    public void subtractDistance(double offset) {
        distance -= offset;
    }

    public enum Instruction {
        FORWARD,
        REVERSE,
        FORWARD_LEFT,
        FORWARD_RIGHT,
    }

}
