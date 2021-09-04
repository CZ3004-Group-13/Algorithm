package simulator.entity;


public class ComplexInstruction {

    private final Direction finalDirection;
    private final Instruction instruction;
    private double distance = 0;

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

    public ComplexInstruction(Instruction instruction, Direction finalDirection) {
        this.instruction = instruction;
        this.finalDirection = finalDirection;

    }

    public ComplexInstruction(Instruction instruction, Direction finalDirection, double distance) {
        this.instruction = instruction;
        this.finalDirection = finalDirection;
        this.distance = distance;
    }

}
