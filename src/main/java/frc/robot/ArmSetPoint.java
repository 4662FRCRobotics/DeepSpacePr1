package frc.robot;

public enum ArmSetPoint {
    PARK( "park" ), 
    BALL1( "ball1" ),
    BALL2( "ball2" ),
    BALL3( "ball3" ),
    PORT1( "port1" ),
    PORT2( "port2" ),
    PORT3( "port3" ),
    BALLCS( "ballcs"),
    PORTCS( "portcs");

    private String StrArmSetPoint;

    ArmSetPoint(String armSetPoint){
        this.StrArmSetPoint = armSetPoint;
    }

    public String getStrArmSetPoint(){
        return StrArmSetPoint;
    }
}