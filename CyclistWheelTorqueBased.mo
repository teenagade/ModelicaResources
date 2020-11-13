model CyclistWheelTorqueBased
import SI = Modelica.SIunits;
  
  parameter SI.Density rho=1.22 "Air Density";
  parameter SI.Mass wRider=70 "Rider Weight";
  parameter SI.Mass wBike=7 "Bike Weight";
  parameter SI.Mass wTotal=wRider+wBike "Total Bike and Rider Weight";
  parameter SI.Area A=0.4286 "Frontal Area";
  parameter SI.Velocity vAir=0 "Headwind, negative for tailwind";
  parameter SI.Acceleration g=9.81 "Gravitational constant";
  parameter SI.Length WheelDiameter=0.7 "Wheel Diameter";
  parameter SI.Power PLegs=160 "Power at the Crank";
  parameter Real Cd=0.88 "Drag Coefficient";
  parameter Real Crr=0.005 "Rolling Resistance Coefficient";
  parameter Real gradient=0 "Road Gradient";
  SI.Velocity V(start=2.78) "Velocity of Cyclist";
  SI.Acceleration acceleration;
  SI.AngularVelocity WheelSpeed "Wheel Angular Velocity";
  SI.Torque DriveTorque "Torque at the wheel";
  SI.Force DriveForce "Driving Force at the wheel", FGravity, FDrag, FRolling, FResist;
  SI.Length distance "Distance Travelled";
  
  equation
    FGravity = g*sin(atan(gradient/100))*wTotal;
    FRolling = g*cos(atan(gradient/100))*wTotal*Crr;
    FDrag = 0.5*Cd*A*rho*V^2;
    FResist = FDrag + FGravity + FRolling;
    WheelSpeed = V/(WheelDiameter*2);
    DriveTorque = 0.98*PLegs/WheelSpeed;
    DriveForce = DriveTorque*WheelDiameter;
    acceleration = (DriveForce-FResist)/wTotal;
    acceleration = der(V);
    der(distance)=V;

end CyclistWheelTorqueBased;
