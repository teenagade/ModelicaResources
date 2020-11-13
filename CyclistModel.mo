model CyclistModel

import SI = Modelica.SIunits;
  
  parameter SI.Density rho=1.22 "Air Density";
  parameter SI.Mass wRider=70 "Rider Weight";
  parameter SI.Mass wBike=7 "Bike Weight";
  parameter SI.Mass wTotal=wRider+wBike "Total Bike and Rider Weight";
  parameter SI.Area A=0.509 "Frontal Area";
  parameter SI.Velocity vAir=0 "Headwind, negative for tailwind";
  parameter SI.Acceleration g=9.81 "Gravitational constant";
  parameter Real Cd=0.63 "Drag Coefficient";
  parameter Real gradient=1 "Road gradient as a percentage, neagtive for downhill";
  parameter Real Crr=0.005 "Rolling Resistance Coefficient";
  SI.Velocity V(start=2.78) "Velocity of Cyclist";
  SI.Force FGravity, FRolling, FDrag, FResist;
  SI.Power PLegs=36.96; 
  SI.Acceleration  acceleration;
  SI.Length distance;
    
  equation
    FGravity = g*sin(atan(gradient/100))*wTotal;
    FRolling = g*cos(atan(gradient/100))*wTotal*Crr;
    FDrag = 0.5*Cd*A*rho*V^2;

    FResist = FDrag + FGravity + FRolling;
    0.98 * PLegs = (wTotal * acceleration - FResist) * V;
    acceleration = der(V);
    der(distance) = V;

end Cyclist2;
