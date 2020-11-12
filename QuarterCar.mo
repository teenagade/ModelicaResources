model QuarterCar

  import SI = Modelica.SIunits;
  
  parameter SI.Mass mass_veh=472 "Mass of the vehicle in kg";
  parameter SI.Mass mass_unsprung=39.1 "Unsprung mass in kg";
  parameter SI.TranslationalSpringConstant k_spring=36190 "Spring Rate in N/m";
  parameter SI.TranslationalDampingConstant c_spring=0 "Damper Rate in Ns/m";
  parameter SI.TranslationalSpringConstant k_tyre=160000 "Spring Rate in N/m";
  parameter SI.TranslationalDampingConstant c_tyre=0 "Tyre Damping Rate in Ns/m";
  parameter SI.Length step_height=0.075;
  parameter SI.Time time_from_start = 0.02;


// Variables to solve for     
  Real pi=2*Modelica.Math.asin(1.0);
  Real z_road, z_tyre, z_body;
  Real z_road_dot, z_tyre_dot, z_body_dot;
  Real z_tyre_dotdot, z_body_dotdot;

initial equation
  z_tyre = 0.0;
  z_body = 0.0;
    
equation
  z_body_dotdot = (c_spring*(z_tyre_dot-z_body_dot) + k_spring*(z_tyre-z_body))/mass_veh;
  z_tyre_dotdot = (c_spring*(z_body_dot-z_tyre_dot) + k_spring*(z_body-z_tyre) + c_tyre*(z_road_dot-z_tyre_dot) + k_tyre*(z_road-z_tyre)) / mass_unsprung;
  
if time < time_from_start then
  z_road = step_height/2*(1-cos(pi*time/time_from_start));
else
  z_road = step_height;
end if;

  der(z_tyre) = z_tyre_dot;
  der(z_body) = z_body_dot;
  der(z_road) = z_road_dot;
  der(z_tyre_dot) = z_tyre_dotdot;
  der(z_body_dot) = z_body_dotdot;
  
end QuarterCar;
