model RideModel

  import SI = Modelica.SIunits;
  
  parameter SI.Mass massVeh=667 "Mass of the vehicle";
  parameter SI.Inertia Iyy=211 "Iyy Total of the vehicle";
  parameter SI.Length wheelbase=2.362  "Wheelbase of vehicle";
  parameter Real cgLocation=0.486 "Percentage Front";
  parameter SI.TranslationalSpringConstant kf=69130 "Spring Rate in N/m";
  parameter SI.TranslationalDampingConstant cf=0 "Spring Rate in N/m";
  parameter SI.TranslationalSpringConstant kr=19360 "Spring Rate in N/m";
  parameter SI.TranslationalDampingConstant cr=0 "Spring Rate in N/m";  
  parameter SI.TranslationalSpringConstant k_tyre_f=230000 "Spring Rate in N/m";
  parameter SI.TranslationalDampingConstant c_tyre_f=0 "Spring Rate in N/m";
  parameter SI.TranslationalSpringConstant k_tyre_r=230000 "Spring Rate in N/m";
  parameter SI.TranslationalDampingConstant c_tyre_r=0 "Spring Rate in N/m";  
  parameter Real motion_ratio_f=0.68 "Front Motion Ratio";  
  parameter Real motion_ratio_r=0.95 "Rear Motion Ratio";
  
  parameter Real time_from_start=0.02 "Time from start";
  parameter Real step_height=0.075 "Height of Step";
  parameter Real velocity = 27.8 "Speed of vehicle";
  
  Real k_ride_f = kf*motion_ratio_f^2;
  Real k_ride_r = kr*motion_ratio_r^2;
  
  Real k_wheel_f = 2*(k_ride_f*k_tyre_f)/(k_ride_f+k_tyre_f);
  Real k_wheel_r = 2*(k_ride_r*k_tyre_r)/(k_ride_r+k_tyre_r);
  
    
  Real pi=2*Modelica.Math.asin(1.0);
  Real a=cgLocation*wheelbase;
  Real b = (1-cgLocation)*wheelbase;
  
  Real z_road_f, z_road_f_dot;
  Real z_road_r, z_road_r_dot;
  Real z, z_dot, z_dotdot;
  Real theta, theta_dot, theta_dotdot;
  

equation

  z_dotdot = (k_wheel_f*(z_road_f-z) + k_wheel_r*(z_road_r-z) + cf*(z_road_f_dot-z_dot) + cr*(z_road_r_dot-z_dot) - a*k_wheel_f*theta + b*k_wheel_r*theta - a*cf*theta_dot + b*cr*theta_dot)/massVeh;
  
  theta_dotdot = (-k_wheel_f*a*(z_road_f-z) + k_wheel_r*b*(z_road_r-z) - cf*a*(z_road_f_dot-z_dot) + cr*b*(z_road_r_dot-z_dot) - k_wheel_f*a^2*theta -k_wheel_r*b^2*theta - cf*a^2*theta_dot -cr*b^2*theta_dot)/Iyy;
  
  der(z) = z_dot;
  der(theta) = theta_dot;
  der(z_dot) = z_dotdot;
  der(theta_dot) = theta_dotdot;
  der(z_road_f) = z_road_f_dot;
  der(z_road_r) = z_road_r_dot;
  
  if time < time_from_start then
    z_road_f = step_height/2*(1-cos(pi*time/time_from_start));
  else
    z_road_f = step_height;
  end if;  

  if time - (wheelbase/velocity) < 0 then
    z_road_r = 0;
  elseif time < time_from_start + (wheelbase/velocity) then
    z_road_r = step_height/2*(1-cos(pi*(time-(wheelbase/velocity))/time_from_start));
  else
    z_road_r = step_height;
  end if;  
    
end RideModel;
