model RideModel_6DOF
  import SI = Modelica.SIunits;
  
  parameter SI.Mass mass_total=1098.6 "Mass of the vehicle";
  parameter SI.Mass mass_unsprung_f = 27.5 "Unsprung mass front";
  parameter SI.Mass mass_unsprung_r = 30.75 "Unsprung mass rear";
  parameter SI.Inertia Ixx=181 "Ixx Total of the vehicle";  
  parameter SI.Inertia Iyy=1310 "Iyy Total of the vehicle";
  parameter SI.Length wheelbase=3.005  "Wheelbase of vehicle";
  parameter SI.Length tf=1.57  "Front track of vehicle";
  parameter SI.Length tr=1.55  "Rear track of vehicle";
  parameter Real cgLocation=0.501 "Percentage Front";
  parameter SI.TranslationalSpringConstant k_spring_f=280000 "Spring Rate in N/m";
  parameter SI.TranslationalDampingConstant cfl=280 "Damper Rate in N/m";
  parameter SI.TranslationalDampingConstant cfr=280 "Damper Rate in N/m";
  parameter SI.TranslationalSpringConstant k_spring_r=120000 "Spring Rate in N/m";
  parameter SI.TranslationalDampingConstant crl=120 "Damper Rate in N/m";
  parameter SI.TranslationalDampingConstant crr=120 "Damper Rate in N/m";
  parameter SI.TranslationalSpringConstant k_tyre_f=350000 "Spring Rate in N/m";
  parameter SI.TranslationalDampingConstant c_tyre_f=350 "Spring Rate in N/m";
  parameter SI.TranslationalSpringConstant k_tyre_r=350000 "Spring Rate in N/m";
  parameter SI.TranslationalDampingConstant c_tyre_r=350 "Spring Rate in N/m";  
  parameter Real spring_motion_ratio_f=1.0 "Front Spring Motion Ratio";  
  parameter Real spring_motion_ratio_r=1.0 "Rear Spring Motion Ratio";
  
  parameter Real time_from_start=0.02 "Time from start";
  parameter Real step_height=0.0075 "Height of Step";
  parameter Real velocity = 27.8 "Speed of vehicle";
  
  Real mass_sprung = mass_total - 2*(mass_unsprung_f+mass_unsprung_r);
  
  Real kfl = k_spring_f*spring_motion_ratio_f^2;
  Real krl = k_spring_r*spring_motion_ratio_r^2;
  Real kfr = kfl;
  Real krr = krl;
    
  Real pi=2*Modelica.Math.asin(1.0);
  Real a = (1-cgLocation)*wheelbase;
  Real b = cgLocation*wheelbase;
  
  Real z_road_fl, z_road_fl_dot, z_road_fr, z_road_fr_dot;
  Real z_road_rl, z_road_rl_dot, z_road_rr, z_road_rr_dot;
  Real z_tyre_fl, z_tyre_fl_dot, z_tyre_fl_dotdot; 
  Real z_tyre_fr, z_tyre_fr_dot, z_tyre_fr_dotdot;
  Real z_tyre_rl, z_tyre_rl_dot, z_tyre_rl_dotdot;
  Real z_tyre_rr, z_tyre_rr_dot, z_tyre_rr_dotdot;
  Real z, z_dot, z_dotdot;
  Real theta, theta_dot, theta_dotdot;
  
  Real damper_fl_velocity, damper_fr_velocity, damper_fl_force, damper_fr_force;
  Real damper_rl_velocity, damper_rr_velocity, damper_rl_force, damper_rr_force;

initial equation
  damper_fl_velocity = 0.001;
  damper_fr_velocity = 0.001;
  damper_rl_velocity = 0.001;
  damper_rr_velocity = 0.001;

equation

  
  z_dotdot*mass_sprung = kfl*(z_tyre_fl - (z + theta*a)) + kfr*(z_tyre_fr - (z + theta*a)) + krl*(z_tyre_rl - (z - theta*b)) + krr*(z_tyre_rr - (z - theta*b)) + cfl*(z_tyre_fl_dot - (z_dot + theta_dot*a)) + cfr*(z_tyre_fr_dot - (z + theta_dot*a)) + crl*(z_tyre_rl_dot - (z - theta*b)) + crr*(z_tyre_rr_dot - (z_dot - theta_dot*b));

  theta_dotdot*Iyy = kfl*a*(z_tyre_fl - (z + theta*a)) + kfr*a*(z_tyre_fr - (z + theta*a)) - krl*b*(z_tyre_rl - (z - theta*b)) - krr*b*(z_tyre_rr - (z - theta*b)) + cfl*a*(z_tyre_fl_dot - (z_dot + theta_dot*a)) + cfr*a*(z_tyre_fr_dot - (z_dot + theta_dot*a)) - crl*b*(z_tyre_rl_dot - (z_dot - theta_dot*b)) - crr*b*(z_tyre_rr_dot - (z_dot - theta_dot*b));

  damper_fl_velocity = (z_tyre_fl_dot - (z_dot + theta_dot*a)) + a*(z_tyre_fl_dot - (z_dot + theta_dot*a));
  cfl = damper_fl_force/damper_fl_velocity;
  
  damper_fr_velocity = (z_tyre_fr_dot - (z_dot + theta_dot*a)) + a*(z_tyre_fr_dot - (z_dot + theta_dot*a));
  cfr = damper_fr_force/damper_fr_velocity;
  
  damper_rl_velocity = (z_tyre_rl_dot - (z_dot - theta_dot*b)) - b*(z_tyre_rl_dot - (z_dot - theta_dot*b));
  crl = damper_rl_force/damper_rl_velocity;
  
  damper_rr_velocity = (z_tyre_rr_dot - (z_dot - theta_dot*b)) - b*(z_tyre_rr_dot - (z_dot - theta_dot*b));
  crr = damper_rr_force/damper_rr_velocity;

  z_tyre_fl_dotdot*mass_unsprung_f = k_tyre_f*(z_road_fl - z_tyre_fl) + kfl*(z + theta*a - z_tyre_fl) + cfl*(z_dot + theta_dot*a - z_tyre_fl_dot);

  z_tyre_fr_dotdot*mass_unsprung_f = k_tyre_f*(z_road_fr - z_tyre_fr) + kfr*(z + theta*a - z_tyre_fr) + cfr*(z_dot + theta_dot*a - z_tyre_fr_dot);

  z_tyre_rl_dotdot*mass_unsprung_r = k_tyre_r*(z_road_rl - z_tyre_rl) + krl*(z - theta*b - z_tyre_rl) + crl*(z_dot - theta_dot*b - z_tyre_rl_dot);

  z_tyre_rr_dotdot*mass_unsprung_r = k_tyre_r*(z_road_rr - z_tyre_rr) + krr*(z - theta*b - z_tyre_rr) + crr*(z_dot - theta_dot*b - z_tyre_rr_dot);
  
  der(z) = z_dot;
  der(theta) = theta_dot;
  der(z_dot) = z_dotdot;
  der(theta_dot) = theta_dotdot;
  der(z_tyre_fl) = z_tyre_fl_dot;
  der(z_tyre_rl) = z_tyre_rl_dot;
  der(z_tyre_fr) = z_tyre_fr_dot;
  der(z_tyre_rr) = z_tyre_rr_dot;
  der(z_tyre_fl_dot) = z_tyre_fl_dotdot;
  der(z_tyre_fr_dot) = z_tyre_fr_dotdot;
  der(z_tyre_rl_dot) = z_tyre_rl_dotdot;
  der(z_tyre_rr_dot) = z_tyre_rr_dotdot;  
  der(z_road_fl) = z_road_fl_dot;
  der(z_road_fr) = z_road_fr_dot;
  der(z_road_rl) = z_road_rl_dot;
  der(z_road_rr) = z_road_rr_dot;
  
  if time < time_from_start then
    z_road_fl = step_height/2*(1-cos(pi*time/time_from_start));
  else
    z_road_fl = step_height;
  end if;  

  z_road_rl = z_road_fl;
  z_road_fr = z_road_fl;
  z_road_rr = z_road_rl;
   
end RideModel_6DOF;
