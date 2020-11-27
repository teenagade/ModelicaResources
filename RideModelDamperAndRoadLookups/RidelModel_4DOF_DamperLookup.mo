model RidelModel_4DOF_DamperLookup

  import SI = Modelica.SIunits;
  
  parameter SI.Mass mass_total=667 "Mass of the vehicle";
  parameter SI.Mass mass_unsprung_f=46 "Unspring mass front";
  parameter SI.Mass mass_unsprung_r=85 "Unsprung mass rear";
  parameter SI.Inertia Iyy=211 "Iyy Total of the vehicle";
  parameter SI.Length wheelbase=2.362  "Wheelbase of vehicle";
  parameter Real cgLocation=0.486 "Percentage Front";
  parameter SI.TranslationalSpringConstant kf=69130 "Spring Rate in N/m";
//  parameter SI.TranslationalDampingConstant cf=0 "Spring Rate in N/m";
  parameter SI.TranslationalSpringConstant kr=19360 "Spring Rate in N/m";
  //  parameter SI.TranslationalDampingConstant cr=0 "Spring Rate in N/m";
  parameter SI.TranslationalSpringConstant k_tyre_f=230000 "Spring Rate in N/m";
  parameter SI.TranslationalDampingConstant c_tyre_f=400 "Spring Rate in N/m";
  parameter SI.TranslationalSpringConstant k_tyre_r=230000 "Spring Rate in N/m";
  parameter SI.TranslationalDampingConstant c_tyre_r=400 "Spring Rate in N/m";  
  parameter Real motion_ratio_f=0.68 "Front Motion Ratio";  
  parameter Real motion_ratio_r=0.95 "Rear Motion Ratio";
  
  parameter Real time_from_start=0.02 "Time from start";
  parameter Real step_height=0.075 "Height of Step";
  parameter Real velocity = 27.8 "Speed of vehicle";
  
  Real mass_sprung = mass_total - mass_unsprung_f - mass_unsprung_r;
  
  Real cf, damper_f_dot;
  Real cr, damper_r_dot;
  
  Real k_ride_f = 2*kf*motion_ratio_f^2;
  Real k_ride_r = 2*kr*motion_ratio_r^2;
  
  Real pi=2*Modelica.Math.asin(1.0);
  Real a=cgLocation*wheelbase;
  Real b = (1-cgLocation)*wheelbase;
  
  Real z_road_f, z_road_f_dot;
  Real z_road_r, z_road_r_dot;
  Real z, z_dot, z_dotdot;
  Real theta, theta_dot, theta_dotdot;
  Real z_tyre_f, z_tyre_f_dot, z_tyre_f_dotdot;
  Real z_tyre_r, z_tyre_r_dot, z_tyre_r_dotdot;

  Modelica.Blocks.Tables.CombiTable1D damperCurvesFront(columns = 2:2, fileName = "D:/Documents/DataDriven/Modelica/front_dampers.txt", smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments, tableName = "CR06", tableOnFile = true, verboseRead = false)  annotation(
    Placement(visible = true, transformation(origin = {-10, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain multiplyBy1000_F(k = 1000)  annotation(
    Placement(visible = true, transformation(origin = {-42, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Modelica.Blocks.Tables.CombiTable1D damperCurvesRear(columns = 2:2, fileName = "D:/Documents/DataDriven/Modelica/rear_dampers.txt", smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments, tableName = "CR00", tableOnFile = true, verboseRead = false)  annotation(
    Placement(visible = true, transformation(origin = {-10, -26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain multiplyBy1000_R(k = 1000)  annotation(
    Placement(visible = true, transformation(origin = {-42, -26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  
initial equation
  z_tyre_r_dot = 0;
  z_tyre_r = 0;
  z_tyre_f_dot = 0;
  z_tyre_f = 0;
  z = 0;
  z_dot = 0;
  theta = 0;
  theta_dot = 0;

equation
  connect(multiplyBy1000_R.y, damperCurvesRear.u[1]) annotation(
    Line(points = {{-31, -28}, {-26.5, -28}, {-26.5, -26}, {-22, -26}}, color = {0, 0, 127}));
  connect(damper_r_dot, multiplyBy1000_R.u) annotation(
    Line);
  connect(damperCurvesRear.y[1], cr) annotation(
    Line);
  connect(damper_f_dot, multiplyBy1000_F.u);
  connect(multiplyBy1000_F.y, damperCurvesFront.u[1]) annotation(
    Line(points = {{-30, 10}, {-24, 10}, {-24, 10}, {-22, 10}}, color = {0, 0, 127}));
  connect(damperCurvesFront.y[1], cf);
  
  z_dotdot = (k_ride_f*(z_tyre_f-z) + k_ride_r*(z_tyre_r-z) + cf*(z_tyre_f_dot-z_dot) + cr*(z_tyre_r_dot-z_dot) - a*k_ride_f*theta + b*k_ride_r*theta - a*cf*theta_dot + b*cr*theta_dot)/mass_sprung;
  
  theta_dotdot = (-k_ride_f*a*(z_tyre_f-z) + k_ride_r*b*(z_tyre_r-z) - cf*a*(z_tyre_f_dot-z_dot) + cr*b*(z_tyre_r_dot-z_dot) - k_ride_f*a^2*theta -k_ride_r*b^2*theta - cf*a^2*theta_dot -cr*b^2*theta_dot)/Iyy;
  
  z_tyre_f_dotdot = (k_tyre_f*(z_road_f-z_tyre_f) + c_tyre_f*(z_road_f_dot-z_tyre_f_dot))/mass_unsprung_f;
  
  z_tyre_r_dotdot = (k_tyre_r*(z_road_r-z_tyre_r) + c_tyre_r*(z_road_r_dot-z_tyre_r_dot))/mass_unsprung_r;

  damper_f_dot = (((z_tyre_f_dot - z_dot - a*theta_dot)/mass_sprung) + (a*(z_tyre_f_dot - z_dot - 2*a*theta_dot))/Iyy)/(2*motion_ratio_f);

  damper_r_dot = (((z_tyre_r_dot - z_dot + b*theta_dot)/mass_sprung) + (b*(z_tyre_r_dot - z_dot - 2*b*theta_dot))/Iyy)/(2*motion_ratio_r);  
  
  der(z) = z_dot;
  der(theta) = theta_dot;
  der(z_dot) = z_dotdot;
  der(theta_dot) = theta_dotdot;
  der(z_road_f) = z_road_f_dot;
  der(z_road_r) = z_road_r_dot;
  der(z_tyre_f) = z_tyre_f_dot;
  der(z_tyre_r) = z_tyre_r_dot; 
  der(z_tyre_f_dot) = z_tyre_f_dotdot;
  der(z_tyre_r_dot) = z_tyre_r_dotdot;   
  
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
    
end RidelModel_4DOF_DamperLookup;
