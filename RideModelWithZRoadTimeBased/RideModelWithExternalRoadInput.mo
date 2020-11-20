model RideModelWithExternalRoadInput
  RideModel_7DOF_zRoadVector RideModel annotation(
    Placement(visible = true, transformation(origin = {33, 13}, extent = {{-39, -39}, {39, 39}}, rotation = 0)));
  Modelica.Blocks.Sources.CombiTimeTable z_road(columns = 2:5, extrapolation = Modelica.Blocks.Types.Extrapolation.LastTwoPoints, fileName = "D:/Documents/DataDriven/Modelica/z_road.txt", offset = {0}, smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments, startTime = 0, tableName = "z_road", tableOnFile = true, timeScale = 1, verboseRead = false)  annotation(
    Placement(visible = true, transformation(origin = {-78, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

equation
  connect(z_road.y[1], RideModel.z_road_fl) annotation(
    Line(points = {{-66, 60}, {-34, 60}, {-34, 46}, {-14, 46}, {-14, 44}}, color = {0, 0, 127}, thickness = 0.5));
  connect(z_road.y[2], RideModel.z_road_fr) annotation(
    Line(points = {{-66, 60}, {-40, 60}, {-40, 29}, {-14, 29}}, color = {0, 0, 127}, thickness = 0.5));
  connect(z_road.y[3], RideModel.z_road_rl) annotation(
    Line(points = {{-66, 60}, {-46, 60}, {-46, 12}, {-14, 12}, {-14, 14}}, color = {0, 0, 127}, thickness = 0.5));
  connect(z_road.y[4], RideModel.z_road_rr) annotation(
    Line(points = {{-66, 60}, {-52, 60}, {-52, -2}, {-14, -2}, {-14, -2}}, color = {0, 0, 127}, thickness = 0.5));    
  annotation(
    uses(Modelica(version = "3.2.2")));
    
end RideModelWithExternalRoadInput;
