model DWB_Suspension
  inner Modelica.Mechanics.MultiBody.World world
    annotation (Placement(transformation(extent={{-100,-100},{-80,-80}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed Ground
    annotation (Placement(transformation(extent={{-86,-46},{-66,-26}})));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation Chassis_to_damper(
    r(displayUnit="mm") = {0,0.05,0.33},
    rotationType=Modelica.Mechanics.MultiBody.Types.RotationTypes.TwoAxesVectors,
    n_y={0,-1,0},
    n_x={0,0,1})
    annotation (Placement(transformation(extent={{-32,68},{-12,88}})));

  Modelica.Mechanics.MultiBody.Parts.FixedRotation Chassis_to_rocker(
    rotationType=Modelica.Mechanics.MultiBody.Types.RotationTypes.TwoAxesVectors,
    n_y={0,-1,0},
    r(displayUnit="mm") = {0,0.4,0.35},
    n_x={0,0,1})
    annotation (Placement(transformation(extent={{-32,14},{-12,34}})));

  Modelica.Mechanics.MultiBody.Parts.FixedRotation Chassis_to_Arm1_Right(
    n_y={0,-1,0},
    n_x={0,0,-1},
    r(displayUnit="mm") = {0,0.5,0.15},
    rotationType=Modelica.Mechanics.MultiBody.Types.RotationTypes.RotationAxis,+
    n={0,1,0},
    angle=90)
    annotation (Placement(transformation(extent={{-32,-46},{-12,-26}})));

  Modelica.Mechanics.MultiBody.Parts.FixedRotation Chassis_to_Arm2_Right(
    n_y={0,-1,0},
    n_x={0,0,-1},
    rotationType=Modelica.Mechanics.MultiBody.Types.RotationTypes.RotationAxis,
    n={0,1,0},
    angle=90,
    r(displayUnit="mm") = {0,0.5,-0.15})
    annotation (Placement(transformation(extent={{-32,-74},{-12,-54}})));

  Modelica.Mechanics.MultiBody.Parts.BodyBox Solid(
    width(displayUnit="mm") = 1,
    r_shape(displayUnit="mm") = {0,-0.5,0},
    widthDirection={1,0,0},
    height(displayUnit="mm") = 0.69,
    r(displayUnit="mm") = {0,0.5,0}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-56,-12})));
  Modelica.Mechanics.MultiBody.Joints.Revolute Chassis_Damper_Rev_Joint(n={0,
        0,1}, cylinderColor={255,255,0},
    stateSelect=StateSelect.always)
    annotation (Placement(transformation(extent={{0,68},{20,88}})));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation Damper_to_Chassis(
    rotationType=Modelica.Mechanics.MultiBody.Types.RotationTypes.TwoAxesVectors,
    r(displayUnit="mm") = {0,0,0.035},
    n_x={0,0,-1},
    n_y={1,0,0})
    annotation (Placement(transformation(extent={{54,68},{34,88}})));

  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Cylinder(
                     diameter(displayUnit="mm") = 0.04,
    r(displayUnit="mm") = {0,0,0.035},
    r_shape(displayUnit="mm") = {0,0,-0.035})
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={76,100})));
  Modelica.Mechanics.MultiBody.Joints.Revolute Chassis_Rocker_Rev_Joint(n={0,
        0,1}, cylinderColor={255,255,0})
    annotation (Placement(transformation(extent={{-2,14},{18,34}})));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation Rocker_to_Chassis(
    rotationType=Modelica.Mechanics.MultiBody.Types.RotationTypes.TwoAxesVectors,
    r(displayUnit="mm") = {0,-0.175,-0.05},
    n_x={0,0,1},
    n_y={0,1,0})
    annotation (Placement(transformation(extent={{52,14},{32,34}})));

  Modelica.Mechanics.MultiBody.Parts.BodyBox Rocker(
    widthDirection={1,0,0},
    width(displayUnit="mm") = 0.005,
    height(displayUnit="mm") = 0.1,
    length(displayUnit="mm"),
    r_shape(displayUnit="mm") = {0,-0.175,0},
    r(displayUnit="mm") = {0,0.175,0}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={74,14})));
  Modelica.Mechanics.MultiBody.Joints.Revolute Chassis_Arm1_Rev_Joint(n={0,0,
        1}, cylinderColor={255,255,0})
    annotation (Placement(transformation(extent={{-2,-46},{18,-26}})));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Arm1(
    r(displayUnit="mm") = {0,0,0.35},
    r_shape(displayUnit="mm") = {0,0,-0.35},
    diameter(displayUnit="mm") = 0.04) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={74,-18})));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation Arm_to_Chassis1(
    rotationType=Modelica.Mechanics.MultiBody.Types.RotationTypes.TwoAxesVectors,
    r(displayUnit="mm") = {0,0,-0.35},
    n_x={1,0,0},
    n_y={0,0,1})
    annotation (Placement(transformation(extent={{50,-46},{30,-26}})));

  Modelica.Mechanics.MultiBody.Joints.Revolute Chassis_Arm2_Rev_Joint(n={0,0,
        1}, cylinderColor={255,255,0})
    annotation (Placement(transformation(extent={{-2,-74},{18,-54}})));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Arm2(
    r(displayUnit="mm") = {0,0,0.35},
    r_shape(displayUnit="mm") = {0,0,-0.35},
    diameter(displayUnit="mm") = 0.04) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={72,-84})));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation Arm_to_Chassis2(
    rotationType=Modelica.Mechanics.MultiBody.Types.RotationTypes.TwoAxesVectors,
    r(displayUnit="mm") = {0,0,-0.35},
    n_x={1,0,0},
    n_y={0,0,1})
    annotation (Placement(transformation(extent={{50,-74},{30,-54}})));

  Modelica.Mechanics.MultiBody.Parts.FixedRotation Arm_to_wheel2(
    rotationType=Modelica.Mechanics.MultiBody.Types.RotationTypes.TwoAxesVectors,
    r(displayUnit="mm") = {0,0,0.35},
    n_x={0,0,1},
    n_y={-1,0,0})
    annotation (Placement(transformation(extent={{102,-74},{122,-54}})));

  Modelica.Mechanics.MultiBody.Joints.Revolute Arm2_wheel_Rev_Joint(n={0,0,1},
      cylinderColor={255,255,0})
    annotation (Placement(transformation(extent={{138,-74},{158,-54}})));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation Wheel_to_arm2(
    rotationType=Modelica.Mechanics.MultiBody.Types.RotationTypes.TwoAxesVectors,
    r(displayUnit="mm") = {0,-0.15,0.15},
    n_x={0,0,-1},
    n_y={0,1,0})
    annotation (Placement(transformation(extent={{192,-74},{172,-54}})));

  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Wheel(
    diameter(displayUnit="mm") = 1,
    r(displayUnit="mm") = {0,0,0.15},
    r_shape(displayUnit="mm") = {0,0,-0.15},
    color={155,155,155})
    annotation (Placement(transformation(extent={{326,-74},{346,-54}})));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation Arm_to_wheel1(
    rotationType=Modelica.Mechanics.MultiBody.Types.RotationTypes.TwoAxesVectors,
    r(displayUnit="mm") = {0,0,0.35},
    n_x={0,0,1},
    n_y={-1,0,0})
    annotation (Placement(transformation(extent={{100,-46},{120,-26}})));

  Modelica.Mechanics.MultiBody.Parts.FixedRotation Rocker_to_push(
    n_y={0,-1,0},
    n_x={0,0,-1},
    r(displayUnit="mm") = {0,-0.175,0.05},
    rotationType=Modelica.Mechanics.MultiBody.Types.RotationTypes.PlanarRotationSequence,
    sequence={2,1,3},
    angles={90,180,-36.8699})
    annotation (Placement(transformation(extent={{102,14},{122,34}})));

  Modelica.Mechanics.MultiBody.Joints.Revolute Rocker_Push_Rev_Joint(n={0,0,1},
      cylinderColor={255,255,0})
    annotation (Placement(transformation(extent={{160,14},{140,34}})));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Push(
    diameter(displayUnit="mm") = 0.04,
    r(displayUnit="mm") = {0,0,0.5},
    r_shape(displayUnit="mm") = {0,0,-0.5}) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={198,2})));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation Push_to_chassis(
    sequence={2,1,3},
    angles={90,180,-36.8699},
    r(displayUnit="mm") = {0,0,-0.5},
    rotationType=Modelica.Mechanics.MultiBody.Types.RotationTypes.TwoAxesVectors,
    n_x={1,0,0},
    n_y={0,0,1})
    annotation (Placement(transformation(extent={{188,14},{168,34}})));

  Modelica.Mechanics.MultiBody.Parts.FixedRotation Wheel_to_arm1(
    rotationType=Modelica.Mechanics.MultiBody.Types.RotationTypes.TwoAxesVectors,
    n_x={0,0,-1},
    n_y={0,1,0},
    r(displayUnit="mm") = {0,0.15,0.15})
    annotation (Placement(transformation(extent={{190,-46},{170,-26}})));

  Modelica.Mechanics.MultiBody.Joints.RevolutePlanarLoopConstraint
    Arm1_wheel_rev_joint
    annotation (Placement(transformation(extent={{140,-46},{160,-26}})));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation push_to_wheel(
    sequence={2,1,3},
    angles={90,180,-36.8699},
    r(displayUnit="mm") = {0,0,0.5},
    rotationType=Modelica.Mechanics.MultiBody.Types.RotationTypes.TwoAxesVectors,
    n_x={0,0,1},
    n_y={-1,0,0})
    annotation (Placement(transformation(extent={{216,14},{236,34}})));

  Modelica.Mechanics.MultiBody.Parts.FixedRotation Wheel_to_push(
    r(displayUnit="mm") = {0,-0.15,0.15},
    n_x={0,0,-1},
    n_y={0,1,0},
    rotationType=Modelica.Mechanics.MultiBody.Types.RotationTypes.PlanarRotationSequence,
    sequence={2,1,3},
    angles={90,0,-36.8699})
    annotation (Placement(transformation(extent={{302,-18},{282,2}})));

  Modelica.Mechanics.MultiBody.Joints.RevolutePlanarLoopConstraint
    Push_Wheel_Rev_Joint
    annotation (Placement(transformation(extent={{244,-6},{264,14}})));
  Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(
    n={0,0,1},
    useAxisFlange=true,
    s(displayUnit="mm", start=0))
    annotation (Placement(transformation(extent={{100,68},{120,88}})));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Piston(
    r(displayUnit="mm") = {0,0,0.035},
    r_shape(displayUnit="mm") = {0,0,-0.035},
    diameter(displayUnit="mm") = 0.02,
    color={155,0,0}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={138,100})));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation Damper_to_Rocker(
    sequence={2,1,3},
    angles={90,180,-36.8699},
    rotationType=Modelica.Mechanics.MultiBody.Types.RotationTypes.TwoAxesVectors,
    n_x={0,0,1},
    r(displayUnit="mm") = {0,0,-0.035},
    n_y={1,0,0})
    annotation (Placement(transformation(extent={{158,68},{178,88}})));

  Modelica.Mechanics.MultiBody.Joints.RevolutePlanarLoopConstraint
    Damper_Rocker_Rev_Joint(cylinderColor={0,0,0})
    annotation (Placement(transformation(extent={{220,68},{200,88}})));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation Rocker_to_Damper(
    sequence={2,1,3},
    angles={90,180,-36.8699},
    rotationType=Modelica.Mechanics.MultiBody.Types.RotationTypes.TwoAxesVectors,
    r(displayUnit="mm") = {0,0.175,0.05},
    n_x={0,0,1},
    n_y={0,1,0})
    annotation (Placement(transformation(extent={{258,68},{238,88}})));

  Modelica.Mechanics.Translational.Sources.Position position
    annotation (Placement(transformation(extent={{88,150},{108,170}})));
  Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePosition(
      resolveInFrame=Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world)
    annotation (Placement(transformation(extent={{332,78},{352,98}})));
  Modelica.Blocks.Interfaces.RealInput s_ref1
    "Reference position of flange as input signal"
    annotation (Placement(transformation(extent={{-142,140},{-102,180}})));
  Modelica.Blocks.Interfaces.RealOutput r1[3]
    "Absolute position vector resolved in frame defined by resolveInFrame"
    annotation (Placement(transformation(extent={{372,78},{392,98}})));
  Modelica.Blocks.Math.Add add(k1=0.001, k2=0.001)
    annotation (Placement(transformation(extent={{-36,144},{-16,164}})));
  Modelica.Blocks.Sources.Constant const(k=-137.6)
    annotation (Placement(transformation(extent={{-80,122},{-60,142}})));
equation
  connect(Ground.frame_b, Chassis_to_damper.frame_a) annotation (Line(
      points={{-66,-36},{-40,-36},{-40,78},{-32,78}},
      color={95,95,95},
      thickness=0.5));
  connect(Chassis_to_rocker.frame_a, Chassis_to_damper.frame_a) annotation (
      Line(
      points={{-32,24},{-40,24},{-40,78},{-32,78}},
      color={95,95,95},
      thickness=0.5));
  connect(Chassis_to_Arm1_Right.frame_a, Chassis_to_damper.frame_a)
    annotation (Line(
      points={{-32,-36},{-40,-36},{-40,78},{-32,78}},
      color={95,95,95},
      thickness=0.5));
  connect(Chassis_to_Arm2_Right.frame_a, Chassis_to_damper.frame_a)
    annotation (Line(
      points={{-32,-64},{-40,-64},{-40,78},{-32,78}},
      color={95,95,95},
      thickness=0.5));
  connect(Solid.frame_a, Chassis_to_damper.frame_a) annotation (Line(
      points={{-56,-22},{-56,-36},{-40,-36},{-40,78},{-32,78}},
      color={95,95,95},
      thickness=0.5));
  connect(Chassis_to_damper.frame_b, Chassis_Damper_Rev_Joint.frame_a)
    annotation (Line(
      points={{-12,78},{0,78}},
      color={95,95,95},
      thickness=0.5));
  connect(Damper_to_Chassis.frame_b, Chassis_Damper_Rev_Joint.frame_b)
    annotation (Line(
      points={{34,78},{20,78}},
      color={95,95,95},
      thickness=0.5));
  connect(Chassis_to_rocker.frame_b, Chassis_Rocker_Rev_Joint.frame_a)
    annotation (Line(
      points={{-12,24},{-2,24}},
      color={95,95,95},
      thickness=0.5));
  connect(Rocker_to_Chassis.frame_b, Chassis_Rocker_Rev_Joint.frame_b)
    annotation (Line(
      points={{32,24},{18,24}},
      color={95,95,95},
      thickness=0.5));
  connect(Rocker.frame_a, Rocker_to_Chassis.frame_a) annotation (Line(
      points={{74,24},{52,24}},
      color={95,95,95},
      thickness=0.5));
  connect(Chassis_to_Arm1_Right.frame_b, Chassis_Arm1_Rev_Joint.frame_a)
    annotation (Line(
      points={{-12,-36},{-2,-36}},
      color={95,95,95},
      thickness=0.5));
  connect(Chassis_Arm1_Rev_Joint.frame_b, Arm_to_Chassis1.frame_b)
    annotation (Line(
      points={{18,-36},{30,-36}},
      color={95,95,95},
      thickness=0.5));
  connect(Arm1.frame_a, Arm_to_Chassis1.frame_a) annotation (Line(
      points={{74,-28},{74,-36},{50,-36}},
      color={95,95,95},
      thickness=0.5));
  connect(Chassis_to_Arm2_Right.frame_b, Chassis_Arm2_Rev_Joint.frame_a)
    annotation (Line(
      points={{-12,-64},{-2,-64}},
      color={95,95,95},
      thickness=0.5));
  connect(Chassis_Arm2_Rev_Joint.frame_b, Arm_to_Chassis2.frame_b)
    annotation (Line(
      points={{18,-64},{30,-64}},
      color={95,95,95},
      thickness=0.5));
  connect(Arm2.frame_a, Arm_to_Chassis2.frame_a) annotation (Line(
      points={{72,-74},{72,-64},{50,-64}},
      color={95,95,95},
      thickness=0.5));
  connect(Arm_to_wheel2.frame_b, Arm2_wheel_Rev_Joint.frame_a) annotation (
      Line(
      points={{122,-64},{138,-64}},
      color={95,95,95},
      thickness=0.5));
  connect(Arm2_wheel_Rev_Joint.frame_b, Wheel_to_arm2.frame_b) annotation (
      Line(
      points={{158,-64},{172,-64}},
      color={95,95,95},
      thickness=0.5));
  connect(Wheel_to_arm2.frame_a, Wheel.frame_a) annotation (Line(
      points={{192,-64},{326,-64}},
      color={95,95,95},
      thickness=0.5));
  connect(Arm_to_wheel2.frame_a, Arm_to_Chassis2.frame_a) annotation (Line(
      points={{102,-64},{50,-64}},
      color={95,95,95},
      thickness=0.5));
  connect(Arm_to_wheel1.frame_a, Arm_to_Chassis1.frame_a) annotation (Line(
      points={{100,-36},{50,-36}},
      color={95,95,95},
      thickness=0.5));
  connect(Rocker.frame_a, Rocker_to_push.frame_a) annotation (Line(
      points={{74,24},{102,24}},
      color={95,95,95},
      thickness=0.5));
  connect(Push.frame_a, Push_to_chassis.frame_a) annotation (Line(
      points={{198,12},{198,24},{188,24}},
      color={95,95,95},
      thickness=0.5));
  connect(Arm_to_wheel1.frame_b, Arm1_wheel_rev_joint.frame_a) annotation (
      Line(
      points={{120,-36},{140,-36}},
      color={95,95,95},
      thickness=0.5));
  connect(Arm1_wheel_rev_joint.frame_b, Wheel_to_arm1.frame_b) annotation (
      Line(
      points={{160,-36},{170,-36}},
      color={95,95,95},
      thickness=0.5));
  connect(Wheel_to_arm1.frame_a, Wheel.frame_a) annotation (Line(
      points={{190,-36},{198,-36},{198,-64},{326,-64}},
      color={95,95,95},
      thickness=0.5));
  connect(push_to_wheel.frame_a, Push_to_chassis.frame_a) annotation (Line(
      points={{216,24},{188,24}},
      color={95,95,95},
      thickness=0.5));
  connect(Wheel_to_push.frame_a, Wheel.frame_a) annotation (Line(
      points={{302,-8},{308,-8},{308,-64},{326,-64}},
      color={95,95,95},
      thickness=0.5));
  connect(Rocker_Push_Rev_Joint.frame_a, Push_to_chassis.frame_b) annotation (
     Line(
      points={{160,24},{168,24}},
      color={95,95,95},
      thickness=0.5));
  connect(Rocker_Push_Rev_Joint.frame_b, Rocker_to_push.frame_b) annotation (
      Line(
      points={{140,24},{130,24},{130,24},{122,24}},
      color={95,95,95},
      thickness=0.5));
  connect(push_to_wheel.frame_b, Push_Wheel_Rev_Joint.frame_a) annotation (
      Line(
      points={{236,24},{240,24},{240,4},{244,4}},
      color={95,95,95},
      thickness=0.5));
  connect(Push_Wheel_Rev_Joint.frame_b, Wheel_to_push.frame_b) annotation (
      Line(
      points={{264,4},{274,4},{274,-8},{282,-8}},
      color={95,95,95},
      thickness=0.5));
  connect(prismatic.frame_b, Piston.frame_a) annotation (Line(
      points={{120,78},{138,78},{138,90}},
      color={95,95,95},
      thickness=0.5));
  connect(Damper_to_Rocker.frame_a, Piston.frame_a) annotation (Line(
      points={{158,78},{138,78},{138,90}},
      color={95,95,95},
      thickness=0.5));
  connect(Damper_to_Chassis.frame_a, prismatic.frame_a) annotation (Line(
      points={{54,78},{100,78}},
      color={95,95,95},
      thickness=0.5));
  connect(Cylinder.frame_a, prismatic.frame_a) annotation (Line(
      points={{76,90},{76,78},{100,78}},
      color={95,95,95},
      thickness=0.5));
  connect(Rocker_to_Damper.frame_a, Rocker.frame_a) annotation (Line(
      points={{258,78},{270,78},{270,46},{74,46},{74,24}},
      color={95,95,95},
      thickness=0.5));
  connect(Damper_to_Rocker.frame_b, Damper_Rocker_Rev_Joint.frame_b)
    annotation (Line(
      points={{178,78},{200,78}},
      color={95,95,95},
      thickness=0.5));
  connect(Rocker_to_Damper.frame_b, Damper_Rocker_Rev_Joint.frame_a)
    annotation (Line(
      points={{238,78},{220,78}},
      color={95,95,95},
      thickness=0.5));
  connect(position.flange, prismatic.axis) annotation (Line(points={{108,160},
          {114,160},{114,84},{118,84}}, color={0,127,0}));
  connect(absolutePosition.frame_a, Wheel.frame_a) annotation (Line(
      points={{332,88},{322,88},{322,-38},{308,-38},{308,-64},{326,-64}},
      color={95,95,95},
      thickness=0.5));
  connect(absolutePosition.r[3], r1[1]) annotation (Line(points={{353,88.6667},
          {362,88.6667},{362,88},{372,88},{372,81.3333},{382,81.3333}}, color=
         {0,0,127}));
  connect(s_ref1, add.u1)
    annotation (Line(points={{-122,160},{-38,160}}, color={0,0,127}));
  connect(const.y, add.u2) annotation (Line(points={{-59,132},{-48,132},{-48,
          148},{-38,148}}, color={0,0,127}));
  connect(add.y, position.s_ref) annotation (Line(points={{-15,154},{36,154},
          {36,160},{86,160}}, color={0,0,127}));
  annotation (Diagram(coordinateSystem(extent={{-100,-100},{360,220}})), Icon(
        coordinateSystem(extent={{-100,-100},{360,220}}), graphics={
        Rectangle(
          extent={{84,132},{312,-22}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.HorizontalCylinder),
        Rectangle(
          extent={{-22,72},{84,62}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={215,215,215}),
        Rectangle(
          extent={{-22,22},{84,12}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={215,215,215}),
        Polygon(
          points={{-26,22},{88,146},{94,140},{-22,16},{-26,22}},
          lineThickness=1,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={0,0,0},
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Polygon(
          points={{94,138},{98,136},{94,138}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={95,95,95}),
        Polygon(
          points={{86,146},{106,132},{118,158},{86,146}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={0,0,0},
          lineColor={0,0,0}),
        Rectangle(
          extent={{-82,138},{-22,-38}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={95,95,95}),
        Text(
          extent={{6,-28},{292,-94}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          lineThickness=1,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={0,0,0},
          textString="Simple STR Pushrod")}),
    experiment(StopTime=10));
end DWB_Suspension;
