package PruefungsVorleistung
  package Electrics
    package BasisElements
      model Widerstand "Modell eines Widerstands"
        extends PruefungsVorleistung.Interfaces.TwoPin;
        parameter Modelica.Units.SI.Resistance R = 10 "Widerstand";
      equation
        v = R*p.i;
        annotation(
          Icon(graphics = {Line(origin = {-64, 0}, points = {{-24, 0}, {24, 0}, {24, 0}, {24, 0}}, color = {0, 0, 255}, thickness = 1), Rectangle(lineColor = {0, 0, 255}, lineThickness = 1, extent = {{-40, 20}, {40, -20}}), Line(origin = {67.3396, -0.230041}, points = {{-28, 5.55112e-17}, {24, 0}, {24, 0}, {24, 0}}, color = {0, 0, 255}, thickness = 1), Text(origin = {0, 55}, extent = {{-98, 15}, {98, -15}}, textString = "%name"), Text(origin = {1, -42}, extent = {{-103, 14}, {103, -14}}, textString = "R = %R")}));
      end Widerstand;

      model Erdung "Modell einer Erdung"
        PruefungsVorleistung.Interfaces.Pin p annotation(
          Placement(visible = true, transformation(origin = {4, 98}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 94}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        p.v = 0;
        annotation(
          Icon(graphics = {Line(origin = {0, 42}, points = {{0, 46}, {0, -42}}, color = {0, 0, 255}, thickness = 1), Line(origin = {22.9281, -40.8536}, rotation = -90, points = {{0, 8}, {0, -54}}, color = {0, 0, 255}, thickness = 1), Line(origin = {21.8859, -20.531}, rotation = -90, points = {{0, 38}, {0, -82}}, color = {0, 0, 255}, thickness = 1), Line(origin = {0.260566, -0.469002}, rotation = -90, points = {{0, 80}, {0, -80}}, color = {0, 0, 255}, thickness = 1), Text(origin = {0, -76}, extent = {{-100, 24}, {100, -24}}, textString = "%name")}));
      end Erdung;

      model SpannungsquelleDC "Modell einer Spannungsquelle"
        extends PruefungsVorleistung.Interfaces.TwoPin;
        parameter Modelica.Units.SI.ElectricPotential A = 12 "Amplitude";
      equation
        v = A;
        annotation(
          Icon(graphics = {Ellipse(lineColor = {0, 0, 255}, lineThickness = 1, extent = {{-30, -30}, {30, 30}}), Line(origin = {-2, 0}, points = {{-90, 0}, {90, 0}}, color = {0, 0, 255}, thickness = 1), Text(origin = {0, 80}, extent = {{-100, 16}, {100, -16}}, textString = "%name"), Text(origin = {-1, -70}, extent = {{-99, 18}, {99, -18}}, textString = "A = %A
          f = %f")}));
      end SpannungsquelleDC;

      model StromStaerkenMesser "Modell eines Stromstaekenmessers"
        extends PruefungsVorleistung.Interfaces.TwoPin;
        PruefungsVorleistung.Interfaces.RealOutput i annotation(
          Placement(visible = true, transformation(origin = {2, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, -102}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        p.v = n.v;
        p.i = i;
        annotation(
          Icon(graphics = {Ellipse(origin = {2, -3}, extent = {{-86, 89}, {86, -89}}), Line(origin = {-83.59, -1.17}, points = {{-4.17082, 1.17082}, {-0.17082, 1.17082}, {-10.1708, 1.17082}}, color = {0, 0, 255}, thickness = 1), Line(origin = {92.0763, -1.40516}, points = {{-4.17082, 1.17082}, {-0.17082, 1.17082}, {-4.1708, 1.17082}}, color = {0, 0, 255}, thickness = 1), Line(origin = {0, -96}, points = {{0, 4}, {0, -4}}, thickness = 1), Line(origin = {23.79, 22.79}, points = {{-23.793, -24.793}, {24.207, 25.207}, {24.207, 15.207}, {16.207, 25.207}, {24.207, 25.207}, {24.207, 15.207}}, thickness = 0.75), Text(origin = {0, 116}, extent = {{-100, 16}, {100, -16}}, textString = "%name")}));
      end StromStaerkenMesser;

      model Schalter "Model eines idealen Schalters ohne Lichtbogen effekt"
        extends PruefungsVorleistung.Interfaces.TwoPin;
        PruefungsVorleistung.Interfaces.WahrheitswertInput off annotation(
          Placement(visible = true, transformation(origin = {0, 104}, extent = {{24, -24}, {-24, 24}}, rotation = 90), iconTransformation(origin = {0, 96}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
        
        parameter Modelica.Units.SI.Resistance Ron(final min=0) = 1e-5
          "Widerstand bei geschlossenem Schalter";
        parameter Modelica.Units.SI.Conductance Goff(final min=0) = 1e-5
          "Leitfähigkeit bei offenem Schalter";
        
        
        Real s(final unit="1") "Hilfsvariable";
        constant Modelica.Units.SI.Voltage unitVoltage=1 annotation (HideResult=true);
        constant Modelica.Units.SI.Current unitCurrent=1 annotation (HideResult=true);
      
      equation
        v = (s*unitCurrent)*(if off then 1 else Ron);
        p.i = (s*unitVoltage)*(if off then Goff else 1);
        
        annotation(
          Icon(graphics = {Line(origin = {-26.32, 14.82}, points = {{-65.6836, -14.8164}, {-9.68358, -14.8164}, {66.3164, 15.1836}, {-9.68358, -14.8164}}, color = {0, 0, 255}, thickness = 1), Line(origin = {59, 0}, points = {{33, 0}, {-33, 0}}, color = {0, 0, 255}, thickness = 1), Text(origin = {0, -84}, extent = {{-100, 16}, {100, -16}}, textString = "%name")}));
      end Schalter;

      model VariablerWiderstand "Modell eines Variablen Widerstands"
        extends PruefungsVorleistung.Interfaces.TwoPin;
        Interfaces.RealInput R(unit = "Ohm") annotation(
          Placement(visible = true, transformation(origin = {-6, 82}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {1, 103}, extent = {{-37, -37}, {37, 37}}, rotation = 0)));
      equation
        v = R*p.i;
        annotation(
          Icon(graphics = {Line(origin = {-64, 0}, points = {{-24, 0}, {24, 0}, {24, 0}, {24, 0}}, color = {0, 0, 255}, thickness = 1), Rectangle(lineColor = {0, 0, 255}, lineThickness = 1, extent = {{-40, 20}, {40, -20}}), Line(origin = {67.3396, -0.230041}, points = {{-28, 5.55112e-17}, {24, 0}, {24, 0}, {24, 0}}, color = {0, 0, 255}, thickness = 1), Text(origin = {0, 55}, extent = {{-98, 15}, {98, -15}}, textString = "%name"), Text(origin = {1, -42}, extent = {{-103, 14}, {103, -14}}, textString = "R = %R")}));
      end VariablerWiderstand;
    end BasisElements;
  end Electrics;

  model Sicherung
    Modelica.Blocks.Sources.Trapezoid trapezoid(amplitude = -999, falling = 400, offset = 999 + 1, period = 1000, rising = 400, width = 100) annotation(
      Placement(visible = true, transformation(origin = {118, -62}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    PruefungsVorleistung.Electrics.BasisElements.SpannungsquelleDC spannungsquelleDC(A = 100) annotation(
      Placement(visible = true, transformation(origin = {-74, -62}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    PruefungsVorleistung.Electrics.BasisElements.Erdung erdung annotation(
      Placement(visible = true, transformation(origin = {-6, -84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    PruefungsVorleistung.Electrics.BasisElements.StromStaerkenMesser stromStaerkenMesser annotation(
      Placement(visible = true, transformation(origin = {-30, -40}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    PruefungsVorleistung.Electrics.BasisElements.VariablerWiderstand variablerWiderstand annotation(
      Placement(visible = true, transformation(origin = {62, -62}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  PruefungsVorleistung.maths.Betrag betrag annotation(
      Placement(visible = true, transformation(origin = {-30, -4}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  PruefungsVorleistung.maths.Konstante konstante(k = 10)  annotation(
      Placement(visible = true, transformation(origin = {90, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  PruefungsVorleistung.maths.konstanterWahrheitswert konstanterWahrheitswert(value = false)  annotation(
      Placement(visible = true, transformation(origin = {2, 18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  PruefungsVorleistung.maths.groesserAls groesserAls annotation(
      Placement(visible = true, transformation(origin = {50, 44}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  PruefungsVorleistung.Logical.RsFlipFlop rsFlipFlop(Qini = false)  annotation(
      Placement(visible = true, transformation(origin = {52, 2}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  PruefungsVorleistung.Electrics.BasisElements.Schalter schalter(Goff = 1e-6, Ron = 1e-6)  annotation(
      Placement(visible = true, transformation(origin = {30, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  PruefungsVorleistung.maths.PT1 pt1(T = 0.001)  annotation(
      Placement(visible = true, transformation(origin = {-4, 62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(erdung.p, spannungsquelleDC.n) annotation(
      Line(points = {{-6, -74}, {-74, -74}, {-74, -72}}, color = {0, 0, 255}));
    connect(stromStaerkenMesser.p, spannungsquelleDC.p) annotation(
      Line(points = {{-40, -40}, {-74, -40}, {-74, -52}}, color = {0, 0, 255}));
    connect(variablerWiderstand.n, erdung.p) annotation(
      Line(points = {{62, -72}, {62, -74}, {-6, -74}}, color = {0, 0, 255}));
    connect(trapezoid.y, variablerWiderstand.R) annotation(
      Line(points = {{108, -62}, {72, -62}}, color = {0, 0, 127}));
    connect(stromStaerkenMesser.i, betrag.realOutput) annotation(
      Line(points = {{-30, -30}, {-30, -14}}));
    connect(groesserAls.y, rsFlipFlop.S) annotation(
      Line(points = {{46, 32}, {46, 24}, {59, 24}, {59, 13}}, color = {255, 0, 255}));
    connect(schalter.n, variablerWiderstand.p) annotation(
      Line(points = {{40, -40}, {62, -40}, {62, -52}}, color = {0, 0, 255}));
    connect(schalter.p, stromStaerkenMesser.n) annotation(
      Line(points = {{20, -40}, {-20, -40}}, color = {0, 0, 255}));
    connect(rsFlipFlop.y1, schalter.off) annotation(
      Line(points = {{59, -9}, {59, -24}, {30, -24}, {30, -30}}, color = {255, 0, 255}));
    connect(rsFlipFlop.R, konstanterWahrheitswert.y) annotation(
      Line(points = {{47, 13}, {47, 18}, {12, 18}}, color = {255, 0, 255}));
  connect(pt1.y, groesserAls.realInput) annotation(
      Line(points = {{6, 62}, {44, 62}, {44, 54}}));
  connect(betrag.realInput, pt1.u) annotation(
      Line(points = {{-30, 6}, {-30, 62}, {-14, 62}}));
  connect(konstante.realOutput, groesserAls.realInput1) annotation(
      Line(points = {{80, 40}, {60, 40}}));
  end Sicherung;

  package maths
   
model Betrag"Betragsbildung y = abs(x)" 
      PruefungsVorleistung.Interfaces.RealInput realInput annotation(
        Placement(visible = true, transformation(origin = {-14, -48}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, -100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      PruefungsVorleistung.Interfaces.RealOutput realOutput annotation(
        Placement(visible = true, transformation(origin = {-20, 26}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      realOutput = abs(realInput);
      annotation(
        Icon(graphics = {Rectangle(lineThickness = 1, extent = {{-100, 100}, {100, -100}}), Text(origin = {-10, 1}, rotation = -90, extent = {{-100, 71}, {100, -71}}, textString = "| x |"), Text(origin = {126, 0}, rotation = -90, extent = {{-100, 26}, {100, -26}}, textString = "%name")}));
    end Betrag;

    model Konstante "Konstante"
  PruefungsVorleistung.Interfaces.RealOutput realOutput annotation(
        Placement(visible = true, transformation(origin = {-12, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {108, -2}, extent = {{-26, -26}, {26, 26}}, rotation = 90)));
    parameter Real k(start=1) "Konstante Ausgabe";
    
    equation

    realOutput = k;
    
    annotation(
        Icon(graphics = {Rectangle(origin = {1, -1}, lineThickness = 1, extent = {{-99, 98}, {99, -98}}), Text(origin = {-3, -131}, extent = {{-95, 31}, {95, -31}}, textString = "%name"), Text(origin = {2, 3}, extent = {{-78, 57}, {78, -57}}, textString = "k = %k")}));
end Konstante;

    model konstanterWahrheitswert" Konstanter Wahrheitswert"
  PruefungsVorleistung.Interfaces.WahrheitswertOutput y annotation(
        Placement(visible = true, transformation(origin = {94, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {104, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    parameter Boolean value(start=false);
    
    equation
    
    y = value;

    annotation(
        Icon(graphics = {Rectangle( lineColor = {255, 0, 255}, lineThickness = 1, extent = {{-100, 100}, {100, -100}}), Text(origin = {-1, -138}, extent = {{99, -38}, {-99, 38}}, textString = "%name"), Text(origin = {0, 6}, extent = {{-98, 34}, {98, -34}}, textString = "%value")}));
end konstanterWahrheitswert;

    model groesserAls"Groesser Gleich Vergleich "
  PruefungsVorleistung.Interfaces.RealInput realInput annotation(
        Placement(visible = true, transformation(origin = {-6, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-101, -65}, extent = {{-29, -29}, {29, 29}}, rotation = 90)));
  PruefungsVorleistung.Interfaces.RealInput realInput1 annotation(
        Placement(visible = true, transformation(origin = {60, 86}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {50, 102}, extent = {{-30, -30}, {30, 30}}, rotation = 0)));
  PruefungsVorleistung.Interfaces.WahrheitswertOutput y annotation(
        Placement(visible = true, transformation(origin = {12, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {108, -60}, extent = {{-22, -22}, {22, 22}}, rotation = 0)));
    
    
    equation
 y = realInput >= realInput1;
     
    annotation(
        Icon(graphics = {Text(origin = {0, 35}, rotation = 180, extent = {{-99, 64}, {99, -64}}, textString = ">="), Text(origin = {-3, -131}, extent = {{97, -31}, {-97, 31}}, textString = "%name"), Rectangle(lineThickness = 1, extent = {{-100, 100}, {100, -100}})}));
end groesserAls;

    model PT1 "Modell eines proportionalen Uebertragungsglieds 1. Ordnung"
  PruefungsVorleistung.Interfaces.RealInput u annotation(
        Placement(visible = true, transformation(origin = {-106, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -2}, extent = {{10, 10}, {-10, -10}}, rotation = -90)));
  PruefungsVorleistung.Interfaces.RealOutput y(start = 0, fixed = true) annotation(
        Placement(visible = true, transformation(origin = {102, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
    
    
      parameter Real k(unit="1")=1 "Verstaerkung";
      parameter Modelica.Units.SI.Time T(start=1) "Zeitkonstante";
    
    equation

    
    der(y) = (k*u - y)/T;
    
    annotation(
        Icon(graphics = {Rectangle(lineThickness = 1, extent = {{-100, 100}, {100, -100}}), Text(origin = {0, -124}, extent = {{-100, 20}, {100, -20}}, textString = "%name"), Line(origin = {-0.13, -13.13}, points = {{-99.8675, -86.8675}, {-97.8675, -80.8675}, {-93.8675, -74.8675}, {-91.8675, -66.8675}, {-89.8675, -62.8675}, {-85.8675, -56.8675}, {-83.8675, -50.8675}, {-81.8675, -44.8675}, {-79.8675, -38.8675}, {-75.8675, -34.8675}, {-73.8675, -28.8675}, {-69.8675, -24.8675}, {-67.8675, -18.8675}, {-61.8675, -12.8675}, {-57.8675, -6.86754}, {-53.8675, -0.867544}, {-49.8675, 5.13246}, {-45.8675, 11.1325}, {-39.8675, 15.1325}, {-33.8675, 23.1325}, {-27.8675, 29.1325}, {-23.8675, 33.1325}, {-17.8675, 41.1325}, {-11.8675, 47.1325}, {-5.86754, 53.1325}, {2.13246, 59.1325}, {14.1325, 65.1325}, {24.1325, 71.1325}, {34.1325, 75.1325}, {42.1325, 77.1325}, {48.1325, 79.1325}, {56.1325, 81.1325}, {62.1325, 83.1325}, {70.1325, 85.1325}, {76.1325, 87.1325}, {84.1325, 87.1325}, {90.1325, 87.1325}, {94.1325, 87.1325}, {98.1325, 87.1325}, {100.132, 87.1325}, {100.132, 87.1325}}, thickness = 1), Text(origin = {0, 121}, extent = {{-100, 17}, {100, -17}}, textString = "T = %T, K = %k")}));
end PT1;
  end maths;

  package Interfaces
    connector Pin
      Modelica.Units.SI.ElectricPotential v "Spannungspotential";
      flow Modelica.Units.SI.Current i "Stromstaerke";
      annotation(
        Diagram,
        Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {0, 0, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}})}));
    end Pin;

    connector NPin
      Modelica.Units.SI.Voltage v "Spannungspotential";
      flow Modelica.Units.SI.Current i "Stromstaerke";
      annotation(
        Diagram,
        Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {0, 0, 255}, lineThickness = 2, extent = {{-100, 100}, {100, -100}})}));
    end NPin;

    partial model TwoPin "abstrackte Modell mit zwei Pins"
      Modelica.Units.SI.ElectricPotential v "Spannungsabfall";
      Pin p annotation(
        Placement(visible = true, transformation(origin = {-98, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-98, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      NPin n annotation(
        Placement(visible = true, transformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      0 = p.i + n.i;
      v = p.v - n.v;
    end TwoPin;

    connector RealOutput = output Real "'output Real' as connector" annotation(
      Icon(graphics = {Line(origin = {5.89, -14.81}, points = {{-63.8947, 44.807}, {64.1053, 44.807}, {-5.89468, -45.193}, {-63.8947, 44.807}, {64.1053, 44.807}}, thickness = 1)}));
    connector WahrheitswertInput = input Boolean annotation(
      defaultComponentName = "u",
      Icon(graphics = {Polygon(lineColor = {255, 0, 255}, fillColor = {255, 0, 255}, fillPattern = FillPattern.Solid, points = {{-100, 100}, {100, 0}, {-100, -100}, {-100, 100}})}, coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.2)),
      Diagram(coordinateSystem(preserveAspectRatio = true, initialScale = 0.2, extent = {{-100, -100}, {100, 100}}), graphics = {Polygon(lineColor = {255, 0, 255}, fillColor = {255, 0, 255}, fillPattern = FillPattern.Solid, points = {{0, 34}, {64, 0}, {0, -32}, {0, 34}}), Text(textColor = {255, 0, 255}, extent = {{-10, 85}, {-10, 60}}, textString = "%name")}),
      Documentation(info = "<html>
    <p>
    Connector with one input signal of type Boolean.
    </p>
    </html>"));
    connector WahrheitswertOutput = output Boolean annotation(
      defaultComponentName = "y",
      Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Polygon(points = {{-100, 100}, {100, 0}, {-100, -100}, {-100, 100}}, lineColor = {255, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid)}),
      Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Polygon(points = {{-100, 50}, {0, 0}, {-100, -50}, {-100, 50}}, lineColor = {255, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Text(extent = {{30, 110}, {30, 60}}, textColor = {255, 0, 255}, textString = "%name")}),
      Documentation(info = "<html>
    <p>
    Connector with one output signal of type Boolean.
    </p>
    </html>"));
    connector RealInput = input Real "'output Real' as connector" annotation(
      Icon(graphics = {Line(origin = {5.89, -14.81}, points = {{-63.8947, 44.807}, {64.1053, 44.807}, {-5.89468, -45.193}, {-63.8947, 44.807}, {64.1053, 44.807}}, thickness = 1), Polygon(origin = {6, -15}, fillPattern = FillPattern.Solid, points = {{-64, 45}, {-6, -45}, {64, 45}, {-64, 45}})}));
  end Interfaces;
  
  model SicherungModelicaKomonenten
    Modelica.Electrical.Analog.Basic.Ground ground annotation(
      Placement(visible = true, transformation(origin = {-6, -84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V = 100)  annotation(
      Placement(visible = true, transformation(origin = {-58, -64}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor annotation(
      Placement(visible = true, transformation(origin = {-28, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Electrical.Analog.Ideal.IdealOpeningSwitch switch annotation(
      Placement(visible = true, transformation(origin = {24, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Logical.RSFlipFlop rSFlipFlop annotation(
      Placement(visible = true, transformation(origin = {38, -4}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const(k = 10)  annotation(
      Placement(visible = true, transformation(origin = {-44, 72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression(y = false)  annotation(
      Placement(visible = true, transformation(origin = {2, 18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.VariableResistor resistor annotation(
      Placement(visible = true, transformation(origin = {64, -62}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Blocks.Math.Abs abs1 annotation(
      Placement(visible = true, transformation(origin = {-28, -14}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Sources.Trapezoid trapezoid(amplitude = -999, falling = 400, offset = 999 + 1, period = 1000, rising = 400, width = 100)  annotation(
      Placement(visible = true, transformation(origin = {118, -62}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Logical.GreaterEqual greaterEqual annotation(
      Placement(visible = true, transformation(origin = {44, 38}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Blocks.Continuous.FirstOrder firstOrder(T = 0.01, k = 1)  annotation(
      Placement(visible = true, transformation(origin = {-28, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  equation
    connect(constantVoltage.p, ground.p) annotation(
      Line(points = {{-58, -74}, {-6, -74}}, color = {0, 0, 255}));
  connect(ground.p, resistor.n) annotation(
      Line(points = {{-6, -74}, {64, -74}, {64, -72}}, color = {0, 0, 255}));
  connect(resistor.p, switch.n) annotation(
      Line(points = {{64, -52}, {64, -46}, {34, -46}}, color = {0, 0, 255}));
  connect(switch.p, currentSensor.p) annotation(
      Line(points = {{14, -46}, {-18, -46}}, color = {0, 0, 255}));
  connect(currentSensor.n, constantVoltage.n) annotation(
      Line(points = {{-38, -46}, {-58, -46}, {-58, -54}}, color = {0, 0, 255}));
  connect(currentSensor.i, abs1.u) annotation(
      Line(points = {{-28, -34}, {-28, -26}}, color = {0, 0, 127}));
  connect(abs1.y, firstOrder.u) annotation(
      Line(points = {{-28, -2}, {-28, 16}}, color = {0, 0, 127}));
  connect(greaterEqual.y, rSFlipFlop.S) annotation(
      Line(points = {{44, 27}, {44, 8}}, color = {255, 0, 255}));
  connect(rSFlipFlop.Q, switch.control) annotation(
      Line(points = {{44, -15}, {44, -34}, {24, -34}}, color = {255, 0, 255}));
  connect(trapezoid.y, resistor.R) annotation(
      Line(points = {{107, -62}, {76, -62}}, color = {0, 0, 127}));
  connect(rSFlipFlop.R, booleanExpression.y) annotation(
      Line(points = {{32, 8}, {32, 18}, {14, 18}}, color = {255, 0, 255}));
  connect(greaterEqual.u1, firstOrder.y) annotation(
      Line(points = {{44, 50}, {44, 56}, {-28, 56}, {-28, 40}}, color = {0, 0, 127}));
  connect(greaterEqual.u2, const.y) annotation(
      Line(points = {{52, 50}, {52, 72}, {-32, 72}}, color = {0, 0, 127}));
  end SicherungModelicaKomonenten;
  
  package Logical
   
model NOR "Logisches NOR Gatter: y =  Nicht (u oder u1)"
      Interfaces.WahrheitswertInput u annotation(
        Placement(visible = true, transformation(origin = {-80, 54}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-94, 56}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Interfaces.WahrheitswertInput u1 annotation(
        Placement(visible = true, transformation(origin = {-60, -18}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-94, -44}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Interfaces.WahrheitswertOutput y annotation(
        Placement(visible = true, transformation(origin = {88, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-24, -24}, {24, 24}}, rotation = 0)));
    equation
      y = not (u or u1);
      annotation(
        Icon(graphics = {Rectangle(origin = {1, -1}, lineColor = {255, 0, 255}, lineThickness = 1, extent = {{-99, 99}, {99, -99}}), Text(origin = {0, 2}, extent = {{-58, 42}, {58, -42}}, textString = "NOR")}));
    end NOR;

    model PRE "Unterbricht algebraische Schleifen um eine verschwindend kleine Zeitverzögerung (y = pre(u): Ereignisiteration wird fortgesetzt, bis u = pre(u))"
  PruefungsVorleistung.Interfaces.WahrheitswertInput u annotation(
        Placement(visible = true, transformation(origin = {-100, 26}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-96, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  PruefungsVorleistung.Interfaces.WahrheitswertOutput y annotation(
        Placement(visible = true, transformation(origin = {86, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {107, 1}, extent = {{-17, -17}, {17, 17}}, rotation = 0)));
    
    parameter Boolean pre_u_start=false "Start Wert von pre(u) zur initialisierung";
    
    
    initial equation
      pre(u) = pre_u_start;
    equation
      y = pre(u);

    annotation(
        Icon(graphics = {Rectangle(lineColor = {255, 0, 255}, lineThickness = 1, extent = {{-100, 100}, {100, -100}}), Text(origin = {3, 0}, extent = {{-65, 40}, {65, -40}}, textString = "PRE")}));
end PRE;

    model RsFlipFlop "Logisches Flip-Flop"
    parameter Boolean Qini=false "Start wert von Q bei Initialisierung";
      PruefungsVorleistung.Interfaces.WahrheitswertInput S annotation(
            Placement(visible = true, transformation(origin = {-104, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      PruefungsVorleistung.Interfaces.WahrheitswertInput R annotation(
            Placement(visible = true, transformation(origin = {-104, -54}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      PruefungsVorleistung.Interfaces.WahrheitswertOutput y1 annotation(
          Placement(visible = true, transformation(origin = {106, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      PruefungsVorleistung.Interfaces.WahrheitswertOutput y2 annotation(
          Placement(visible = true, transformation(origin = {108, -54}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Logical.NOR nor1 annotation(
            Placement(visible = true, transformation(origin = {-30, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Logical.PRE pre(pre_u_start=not (Qini))  annotation(
            Placement(visible = true, transformation(origin = {14, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Logical.NOR nor2 annotation(
        Placement(visible = true, transformation(origin = {0, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(nor1.y, pre.u) annotation(
        Line(points = {{-18, 48}, {4, 48}}, color = {255, 0, 255}));
      connect(nor1.u, S) annotation(
            Line(points = {{-40, 54}, {-66, 54}, {-66, 60}, {-104, 60}}, color = {255, 0, 255}));
      connect(nor2.u1, R) annotation(
            Line(points = {{-10, -40}, {-44, -40}, {-44, -54}, {-104, -54}}, color = {255, 0, 255}));
      connect(nor2.u, pre.y) annotation(
            Line(points = {{-10, -30}, {-26, -30}, {-26, 0}, {50, 0}, {50, 48}, {24, 48}}, color = {255, 0, 255}));
      connect(nor2.y, nor1.u1) annotation(
            Line(points = {{12, -36}, {30, -36}, {30, 18}, {-60, 18}, {-60, 44}, {-40, 44}}, color = {255, 0, 255}));
      connect(pre.y, y2) annotation(
            Line(points = {{24, 48}, {64, 48}, {64, -54}, {108, -54}}, color = {255, 0, 255}));
      connect(nor2.y, y1) annotation(
        Line(points = {{12, -36}, {72, -36}, {72, 60}, {106, 60}}, color = {255, 0, 255}));
    annotation(
        Icon(graphics = {Text(origin = {-68, -52}, extent = {{-36, 32}, {36, -32}}, textString = "S"), Text(origin = {-68, 67}, extent = {{-32, 31}, {32, -31}}, textString = "R"), Text(origin = {73, 69}, extent = {{-33, 25}, {33, -25}}, textString = "Q"), Text(origin = {69, -46}, extent = {{-29, 28}, {29, -28}}, textString = "Q!"), Text(origin = {0, 128}, extent = {{-100, 26}, {100, -26}}, textString = "%name"), Rectangle(lineColor = {255, 0, 255}, lineThickness = 1, extent = {{-100, 100}, {100, -100}})}));
end RsFlipFlop;
  end Logical;
  annotation(
    uses(Modelica(version = "4.0.0")));
end PruefungsVorleistung;
