PROGRAM TrafficLight
  VAR
    ManualON AT %IX100.0 : BOOL;
    ManualOFF AT %IX100.1 : BOOL;
    IR1ON AT %IX100.5 : BOOL;
    IR1OFF AT %IX100.2 : BOOL;
    IR2ON AT %IX100.3 : BOOL;
    IR2OFF AT %IX100.4 : BOOL;
    Lamp AT %QX100.0 : BOOL;
  END_VAR
  VAR
    LAMP1 : BOOL;
    LAMP2 : BOOL;
    LAMP3 : BOOL;
    LAMP4 : BOOL;
  END_VAR
  VAR
    AmberBlink1 AT %QX100.1 : BOOL;
    AmberBlink2 AT %QX100.2 : BOOL;
    RedBlink1 AT %QX100.4 : BOOL;
    RedBlink2 AT %QX100.5 : BOOL;
    RedBlink3 AT %QX100.6 : BOOL;
    RedBlink4 AT %QX100.7 : BOOL;
    HMIon AT %QX100.8 : BOOL;
    HMIoff AT %QX100.9 : BOOL;
  END_VAR
  VAR
    TON0 : TON;
    TOF0 : TOF;
    CTU0 : CTU;
    TOF1 : TOF;
    TON1 : TON;
    TOF3 : TOF;
    TON2 : TON;
    R_TRIG1 : R_TRIG;
  END_VAR

  LAMP1 := ManualOFF AND IR1OFF AND (LAMP1 OR NOT(IR1ON));
  LAMP2 := ManualOFF AND IR2OFF AND (LAMP2 OR NOT(IR2ON));
  LAMP3 := ManualOFF AND (LAMP3 OR NOT(ManualON));
  LAMP4 := NOT(HMIoff) AND (HMIon OR LAMP4);
  Lamp := LAMP1 OR LAMP2 OR LAMP3 OR LAMP4;
  TOF1(IN := HMIon OR NOT(IR2ON) OR NOT(IR1ON) OR NOT(ManualON), PT := T#20000ms);
  TON0(IN := (NOT(AmberBlink1) OR NOT(AmberBlink2)) AND TOF1.Q, PT := T#500ms);
  TOF0(IN := TON0.Q, PT := T#500ms);
  AmberBlink1 := TOF0.Q;
  AmberBlink2 := TOF0.Q;
  R_TRIG1(CLK := NOT(AmberBlink2) AND TOF1.Q);
  TON2(IN := NOT(Lamp), PT := T#20000ms);
  CTU0(CU := R_TRIG1.Q, R := TON2.Q, PV := 15);
  TON1(IN := (NOT(RedBlink2) OR NOT(RedBlink1)) AND CTU0.Q, PT := T#500ms);
  TOF3(IN := TON1.Q, PT := T#500ms);
  RedBlink1 := TOF3.Q;
  RedBlink2 := TOF3.Q;
  RedBlink3 := TOF3.Q;
  RedBlink4 := TOF3.Q;
END_PROGRAM


CONFIGURATION Config0

  RESOURCE Res0 ON PLC
    TASK task0(INTERVAL := T#20ms,PRIORITY := 0);
    PROGRAM instance0 WITH task0 : TrafficLight;
  END_RESOURCE
END_CONFIGURATION
