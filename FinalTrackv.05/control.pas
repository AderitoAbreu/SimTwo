//Final_Track_V5 project to Dissertation presented to the School of Technology and Management of Bragança to obtain the Master Degree in Industrial Engineering
//AdéritoAbreu
//30968
//31/10/2020

var
  t: double;

  CameraLine0: array [0..319] of TRGBAColor;         //RGB Resolution.
  CameraLine1: array [0..319] of TRGBAColor;         //RGB Resolution.
  CameraLine2: array [0..319] of TRGBAColor;         //RGB Resolution.

  CameraLineStop: array [180 ..190] of TRGBAColor;

  RGBline0: array [0 ..319] of integer;
  RGBline1: array [0 ..319] of integer;
  RGBline2: array [0 ..319] of integer;

  RlineStop :array [180 ..190] of integer;
  GlineStop :array [180 ..190] of integer;
  BlineStop :array [180 ..190] of integer;

  robposx, robposY, robtheta, StopYPoint, StopXPoint, StopThetaPoint, def1, def2: double;

  //Tpoint, Tpoint1, Tpoint2 : integer;
  Tpoint3, Tpoint4, Tpoint5 : double;                //Just 3 thresholds Points.
  y: integer;
  j, i: integer;
  w: double;
  v, k3, k4, k5: double;

  LineDetected3, LineDetected4, LineDetected5 : double;

  firstRay, lastRay: integer;
  iLaser: integer;

  LaserValues: Matrix;                               //Lidar Input.

  Distance: double;
  GasValue: double;
  frac1, frac2: double;

  vref, steer: double;

  erro3, erro4, erro5: double;
  d: double;

  go: boolean;
  StopPoint: boolean;
  StopLight: boolean;

  state: integer;

procedure TurnObstacleLeft;                        //State 2
begin

SetRCValue(2, 5 ,'TurnObstacleLeft');
SetRCValue(18, 5 ,'Warning!');
SetRCValue(20, 5 ,'');

  SetAxisSpeedRef(0, 0, 3.5);
  SetAxisSpeedRef(0, 1, -3.5);
  SetAxisPosRef(0, 3, -0.6);
  SetAxisPosRef(0, 5, -0.6);

for i := 0 to 9 do begin

  SetRCValue(15, i+1, format('%.3g',[mgetv(LaserValues, i, 0)]));
  if ((mgetv(LaserValues, 4, 0) > 1.1 ) and (state = 2)) or ((mgetv(LaserValues, 4, 0) < 0 ) and (state = 2)) then begin
  state:= 3;

end;
end;
end;

procedure FollowLeft;                            //State 3
begin
SetRCValue(2, 5 ,'FollowLeft');
SetRCValue(18, 5 ,'Warning!');
SetRCValue(20, 5 ,'');

for i:= 1 to 319 do begin
  CameraLine0[i-1] := GetCameraPixel(i, 90); //Runs 1 positional line at y=90;
  RGBline0[i-1] := (CameraLine0[i-1].red+CameraLine0[i-1].green+CameraLine0[i-1].blue)/3;

    if RGBline0[i-1] >= 120 then begin // Scale of 15(track) until 255(line/boundary), 240/2=120.
      Tpoint3 := i;
      SetRCValue(9, 5 ,format('%.3g',[Tpoint3]));
      SetRCValue(11, 3 ,'LD');
      Break;
    end
    else begin
      SetRCValue(11, 3 ,'LND');
      Tpoint3:= 0;
    end;
end;

for i:= 1 to 319 do  begin
  CameraLine1[i-1] := GetCameraPixel(i, 90); //Runs 2 positional line at y=90;
  RGBline1[i-1] := (CameraLine1[i-1].red+CameraLine1[i-1].green+CameraLine1[i-1].blue)/3;

    if RGBline1[i-1] >= 120 then begin// Scale of 15(track) until 255(line/boundary), 240/2=120.
      Tpoint4 := i;
      SetRCValue(10, 5 ,format('%.3g',[Tpoint4]));
      SetRCValue(10, 3 ,'LD');
      Break;
    end
    else begin
      SetRCValue(10, 3 ,'LND');
      Tpoint4:= 0;
    end;
end;

for i:= 1 to 319 do begin
  CameraLine2[i-1] := GetCameraPixel(i, 90); //Runs 3 positional line at y=90;
  RGBline2[i-1] := (CameraLine2[i-1].red+CameraLine2[i-1].green+CameraLine2[i-1].blue)/3;

    if RGBline2[i-1] >= 120 then begin  // Scale of 15(track) until 255(line/boundary), 240/2=120.
      Tpoint5 := i;
      SetRCValue(11, 5 ,format('%.3g',[Tpoint5]));
      SetRCValue(9, 3 ,'LD');
      Break;
    end
    else begin
      SetRCValue(9, 3 ,'LND')
      Tpoint5:= 0;
    end;
end;
erro3:=  (Tpoint3 - 97);                      //thresholds
erro4:= (Tpoint4 - 122);
erro5:= (Tpoint5 - 141);

if (tpoint3 <> 0) or (tpoint4 <> 0) or (tpoint5 <> 0) then
  w:= 0;

if (tpoint3 <> 0)  then
  w:= erro3*k3;

if (tpoint4 <> 0) then
  w:= w + erro4*k4;

if (tpoint5 <> 0) then
  w:= w + erro5*k5;                          //error final calculation

if w > 35 then                               //sterring error saturation
  w := 35;

if w < -35 then
  w := -35;

steer:= rad(w);
SetRCValue(2, 3 ,format('%.3g',[w]));
SetRCValue(2, 2 ,format('%.3g',[steer]));

vref := -(abs(w)/35) + 3.5 ;  //Line Equation for vref(error)
SetRCValue(3, 2 ,format('%.3g',[vref]));

if RCButtonPressed(7, 7) then
  Go:= true;
if RCButtonPressed(7, 8) then
  Go:= false;

If (Go = true) then begin                      //Output values
  SetAxisSpeedRef(0, 0, vref);
  SetAxisSpeedRef(0, 1, -vref);
  SetAxisPosRef(0, 3, steer);
  SetAxisPosRef(0, 5, steer);
end;

for i := 0 to 9 do begin

  SetRCValue(15, i+1, format('%.3g',[mgetv(LaserValues, i, 0)]));
  if ((mgetv(LaserValues, 0, 0) > 1.1 ) and (state = 3)) or ((mgetv(LaserValues, 0, 0) < 0 ) and (state = 3)) then begin
  state:= 1;

end;
end;

end;

procedure TurnObstacleRight;                       //state 4
begin

SetRCValue(2, 5 ,'TurnObstacleRight');
SetRCValue(18, 5 ,'Warning!');
SetRCValue(20, 5 ,'');
SetAxisSpeedRef(0, 0, 3.5);
SetAxisSpeedRef(0, 1, -3.5);
SetAxisPosRef(0, 3, 0.4);
SetAxisPosRef(0, 5, 0.4);

for i := 0 to 9 do begin

  SetRCValue(15, i+1, format('%.3g',[mgetv(LaserValues, i, 0)]));
  if ((mgetv(LaserValues, 0, 0) > 0 ) and (state = 4))  then begin
  state:= 1;

end;
end;

end;

procedure PreFollowObstacleRight;               //state 5
begin

SetRCValue(2, 5 ,'PreFollowObstacleRight');
SetRCValue(18, 5 ,'');
SetRCValue(20, 5 ,'G0');

SetAxisSpeedRef(0, 0, 3.5);
SetAxisSpeedRef(0, 1, -3.5);
SetAxisPosRef(0, 3, -0.6);
SetAxisPosRef(0, 5, -0.6);

if (steer = 0.11) and (state = 5) then begin
state:= 1

end;
end;


procedure FollowRight;                            //state 1
begin
SetRCValue(2,5 ,'FollowRight');
SetRCValue(18, 5 ,'');
SetRCValue(20, 5 ,'G0');
for i:= 319 downto 50 do  begin
  CameraLine0[i-1] := GetCameraPixel(i, 60); //Runs 1 positional line at y=60;
  RGBline0[i-1] := (CameraLine0[i-1].red+CameraLine0[i-1].green+CameraLine0[i-1].blue)/3;

    if RGBline0[i-1] >= 120 then begin // Scale of 15(track) until 255(line/boundary), 240/2=120.
      Tpoint3 := i;
      SetRCValue(9, 5 ,format('%.3g',[Tpoint3]));
      SetRCValue(11, 3 ,'LD');
      Break;
    end
    else begin
      SetRCValue(11, 3 ,'LND');
      Tpoint3:= 0;
    end;
end;

for i:= 319 downto 50 do  begin
  CameraLine1[i-1] := GetCameraPixel(i, 96); //Runs 2 positional line at y=96;
  RGBline1[i-1] := (CameraLine1[i-1].red+CameraLine1[i-1].green+CameraLine1[i-1].blue)/3;

    if RGBline1[i-1] >= 120 then begin// Scale of 15(track) until 255(line/boundary), 240/2=120.
      Tpoint4 := i;
      SetRCValue(10, 5 ,format('%.3g',[Tpoint4]));
      SetRCValue(10, 3 ,'LD');
      Break;
    end
    else begin
      SetRCValue(10, 3 ,'LND');
      Tpoint4:= 0;
    end;
end;

for i:= 319 downto 50 do  begin
  CameraLine2[i-1] := GetCameraPixel(i, 144); ////Runs 3 positional line at y=144;
  RGBline2[i-1] := (CameraLine2[i-1].red+CameraLine2[i-1].green+CameraLine2[i-1].blue)/3;

    if RGBline2[i-1] >= 120 then begin  // Scale of 15(track) until 255(line/boundary), 240/2=120.
      Tpoint5 := i;
      SetRCValue(11, 5 ,format('%.3g',[Tpoint5]));
      SetRCValue(9, 3 ,'LD');
      Break;
    end
    else begin
      SetRCValue(9, 3 ,'LND')
      Tpoint5:= 0;
    end;
end;

erro3:=  (Tpoint3 - 169);                             // //thresholds
erro4:= (Tpoint4 - 197);
erro5:= (Tpoint5 - 222);

if (tpoint3 <> 0) or (tpoint4 <> 0) or (tpoint5 <> 0) then
  w:= 0;

if (tpoint3 <> 0)  then                               //error final calculation
  w:= erro3*k3;

if (tpoint4 <> 0) then
  w:= w + erro4*k4;

if (tpoint5 <> 0) then
  w:= w + erro5*k5;

if w > 35 then                                        //sterring error saturation
  w := 35;

if w < -35 then
  w := -35;

//steer:= rad(GetRCValue(5,6));
steer:= rad(w);
SetRCValue(2, 3 ,format('%.3g',[w]));
SetRCValue(2, 2 ,format('%.3g',[steer]));

vref := -(abs(w)/35) + v ;  //Line equation vref(error)
SetRCValue(3, 2 ,format('%.3g',[vref]));


end;

procedure RedStopLine;                         //state 6
begin
SetRCValue(2,5 ,'RedStopLine');
SetRCValue(18, 5 ,'Warning!');
SetRCValue(20, 5 ,'');

vref := 0;

StopXPoint:=robposx;
StopYPoint:=robposy;
StopThetaPoint:= robtheta;

SetRCValue(12, 8 ,format('%.3g',[StopXPoint]));
SetRCValue(13, 8 ,format('%.3g',[StopYPoint]));
SetRCValue(14, 8 ,format('%.3g',[StopThetaPoint]));

if RCButtonPressed(3, 6) then
state:= 1;
if RCButtonPressed(3, 7) then
state:= 8;
if RCButtonPressed(3, 8) then
state:= 7;
if RCButtonPressed(3, 9) then
state:= 9;

end;

procedure GoStraight;                            //state 7
begin
SetRCValue(2,5 ,'GoStraight');
robposx := GetRobotX(0);
robposy := GetRobotY(0);
robtheta := 57.2958*(GetRobotTheta(0));
SetRCValue(9, 8 ,format('%.3g',[robposx]));
SetRCValue(10, 8 ,format('%.3g',[robposy]));
SetRCValue(11, 8 ,format('%.3g',[robtheta]));


def1:= abs(robposx - StopXPoint);
def2:= abs(robposy - StopYPoint);

SetRCValue(12, 10 ,format('%.3g',[def1]));
SetRCValue(13, 10 ,format('%.3g',[def2]));


If ((Go = true) and (def1 <= 1.5)) and (stoplight = true) then begin

  SetAxisSpeedRef(0, 0, 5);
  SetAxisSpeedRef(0, 1, -5);
  SetAxisPosRef(0, 3, 0);
  SetAxisPosRef(0, 5, 0);
end
else begin
state:=1;
end;
If ((Go = true) and (def2 <= 1.5)) and (stoplight = true)then begin

  SetAxisSpeedRef(0, 0, 5);
  SetAxisSpeedRef(0, 1, -5);
  SetAxisPosRef(0, 3, 0);
  SetAxisPosRef(0, 5, 0);
end
else begin
state:=1;
end;
end;

procedure TurnLeft;                                   //state 8
begin
SetRCValue(2,5 ,'TurnLeft');

robposx := GetRobotX(0);
robposy := GetRobotY(0);
robtheta := 57.2958*(GetRobotTheta(0));
SetRCValue(9, 8 ,format('%.3g',[robposx]));
SetRCValue(10, 8 ,format('%.3g',[robposy]));
SetRCValue(11, 8 ,format('%.3g',[robtheta]));


def1:= abs(robposx - StopXPoint);
def2:= abs(robposy - StopYPoint);

SetRCValue(12, 10 ,format('%.3g',[def1]));
SetRCValue(13, 10 ,format('%.3g',[def2]));

If ((Go = true) and (def1 <= 1.7)) and (stoplight = true) then begin
SetAxisSpeedRef(0, 0, 5);
SetAxisSpeedRef(0, 1, -5);
SetAxisPosRef(0, 3, -0.15);
SetAxisPosRef(0, 5, -0.15);
end
else begin
state:=1;
end;

If ((Go = true) and (def2 <= 1.7)) and (stoplight = true) then begin
SetAxisSpeedRef(0, 0, 5);
SetAxisSpeedRef(0, 1, -5);
SetAxisPosRef(0, 3, -0.15);
SetAxisPosRef(0, 5, -0.15);
end
else begin
state:=1;
end;
end;

procedure TurnRight;                                 //state 9
begin
SetRCValue(2,5 ,'TurnRight');
robposx := GetRobotX(0);
robposy := GetRobotY(0);
robtheta := 57.2958*(GetRobotTheta(0));
SetRCValue(9, 8 ,format('%.3g',[robposx]));
SetRCValue(10, 8 ,format('%.3g',[robposy]));
SetRCValue(11, 8 ,format('%.3g',[robtheta]));


def1:= abs(robposx - StopXPoint);
def2:= abs(robposy - StopYPoint);

SetRCValue(12, 10 ,format('%.3g',[def1]));
SetRCValue(13, 10 ,format('%.3g',[def2]));

If ((Go = true) and (def1 <= 0.5)) and (stoplight = true)then begin
SetAxisSpeedRef(0, 0, 5);
SetAxisSpeedRef(0, 1, -5);
SetAxisPosRef(0, 3, 0.40);
SetAxisPosRef(0, 5, 0.40);
end
else begin
state:=1;
end;

If ((Go = true) and (def2 <= 0.5)) and (stoplight = true) then begin
SetAxisSpeedRef(0, 0, 5);
SetAxisSpeedRef(0, 1, -5);
SetAxisPosRef(0, 3, 0.40);
SetAxisPosRef(0, 5, 0.40);
end
else begin
state:=1;
end;
end;


procedure control;
begin
for i:=  182 to 190 do begin

  CameraLineStop[i] := GetCameraPixel(160, i); //Runs positional line at x=160 - input to stop move on red stop line;
  RlineStop[i] := (CameraLineStop[i].red);
  GlineStop[i] := (CameraLineStop[i].green);
  BlineStop[i] := (CameraLineStop[i].blue);

  SetRCValue(i-164, 1,format('%.d', [RlineStop[i]]));
  SetRCValue(i-164, 2,format('%.d', [GlineStop[i]]));
  SetRCValue(i-164, 3,format('%.d', [BlineStop[i]]));

   if ((RlineStop[i] < 60 ) and (GlineStop[i] < 60 ) and (BlineStop[i] > 220 )) and ((RlineStop[i-1] < 60 ) and (GlineStop[i-1] < 60 ) and (BlineStop[i-1] > 220 )) and ((RlineStop[i-2] < 60 ) and (GlineStop[i-2] < 60 ) and (BlineStop[i-2] > 220 )) and (state=1) then begin
      State := 6;
      SetRCValue(3, 5 ,'STOP');
      Break;
    end
    else begin
      SetRCValue(3, 5 ,'GO');
      State := State;
    end;

end;

robposx := GetRobotX(0);
robposy := GetRobotY(0);
robtheta := 57.2958*(GetRobotTheta(0));

if RCButtonPressed(18, 7) then begin                       //traffic lights color change to red
SetObstacleColor(1,255,0,0);
SetObstacleColor(2,255,0,0);
SetObstacleColor(3,255,0,0);
SetObstacleColor(4,255,0,0);
StopLight:= false;
end;

if RCButtonPressed(18, 8) then begin                       //traffic lights color change to green
SetObstacleColor(1,0,255,0);
SetObstacleColor(2,0,255,0);
SetObstacleColor(3,0,255,0);
SetObstacleColor(4,0,255,0);
StopLight:= true;
end;

SetRCValue(9, 8 ,format('%.3g',[robposx]));
SetRCValue(10, 8 ,format('%.3g',[robposy]));
SetRCValue(11, 8 ,format('%.3g',[robtheta]));


if RCButtonPressed(7, 1) then
  SetRobotPos(0, GetRCValue(7,2), GetRCValue(7,3), 0, rad(GetRCValue(7,4)));

if RCButtonPressed(8, 1) then
  SetRobotPos(0, GetRCValue(8,2), GetRCValue(8,3), 0, rad(GetRCValue(8,4)));

if RCButtonPressed(7, 10) then
  SetRobotPos(0, GetRCValue(7, 11), GetRCValue(7,12), 0, rad(GetRCValue(7,13)));

if RCButtonPressed(8, 10) then
  SetRobotPos(0, GetRCValue(8, 11), GetRCValue(8,12), 0, rad(GetRCValue(8,13)));

if RCButtonPressed(9, 10) then
  SetRobotPos(0, GetRCValue(9, 11), GetRCValue(9,12), 0, rad(GetRCValue(9,13)));

if RCButtonPressed(9, 1) then
  k3:= GetRCValue(9,2);

if RCButtonPressed(10, 1) then
  k4:= GetRCValue(10,2);

if RCButtonPressed(11, 1) then
  k5:= GetRCValue(11,2);

if RCButtonPressed(4, 1) then
  v:= GetRcValue(4,2);

if RCButtonPressed(7, 7) then
  Go:= true;
if RCButtonPressed(7, 8) then
  Go:= false;

If (Go = true) then begin
  SetAxisSpeedRef(0, 0, vref);
  SetAxisSpeedRef(0, 1, -vref);
  SetAxisPosRef(0, 3, steer);
  SetAxisPosRef(0, 5, steer);
end;

LaserValues := GetSensorValues(0, iLaser);                    //Lidar get value input

for i := 3 to 5 do begin
  SetRCValue(15, i, format('%.3g',[mgetv(LaserValues, i, 0)]));
  if (mgetv(LaserValues, i, 0) > 0) and (mgetv(LaserValues, i, 0)<= 1.1) and (state = 1) then begin
  state:= 2;
end;
end;
SetRCValue(2, 6 ,format('%.d',[state]));
if (state = 3) then begin
FollowLeft();
end else if (state = 1) then begin
FollowRight();
end else if (state = 2) then begin
TurnObstacleLeft();
end else if (state = 4) then begin
TurnObstacleRight();
end else if (state = 5) then begin
PreFollowObstacleRight();
end else if (state = 6) then begin
RedStopLine();
end else if (state = 7) then begin
GoStraight();
end else if (state = 8) then begin
TurnLeft();
end else if (state = 9) then begin
TurnRight();
end;

end;

procedure Initialize;

begin
  SetObstacleColor(1,0,255,0);
  SetObstacleColor(2,0,255,0);
  SetObstacleColor(3,0,255,0);
  SetObstacleColor(4,0,255,0);
  state:= 1;
  iLaser := GetSensorIndex(0, 'ranger2d');
  StopLight:= False;
  w:= 0;
  v:=5;

  k3:= 0.10;               //initial value of line3 force
  k4:= 0.12;               //initial value of line2 force
  k5:= 0.15;               //initial value of line1 force
  Go:= false;
  y:= 1;
  t:= 0;

end;
