clear;
lastX = 0;
lastY = 0;
lastHeading = 0;
TRACK_WIDTH = 14.2;
dRight = 0.22931333237881701;
dLeft = 0.3603495223095695;
phi = (dRight - dLeft) / TRACK_WIDTH
rCenter = ((dRight + dLeft) / 2) / phi
Px = lastX - rCenter * sin(lastHeading);
currentX = Px + rCenter * cos(phi + lastHeading - 90)
Py = lastY + rCenter * cos(lastHeading);
currentY = Py + rCenter * sin(phi + lastHeading - 90)
currentHeading = (lastHeading + phi)*180/pi