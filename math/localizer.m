testStrafe;
testStraight;

function testStrafe()
    lastX = 3;
    lastY = 2;
    lastHeading = 0.2;
    phi=0:0.5:6.28;
    rs = 14.2;
    rb = 5.5;
    B = (rs+rb) * phi;
    Px = lastX - rs * cos(lastHeading - pi);
    Py = lastY - rs * sin(lastHeading - pi);
    currentX = Px + rs * cos(phi + lastHeading - pi);
    currentY = Py + rs * sin(phi + lastHeading - pi);

    deltaX = rs * sin(phi);
    deltaY = rs * (1 - cos(phi));
    currentX2 = lastX + deltaX * cos(lastHeading - pi/2) - deltaY * sin(lastHeading - pi/2);
    currentY2 = lastY + deltaY * cos(lastHeading - pi/2) + deltaX * sin(lastHeading - pi/2);
    figure;
    plot(currentX, currentY, 'r*', currentX2, currentY2, 'g');
    title('testStrafe');    
end

function testStraight()
    lastX = 3;
    lastY = 2;
    lastHeading = 0.2;
    TRACK_WIDTH = 14.2;
    dLeft = 0:8:32;
    dRight = 0:16:64;
    disp('math from MIT course');
    phi = (dRight - dLeft) / TRACK_WIDTH;
    rCenter = ((dRight + dLeft) / 2) / phi;
    Px = lastX - rCenter * sin(lastHeading);
    Py = lastY + rCenter * cos(lastHeading);
    currentX = Px + rCenter * cos(phi + lastHeading - pi/2);
    currentY = Py + rCenter * sin(phi + lastHeading - pi/2);
    currentHeading = (lastHeading + phi)*180/pi;

    disp('math for 3 wheel odometry');
    r = TRACK_WIDTH/2;
    r_t = r*(dRight+dLeft)/(dRight-dLeft)
    tx = r_t * (cos(phi) - 1);
    ty = r_t * sin(phi);
    currentX2 = lastX + tx * cos(lastHeading - pi/2) - ty * sin(lastHeading - pi/2);
    currentY2 = lastY + ty * cos(lastHeading - pi/2) + tx * sin(lastHeading - pi/2);
    currentHeading = (lastHeading + phi)*180/pi
    plot(currentX, currentY, '+r', currentX2, currentY2, '*b', lastX, currentX2-currentX, 'g',  lastX, currentY2-currentY, 'y' );
    legend('FTC', 'MIT', 'delta X', 'delta Y');
    grid on;

    figure;
    phi=0:0.01:6.28
    Px = lastX - rCenter * sin(lastHeading);
    Py = lastY + rCenter * cos(lastHeading);
    currentX = Px + rCenter * cos(phi + lastHeading - pi/2);
    currentY = Py + rCenter * sin(phi + lastHeading - pi/2);

    tx = r_t * (cos(phi) - 1);
    ty = r_t * sin(phi);
    currentX2 = lastX + tx * cos(lastHeading - pi/2) - ty * sin(lastHeading - pi/2);
    currentY2 = lastY + ty * cos(lastHeading - pi/2) + tx * sin(lastHeading - pi/2);
    plot(currentX, currentY, '.r', currentX2, currentY2, '*b', phi, currentX2-currentX, 'g',  phi, currentY2-currentY, 'y' );
    legend('FTC', 'MIT', 'delta X', 'delta Y');
    grid on;
    title('testStraight');
end