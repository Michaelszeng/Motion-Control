clear
Kc=6.84;
Pc=75/120;
Kp = 0.6*Kc
Ki = 1.2*Kc/Pc
Kd = 3*Kc*Pc/40;

s = tf('s');
C = Kp + Ki/s + Kd*s
C1 = pid(Kp,Ki,Kd)
tf(C1)
return;
x = 0:0.001:1;
y0 = 2 * x;
y1 = 2 * sqrt(x .* (1 -x));
y = 2*x + 2 * sqrt(x .* (1 -x));
plot(x, y, x, y0, x, y1);
legend('2*x + 2 * sqrt(x .* (1 -x))', '2*x', '2 * sqrt(x .* (1 -x))');
grid on;
max(y)
dy = y(2:end) - y(1:end-1);
dx = x(2:end) - x(1:end-1);
v = dy./dx;
figure
plot(v);
grid on;