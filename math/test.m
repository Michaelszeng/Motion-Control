clear
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