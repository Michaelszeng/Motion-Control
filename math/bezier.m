
function [X,Y]=bezier(x,y)  %定义这个子函数，可以被main调用，这里我们直接坐标赋值
x=[5,8, 10,15,20,25, 30]
y=[0 5 8 10 9 1 0]

n=length(x);  %获取输入的坐标点的个数，例如：测试的X坐标[5,10,15,20,25]为作业要求4次曲线
t=linspace(0,1);  % 改变式中的t值，则点就会在空间移动
xx=0;  %初始化X
yy=0;  %初始化Y
for k=0:n-1  %伯恩斯坦的一般形式，我们输入了5个坐标，因此是四次贝塞尔曲线
    tmp=nchoosek(n-1,k)*t.^k.*(1-t).^(n-1-k);  % nchoosek 来实现二项式系数或所有组合语法: C = nchoosek(n,k)
    xx=xx+tmp*x(k+1);  % X轴坐标
    yy=yy+tmp*y(k+1);  %  Y轴坐标
end
if nargout==2   %输出参数的个是为2个时
    X=xx;Y=yy;
end
 h=plot(xx,yy);  %画四次贝塞尔曲线
 hold on      %保留本曲线
 m=plot(x,y);   %画原始点
 title('Bezier curve');
 
figure
xq = 0:0.1:30;
vq1 = interp1(x,y,xq, 'spline')
plot(x,y,'o',xq,vq1,'r:.');
xlim([0 30]);
title('Spline Interpolation');