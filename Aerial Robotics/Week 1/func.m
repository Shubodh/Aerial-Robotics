function xdot=func(t,x)
xdot= [x(2),7000 - 7*x(2) - 10*x(1)];

xdot=xdot';
