%hw2_17
[x,t] = ode45(@(t,x) 3*cos(x)+sin(t),[0 10],2);
disp([x,t])