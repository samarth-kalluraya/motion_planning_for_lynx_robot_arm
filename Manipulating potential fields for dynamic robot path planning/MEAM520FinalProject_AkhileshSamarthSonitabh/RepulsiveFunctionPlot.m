% We use these lines of code to plot the two repulsive force functions that
% we have defined in our report. This is to see the difference of forces at
% different distances from the obstacle.

x = linspace(0,100,101);

F_rep1 = 1000*((1./x) - (1/50));

F_rep2 = 1000*((1./x) - (1/50)).*(1./(x.^2));

plot(x,F_rep1);
hold on
plot(x,F_rep2);