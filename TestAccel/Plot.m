set(0, 'DefaultAxesFontName', 'Times');
data=readmatrix("capture1.csv");

x=data(:,2);
y=data(:,3);
z=data(:,4);


plot(y)
hold on
text(90,0.9925,'standard afvigelse = 20.7 \mug','FontName','Times')
ylabel("Acceleration [g]")
xlabel("Sample nummer  [-]")
legend("Acceleration y")
exportgraphics(gca,"AccTest.pdf")

var(y)



hold off
