scatter3(Fx1,Fz1,M_max);
hold on
scatter3(Fx1,Fz1,M_min);
xlabel('Fx(N)');
ylabel('Fz(N)');
zlabel('M(Nm)');
title('Cone Generate through Visual Studio');
sgcurrent=counterclock.*current;
test2=sgcurrent*2.69/1000
test2=test2*1.0709;
scatter3(Fx,Fz,test2);
scatter3(Fx,Fz,counterclock.*newMy);

