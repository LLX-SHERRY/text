dJoint = rad2deg(Joint);

figure(10)
grid on
hold on
plot(dJoint(:,3))
plot(dJoint(:,9))
legend('leg3')

figure(11)
grid on
hold on
plot(dJoint(:,4))
plot(dJoint(:,10))
legend('leg4')

figure(12)
grid on
hold on
plot(dJoint(:,5))
plot(dJoint(:,11))
legend('leg5')

figure(13)
grid on
hold on
plot(dJoint(:,18))
plot(dJoint(:,21))
legend('arm3')

figure(15)
grid on
hold on
plot(dJoint(:,17))
plot(dJoint(:,20))
legend('arm2')

figure(14)
grid on
hold on
plot(dJoint(:,14))
legend('waist pitch')