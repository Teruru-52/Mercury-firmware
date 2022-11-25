%% length = 0.144
clear
close all
ts = 0.001;

v_max = 0.3;
a = v_max / 0.3;

t = 0:ts:0.78;

for i = 1:300
    v_ref(i) = a * t(i);
end

for i = 301:480
    v_ref(i) = v_max;
end

for i = 481:780
    v_ref(i) = v_max - a * (t(i) - 0.48);
end
v_ref(781) = 0;

v_ref = v_ref';
plot(t, v_ref, 'LineWidth',3);

length = 0;
for i = 1:781
    length = length + v_ref(i) * ts;
end

dlmwrite('ref1.csv', v_ref, 'precision', '%.3f');

%% length = 0.18
clear
close all
ts = 0.001;

v_max = 0.3;
a = v_max / 0.3;

t = 0:ts:0.9;

for i = 1:300
    v_ref(i) = a * t(i);
end

for i = 301:600
    v_ref(i) = v_max;
end

for i = 601:900
    v_ref(i) = v_max - a * (t(i) - 0.6);
end
v_ref(901) = 0;

v_ref = v_ref';
plot(t, v_ref, 'LineWidth',3);

length = 0;
for i = 1:901
    length = length + v_ref(i) * ts;
end

dlmwrite('ref2.csv', v_ref, 'precision', '%.3f');

%% length = 0.9
clear
close all
ts = 0.001;

v_max = 0.2;
a = v_max / 0.2;

t = 0:ts:0.65;

for i = 1:200
    v_ref(i) = a * t(i);
end

for i = 201:450
    v_ref(i) = v_max;
end

for i = 451:650
    v_ref(i) = v_max - a * (t(i) - 0.45);
end
v_ref(651) = 0;

v_ref = v_ref';
plot(t, v_ref, 'LineWidth',3);

length = 0;
for i = 1:651
    length = length + v_ref(i) * ts;
end

dlmwrite('ref3.csv', v_ref, 'precision', '%.3f');