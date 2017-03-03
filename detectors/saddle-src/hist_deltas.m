filepath = '../outputs/deltas.txt';

A = importdata(filepath, ' ', 0);
minDelta = min(A(2:end));
maxDelta = max(A(2:end));

x = minDelta:maxDelta;
H = histc(A(2:end),x);

figure, bar(x,H);
grid on;
xlabel('Delta');
ylabel('Frecuency');
title('');
