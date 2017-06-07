function train()

MAX_SAMPLES = 25000;
start_sample = 1;

BIAS = true;
weights = true;
if(weights)
    w = load('Weights_00000.dat');
end
% structure A:
% 1) height
% 2) gain
% 3) cov div
% 4-end) textons
A = load('Training_set_00000.dat');

% inds = find(A(:,3) ~= -0.025);
% A = A(inds, :);

A = A(start_sample:min([MAX_SAMPLES, size(A,1)]), :);
b = A(:,2);
f = A(:,4:end);
if(BIAS)
    AA = [f, ones(size(A,1),1)];
else
    AA = f;
end
x = AA \ b;
y = AA * x;
fid = fopen('Weights_MATLAB.dat', 'w');
for i = 1:length(x)-1
    fprintf(fid, '%f ', x(i));
end
fprintf(fid, '%f', x(end));
fclose(fid);
height_gain_estimate = y;
fprintf('Abs error = %f\n', mean(abs(y-b)));
if(weights)
    Z = AA * w';
end
figure(); plot(y); hold on; plot(b);
if(weights)
    plot(Z);
end
title('Height')
legend({'height gain estimate', 'height gain', 'onboard gain estimate'});

figure(); plot(smooth(y, 20)); hold on; plot(b);
if(weights)
    plot(smooth(Z, 20));
end
title('Smoothed Height')
legend({'height gain estimate', 'height gain', 'onboard gain estimate'});

figure();
bar(x, 'FaceColor', [1 0 0]); % hold on; bar(w);

figure();
plot(A(:,1)); hold on; plot(A(:,2));
legend({'Height', 'Gain'});

b = A(:,1);
f = A(:,4:end);
if(BIAS)
    AA = [f, ones(size(A,1),1)];
else
    AA = f;
end
x = AA \ b;
y = AA * x;
figure(); plot(smooth(y, 20)); hold on; plot(b);

b = A(:,1);
f = height_gain_estimate;
AA = [f, ones(size(A,1),1)];
x = AA \ b;
y = AA * x;
plot(smooth(y,20));
title('Sonar height')
legend({'estimate trained with sonar', 'sonar', 'scaled gain estimate'});

figure();
plot(A(:, 3));
title('Cov div');

entr = getEntropies(f);
p_peak = zeros(1, size(f,2));
p_peak(1) = 1;
min_entr = getEntropy(p_peak);
p_uniform = ones(1, size(f,2)) ./ size(f,2);
max_entr = getEntropy(p_uniform);
figure();
histogram(entr, 30);
title(['Entropies: min = ' num2str(min_entr) ', max = ' num2str(max_entr)]);

function entropies = getEntropies(f)
n_el = size(f,1);
entropies = zeros(n_el,1);
for el = 1:n_el
   entropies(el) = getEntropy(f(el, :));
end






