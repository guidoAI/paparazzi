function train()

MAX_SAMPLES = 25000;
start_sample = 1;

BIAS = true;
weights = true;
if(weights)
    w = load('Weights_00000.dat');
end
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
fprintf('Abs error = %f\n', mean(abs(y-b)));
if(weights)
    Z = AA * w';
end
figure(); plot(y); hold on; plot(b);
if(weights)
    plot(Z);
end
title('Height')
legend({'height estimate', 'height sonar', 'onboard height estimate'});

figure(); plot(smooth(y, 20)); hold on; plot(b);
if(weights)
    plot(smooth(Z, 20));
end
title('Smoothed Height')
legend({'height estimate', 'height sonar', 'onboard height estimate'});

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
title('Sonar height')
legend({'estimate', 'sonar'});

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






