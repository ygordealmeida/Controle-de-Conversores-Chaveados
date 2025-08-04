Kp = 0.1665;
Ti = 5.045725e-4;

num = Kp * [Ti 1];   % Numerador: Kp*(Ti*s + 1)
den = [Ti 0];         % Denominador: Ti*s

G_s = tf(num, den)

Ts = 1e-5;  % 10 us

G_z = c2d(G_s, Ts, 'tustin')

[num_d, den_d] = tfdata(G_z, 'v')

% Coeficientes extraídos:
b0 = num_d(1)
b1 = num_d(2)
a1 = den_d(2) 

