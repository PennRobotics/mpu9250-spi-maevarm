clear all;
SERIAL_PORT = '/dev/tty.usbmodem411';
%SERIAL_PORT = '/dev/ttyACM0';
BAUD_RATE = 115200;

%% Data Collection

h = serial(SERIAL_PORT, 'BaudRate', BAUD_RATE);
set(h, 'InputBufferSize', 2048);
fopen(h);

fprintf(h, 'G');
tic;

values = 20;
points = 16000;

%t = zeros(1, points);
%out = zeros(values, points);
%for i = 1 : points * values
%    t(i) = fread(h, 1, 'uint32');
%    get(h, 'BytesAvailable')
%    out(:, i) = fread(h, values, 'int16');

i = 1;
while i < points * values
    ints = floor(get(h, 'BytesAvailable') / 2);
    if ints > 0
        stream(i : i + ints - 1) = fread(h, ints, 'int16');
        i = i + ints;
    end
end
toc
fprintf(h, 'S');
out = reshape(stream(1 : size(stream, 2) - mod(size(stream, 2), values)), values, floor(size(stream, 2) / values));
t = out(1, :) + (out(1, :) < 0) * 65536 + 65536 * (out(2, :) + (out(2, :) < 0) * 65536);
data = out(3 : values, :);
fclose(h);

%% Visualization

t_scale = 1 / 8000;

a_fullscale = 16;  % +/-2g
a_scale = (a_fullscale * 2) / 65535;
a_max = 8;

g_fullscale = 250;  % +/-250
m_scale = 0.6;  % 0.6µT/bit
m_max = 500;

t = (t - t(1)) * t_scale;
g_scale = (g_fullscale * 2) / 65535;
g_max = 100;

figure(1)
subplot(9, 2,  1); plot(t, a_scale * data( 1, :)', 'r'); axis([min(t) max(t) -a_max a_max]); ylabel('Ax'); title('Sensor A');
subplot(9, 2,  3); plot(t, a_scale * data( 2, :)', 'g'); axis([min(t) max(t) -a_max a_max]); ylabel('Ay');
subplot(9, 2,  5); plot(t, a_scale * data( 3, :)', 'b'); axis([min(t) max(t) -a_max a_max]); ylabel('Az');

subplot(9, 2,  7); plot(t, g_scale * data( 4, :)', 'r'); axis([min(t) max(t) -g_max g_max]); ylabel('Gx');
subplot(9, 2,  9); plot(t, g_scale * data( 5, :)', 'g'); axis([min(t) max(t) -g_max g_max]); ylabel('Gy');
subplot(9, 2, 11); plot(t, g_scale * data( 6, :)', 'b'); axis([min(t) max(t) -g_max g_max]); ylabel('Gz');

subplot(9, 2, 13); plot(t, m_scale * data( 7, :)', 'r'); axis([min(t) max(t) -m_max m_max]); ylabel('Mx');
subplot(9, 2, 15); plot(t, m_scale * data( 8, :)', 'g'); axis([min(t) max(t) -m_max m_max]); ylabel('My');
subplot(9, 2, 17); plot(t, m_scale * data( 9, :)', 'b'); axis([min(t) max(t) -m_max m_max]); ylabel('Mz');

subplot(9, 2,  2); plot(t, a_scale * data(10, :)', 'r'); axis([min(t) max(t) -a_max a_max]); ylabel('Ax'); title('Sensor B');
subplot(9, 2,  4); plot(t, a_scale * data(11, :)', 'g'); axis([min(t) max(t) -a_max a_max]); ylabel('Ay');
subplot(9, 2,  6); plot(t, a_scale * data(12, :)', 'b'); axis([min(t) max(t) -a_max a_max]); ylabel('Az');

subplot(9, 2,  8); plot(t, g_scale * data(13, :)', 'r'); axis([min(t) max(t) -g_max g_max]); ylabel('Gx');
subplot(9, 2, 10); plot(t, g_scale * data(14, :)', 'g'); axis([min(t) max(t) -g_max g_max]); ylabel('Gy');
subplot(9, 2, 12); plot(t, g_scale * data(15, :)', 'b'); axis([min(t) max(t) -g_max g_max]); ylabel('Gz');

subplot(9, 2, 14); plot(t, m_scale * data(16, :)', 'r'); axis([min(t) max(t) -m_max m_max]); ylabel('Mx');
subplot(9, 2, 16); plot(t, m_scale * data(17, :)', 'g'); axis([min(t) max(t) -m_max m_max]); ylabel('My');
subplot(9, 2, 18); plot(t, m_scale * data(18, :)', 'b'); axis([min(t) max(t) -m_max m_max]); ylabel('Mz');


%%
