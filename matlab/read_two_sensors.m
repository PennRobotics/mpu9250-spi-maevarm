clear;

%% Serial object configuration

SERIAL_PORT = '/dev/tty.usbmodem4101';
%SERIAL_PORT = '/dev/ttyACM0';
BAUD_RATE = 115200;
h = serial(SERIAL_PORT, 'BaudRate', BAUD_RATE);
set(h, 'InputBufferSize', 2048);
fopen(h);

%% Settings

sample_rate = 8000;
points = 16000;
a_fullscale = 16;  % options: 2, 4, 8, 16
g_fullscale = 2000;  % options: 250, 500, 1000, 2000

%% send the accelerometer full-scale setting to the microcontroller

switch (a_fullscale)
    case 2
        fprintf(h, '1');
    case 4
        fprintf(h, '2');
    case 8
        fprintf(h, '3');
    case 16
        fprintf(h, '4');
end

switch (g_fullscale)
    case 250
        fprintf(h, 'a');
    case 500
        fprintf(h, 'b');
    case 1000
        fprintf(h, 'c');
    case 2000
        fprintf(h, 'd');
end

%% start recording

fprintf(h, 'G');
tic;

%% record

values = 20;
i = 1;
while i < points * values
    ints = floor(get(h, 'BytesAvailable') / 2);
    if ints > 0
        stream(i : i + ints - 1) = fread(h, ints, 'int16');
        i = i + ints;
    end
end
toc

%% stop recording

fprintf(h, 'S');

%% reshape the data

out = reshape(stream(1 : size(stream, 2) - mod(size(stream, 2), values)), values, floor(size(stream, 2) / values));
t = out(1, :) + (out(1, :) < 0) * 65536 + 65536 * (out(2, :) + (out(2, :) < 0) * 65536);
data = out(3 : values, :);
fclose(h);

%% plot the data

a_scale = (a_fullscale * 2) / 65535;
a_max = a_fullscale;

m_scale = 0.6;  % 0.6µT/bit
m_max = 500;

t_scale = 1 / sample_rate;
t = (t - t(1)) * t_scale;

g_scale = (g_fullscale * 2) / 65535;
g_max = g_fullscale;

figure(1)
subplot(9, 2,  1); plot(t, a_scale * data( 1, :)', 'r'); axis([min(t), max(t), -a_max, a_max]); ylabel('Ax'); title('Sensor A');
subplot(9, 2,  3); plot(t, a_scale * data( 2, :)', 'g'); axis([min(t), max(t), -a_max, a_max]); ylabel('Ay');
subplot(9, 2,  5); plot(t, a_scale * data( 3, :)', 'b'); axis([min(t), max(t), -a_max, a_max]); ylabel('Az');

subplot(9, 2,  7); plot(t, g_scale * data( 4, :)', 'r'); axis([min(t), max(t), -g_max, g_max]); ylabel('Gx');
subplot(9, 2,  9); plot(t, g_scale * data( 5, :)', 'g'); axis([min(t), max(t), -g_max, g_max]); ylabel('Gy');
subplot(9, 2, 11); plot(t, g_scale * data( 6, :)', 'b'); axis([min(t), max(t), -g_max, g_max]); ylabel('Gz');

subplot(9, 2, 13); plot(t, m_scale * data( 7, :)', 'r'); axis([min(t), max(t), -m_max, m_max]); ylabel('Mx');
subplot(9, 2, 15); plot(t, m_scale * data( 8, :)', 'g'); axis([min(t), max(t), -m_max, m_max]); ylabel('My');
subplot(9, 2, 17); plot(t, m_scale * data( 9, :)', 'b'); axis([min(t), max(t), -m_max, m_max]); ylabel('Mz');

subplot(9, 2,  2); plot(t, a_scale * data(10, :)', 'r'); axis([min(t), max(t), -a_max, a_max]); ylabel('Ax'); title('Sensor B');
subplot(9, 2,  4); plot(t, a_scale * data(11, :)', 'g'); axis([min(t), max(t), -a_max, a_max]); ylabel('Ay');
subplot(9, 2,  6); plot(t, a_scale * data(12, :)', 'b'); axis([min(t), max(t), -a_max, a_max]); ylabel('Az');

subplot(9, 2,  8); plot(t, g_scale * data(13, :)', 'r'); axis([min(t), max(t), -g_max, g_max]); ylabel('Gx');
subplot(9, 2, 10); plot(t, g_scale * data(14, :)', 'g'); axis([min(t), max(t), -g_max, g_max]); ylabel('Gy');
subplot(9, 2, 12); plot(t, g_scale * data(15, :)', 'b'); axis([min(t), max(t), -g_max, g_max]); ylabel('Gz');

subplot(9, 2, 14); plot(t, m_scale * data(16, :)', 'r'); axis([min(t), max(t), -m_max, m_max]); ylabel('Mx');
subplot(9, 2, 16); plot(t, m_scale * data(17, :)', 'g'); axis([min(t), max(t), -m_max, m_max]); ylabel('My');
subplot(9, 2, 18); plot(t, m_scale * data(18, :)', 'b'); axis([min(t), max(t), -m_max, m_max]); ylabel('Mz');

%%
