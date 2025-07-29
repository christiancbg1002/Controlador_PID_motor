clc; clear;

% --- Configuración serial
s = configurarSerial("COM6", 115200);

% --- Enviar comando de inicio
pause(2);
writeline(s, "START");
disp("Comando START enviado. Presiona Ctrl+C para detener.");

% --- Inicializar figura
h = inicializarGrafica();

% --- Recolección en tiempo real
tiempo_raw = [];
rpm_raw = [];
try
    while true
        [t, r] = leerDato(s);
        if ~isempty(t)
            tiempo_raw(end+1) = t;
            rpm_raw(end+1) = r;

            if mod(length(tiempo_raw), 10) == 0
                actualizarGrafica(h, tiempo_raw, rpm_raw);
            end
        end
    end
catch
    disp("Captura detenida por el usuario.");
end

clear s;
fprintf('Lectura finalizada. Total muestras: %d\n', length(tiempo_raw));

% --- Interpolación y guardado
guardarDatosInterpolados(tiempo_raw, rpm_raw, 0.05, 1.0, 300, 'respuesta_motor_interp.csv');

%% ------------------- FUNCIONES ------------------- %%
function s = configurarSerial(puerto, baud)
    s = serialport(puerto, baud);
    configureTerminator(s, "LF");
    flush(s);
end

function h = inicializarGrafica()
    figure;
    h = plot(NaN, NaN, 'b.-');
    xlabel('Tiempo (s)'); ylabel('Velocidad (RPM)');
    title('Respuesta del Motor en Tiempo Real');
    grid on;
    ylim([0, 400]);
end

function [t, r] = leerDato(s)
    t = []; r = [];
    if s.NumBytesAvailable > 0
        linea = readline(s);
        partes = split(linea, ',');
        if numel(partes) == 2
            t = str2double(partes{1});
            r = str2double(partes{2});
            if ~isfinite(t) || ~isfinite(r)
                t = []; r = [];
            end
        end
    end
end

function actualizarGrafica(h, tiempo, rpm)
    set(h, 'XData', tiempo, 'YData', rpm);
    xlim([max(0, tiempo(end)-10), tiempo(end)]);  % Muestra últimos 10s
    drawnow;
end

function guardarDatosInterpolados(tiempo, rpm, paso, t_escalon, ref_valor, archivo)
    t_min = ceil(tiempo(1)/paso)*paso;
    t_max = floor(tiempo(end)/paso)*paso;
    t_uniforme = t_min:paso:t_max;
    rpm_interp = interp1(tiempo, rpm, t_uniforme, 'linear');

    referencia = zeros(size(t_uniforme));
    referencia(t_uniforme >= t_escalon) = ref_valor;

    tabla = table(t_uniforme', rpm_interp', referencia', ...
        'VariableNames', {'Tiempo_s', 'RPM', 'Referencia'});
    writetable(tabla, archivo);
    fprintf('Datos interpolados guardados en "%s".\n', archivo);
end
