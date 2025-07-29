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
ref_raw = [];
error_raw = [];

try
    while true
        [t, r, ref, err] = leerDato(s);
        if ~isempty(t)
            tiempo_raw(end+1) = t;
            rpm_raw(end+1) = r;
            ref_raw(end+1) = ref;
            error_raw(end+1) = err;

            if mod(length(tiempo_raw), 10) == 0
                actualizarGrafica(h, tiempo_raw, rpm_raw, ref_raw, error_raw);
            end
        end
    end
catch
    disp("Captura detenida por el usuario.");
end

clear s;
fprintf('Lectura finalizada. Total muestras: %d\n', length(tiempo_raw));

% --- Interpolación y guardado
guardarDatosInterpolados(tiempo_raw, rpm_raw, ref_raw, error_raw, 0.05, 'respuesta_motor_interp.csv');

% --- Cálculo del tiempo de establecimiento
settling_time = calcularTiempoEstablecimiento(tiempo_raw, rpm_raw, ref_raw, 0.05);

if isempty(settling_time)
    fprintf('No se alcanzó el estado estable dentro de ±5%% del valor de referencia.\n');
else
    fprintf('Tiempo de establecimiento: %.3f segundos\n', settling_time);
end

%% ------------------- FUNCIONES ------------------- %%

function s = configurarSerial(puerto, baud)
    s = serialport(puerto, baud);
    configureTerminator(s, "LF");
    flush(s);
end

function h = inicializarGrafica()
    figure;
    hold on;
    h.rpm   = plot(NaN, NaN, 'b.-',  'DisplayName', 'RPM medida');
    h.ref   = plot(NaN, NaN, 'r--',  'DisplayName', 'Referencia');
    h.error = plot(NaN, NaN, 'g-',   'DisplayName', 'Error');
    xlabel('Tiempo (s)');
    ylabel('Velocidad (RPM) / Error');
    title('Respuesta del Motor en Tiempo Real');
    legend('show');
    grid on;
    ylim([-100, 400]);
end

function [t, r, ref, err] = leerDato(s)
    t = []; r = []; ref = []; err = [];
    if s.NumBytesAvailable > 0
        linea = readline(s);
        partes = split(linea, ',');
        if numel(partes) == 4
            t    = str2double(partes{1});
            r    = str2double(partes{2});
            ref  = str2double(partes{3});
            err  = str2double(partes{4});
            if ~isfinite(t) || ~isfinite(r) || ~isfinite(ref) || ~isfinite(err)
                t = []; r = []; ref = []; err = [];
            end
        end
    end
end

function actualizarGrafica(h, tiempo, rpm, referencia, error)
    set(h.rpm,   'XData', tiempo, 'YData', rpm);
    set(h.ref,   'XData', tiempo, 'YData', referencia);
    set(h.error, 'XData', tiempo, 'YData', error);
    xlim([max(0, tiempo(end)-10), tiempo(end)]);
    drawnow;
end

function guardarDatosInterpolados(tiempo, rpm, referencia, error, paso, archivo)
    t_min = ceil(tiempo(1)/paso)*paso;
    t_max = floor(tiempo(end)/paso)*paso;
    t_uniforme = t_min:paso:t_max;

    rpm_interp   = interp1(tiempo, rpm,        t_uniforme, 'linear');
    ref_interp   = interp1(tiempo, referencia, t_uniforme, 'linear');
    error_interp = interp1(tiempo, error,      t_uniforme, 'linear');

    tabla = table(t_uniforme', rpm_interp', ref_interp', error_interp', ...
        'VariableNames', {'Tiempo_s', 'RPM', 'Referencia', 'Error'});
    writetable(tabla, archivo);
    fprintf('Datos interpolados guardados en "%s".\n', archivo);
end

function ts = calcularTiempoEstablecimiento(tiempo, rpm, referencia, tolerancia)
    % Se asume que la referencia puede ser variable
    error_abs = abs(rpm - referencia);
    banda = abs(referencia) * tolerancia;

    dentro_banda = error_abs <= banda;

    ts = [];
    for i = 1:length(dentro_banda)
        if dentro_banda(i)
            % Verifica si permanece dentro de la banda hasta el final
            if all(dentro_banda(i:end))
                ts = tiempo(i);
                break;
            end
        end
    end
end
