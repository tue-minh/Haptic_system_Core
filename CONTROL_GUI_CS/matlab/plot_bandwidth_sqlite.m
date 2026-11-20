%% plot_bandwidth_sqlite.m
% Verify & plot Experiment 1b (Bandwidth Sweep) logged by ControlGui_CS into SQLite.
%
% Usage:
%   plot_bandwidth_sqlite()                          % opens file picker
%   plot_bandwidth_sqlite('recordings/bandwidth_xxx.db')

function plot_bandwidth_sqlite(dbPath)
    if nargin < 1 || isempty(dbPath)
        [f, p] = uigetfile('*.db', 'Select Bandwidth Experiment SQLite DB');
        if isequal(f, 0)
            disp('Cancelled.');
            return;
        end
        dbPath = fullfile(p, f);
    end

    if ~isfile(dbPath)
        error('SQLite file not found: %s', dbPath);
    end

    conn = sqlite(dbPath, 'readonly');
    cleanup = onCleanup(@() close(conn));

    metaTbl = fetch(conn, 'SELECT key, value FROM experiment_meta');
    samplesTbl = fetch(conn, [ ...
        'SELECT t_s, freq_hz, step_idx, is_settling, u_pct, u_byte, ', ...
        'ch0_mv, ch1_mv, ch2_mv, ch3_mv, ch4_mv ', ...
        'FROM samples ORDER BY id ASC']);
    bodeTbl = fetch(conn, [ ...
        'SELECT step_idx, freq_hz, n_samples, amp_mv, phase_deg, mag_db, dc_offset_mv ', ...
        'FROM bode_summary ORDER BY freq_hz ASC']);

    % Extract metadata
    chIdx = 0;
    if istable(metaTbl)
        keys = string(metaTbl.key);
        vals = string(metaTbl.value);
    else
        keys = string(metaTbl(:, 1));
        vals = string(metaTbl(:, 2));
    end
    hit = find(keys == "channel_index", 1);
    if ~isempty(hit)
        chIdx = str2double(vals(hit));
        if chIdx < 0 || chIdx > 4, chIdx = 0; end
    end

    % Convert samples table/cell to arrays
    if istable(samplesTbl)
        t_s      = double(samplesTbl.t_s);
        freq_hz  = double(samplesTbl.freq_hz);
        step_idx = double(samplesTbl.step_idx);
        settle   = double(samplesTbl.is_settling);
        u_pct    = double(samplesTbl.u_pct);
        ch_mv    = double(samplesTbl.(sprintf('ch%d_mv', chIdx)));
    else
        t_s      = cell2mat(samplesTbl(:, 1));
        freq_hz  = cell2mat(samplesTbl(:, 2));
        step_idx = cell2mat(samplesTbl(:, 3));
        settle   = cell2mat(samplesTbl(:, 4));
        u_pct    = cell2mat(samplesTbl(:, 5));
        ch_mv    = cell2mat(samplesTbl(:, 7 + chIdx));
    end

    %% Figure 1: Time-domain Command vs Measured ADC
    figure('Name', 'Experiment 1b - Time Domain', 'Color', 'w');
    yyaxis left;
    plot(t_s, u_pct, 'b-', 'LineWidth', 1.2);
    ylabel('Command u(t) [%]');
    ylim([-5, 105]);

    yyaxis right;
    plot(t_s, ch_mv, 'r-', 'LineWidth', 1.1);
    ylabel(sprintf('Measured ADC Ch%d [mV]', chIdx));
    grid on;
    xlabel('Time [s]');
    title(sprintf('Stepped-Sine Response (Ch%d) — %s', chIdx, dbPath), 'Interpreter', 'none');
    legend('Command u(t) [%]', sprintf('ADC Ch%d [mV]', chIdx), 'Location', 'best');

    %% Cross-check Sine Fit in MATLAB from raw samples
    uniqueSteps = unique(step_idx, 'stable');
    fList = zeros(numel(uniqueSteps), 1);
    ampList = zeros(numel(uniqueSteps), 1);
    phaseList = zeros(numel(uniqueSteps), 1);

    for k = 1:numel(uniqueSteps)
        st = uniqueSteps(k);
        mask = (step_idx == st) & (settle == 0);
        if nnz(mask) < 6
            continue;
        end
        tk = t_s(mask);
        tk = tk - tk(1);
        yk = ch_mv(mask);
        fk = freq_hz(find(mask, 1));
        w = 2 * pi * fk;

        X = [ones(size(tk)), cos(w * tk), sin(w * tk)];
        beta = X \ yk;
        a = beta(2);
        b = beta(3);
        fList(k) = fk;
        ampList(k) = hypot(a, b);
        phaseList(k) = atan2d(a, b);
    end

    valid = fList > 0;
    fList = fList(valid);
    ampList = ampList(valid);
    phaseList = phaseList(valid);

    [fSorted, ord] = sort(fList);
    ampSorted = ampList(ord);
    phaseSorted = phaseList(ord);
    aRef = ampSorted(1);
    magDbMatlab = 20 * log10(max(1e-6, ampSorted) / max(1e-6, aRef));

    % Interpolate -3 dB cutoff fc
    fcHz = NaN;
    for k = 1:numel(fSorted)-1
        if magDbMatlab(k) >= -3 && magDbMatlab(k+1) <= -3
            frac = (-3 - magDbMatlab(k)) / (magDbMatlab(k+1) - magDbMatlab(k));
            fcHz = fSorted(k) + frac * (fSorted(k+1) - fSorted(k));
            break;
        end
    end

    %% Figure 2: Bode Plot (MATLAB fit vs C# bode_summary)
    figure('Name', 'Experiment 1b - Bode Plot', 'Color', 'w');
    subplot(2, 1, 1);
    semilogx(fSorted, magDbMatlab, 'o-', 'LineWidth', 1.6, 'MarkerSize', 6);
    hold on;
    if ~isempty(bodeTbl)
        if istable(bodeTbl)
            fCs = double(bodeTbl.freq_hz);
            mCs = double(bodeTbl.mag_db);
            pCs = double(bodeTbl.phase_deg);
        else
            fCs = cell2mat(bodeTbl(:, 2));
            mCs = cell2mat(bodeTbl(:, 6));
            pCs = cell2mat(bodeTbl(:, 5));
        end
        semilogx(fCs, mCs, 'x--', 'LineWidth', 1.2, 'MarkerSize', 7);
    end
    yline(-3, '--r', '-3 dB', 'LineWidth', 1.2);
    if ~isnan(fcHz)
        xline(fcHz, ':k', sprintf('fc = %.2f Hz', fcHz), 'LineWidth', 1.4);
    end
    grid on;
    ylabel('Magnitude [dB]');
    title(sprintf('Bode Frequency Response (Ch%d) | -3 dB fc = %.2f Hz', chIdx, fcHz));
    if ~isempty(bodeTbl)
        legend('MATLAB Fit', 'C# GUI Summary', '-3 dB', 'Location', 'southwest');
    end

    subplot(2, 1, 2);
    semilogx(fSorted, unwrap(deg2rad(phaseSorted)) * 180/pi, 'o-', 'LineWidth', 1.6);
    if ~isempty(bodeTbl)
        hold on;
        semilogx(fCs, unwrap(deg2rad(pCs)) * 180/pi, 'x--', 'LineWidth', 1.2);
    end
    grid on;
    xlabel('Frequency [Hz]');
    ylabel('Phase [deg]');
end
