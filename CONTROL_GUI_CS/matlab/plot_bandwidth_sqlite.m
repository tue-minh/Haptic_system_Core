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

    % Extract metadata helper
    if istable(metaTbl)
        keys = string(metaTbl.key);
        vals = string(metaTbl.value);
    else
        keys = string(metaTbl(:, 1));
        vals = string(metaTbl(:, 2));
    end
    getMeta = @(k, defVal) localGetMeta(keys, vals, k, defVal);

    chIdx = str2double(getMeta("channel_index", "0"));
    if isnan(chIdx) || chIdx < 0 || chIdx > 4, chIdx = 0; end
    sweepMode = lower(strtrim(getMeta("sweep_mode", "stepped_sine")));

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
    plot(t_s, u_pct, 'b-', 'LineWidth', 1.1);
    ylabel('Command u(t) [%]');
    ylim([-5, 105]);

    yyaxis right;
    plot(t_s, ch_mv, 'r-', 'LineWidth', 1.0);
    ylabel(sprintf('Measured ADC Ch%d [mV]', chIdx));
    grid on;
    xlabel('Time [s]');
    title(sprintf('Exp 1b (%s, Ch%d) — %s', sweepMode, chIdx, dbPath), 'Interpreter', 'none');
    legend('Command u(t) [%]', sprintf('ADC Ch%d [mV]', chIdx), 'Location', 'best');

    %% Cross-check Sine / Chirp Demodulation in MATLAB
    if sweepMode == "chirp"
        f0 = str2double(getMeta("chirp_f0_hz", "0.1"));
        f1 = str2double(getMeta("chirp_f1_hz", "10.0"));
        totalDur = str2double(getMeta("total_duration_sec", string(max(t_s))));
        settleSec = str2double(getMeta("chirp_settle_sec", "1.0"));
        uaPct = str2double(getMeta("ua_pct", "25.0"));
        nBins = round(str2double(getMeta("freq_points", "25")));
        if isnan(nBins) || nBins < 10, nBins = 25; end

        chirpDur = max(2.0, totalDur - settleSec);
        maskActive = (settle == 0);
        tChirp = t_s(maskActive);
        if ~isempty(tChirp)
            tChirp = tChirp - tChirp(1);
        end
        uAct = u_pct(maskActive);
        yAct = ch_mv(maskActive);

        evalFreqs = logspace(log10(min(f0, f1)), log10(max(f0, f1)), nBins).';
        [fSorted, ampSorted, phaseSorted] = localDemodChirp(tChirp, uAct, yAct, f0, f1, chirpDur, uaPct, evalFreqs);
    else
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
            uk = u_pct(mask);
            fk = freq_hz(find(mask, 1));
            w = 2 * pi * fk;

            X = [ones(size(tk)), cos(w * tk), sin(w * tk)];
            by = X \ yk;
            bu = X \ uk;
            fList(k) = fk;
            ampList(k) = hypot(by(2), by(3));
            phY = atan2d(by(2), by(3));
            phU = atan2d(bu(2), bu(3));
            phaseList(k) = mod(phY - phU + 180, 360) - 180;
        end

        valid = fList > 0;
        fList = fList(valid);
        ampList = ampList(valid);
        phaseList = phaseList(valid);

        [fSorted, ord] = sort(fList);
        ampSorted = ampList(ord);
        phaseSorted = phaseList(ord);
    end

    if isempty(fSorted)
        warning('No valid Bode points extracted.');
        return;
    end

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

    % Fit 2nd-order + time delay model for smooth curve (Paper Fig. 6)
    fDense = logspace(log10(fSorted(1)), log10(fSorted(end)), 200).';
    magFit = [];
    phaseFit = [];
    try
        [magFit, phaseFit, fcFit] = localFitSecondOrderModel(fSorted, magDbMatlab, phaseSorted, fDense);
        if isnan(fcHz) && ~isnan(fcFit)
            fcHz = fcFit;
        end
    catch
    end

    %% Figure 2: Bode Plot (Measured vs Model Fit & C# bode_summary)
    figure('Name', 'Experiment 1b - Bode Plot', 'Color', 'w');
    subplot(2, 1, 1);
    semilogx(fSorted, magDbMatlab, 'o', 'LineWidth', 1.4, 'MarkerSize', 6, 'DisplayName', 'Measured (MATLAB)');
    hold on;
    if ~isempty(magFit)
        semilogx(fDense, magFit, '-', 'LineWidth', 1.8, 'DisplayName', '2nd-Order Fit');
    end
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
        semilogx(fCs, mCs, 'x--', 'LineWidth', 1.1, 'MarkerSize', 6, 'DisplayName', 'C# GUI Summary');
    end
    yline(-3, '--r', '-3 dB', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    if ~isnan(fcHz)
        xline(fcHz, ':k', sprintf('fc = %.2f Hz', fcHz), 'LineWidth', 1.4, 'HandleVisibility', 'off');
    end
    grid on;
    ylabel('Magnitude [dB]');
    title(sprintf('Bode Frequency Response (%s, Ch%d) | -3 dB fc = %.2f Hz', sweepMode, chIdx, fcHz), 'Interpreter', 'none');
    legend('Location', 'southwest');

    subplot(2, 1, 2);
    phUnwrapped = unwrap(deg2rad(phaseSorted)) * 180/pi;
    semilogx(fSorted, phUnwrapped, 'o', 'LineWidth', 1.4, 'MarkerSize', 6, 'DisplayName', 'Measured (MATLAB)');
    hold on;
    if ~isempty(phaseFit)
        semilogx(fDense, phaseFit, '-', 'LineWidth', 1.8, 'DisplayName', '2nd-Order + Delay Fit');
    end
    if ~isempty(bodeTbl)
        semilogx(fCs, unwrap(deg2rad(pCs)) * 180/pi, 'x--', 'LineWidth', 1.1, 'MarkerSize', 6, 'DisplayName', 'C# GUI Summary');
    end
    grid on;
    xlabel('Frequency [Hz]');
    ylabel('Phase [deg]');
end

function val = localGetMeta(keys, vals, keyName, defaultVal)
    hit = find(keys == keyName, 1);
    if isempty(hit)
        val = string(defaultVal);
    else
        val = vals(hit);
    end
end

function [fOut, ampOut, phOut] = localDemodChirp(tChirp, uAct, yAct, f0, f1, chirpDur, uaPct, evalFreqs)
    lnr = log(f1 / f0);
    phiFn = @(tv) (2*pi*f0*chirpDur / lnr) * (exp(lnr * tv / chirpDur) - 1.0);
    if abs(f1 - f0) <= 1e-6 * max(f0, f1)
        phiFn = @(tv) 2*pi*f0*tv;
    end
    tSkip = min(0.35, 0.02 * chirpDur);

    nB = numel(evalFreqs);
    fOut = zeros(nB, 1);
    ampOut = zeros(nB, 1);
    phOut = zeros(nB, 1);
    valid = false(nB, 1);

    for b = 1:nB
        fc = evalFreqs(b);
        tc = chirpDur * log(fc / f0) / lnr;
        halfW = min(max(1.75 / fc, 0.6), max(1.5, chirpDur * 0.22));
        mask = (tChirp >= tc - halfW) & (tChirp <= tc + halfW);
        if tc >= 0.25
            mask2 = mask & (tChirp >= tSkip);
            if nnz(mask2) >= 8
                mask = mask2;
            end
        end
        if nnz(mask) < 6
            continue;
        end
        tw = tChirp(mask);
        uw = uAct(mask);
        yw = yAct(mask);
        phw = phiFn(tw);
        w = max(0.5 * (1.0 + cos(pi * max(-1, min(1, (tw - tc) / halfW)))), 1e-3);
        sqw = sqrt(w);

        X = [ones(size(tw)), tw - mean(tw), cos(phw), sin(phw)];
        Xw = X .* sqw;
        bu = Xw \ (uw .* sqw);
        by = Xw \ (yw .* sqw);

        ampU = hypot(bu(3), bu(4));
        ampY = hypot(by(3), by(4));
        if ampU < 1e-4
            continue;
        end
        fOut(b) = fc;
        ampOut(b) = ampY * (uaPct / ampU);
        phOut(b) = mod(atan2d(by(3), by(4)) - atan2d(bu(3), bu(4)) + 180, 360) - 180;
        valid(b) = true;
    end

    [fOut, ord] = sort(fOut(valid));
    ampOut = ampOut(valid);
    ampOut = ampOut(ord);
    phOut = phOut(valid);
    phOut = phOut(ord);
end

function [magFit, phaseFit, fcFit] = localFitSecondOrderModel(f, magDb, phaseDeg, fDense)
    phUnwrap = unwrap(deg2rad(phaseDeg)) * 180/pi;
    evalModel = @(p, fx) deal( ...
        20*log10(abs( (2*pi*exp(p(1)))^2 ./ ((1j*2*pi*fx).^2 + 2*exp(p(2))*(2*pi*exp(p(1)))*(1j*2*pi*fx) + (2*pi*exp(p(1)))^2) )), ...
        unwrap(angle( ((2*pi*exp(p(1)))^2 ./ ((1j*2*pi*fx).^2 + 2*exp(p(2))*(2*pi*exp(p(1)))*(1j*2*pi*fx) + (2*pi*exp(p(1)))^2)) .* exp(-1j*2*pi*fx*max(0,p(3))) )) * 180/pi ...
    );
    costFn = @(p) localModelCost(p, f, magDb, phUnwrap, evalModel);
    p0 = [log(4.5), log(0.7), 0.015];
    opts = optimset('Display', 'off', 'MaxIter', 400);
    pBest = fminsearch(costFn, p0, opts);
    [magFit, phaseFit] = evalModel(pBest, fDense);
    magFit = magFit - magFit(1);
    fcFit = NaN;
    idx = find(magFit(1:end-1) >= -3 & magFit(2:end) <= -3, 1);
    if ~isempty(idx)
        frac = (-3 - magFit(idx)) / (magFit(idx+1) - magFit(idx));
        fcFit = fDense(idx) + frac * (fDense(idx+1) - fDense(idx));
    end
end

function err = localModelCost(p, f, magDb, phUnwrap, evalModel)
    [m, ph] = evalModel(p, f);
    m = m - m(1);
    err = sum((m - magDb).^2) + 0.002 * sum((ph - phUnwrap).^2);
end
