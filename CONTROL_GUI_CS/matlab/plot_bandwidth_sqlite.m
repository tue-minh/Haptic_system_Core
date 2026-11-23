%% plot_bandwidth_sqlite.m
% Verify & plot Experiment 1b (Bandwidth Sweep) logged by ControlGui_CS into SQLite.
% Supports plotting 1 file or overlaying multiple valve DBs (e.g. Valve_A..Valve_E)
% in the same Time-Domain & Bode charts.
%
% Usage:
%   plot_bandwidth_sqlite()                                  % auto-loads Valve_A..E in ../ControlGui.Wpf/recordings if found, else file picker (MultiSelect)
%   plot_bandwidth_sqlite('pick')                            % opens multi-file picker
%   plot_bandwidth_sqlite('path/to/folder')                  % plots all Valve_*.db (or *.db) in folder together
%   plot_bandwidth_sqlite('Valve_A.db')                      % single file (detailed + 2nd-order fit + C# summary)
%   plot_bandwidth_sqlite({'Valve_A.db', 'Valve_B.db', ...}) % multiple files in one chart
%   plot_bandwidth_sqlite('Valve_A.db', 'Valve_B.db', ...)   % varargin multiple files in one chart

function plot_bandwidth_sqlite(varargin)
    dbPaths = localResolveDbPaths(varargin{:});
    if isempty(dbPaths)
        disp('Cancelled / no database files selected.');
        return;
    end

    nFiles = numel(dbPaths);
    results = cell(nFiles, 1);
    for i = 1:nFiles
        if ~isfile(dbPaths{i})
            error('SQLite file not found: %s', dbPaths{i});
        end
        results{i} = localProcessDb(dbPaths{i});
    end

    % Single-file detailed mode (keeps 2nd-order fit & C# GUI summary comparison)
    if nFiles == 1
        localPlotSingle(results{1});
        return;
    end

    % Multi-file mode: plot all valves together in the same Time-Domain & Bode charts
    localPlotMulti(results);
end

function dbPaths = localResolveDbPaths(varargin)
    dbPaths = {};
    thisDir = fileparts(mfilename('fullpath'));
    defaultRecDir = fullfile(thisDir, '..', 'ControlGui.Wpf', 'recordings');

    if nargin == 0
        % Auto-detect Valve_A.db .. Valve_E.db in ../ControlGui.Wpf/recordings
        letters = {'A', 'B', 'C', 'D', 'E'};
        autoList = {};
        for k = 1:numel(letters)
            cand = fullfile(defaultRecDir, sprintf('Valve_%s.db', letters{k}));
            if isfile(cand)
                autoList{end+1} = cand; %#ok<AGROW>
            end
        end
        if ~isempty(autoList)
            dbPaths = autoList;
            return;
        end
        dbPaths = localPickFiles(defaultRecDir);
        return;
    end

    if nargin == 1
        arg = varargin{1};
        if (ischar(arg) || (isstring(arg) && isscalar(arg)))
            s = char(arg);
            if strcmpi(strtrim(s), 'pick')
                dbPaths = localPickFiles(defaultRecDir);
                return;
            elseif isfolder(s)
                d = dir(fullfile(s, 'Valve_*.db'));
                if isempty(d)
                    d = dir(fullfile(s, '*.db'));
                end
                [~, ord] = sort({d.name});
                d = d(ord);
                dbPaths = arrayfun(@(x) fullfile(x.folder, x.name), d, 'UniformOutput', false);
                return;
            elseif contains(s, '*')
                d = dir(s);
                [~, ord] = sort({d.name});
                d = d(ord);
                dbPaths = arrayfun(@(x) fullfile(x.folder, x.name), d, 'UniformOutput', false);
                return;
            else
                dbPaths = {s};
                return;
            end
        elseif iscell(arg)
            dbPaths = cellfun(@char, arg(:)', 'UniformOutput', false);
            return;
        elseif isstring(arg)
            dbPaths = cellstr(arg(:)');
            return;
        end
    end

    % Varargin list of paths
    for i = 1:nargin
        arg = varargin{i};
        if iscell(arg)
            sub = cellfun(@char, arg(:)', 'UniformOutput', false);
            dbPaths = [dbPaths, sub]; %#ok<AGROW>
        elseif isstring(arg)
            dbPaths = [dbPaths, cellstr(arg(:)')]; %#ok<AGROW>
        else
            dbPaths{end+1} = char(arg); %#ok<AGROW>
        end
    end
end

function dbPaths = localPickFiles(startDir)
    dbPaths = {};
    filterSpec = '*.db';
    if isfolder(startDir)
        filterSpec = fullfile(startDir, '*.db');
    end
    [f, p] = uigetfile(filterSpec, 'Select Bandwidth Experiment SQLite DB(s)', 'MultiSelect', 'on');
    if isequal(f, 0)
        return;
    end
    if iscell(f)
        dbPaths = cellfun(@(fn) fullfile(p, fn), f, 'UniformOutput', false);
    else
        dbPaths = {fullfile(p, f)};
    end
end

function res = localProcessDb(dbPath)
    conn = sqlite(dbPath, 'readonly');
    cleanup = onCleanup(@() close(conn)); %#ok<NASGU>

    metaTbl = fetch(conn, 'SELECT key, value FROM experiment_meta');
    samplesTbl = fetch(conn, [ ...
        'SELECT t_s, freq_hz, step_idx, is_settling, u_pct, u_byte, ', ...
        'ch0_mv, ch1_mv, ch2_mv, ch3_mv, ch4_mv ', ...
        'FROM samples ORDER BY id ASC']);
    bodeTbl = fetch(conn, [ ...
        'SELECT step_idx, freq_hz, n_samples, amp_mv, phase_deg, mag_db, dc_offset_mv ', ...
        'FROM bode_summary ORDER BY freq_hz ASC']);

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
        error('No valid Bode points extracted from %s.', dbPath);
    end

    aRef = ampSorted(1);
    magDbMatlab = 20 * log10(max(1e-6, ampSorted) / max(1e-6, aRef));
    phaseUnwrapped = unwrap(deg2rad(phaseSorted)) * 180 / pi;

    % Interpolate -3 dB cutoff fc
    fcHz = NaN;
    for k = 1:numel(fSorted)-1
        if magDbMatlab(k) >= -3 && magDbMatlab(k+1) <= -3
            frac = (-3 - magDbMatlab(k)) / (magDbMatlab(k+1) - magDbMatlab(k));
            fcHz = fSorted(k) + frac * (fSorted(k+1) - fSorted(k));
            break;
        end
    end

    % Fit 2nd-order + time delay model
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

    % C# bode_summary columns
    fCs = []; mCs = []; pCs = [];
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
    end

    [~, baseName, ~] = fileparts(dbPath);
    shortName = regexprep(baseName, '_', ' ');

    res = struct();
    res.dbPath = dbPath;
    res.label = sprintf('%s (Ch%d)', shortName, chIdx);
    res.shortName = shortName;
    res.chIdx = chIdx;
    res.sweepMode = sweepMode;
    res.u0Pct = str2double(getMeta("u0_pct", "NaN"));
    res.uaPct = str2double(getMeta("ua_pct", "NaN"));
    res.t_s = t_s;
    res.u_pct = u_pct;
    res.ch_mv = ch_mv;
    res.fSorted = fSorted;
    res.ampSorted = ampSorted;
    res.magDb = magDbMatlab;
    res.phaseDeg = phaseSorted;
    res.phaseUnwrapped = phaseUnwrapped;
    res.fcHz = fcHz;
    res.fDense = fDense;
    res.magFit = magFit;
    res.phaseFit = phaseFit;
    res.fCs = fCs;
    res.mCs = mCs;
    res.pCs = pCs;
end

function localPlotSingle(r)
    %% Figure 1: Time-domain Command vs Measured ADC
    figure('Name', 'Experiment 1b - Time Domain', 'Color', 'w');
    yyaxis left;
    plot(r.t_s, r.u_pct, 'b-', 'LineWidth', 1.1);
    ylabel('Command u(t) [%]');
    ylim([-5, 105]);

    yyaxis right;
    plot(r.t_s, r.ch_mv, 'r-', 'LineWidth', 1.0);
    ylabel(sprintf('Measured ADC Ch%d [mV]', r.chIdx));
    grid on;
    xlabel('Time [s]');
    title(sprintf('Exp 1b (%s, Ch%d) — %s', r.sweepMode, r.chIdx, r.dbPath), 'Interpreter', 'none');
    legend('Command u(t) [%]', sprintf('ADC Ch%d [mV]', r.chIdx), 'Location', 'best');

    %% Figure 2: Bode Plot (Measured vs Model Fit & C# bode_summary)
    figure('Name', 'Experiment 1b - Bode Plot', 'Color', 'w');
    ax1 = subplot(2, 1, 1);
    semilogx(r.fSorted, r.magDb, 'o', 'LineWidth', 1.4, 'MarkerSize', 6, 'DisplayName', 'Measured (MATLAB)');
    hold on;
    if ~isempty(r.magFit)
        semilogx(r.fDense, r.magFit, '-', 'LineWidth', 1.8, 'DisplayName', '2nd-Order Fit');
    end
    if ~isempty(r.fCs)
        semilogx(r.fCs, r.mCs, 'x--', 'LineWidth', 1.1, 'MarkerSize', 6, 'DisplayName', 'C# GUI Summary');
    end
    yline(-3, '--r', '-3 dB', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    if ~isnan(r.fcHz)
        xline(r.fcHz, ':k', sprintf('fc = %.2f Hz', r.fcHz), 'LineWidth', 1.4, 'HandleVisibility', 'off');
    end
    grid on;
    ylabel('Magnitude [dB]');
    title(sprintf('Bode Frequency Response (%s, %s) | -3 dB fc = %.2f Hz', r.sweepMode, r.label, r.fcHz), 'Interpreter', 'none');
    legend('Location', 'southwest');

    ax2 = subplot(2, 1, 2);
    semilogx(r.fSorted, r.phaseUnwrapped, 'o', 'LineWidth', 1.4, 'MarkerSize', 6, 'DisplayName', 'Measured (MATLAB)');
    hold on;
    if ~isempty(r.phaseFit)
        semilogx(r.fDense, r.phaseFit, '-', 'LineWidth', 1.8, 'DisplayName', '2nd-Order + Delay Fit');
    end
    if ~isempty(r.fCs)
        semilogx(r.fCs, unwrap(deg2rad(r.pCs)) * 180/pi, 'x--', 'LineWidth', 1.1, 'MarkerSize', 6, 'DisplayName', 'C# GUI Summary');
    end
    grid on;
    xlabel('Frequency [Hz]');
    ylabel('Phase [deg]');
    linkaxes([ax1, ax2], 'x');
end

function localPlotMulti(results)
    n = numel(results);
    colors = lines(max(n, 5));
    markers = {'o', 's', '^', 'd', 'v', 'p', 'h', '*'};

    fprintf('\n=== Bandwidth Summary (%d Valves) ===\n', n);
    fprintf('%-16s | Ch |   u0 / ua (%%) | Amp@0.1Hz (mV) | -3dB fc (Hz) | Samples\n', 'Valve');
    fprintf('%s\n', repmat('-', 1, 80));
    for i = 1:n
        r = results{i};
        fprintf('%-16s | %2d | %5.1f / %5.1f | %14.2f | %12.2f | %7d\n', ...
            r.shortName, r.chIdx, r.u0Pct, r.uaPct, r.ampSorted(1), r.fcHz, numel(r.t_s));
    end
    fprintf('\n');

    %% Figure 1: Combined Time-Domain Waveform (Command + All Valves on Same Chart)
    figure('Name', sprintf('Experiment 1b - Time Domain (%d Valves)', n), 'Color', 'w', 'Position', [80, 120, 1050, 620]);
    axU = subplot(4, 1, 1);
    plot(results{1}.t_s, results{1}.u_pct, 'k-', 'LineWidth', 1.1);
    grid on;
    ylabel('u(t) [%]');
    title(sprintf('Time-Domain Excitation & Response (%d Valves)', n));
    if ~isnan(results{1}.u0Pct) && ~isnan(results{1}.uaPct)
        legend(sprintf('Command u(t): u0=%.1f%%, ua=%.1f%%', results{1}.u0Pct, results{1}.uaPct), 'Location', 'best');
    else
        legend('Command u(t) [%]', 'Location', 'best');
    end

    axY = subplot(4, 1, 2:4);
    hold on;
    for i = 1:n
        r = results{i};
        c = colors(i, :);
        plot(r.t_s, r.ch_mv, '-', 'Color', c, 'LineWidth', 1.0, 'DisplayName', r.label);
    end
    grid on;
    xlabel('Time [s]');
    ylabel('Measured ADC [mV]');
    legend('Location', 'best');
    linkaxes([axU, axY], 'x');

    %% Figure 2: Combined Bode Plot (All Valves in One Chart)
    figure('Name', sprintf('Experiment 1b - Combined Bode (%d Valves)', n), 'Color', 'w', 'Position', [140, 80, 1050, 680]);
    axMag = subplot(2, 1, 1);
    hold on;
    for i = 1:n
        r = results{i};
        c = colors(i, :);
        mk = markers{mod(i-1, numel(markers)) + 1};
        if isnan(r.fcHz)
            legStr = r.label;
        else
            legStr = sprintf('%s | fc = %.2f Hz', r.label, r.fcHz);
        end
        semilogx(r.fSorted, r.magDb, ['-' mk], 'Color', c, 'LineWidth', 1.5, ...
            'MarkerSize', 5, 'MarkerFaceColor', 'w', 'DisplayName', legStr);
    end
    set(axMag, 'XScale', 'log');
    yline(-3, '--r', '-3 dB', 'LineWidth', 1.3, 'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'off');
    grid on;
    ylabel('Magnitude [dB]');
    title(sprintf('Combined Bode Frequency Response (%d Valves)', n));
    legend('Location', 'southwest', 'FontSize', 9);

    axPh = subplot(2, 1, 2);
    hold on;
    for i = 1:n
        r = results{i};
        c = colors(i, :);
        mk = markers{mod(i-1, numel(markers)) + 1};
        semilogx(r.fSorted, r.phaseUnwrapped, ['-' mk], 'Color', c, 'LineWidth', 1.5, ...
            'MarkerSize', 5, 'MarkerFaceColor', 'w', 'DisplayName', r.label);
    end
    set(axPh, 'XScale', 'log');
    grid on;
    xlabel('Frequency [Hz]');
    ylabel('Phase [deg]');
    legend('Location', 'southwest', 'FontSize', 9);
    linkaxes([axMag, axPh], 'x');
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
