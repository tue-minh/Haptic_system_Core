%% plot_gripper_transition_sqlite.m
% Plot & analyze Experiment 2 (Gripper Closure Transition) logged by
% GripperTransitionWindow.xaml.cs / GripperTransitionRunner.cs into SQLite.
%
% Usage:
%   plot_gripper_transition_sqlite()                                  % opens file picker
%   plot_gripper_transition_sqlite('recordings/gripper_transition_xxx.db')

function plot_gripper_transition_sqlite(dbPath)
    if nargin < 1 || isempty(dbPath)
        [f, p] = uigetfile('*.db', 'Select Gripper Transition Experiment SQLite DB');
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

    metaTbl    = fetch(conn, 'SELECT COALESCE(key, '''') AS key, COALESCE(CAST(value AS TEXT), '''') AS value FROM experiment_meta');
    summaryTbl = fetch(conn, 'SELECT COALESCE(key, '''') AS key, COALESCE(CAST(value AS TEXT), ''NaN'') AS value FROM transition_summary');
    colCntTbl  = fetch(conn, 'SELECT COUNT(*) AS cnt FROM pragma_table_info(''samples'') WHERE name IN (''step_x_mm'', ''step_y_mm'', ''step_z_mm'')');
    if istable(colCntTbl)
        hasStepCols = double(colCntTbl.cnt(1)) == 3;
    else
        hasStepCols = double(colCntTbl{1, 1}) == 3;
    end
    stepSelect = '';
    if hasStepCols
        stepSelect = [ ...
            'COALESCE(CAST(step_x_mm AS TEXT), ''NaN'') AS step_x_mm, ', ...
            'COALESCE(CAST(step_y_mm AS TEXT), ''NaN'') AS step_y_mm, ', ...
            'COALESCE(CAST(step_z_mm AS TEXT), ''NaN'') AS step_z_mm, '];
    end
    samplesTbl = fetch(conn, [ ...
        'SELECT ', ...
        'COALESCE(CAST(t_s AS TEXT), ''NaN'') AS t_s, ', ...
        'COALESCE(CAST(epoch_s AS TEXT), ''NaN'') AS epoch_s, ', ...
        'COALESCE(phase, '''') AS phase, ', ...
        'COALESCE(CAST(phase_t_s AS TEXT), ''NaN'') AS phase_t_s, ', ...
        'COALESCE(CAST(target_pos AS TEXT), ''NaN'') AS target_pos, ', ...
        'COALESCE(CAST(servo_pos AS TEXT), ''NaN'') AS servo_pos, ', ...
        'COALESCE(CAST(servo_load_pct AS TEXT), ''NaN'') AS servo_load_pct, ', ...
        'COALESCE(CAST(servo_volt_v AS TEXT), ''NaN'') AS servo_volt_v, ', ...
        'COALESCE(CAST(adc0_mv AS TEXT), ''NaN'') AS adc0_mv, ', ...
        'COALESCE(CAST(adc1_mv AS TEXT), ''NaN'') AS adc1_mv, ', ...
        'COALESCE(CAST(adc2_mv AS TEXT), ''NaN'') AS adc2_mv, ', ...
        'COALESCE(CAST(adc3_mv AS TEXT), ''NaN'') AS adc3_mv, ', ...
        'COALESCE(CAST(adc4_mv AS TEXT), ''NaN'') AS adc4_mv, ', ...
        'COALESCE(CAST(L_Fx_N AS TEXT), ''NaN'') AS L_Fx_N, ', ...
        'COALESCE(CAST(L_Fy_N AS TEXT), ''NaN'') AS L_Fy_N, ', ...
        'COALESCE(CAST(L_Fz_N AS TEXT), ''NaN'') AS L_Fz_N, ', ...
        'COALESCE(CAST(L_Fmag_N AS TEXT), ''NaN'') AS L_Fmag_N, ', ...
        'COALESCE(CAST(L_Fshear_N AS TEXT), ''NaN'') AS L_Fshear_N, ', ...
        'COALESCE(CAST(L_mu AS TEXT), ''NaN'') AS L_mu, ', ...
        'COALESCE(CAST(R_Fx_N AS TEXT), ''NaN'') AS R_Fx_N, ', ...
        'COALESCE(CAST(R_Fy_N AS TEXT), ''NaN'') AS R_Fy_N, ', ...
        'COALESCE(CAST(R_Fz_N AS TEXT), ''NaN'') AS R_Fz_N, ', ...
        'COALESCE(CAST(R_Fmag_N AS TEXT), ''NaN'') AS R_Fmag_N, ', ...
        'COALESCE(CAST(R_Fshear_N AS TEXT), ''NaN'') AS R_Fshear_N, ', ...
        stepSelect, ...
        'COALESCE(CAST(R_mu AS TEXT), ''NaN'') AS R_mu ', ...
        'FROM samples ORDER BY id ASC']);

    if isempty(samplesTbl)
        error('No rows found in samples table of %s', dbPath);
    end

    % Metadata helpers
    [mKeys, mVals] = localExtractKV(metaTbl);
    getMeta = @(k, defVal) localGetStr(mKeys, mVals, k, defVal);

    [sKeys, sVals] = localExtractNumericKV(summaryTbl);
    getSum = @(k) localGetNum(sKeys, sVals, k, NaN);

    % Extract columns (compatible with both table and cell outputs)
    t_s      = localColDouble(samplesTbl, 't_s', 1);
    phase    = localColString(samplesTbl, 'phase', 3);
    tgtPos   = localColDouble(samplesTbl, 'target_pos', 5);
    srvPos   = localColDouble(samplesTbl, 'servo_pos', 6);
    srvLoad  = localColDouble(samplesTbl, 'servo_load_pct', 7);

    adcMv = zeros(numel(t_s), 5);
    for c = 0:4
        adcMv(:, c + 1) = localColDouble(samplesTbl, sprintf('adc%d_mv', c), 9 + c);
    end

    lFz     = localColDouble(samplesTbl, 'L_Fz_N', 16);
    lFmag   = localColDouble(samplesTbl, 'L_Fmag_N', 17);
    lFshear = localColDouble(samplesTbl, 'L_Fshear_N', 18);
    lMu     = localColDouble(samplesTbl, 'L_mu', 19);

    rFz     = localColDouble(samplesTbl, 'R_Fz_N', 22);
    rFmag   = localColDouble(samplesTbl, 'R_Fmag_N', 23);
    rFshear = localColDouble(samplesTbl, 'R_Fshear_N', 24);
    if hasStepCols
        stepX = localColDouble(samplesTbl, 'step_x_mm', 25);
        stepY = localColDouble(samplesTbl, 'step_y_mm', 26);
        stepZ = localColDouble(samplesTbl, 'step_z_mm', 27);
        rMu   = localColDouble(samplesTbl, 'R_mu', 28);
        hasStepperMotion = any(~isnan(stepX) | ~isnan(stepY) | ~isnan(stepZ));
    else
        stepX = nan(size(t_s));
        stepY = nan(size(t_s));
        stepZ = nan(size(t_s));
        rMu   = localColDouble(samplesTbl, 'R_mu', 25);
        hasStepperMotion = false;
    end

    % Read summary onset markers
    lOnsetS   = getSum("L_onset_s");
    lOnsetPos = getSum("L_onset_pos");
    rOnsetS   = getSum("R_onset_s");
    rOnsetPos = getSum("R_onset_pos");
    vOnsetS   = getSum("valve_onset_s");
    vOnsetPos = getSum("valve_onset_pos");
    vBaseMv   = getSum("valve_baseline_mv");
    vThreshMv = str2double(getMeta("valve_threshold_mv", "25"));

    % Print summary to Command Window
    fprintf('\n=== Gripper Transition Experiment Summary ===\n');
    fprintf('File       : %s\n', dbPath);
    fprintf('Samples    : %d (Fs target = %s Hz, Speed = %s steps/s, Open=%s -> Close=%s)\n', ...
        numel(t_s), getMeta("sample_rate_hz", "?"), getMeta("speed_steps_s", "?"), ...
        getMeta("open_pos", "?"), getMeta("close_pos", "?"));
    fprintf('L onset    : %.3f s @ pos %.0f\n', lOnsetS, lOnsetPos);
    fprintf('R onset    : %.3f s @ pos %.0f\n', rOnsetS, rOnsetPos);
    fprintf('Valve onset: %.3f s @ pos %.0f (baseline %.1f mV)\n', vOnsetS, vOnsetPos, vBaseMv);
    firstForceOnset = min([lOnsetS, rOnsetS], [], 'omitnan');
    if ~isnan(firstForceOnset) && ~isnan(vOnsetS)
        fprintf('Valve lag  : %.1f ms after first contact\n', (vOnsetS - firstForceOnset) * 1000.0);
    end
    fprintf('=============================================\n\n');

    %% Figure 1: Time-domain overview (4 synchronized subplots)
    figure('Name', 'Experiment 2 - Gripper Transition (Time Domain)', ...
        'Color', 'w', 'Position', [80, 60, 1180, 860]);

    % Find phase transition timestamps
    phaseChanges = find(phase(2:end) ~= phase(1:end-1)) + 1;

    nSub = 4 + double(hasStepperMotion);

    % 1) Servo Position & Load
    ax1 = subplot(nSub, 1, 1);
    yyaxis left;
    plot(t_s, srvPos, 'k-', 'LineWidth', 1.4, 'DisplayName', 'Servo Pos');
    hold on;
    plot(t_s, tgtPos, '--', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.0, 'DisplayName', 'Target Pos');
    ylabel('Servo Step');
    yyaxis right;
    plot(t_s, srvLoad, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.0, 'DisplayName', 'Load [%]');
    ylabel('Load [%]');
    grid on;
    for idx = phaseChanges.'
        xline(t_s(idx), ':', char(phase(idx)), 'Color', [0.5 0.5 0.5], ...
            'LabelVerticalAlignment', 'bottom', 'HandleVisibility', 'off');
    end
    localDrawOnsetLines(lOnsetS, rOnsetS, vOnsetS, false);
    title(sprintf('Exp 2: Gripper Closure Transition — %s', dbPath), 'Interpreter', 'none');
    legend('Location', 'best');

    % 2) PaXini Normal & Total Forces
    ax2 = subplot(nSub, 1, 2);
    plot(t_s, lFmag, 'b-', 'LineWidth', 1.4, 'DisplayName', 'Left |F| [N]');
    hold on;
    plot(t_s, lFz, 'b--', 'LineWidth', 1.0, 'DisplayName', 'Left Fz [N]');
    plot(t_s, rFmag, 'Color', [0.13 0.55 0.13], 'LineWidth', 1.4, 'DisplayName', 'Right |F| [N]');
    plot(t_s, rFz, '--', 'Color', [0.13 0.55 0.13], 'LineWidth', 1.0, 'DisplayName', 'Right Fz [N]');
    localDrawOnsetLines(lOnsetS, rOnsetS, vOnsetS, true);
    grid on;
    ylabel('Force [N]');
    legend('Location', 'best');

    % 3) Valve ADC Channels (CH0..CH4)
    ax3 = subplot(nSub, 1, 3);
    valveColors = [
        0.86 0.08 0.24;  % CH0 (A)
        1.00 0.55 0.00;  % CH1 (B)
        0.18 0.55 0.34;  % CH2 (C)
        0.25 0.41 0.88;  % CH3 (D)
        0.50 0.00 0.50;  % CH4 (E)
    ];
    valveLabels = {'CH0 (Valve A)', 'CH1 (Valve B)', 'CH2 (Valve C)', 'CH3 (Valve D)', 'CH4 (Valve E)'};
    hold on;
    for c = 1:5
        plot(t_s, adcMv(:, c), 'Color', valveColors(c, :), 'LineWidth', 1.3, 'DisplayName', valveLabels{c});
    end
    if ~isnan(vBaseMv) && ~isnan(vThreshMv)
        yline(vBaseMv + vThreshMv, '-.m', sprintf('Valve Thresh (%.0f mV)', vBaseMv + vThreshMv), ...
            'LineWidth', 1.0, 'HandleVisibility', 'off');
    end
    localDrawOnsetLines(lOnsetS, rOnsetS, vOnsetS, false);
    grid on;
    ylabel('Valve ADC [mV]');
    legend('Location', 'best');

    % 4) Shear Force & Friction Coefficient mu
    ax4 = subplot(nSub, 1, 4);
    yyaxis left;
    plot(t_s, lFshear, 'b-', 'LineWidth', 1.2, 'DisplayName', 'Left Fshear [N]');
    hold on;
    plot(t_s, rFshear, '-', 'Color', [0.13 0.55 0.13], 'LineWidth', 1.2, 'DisplayName', 'Right Fshear [N]');
    ylabel('Shear |Ft| [N]');
    yyaxis right;
    plot(t_s, lMu, 'c:', 'LineWidth', 1.2, 'DisplayName', 'Left \mu');
    plot(t_s, rMu, 'm:', 'LineWidth', 1.2, 'DisplayName', 'Right \mu');
    ylabel('Friction \mu');
    localDrawOnsetLines(lOnsetS, rOnsetS, vOnsetS, false);
    grid on;
    legend('Location', 'best');

    if hasStepperMotion
        ax5 = subplot(nSub, 1, 5);
        plot(t_s, stepX, 'r-', 'LineWidth', 1.3, 'DisplayName', 'Stepper X [mm]');
        hold on;
        plot(t_s, stepY, 'g-', 'LineWidth', 1.3, 'DisplayName', 'Stepper Y [mm]');
        plot(t_s, stepZ, 'b-', 'LineWidth', 1.3, 'DisplayName', 'Stepper Z [mm]');
        grid on;
        ylabel('Stepper [mm]');
        xlabel('Time [s]');
        legend('Location', 'best');
        linkaxes([ax1, ax2, ax3, ax4, ax5], 'x');
    else
        xlabel('Time [s]');
        linkaxes([ax1, ax2, ax3, ax4], 'x');
    end

    %% Figure 2: Transition vs Servo Position (Closing vs Releasing)
    figure('Name', 'Experiment 2 - Force & Valve vs Servo Position', ...
        'Color', 'w', 'Position', [140, 100, 1080, 680]);

    isClose = (phase == "Closing") | (phase == "Holding");
    isRel   = (phase == "Releasing");

    subplot(2, 1, 1);
    hold on;
    if any(isClose)
        plot(srvPos(isClose), lFmag(isClose), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Left |F| (Close/Hold)');
        plot(srvPos(isClose), rFmag(isClose), '-', 'Color', [0.13 0.55 0.13], 'LineWidth', 1.5, 'DisplayName', 'Right |F| (Close/Hold)');
    end
    if any(isRel)
        plot(srvPos(isRel), lFmag(isRel), 'b--', 'LineWidth', 1.1, 'DisplayName', 'Left |F| (Release)');
        plot(srvPos(isRel), rFmag(isRel), '--', 'Color', [0.13 0.55 0.13], 'LineWidth', 1.1, 'DisplayName', 'Right |F| (Release)');
    end
    if ~any(isClose) && ~any(isRel)
        plot(srvPos, lFmag, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Left |F|');
        plot(srvPos, rFmag, '-', 'Color', [0.13 0.55 0.13], 'LineWidth', 1.5, 'DisplayName', 'Right |F|');
    end
    if ~isnan(lOnsetPos) && lOnsetPos > 0
        xline(lOnsetPos, '--b', sprintf('L onset @ %d', round(lOnsetPos)), 'HandleVisibility', 'off');
    end
    if ~isnan(rOnsetPos) && rOnsetPos > 0
        xline(rOnsetPos, '--g', sprintf('R onset @ %d', round(rOnsetPos)), 'HandleVisibility', 'off');
    end
    grid on;
    ylabel('Force |F| [N]');
    title('Force & Valve Response vs Gripper Servo Position');
    legend('Location', 'best');

    subplot(2, 1, 2);
    hold on;
    for c = 1:5
        if any(isClose)
            plot(srvPos(isClose), adcMv(isClose, c), 'Color', valveColors(c, :), ...
                'LineWidth', 1.4, 'DisplayName', valveLabels{c});
        else
            plot(srvPos, adcMv(:, c), 'Color', valveColors(c, :), ...
                'LineWidth', 1.4, 'DisplayName', valveLabels{c});
        end
    end
    if ~isnan(vOnsetPos) && vOnsetPos > 0
        xline(vOnsetPos, '--m', sprintf('Valve onset @ %d', round(vOnsetPos)), 'HandleVisibility', 'off');
    end
    grid on;
    xlabel('Servo Position [step]');
    ylabel('Valve ADC [mV]');
    legend('Location', 'best');
end

function localDrawOnsetLines(lS, rS, vS, showLabels)
    if ~isnan(lS)
        if showLabels
            xline(lS, '--b', sprintf('L %.2fs', lS), 'LineWidth', 1.1, 'HandleVisibility', 'off');
        else
            xline(lS, '--b', 'LineWidth', 1.0, 'HandleVisibility', 'off');
        end
    end
    if ~isnan(rS)
        if showLabels
            xline(rS, '--', sprintf('R %.2fs', rS), 'Color', [0.13 0.55 0.13], ...
                'LineWidth', 1.1, 'HandleVisibility', 'off');
        else
            xline(rS, '--', 'Color', [0.13 0.55 0.13], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        end
    end
    if ~isnan(vS)
        if showLabels
            xline(vS, '--m', sprintf('Valve %.2fs', vS), 'LineWidth', 1.1, 'HandleVisibility', 'off');
        else
            xline(vS, '--m', 'LineWidth', 1.0, 'HandleVisibility', 'off');
        end
    end
end

function [keys, vals] = localExtractKV(tbl)
    if isempty(tbl)
        keys = strings(0, 1);
        vals = strings(0, 1);
        return;
    end
    if istable(tbl)
        keys = string(tbl.key);
        vals = string(tbl.value);
    else
        keys = string(tbl(:, 1));
        vals = string(tbl(:, 2));
    end
end

function [keys, vals] = localExtractNumericKV(tbl)
    if isempty(tbl)
        keys = strings(0, 1);
        vals = zeros(0, 1);
        return;
    end
    if istable(tbl)
        keys = string(tbl.key);
        vals = localCellOrArrayToDouble(tbl.value);
    else
        keys = string(tbl(:, 1));
        vals = localCellOrArrayToDouble(tbl(:, 2));
    end
end

function val = localGetStr(keys, vals, keyName, defaultVal)
    hit = find(keys == keyName, 1);
    if isempty(hit)
        val = string(defaultVal);
    else
        val = vals(hit);
    end
end

function val = localGetNum(keys, vals, keyName, defaultVal)
    hit = find(keys == keyName, 1);
    if isempty(hit)
        val = defaultVal;
    else
        val = vals(hit);
    end
end

function col = localColDouble(tbl, colName, colIdx)
    if istable(tbl)
        raw = tbl.(colName);
    else
        raw = tbl(:, colIdx);
    end
    col = localCellOrArrayToDouble(raw);
end

function out = localCellOrArrayToDouble(raw)
    if isnumeric(raw)
        out = double(raw);
        return;
    end
    if isstring(raw) || iscategorical(raw)
        out = str2double(string(raw));
        return;
    end
    if iscell(raw)
        n = numel(raw);
        out = nan(n, 1);
        for i = 1:n
            v = raw{i};
            if isnumeric(v) && isscalar(v)
                out(i) = double(v);
            elseif (ischar(v) || isstring(v)) && ~isempty(v)
                out(i) = str2double(v);
            end
        end
        return;
    end
    out = double(raw);
end

function col = localColString(tbl, colName, colIdx)
    if istable(tbl)
        col = string(tbl.(colName));
    else
        col = string(tbl(:, colIdx));
    end
end
