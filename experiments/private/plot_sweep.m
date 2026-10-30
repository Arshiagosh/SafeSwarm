function plot_sweep(ax, rows, xfield, yfield, kinds, ctrls)
    % plot_sweep(ax, rows, xfield, yfield, kinds, ctrls) — one line per dynamics/controller
    %   For yfield 'collision_rate' the 95 % interval is drawn as error bars.
    hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    styles = {'-o', '--s', '-^', '--d'};
    k = 0; names = {};
    for b = 1:numel(kinds)
        for c = 1:numel(ctrls)
            k = k + 1;
            sel = strcmp({rows.dynamics}, kinds{b}) & strcmp({rows.controller}, ctrls{c});
            x = [rows(sel).(xfield)];
            y = [rows(sel).(yfield)];
            if strcmp(yfield, 'collision_rate')
                errorbar(ax, x, y, y - [rows(sel).collision_rate_lo], ...
                         [rows(sel).collision_rate_hi] - y, styles{k}, 'LineWidth', 1.3);
            else
                plot(ax, x, y, styles{k}, 'LineWidth', 1.3);
            end
            names{end+1} = sprintf('%s, %s', upper(kinds{b}), strrep(ctrls{c}, '_', ' ')); %#ok<AGROW>
        end
    end
    legend(ax, names, 'Location', 'best', 'AutoUpdate', 'off');
end
