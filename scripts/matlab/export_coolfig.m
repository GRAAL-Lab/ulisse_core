function export_coolfig(filename, options, f, s_title, s_xlabel, s_ylabel)

set(gcf,'units','normalized','outerposition',[0 0 1 1]);
set(gca,'box','on');

% set axis and title labels
title(s_title);
xlabel(s_xlabel);
ylabel(s_ylabel);

% set font sizes
h_xlabel = get(gca,'XLabel');
set(h_xlabel,'FontSize', 40, 'Interpreter', 'latex'); 
h_ylabel = get(gca,'YLabel');
set(h_ylabel,'FontSize', 40, 'Interpreter', 'latex'); 
h_title = get(gca,'title');
set(h_title,'FontSize', 40, 'Interpreter', 'latex'); 
set(gca,'FontSize',30);
%h_legend = legend;
%set(h_legend,'FontSize', 30, 'Interpreter', 'latex'); 

% background color
set(f, 'Color', 'w');

% set line width
h_line = findobj(f, 'type', 'line');
set(h_line, 'LineWidth', 2);

% export
if (strcmp(options,'no') == 0)
    export_fig(filename, options);
end