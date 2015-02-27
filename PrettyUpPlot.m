function PrettyUpPlot
% This function will format a plot into a nice pretty format

% Figure Parameters
% set(gcf, 'paperunits', 'points' )
% set(gcf,'position',[100 100, 600 450]); %
% set(gcf,'papersize',[600 450]);
% set(gcf,'PaperPositionMode','Auto');

set(findall(gcf,'type','text'),'fontSize',26,'fontWeight','normal')
set(gca,'LooseInset',get(gca,'TightInset'))
set(gcf, 'Color', [1 1 1]);

% Add UI Control forS Copy
uicontrol('Style', 'pushbutton', 'String', 'Copy', 'Callback', 'print -dmeta -noui')

% Set Legend
% legend('-DynamicLegend')

% Set Interpreter to LaTeX
% set(0, 'defaultTextInterpreter', 'latex');


end

