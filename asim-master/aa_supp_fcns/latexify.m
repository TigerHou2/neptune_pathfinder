function latexify
%LATEXIFY Summary of this function goes here
%   Detailed explanation goes here

set(findall(gcf,'-property','FontSize'),'FontSize',16)
set(findall(gcf,'-property','ticklabelinterpreter'),...
                            'ticklabelinterpreter','latex')
set(findall(gcf,'-property','interpreter'),'interpreter','latex')

set( gca, 'Color', [1 1 1] )

set(gcf, 'PaperPositionMode', 'manual')
set(gcf, 'Color', [1 1 1])
set(gcf, 'PaperUnits', 'centimeters')
set(gcf, 'PaperSize', [19 19])
set(gcf, 'Units', 'centimeters' )
set(gcf, 'Position', [0.2 1.2 19 19])
set(gcf, 'PaperPosition', [0.2 1.2 19 19])

end

