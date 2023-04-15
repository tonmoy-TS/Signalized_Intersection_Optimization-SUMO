%% plottext — scratch script demonstrating MATLAB text annotation in subplots
%
%   NOT part of the ECO-AND simulation pipeline.
%   Kept as a reference snippet for adding text boxes and labels to figures.
%   Two approaches are shown:
%     subplot 1 — annotation() places a textbox using normalised figure coords
%     subplot 2 — text() places a string using normalised axes coords

variable1 = 12.5;
variable2 = 'Important message';
variable3 = 100;

subplot(2, 1, 1)
% annotation() coords are [left bottom width height] in normalised figure units (0–1).
annotation('textbox', [0.1 0.5 0.5 0.2], ...
    'String', sprintf('Variable 1: %.1d', variable3), ...
    'FontSize', 16, 'BackgroundColor', 'w');

subplot(2, 1, 2)
% text() coords are in normalised axes units when using 'Units','normalized'.
text(0.5, 0.9, sprintf('Subplot 2: %s', variable2), ...
    'HorizontalAlignment', 'center', ...
    'VerticalAlignment',   'middle', ...
    'FontSize', 14, 'Color', 'r');
