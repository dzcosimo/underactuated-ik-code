function save_figure(f, outdir, name)
%SAVE_FIGURE  Save figure F as <outdir>/<name>.png (150 dpi) and .eps (vector), then close it.
if ~exist(outdir, 'dir'), mkdir(outdir); end
ax = findobj(f, 'Type', 'axes');
for k = 1:numel(ax), ax(k).Toolbar.Visible = 'off'; end   % keep the axes toolbar out of the export
exportgraphics(f, fullfile(outdir, [name '.png']), 'Resolution', 150);
exportgraphics(f, fullfile(outdir, [name '.eps']), 'ContentType', 'vector');
close(f);
end
