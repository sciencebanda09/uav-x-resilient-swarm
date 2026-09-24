function uav_x_3d_replay(logPath, outputPath)
%UAV_X_3D_REPLAY MATLAB-native 3D replay with UAVs, terrain and mission overlays.
if nargin < 2, outputPath = ''; end
records = load_log(logPath);
ticks = records(cellfun(@(r) strcmp(r.record_type, 'tick'), records));
if isempty(ticks), error('UAV-X log contains no tick records.'); end
root = fileparts(fileparts(mfilename('fullpath')));
% Headless batch rendering must not depend on the browser/WebGL figure
% backend.  The software OpenGL/painters combination is also reproducible
% across MATLAB desktop and CI environments.
try
    opengl('software');
catch
end
set(groot,'defaultFigureRenderer','opengl');
terrain = jsondecode(fileread(fullfile(root, 'viewer', 'public', 'scene', 'terrain_heightmap.json')));
stride = max(1, floor(double(terrain.grid) / 45));
terrainZ = double(terrain.elevation_m(1:stride:end, 1:stride:end));
arena = double(terrain.arena_m); axisM = linspace(0, arena, size(terrainZ, 1));
[X,Y] = meshgrid(axisM, axisM);
fig = figure('Color',[.02 .04 .07], 'Name','UAV-X MATLAB 3D Replay', 'Position',[80 60 1400 860], 'Resize','off');
ax = axes('Parent',fig,'Color',[.02 .04 .07]); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal'); view(ax,3);
xlabel(ax,'East (m)'); ylabel(ax,'North (m)'); zlabel(ax,'Altitude (m)'); colormap(ax,parula(128));
surf(ax,X,Y,terrainZ,'EdgeColor','none','FaceAlpha',.62); camlight(ax,'headlight'); lighting(ax,'gouraud');
xlim(ax,[0 arena]); ylim(ax,[0 arena]); zlim(ax,[0 max(160,max(terrainZ(:))+45)]); axis(ax,'vis3d');
set(ax,'XLimMode','manual','YLimMode','manual','ZLimMode','manual');
plot3(ax,[0 arena arena 0 0],[0 0 arena arena 0],max(terrainZ(:))+4+zeros(1,5),'--','Color',[1 .8 .1]);
draw_obstacles(ax,fullfile(root,'scene','obstacles.json'),axisM,terrainZ);
gcs = [40 40 sample_ground(40,40,axisM,terrainZ)+20];
plot3(ax,gcs(1),gcs(2),gcs(3),'s','MarkerSize',12,'MarkerFaceColor',[0 .8 1],'MarkerEdgeColor','k');
text(ax,gcs(1),gcs(2),gcs(3)+8,'GCS | command + data sink','Color',[0 .9 1],'FontWeight','bold','BackgroundColor','k');
info = annotation(fig,'textbox',[.72 .12 .26 .70],'String','UAV-X MISSION STATUS','Color','w','BackgroundColor',[0 0 0],'EdgeColor',[0 .75 1],'FontName','Consolas','FontSize',9,'VerticalAlignment','top','FitBoxToText','off');
% Reuse HUD text objects. Creating new text objects every frame makes the
% labels accumulate and appear as overlapping words in the exported replay.
hudTop = text(ax,.015,.96,'','Units','normalized','Color','w','FontWeight','bold','BackgroundColor','k');
hudBottom = text(ax,.015,.025,'','Units','normalized','Color','w','FontSize',9,'BackgroundColor','k');
dynamic = hgtransform('Parent',ax); writer = open_writer(outputPath);
cleanup = onCleanup(@() close_writer(writer)); %#ok<NASGU>
for k=1:numel(ticks)
    if ~ishandle(fig) || ~isgraphics(ax,'axes'), break; end
    children = get(dynamic,'Children');
    if ~isempty(children), delete(children); end
    tick=ticks{k}; draw_tick(dynamic,ax,tick,axisM,terrainZ,gcs);
    mapped = number_field(tick,'survey_coverage_fraction',0)*100;
    surveyed = sum(arrayfun(@(p) strcmp(char(p.status),'SURVEYED'),tick.pois));
    connected = sum(arrayfun(@(u) logical(u.gcs_reachable)&&~logical(u.failed),tick.uavs));
    info.String = mission_panel(tick,connected,surveyed,mapped);
    title(ax,sprintf('UAV-X RESILIENT BVLOS SWARM | t = %.1f s',tick.time_s),'Color','w','FontWeight','bold');
    hudTop.String = sprintf('Connected %d/%d | Surveyed %d/%d | Mapped %.1f%%',connected,numel(tick.uavs),surveyed,numel(tick.pois),mapped);
    hudBottom.String = 'GREEN dashed = survey assignment | CYAN = relay/data | GREEN ring = camera footprint';
    drawnow;
    if strcmp(writer.kind,'gif') || strcmp(writer.kind,'video'), writer = append_frame(writer,fig,k); elseif strcmp(writer.kind,'png') && k==1, save_png(fig,writer.path); end
end
end

function draw_tick(parent,ax,tick,axisM,terrainZ,gcs)
colors=struct('SURVEY',[.05 1 .35],'RELAY',[0 .75 1],'RECOVER',[1 .55 0],'RETURN',[1 .1 .3],'CHARGE',[.8 .25 1],'STANDBY',[.75 .8 .85]);
uavMap=struct(); for i=1:numel(tick.uavs), uavMap.(matlab.lang.makeValidName(char(tick.uavs(i).id)))=tick.uavs(i); end
for i=1:numel(tick.links)
    link=tick.links(i); if ~logical(link.available), continue; end
    a=link_point(char(link.source_id),uavMap,gcs); b=link_point(char(link.target_id),uavMap,gcs);
    if ~isempty(a)&&~isempty(b), plot3(ax,[a(1) b(1)],[a(2) b(2)],[a(3) b(3)],'-','Color',[0 .75 1],'LineWidth',1.2,'Parent',parent); end
end
for i=1:numel(tick.pois)
    p=tick.pois(i); pos=double(p.position_m); c=[.98 .72 .05];
    if strcmp(char(p.status),'SURVEYED'), c=[.05 1 .35]; end; if double(p.priority)==1, c=[1 .1 .3]; end
    plot3(ax,pos(1),pos(2),pos(3)+3,'*','Color',c,'MarkerSize',10,'LineWidth',1.5,'Parent',parent);
end
for i=1:numel(tick.uavs)
    u=tick.uavs(i); pos=double(u.position_m); role=char(u.role); c=[1 1 1]; if isfield(colors,role), c=colors.(role); end
    draw_drone(parent,pos,double(u.attitude_rpy_rad),c,char(u.id));
    if isfield(u,'task_id') && ~isempty(u.task_id)
        p=find_poi(tick.pois,char(u.task_id)); if ~isempty(p), q=double(p.position_m); plot3(ax,[pos(1) q(1)],[pos(2) q(2)],[pos(3) q(3)+4],'--','Color',[.1 1 .35],'LineWidth',1.5,'Parent',parent); end
    end
    if isfield(u,'survey_capture_active') && logical(u.survey_capture_active)
        r=double(u.survey_footprint_radius_m); th=linspace(0,2*pi,48); plot3(ax,pos(1)+r*cos(th),pos(2)+r*sin(th),pos(3)+zeros(size(th)),'-','Color',[.1 1 .35],'LineWidth',1.2,'Parent',parent);
    end
end
end

function draw_drone(parent,pos,rpy,color,label)
T=hgtransform('Parent',parent); set(T,'Matrix',makehgtform('translate',pos)*makehgtform('zrotate',rpy(3))*makehgtform('yrotate',rpy(2))*makehgtform('xrotate',rpy(1)));
v=[-3 -2 -.6;3 -2 -.6;3 2 -.6;-3 2 -.6;-3 -2 .6;3 -2 .6;3 2 .6;-3 2 .6]; f=[1 2 3 4;5 8 7 6;1 5 6 2;2 6 7 3;3 7 8 4;4 8 5 1];
patch('Parent',T,'Vertices',v,'Faces',f,'FaceColor',color,'EdgeColor','k');
for i=0:5
    a=i*pi/3; x=5.3*cos(a); y=5.3*sin(a); line('Parent',T,'XData',[0 x],'YData',[0 y],'ZData',[0 0],'Color',[.08 .1 .14],'LineWidth',2.5);
    th=linspace(0,2*pi,24); line('Parent',T,'XData',x+1.8*cos(th),'YData',y+1.8*sin(th),'ZData',.7+zeros(size(th)),'Color',[.8 .95 1],'LineWidth',1);
end
end

function textValue = mission_panel(tick,connected,surveyed,mapped)
lines={sprintf('UAV-X MISSION STATUS'),sprintf('Connected: %d/%d',connected,numel(tick.uavs)),sprintf('Surveyed:  %d/%d',surveyed,numel(tick.pois)),sprintf('Mapped:    %.1f%%',mapped),'','UAV ROLE / TASK / BATTERY'};
for i=1:numel(tick.uavs)
    u=tick.uavs(i); task='--'; if isfield(u,'task_id') && ~isempty(u.task_id), task=char(u.task_id); end
    if numel(task)>12, task=task(1:12); end
    state=''; if logical(u.failed), state=' FAILED'; elseif isfield(u,'survey_capture_active') && logical(u.survey_capture_active), state=' CAPTURE'; end
    lines{end+1}=sprintf('%-6s %-8s %-12s %5.1f%%%s',char(u.id),char(u.role),task,double(u.battery_pct),state); %#ok<AGROW>
end
lines{end+1}=''; lines{end+1}='PoI colors:'; lines{end+1}='  green = surveyed'; lines{end+1}='  yellow = pending'; lines{end+1}='  red = emergency';
textValue=strjoin(lines,newline);
end

function draw_obstacles(ax,path,axisM,terrainZ)
if ~isfile(path), return; end; data=jsondecode(fileread(path));
for i=1:numel(data.obstacles)
    o=data.obstacles(i); c=double(o.center_m); s=double(o.size_m); base=sample_ground(c(1),c(2),axisM,terrainZ); color=[.55 .58 .58];
    if strcmp(char(o.kind),'tower'), color=[.25 .5 .65]; elseif strcmp(char(o.kind),'rubble'), color=[.45 .32 .22]; end
    x=c(1)+[-1 1 1 -1 -1 1 1 -1]*s(1)/2; y=c(2)+[-1 -1 1 1 -1 -1 1 1]*s(2)/2; z=base+[0 0 0 0 1 1 1 1]*s(3);
    patch(ax,'Vertices',[x(:) y(:) z(:)],'Faces',[1 2 3 4;5 8 7 6;1 5 6 2;2 6 7 3;3 7 8 4;4 8 5 1],'FaceColor',color,'FaceAlpha',.72,'EdgeColor',[.1 .1 .1]);
end
end

function q=link_point(id,uavMap,gcs)
if strcmp(id,'GCS'), q=gcs; else, key=matlab.lang.makeValidName(id); if isfield(uavMap,key), q=double(uavMap.(key).position_m); else, q=[]; end, end
end
function p=find_poi(pois,id), p=[]; for i=1:numel(pois), if strcmp(char(pois(i).id),id), p=pois(i); return; end, end, end
function v=sample_ground(x,y,axisM,terrainZ), v=interp2(axisM,axisM,terrainZ,min(max(x,axisM(1)),axisM(end)),min(max(y,axisM(1)),axisM(end)),'linear'); end
function v=number_field(s,n,d), if isfield(s,n), v=double(s.(n)); else, v=d; end, end
function w=open_writer(path), w.kind='none'; w.path=path; w.obj=[]; w.frameSize=[]; if isempty(path), return; end; [~,~,e]=fileparts(path); e=lower(e); if strcmp(e,'.gif'), w.kind='gif'; elseif strcmp(e,'.mp4'), w.kind='video'; w.obj=VideoWriter(path,'MPEG-4'); w.obj.FrameRate=10; open(w.obj); elseif strcmp(e,'.png'), w.kind='png'; else, error('Output must be .gif, .mp4, or .png.'); end, end
function w=append_frame(w,fig,k)
if ~isgraphics(fig,'figure'), return; end
drawnow;
% getframe relies on the interactive graphics/WebGL capture backend in newer
% MATLAB releases and can fail in -batch mode.  PRINT uses the stable raster
% export path and works for both desktop and headless rendering.
pngPath=[tempname '.png'];
print(fig,pngPath,'-dpng','-r100');
frame.cdata=imread(pngPath);
frame.colormap=[];
if isfile(pngPath), delete(pngPath); end
if isempty(w.frameSize)
    sourceSize=size(frame.cdata);
    w.frameSize=[max(2,2*floor(sourceSize(1)/2)), max(2,2*floor(sourceSize(2)/2))];
end
frame.cdata=fit_frame(frame.cdata,w.frameSize);
if strcmp(w.kind,'video'), writeVideo(w.obj,frame); return; end
[im,map]=rgb2ind(frame2im(frame),256);
if k==1, imwrite(im,map,w.path,'gif','LoopCount',inf,'DelayTime',.1); else, imwrite(im,map,w.path,'gif','WriteMode','append','DelayTime',.1); end
end
function cdata=fit_frame(cdata,targetSize)
if ndims(cdata)==2, cdata=repmat(cdata,1,1,3); end
source=uint8(cdata); cdata=zeros(targetSize(1),targetSize(2),3,'uint8');
h=min(size(source,1),targetSize(1)); w=min(size(source,2),targetSize(2)); cdata(1:h,1:w,:)=source(1:h,1:w,1:3);
end
function save_png(fig,path)
drawnow;
% PRINT is more compatible than EXPORTGRAPHICS across MATLAB releases and
% works with figures containing annotations and 3-D axes.
print(fig,path,'-dpng','-r150');
end
function close_writer(w), if strcmp(w.kind,'video') && ~isempty(w.obj), close(w.obj); end, end
