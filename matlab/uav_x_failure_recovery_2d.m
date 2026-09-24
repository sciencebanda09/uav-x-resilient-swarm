function uav_x_failure_recovery_2d(logPath, outputPath)
%UAV_X_FAILURE_RECOVERY_2D MATLAB-native cinematic Stage 1 replay.
% Uses the canonical JSONL log, software-safe 2D painters rendering, and
% VideoWriter MPEG-4 output so it works in MATLAB batch mode without WebGL.
if nargin < 2, outputPath = 'artifacts/stage1_failure_recovery_matlab.mp4'; end
records = load_log(logPath);
ticks = records(cellfun(@(r) strcmp(r.record_type, 'tick'), records));
events = records(cellfun(@(r) strcmp(r.record_type, 'event'), records));
if isempty(ticks), error('UAV-X log contains no tick records.'); end
root = fileparts(fileparts(mfilename('fullpath')));
fig = figure('Visible','off','Color',[.025 .04 .065], 'Renderer','painters', ...
             'Position',[80 60 1600 900], 'Resize','off');
writer = VideoWriter(outputPath,'MPEG-4'); writer.FrameRate=10; open(writer);
cleanup = onCleanup(@() close(writer)); %#ok<NASGU>
obstacles = load_obstacles(fullfile(root,'scene','obstacles.json'));
trailX = cell(1,numel(ticks{1}.uavs)); trailY = trailX;
times = cellfun(@(r) double(r.time_s), ticks);
completion = zeros(size(times)); availability = zeros(size(times));
for k=1:numel(ticks)
    tick=ticks{k};
    for u=1:numel(tick.uavs)
        if numel(trailX{u}) > 35
            trailX{u}=trailX{u}(end-34:end); trailY{u}=trailY{u}(end-34:end);
        end
        trailX{u}(end+1)=double(tick.uavs(u).position_m(1));
        trailY{u}(end+1)=double(tick.uavs(u).position_m(2));
    end
    surveyed=sum(arrayfun(@(p) strcmp(char(p.status),'SURVEYED'),tick.pois));
    active=arrayfun(@(u) ~logical(u.failed),tick.uavs);
    connected=sum(arrayfun(@(u) logical(u.gcs_reachable) && ~logical(u.failed),tick.uavs));
    completion(k)=surveyed/max(1,numel(tick.pois)); availability(k)=connected/max(1,sum(active));
    clf(fig);
    mapAx=axes('Parent',fig,'Position',[.045 .16 .57 .75]); hold(mapAx,'on');
    draw_map(mapAx,tick,obstacles,trailX,trailY,k);
    title(mapAx,sprintf('UAV-X  |  %s  |  t = %.0f s',phase_name(tick,events),double(tick.time_s)), ...
          'Color','w','FontWeight','bold','FontSize',15,'HorizontalAlignment','left');
    xlabel(mapAx,'East (m)'); ylabel(mapAx,'North (m)');
    statusAx=axes('Parent',fig,'Position',[.655 .52 .30 .39]); axis(statusAx,'off');
    draw_status(statusAx,tick,connected,surveyed);
    metricsAx=axes('Parent',fig,'Position',[.655 .16 .30 .27]); hold(metricsAx,'on');
    draw_metrics(metricsAx,tick);
    timelineAx=axes('Parent',fig,'Position',[.045 .055 .91 .055]); hold(timelineAx,'on');
    draw_timeline(timelineAx,times,completion,availability,events,double(tick.time_s));
    annotation(fig,'textbox',[.05 .925 .9 .045],'String',incident_label(tick,events), ...
               'Color',[1 .78 .25],'EdgeColor','none','HorizontalAlignment','center', ...
               'FontName','Helvetica','FontWeight','bold','FontSize',12);
    annotation(fig,'textbox',[.655 .925 .30 .04],'String','MISSION CONTROL  /  CANONICAL LOG', ...
               'Color',[0 .82 1],'EdgeColor','none','FontWeight','bold','FontSize',11);
    drawnow;
    frame=print_frame(fig); writeVideo(writer,frame);
end
close(fig);
end

function frame=print_frame(fig)
pngPath=[tempname '.png']; print(fig,pngPath,'-dpng','-r100');
frame.cdata=imread(pngPath); frame.colormap=[]; if isfile(pngPath), delete(pngPath); end
end

function draw_map(ax,tick,obstacles,trailX,trailY,k)
axis(ax,[0 500 0 500]); axis(ax,'equal'); grid(ax,'on'); set(ax,'Color',[.025 .04 .065], ...
    'XColor',[.65 .75 .85],'YColor',[.65 .75 .85],'GridColor',[.12 .2 .3]);
rectangle(ax,'Position',[2 2 496 496],'EdgeColor',[1 .78 .25],'LineStyle','--','LineWidth',1.2);
for i=1:numel(obstacles)
    o=obstacles(i); c=double(o.center_m); s=double(o.size_m); fc=[.35 .25 .2];
    if strcmp(char(o.kind),'tower'), fc=[.16 .35 .48]; end
    rectangle(ax,'Position',[c(1)-s(1)/2 c(2)-s(2)/2 s(1) s(2)],'FaceColor',fc,'EdgeColor',[.7 .75 .8],'FaceAlpha',.35);
end
scatter(ax,40,40,150,[0 .8 1],'s','filled','MarkerEdgeColor','w'); text(ax,50,48,'GCS','Color',[0 .85 1],'FontWeight','bold');
for i=1:numel(tick.links)
    l=tick.links(i); if ~logical(l.available), continue; end
    a=link_pos(l.source_id,tick.uavs,[40 40]); b=link_pos(l.target_id,tick.uavs,[40 40]);
    if ~isempty(a)&&~isempty(b), q=1-double(l.packet_loss); plot(ax,[a(1) b(1)],[a(2) b(2)],'-','Color',[0 .65 1],'LineWidth',.7+1.2*q); end
end
for i=1:numel(tick.pois)
    p=tick.pois(i); c=[.98 .72 .05]; if strcmp(char(p.status),'SURVEYED'), c=[.05 1 .35]; end; if double(p.priority)==1,c=[1 .1 .3];end
    scatter(ax,double(p.position_m(1)),double(p.position_m(2)),115,c,'*','LineWidth',1);
end
roles=struct('SURVEY',[.05 1 .35],'RELAY',[0 .75 1],'RECOVER',[1 .55 0],'RETURN',[1 .1 .3],'CHARGE',[.8 .25 1],'STANDBY',[.75 .8 .85]);
for i=1:numel(tick.uavs)
    u=tick.uavs(i); pos=double(u.position_m); c=[.42 .48 .55]; if ~logical(u.failed), role=char(u.role); if isfield(roles,role),c=roles.(role);end,end
    if numel(trailX{i})>1, plot(ax,trailX{i},trailY{i},'-','Color',.55*c+.45*[1 1 1],'LineWidth',1.2); end
    scatter(ax,pos(1),pos(2),125,c,'o','filled','MarkerEdgeColor','w');
    text(ax,pos(1)+5,pos(2)+5,strrep(char(u.id),'UAV-','U'),'Color',c,'FontSize',8,'FontWeight','bold');
    if isfield(u,'task_id') && ~isempty(u.task_id), p=find_poi(tick.pois,char(u.task_id)); if ~isempty(p), q=double(p.position_m); plot(ax,[pos(1) q(1)],[pos(2) q(2)],'--','Color',[.1 1 .35],'LineWidth',1);end,end
end
end

function draw_status(ax,tick,connected,surveyed)
text(ax,0,.98,sprintf('SURVEYED   %d / %d',surveyed,numel(tick.pois)),'Color',[.05 1 .35],'FontSize',18,'FontWeight','bold','VerticalAlignment','top');
text(ax,0,.82,sprintf('CONNECTED  %d / %d',connected,numel(tick.uavs)-sum(arrayfun(@(u)logical(u.failed),tick.uavs))),'Color',[0 .8 1],'FontSize',18,'FontWeight','bold','VerticalAlignment','top');
text(ax,0,.66,sprintf('MAPPED     %.1f%%',double(tick.survey_coverage_fraction)*100),'Color',[1 .78 .25],'FontSize',18,'FontWeight','bold','VerticalAlignment','top');
text(ax,0,.49,'UAV / ROLE / BATTERY','Color',[.75 .82 .9],'FontSize',10,'FontWeight','bold');
y=.40; for i=1:numel(tick.uavs)
    u=tick.uavs(i); c=[.8 .85 .9]; if ~logical(u.failed), c=[.1 .9 .45]; end
    task=''; if isfield(u,'task_id')&&~isempty(u.task_id),task=char(u.task_id);end
    text(ax,0,y,sprintf('%-5s %-8s %5.1f%% %s',strrep(char(u.id),'UAV-','U'),char(u.role),double(u.battery_pct),task),'Color',c,'FontName','Consolas','FontSize',9); y=y-.075;
end
axis(ax,[0 1 0 1]);
end

function draw_metrics(ax,tick)
labels=arrayfun(@(u)strrep(char(u.id),'UAV-','U'),tick.uavs,'UniformOutput',false); vals=arrayfun(@(u)double(u.battery_pct),tick.uavs);
barh(ax,vals,.72,'FaceColor',[.1 .65 .8]); set(ax,'YTick',1:numel(labels),'YTickLabel',labels,'YColor',[.7 .8 .9],'XColor',[.7 .8 .9],'Color',[.025 .04 .065]); xlim(ax,[0 100]); xlabel(ax,'Battery (%)','Color',[.7 .8 .9]); grid(ax,'on');
end

function draw_timeline(ax,times,completion,availability,events,current)
plot(ax,times,completion,'Color',[.05 1 .35],'LineWidth',2); plot(ax,times,availability,'Color',[0 .8 1],'LineWidth',1.5);
for i=1:numel(events)
    e=events{i}; if double(e.time_s)<=current && isfield(e,'event_type') && any(strcmp(char(e.event_type),{'UAV_FAILURE','LINK_OUTAGE','RETURN_HOME','EMERGENCY_POI'})), xline(ax,double(e.time_s),'Color',[1 .55 .1],'LineWidth',1);end
end
axis(ax,[0 max(times) 0 1.05]); set(ax,'Color',[.025 .04 .065],'XColor',[.65 .75 .85],'YColor',[.65 .75 .85]); grid(ax,'on');
end

function textValue=phase_name(tick,events)
textValue='NORMAL SURVEY'; t=double(tick.time_s);
for i=1:numel(events), e=events{i}; if double(e.time_s)<=t && isfield(e,'event_type')
    if strcmp(char(e.event_type),'UAV_FAILURE'), textValue='SWARM RECOVERY'; end
    if strcmp(char(e.event_type),'LINK_OUTAGE') && t<double(e.time_s)+15, textValue='COMMUNICATION OUTAGE'; end
end,end
end
function textValue=incident_label(tick,events)
textValue=''; t=double(tick.time_s); for i=1:numel(events), e=events{i}; if abs(double(e.time_s)-t)<.51 && isfield(e,'event_type') && any(strcmp(char(e.event_type),{'UAV_FAILURE','LINK_OUTAGE','RETURN_HOME','EMERGENCY_POI'})), textValue=strrep(char(e.event_type),'_',' '); return; end,end
end
function p=link_pos(id,uavs,gcs), p=[]; if strcmp(char(id),'GCS'),p=gcs;return;end; for i=1:numel(uavs),if strcmp(char(uavs(i).id),char(id)),p=double(uavs(i).position_m);return;end,end,end
function p=find_poi(pois,id), p=[]; for i=1:numel(pois),if strcmp(char(pois(i).id),id),p=pois(i);return;end,end,end
function data=load_obstacles(path), data=[]; if isfile(path), payload=jsondecode(fileread(path)); if isfield(payload,'obstacles'),data=payload.obstacles;end,end,end
