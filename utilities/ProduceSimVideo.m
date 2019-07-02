function ProduceSimVideo(data,ScalarFieldSelection,behavior,field_width,SimParams)
% <PRODUCESIMVIDEO> - Creates an AVI video of simulation results.
%
% Video plotting function created with many performance optimization
% methods to allow for plotting large numbers of robot positions and saving
% subsequent image frames quickly. Most performance benefits are due to the
% direct access of XData and YData properties within the pre-contstructed
% matlab line objects. 
%
% Syntax:  ProduceSimVideo(data)
%
% Inputs:
%    data - simulation output data structure (see sim_runner for definition)

% Outputs:
%    NA - NA

% Extract config data from struct
DesiredValue= SimParams.DESIRED_VALUE;
SensorRange=SimParams.SENSOR_RANGE;
% Option to skip saving the video
SAVE_VIDEO = SimParams.SaveVideo;

if SAVE_VIDEO
    video_filename = data(1).cfg.simulation_name;
    
    % If video file already exists, append datetime data to its filename
    if exist(fullfile(cd,'results','videos',filesep,[video_filename,'.avi'])) == 2
        video_filename=fullfile(cd,'results','videos',filesep,[video_filename,'_',datestr(now,'yyyy-mm-dd'),'_',datestr(now,'HH_MM_SS'),'.avi']);
    else
        video_filename=fullfile(cd,'results','videos',filesep,[video_filename,'.avi']);
    end
    
    % Setup videowriter
    v = VideoWriter(video_filename,'Motion JPEG AVI');
    v.FrameRate=60;
    v.Quality=70;
    open(v)
    F(numel(data(1).robot_ds)) = struct('cdata',[],'colormap',[]);
end
 

% First plot scalar field 
fig = figure('name','Simulation Output','Units','normalized','position',[.3 .2 .4 .6]);


for i = 1:numel(data)
    num_points(i) = numel(data(i).robot_ds(1).x);
    
end
max_points = max(num_points);

tStart = tic;
fprintf(1,'Constructing robot line objects... \n');

% Use tightly packed subplots
% [ha, pos] = tight_subplot(ceil(sqrt(numel(data))), ceil(sqrt(numel(data))), [.01 .03],[.01 .05],[.01 .01]);
%[ha, pos] = tight_subplot(ceil(sqrt(numel(data))), ceil(sqrt(numel(data))), [.01 .01],[.01 .05],[.01 .01]);
[ha, pos] = tight_subplot(ceil(sqrt(numel(data))), ceil(sqrt(numel(data))), [.03 .03]./ceil(sqrt(numel(data))),[.01 .05],[.03 .03]./ceil(sqrt(numel(data))));
% Construct empty plot objects for each robot in each simulation 
figtitle = sgtitle("Initial Title");
figtitle.FontSize =12;
for i=1:numel(ha)
    
    % Create square grid of subplots for all parallel simulations 
%     ax(i) = subplot(ceil(sqrt(numel(data))),ceil(sqrt(numel(data))),i);
    ax(i) = ha(i);
    if numel(data)~=1
        ax(i).XTick = []; 
        ax(i).YTick = []; 
        ax(i).ZTick = []; 
    end
    if i<= numel(data)
        ax(i).NextPlot = 'replaceChildren';
        xlim(ax(i),[-field_width,field_width]); ylim(ax(i),[-field_width,field_width]);

        [s,Z] = PlotScalarField(ax(i),ScalarFieldSelection,500,DesiredValue,field_width,behavior);
        zlim(ax(i),[min(min(Z))-50, max(max(Z))+50]);
        view(ax(i),[0,90])  % Note: view is flipped when using tight_subplot
        cmap = hsv(numel(data(i).robot_ds));
        rx= [data(i).robot_ds.x];
        ry = [data(i).robot_ds.y];
        rz = [data(i).robot_ds.sensor_value];
        %h_line(i,1)= plot3(ax(i),rx(1,:),ry(1,:),rz(1,:)+50),'Marker','o','MarkerFaceColor',cmap,'MarkerEdgeColor','bla','LineWidth',1,'Color',cmap);
        h_line(i,1) = scatter3(ax(i),rx(1,:), ry(1,:), rz(1,:)+50,25,cmap,'filled');
    end
end

% sgtitle(strrep(['Parallel Sim Output: ',data(1).cfg.simulation_name],'_','-'));
% lbl = text(title_axis,'String',strrep(['Sim Output: ',data(1).cfg.simulation_name],'_','-'), ...
%     'FontSize',13,'FontWeight','bold','Position',[0.5, 1-.025,0],...
%     'Units','normalized','HorizontalAlignment','center');

% lbl = text(title_axis,'String',strrep(['Sim Output: ',data(1).cfg.simulation_name],'_','-'), ...
%     'FontSize',13,'FontWeight','bold','Position',[0.5, 1-.025,0],...
%     'Units','normalized','HorizontalAlignment','center');
% 
% title(['\fontsize{15} ', 'Parallel Sim Output: ', char(10) ...
%     '\fontsize{12} ', behavior, char(10) ...
%     '\fontsize{10} ', sprintf('Number of Robots = %i',N), char(10)],'interpreter','tex');

tElapsed = toc(tStart);
fprintf(1,'Elapsed time  = %4.2f sec \n',tElapsed);
fprintf(1,'Creating video frames... \n')

% Animate the simulation results and save video frames
%k is the index for the timestamp to be plotted
for k=1:max_points
    for i=1:numel(data)
        if k>=num_points(i)
            m = num_points(i);
        else
            m = k;
        end
        rx= [data(i).robot_ds.x];
        ry = [data(i).robot_ds.y];
        rz = [data(i).robot_ds.sensor_value];
        %h_line(i,1)= plot3(ax(i),rx(1,:),ry(1,:),rz(1,:)+50),'Marker','o','MarkerFaceColor',cmap,'MarkerEdgeColor','bla','LineWidth',1,'Color',cmap);
        set(h_line(i,1),'xdata',rx(m,:), 'ydata',ry(m,:),'zdata', rz(m,:)+50);
        j=1;
        if strcmp(behavior,'Ridge Follow')
            for p = 1:numel(data(i).robot_ds)
                RobotParams(4*p-3) = data(i).robot_ds(p).x(m);
                RobotParams(4*p-2) = data(i).robot_ds(p).y(m);
                RobotParams(4*p-1) = 0;
                RobotParams(4*p) = data(i).robot_ds(p).sensor_value(m);
            end
            Sensor_Value = RobotParams(4:4:end);
            xs = RobotParams(1:4:end);
            ys = RobotParams(2:4:end);
            while sum(~isnan(Sensor_Value)) >2
                max_robot_i=find(Sensor_Value==max(Sensor_Value));
                if length(max_robot_i)>1
                    max_robot_i = max_robot_i(1);
                end
                for s = 1:numel(data(i).robot_ds)
                    d_from_max(s) = sqrt( ( xs(max_robot_i)-xs(s) ).^2 + ( ys(max_robot_i)-ys(s)).^2 );
                end
                [Vrf,rs] = SwarmSimFollowRidge(RobotParams, max_robot_i,SensorRange);
                mx= data(i).robot_ds(max_robot_i).x(m);
                my= data(i).robot_ds(max_robot_i).y(m);
                mz= data(i).robot_ds(max_robot_i).sensor_value(m);
                if max(d_from_max)< SensorRange
                    mxr= max(d_from_max)*Vrf(1)+mx;
                    myr= max(d_from_max)*Vrf(2)+my;
                else
                    mxr= SensorRange*Vrf(1)+mx;
                    myr= SensorRange*Vrf(2)+my;
                end
                mzr= readScalarField(mxr,myr,ScalarFieldSelection);
                
%                 if j+2>length(h_line)
%                     h_line(i,j) = line('Marker','o','MarkerFaceColor','k','LineWidth',3);
%                     h_line(i,j+1) = line('Marker','o','MarkerFaceColor','w','LineWidth',3,'MaximumNumPoints',1);
%                 end
                if length(h_line(i,:))< (j+2)    ||(~isgraphics(h_line(i,j+1)) || ~isgraphics(h_line(i,j+2)))
                    h_line(i,j+1) = plot3(ax(i),[mx mxr] ,[my, myr],[mz+50, mzr+50],'k','LineWidth',3);
                    h_line(i,j+2) = plot3(ax(i),[mx],[my],[mz+50],'w','LineWidth',3);
                else
                    set( h_line(i,j+1),'xdata',[mx mxr],'ydata',[my, myr],'zdata',[mz+50, mzr+50],'Color','black','LineWidth',3);
                    set( h_line(i,j+2),'xdata',[mx],'ydata',[my],'zdata',[mz+50],'Color','white','LineWidth',3);
                end
                Sensor_Value(d_from_max<=SensorRange) =nan;
                j=j+2;
            end
        while j< length(h_line(i,:))
            set( h_line(i,j),'xdata',[],'ydata',[],'zdata',[]);
            j=j+1;
        end
        end
        pause(0.01)
    end
    %drawnow limitrate
    pause(0.01)
    set(0,'CurrentFigure',fig)
    
% title(['\fontsize{15} ', 'Parallel Sim Output: ', char(10) ...
%     '\fontsize{12} ', behavior, char(10) ...
%     '\fontsize{10} ', sprintf('Number of Robots = %i',N), char(10)],'interpreter','tex');

    figtitle.String = strrep(sprintf('%s\nTime: %0.1f s\n',data(1).cfg.simulation_name,data(1).robot_ds(1).time(m)),'_','-');
    %drawnow;
    if SAVE_VIDEO
        F(k) = getframe(fig);
    end
%         % Update position points for each robot
%     for n = 1:length(h_line(:,1))
%         for o = 1:length(h_line(1,:))
%             if isa(h_line(n,o),'AnimatedLine')
%                 clearpoints(h_line(n,o));
%             end
%         end
%     end
end
tElapsed = toc(tStart);
fprintf(1,'Elapsed time  = %4.2f sec \n',tElapsed);

% Now write frames to video
if SAVE_VIDEO
    for k=1:max_points
        try
            writeVideo(v,F(k))
        catch ME
            disp('an issue with the file size')
            rethow(ME)
        end
    end
    close(v)
    % Note completion
fprintf(1,'AVI File Saved! \n')
end


end

%% tight_subplot()
function [ha, pos] = tight_subplot(Nh, Nw, gap, marg_h, marg_w)
% tight_subplot creates "subplot" axes with adjustable gaps and margins
%
% [ha, pos] = tight_subplot(Nh, Nw, gap, marg_h, marg_w)
%
%   in:  Nh      number of axes in hight (vertical direction)
%        Nw      number of axes in width (horizontaldirection)
%        gap     gaps between the axes in normalized units (0...1)
%                   or [gap_h gap_w] for different gaps in height and width 
%        marg_h  margins in height in normalized units (0...1)
%                   or [lower upper] for different lower and upper margins 
%        marg_w  margins in width in normalized units (0...1)
%                   or [left right] for different left and right margins 
%
%  out:  ha     array of handles of the axes objects
%                   starting from upper left corner, going row-wise as in
%                   subplot
%        pos    positions of the axes objects
%
%  Example: ha = tight_subplot(3,2,[.01 .03],[.1 .01],[.01 .01])
%           for ii = 1:6; axes(ha(ii)); plot(randn(10,ii)); end
%           set(ha(1:4),'XTickLabel',''); set(ha,'YTickLabel','')
% Pekka Kumpulainen 21.5.2012   @tut.fi
% Tampere University of Technology / Automation Science and Engineering

if nargin<3; gap = .02; end
if nargin<4 || isempty(marg_h); marg_h = .05; end
if nargin<5; marg_w = .05; end
if numel(gap)==1; 
    gap = [gap gap];
end
if numel(marg_w)==1; 
    marg_w = [marg_w marg_w];
end
if numel(marg_h)==1; 
    marg_h = [marg_h marg_h];
end
axh = (1-2*sum(marg_h)-(Nh-1)*gap(1))/Nh; 
axw = (1-2*sum(marg_w)-(Nw-1)*gap(2))/Nw;
py = 1-1.25*marg_h(2)-axh; 
% ha = zeros(Nh*Nw,1);
ii = 0;
for ih = 1:Nh
    px = 2*marg_w(1);
    
    for ix = 1:Nw
        ii = ii+1;
        ha(ii) = axes('Units','normalized', ...
            'Position',[px py axw axh]);
        px = px+axw+gap(2);
    end
    py = py-axh-gap(1);
end
if nargout > 1
    pos = get(ha,'Position');
end
ha = ha(:);
end

