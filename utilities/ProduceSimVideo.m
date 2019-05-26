function ProduceSimVideo(data)
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
ScalarFieldSelection = data(1).cfg.ScalarFieldSelection;
behavior = data(1).cfg.behavior;
DesiredValue = [];

% Option to skip saving the video
SAVE_VIDEO = 1;

if SAVE_VIDEO
    video_filename = data(1).cfg.simulation_name;
    
    % If video file already exists, append datetime data to its filename
    if exist([cd,'\results\videos',filesep,video_filename,'.avi']) == 2
        video_filename=[cd,'\results\videos',filesep,video_filename,'_',datestr(now,'yyyy-mm-dd'),'_',datestr(now,'HH_MM_SS'),'.avi'];
    else
        video_filename=[cd,'\results\videos',filesep,video_filename,'.avi'];
    end
    
    % Setup videowriter
    v = VideoWriter(video_filename,'Motion JPEG AVI');
    v.FrameRate=60;
    v.Quality=70;
    open(v)
    F(numel(data(1).robot)) = struct('cdata',[],'colormap',[]);
end

% Get field width for particular scalar field 
[~,field_width] = readScalarField(0,0,ScalarFieldSelection);

% First plot scalar field 
figure('name','Simulation Output','Units','normalized','position',[.3 .2 .4 .6]);
title_axis = axes(gcf,'Position',[0 0 1 1],'Visible','off');

for i = 1:numel(data)
    num_points(i) = numel(data(i).robot(1).x);
end
max_points = max(num_points);

tStart = tic;
fprintf(1,'Constructing robot line objects... \n');

% Use tightly packed subplots
% [ha, pos] = tight_subplot(ceil(sqrt(numel(data))), ceil(sqrt(numel(data))), [.01 .03],[.01 .05],[.01 .01]);
[ha, pos] = tight_subplot(ceil(sqrt(numel(data))), ceil(sqrt(numel(data))), [.01 .01],[.01 .05],[.01 .01]);
[ha, pos] = tight_subplot(ceil(sqrt(numel(data))), ceil(sqrt(numel(data))), [.03 .03]./ceil(sqrt(numel(data))),[.01 .05],[.03 .03]./ceil(sqrt(numel(data))));

% Construct empty plot objects for each robot in each simulation 
for i=1:numel(data)
    
    % Create square grid of subplots for all parallel simulations 
%     ax(i) = subplot(ceil(sqrt(numel(data))),ceil(sqrt(numel(data))),i);
    ax(i) = ha(i);

    ax(i).NextPlot = 'replaceChildren';
    set(ax(i),'YTickLabel',[])
    set(ax(i),'XTickLabel',[])
    xlim(ax(i),[-field_width,field_width]); ylim(ax(i),[-field_width,field_width]);zlim(ax(i),[-30,150])
    
    [s,Z] = PlotScalarField(ax(i),ScalarFieldSelection,500,DesiredValue);
    view(ax(i),[0,90])  % Note: view is flipped when using tight_subplot
    hold all;
    cmap = hsv(numel(data(i).robot));
    for j =1:numel(data(i).robot)
        h_line(i,j)= plot3(ax(i),data(i).robot(j).x(1),data(i).robot(j).y(1),(data(i).robot(j).sensor_value(1)+50),'Marker','o','MarkerFaceColor',cmap(j,:),'MarkerEdgeColor','black','LineWidth',1,'Color',cmap(j,:));
        hold on;
    end
end

% sgtitle(strrep(['Parallel Sim Output: ',data(1).cfg.simulation_name],'_','-'));
lbl = text(title_axis,'String',strrep(['Parallel Sim Output: ',data(1).cfg.simulation_name],'_','-'), ...
    'FontSize',13,'FontWeight','bold','Position',[0.5, 1-.025,0],...
    'Units','normalized','HorizontalAlignment','center');

tElapsed = toc(tStart);
fprintf(1,'Elapsed time  = %4.2f sec \n',tElapsed);
fprintf(1,'Creating video frames... \n')

% Animate the simulation results and save video frames
for k=1:max_points
    for i=1:numel(data)
        if k>=num_points(i)
            m = num_points(i);
        else
            m = k;
        end
        for j=1:numel(data(i).robot)
            set(h_line(i,j),'xdata',data(i).robot(j).x(m),'ydata',data(i).robot(j).y(m),'zdata',(data(i).robot(j).sensor_value(m)+50));
        end
    end
%         drawnow limitrate
    drawnow;
    if SAVE_VIDEO
        F(k) = getframe(gcf);
    end
end

tElapsed = toc(tStart);
fprintf(1,'Elapsed time  = %4.2f sec \n',tElapsed);

% Now write frames to video
if SAVE_VIDEO
    for k=1:max_points
        writeVideo(v,F(k))
    end
    close(v)
end

% Note completion
fprintf(1,'AVI File Saved! \n')
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
axh = (1-sum(marg_h)-(Nh-1)*gap(1))/Nh; 
axw = (1-sum(marg_w)-(Nw-1)*gap(2))/Nw;
py = 1-marg_h(2)-axh; 
% ha = zeros(Nh*Nw,1);
ii = 0;
for ih = 1:Nh
    px = marg_w(1);
    
    for ix = 1:Nw
        ii = ii+1;
        ha(ii) = axes('Units','normalized', ...
            'Position',[px py axw axh], ...
            'XTickLabel','', ...
            'YTickLabel','');
        px = px+axw+gap(2);
    end
    py = py-axh-gap(1);
end
if nargout > 1
    pos = get(ha,'Position');
end
ha = ha(:);
end

