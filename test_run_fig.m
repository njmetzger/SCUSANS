fig=figure
FIELD_WIDTH=400;
N=3;

        % First plot scalar field
        ax=gca;
        ax.XLim=[-FIELD_WIDTH FIELD_WIDTH];
        ax.YLim=[-FIELD_WIDTH FIELD_WIDTH];
        cmap = hsv(N);
        title('Click to select robot initial positions')
        
        res=100;
        xdivs=linspace(ax.XLim(1),ax.XLim(2),res);
        ydivs=linspace(ax.YLim(1),ax.YLim(2),res);
        [X,Y] = meshgrid(xdivs,ydivs);
        Z=readScalarField(X,Y);
        surf(X,Y,Z);
        view([0 90])
        hold on
        