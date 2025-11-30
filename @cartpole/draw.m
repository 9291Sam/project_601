function draw(sys,x)

% parameters
m = sys.mass_pendulum;
M = sys.mass_cart;
L = sys.length_pole;

% dimensions
W  = 1*sqrt(M/5);   % cart width
H  = 0.5*sqrt(M/5); % cart height
wr = 0.2;           % wheel radius
mr = 0.3*sqrt(m);   % mass radius

figure;

for k = 1:size(x,1)
    
    z  = x(k,1);  % cart position
    th = x(k,3);  % pendulum angle

    % positions
    y = wr/2+H/2; % cart vertical position
    w1x = z-.9*W/2;
    w1y = 0;
    w2x = z+.9*W/2-wr;
    w2y = 0;

    px = z + L*sin(th);
    py = y - L*cos(th);

    plot([-10 10],[0 0],'w','LineWidth',2)
    hold on
    rectangle('Position',[z-W/2,y-H/2,W,H],'Curvature',.1,'FaceColor',[1 0.1 0.1],'EdgeColor',[1 1 1])
    rectangle('Position',[w1x,w1y,wr,wr],'Curvature',1,'FaceColor',[1 1 1],'EdgeColor',[1 1 1])
    rectangle('Position',[w2x,w2y,wr,wr],'Curvature',1,'FaceColor',[1 1 1],'EdgeColor',[1 1 1])

    plot([z px],[y py],'w','LineWidth',2)

    rectangle('Position',[px-mr/2,py-mr/2,mr,mr],'Curvature',1,'FaceColor',[.3 0.3 1],'EdgeColor',[1 1 1])

    % set(gca,'YTick',[])
    % set(gca,'XTick',[])
    xlim([-5 5]);
    ylim([-2 2.5]);
    set(gca,'Color','k','XColor','w','YColor','w')
    set(gcf,'Position',[10 100 800 400])
    % set(gcf,'units','normalized','outerposition',[0 0 1 1])
    set(gcf,'Color','k')
    set(gcf,'InvertHardcopy','off')

    % box off
    drawnow
    hold off

end

end