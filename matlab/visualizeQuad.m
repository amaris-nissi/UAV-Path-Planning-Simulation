function P = visualizeQuad(S)
% visualizeQuad : Takes in an input structure S and visualizes the resulting
%                 3D motion in approximately real-time.  Outputs the data
%                 used to form the plot.
%
%
% INPUTS
%
% S ---------- Structure with the following elements:
%
%           rMat = 3xM matrix of quad positions, in meters
%
%           eMat = 3xM matrix of quad attitudes, in radians
%
%           tVec = Mx1 vector of times corresponding to each measurement in
%                  xevwMat
%
%  plotFrequency = The scalar number of frames of the plot per each second of
%                  input data.  Expressed in Hz.
%
%         bounds = 6x1, the 3d axis size vector
%
%    makeGifFlag = Boolean (if true, export the current plot to a .gif)
%
%    gifFileName = A string with the file name of the .gif if one is to be
%                  created.  Make sure to include the .gif exentsion.
%
%
% OUTPUTS
%
% P ---------- Structure with the following elements:
%
%          tPlot = Nx1 vector of time points used in the plot, sampled based
%                  on the frequency of plotFrequency
%
%          rPlot = 3xN vector of positions used to generate the plot, in
%                  meters.
%
%          ePlot = 3xN vector of attitudes used to generate the plot, in
%                  radians.
%
%+------------------------------------------------------------------------------+
  
% Important params
figureNumber = 42; figure(figureNumber); clf;
fcounter = 0; %frame counter for gif maker
m = length(S.tVec);

% RBG scaled on [0,1] for the color orange
rgbOrange=[1 .4 0];

% Parameters for the rotors
rotorLocations=[0.105 0.105 -0.105 -0.105
    0.105 -0.105 0.105 -0.105
    0 0 0 0];
r_rotor = .062;

% Determines the location of the corners of the body box in the body frame,
% in meters
bpts=[ 120  120 -120 -120  120  120 -120 -120
    28  -28   28  -28   28  -28   28  -28
    20   20   20   20   -30   -30   -30   -30 ]*1e-3;
% Rectangles representing each side of the body box
b1 = [bpts(:,1) bpts(:,5) bpts(:,6) bpts(:,2) ];
b2 = [bpts(:,1) bpts(:,5) bpts(:,7) bpts(:,3) ];
b3 = [bpts(:,3) bpts(:,7) bpts(:,8) bpts(:,4) ];
b4 = [bpts(:,1) bpts(:,3) bpts(:,4) bpts(:,2) ];
b5 = [bpts(:,5) bpts(:,7) bpts(:,8) bpts(:,6) ];
b6 = [bpts(:,2) bpts(:,6) bpts(:,8) bpts(:,4) ];

% Create a circle for each rotor
t_circ=linspace(0,2*pi,20);
circpts=zeros(3,20);
for i=1:20
    circpts(:,i)=r_rotor*[cos(t_circ(i));sin(t_circ(i));0];
end

% Plot single epoch if m==1
if m==1
    figure(figureNumber);
    
    % Extract params
    RIB = euler2dcm(S.eMat(1:3))';
    r = S.rMat(1:3);
    
    % Translate, rotate, and plot the rotors
    hold on
    view(3)
    rotor1_circle=r*ones(1,20)+RIB*(circpts(:,:)+rotorLocations(:,1)*ones(1,20));
    rotor1plot = plot3(rotor1_circle(1,:), rotor1_circle(2,:), rotor1_circle(3,:),...
        'Color',rgbOrange);
    hold on
    rotor2_circle=r*ones(1,20)+RIB*(circpts(:,:)+rotorLocations(:,2)*ones(1,20));
    rotor2plot = plot3(rotor2_circle(1,:), rotor2_circle(2,:), rotor2_circle(3,:),...
        'Color',rgbOrange);
    hold on
    rotor3_circle=r*ones(1,20)+RIB*(circpts(:,:)+rotorLocations(:,3)*ones(1,20));
    rotor3plot = plot3(rotor3_circle(1,:), rotor3_circle(2,:), rotor3_circle(3,:),...
        'black');
    hold on
    rotor4_circle=r*ones(1,20)+RIB*(circpts(:,:)+rotorLocations(:,4)*ones(1,20));
    rotor4plot = plot3(rotor4_circle(1,:), rotor4_circle(2,:), rotor4_circle(3,:),...
        'black');
    
    % Plot the body 
    b1r=r*ones(1,4)+RIB*b1; b2r=r*ones(1,4)+RIB*b2; b3r=r*ones(1,4)+RIB*b3;
    b4r=r*ones(1,4)+RIB*b4; b5r=r*ones(1,4)+RIB*b5; b6r=r*ones(1,4)+RIB*b6;
    X = [b1r(1,:)' b2r(1,:)' b3r(1,:)' b4r(1,:)' b5r(1,:)' b6r(1,:)'];
    Y = [b1r(2,:)' b2r(2,:)' b3r(2,:)' b4r(2,:)' b5r(2,:)' b6r(2,:)'];
    Z = [b1r(3,:)' b2r(3,:)' b3r(3,:)' b4r(3,:)' b5r(3,:)' b6r(3,:)'];
    hold on
    bodyplot=patch(X,Y,Z,[.5 .5 .5]);
    
    % Plot the body axes
    bodyX=0.5*RIB*[1;0;0]; bodyY=0.5*RIB*[0;1;0]; bodyZ=0.5*RIB*[0;0;1];
    hold on
    axis1 = quiver3(r(1),r(2),r(3),bodyX(1),bodyX(2),bodyX(3),'red');
    hold on
    axis2 = quiver3(r(1),r(2),r(3),bodyY(1),bodyY(2),bodyY(3),'blue');
    hold on
    axis3 = quiver3(r(1),r(2),r(3),bodyZ(1),bodyZ(2),bodyZ(3),'green');
    axis(S.bounds)
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    grid on
    
    P.tPlot = S.tVec;
    P.rPlot = S.rMat;
    P.ePlot = S.eMat;
    
elseif m>1 % Interpolation must be used to smooth timing
    
    % Create time vectors
    tf = 1/S.plotFrequency;
    tmax = S.tVec(m); tmin = S.tVec(1);
    tPlot = tmin:tf:tmax;
    tPlotLen = length(tPlot);
    
    % Interpolate to regularize times
    [t2unique, indUnique] = unique(S.tVec);
    rPlot = (interp1(t2unique, S.rMat(indUnique,:), tPlot))';
    ePlot = (interp1(t2unique, S.eMat(indUnique,:), tPlot))';
    
    figure(figureNumber);
    
    % Iterate through points
    for i=1:tPlotLen
        
        % Start timer
        tic
        
        % Extract data
        RIB = euler2dcm(ePlot(1:3,i))';
        r = rPlot(1:3,i);
        
        % Translate, rotate, and plot the rotors
        hold on
        view(3)
        rotor1_circle=r*ones(1,20)+RIB*(circpts(:,:)+rotorLocations(:,1)*ones(1,20));
        rotor1plot = plot3(rotor1_circle(1,:), rotor1_circle(2,:),...
            rotor1_circle(3,:), 'Color',rgbOrange);
        hold on
        rotor2_circle=r*ones(1,20)+RIB*(circpts(:,:)+rotorLocations(:,2)*ones(1,20));
        rotor2plot = plot3(rotor2_circle(1,:), rotor2_circle(2,:),...
            rotor2_circle(3,:), 'Color',rgbOrange);
        hold on
        rotor3_circle=r*ones(1,20)+RIB*(circpts(:,:)+rotorLocations(:,3)*ones(1,20));
        rotor3plot = plot3(rotor3_circle(1,:), rotor3_circle(2,:),...
            rotor3_circle(3,:), 'black');
        hold on
        rotor4_circle=r*ones(1,20)+RIB*(circpts(:,:)+rotorLocations(:,4)*ones(1,20));
        rotor4plot = plot3(rotor4_circle(1,:), rotor4_circle(2,:),...
            rotor4_circle(3,:), 'black');
        
        % Translate, rotate, and plot the body
        b1r=r*ones(1,4)+RIB*b1; b2r=r*ones(1,4)+RIB*b2; b3r=r*ones(1,4)+RIB*b3;
        b4r=r*ones(1,4)+RIB*b4; b5r=r*ones(1,4)+RIB*b5; b6r=r*ones(1,4)+RIB*b6;
        X = [b1r(1,:)' b2r(1,:)' b3r(1,:)' b4r(1,:)' b5r(1,:)' b6r(1,:)'];
        Y = [b1r(2,:)' b2r(2,:)' b3r(2,:)' b4r(2,:)' b5r(2,:)' b6r(2,:)'];
        Z = [b1r(3,:)' b2r(3,:)' b3r(3,:)' b4r(3,:)' b5r(3,:)' b6r(3,:)'];
        hold on
        bodyplot=patch(X,Y,Z,[.5 .5 .5]);
        
        % Translate, rotate, and plot body axes
        bodyX=0.5*RIB*[1;0;0]; bodyY=0.5*RIB*[0;1;0]; bodyZ=0.5*RIB*[0;0;1];
        hold on
        axis1 = quiver3(r(1),r(2),r(3),bodyX(1),bodyX(2),bodyX(3),'red');
        hold on
        axis2 = quiver3(r(1),r(2),r(3),bodyY(1),bodyY(2),bodyY(3),'blue');
        hold on
        axis3 = quiver3(r(1),r(2),r(3),bodyZ(1),bodyZ(2),bodyZ(3),'green');
        % Fix up plot style
        axis(S.bounds)
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        grid on
        
        tP=toc;
        % Pause to stay near-real-time
        pause(max(0.001,tf-tP))
        
        % Gif stuff
        if S.makeGifFlag
            fcounter=fcounter+1;
            frame=getframe(figureNumber);
            im=frame2im(frame);
            [imind,cm]=rgb2ind(im,256);
            if fcounter==1
                imwrite(imind,cm,S.gifFileName,'gif','Loopcount',inf,...
                    'DelayTime',tf);
            else
                imwrite(imind,cm,S.gifFileName,'gif','WriteMode','append',...
                    'DelayTime',tf);
            end
        end
        
        % Clear plot before next iteration, unless at final time step
        if i<tPlotLen
            delete(rotor1plot)
            delete(rotor2plot)
            delete(rotor3plot)
            delete(rotor4plot)
            delete(bodyplot)
            delete(axis1)
            delete(axis2)
            delete(axis3)
        end
    end
    
    P.tPlot = tPlot;
    P.ePlot = ePlot;
    P.rPlot = rPlot;
end

end

