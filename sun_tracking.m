%% Description

% This final code solves for tube capture of incident and reflected light 
% over a full ellipse during a half day incidence.
% 
% This code reflects the most up to date solution as of 9/5/10, and
% incorporates solutions to concerns raised at solar team lab meeting of 
% 7/29/10
% 
% Specifically, this code correctly accounts for conservation of energy of 
% incident light as well as variable light intensity over the course of the
% day
% 
% This code also takes into account shadowing effects that are present when
% incident light is intercepted by the tube without ever reaching the
% mirror
%
% This code solves for the optimal tube size and placement, and then plots 
% this tube along with the ellipse and all incident and reflected light
% 
% The percent capture calculated by this code is about 55% for a tube of
% size radius .755" and location of -.2" below x = 0 (ellipse horizontal
% line of symmetry). 

%% Define Ellipse, Tube and Theta variables
clear all
home 
hold on

a = 6/2; %major axis
b = 3/2; %minor axis

x = linspace(-2.99,2.99);
y = -b.*sqrt(1-(x./a).^2); %equation of ellipse

slope = b*x./(a*sqrt(a^2-x.^2)); %tangent slope to ellipse at any point
beta = atan(slope); % angle that tangent slope makes with horizontal

theta = [90]*(pi/180); % angles of incidence of sun's rays 
numThetas = length(theta); % number of theta values

cent = (-1.4:.1:1.4); % 29 possible locations for the center of the tube, 
% ranging from -1.4 (bottom of the ellipse) to +1.4 (top of ellipse)
numCenters = length(cent); % number of center values

rad = (.05:.015:.755); % 31 possible values for the radius of the circle
numRadii = length(rad); % number of radius values

%% Create Matrixes to be filled by yReflect values and intersection indicators

intCheckTotal = zeros(numThetas,100);

sumAll = zeros(numCenters, numRadii);

allRays = zeros(1, numThetas);

%% Solve for best tube
 for c = 1:numCenters %cycle through all center locations
    for r = 1:numRadii %cycle through all radius values
        for i = 1:numThetas %cycle through all incident theta
            numRays = 100*sin(theta(1,i)); %number of incident rays on horizontal line of x=0 depends on sin(theta) (conservation on energy)
    
            for j = 1:numRays
                x_spacing = (-2.99:2.99*2/numRays:2.99); % conservation of energy: equally space incident light
                
                rayIntercept = tan(theta(1,i))*-1*x_spacing(1,j); % Define y-intercept of Incident Ray
                Incident = tan(theta(1,i))*x_spacing + rayIntercept ; % Define Incident Ray
               
                if theta(1,i) == 90*(pi/180) % if incident light is vertical
                    raySlope = inf; % linecirc function requires an "inf" for slope input
                    rayIntercept = x_spacing(1,j); % linecirc function requires x-intercept
                else 
                    raySlope = tan(theta(1,i)); % standard slope input calculation
                end 
                
                intersect = linecirc(raySlope, rayIntercept, 0, cent(1,c), rad(1,r)); % checks to see if ray defined by phi and yReflect intesects circle defined by 0, cent, rad
                intCheck2 = ~isnan(intersect); % if there's an intersect, change the coordinates to a pair of 1's. If no intersect, return two 0's 
                if intCheck2 == 1                                
                    intCheck1 = sin(theta(1,i)); % reduces intCheck2 to one indicator instead of two
                    
             
                else       
                    
                    rayIntercept = tan(theta(1,i))*-1*x_spacing(1,j); % re-declare y-intercept of Incident Ray
                    xPosition = -(a*b*(a^2*tan(theta(1,i))^2 + b^2 - rayIntercept^2)^(1/2) + a^2*rayIntercept*tan(theta(1,i)))/(a^2*tan(theta(1,i))^2 + b^2); % general solution of line and ellipse intersection
                    xSlope = b*xPosition/(a*sqrt(a^2-xPosition^2)); % tangent slope to ellipse at line-ellipse intersect point
                    xBeta = atan(xSlope); % angle of tangent slope at intersect point

                    alpha = theta(1,i) - xBeta; % for each loop, this takes one value of theta and determines the angle of reflectance off of ellipse
                    phi = alpha-xBeta; %component of angle of reflectance that intersects horizontal 
                    h=tan(phi)*xPosition; 
                    yPosition = -b.*sqrt(1-(xPosition./a).^2); % y position at line-ellipse intersect point
                    yReflect = yPosition+h; % y intercept of reflected rays

                    intersect = linecirc(tan(-phi), yReflect, 0, cent(1,c), rad(1,r)); % checks to see if ray defined by phi and yReflect intesects circle defined by 0, cent, rad
                    intCheck2 = ~isnan(intersect); % if there's an intersect, change the coordinates to a pair of 1's. If no intersect, return two 0's                            
                    if intCheck2 == 1                                
                        intCheck1 = sin(theta(1,i)); % reduces intCheck2 to one indicator instead of two
                       
                    else                
                        intCheck1 = 0; % reduces intCheck2 to one zero instead of two
                       
                    end 
                end 
                intCheckTotal(i,j) = intCheck1; % creates a matrix indicating whether there was a ray intersection for each particular circle radius and center, 
                
                allRays(1,i) = numRays*sin(theta(1,i)); % logs the number of rays for each incident angle
                totalRays = sum(allRays); % adds up total number of incident rays in order to determine percent capture
                sumAll(c,r) = sum(sum(intCheckTotal)); % sum of all intersection indicators
                percent = sumAll./totalRays; % percent capture matrix representing each possible tube combination
            end 
        end

    end 
end

%% Filter and analyze data to determine best result

findPercent = find(percent > .75); % find locations of >75% capture in percent matrix

maxCapture = max(max(percent)); %maximum percent capture
findMax = find(percent == maxCapture);%location of maximum percent capture

%% Output tube location and size based on data analysis results

s = [c,r];
[CV RV] = ind2sub(s, findPercent); %find indicies of >75% capture
RADII = rad(1,RV); % corresponding radii
CENTERS = cent(1,CV); % corresponding center location

if isempty(RADII) == 0 % if solutions of >75% exist,
    
    disp('Tube size and location of:')
    RADIUS = RADII(1,1) %output smallest radius
    CENTER = CENTERS(1,1)%corresponding center location
    disp('Results in:')
    disp(percent(CV(1,1), RV(1,1)))
    disp('capture')
    
else %if no solution of >75% exists, find location of max capture
    disp('There is no solution greater than 75% capture. Maximum capture of')
    disp(maxCapture) 
    disp('occurs for tube with:')
    
    [CV RV] = ind2sub(s, findMax); % find indicies of max capture
    RADII = rad(1,RV); % corresponding radii
    CENTERS = cent(1,CV); % corresponding centers

    RADIUS = RADII(1,1) % smallest tube radius that captures max capture value
    CENTER = CENTERS(1,1) % corresponding center
end 

%% Plot Ellipse 
plot(x,y, 'b')
grid on 
axis([-3 3 -3 3]);
box on
axis equal
%% Plot rays
% same algorithm as above, but only plots the solution tube

 for i = 1:numThetas

    numRays = floor(100*sin(theta(1,i)));

    for j = 1:numRays
        x_spacing = (-2.99:2.99*2/(numRays-1):2.99);

        rayIntercept = tan(theta(1,i))*-1*x_spacing(1,j); % Define y-intercept of Incident Ray
        Incident = tan(theta(1,i))*x_spacing + rayIntercept ; % Define Incident Ray
        plot(x_spacing,Incident, 'y') % Plot Incident Rays in yellow

        if theta(1,i) == 90*(pi/180)
            raySlope = inf;
            rayIntercept = x_spacing(1,j);
        else 
            raySlope = tan(theta(1,i));
        end 

        intersect = linecirc(raySlope, rayIntercept, 0, CENTER, RADIUS); 
        intCheck2 = ~isnan(intersect); % if there's an intersect, change the coordinates to a pair of 1's. If no intersect, return two 0's 
        if intCheck2 == 1                                
            intCheck1 = sin(theta(1,i)); % reduces intCheck2 to one indicator instead of two
            plot(x_spacing,Incident, 'g') % Plot Incident Rays that intersect tube in green

        else    

            rayIntercept = tan(theta(1,i))*-1*x_spacing(1,j); % re-declare y-intercept of Incident Ray    
            xPosition = -(a*b*(a^2*tan(theta(1,i))^2 + b^2 - rayIntercept^2)^(1/2) + a^2*rayIntercept*tan(theta(1,i)))/(a^2*tan(theta(1,i))^2 + b^2);
            xSlope = b*xPosition/(a*sqrt(a^2-xPosition^2)); %tangent slope to ellipse at line-ellipse intersection
            xBeta = atan(xSlope); % angle that tangent slope makes with horizontal

            alpha = theta(1,i) - xBeta; 
            phi = alpha-xBeta; % component of angle of reflectance that intersects horizontal
            h=tan(phi)*xPosition; 
            yPosition = -b.*sqrt(1-(xPosition./a).^2);
            yReflect = yPosition+h; % y intercept of reflected rays

            intersect = linecirc(tan(-phi), yReflect, 0, CENTER, RADIUS); % checks to see if ray defined by phi and yReflect intesects circle defined by 0, cent, rad
            intCheck2 = ~isnan(intersect); % if there's an intersect, change the coordinates to a pair of 1's. If no intersect, return two 0's                            
            if intCheck2 == 1                                
                intCheck1 = sin(theta(1,i)); % reduces intCheck2 to one indicator instead of two

            else                
                intCheck1 = 0; % reduces intCheck2 to one zero instead of two
            end 
        end

        Reflected = tan(-phi)*x_spacing + yReflect;
        plot(x_spacing, Reflected, 'r') % plot reflected rays in red
        whitebg([(47/255) (79/255) (79/255)]);
    end
end 

%% Plot Tube

c = rsmak('circle', RADIUS, [0 CENTER]);
fnplt(c, 'b') % plot tube solution

hold off
