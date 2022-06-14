function ref = generateReference(tEnd, tSample, tChange, ...
                                    speed, lWidth)
% Returns a time-dependent reference trajectory for the double lane-change.
% The lane change starts after 5 seconds, then the reference stays
% in the second lane for 10 seconds before returning back. How quick the
% lane change itself is carried out depends on the tChange variable.
% INPUTS:
%   tEnd - at which time instance to stop the reference trajectory. Set the
%       time to be longer than desired simulation time by the same amount
%       as the prediction horizon of the MPC controller. Set the tEnd 
%       smaller than '15 + tChange' to generate the reference for
%       a single lane change. 
%   tSample - the sample time of the reference trajectory, must be the same
%       as the time step of the MPC.
%   tChange - how quick the lane change should happen reached.
%   speed - the speed at which the bicycle will be travelling.
%   lWidth - the width of the lane change.
% OUTPUT:
%   ref - the generated reference. First column is time, second is
%   longitudinal distance, third is lateral distance.

    t_ref = (0:tSample:tEnd)';
    x_ref = t_ref.*speed;
    y_ref = zeros(length(t_ref),1);
    
    % First lanechange
    for i = floor(5/tSample+1) : floor((5+tChange)/tSample)
        y_ref(i) = y_ref(i-1) + (lWidth*tSample)/tChange;
    end
    % Riding in the second lane
    for i = floor((5+tChange)/tSample+1) : floor((15+tChange)/tSample)
        y_ref(i) = lWidth;
    end
    % Second lanechange
    for i = floor((15+tChange)/tSample+1) : floor((15+2*tChange)/tSample)
        y_ref(i) = y_ref(i-1) - (lWidth*tSample)/tChange;
        if y_ref(i) < 0
            y_ref(i) = 0;
        end
    end

    ref = [t_ref, x_ref, y_ref];
end