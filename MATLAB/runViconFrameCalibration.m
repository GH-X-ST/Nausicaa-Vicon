function result = runViconFrameCalibration(subjectName, knownPositionM, knownEulerXYZDeg, options)
% runViconFrameCalibration compares a Vicon pose with a known arena pose.
%   result = runViconFrameCalibration(subjectName, knownPositionM,
%   knownEulerXYZDeg) averages a stationary aircraft pose and returns the
%   measured pose and measured-minus-reference error.
%
%   knownPositionM is [X Y Z] in metres. knownEulerXYZDeg is
%   [roll pitch yaw] in degrees. Both use the configured global Vicon frame.
%   Host sets the Vicon server address. SampleCount sets the number of
%   visible frames to average. The function prints the comparison without
%   changing the Vicon configuration.

    arguments
        subjectName (1, 1) string
        knownPositionM (1, 3) double {mustBeFinite}
        knownEulerXYZDeg (1, 3) double {mustBeFinite}
        options.Host (1, 1) string = viconTracker.DefaultHost
        options.SampleCount (1, 1) double {mustBeFinite, mustBeInteger, mustBePositive} = 200
    end

    tracker = viconTracker(subjectName, Host=options.Host);
    trackerCleanup = onCleanup(@() delete(tracker));

    [positionMean, eulerMean] = meanPose( ...
        tracker, subjectName, options.SampleCount);
    positionError = positionMean - knownPositionM;
    eulerError = wrapAngles(eulerMean-deg2rad(knownEulerXYZDeg));

    result = struct( ...
        "positionMeanM", positionMean, ...
        "positionErrorM", positionError, ...
        "eulerXYZMeanRad", eulerMean, ...
        "eulerXYZErrorRad", eulerError);

    fprintf("Position mean (m): [%.4f %.4f %.4f]\n", result.positionMeanM);
    fprintf("Position error, measured-reference (m): [%.4f %.4f %.4f]\n", result.positionErrorM);
    fprintf("Euler XYZ mean (deg): [%.4f %.4f %.4f]\n", rad2deg(result.eulerXYZMeanRad));
    fprintf("Euler XYZ error, measured-reference (deg): [%.4f %.4f %.4f]\n", ...
        rad2deg(result.eulerXYZErrorRad));
end

function [positionMean, eulerMean] = meanPose(tracker, subjectName, sampleCount)
    positionSum = zeros(1, 3);
    eulerSineSum = zeros(1, 3);
    eulerCosineSum = zeros(1, 3);
    collectedCount = 0;

    while collectedCount < sampleCount
        frameData = tracker.read();
        if isKey(frameData.states, subjectName)
            state = frameData.states(subjectName);
            positionSum = positionSum + state.positionM;
            eulerSineSum = eulerSineSum + sin(state.eulerXYZRad);
            eulerCosineSum = eulerCosineSum + cos(state.eulerXYZRad);
            collectedCount = collectedCount + 1;
        end
    end

    positionMean = positionSum/sampleCount;
    eulerMean = atan2(eulerSineSum, eulerCosineSum);
end

function angles = wrapAngles(angles)
    angles = mod(angles+pi, 2*pi) - pi;
end
