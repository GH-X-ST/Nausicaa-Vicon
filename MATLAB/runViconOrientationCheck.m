function passed = runViconOrientationCheck(subjectName, options)
% runViconOrientationCheck checks an aircraft's Vicon axis directions.
%   passed = runViconOrientationCheck(subjectName) guides six physical
%   motions and reports whether their position, attitude, and angular-rate
%   signs match the flight-arena coordinate frame.
%
%   subjectName is the case-sensitive aircraft subject in Vicon Tracker.
%   Host sets the Vicon server address. MotionDurationSeconds sets the time
%   allowed for each motion. The function prompts the user, prints each
%   result, and returns true only when all six checks pass.

    arguments
        subjectName (1, 1) string
        options.Host (1, 1) string = "192.168.0.100:801"
        options.MotionDurationSeconds (1, 1) double {mustBeFinite, mustBePositive} = 3.0
    end

    referenceSampleCount = 50;
    minTranslationM = 0.15;
    minRotationRad = deg2rad(8.0);
    minAngularRate = 0.10;
    checkNames = ["forward"; "left"; "up"; "roll right"; "pitch up"; "yaw right"];
    instructions = [
        "move the aircraft forward along +X and hold"
        "move the aircraft left along +Y and hold"
        "move the aircraft upward along +Z and hold"
        "roll right with the right wing down and hold"
        "pitch the nose up and hold"
        "yaw the nose right and hold"
    ];
    checkTypes = ["translation"; "translation"; "translation"; "rotation"; "rotation"; "rotation"];
    axisIndices = [1; 2; 3; 1; 2; 3];
    expectedSigns = [1; 1; 1; 1; -1; -1];

    tracker = viconTracker(subjectName, Host=options.Host);
    trackerCleanup = onCleanup(@() delete(tracker));

    fprintf("Align the aircraft nose with +X, left wing with +Y, and top with +Z.\n");
    input("Hold the reference pose, then press Enter.", "s");
    [referencePosition, referenceEuler] = meanPose(tracker, referenceSampleCount);

    passed = true;
    for checkIndex = 1:numel(checkNames)
        fprintf("\nReturn to the reference pose. After pressing Enter, %s.\n", instructions(checkIndex));
        input("Press Enter to start.", "s");
        [finalPosition, finalEuler, peakAngularRate] = captureMotion( ...
            tracker, options.MotionDurationSeconds);

        axisIndex = axisIndices(checkIndex);
        expectedSign = expectedSigns(checkIndex);
        if checkTypes(checkIndex) == "rotation"
            poseChange = wrapAngle(finalEuler(axisIndex)-referenceEuler(axisIndex));
            peakRate = peakAngularRate(axisIndex);
            stepPassed = (expectedSign*poseChange >= minRotationRad) && ...
                (expectedSign*peakRate >= minAngularRate);
            status = passStatus(stepPassed);
            fprintf("%s: %s: change=%+.3f deg, peak rate=%+.3f deg/s\n", ...
                status, checkNames(checkIndex), rad2deg(poseChange), rad2deg(peakRate));
        else
            poseChange = finalPosition(axisIndex) - referencePosition(axisIndex);
            stepPassed = expectedSign*poseChange >= minTranslationM;
            status = passStatus(stepPassed);
            fprintf("%s: %s: change=%+.3f m\n", status, checkNames(checkIndex), poseChange);
        end
        passed = passed && stepPassed;
    end

    fprintf("\nOverall: %s\n", passStatus(passed));
end

function [positionMean, eulerMean] = meanPose(tracker, sampleCount)
    positionSum = zeros(1, 3);
    eulerSineSum = zeros(1, 3);
    eulerCosineSum = zeros(1, 3);
    collectedCount = 0;

    while collectedCount < sampleCount
        states = tracker.read();
        pose = states(1, 1:6);
        if all(isfinite(pose))
            positionSum = positionSum + pose(1:3);
            eulerSineSum = eulerSineSum + sin(pose(4:6));
            eulerCosineSum = eulerCosineSum + cos(pose(4:6));
            collectedCount = collectedCount + 1;
        end
    end

    positionMean = positionSum/sampleCount;
    eulerMean = atan2(eulerSineSum, eulerCosineSum);
end

function [positionMean, eulerMean, peakAngularRate] = captureMotion(tracker, durationSeconds)
    positionSum = zeros(1, 3);
    eulerSineSum = zeros(1, 3);
    eulerCosineSum = zeros(1, 3);
    finalSampleCount = 0;
    peakAngularRate = nan(1, 3);
    finalWindowStart = 0.8*durationSeconds;
    motionClock = tic();

    while toc(motionClock) < durationSeconds
        [states, valid] = tracker.read();
        pose = states(1, 1:6);
        if (toc(motionClock) >= finalWindowStart) && all(isfinite(pose))
            positionSum = positionSum + pose(1:3);
            eulerSineSum = eulerSineSum + sin(pose(4:6));
            eulerCosineSum = eulerCosineSum + cos(pose(4:6));
            finalSampleCount = finalSampleCount + 1;
        end

        if valid(1)
            angularRate = states(1, 10:12);
            replacePeak = isnan(peakAngularRate) | (abs(angularRate) > abs(peakAngularRate));
            peakAngularRate(replacePeak) = angularRate(replacePeak);
        end
    end

    if finalSampleCount == 0
        positionMean = nan(1, 3);
        eulerMean = nan(1, 3);
        return
    end

    positionMean = positionSum/finalSampleCount;
    eulerMean = atan2(eulerSineSum, eulerCosineSum);
end

function angle = wrapAngle(angle)
    angle = mod(angle+pi, 2*pi) - pi;
end

function status = passStatus(passed)
    status = "FAIL";
    if passed
        status = "PASS";
    end
end
