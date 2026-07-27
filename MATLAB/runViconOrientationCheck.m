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
        options.Host (1, 1) string = viconTracker.DefaultHost
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
    [referencePosition, referenceEuler] = meanPose( ...
        tracker, subjectName, referenceSampleCount);

    passed = true;
    for checkIndex = 1:numel(checkNames)
        fprintf("\nReturn to the reference pose. After pressing Enter, %s.\n", instructions(checkIndex));
        input("Press Enter to start.", "s");
        motionData = captureMotion( ...
            tracker, subjectName, options.MotionDurationSeconds);
        if isempty(motionData.positionM)
            fprintf("FAIL: %s: aircraft not visible\n", checkNames(checkIndex));
            passed = false;
            continue
        end

        axisIndex = axisIndices(checkIndex);
        expectedSign = expectedSigns(checkIndex);
        [poseChange, peakRate] = measureChange( ...
            motionData, referencePosition, referenceEuler, ...
            checkTypes(checkIndex), axisIndex);
        if checkTypes(checkIndex) == "rotation"
            stepPassed = (expectedSign*poseChange >= minRotationRad) && ...
                (expectedSign*peakRate >= minAngularRate);
            status = passStatus(stepPassed);
            if isnan(peakRate)
                peakRate = 0.0;
            end
            fprintf("%s: %s: change=%+.3f deg, peak rate=%+.3f deg/s\n", ...
                status, checkNames(checkIndex), rad2deg(poseChange), rad2deg(peakRate));
        else
            stepPassed = expectedSign*poseChange >= minTranslationM;
            status = passStatus(stepPassed);
            fprintf("%s: %s: change=%+.3f m\n", status, checkNames(checkIndex), poseChange);
        end
        passed = passed && stepPassed;
    end

    fprintf("\nOverall: %s\n", passStatus(passed));
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

function motionData = captureMotion(tracker, subjectName, durationSeconds)
    sampleCapacity = 0;
    sampleCount = 0;
    positionM = zeros(0, 3);
    eulerXYZRad = zeros(0, 3);
    angularVelocityBodyRadPerS = zeros(0, 3);
    motionValid = false(0, 1);
    motionClock = tic();

    while toc(motionClock) < durationSeconds
        frameData = tracker.read();
        if sampleCapacity == 0
            sampleCapacity = max(1, ceil( ...
                durationSeconds*frameData.frameRateHz)+1);
            positionM = zeros(sampleCapacity, 3);
            eulerXYZRad = zeros(sampleCapacity, 3);
            angularVelocityBodyRadPerS = zeros(sampleCapacity, 3);
            motionValid = false(sampleCapacity, 1);
        end

        if isKey(frameData.states, subjectName)
            state = frameData.states(subjectName);
            sampleCount = sampleCount + 1;
            positionM(sampleCount, :) = state.positionM;
            eulerXYZRad(sampleCount, :) = state.eulerXYZRad;
            angularVelocityBodyRadPerS(sampleCount, :) = ...
                state.angularVelocityBodyRadPerS;
            motionValid(sampleCount) = state.motionValid;
        end
    end

    motionData = struct( ...
        "positionM", positionM(1:sampleCount, :), ...
        "eulerXYZRad", eulerXYZRad(1:sampleCount, :), ...
        "angularVelocityBodyRadPerS", ...
        angularVelocityBodyRadPerS(1:sampleCount, :), ...
        "motionValid", motionValid(1:sampleCount));
end

function [poseChange, peakRate] = measureChange( ...
        motionData, referencePosition, referenceEuler, ...
        checkType, axisIndex)
    sampleCount = size(motionData.positionM, 1);
    finalSampleCount = max(1, floor(sampleCount/5));
    finalIndices = sampleCount-finalSampleCount+1:sampleCount;

    if checkType == "rotation"
        finalEuler = meanEuler(motionData.eulerXYZRad(finalIndices, :));
        poseChange = wrapAngle( ...
            finalEuler(axisIndex)-referenceEuler(axisIndex));
        validRates = motionData.angularVelocityBodyRadPerS( ...
            motionData.motionValid, axisIndex);
        if isempty(validRates)
            peakRate = NaN;
        else
            [~, peakIndex] = max(abs(validRates));
            peakRate = validRates(peakIndex);
        end
    else
        finalPosition = mean(motionData.positionM(finalIndices, :), 1);
        poseChange = finalPosition(axisIndex) - referencePosition(axisIndex);
        peakRate = NaN;
    end
end

function eulerMean = meanEuler(eulerXYZRad)
    eulerMean = atan2( ...
        mean(sin(eulerXYZRad), 1), ...
        mean(cos(eulerXYZRad), 1));
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
