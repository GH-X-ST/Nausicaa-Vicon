classdef (Sealed) viconTracker < handle
    % viconTracker streams synchronized full states for Vicon rigid bodies.

    properties (Constant)
        DefaultHost = "192.168.0.100:801"
        DefaultDerivativeCutoffHz = 8.0
    end

    properties (SetAccess = private)
        Host (1, 1) string
        SubjectNames (:, 1) string
        DerivativeCutoffHz (1, 1) double
    end

    properties (Access = private)
        Client
        SegmentNames (:, 1) string
        PreviousFrameNumbers (:, 1) double
        PreviousPositionsM (:, 3) double
        PreviousRotations (:, :, :) double
        PreviousVelocitiesWorld (:, 3) double
        PreviousAngularVelocitiesWorld (:, 3) double
    end

    methods
        function obj = viconTracker(subjectNames, options)
        % viconTracker connects to Vicon and selects rigid bodies.
            arguments
                subjectNames string = strings(0, 1)
                options.Host (1, 1) string = viconTracker.DefaultHost
                options.DerivativeCutoffHz (1, 1) double {mustBeFinite} = ...
                    viconTracker.DefaultDerivativeCutoffHz
            end

            obj.Host = options.Host;
            obj.DerivativeCutoffHz = options.DerivativeCutoffHz;

            sdkPath = fullfile(fileparts(mfilename("fullpath")), "vicon_sdk", ...
                "ViconDataStreamSDK_DotNET.dll");
            NET.addAssembly(char(sdkPath));

            obj.Client = ViconDataStreamSDK.DotNET.Client();
            connection = obj.Client.Connect(char(obj.Host));
            if connection.Result ~= ViconDataStreamSDK.DotNET.Result.Success
                error("viconTracker:ConnectionFailed", ...
                    "Could not connect to the Vicon server at %s.", obj.Host);
            end

            obj.Client.SetBufferSize(uint32(1));
            obj.Client.EnableSegmentData();
            obj.Client.SetStreamMode(ViconDataStreamSDK.DotNET.StreamMode.ServerPush);
            obj.Client.SetAxisMapping( ...
                ViconDataStreamSDK.DotNET.Direction.Forward, ...
                ViconDataStreamSDK.DotNET.Direction.Left, ...
                ViconDataStreamSDK.DotNET.Direction.Up);

            frameOutput = obj.Client.GetFrame();
            if frameOutput.Result ~= ViconDataStreamSDK.DotNET.Result.Success
                error("viconTracker:NoFrame", "Vicon DataStream did not return a frame.");
            end

            if isempty(subjectNames)
                subjectCount = double(obj.Client.GetSubjectCount().SubjectCount);
                subjectNames = strings(subjectCount, 1);
                for subjectIndex = 1:subjectCount
                    subjectOutput = obj.Client.GetSubjectName(uint32(subjectIndex-1));
                    subjectNames(subjectIndex) = string(subjectOutput.SubjectName);
                end
            end

            obj.SubjectNames = subjectNames(:);
            subjectCount = numel(obj.SubjectNames);
            obj.SegmentNames = strings(subjectCount, 1);
            for subjectIndex = 1:subjectCount
                subjectName = char(obj.SubjectNames(subjectIndex));
                segmentOutput = obj.Client.GetSubjectRootSegmentName(subjectName);
                if segmentOutput.Result ~= ViconDataStreamSDK.DotNET.Result.Success
                    error("viconTracker:SubjectUnavailable", ...
                        "Vicon subject '%s' is unavailable.", obj.SubjectNames(subjectIndex));
                end
                obj.SegmentNames(subjectIndex) = string(segmentOutput.SegmentName);
            end

            obj.PreviousFrameNumbers = nan(subjectCount, 1);
            obj.PreviousPositionsM = nan(subjectCount, 3);
            obj.PreviousRotations = nan(3, 3, subjectCount);
            obj.PreviousVelocitiesWorld = zeros(subjectCount, 3);
            obj.PreviousAngularVelocitiesWorld = zeros(subjectCount, 3);
        end

        function frameData = read(obj)
        % read returns one synchronized frame containing visible states.
            arguments
                obj (1, 1) viconTracker
            end

            frameOutput = obj.Client.GetFrame();
            counterTicks = double(System.Diagnostics.Stopwatch.GetTimestamp());
            counterFrequencyHz = double(System.Diagnostics.Stopwatch.Frequency);
            receiveTimeS = counterTicks/counterFrequencyHz;
            if frameOutput.Result ~= ViconDataStreamSDK.DotNET.Result.Success
                error("viconTracker:NoFrame", "Vicon DataStream did not return a frame.");
            end

            frameNumber = double(obj.Client.GetFrameNumber().FrameNumber);
            frameRateHz = double(obj.Client.GetFrameRate().FrameRateHz);
            latencyS = double(obj.Client.GetLatencyTotal().Total);
            subjectCount = numel(obj.SubjectNames);
            states = configureDictionary("string", "struct");
            occludedSubjects = strings(subjectCount, 1);
            occludedCount = 0;

            for subjectIndex = 1:subjectCount
                subjectName = obj.SubjectNames(subjectIndex);
                segmentName = obj.SegmentNames(subjectIndex);
                [poseData, occluded] = obj.readPose(subjectName, segmentName);
                if occluded
                    occludedCount = occludedCount + 1;
                    occludedSubjects(occludedCount) = subjectName;
                    obj.PreviousFrameNumbers(subjectIndex) = NaN;
                    continue
                end

                states(subjectName) = obj.stateFromPose( ...
                    subjectIndex, frameNumber, frameRateHz, poseData);
            end

            frameData = struct( ...
                "frameNumber", frameNumber, ...
                "estimatedCaptureTimeS", receiveTimeS-latencyS, ...
                "frameRateHz", frameRateHz, ...
                "latencyS", latencyS, ...
                "states", states, ...
                "occludedSubjects", occludedSubjects(1:occludedCount));
        end

        function delete(obj)
        % delete disconnects the Vicon DataStream client.
            if ~isempty(obj.Client)
                obj.Client.Disconnect();
            end
        end
    end

    methods (Access = private)
        function [poseData, occluded] = readPose(obj, subjectName, segmentName)
            translation = obj.Client.GetSegmentGlobalTranslation( ...
                char(subjectName), char(segmentName));
            euler = obj.Client.GetSegmentGlobalRotationEulerXYZ( ...
                char(subjectName), char(segmentName));
            quaternion = obj.Client.GetSegmentGlobalRotationQuaternion( ...
                char(subjectName), char(segmentName));
            success = ViconDataStreamSDK.DotNET.Result.Success;
            if translation.Result ~= success || euler.Result ~= success || ...
                    quaternion.Result ~= success
                error("viconTracker:PoseUnavailable", ...
                    "Could not read Vicon subject '%s'.", subjectName);
            end

            occluded = logical(translation.Occluded) || ...
                logical(euler.Occluded) || logical(quaternion.Occluded);
            if occluded
                poseData = struct();
                return
            end

            positionM = reshape(double(translation.Translation), 1, 3)./1000.0;
            eulerXYZRad = reshape(double(euler.Rotation), 1, 3);
            quaternionXYZW = reshape(double(quaternion.Rotation), 1, 4);
            poseData = struct( ...
                "positionM", positionM, ...
                "eulerXYZRad", eulerXYZRad, ...
                "quaternionXYZW", quaternionXYZW, ...
                "rotationBodyToWorld", ...
                viconTracker.quaternion2Rotation(quaternionXYZW));
        end

        function state = stateFromPose( ...
                obj, subjectIndex, frameNumber, frameRateHz, poseData)
            velocityWorldMPerS = zeros(1, 3);
            angularVelocityWorldRadPerS = zeros(1, 3);
            previousFrameNumber = obj.PreviousFrameNumbers(subjectIndex);
            motionValid = ~isnan(previousFrameNumber) && ...
                frameNumber > previousFrameNumber && frameRateHz > 0.0;

            if motionValid
                frameCount = frameNumber - previousFrameNumber;
                deltaTimeS = frameCount/frameRateHz;
                previousPositionM = obj.PreviousPositionsM(subjectIndex, :);
                rawVelocityWorld = ...
                    (poseData.positionM-previousPositionM)/deltaTimeS;
                previousRotation = obj.PreviousRotations(:, :, subjectIndex);
                relativeWorldRotation = ...
                    poseData.rotationBodyToWorld*previousRotation.';
                rawAngularVelocityWorld = ...
                    viconTracker.rotationVector(relativeWorldRotation)/deltaTimeS;

                if obj.DerivativeCutoffHz > 0.0
                    alpha = 1.0 - exp( ...
                        -2.0*pi*obj.DerivativeCutoffHz*deltaTimeS);
                    previousVelocityWorld = ...
                        obj.PreviousVelocitiesWorld(subjectIndex, :);
                    previousAngularVelocityWorld = ...
                        obj.PreviousAngularVelocitiesWorld(subjectIndex, :);
                    velocityWorldMPerS = previousVelocityWorld + ...
                        alpha*(rawVelocityWorld-previousVelocityWorld);
                    angularVelocityWorldRadPerS = ...
                        previousAngularVelocityWorld + ...
                        alpha*(rawAngularVelocityWorld- ...
                        previousAngularVelocityWorld);
                else
                    velocityWorldMPerS = rawVelocityWorld;
                    angularVelocityWorldRadPerS = rawAngularVelocityWorld;
                end
            end

            rotationBodyToWorld = poseData.rotationBodyToWorld;
            velocityBodyMPerS = ...
                (rotationBodyToWorld.'*velocityWorldMPerS.').';
            angularVelocityBodyRadPerS = ...
                (rotationBodyToWorld.'*angularVelocityWorldRadPerS.').';
            state = struct( ...
                "positionM", poseData.positionM, ...
                "eulerXYZRad", poseData.eulerXYZRad, ...
                "quaternionXYZW", poseData.quaternionXYZW, ...
                "velocityWorldMPerS", velocityWorldMPerS, ...
                "velocityBodyMPerS", velocityBodyMPerS, ...
                "angularVelocityBodyRadPerS", angularVelocityBodyRadPerS, ...
                "motionValid", motionValid);

            obj.PreviousFrameNumbers(subjectIndex) = frameNumber;
            obj.PreviousPositionsM(subjectIndex, :) = poseData.positionM;
            obj.PreviousRotations(:, :, subjectIndex) = rotationBodyToWorld;
            obj.PreviousVelocitiesWorld(subjectIndex, :) = velocityWorldMPerS;
            obj.PreviousAngularVelocitiesWorld(subjectIndex, :) = ...
                angularVelocityWorldRadPerS;
        end
    end

    methods (Static, Access = private)
        function rotation = quaternion2Rotation(quaternionXYZW)
            quaternionXYZW = quaternionXYZW./norm(quaternionXYZW);
            x = quaternionXYZW(1);
            y = quaternionXYZW(2);
            z = quaternionXYZW(3);
            w = quaternionXYZW(4);
            rotation = [ ...
                1.0 - 2.0*(y*y+z*z), 2.0*(x*y-z*w), 2.0*(x*z+y*w); ...
                2.0*(x*y+z*w), 1.0 - 2.0*(x*x+z*z), 2.0*(y*z-x*w); ...
                2.0*(x*z-y*w), 2.0*(y*z+x*w), 1.0 - 2.0*(x*x+y*y)];
        end

        function vector = rotationVector(rotation)
            cosine = max(-1.0, min(1.0, (trace(rotation)-1.0)/2.0));
            angle = acos(cosine);
            veeVector = [ ...
                rotation(3, 2) - rotation(2, 3), ...
                rotation(1, 3) - rotation(3, 1), ...
                rotation(2, 1) - rotation(1, 2)];
            if angle < 1.0e-6
                vector = 0.5*veeVector;
            else
                vector = angle*veeVector/(2.0*sin(angle));
            end
        end
    end
end
