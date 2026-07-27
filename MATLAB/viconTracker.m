classdef viconTracker < handle
    % viconTracker streams synchronized full states for Vicon rigid bodies.

    properties (Constant)
        StateNames = ["x", "y", "z", "roll", "pitch", "yaw", ...
            "u", "v", "w", "p", "q", "r"]
    end

    properties (SetAccess = private)
        SubjectNames (:, 1) string
    end

    properties (Access = private)
        Client
        SegmentNames (:, 1) string
        PreviousFrameNumbers (:, 1) double
        PreviousPositions (:, 3) double
        PreviousRotations (:, :, :) double
    end

    methods
        function obj = viconTracker(subjectNames, options)
            % viconTracker connects to Vicon and selects rigid bodies.
            arguments
                subjectNames string = strings(0, 1)
                options.Host (1, 1) string = "192.168.0.100:801"
            end

            sdkPath = fullfile(fileparts(mfilename("fullpath")), "vicon_sdk", ...
                "ViconDataStreamSDK_DotNET.dll");
            NET.addAssembly(char(sdkPath));

            obj.Client = ViconDataStreamSDK.DotNET.Client();
            connection = obj.Client.Connect(char(options.Host));
            if connection.Result ~= ViconDataStreamSDK.DotNET.Result.Success
                error("viconTracker:ConnectionFailed", ...
                    "Could not connect to the Vicon server at %s.", options.Host);
            end

            obj.Client.EnableSegmentData();
            obj.Client.SetBufferSize(uint32(1));
            obj.Client.SetStreamMode(ViconDataStreamSDK.DotNET.StreamMode.ServerPush);
            obj.Client.SetAxisMapping( ...
                ViconDataStreamSDK.DotNET.Direction.Forward, ...
                ViconDataStreamSDK.DotNET.Direction.Left, ...
                ViconDataStreamSDK.DotNET.Direction.Up);

            frame = obj.Client.GetFrame();
            if frame.Result ~= ViconDataStreamSDK.DotNET.Result.Success
                error("viconTracker:NoFrame", "Vicon DataStream did not return a frame.");
            end

            if isempty(subjectNames)
                subjectCount = double(obj.Client.GetSubjectCount().SubjectCount);
                subjectNames = strings(subjectCount, 1);
                for subjectIndex = 1:subjectCount
                    subject = obj.Client.GetSubjectName(uint32(subjectIndex - 1));
                    subjectNames(subjectIndex) = string(subject.SubjectName);
                end
            end

            obj.SubjectNames = subjectNames(:);
            subjectCount = numel(obj.SubjectNames);
            obj.SegmentNames = strings(subjectCount, 1);
            for subjectIndex = 1:subjectCount
                subjectName = char(obj.SubjectNames(subjectIndex));
                segment = obj.Client.GetSubjectRootSegmentName(subjectName);
                if segment.Result ~= ViconDataStreamSDK.DotNET.Result.Success
                    error("viconTracker:SubjectUnavailable", ...
                        "Vicon subject '%s' is unavailable.", obj.SubjectNames(subjectIndex));
                end
                obj.SegmentNames(subjectIndex) = string(segment.SegmentName);
            end

            obj.PreviousFrameNumbers = nan(subjectCount, 1);
            obj.PreviousPositions = nan(subjectCount, 3);
            obj.PreviousRotations = nan(3, 3, subjectCount);
        end

        function [states, valid, frameNumber] = read(obj)
            % read returns one synchronized N-by-12 rigid-body state matrix.
            arguments
                obj (1, 1) viconTracker
            end

            frame = obj.Client.GetFrame();
            if frame.Result ~= ViconDataStreamSDK.DotNET.Result.Success
                error("viconTracker:NoFrame", "Vicon DataStream did not return a frame.");
            end

            frameNumber = double(obj.Client.GetFrameNumber().FrameNumber);
            frameRateHz = double(obj.Client.GetFrameRate().FrameRateHz);
            subjectCount = numel(obj.SubjectNames);
            states = nan(subjectCount, 12);
            valid = false(subjectCount, 1);
            success = ViconDataStreamSDK.DotNET.Result.Success;

            for subjectIndex = 1:subjectCount
                subjectName = char(obj.SubjectNames(subjectIndex));
                segmentName = char(obj.SegmentNames(subjectIndex));
                translation = obj.Client.GetSegmentGlobalTranslation(subjectName, segmentName);
                euler = obj.Client.GetSegmentGlobalRotationEulerXYZ(subjectName, segmentName);
                rotationMatrix = obj.Client.GetSegmentGlobalRotationMatrix(subjectName, segmentName);

                poseAvailable = translation.Result == success && ...
                    euler.Result == success && rotationMatrix.Result == success && ...
                    ~translation.Occluded && ...
                    ~euler.Occluded && ~rotationMatrix.Occluded;
                if ~poseAvailable
                    obj.PreviousFrameNumbers(subjectIndex) = NaN;
                    continue
                end

                position = reshape(double(translation.Translation), 1, 3) ./ 1000.0;
                angles = reshape(double(euler.Rotation), 1, 3);
                rotation = reshape(double(rotationMatrix.Rotation), 3, 3).';
                states(subjectIndex, 1:6) = [position, angles];

                previousFrame = obj.PreviousFrameNumbers(subjectIndex);
                if ~isnan(previousFrame) && frameNumber > previousFrame
                    deltaTime = (frameNumber - previousFrame) ./ frameRateHz;
                    previousPosition = obj.PreviousPositions(subjectIndex, :);
                    velocityWorld = (position - previousPosition) ./ deltaTime;
                    velocityBody = (rotation.' * velocityWorld.').';

                    previousRotation = obj.PreviousRotations(:, :, subjectIndex);
                    relativeRotation = rotation * previousRotation.';
                    rotationVector = viconTracker.rotationVector(relativeRotation);
                    angularVelocityBody = (rotation.' * rotationVector.').' ./ deltaTime;
                    states(subjectIndex, 7:12) = [velocityBody, angularVelocityBody];
                    valid(subjectIndex) = true;
                end

                obj.PreviousFrameNumbers(subjectIndex) = frameNumber;
                obj.PreviousPositions(subjectIndex, :) = position;
                obj.PreviousRotations(:, :, subjectIndex) = rotation;
            end
        end

        function delete(obj)
            % delete disconnects the Vicon DataStream client.
            if ~isempty(obj.Client)
                obj.Client.Disconnect();
            end
        end
    end

    methods (Static, Access = private)
        function vector = rotationVector(rotation)
            cosine = max(-1.0, min(1.0, (trace(rotation) - 1.0) ./ 2.0));
            angle = acos(cosine);
            veeVector = [ ...
                rotation(3, 2) - rotation(2, 3), ...
                rotation(1, 3) - rotation(3, 1), ...
                rotation(2, 1) - rotation(1, 2)];
            if angle < 1.0e-6
                vector = 0.5 .* veeVector;
            else
                vector = angle .* veeVector ./ (2.0 .* sin(angle));
            end
        end
    end
end
