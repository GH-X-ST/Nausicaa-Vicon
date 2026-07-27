function runViconTracker(subjectNames, options)
% runViconTracker streams full states from the Imperial flight arena.
%   runViconTracker tracks every subject available when the connection
%   opens. Pass subject names to select rigid bodies. The function displays
%   one complete sample every 20 Vicon frames. Press Ctrl+C to stop.

    arguments
        subjectNames string = strings(0, 1)
        options.Host (1, 1) string = viconTracker.DefaultHost
        options.DerivativeCutoffHz (1, 1) double {mustBeFinite} = ...
            viconTracker.DefaultDerivativeCutoffHz
    end

    tracker = viconTracker( ...
        subjectNames, ...
        Host=options.Host, ...
        DerivativeCutoffHz=options.DerivativeCutoffHz);
    trackerCleanup = onCleanup(@() delete(tracker));

    while true
        frameData = tracker.read();
        if mod(frameData.frameNumber, 20) ~= 0
            continue
        end

        fprintf( ...
            "frame=%d rate=%.1f Hz latency=%.1f ms\n", ...
            frameData.frameNumber, ...
            frameData.frameRateHz, ...
            1000.0*frameData.latencyS);
        visibleSubjects = keys(frameData.states);
        for subjectIndex = 1:numel(visibleSubjects)
            subjectName = visibleSubjects(subjectIndex);
            fprintf("  %s:\n", subjectName);
            disp(frameData.states(subjectName));
        end
        if ~isempty(frameData.occludedSubjects)
            fprintf( ...
                "  occluded: %s\n", ...
                strjoin(frameData.occludedSubjects, ", "));
        end
    end
end
