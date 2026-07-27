function sendPPMChannels(serialConnection, pulseWidthsMicroseconds)
% sendPPMChannels writes one command to an open serial connection.

    arguments
        serialConnection (1, 1) serialport
        pulseWidthsMicroseconds (1, 8) double
    end

    packet = encodePPMPacket(pulseWidthsMicroseconds);
    write(serialConnection, packet, "uint8");
end
