function packet = encodePPMPacket(pulseWidthsMicroseconds)
% encodePPMPacket encodes eight receiver-channel pulse widths.

    arguments
        pulseWidthsMicroseconds (1, 8) double { ...
            mustBeInteger, ...
            mustBeBetween(pulseWidthsMicroseconds, 1000, 2000)}
    end

    pulseWidths = uint16(pulseWidthsMicroseconds);
    lowBytes = uint8(bitand(pulseWidths, uint16(255)));
    highBytes = uint8(bitshift(pulseWidths, -8));
    payload = reshape([lowBytes; highBytes], 1, []);
    packet = [uint8(80), payload];
end
