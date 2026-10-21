function data = cut_data(a)
% Decode complete UART frames; retain only unfinished bytes for the next call.
% Header: [254 1 255 id1 id2 length_lo length_hi], checksum, CR/LF.
a = uint8(a(:).');
data = struct('N', 0, 'rest_data', uint8([]), 'complete_data', false, ...
    'id1', 0, 'id2', 0, 'grab', uint8([]), 'N_bytes', 0, 'csm_sent', 0, 'csm_check', 0);
packet = data;
pos = 1;
count = 0;
while pos <= length(a)
    starts = strfind(char(a(pos:end)), char([254 1 255]));
    if isempty(starts)
        data(1).rest_data = a(max(pos, length(a)-1):end); % possible partial header
        break
    end
    pos = pos + starts(1) - 1;
    if length(a)-pos+1 < 7
        data(1).rest_data = a(pos:end);
        break
    end
    head = a(pos:pos+6);
    N_bytes = double(head(6)) + 256*double(head(7));
    if N_bytes > 246 % maximum payload in the firmware protocol
        warning('UART:Receive', 'Rejected UART payload length %d.', N_bytes);
        pos = pos + 1;
        continue
    end
    frame_length = N_bytes + 10;
    if length(a)-pos+1 < frame_length
        data(1).rest_data = a(pos:end);
        break
    end
    grab = a(pos+7:pos+6+N_bytes);
    csm_sent = a(pos+7+N_bytes);
    csm_check = uint8(mod(sum(head)+sum(grab), 256));
    if ~isequal(a(pos+8+N_bytes:pos+9+N_bytes), uint8([13 10]))
        warning('UART:Receive', 'Rejected UART frame with invalid CR/LF.');
        pos = pos + 1;
        continue
    end
    if csm_sent ~= csm_check
        warning('UART:Receive', 'Rejected UART frame with invalid checksum.');
        pos = pos + frame_length;
        continue
    end
    count = count + 1;
    packet.complete_data = true;
    packet.id1 = head(4);
    packet.id2 = head(5);
    packet.grab = grab;
    packet.N_bytes = N_bytes;
    packet.csm_sent = csm_sent;
    packet.csm_check = csm_check;
    data(count) = packet;
    pos = pos + frame_length; % never search inside an accepted payload
end
data(1).N = count;
end
