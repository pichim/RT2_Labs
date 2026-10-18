function data = cut_data(a)
try
cp1 = char([254 1 255]);
ca = char(a(:))';
if length(a)<8
    data.N=0;
    data.rest_data = a;
    data.complete_data = false;
    return
end
n = strfind(ca,cp1);
data.N = length(n);
N_bytes = -1;
for nk=1:data.N
    if length(a)<(n(nk)+6)
        data(nk).complete_data = false;
        data(1).rest_data = a(n(nk):end);
        N_bytes = 0;
        break
    end
    head = a(n(nk):(n(nk)+6));
    N_bytes = uint16(head(6)) + 256 * uint16(head(7));
    en = double(min(n(nk)+N_bytes+7,length(a)));
    dat =  a((n(nk)+7):en);
    if length(dat)< (N_bytes+1) % also csm
        data(nk).complete_data = false;
        data(1).rest_data = a(n(nk):end);    
        N_bytes = 0;
        break;
    end
    data(nk).complete_data = true;
    data(nk).id1 = head(4);
    data(nk).id2 = head(5);

    if  data(nk).complete_data
        data(nk).grab = dat(1:N_bytes);
        data(nk).N_bytes = N_bytes;
        data(nk).csm_sent = dat(N_bytes+1);
        data(nk).csm_check = uint8(mod(sum(head)+sum(data(nk).grab),256));
        if data(nk).csm_sent ~= data(nk).csm_check
            3;
        end
            
    else
        data(nk).grab = dat(1:end);  
    end
end
if N_bytes == -1
     data(1).rest_data = a;
elseif N_bytes>0 && length(a)>(n(nk)+uint32(N_bytes)+9)
     data(1).rest_data = a((n(nk)+uint32(N_bytes)+9):end);
end
catch ME
    3;
end