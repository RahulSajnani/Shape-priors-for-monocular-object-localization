function out =  importdataFromNet(CarId, SeqId, StartFrame,EndFrame)
    
    info = importdata('Multi/infofile.txt');
    result_kp = importdata('Multi/result_KP.txt');
    keypoints = [];
    k = 0;
    
    for i = 1:size(info,1)
        if info(i,2) == SeqId
            if info(i,3) == StartFrame + k
                if info(i,4) == CarId
                    keypoints = [keypoints; result_kp(i,:)];
                    k = k + 1;
                end
            end
        end
        if StartFrame + k > EndFrame + 1
            break;
    end
                    
    out = keypoints;            
end