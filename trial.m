function track = storeVals(sequences, frames, ids)
%prompt = 'Number of cars to receive data';
%x = input(prompt);
x = size(sequence,2);
track = [];
for i = 1:x
%     promptf = 'input frame';
%     prompts = 'input sequence';
%     promptid = 'input id';
%     seq = input(prompts);
%     frame = input(promptf);
%     frame = frame + 1;
%     id = input(promptid)
    seq = sequences(i)
    frame = frames(i) + 1
    id = ids(i)
    
    ans = readLabels('../../../data_tracking_label_2/training/label_02', seq);
    ans = struct2table(ans{frame})
    for m = 1:size(ans,1)
        if ans{m,2} == id
            temp = ans(m,{'frame','id','x1','y1','x2','y2', 'ry'})
        end
    end 
    ansfin = temp{:,:}
    ansfin = [seq ansfin];
    track = [track; ansfin];
end
end
    
