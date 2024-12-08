function [num_stabbed, stabber] = interval_stabbing(Intervals)
    % Intervals: 2L * 1
    L = size(Intervals, 1) / 2;
    masks = repmat([0;1], L, 1);

    [~, sidx] = sort(Intervals);
    length_sidx = 2*L;
    
    count = 0; num_stabbed = 0; stabber = 0;
    for i = 1:length_sidx
        if masks(sidx(i)) == 0            
            count = count + 1;
            if count > num_stabbed                   
                    num_stabbed = count;
                    stabber = Intervals(sidx(i))+1e-12;  
            end
        else
            count = count - 1;
        end       
    end
end
