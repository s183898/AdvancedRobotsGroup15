function scan = transform(systempose_w,pos_l)
    sizeOfScan = size(pos_l,2);
    scan = zeros(2,sizeOfScan);

    for i = 1:sizeOfScan
    scan(:,i) = [ cos(systempose_w(3)) -sin(systempose_w(3));
            sin(systempose_w(3)) cos(systempose_w(3))]*(pos_l(:,i))+ [systempose_w(1) systempose_w(2)]';
    end
end