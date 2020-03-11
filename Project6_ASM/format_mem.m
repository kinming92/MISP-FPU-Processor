fid1 = fopen('float_add.txt', 'r');
fid2 = fopen('float_add_8.txt', 'w');

l = 1;
while l <= 15
    tline = fgetl(fid1);
    if ~ischar(tline), break, end
    for k = 1:2:8
        fprintf(fid2, '%s\n', tline(k:k+1));
    end
    l = l+1;
end

for k = 1:2:8
        fprintf(fid2, '%s\n', '00');
end

fclose(fid1);
fclose(fid2);